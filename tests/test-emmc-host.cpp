// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Host-side test of ./EmmcHost.h and Gpt.h: the same headers the
// kernel driver compiles, run against a model of the DesignWare MSHC and an eMMC.
//
//   g++ -std=c++17 -Wall -Wextra -Werror -O1 -I . tests/test-emmc-host.cpp -o /tmp/test-emmc-host
//   /tmp/test-emmc-host
//
// The model enforces the facts the driver depends on (see EmmcHost.h): a 32-bit
// store into the 0x2c..0x2f register group is a bug (it clobbers TIMEOUT_CONTROL /
// SOFTWARE_RESET), a card in DATA/RCV answers only CMD12, CMD12 in TRAN is an
// illegal command, and the only commands the driver may ever send are 6 (FLUSH_CACHE only), 7, 12, 13,
// 17, 18 and -- inside a declared range, through ADMA2 only -- 25. A command other than CMD13 sent
// while the card is programming is a violation. DMA is modelled too (ADMA2: the model walks the
// descriptor table the driver wrote and fills a fake window, or, for a write, reads it; any descriptor
// or address outside the window is a stray access and fails the test).
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <set>
#include <utility>
#include <vector>

#include "EmmcHost.h"
#include "Gpt.h"

namespace {

int failures = 0;
#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                        \
      ++failures;                                                                        \
    }                                                                                    \
  } while (0)

struct Model {
  // Controller.
  uint8_t regs[0x1000] = {};
  uint32_t intStatus = 0;
  uint32_t response = 0;
  std::vector<uint32_t> fifo;
  size_t fifoPos = 0;
  bool dataInhibit = false;
  uint8_t resetBits = 0;
  unsigned resetPolls = 0;
  uint64_t clockUs = 0;
  // Card.
  enum State { Idle, Ready, Ident, Stby, Tran, Data, Rcv, Prg, Dis };
  State state = Tran;
  uint32_t rca = 1;
  bool illegalPending = false;
  std::vector<uint8_t> image;
  unsigned prgPollsLeft = 0;
  // Writes.
  unsigned programPolls = 2;       // CMD13 answers "programming" this many times after a write or a flush
  bool cacheEnabled = true;        // FLUSH_CACHE: false = SWITCH_ERROR on the next status
  bool switchErrorPending = false;
  uint32_t extraErrorBits = 0;     // error bits the next status reports (and clears), e.g. WP violation
  unsigned writeCommands = 0, flushCommands = 0;
  std::vector<std::pair<uint64_t, uint32_t>> writes;   // (lba, count) of every CMD25
  // Fault injection.
  unsigned failDataTimeout = 0;   // CMD17 answered, then data timeout, card left in DATA
  unsigned failNoResponse = 0;    // CMD17 never answered
  bool stickInhibitAfterCmd12 = false;  // DATA_INHIBIT stays set until a DATA reset
  unsigned failRunAtBlock = 0;    // CMD18: a data CRC error instead of block N (1-based) of the run
  // A CMD18 run in flight: the next block to serve and how many are left.
  uint64_t multiNext = 0;
  uint32_t multiRemaining = 0;
  unsigned runBlocksServed = 0;
  unsigned multiCommands = 0;
  // DMA: the fake window's first byte has the bus address dmaBase.
  std::vector<uint8_t> dmaMemory;
  uint32_t dmaBase = 0x40010000;
  unsigned dmaTransfers = 0;
  unsigned failDmaError = 0;  // ADMA error instead of the data (card left in DATA)
  unsigned failDmaStall = 0;  // the transfer never completes (card left in DATA)
  unsigned failWriteError = 0;  // CMD25: ADMA error instead of the data phase (card left in RCV)
  unsigned failWriteNeverReady = 0;  // the card stays in PRG for good
  bool dmaStray = false;      // a descriptor or an address outside the window
  // Observations.
  std::set<unsigned> commandsSeen;
  unsigned hostResets = 0, dataResets = 0, cmdResets = 0;
  unsigned violations = 0;
  uint64_t blocksServed = 0;
  bool overrun = false;

  uint32_t cardStatus() {
    uint32_t status = (static_cast<uint32_t>(state) << 9) | (state == Tran ? 1u << 8 : 0);
    if (illegalPending) status |= 1u << 22;
    status |= extraErrorBits;
    if (switchErrorPending) status |= 1u << 7;
    return status;
  }

  // The next block of a CMD18 run into the buffer port, or the injected fault.
  void multiServe() {
    ++runBlocksServed;
    dataInhibit = true;
    if (failRunAtBlock && runBlocksServed == failRunAtBlock) {
      failRunAtBlock = 0;
      multiRemaining = 0;
      fifo.clear();
      fifoPos = 0;
      intStatus |= (1u << 21) | emmc::kIntError;  // data CRC error
      return;
    }
    if ((multiNext + 1) * emmc::kBlockBytes > image.size()) {
      multiRemaining = 0;
      intStatus |= (1u << 20) | emmc::kIntError;  // data timeout: the card has nothing there
      return;
    }
    fifo.assign(emmc::kBlockBytes / 4, 0);
    memcpy(fifo.data(), image.data() + multiNext * emmc::kBlockBytes, emmc::kBlockBytes);
    fifoPos = 0;
    intStatus |= emmc::kIntDataAvailable;
    ++blocksServed;
    ++multiNext;
    --multiRemaining;
  }

  // The controller's ADMA2 engine for one CMD18 run: walks the table at the ADMA address and fills the
  // window. A descriptor that is not valid/transfer, or an address outside the window, is a bug of the driver.
  void serveDma(uint64_t lba, uint32_t count) {
    ++dmaTransfers;
    if ((regs[emmc::kRegHostControl] & emmc::kHc1DmaMask) != emmc::kHc1Adma2) ++violations;
    uint32_t table;
    memcpy(&table, regs + emmc::kRegAdmaAddress, 4);
    dataInhibit = true;
    auto adma = [&] { intStatus |= (1u << 25) | emmc::kIntError; };
    if (failDmaStall) {
      --failDmaStall;
      return;
    }
    if (failDmaError) {
      --failDmaError;
      return adma();
    }
    uint16_t hc2;
    memcpy(&hc2, regs + 0x3e, 2);
    const bool len26 = hc2 & (1u << 10);
    const uint64_t total = static_cast<uint64_t>(count) * emmc::kBlockBytes;
    uint64_t done = 0;
    for (unsigned index = 0; index < 64; ++index) {
      const uint64_t at = static_cast<uint64_t>(table) - dmaBase + index * 8ull;
      if (table < dmaBase || at + 8 > dmaMemory.size()) {
        dmaStray = true;
        return adma();
      }
      uint16_t attributes, length16;
      uint32_t address;
      memcpy(&attributes, dmaMemory.data() + at, 2);
      memcpy(&length16, dmaMemory.data() + at + 2, 2);
      memcpy(&address, dmaMemory.data() + at + 4, 4);
      if (!(attributes & emmc::kAdmaValid) || (attributes & 0x30) != emmc::kAdmaTran) {
        ++violations;
        return adma();
      }
      uint64_t length = len26 ? ((static_cast<uint64_t>(attributes >> 6) << 16) | length16) : length16;
      if (len26 && length == 0) length = 1ull << 26;
      if (!len26 && length == 0) length = 65536;
      if (address < dmaBase || static_cast<uint64_t>(address) - dmaBase + length > dmaMemory.size() || done + length > total) {
        dmaStray = true;
        return adma();
      }
      if (lba * emmc::kBlockBytes + done + length > image.size()) {
        intStatus |= (1u << 20) | emmc::kIntError;  // nothing there
        return;
      }
      memcpy(dmaMemory.data() + (address - dmaBase), image.data() + lba * emmc::kBlockBytes + done, length);
      done += length;
      if (attributes & emmc::kAdmaEnd) break;
    }
    if (done != total) return adma();
    blocksServed += count;
    state = Tran;  // Auto CMD12 has ended the command
    dataInhibit = false;
    intStatus |= emmc::kIntDataEnd;
  }

  // The controller's ADMA2 engine for one CMD25 run: walks the table and copies the window into the card's
  // image. The same checks as for a read; a write outside the image is a failed command, not a resize.
  void serveDmaWrite(uint64_t lba, uint32_t count) {
    ++dmaTransfers;
    if ((regs[emmc::kRegHostControl] & emmc::kHc1DmaMask) != emmc::kHc1Adma2) ++violations;
    uint32_t table;
    memcpy(&table, regs + emmc::kRegAdmaAddress, 4);
    dataInhibit = true;
    auto adma = [&] { intStatus |= (1u << 25) | emmc::kIntError; };
    if (failWriteError) {
      --failWriteError;
      return adma();
    }
    const uint64_t total = static_cast<uint64_t>(count) * emmc::kBlockBytes;
    uint64_t done = 0;
    for (unsigned index = 0; index < 64; ++index) {
      const uint64_t at = static_cast<uint64_t>(table) - dmaBase + index * 8ull;
      if (table < dmaBase || at + 8 > dmaMemory.size()) {
        dmaStray = true;
        return adma();
      }
      uint16_t attributes, length16;
      uint32_t address;
      memcpy(&attributes, dmaMemory.data() + at, 2);
      memcpy(&length16, dmaMemory.data() + at + 2, 2);
      memcpy(&address, dmaMemory.data() + at + 4, 4);
      if (!(attributes & emmc::kAdmaValid) || (attributes & 0x30) != emmc::kAdmaTran) {
        ++violations;
        return adma();
      }
      uint64_t length = length16 ? length16 : 65536;
      if (address < dmaBase || static_cast<uint64_t>(address) - dmaBase + length > dmaMemory.size() || done + length > total) {
        dmaStray = true;
        return adma();
      }
      if (lba * emmc::kBlockBytes + done + length > image.size()) {
        intStatus |= (1u << 20) | emmc::kIntError;
        return;
      }
      memcpy(image.data() + lba * emmc::kBlockBytes + done, dmaMemory.data() + (address - dmaBase), length);
      done += length;
      if (attributes & emmc::kAdmaEnd) break;
    }
    if (done != total) return adma();
    // Auto CMD12 ends the command; the card now programs what it was given.
    state = Prg;
    prgPollsLeft = failWriteNeverReady ? 1000000 : programPolls + 1;
    dataInhibit = false;
    intStatus |= emmc::kIntDataEnd;
  }

  void finishCommand(uint32_t cmdWord, uint32_t modeWord) {
    unsigned index = (cmdWord >> 8) & 0x3f;
    bool hasData = cmdWord & (1u << 5);
    uint32_t argument;
    memcpy(&argument, regs + emmc::kRegArgument, 4);
    commandsSeen.insert(index);
    auto timeout = [&] { intStatus |= (1u << 16) | emmc::kIntError; };
    if (state == Data || state == Rcv) {
      // Only CMD12 (abort) and CMD13 (status) get through; everything else is ignored.
      if (index != 12 && index != 13) return timeout();
    }
    if (state == Prg && index != 13) {
      // Programming: the card holds DAT0 low and accepts CMD13 only; anything else is an illegal command
      // (here: ignored and remembered, which is how the real card reports it on the next status).
      ++violations;
      illegalPending = true;
      return timeout();
    }
    switch (index) {
      case 13:
        if ((argument >> 16) != rca) return timeout();
        response = cardStatus();
        illegalPending = false;
        extraErrorBits = 0;
        switchErrorPending = false;
        if (state == Prg && prgPollsLeft && --prgPollsLeft == 0) state = Tran;
        intStatus |= emmc::kIntResponse;
        return;
      case 6: {
        // Only FLUSH_CACHE (write byte, index 32, value 1) is allowed.
        if (state != Tran) return timeout();
        if (argument != emmc::kFlushCacheArgument) {
          ++violations;
          return timeout();
        }
        ++flushCommands;
        response = cardStatus();
        intStatus |= emmc::kIntResponse;
        if (cacheEnabled) {
          state = Prg;
          prgPollsLeft = programPolls + 1;
        } else {
          switchErrorPending = true;
        }
        return;
      }
      case 25: {
        if (state != Tran) return timeout();
        // A write is block-count-enabled, multi-block, DMA, stopped by Auto CMD12, and not a read.
        const uint32_t wanted = emmc::kTmBlockCountEnable | emmc::kTmMultiBlock | emmc::kTmAutoCmd12 | emmc::kTmDmaEnable;
        if ((modeWord & (wanted | emmc::kTmRead)) != wanted) ++violations;
        ++writeCommands;
        response = cardStatus();
        intStatus |= emmc::kIntResponse;
        state = Rcv;
        uint16_t count;
        memcpy(&count, regs + emmc::kRegBlockCount, 2);
        writes.push_back({argument, count});
        serveDmaWrite(argument, count);
        return;
      }
      case 12:
        if (state == Tran) illegalPending = true;
        state = Tran;
        response = cardStatus();
        intStatus |= emmc::kIntResponse;
        dataInhibit = stickInhibitAfterCmd12;
        fifo.clear();
        fifoPos = 0;
        multiRemaining = 0;
        return;
      case 7:
        if ((argument >> 16) != rca) return timeout();
        if (state == Stby) state = Tran;
        response = cardStatus();
        intStatus |= emmc::kIntResponse;
        return;
      case 17: {
        (void)hasData;
        if (state != Tran) return timeout();
        if (failNoResponse) {
          --failNoResponse;
          return timeout();
        }
        response = cardStatus();
        intStatus |= emmc::kIntResponse;
        state = Data;
        if (failDataTimeout) {
          --failDataTimeout;
          intStatus |= (1u << 20) | emmc::kIntError;  // data timeout
          dataInhibit = true;
          return;
        }
        uint32_t hc2;
        memcpy(&hc2, regs + 0x3c, 4);
        hc2 >>= 16;
        uint32_t stale;
        memcpy(&stale, regs + emmc::kRegDmaAddress, 4);
        uint16_t count;
        memcpy(&count, regs + emmc::kRegBlockCount, 2);
        if ((hc2 & emmc::kHc2HostV4) && stale != 0) overrun = true;  // v4: the 32-bit count wins
        uint64_t lba = argument;
        if ((lba + 1) * emmc::kBlockBytes > image.size()) return timeout();
        fifo.assign(emmc::kBlockBytes / 4, 0);
        memcpy(fifo.data(), image.data() + lba * emmc::kBlockBytes, emmc::kBlockBytes);
        fifoPos = 0;
        dataInhibit = true;
        intStatus |= emmc::kIntDataAvailable;
        ++blocksServed;
        return;
      }
      case 18: {
        if (state != Tran) return timeout();
        if (failNoResponse) {
          --failNoResponse;
          return timeout();
        }
        // A run is block-count-enabled, multi-block, a read, stopped by Auto CMD12, and not DMA.
        const uint32_t wanted = emmc::kTmBlockCountEnable | emmc::kTmMultiBlock | emmc::kTmRead | emmc::kTmAutoCmd12;
        const uint32_t dmaBit = modeWord & emmc::kTmDmaEnable;
        if ((modeWord & (wanted | emmc::kTmDmaEnable | (1u << 3))) != (wanted | dmaBit)) ++violations;
        response = cardStatus();
        intStatus |= emmc::kIntResponse;
        state = Data;
        uint32_t hc2;
        memcpy(&hc2, regs + 0x3c, 4);
        hc2 >>= 16;
        uint32_t stale;
        memcpy(&stale, regs + emmc::kRegDmaAddress, 4);
        if ((hc2 & emmc::kHc2HostV4) && stale != 0) overrun = true;  // v4: the 32-bit count wins
        uint16_t count;
        memcpy(&count, regs + emmc::kRegBlockCount, 2);
        ++multiCommands;
        multiNext = argument;
        multiRemaining = count;
        runBlocksServed = 0;
        if (dmaBit) {
          multiRemaining = 0;
          serveDma(argument, count);
          return;
        }
        multiServe();
        return;
      }
      default:
        ++violations;  // a command the read-only driver must never send
        printf("model: unexpected CMD%u\n", index);
        return timeout();
    }
  }

  uint32_t read(uint32_t offset, unsigned width) {
    clockUs += 1;
    uint32_t value = 0;
    switch (offset) {
      case emmc::kRegBuffer: {
        if (width != 32) ++violations;
        if (fifoPos < fifo.size()) value = fifo[fifoPos++];
        if (fifoPos == fifo.size() && !fifo.empty()) {
          fifo.clear();
          fifoPos = 0;
          if (multiRemaining) {
            multiServe();  // the next block of the run
          } else {
            state = Tran;  // the last block; for a run Auto CMD12 has ended it
            dataInhibit = false;
            intStatus |= emmc::kIntDataEnd;
          }
        }
        return value;
      }
      case emmc::kRegPresentState: {
        uint32_t ps = 0;
        if (dataInhibit) ps |= emmc::kPsDataInhibit;
        if (!fifo.empty()) ps |= emmc::kPsDataAvailable;
        return ps;
      }
      case emmc::kRegIntStatus: return intStatus;
      case emmc::kRegResponse: return response;
      case emmc::kRegSoftwareReset:
        if (resetBits && ++resetPolls >= 2) resetBits = 0;
        return resetBits;
      default: break;
    }
    if (width == 8) return regs[offset];
    if (width == 16) { uint16_t v; memcpy(&v, regs + offset, 2); return v; }
    memcpy(&value, regs + offset, 4);
    return value;
  }

  void write(uint32_t offset, unsigned width, uint32_t value) {
    clockUs += 1;
    // Only 32-bit and 8-bit stores are allowed, and never a 32-bit one into 0x2c..0x2f.
    if (width == 16) ++violations;
    if (width == 32 && offset >= 0x2c && offset < 0x30) ++violations;
    if (offset == emmc::kRegIntStatus) { intStatus &= ~value; return; }
    if (offset == emmc::kRegSoftwareReset) {
      if (value & emmc::kResetAll) {
        ++hostResets;
        memset(regs, 0, sizeof(regs));
        intStatus = 0;
        dataInhibit = false;
        fifo.clear();
      }
      if (value & emmc::kResetData) {
        ++dataResets;
        dataInhibit = false;
        fifo.clear();
        fifoPos = 0;
      }
      if (value & emmc::kResetCmd) ++cmdResets;
      resetBits = static_cast<uint8_t>(value & 7);
      resetPolls = 0;
      return;
    }
    if (width == 8) { regs[offset] = static_cast<uint8_t>(value); return; }
    memcpy(regs + offset, &value, 4);
    if (offset == emmc::kRegTransferMode) {
      uint32_t cmdWord = value >> 16;
      finishCommand(cmdWord, value & 0xffff);
    }
  }
};

Model *current = nullptr;

uint32_t platRead(void *, uint32_t offset, unsigned width) { return current->read(offset, width); }
void platWrite(void *, uint32_t offset, unsigned width, uint32_t value) { current->write(offset, width, value); }
uint64_t platNow() { return current->clockUs; }
void platDelay(uint32_t us) { current->clockUs += us; }
void platLog(const char *format, ...) {
  va_list args;
  va_start(args, format);
  printf("  | ");
  vprintf(format, args);
  va_end(args);
}

const emmc::Platform kPlatform = {platRead, platWrite, platNow, platDelay, platLog};

void fillImage(Model &model, uint64_t blocks) {
  model.image.resize(blocks * emmc::kBlockBytes);
  for (uint64_t lba = 0; lba < blocks; ++lba)
    for (unsigned i = 0; i < emmc::kBlockBytes; ++i)
      model.image[lba * emmc::kBlockBytes + i] = static_cast<uint8_t>(lba * 7 + i * 13 + (lba >> 8));
}

bool blockMatches(const Model &model, uint64_t lba, const uint8_t *data) {
  return memcmp(model.image.data() + lba * emmc::kBlockBytes, data, emmc::kBlockBytes) == 0;
}

void put32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
void put64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }

// A GPT with three partitions, 16384 blocks (8 MiB).
void buildGpt(Model &model, uint64_t blocks) {
  uint8_t *header = model.image.data() + emmc::kBlockBytes;
  uint8_t *entries = model.image.data() + 2 * emmc::kBlockBytes;
  memset(header, 0, emmc::kBlockBytes);
  memset(entries, 0, 32 * emmc::kBlockBytes);
  struct Part { const char *name; uint64_t first, last; };
  const Part parts[] = {{"boot", 64, 1023}, {"super", 1024, 8191}, {"userdata", 8192, blocks - 34}};
  for (unsigned i = 0; i < 3; ++i) {
    uint8_t *entry = entries + i * 128;
    memset(entry, 0xA0 + i, 16);
    memset(entry + 16, 0x10 + i, 16);
    put64(entry + 32, parts[i].first);
    put64(entry + 40, parts[i].last);
    for (unsigned c = 0; parts[i].name[c]; ++c) entry[56 + 2 * c] = static_cast<uint8_t>(parts[i].name[c]);
  }
  memcpy(header, "EFI PART", 8);
  put32(header + 8, 0x00010000);
  put32(header + 12, 92);
  put64(header + 24, 1);
  put64(header + 32, blocks - 1);
  put64(header + 40, 34);
  put64(header + 48, blocks - 34);
  memset(header + 56, 0x5a, 16);
  put64(header + 72, 2);
  put32(header + 80, 128);
  put32(header + 84, 128);
  put32(header + 88, emmc::crc32(entries, 128 * 128));
  put32(header + 16, 0);
  put32(header + 16, emmc::crc32(header, 92));
}

emmc::Host makeHost(Model &model) {
  current = &model;
  emmc::Host host;
  host.attach(&kPlatform, nullptr);
  return host;
}

void testCrc() {
  printf("crc32\n");
  CHECK(emmc::crc32("123456789", 9) == 0xcbf43926u);
  uint32_t incremental = emmc::crc32("1234", 4);
  CHECK(emmc::crc32("56789", 5, incremental) == 0xcbf43926u);
}

void testIdentify() {
  const Model::State starts[] = {Model::Tran, Model::Stby, Model::Data, Model::Rcv};
  for (Model::State start : starts) {
    printf("identify from state %d\n", static_cast<int>(start));
    Model model;
    fillImage(model, 256);
    model.state = start;
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    CHECK(host.usable());
    CHECK(model.state == Model::Tran);
    CHECK(model.regs[0x34] == 0xff && model.regs[0x37] == 0xff);  // INT_ENABLE stays open
    CHECK(model.violations == 0);
  }
  {
    printf("identify waits for a card that is still programming\n");
    Model model;
    fillImage(model, 256);
    model.state = Model::Prg;
    model.prgPollsLeft = 3;
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    CHECK(model.state == Model::Tran);
  }
  {
    printf("identify with RCA 2\n");
    Model model;
    fillImage(model, 256);
    model.rca = 2;
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    CHECK(host.rca() == 2);
  }
  {
    printf("identify with no card\n");
    Model model;
    fillImage(model, 256);
    model.rca = 9;
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kNoCard);
    CHECK(!host.usable());
    uint8_t out[512];
    CHECK(host.readBlocks(0, 1, out) == emmc::kNoCard);
  }
  {
    printf("identify refuses an uninitialised card\n");
    Model model;
    fillImage(model, 256);
    model.state = Model::Ident;
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kProtocol);
  }
}

void testReads() {
  printf("reads\n");
  Model model;
  fillImage(model, 1024);
  emmc::Host host = makeHost(model);
  CHECK(host.identify() == emmc::kOk);
  std::vector<uint8_t> out(100 * emmc::kBlockBytes);
  CHECK(host.readBlocks(37, 100, out.data()) == emmc::kOk);
  for (unsigned i = 0; i < 100; ++i) CHECK(blockMatches(model, 37 + i, out.data() + i * emmc::kBlockBytes));
  CHECK(host.stats.blocks == 100);
  CHECK(host.stats.retries == 0);
  CHECK(model.state == Model::Tran);
  // Reading past the end of the card is an error, not a hang.
  CHECK(host.readBlocks(5000, 1, out.data()) != emmc::kOk);
  CHECK(model.violations == 0);
  for (unsigned index : model.commandsSeen) CHECK(index == 7 || index == 12 || index == 13 || index == 17 || index == 18);
}

void testRecovery() {
  {
    printf("recovery from a data timeout\n");
    Model model;
    fillImage(model, 256);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    model.failDataTimeout = 1;
    uint8_t out[512];
    CHECK(host.readBlocks(9, 1, out) == emmc::kOk);
    CHECK(blockMatches(model, 9, out));
    CHECK(host.stats.retries == 1);
    CHECK(host.stats.recoveries == 1);
    CHECK(model.hostResets == 0);
    CHECK(model.state == Model::Tran);
    CHECK(model.violations == 0);
  }
  {
    printf("recovery from a missing response\n");
    Model model;
    fillImage(model, 256);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    model.failNoResponse = 2;
    uint8_t out[512];
    CHECK(host.readBlocks(3, 1, out) == emmc::kOk);
    CHECK(blockMatches(model, 3, out));
    CHECK(host.stats.retries == 2);
    CHECK(model.hostResets == 0);
    CHECK(model.violations == 0);
  }
  {
    printf("gives up after three attempts\n");
    Model model;
    fillImage(model, 256);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    model.failNoResponse = 100;
    uint8_t out[512];
    CHECK(host.readBlocks(3, 1, out) != emmc::kOk);
    CHECK(host.stats.failures == 1);
    CHECK(host.stats.retries == 3);
  }
  {
    printf("data inhibit that only a DATA reset clears\n");
    Model model;
    fillImage(model, 256);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    model.failDataTimeout = 1;
    model.stickInhibitAfterCmd12 = true;
    uint8_t out[512];
    CHECK(host.readBlocks(11, 1, out) == emmc::kOk);
    CHECK(blockMatches(model, 11, out));
    CHECK(model.dataResets >= 1);
    CHECK(model.hostResets == 0);
  }
}

void testHostV4() {
  printf("host v4 with a stale 32-bit count\n");
  Model model;
  fillImage(model, 256);
  uint16_t hc2 = emmc::kHc2HostV4;
  memcpy(model.regs + 0x3e, &hc2, 2);
  uint32_t stale = 0x1234;
  memcpy(model.regs + emmc::kRegDmaAddress, &stale, 4);  // what a firmware DMA transfer leaves behind
  emmc::Host host = makeHost(model);
  CHECK(host.identify() == emmc::kOk);
  CHECK(host.hostV4());
  uint8_t out[512];
  CHECK(host.readBlocks(5, 1, out) == emmc::kOk);
  CHECK(!model.overrun);
  CHECK(blockMatches(model, 5, out));
}

void testMultiBlock() {
  {
    printf("multi-block reads\n");
    Model model;
    fillImage(model, 1024);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    CHECK(!host.multiBlock());  // off until asked for
    host.setMultiBlock(true);
    std::vector<uint8_t> out(300 * emmc::kBlockBytes);
    CHECK(host.readBlocks(37, 300, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 300; ++i) CHECK(blockMatches(model, 37 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(host.stats.blocks == 300);
    CHECK(host.stats.commands < 20);  // 128 + 128 + 44 blocks: three CMD18s and the card-state commands
    CHECK(model.multiCommands == 3);
    CHECK(model.commandsSeen.count(17) == 0);
    CHECK(model.state == Model::Tran);
    // A single block is still one CMD17.
    CHECK(host.readBlocks(5, 1, out.data()) == emmc::kOk);
    CHECK(blockMatches(model, 5, out.data()));
    CHECK(model.commandsSeen.count(17) == 1);
    CHECK(model.multiCommands == 3);
    CHECK(model.violations == 0);
    CHECK(!model.overrun);
  }
  {
    printf("multi-block: the last blocks of the card\n");
    Model model;
    fillImage(model, 256);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    std::vector<uint8_t> out(16 * emmc::kBlockBytes);
    CHECK(host.readBlocks(240, 16, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 16; ++i) CHECK(blockMatches(model, 240 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(host.readBlocks(248, 16, out.data()) != emmc::kOk);  // runs off the end: an error, not a hang
    CHECK(model.state == Model::Tran);
  }
  {
    printf("multi-block: a data error in the middle of a run is recovered\n");
    Model model;
    fillImage(model, 1024);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    model.failRunAtBlock = 5;
    std::vector<uint8_t> out(64 * emmc::kBlockBytes);
    CHECK(host.readBlocks(100, 64, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 64; ++i) CHECK(blockMatches(model, 100 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(host.stats.retries == 1);
    CHECK(host.stats.recoveries == 1);
    CHECK(host.runFailures() == 0);
    CHECK(host.multiBlock());
    CHECK(model.state == Model::Tran);
    CHECK(model.hostResets == 0);
    CHECK(model.violations == 0);
  }
  {
    printf("multi-block: a run that keeps failing is read block by block, and too many turn runs off\n");
    Model model;
    fillImage(model, 1024);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    std::vector<uint8_t> out(16 * emmc::kBlockBytes);
    for (unsigned round = 1; round <= 8; ++round) {
      model.failNoResponse = 2;  // both attempts of the CMD18 get no answer; the CMD17s after it do
      CHECK(host.readBlocks(10, 16, out.data()) == emmc::kOk);
      for (unsigned i = 0; i < 16; ++i) CHECK(blockMatches(model, 10 + i, out.data() + i * emmc::kBlockBytes));
      CHECK(host.runFailures() == round);
      CHECK(host.multiBlock() == (round < 8));
    }
    // With runs off, the same read is sixteen CMD17s.
    unsigned runs = model.multiCommands;
    CHECK(host.readBlocks(10, 16, out.data()) == emmc::kOk);
    CHECK(model.multiCommands == runs);
    CHECK(model.violations == 0);
  }
  {
    printf("multi-block with a stale 32-bit count (host v4)\n");
    Model model;
    fillImage(model, 256);
    uint16_t hc2 = emmc::kHc2HostV4;
    memcpy(model.regs + 0x3e, &hc2, 2);
    uint32_t stale = 0x1234;
    memcpy(model.regs + emmc::kRegDmaAddress, &stale, 4);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    std::vector<uint8_t> out(32 * emmc::kBlockBytes);
    CHECK(host.readBlocks(5, 32, out.data()) == emmc::kOk);
    CHECK(!model.overrun);
    for (unsigned i = 0; i < 32; ++i) CHECK(blockMatches(model, 5 + i, out.data() + i * emmc::kBlockBytes));
  }
}

// The window the driver is given: 64 KiB of data, then the descriptor table; bus address base + offset.
struct DmaFixture {
  std::vector<uint8_t> memory;
  emmc::DmaWindow window;
};
unsigned dmaFlushes = 0;
std::vector<std::pair<uint8_t *, size_t>> dmaFlushed;
void dmaFlush(void *virt, size_t bytes) {
  ++dmaFlushes;
  dmaFlushed.push_back({static_cast<uint8_t *>(virt), bytes});
}

void makeDmaWindow(Model &model, DmaFixture &fixture) {
  fixture.memory.assign(emmc::kMaxRunBytes + 4096, 0xee);  // 0xee: what a read must overwrite
  model.dmaMemory.assign(fixture.memory.size(), 0xee);
  fixture.window.data = model.dmaMemory.data();            // one memory, two names
  fixture.window.dataBus = model.dmaBase;
  fixture.window.table = model.dmaMemory.data() + emmc::kMaxRunBytes;
  fixture.window.tableBus = model.dmaBase + emmc::kMaxRunBytes;
  fixture.window.flush = dmaFlush;
  dmaFlushes = 0;
  dmaFlushed.clear();
}

void testDma() {
  {
    printf("dma reads\n");
    Model model;
    fillImage(model, 1024);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    CHECK(host.dma());
    std::vector<uint8_t> out(300 * emmc::kBlockBytes);
    CHECK(host.readBlocks(37, 300, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 300; ++i) CHECK(blockMatches(model, 37 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(model.dmaTransfers == 3);
    CHECK(host.stats.dmaBlocks == 300);
    CHECK(host.stats.blocks == 300);
    CHECK(model.multiCommands == 3);
    CHECK(!model.dmaStray);
    CHECK(model.violations == 0);
    CHECK(model.state == Model::Tran);
    CHECK(dmaFlushes >= 6);  // table and data before, data after, per run
    // The data flush covers the run, before the command and after the transfer.
    bool dataFlushed = false;
    for (auto &entry : dmaFlushed) dataFlushed |= entry.first == fixture.window.data && entry.second == emmc::kMaxRunBytes;
    CHECK(dataFlushed);
    // A single block still goes through the buffer port.
    unsigned before = model.dmaTransfers;
    CHECK(host.readBlocks(5, 1, out.data()) == emmc::kOk);
    CHECK(blockMatches(model, 5, out.data()));
    CHECK(model.dmaTransfers == before);
  }
  {
    printf("dma in 26-bit length mode\n");
    Model model;
    fillImage(model, 512);
    uint16_t hc2 = (1u << 10) | emmc::kHc2HostV4;
    memcpy(model.regs + 0x3e, &hc2, 2);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    std::vector<uint8_t> out(128 * emmc::kBlockBytes);
    CHECK(host.readBlocks(100, 128, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 128; ++i) CHECK(blockMatches(model, 100 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(model.dmaTransfers == 1 && !model.dmaStray && model.violations == 0 && !model.overrun);
  }
  {
    printf("dma refused with 64-bit addressing\n");
    Model model;
    fillImage(model, 256);
    uint16_t hc2 = emmc::kHc2Addr64 | emmc::kHc2HostV4;
    memcpy(model.regs + 0x3e, &hc2, 2);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(!host.setDma(&fixture.window));
    CHECK(!host.dma());
    std::vector<uint8_t> out(16 * emmc::kBlockBytes);
    CHECK(host.readBlocks(10, 16, out.data()) == emmc::kOk);
    CHECK(model.dmaTransfers == 0);
  }
  {
    printf("dma: an ADMA error is recovered through the buffer port\n");
    Model model;
    fillImage(model, 512);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    model.failDmaError = 1;
    std::vector<uint8_t> out(64 * emmc::kBlockBytes);
    CHECK(host.readBlocks(200, 64, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 64; ++i) CHECK(blockMatches(model, 200 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(host.dmaFailures() == 1);
    CHECK(host.dma());
    CHECK(host.stats.retries == 1);
    CHECK(host.stats.dmaBlocks == 0);
    CHECK(model.state == Model::Tran);
    CHECK(model.violations == 0);
    // The next run is DMA again.
    CHECK(host.readBlocks(200, 64, out.data()) == emmc::kOk);
    CHECK(host.stats.dmaBlocks == 64);
  }
  {
    printf("dma: a transfer that never completes\n");
    Model model;
    fillImage(model, 512);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    model.failDmaStall = 1;
    std::vector<uint8_t> out(32 * emmc::kBlockBytes);
    CHECK(host.readBlocks(300, 32, out.data()) == emmc::kOk);
    for (unsigned i = 0; i < 32; ++i) CHECK(blockMatches(model, 300 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(host.dmaFailures() == 1);
    CHECK(model.state == Model::Tran);
    CHECK(model.dataResets >= 1);
    CHECK(model.hostResets == 0);
  }
  {
    printf("dma: too many failures switch it off\n");
    Model model;
    fillImage(model, 512);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    std::vector<uint8_t> out(16 * emmc::kBlockBytes);
    for (unsigned round = 1; round <= 8; ++round) {
      model.failDmaError = 1;
      CHECK(host.readBlocks(50, 16, out.data()) == emmc::kOk);
      CHECK(host.dmaFailures() == round);
      CHECK(host.dma() == (round < 8));
    }
    unsigned transfers = model.dmaTransfers;
    CHECK(host.readBlocks(50, 16, out.data()) == emmc::kOk);
    CHECK(model.dmaTransfers == transfers);  // buffer port from now on
    for (unsigned i = 0; i < 16; ++i) CHECK(blockMatches(model, 50 + i, out.data() + i * emmc::kBlockBytes));
    CHECK(model.violations == 0);
  }
}

// ---- writes ----------------------------------------------------------------------------------------
// The card's image, the window and a host with DMA on and a write range set.
struct WriteRig {
  Model model;
  DmaFixture fixture;
  emmc::Host host;
  explicit WriteRig(uint64_t blocks = 1024, uint64_t first = 200, uint64_t last = 699) {
    fillImage(model, blocks);
    makeDmaWindow(model, fixture);
    host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    if (last >= first) host.setWriteRange(first, last);
  }
};

std::vector<uint8_t> pattern(uint32_t blocks, uint8_t seed) {
  std::vector<uint8_t> data(static_cast<size_t>(blocks) * emmc::kBlockBytes);
  for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(seed + i * 31 + (i >> 9));
  return data;
}

void testWrites() {
  {
    printf("writes: refused without a range, without DMA, outside the range\n");
    Model model;
    fillImage(model, 1024);
    DmaFixture fixture;
    makeDmaWindow(model, fixture);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    host.setMultiBlock(true);
    CHECK(host.setDma(&fixture.window));
    std::vector<uint8_t> data = pattern(8, 1);
    const std::vector<uint8_t> before = model.image;
    CHECK(!host.writable());
    CHECK(host.writeBlocks(300, 8, data.data()) == emmc::kProtocol);  // no range yet
    host.setWriteRange(200, 299);
    CHECK(host.writable());
    CHECK(host.writeBlocks(199, 1, data.data()) == emmc::kProtocol);   // one before
    CHECK(host.writeBlocks(300, 1, data.data()) == emmc::kProtocol);   // one after
    CHECK(host.writeBlocks(295, 8, data.data()) == emmc::kProtocol);   // straddles the end
    CHECK(host.writeBlocks(196, 8, data.data()) == emmc::kProtocol);   // straddles the start
    CHECK(host.writeBlocks(250, 0, data.data()) == emmc::kProtocol);   // nothing
    CHECK(host.writeBlocks(250, 1, nullptr) == emmc::kProtocol);
    CHECK(host.writeBlocks(0xffffffffffffffffull, 2, data.data()) == emmc::kProtocol);  // would wrap
    CHECK(host.writeBlocks(0xfffffffeull, 8, data.data()) == emmc::kProtocol);
    host.setDma(nullptr);
    CHECK(!host.writable());
    CHECK(host.writeBlocks(250, 1, data.data()) == emmc::kProtocol);   // no buffer-port path for writes
    CHECK(model.commandsSeen.count(25) == 0);
    CHECK(model.writeCommands == 0);
    CHECK(model.image == before);
    CHECK(model.violations == 0);
    host.clearWriteRange();
    CHECK(host.setDma(&fixture.window));
    CHECK(host.writeBlocks(250, 1, data.data()) == emmc::kProtocol);   // range cleared
    CHECK(model.writeCommands == 0 && model.image == before);
  }
  {
    printf("writes: runs of blocks, nothing outside the blocks asked for\n");
    WriteRig rig;
    std::vector<uint8_t> data = pattern(300, 7);
    const std::vector<uint8_t> before = rig.model.image;
    CHECK(rig.host.writeBlocks(250, 300, data.data()) == emmc::kOk);
    CHECK(rig.model.writeCommands == 3);                     // 128 + 128 + 44
    CHECK(rig.model.writes.size() == 3);
    CHECK(rig.model.writes[0].first == 250 && rig.model.writes[0].second == 128);
    CHECK(rig.model.writes[2].first == 506 && rig.model.writes[2].second == 44);
    CHECK(memcmp(rig.model.image.data() + 250 * emmc::kBlockBytes, data.data(), data.size()) == 0);
    CHECK(memcmp(rig.model.image.data(), before.data(), 250 * emmc::kBlockBytes) == 0);
    CHECK(memcmp(rig.model.image.data() + 550 * emmc::kBlockBytes, before.data() + 550 * emmc::kBlockBytes,
                 before.size() - 550 * emmc::kBlockBytes) == 0);
    CHECK(rig.model.state == Model::Tran);
    CHECK(rig.host.stats.writeBlocks == 300 && rig.host.stats.writeRuns == 3);
    CHECK(rig.host.stats.writeRetries == 0 && rig.host.stats.writeFailures == 0);
    CHECK(rig.model.violations == 0 && !rig.model.dmaStray);
    // What the host read back through the same path is what was written.
    std::vector<uint8_t> back(300 * emmc::kBlockBytes);
    CHECK(rig.host.readBlocks(250, 300, back.data()) == emmc::kOk);
    CHECK(back == data);
    // The data was cleaned out to memory before the command.
    bool dataFlushed = false;
    for (auto &entry : dmaFlushed) dataFlushed |= entry.first == rig.fixture.window.data && entry.second == emmc::kMaxRunBytes;
    CHECK(dataFlushed);
  }
  {
    printf("writes: the edges of the range\n");
    WriteRig rig(1024, 200, 299);
    std::vector<uint8_t> data = pattern(100, 3);
    CHECK(rig.host.writeBlocks(200, 100, data.data()) == emmc::kOk);   // the whole range
    CHECK(memcmp(rig.model.image.data() + 200 * emmc::kBlockBytes, data.data(), data.size()) == 0);
    CHECK(rig.host.writeBlocks(299, 1, data.data()) == emmc::kOk);     // the last block
    CHECK(rig.host.writeBlocks(200, 1, data.data()) == emmc::kOk);     // the first
    CHECK(rig.model.violations == 0);
  }
  {
    printf("writes: the card is polled until it is ready, and nothing else is sent meanwhile\n");
    WriteRig rig;
    rig.model.programPolls = 40;     // spin polls, then 1 ms sleeps
    std::vector<uint8_t> data = pattern(16, 9);
    CHECK(rig.host.writeBlocks(300, 16, data.data()) == emmc::kOk);
    CHECK(rig.model.violations == 0);   // the model counts any command but CMD13 while it is programming
    CHECK(rig.model.state == Model::Tran);
    CHECK(rig.host.lastStatus() & (1u << 8));
  }
  {
    printf("writes: an ADMA error is recovered and the run repeated\n");
    WriteRig rig;
    rig.model.failWriteError = 1;
    std::vector<uint8_t> data = pattern(64, 5);
    CHECK(rig.host.writeBlocks(400, 64, data.data()) == emmc::kOk);
    CHECK(rig.model.writes.size() == 2);                      // the failed command and the repeat
    CHECK(rig.host.stats.writeRetries == 1 && rig.host.stats.writeFailures == 0 && rig.host.stats.writeRuns == 1);
    CHECK(memcmp(rig.model.image.data() + 400 * emmc::kBlockBytes, data.data(), data.size()) == 0);
    CHECK(rig.model.state == Model::Tran);
    CHECK(rig.model.hostResets == 0);
    CHECK(rig.model.violations == 0);
  }
  {
    printf("writes: a run that keeps failing gives up, and says so\n");
    WriteRig rig;
    rig.model.failWriteError = 100;
    std::vector<uint8_t> data = pattern(8, 2);
    const std::vector<uint8_t> before = rig.model.image;
    CHECK(rig.host.writeBlocks(400, 8, data.data()) != emmc::kOk);
    CHECK(rig.host.stats.writeFailures == 1 && rig.host.stats.writeBlocks == 0);
    CHECK(rig.model.image == before);
    CHECK(rig.model.state == Model::Tran);
    CHECK(rig.model.violations == 0);
  }
  {
    printf("writes: a card that never leaves programming is a timeout, not a hang, and gets no second command\n");
    WriteRig rig;
    rig.model.failWriteNeverReady = 1;
    std::vector<uint8_t> data = pattern(8, 4);
    CHECK(rig.host.writeBlocks(400, 8, data.data()) == emmc::kTimeout);
    CHECK(rig.model.writeCommands == 1);
    CHECK(rig.model.violations == 0);
    CHECK(rig.host.stats.writeFailures == 1);
  }
  {
    printf("writes: error bits in the card's answer fail the run, the repeat succeeds\n");
    WriteRig rig;
    rig.model.extraErrorBits = 1u << 26;   // WP_VIOLATION, reported (and cleared) by the first CMD13
    std::vector<uint8_t> data = pattern(8, 6);
    CHECK(rig.host.writeBlocks(500, 8, data.data()) == emmc::kOk);
    CHECK(rig.model.writes.size() == 2);
    CHECK(rig.host.stats.writeRetries == 1);
    CHECK(rig.model.violations == 0);
  }
  {
    printf("flush: CMD6 FLUSH_CACHE, then the card is polled; no cache is not an error\n");
    WriteRig rig;
    CHECK(rig.host.flushCache() == emmc::kOk);
    CHECK(rig.model.flushCommands == 1 && rig.model.state == Model::Tran && rig.model.violations == 0);
    rig.model.cacheEnabled = false;
    CHECK(rig.host.flushCache() == emmc::kOk);
    CHECK(rig.model.flushCommands == 2 && rig.model.state == Model::Tran && rig.model.violations == 0);
    CHECK(rig.host.stats.flushes == 2);
  }
  {
    printf("writes: an ADMA descriptor never points outside the window\n");
    WriteRig rig;
    std::vector<uint8_t> data = pattern(128, 8);
    CHECK(rig.host.writeBlocks(300, 128, data.data()) == emmc::kOk);
    CHECK(!rig.model.dmaStray);
    // A 26-bit-length controller reads the same 16 KiB descriptors.
    uint16_t hc2 = (1u << 10) | emmc::kHc2HostV4;
    memcpy(rig.model.regs + 0x3e, &hc2, 2);
    CHECK(rig.host.writeBlocks(300, 128, data.data()) == emmc::kOk);
    CHECK(!rig.model.dmaStray && rig.model.violations == 0);
  }
}

void testGpt() {
  const uint64_t blocks = 16384;
  {
    printf("gpt\n");
    Model model;
    fillImage(model, blocks);
    buildGpt(model, blocks);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    std::vector<uint8_t> scratch(emmc::kGptScratchBytes);
    std::vector<emmc::GptEntry> entries(emmc::kGptMaxEntries);
    emmc::GptInfo info;
    int status = emmc::readGpt([&](uint64_t lba, uint32_t n, void *out) { return host.readBlocks(lba, n, out) == emmc::kOk; },
                               scratch.data(), info, entries.data(), emmc::kGptMaxEntries);
    CHECK(status == emmc::kGptOk);
    CHECK(info.headerCrcOk && info.entriesCrcOk);
    CHECK(info.used == 3);
    CHECK(info.altLba + 1 == blocks);
    CHECK(emmc::sameName(entries[2].name, "userdata"));
    CHECK(entries[2].first == 8192 && entries[2].last == blocks - 34);
    char guid[37];
    emmc::formatGuid(info.diskGuid, guid);
    CHECK(strcmp(guid, "5a5a5a5a-5a5a-5a5a-5a5a-5a5a5a5a5a5a") == 0);
    // Damage the entry array: still returned, flagged.
    model.image[2 * emmc::kBlockBytes + 3] ^= 1;
    status = emmc::readGpt([&](uint64_t lba, uint32_t n, void *out) { return host.readBlocks(lba, n, out) == emmc::kOk; },
                           scratch.data(), info, entries.data(), emmc::kGptMaxEntries);
    CHECK(status == emmc::kGptOk && info.headerCrcOk && !info.entriesCrcOk);
  }
  {
    printf("no gpt\n");
    Model model;
    fillImage(model, blocks);
    emmc::Host host = makeHost(model);
    CHECK(host.identify() == emmc::kOk);
    std::vector<uint8_t> scratch(emmc::kGptScratchBytes);
    std::vector<emmc::GptEntry> entries(emmc::kGptMaxEntries);
    emmc::GptInfo info;
    int status = emmc::readGpt([&](uint64_t lba, uint32_t n, void *out) { return host.readBlocks(lba, n, out) == emmc::kOk; },
                               scratch.data(), info, entries.data(), emmc::kGptMaxEntries);
    CHECK(status == emmc::kGptNoSignature);
  }
}

}  // namespace

int main() {
  testCrc();
  testIdentify();
  testReads();
  testRecovery();
  testHostV4();
  testMultiBlock();
  testDma();
  testWrites();
  testGpt();
  if (failures) {
    printf("%d check(s) FAILED\n", failures);
    return 1;
  }
  printf("all checks passed\n");
  return 0;
}
