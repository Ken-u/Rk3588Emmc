// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// EmmcHost.h -- polled eMMC host core for the Rockchip RK3588
// DesignWare Mobile Storage Host Controller ("dwcmshc", SDHCI register set) at
// 0xFE2E0000. Reads by default; writes only into a window the caller declares.
//
// What this file is, and is not:
//
//   * It moves data and nothing else. The firmware (EDK2 EmmcDxe / SdMmcPciHcDxe,
//     before it, U-Boot) has already identified the card, picked the bus width,
//     tuned and clocked it (HS400 on this board). The core keeps that state and
//     never resets the controller unless a transfer wedges it.
//   * It writes the card only inside the block range set by setWriteRange(), through ADMA2
//     (CMD25 + Auto CMD12), and only while DMA is on; without a range every write is refused
//     (kProtocol) before any command goes out, and a range can be cleared again. The range is the one
//     safety net between a bug in a block driver and the root image on the same card, so it is
//     checked here and not only by the callers. The other commands it can issue are CMD6 (only
//     FLUSH_CACHE, see flushCache), CMD7 (select, only if the card was left deselected), CMD12 (abort,
//     only if the card was left mid-transfer), CMD13 (status), CMD17 (read one block) and CMD18 (read
//     a run of blocks, stopped by the controller's Auto CMD12).
//   * It is kernel-agnostic: every platform service (register access, clock,
//     delay, log) comes through `Platform`, so tests/test-emmc-host.cpp links the
//     very same code against a model of the controller on a build machine.
//
// Controller facts this code depends on (measured on the RK3588 EVB10):
//
//   * INT_ENABLE must stay fully open. Clearing it does not mask interrupts, it
//     silences the status bits, and every command then "times out" with a status
//     word of zero. Polled operation clears SIGNAL_ENABLE instead.
//   * TIMEOUT_CONTROL (0x2e) is an 8-bit register and must hold 0x0e before a data
//     command; leaving it at zero makes the data timeout counter expire at once.
//     Never store 32 bits there: that also clobbers POWER_CONTROL and INT_STATUS.
//   * CMD0 on an already selected card never completes. This core never sends it.
//   * A card left in DATA/RCV state accepts only CMD12; anything else times out.
//   * Writes go through ADMA2 only (the first PIO write experiments never started
//     the data phase, and every later "failed write" was a command sent while the card was still
//     programming). After the controller's Transfer Complete the card is NOT ready: it holds DAT0 low
//     and sits in the PRG state, and a command other than CMD13 sent then is an illegal command that
//     the card reports in the response to the one after it. So every write polls CMD13 until the card
//     is back in the transfer state with READY_FOR_DATA set, and never sends a CMD12 of its own after a
//     write (Auto CMD12 already did).
//   * PIO reads work. The buffer port holds
//     one block at a time: wait for BUFFER_READ_READY per block, then read 128
//     32-bit words.
//   * DMA addresses are 32 bits wide (CAPABILITIES bit 28 clear), so the controller reaches
//     the first 4 GiB only. The kernel's pages are there once the boot window is big enough
//     (and a bounce buffer is allocated below 4 GiB regardless); PIO needs none of this, which
//     is why the core starts as PIO-only and DMA (ADMA2, one 64 KiB run per command) is an
//     opt-in that the driver switches on only after it has read the same blocks both ways.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace emmc {

constexpr uint32_t kBlockBytes = 512;

// ---------------------------------------------------------------- registers --
// Standard SDHCI register file.
constexpr uint32_t kRegDmaAddress = 0x00;     // also the 32-bit block count in v4 mode
constexpr uint32_t kRegBlockSize = 0x04;      // 16 bit; 14:12 = SDMA boundary
constexpr uint32_t kRegBlockCount = 0x06;     // 16 bit
constexpr uint32_t kRegArgument = 0x08;
constexpr uint32_t kRegTransferMode = 0x0c;   // 16 bit, written together with the command
constexpr uint32_t kRegCommand = 0x0e;        // 16 bit
constexpr uint32_t kRegResponse = 0x10;
constexpr uint32_t kRegBuffer = 0x20;
constexpr uint32_t kRegPresentState = 0x24;
constexpr uint32_t kRegHostControl = 0x28;    // 8 bit
constexpr uint32_t kRegPowerControl = 0x29;   // 8 bit
constexpr uint32_t kRegClockControl = 0x2c;   // 16 bit
constexpr uint32_t kRegTimeoutControl = 0x2e; // 8 bit
constexpr uint32_t kRegSoftwareReset = 0x2f;  // 8 bit
constexpr uint32_t kRegIntStatus = 0x30;
constexpr uint32_t kRegIntEnable = 0x34;
constexpr uint32_t kRegSignalEnable = 0x38;
constexpr uint32_t kRegAutoCmdStatus = 0x3c;  // 16 bit
constexpr uint32_t kRegHostControl2 = 0x3e;   // 16 bit
constexpr uint32_t kRegCapabilities = 0x40;
constexpr uint32_t kRegCapabilities1 = 0x44;
constexpr uint32_t kRegAdmaError = 0x54;      // ADMA error status
constexpr uint32_t kRegAdmaAddress = 0x58;    // 64 bit: the ADMA2 descriptor table (low word here, high word at +4)
constexpr uint32_t kRegHostVersion = 0xfe;    // 16 bit

constexpr uint32_t kPsCmdInhibit = 1u << 0;
constexpr uint32_t kPsDataInhibit = 1u << 1;
constexpr uint32_t kPsDataAvailable = 1u << 11;

constexpr uint32_t kTmDmaEnable = 1u << 0;
constexpr uint32_t kTmBlockCountEnable = 1u << 1;
constexpr uint32_t kTmAutoCmd12 = 1u << 2;
constexpr uint32_t kTmRead = 1u << 4;
constexpr uint32_t kTmMultiBlock = 1u << 5;

constexpr uint32_t kCmdRespNone = 0x00;
constexpr uint32_t kCmdRespLong = 0x01;
constexpr uint32_t kCmdRespShort = 0x02;
constexpr uint32_t kCmdRespShortBusy = 0x03;
constexpr uint32_t kCmdCrc = 1u << 3;
constexpr uint32_t kCmdIndex = 1u << 4;
constexpr uint32_t kCmdData = 1u << 5;

constexpr uint32_t kIntResponse = 1u << 0;
constexpr uint32_t kIntDataEnd = 1u << 1;
constexpr uint32_t kIntDataAvailable = 1u << 5;
constexpr uint32_t kIntError = 1u << 15;
constexpr uint32_t kIntErrorMask = 0xffff8000u;

constexpr uint32_t kResetAll = 1u << 0;
constexpr uint32_t kResetCmd = 1u << 1;
constexpr uint32_t kResetData = 1u << 2;

constexpr uint32_t kHc1DmaMask = 3u << 3;     // Host Control 1: DMA select
constexpr uint32_t kHc1Adma2 = 2u << 3;       // ADMA2 with 32-bit descriptors
constexpr uint32_t kHc2HostV4 = 1u << 12;
constexpr uint32_t kHc2Addr64 = 1u << 13;     // 64-bit addressing: the descriptors would be 12 bytes, this core writes 8

// An ADMA2 descriptor is 8 bytes: attributes (16 bits), length (16 bits), 32-bit address.
constexpr uint32_t kAdmaValid = 1u << 0;
constexpr uint32_t kAdmaEnd = 1u << 1;
constexpr uint32_t kAdmaTran = 2u << 4;       // action: transfer the data
// Descriptors of 16 KiB: a controller in 16-bit and one in 26-bit length mode (HOST_CONTROL2 bit 10, the
// way the firmware left this one) read the same table.
constexpr uint32_t kAdmaChunk = 16 * 1024;

// DesignWare vendor registers. Vendor area 1 is located by the offset in 0xe8.
constexpr uint32_t kRegVendorArea1 = 0xe8;
constexpr uint32_t kVendorAreaMask = 0xfff;
constexpr uint32_t kVendorEmmcControl = 0x2c;  // inside area 1: bit0 CARD_IS_EMMC, bit8 ENHANCED_STROBE
constexpr uint32_t kRegDllCtrl = 0x800;
constexpr uint32_t kRegDllRxClk = 0x804;
constexpr uint32_t kRegDllTxClk = 0x808;
constexpr uint32_t kRegDllStrbin = 0x80c;
constexpr uint32_t kRegDllCmdout = 0x810;
constexpr uint32_t kRegMiscCon = 0x81c;

// Card status (R1) and states.
constexpr uint32_t kCardStateIdle = 0, kCardStateReady = 1, kCardStateIdent = 2, kCardStateStby = 3,
                   kCardStateTran = 4, kCardStateData = 5, kCardStateRcv = 6, kCardStatePrg = 7,
                   kCardStateDis = 8;
constexpr uint32_t kCardErrorBits = 0xfff80000u;  // OUT_OF_RANGE .. ERROR (bits 31:19)

constexpr uint32_t kSdmaBoundary = 7u << 12;      // U-Boot's and the controller's default (4 KiB)
constexpr uint8_t kDataTimeoutExponent = 0x0e;    // what every measured working transfer used

constexpr uint32_t kCommandTimeoutUs = 300 * 1000;
constexpr uint32_t kDataTimeoutUs = 1000 * 1000;
constexpr uint32_t kResetTimeoutUs = 10 * 1000;
constexpr uint32_t kWriteTimeoutUs = 3 * 1000 * 1000;     // the controller moving one run of up to 64 KiB
constexpr uint32_t kProgramTimeoutUs = 5 * 1000 * 1000;   // the card programming it (garbage collection can take long)
constexpr uint32_t kFlushTimeoutUs = 10 * 1000 * 1000;    // CMD6 FLUSH_CACHE, R1b
// CMD6 SWITCH, access "write byte" (3), EXT_CSD[32] FLUSH_CACHE = 1.
constexpr uint32_t kFlushCacheArgument = (3u << 24) | (32u << 16) | (1u << 8);
constexpr uint32_t kCardReadyForData = 1u << 8;

// Results. Negative so that a block count and an error never collide.
enum Result : int {
  kOk = 0,
  kBusy = -1,       // command/data inhibit never cleared
  kTimeout = -2,    // no response / no data before the deadline
  kIoError = -3,    // the controller raised an error interrupt
  kCardError = -4,  // the card reported an error in its R1 status
  kProtocol = -5,   // the card is in a state this core refuses to drive
  kNoCard = -6,     // nothing answered CMD13
};

inline const char *resultName(int result) {
  switch (result) {
    case kOk: return "ok";
    case kBusy: return "busy";
    case kTimeout: return "timeout";
    case kIoError: return "io-error";
    case kCardError: return "card-error";
    case kProtocol: return "protocol";
    case kNoCard: return "no-card";
    default: return "?";
  }
}

// Everything the core needs from its host environment.
struct Platform {
  uint32_t (*read)(void *base, uint32_t offset, unsigned width);              // width 8 / 16 / 32
  void (*write)(void *base, uint32_t offset, unsigned width, uint32_t value);
  uint64_t (*nowUs)();
  void (*delayUs)(uint32_t microseconds);
  void (*log)(const char *format, ...) __attribute__((format(printf, 1, 2)));
};

struct Stats {
  uint64_t blocks = 0;       // blocks delivered to callers
  uint64_t commands = 0;     // commands sent
  uint64_t retries = 0;      // block reads that had to be repeated
  uint64_t recoveries = 0;   // recover() calls
  uint64_t failures = 0;     // block reads that gave up
  uint64_t hostResets = 0;   // RESET_ALL, the last resort
  uint64_t readUs = 0;       // time spent inside block reads
  uint64_t dmaBlocks = 0;    // blocks that arrived by DMA (a part of `blocks`)
  uint64_t writeBlocks = 0;  // blocks written and acknowledged by the card (READY_FOR_DATA)
  uint64_t writeRuns = 0;    // write commands that succeeded
  uint64_t writeRetries = 0; // write runs that had to be repeated
  uint64_t writeFailures = 0;// write runs that gave up
  uint64_t writeUs = 0;      // time spent inside writeBlocks
  uint64_t flushes = 0;      // FLUSH_CACHE commands sent
};

// Memory the controller can reach: physically contiguous, below 4 GiB, mapped cacheable by the kernel.
// `data` holds one run of blocks (kMaxRunBytes), `table` the ADMA2 descriptors of that run. `flush` cleans
// and invalidates the data cache for [virt, virt + bytes): the controller does not snoop the CPU's caches.
struct DmaWindow {
  uint8_t *data;
  uint32_t dataBus;
  uint8_t *table;
  uint32_t tableBus;
  void (*flush)(void *virt, size_t bytes);
};
constexpr uint32_t kMaxRunBlocks = 128;                         // one command: 64 KiB, what the callers ask for
constexpr uint32_t kMaxRunBytes = kMaxRunBlocks * kBlockBytes;
constexpr uint32_t kDmaTableBytes = (kMaxRunBytes / kAdmaChunk) * 8;

inline uint32_t crc32(const void *data, size_t length, uint32_t seed = 0) {
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  uint32_t crc = ~seed;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

class Host {
 public:
  Stats stats;

  void attach(const Platform *platform, void *registers) {
    plat_ = platform;
    base_ = registers;
  }

  uint32_t rca() const { return rca_; }
  bool usable() const { return usable_; }
  bool hostV4() const { return v4_; }

  // ---- register helpers ----------------------------------------------------
  uint32_t rd32(uint32_t offset) const { return plat_->read(base_, offset, 32); }
  uint32_t rd16(uint32_t offset) const { return plat_->read(base_, offset, 16); }
  uint32_t rd8(uint32_t offset) const { return plat_->read(base_, offset, 8); }
  void wr32(uint32_t offset, uint32_t value) const { plat_->write(base_, offset, 32, value); }
  void wr8(uint32_t offset, uint32_t value) const { plat_->write(base_, offset, 8, value & 0xff); }
  // 16-bit register pairs (BLOCK_SIZE/BLOCK_COUNT, TRANSFER_MODE/COMMAND) are
  // always written as one 32-bit store, the way the measured working drivers do.
  void wrPair(uint32_t offset, uint32_t low, uint32_t high) const {
    wr32(offset, (low & 0xffff) | ((high & 0xffff) << 16));
  }

  // ---- diagnostics -----------------------------------------------------------
  // Read-only snapshot of the controller as the firmware left it.
  void dump(const char *tag) {
    uint32_t version = rd16(kRegHostVersion);
    uint32_t hc2 = rd16(kRegHostControl2);
    v4_ = (hc2 & kHc2HostV4) != 0;
    plat_->log("NativeEmmc[%s]: version=0x%04x caps=0x%08x/0x%08x present=0x%08x host-ctrl=0x%02x power=0x%02x "
               "clock=0x%04x timeout=0x%02x host-ctrl2=0x%04x (v4=%d)\n",
               tag, version, rd32(kRegCapabilities), rd32(kRegCapabilities1), rd32(kRegPresentState),
               rd8(kRegHostControl), rd8(kRegPowerControl), rd16(kRegClockControl), rd8(kRegTimeoutControl),
               hc2, v4_ ? 1 : 0);
    plat_->log("NativeEmmc[%s]: int-status=0x%08x int-enable=0x%08x signal-enable=0x%08x auto-cmd=0x%04x "
               "arg=0x%08x dma/count32=0x%08x blk-size=0x%04x blk-count=0x%04x\n",
               tag, rd32(kRegIntStatus), rd32(kRegIntEnable), rd32(kRegSignalEnable), rd16(kRegAutoCmdStatus),
               rd32(kRegArgument), rd32(kRegDmaAddress), rd16(kRegBlockSize), rd16(kRegBlockCount));
    uint32_t area1 = rd16(kRegVendorArea1) & kVendorAreaMask;
    plat_->log("NativeEmmc[%s]: dwcmshc area1=0x%03x emmc-control=0x%04x dll-ctrl=0x%08x rxclk=0x%08x txclk=0x%08x "
               "strbin=0x%08x cmdout=0x%08x misc=0x%08x\n",
               tag, area1, area1 ? rd16(area1 + kVendorEmmcControl) : 0xffff, rd32(kRegDllCtrl), rd32(kRegDllRxClk),
               rd32(kRegDllTxClk), rd32(kRegDllStrbin), rd32(kRegDllCmdout), rd32(kRegMiscCon));
  }

  // ---- bring-up --------------------------------------------------------------
  // Keep the firmware's bus state, find out which RCA the card answers to and
  // bring it to the transfer state. No reset, no CMD0, no re-identification.
  int identify() {
    usable_ = false;
    v4_ = (rd16(kRegHostControl2) & kHc2HostV4) != 0;
    // Status reporting must stay open; polled operation only clears the signals.
    wr32(kRegIntEnable, 0xffffffffu);
    wr32(kRegSignalEnable, 0);
    wr32(kRegIntStatus, 0xffffffffu);

    uint32_t status = 0;
    int result = kNoCard;
    // The eMMC specification gives the host the choice of RCA; EDK2 and U-Boot both use 1.
    for (uint32_t candidate = 1; candidate <= 4; ++candidate) {
      int error = commandRaw(13, candidate << 16, kCmdRespShort, &status);
      if (error == kOk) {
        rca_ = candidate;
        result = kOk;
        break;
      }
    }
    if (result != kOk) {
      plat_->log("NativeEmmc: no card answers CMD13 with RCA 1..4 (present=0x%08x)\n", rd32(kRegPresentState));
      return kNoCard;
    }
    plat_->log("NativeEmmc: CMD13 rca=%u status=0x%08x state=%u\n", rca_, status, (status >> 9) & 0xf);

    // Walk the card to the transfer state. Every round either succeeds, makes
    // progress (select / abort / wait) or gives up; a card in any other state
    // (idle, ready, identification) was never brought up by the firmware and
    // is not ours to initialise.
    for (unsigned round = 0; round < 100; ++round) {
      uint32_t state = (status >> 9) & 0xf;
      if (state == kCardStateTran) {
        usable_ = true;
        return kOk;
      }
      int error = kOk;
      if (state == kCardStateStby) {
        plat_->log("NativeEmmc: card was left deselected, CMD7\n");
        error = commandRaw(7, rca_ << 16, kCmdRespShortBusy, &status);
      } else if (state == kCardStateData || state == kCardStateRcv) {
        plat_->log("NativeEmmc: card was left mid-transfer (state %u), CMD12\n", state);
        error = commandRaw(12, 0, kCmdRespShortBusy, &status);
      } else if (state == kCardStatePrg || state == kCardStateDis) {
        plat_->delayUs(10 * 1000);
      } else {
        plat_->log("NativeEmmc: card is in state %u, refusing to drive it\n", state);
        return kProtocol;
      }
      if (error != kOk) return error;
      error = commandRaw(13, rca_ << 16, kCmdRespShort, &status);
      if (error != kOk) return error;
      plat_->log("NativeEmmc: CMD13 status=0x%08x state=%u\n", status, (status >> 9) & 0xf);
    }
    return kProtocol;
  }

  // ---- reads -----------------------------------------------------------------
  // One command per run of blocks (CMD18 + Auto CMD12) instead of one CMD17 per block. A command
  // costs the card its access time, about a hundred microseconds, so a 4 KiB read as eight CMD17s
  // is several times slower than one CMD18. Off until the caller has compared both on the real card.
  void setMultiBlock(bool on) { multi_ = on; }
  bool multiBlock() const { return multi_; }
  uint32_t runFailures() const { return runFailures_; }

  // Runs through ADMA2 into `window` instead of through the buffer port: the CPU only copies the finished
  // run out of the window. Needs multi-block runs, 32-bit descriptors (HOST_CONTROL2 bit 13 clear) and a
  // window below 4 GiB; refused (returns false) otherwise. nullptr switches it off.
  bool setDma(const DmaWindow *window) {
    dma_ = nullptr;
    dmaOn_ = false;
    if (!window) return true;
    if (!window->data || !window->table || !window->flush || (window->tableBus & 7) || (window->dataBus & 3)) return false;
    if (rd16(kRegHostControl2) & kHc2Addr64) return false;
    dma_ = window;
    dmaOn_ = true;
    dmaFailures_ = 0;
    return true;
  }
  bool dma() const { return dma_ && dmaOn_; }
  uint32_t dmaFailures() const { return dmaFailures_; }

  // Read `count` 512-byte blocks. A failed run is read again block by block; each block is retried
  // (with recovery in between) before the read gives up.
  int readBlocks(uint64_t lba, uint32_t count, void *out) {
    if (!usable_) return kNoCard;
    uint8_t *bytes = static_cast<uint8_t *>(out);
    uint64_t begin = plat_->nowUs();
    int result = kOk;
    uint32_t done = 0;
    while (done < count && result == kOk) {
      uint32_t run = count - done;
      if (!multi_ || run == 1) {
        result = readSingles(lba + done, 1, bytes + static_cast<size_t>(done) * kBlockBytes);
        if (result == kOk) ++done;
        continue;
      }
      if (run > kMaxRun) run = kMaxRun;
      uint8_t *at = bytes + static_cast<size_t>(done) * kBlockBytes;
      if (readRunRetry(lba + done, run, at) == kOk) {
        stats.blocks += run;
        done += run;
        continue;
      }
      // The run kept failing. Its blocks may still be readable one by one; a card that fails runs
      // over and over is taken off the multi-block path for good.
      if (++runFailures_ >= kRunFailureLimit) {
        multi_ = false;
        plat_->log("NativeEmmc: %u failed multi-block reads, back to one block per command\n", runFailures_);
      }
      result = readSingles(lba + done, run, at);
      if (result == kOk) done += run;
    }
    stats.readUs += plat_->nowUs() - begin;
    return result;
  }

  // ---- writes ----------------------------------------------------------------
  // The only blocks writeBlocks may touch: [first, last]. Nothing can be written until this is set,
  // and not while DMA is off (writes have no buffer-port path).
  void setWriteRange(uint64_t first, uint64_t last) {
    writeRange_ = last >= first;
    writeFirst_ = first;
    writeLast_ = last;
  }
  void clearWriteRange() { writeRange_ = false; }
  bool writeRange(uint64_t *first, uint64_t *last) const {
    if (first) *first = writeFirst_;
    if (last) *last = writeLast_;
    return writeRange_;
  }
  bool writable() const { return usable_ && writeRange_ && dma(); }

  // Write `count` 512-byte blocks from `in`. Runs of up to kMaxRunBlocks; each run is a CMD25 through
  // ADMA2 followed by CMD13 polling until the card has finished programming it, repeated once (after a
  // recovery) when it fails. Returns kOk only when the last run is acknowledged by the card.
  int writeBlocks(uint64_t lba, uint32_t count, const void *in) {
    if (!usable_) return kNoCard;
    if (!writeRange_ || !dma() || !in || count == 0 || lba < writeFirst_ || lba > writeLast_ || count - 1 > writeLast_ - lba) {
      if (logBudget())
        plat_->log("NativeEmmc: write of %u blocks at %llu refused (range %s %llu..%llu, DMA %s)\n", count,
                   static_cast<unsigned long long>(lba), writeRange_ ? "set" : "not set", static_cast<unsigned long long>(writeFirst_),
                   static_cast<unsigned long long>(writeLast_), dma() ? "on" : "off");
      return kProtocol;
    }
    const uint8_t *bytes = static_cast<const uint8_t *>(in);
    const uint64_t begin = plat_->nowUs();
    int result = kOk;
    uint32_t done = 0;
    while (done < count && result == kOk) {
      uint32_t run = count - done;
      if (run > kMaxRun) run = kMaxRun;
      result = writeRunRetry(lba + done, run, bytes + static_cast<size_t>(done) * kBlockBytes);
      if (result == kOk) done += run;
    }
    stats.writeUs += plat_->nowUs() - begin;
    return result;
  }

  // Make the card put what it holds in its cache on the medium (CMD6 SWITCH, EXT_CSD[32] FLUSH_CACHE = 1),
  // then wait until it is ready again. A card without a cache, or with it switched off, answers with
  // SWITCH_ERROR on the next status; that is not an error here, it only means there was nothing to flush.
  int flushCache() {
    if (!usable_) return kNoCard;
    ++stats.flushes;
    uint32_t status = 0;
    int result = commandRaw(6, kFlushCacheArgument, kCmdRespShortBusy, &status, kFlushTimeoutUs);
    if (result != kOk) {
      recover("flush cache");
      return result;
    }
    result = waitProgrammed(kProgramTimeoutUs);
    if (result == kOk && (lastStatus_ & (1u << 7)) && !switchErrorSeen_) {
      switchErrorSeen_ = true;
      plat_->log("NativeEmmc: FLUSH_CACHE answered SWITCH_ERROR (status 0x%08x): the card has no cache or it is off\n", lastStatus_);
    }
    return result;
  }
  uint32_t lastStatus() const { return lastStatus_; }

 private:
  const Platform *plat_ = nullptr;
  void *base_ = nullptr;
  uint32_t rca_ = 1;
  bool usable_ = false;
  bool v4_ = false;
  bool multi_ = false;
  const DmaWindow *dma_ = nullptr;
  bool dmaOn_ = false;
  uint32_t dmaFailures_ = 0;
  uint32_t runFailures_ = 0;
  uint32_t errorLogs_ = 0;
  bool writeRange_ = false;
  uint64_t writeFirst_ = 0, writeLast_ = 0;
  uint32_t lastStatus_ = 0;
  bool switchErrorSeen_ = false;

  static constexpr unsigned kAttempts = 3;
  static constexpr unsigned kRunAttempts = 2;
  static constexpr uint32_t kMaxRun = kMaxRunBlocks;
  static constexpr uint32_t kRunFailureLimit = 8;

  bool logBudget() { return errorLogs_++ < 24; }

  int waitState(uint32_t mask, uint32_t want, uint32_t timeoutUs) const {
    uint64_t deadline = plat_->nowUs() + timeoutUs;
    for (;;) {
      if ((rd32(kRegPresentState) & mask) == want) return kOk;
      if (plat_->nowUs() > deadline) return kBusy;
      plat_->delayUs(1);
    }
  }

  // Reset one block of the controller and wait for the bit to clear. A reset that
  // is still asserted when the next command is written makes that command vanish
  // without any interrupt, which looks exactly like a dead card.
  int reset(uint32_t bits) {
    wr8(kRegSoftwareReset, bits);
    uint64_t deadline = plat_->nowUs() + kResetTimeoutUs;
    for (;;) {
      if ((rd8(kRegSoftwareReset) & bits) == 0) {
        plat_->delayUs(1);
        return kOk;
      }
      if (plat_->nowUs() > deadline) {
        plat_->log("NativeEmmc: software reset 0x%x did not clear\n", bits);
        return kTimeout;
      }
      plat_->delayUs(1);
    }
  }

  // One command without a data phase. Never recovers: recover() itself uses it.
  int commandRaw(uint8_t index, uint32_t argument, uint32_t responseType, uint32_t *response, uint32_t timeoutUs = kCommandTimeoutUs) {
    uint32_t flags = responseType == kCmdRespNone ? 0 : (kCmdCrc | kCmdIndex | responseType);
    if (waitState(kPsCmdInhibit, 0, kCommandTimeoutUs) != kOk) {
      if (logBudget()) plat_->log("NativeEmmc: cmd %u: command line busy (present=0x%08x)\n", index, rd32(kRegPresentState));
      reset(kResetCmd);
      return kBusy;
    }
    ++stats.commands;
    wr32(kRegIntStatus, 0xffffffffu);
    wr32(kRegArgument, argument);
    wrPair(kRegTransferMode, 0, (static_cast<uint32_t>(index) << 8) | flags);

    uint64_t deadline = plat_->nowUs() + timeoutUs;
    for (;;) {
      uint32_t status = rd32(kRegIntStatus);
      if (status & kIntErrorMask) {
        if (logBudget()) plat_->log("NativeEmmc: cmd %u failed, int-status=0x%08x\n", index, status);
        wr32(kRegIntStatus, status);
        reset(kResetCmd);
        return (status & (1u << 16)) ? kTimeout : kIoError;
      }
      if (status & kIntResponse) {
        // R1b: the card holds DAT0 low while it is busy.
        if (responseType == kCmdRespShortBusy && (rd32(kRegPresentState) & kPsDataInhibit)) {
          if (plat_->nowUs() > deadline) {
            reset(kResetData);
            return kTimeout;
          }
          plat_->delayUs(1);
          continue;
        }
        if (response && responseType != kCmdRespNone) *response = rd32(kRegResponse);
        wr32(kRegIntStatus, status);
        return kOk;
      }
      if (plat_->nowUs() > deadline) {
        if (logBudget())
          plat_->log("NativeEmmc: cmd %u timed out, int-status=0x%08x present=0x%08x clock=0x%04x\n", index, status,
                     rd32(kRegPresentState), rd16(kRegClockControl));
        reset(kResetCmd);
        return kTimeout;
      }
      plat_->delayUs(1);
    }
  }

  // Bring the card and the data path back to a usable state after a failed
  // transfer: CMD12 only if the card is mid-transfer (sent to a card in TRAN it
  // is an illegal command that poisons the next status), then DATA reset, and
  // RESET_ALL only when nothing else frees the data lines.
  void recover(const char *why) {
    ++stats.recoveries;
    if (logBudget()) plat_->log("NativeEmmc: recovering (%s)\n", why);
    wr32(kRegIntStatus, 0xffffffffu);
    uint32_t status = 0;
    if (commandRaw(13, rca_ << 16, kCmdRespShort, &status) == kOk) {
      uint32_t state = (status >> 9) & 0xf;
      if (state != kCardStateData && state != kCardStateRcv) {
        wr32(kRegIntStatus, 0xffffffffu);
        if (!(rd32(kRegPresentState) & kPsDataInhibit)) return;
      }
    }
    commandRaw(12, 0, kCmdRespShortBusy, &status);
    wr32(kRegIntStatus, 0xffffffffu);
    if (!(rd32(kRegPresentState) & kPsDataInhibit)) return;
    if (reset(kResetData) == kOk) {
      wr32(kRegIntStatus, 0xffffffffu);
      if (!(rd32(kRegPresentState) & kPsDataInhibit)) return;
    }
    plat_->log("NativeEmmc: data path stuck (present=0x%08x), resetting the host\n", rd32(kRegPresentState));
    ++stats.hostResets;
    reset(kResetAll);
    wr32(kRegIntEnable, 0xffffffffu);
    wr32(kRegSignalEnable, 0);
    wr32(kRegIntStatus, 0xffffffffu);
    if (commandRaw(13, rca_ << 16, kCmdRespShort, &status) == kOk)
      plat_->log("NativeEmmc: after host reset the card is in state %u (status 0x%08x)\n", (status >> 9) & 0xf, status);
    else
      plat_->log("NativeEmmc: after host reset the card does not answer CMD13\n");
  }

  int readBlockRetry(uint64_t lba, uint8_t *out) {
    int result = kIoError;
    for (unsigned attempt = 0; attempt < kAttempts; ++attempt) {
      result = readBlockPio(lba, out);
      if (result == kOk) return kOk;
      ++stats.retries;
      if (logBudget())
        plat_->log("NativeEmmc: read of block %llu failed (%s), attempt %u of %u\n",
                   static_cast<unsigned long long>(lba), resultName(result), attempt + 1, kAttempts);
      recover(resultName(result));
    }
    ++stats.failures;
    return result;
  }

  // `count` blocks, one CMD17 each.
  int readSingles(uint64_t lba, uint32_t count, uint8_t *out) {
    for (uint32_t index = 0; index < count; ++index) {
      int result = readBlockRetry(lba + index, out + static_cast<size_t>(index) * kBlockBytes);
      if (result != kOk) return result;
      ++stats.blocks;
    }
    return kOk;
  }

  // One CMD18 run, repeated (after a recovery) when it fails. With DMA on, the first attempt is the DMA
  // one and the second goes through the buffer port: a failing DMA never costs more than one run, and
  // enough failed ones switch it off for good.
  int readRunRetry(uint64_t lba, uint32_t count, uint8_t *out) {
    int result = kIoError;
    for (unsigned attempt = 0; attempt < kRunAttempts; ++attempt) {
      const bool byDma = dma() && attempt == 0;
      result = byDma ? readRunDma(lba, count, out) : readRunPio(lba, count, out);
      if (result == kOk) {
        if (byDma) stats.dmaBlocks += count;
        return kOk;
      }
      if (byDma && ++dmaFailures_ >= kRunFailureLimit) {
        dmaOn_ = false;
        plat_->log("NativeEmmc: %u failed DMA reads, back to the buffer port\n", dmaFailures_);
      }
      ++stats.retries;
      if (logBudget())
        plat_->log("NativeEmmc: read of %u blocks at %llu by %s failed (%s), attempt %u of %u\n", count,
                   static_cast<unsigned long long>(lba), byDma ? "DMA" : "PIO", resultName(result), attempt + 1, kRunAttempts);
      recover(resultName(result));
    }
    return result;
  }

  // One CMD18 with ADMA2: the controller writes the run into dma_->data, the descriptor table in
  // dma_->table says where, Auto CMD12 ends the command and Transfer Complete says it is over.
  int readRunDma(uint64_t lba, uint32_t count, uint8_t *out) {
    if (!dma_ || count == 0 || count > kMaxRun || lba + count > 0x100000000ull) return kProtocol;
    if (waitState(kPsCmdInhibit | kPsDataInhibit, 0, kCommandTimeoutUs) != kOk) return kBusy;
    const uint32_t bytes = count * kBlockBytes;
    const uint32_t chunks = (bytes + kAdmaChunk - 1) / kAdmaChunk;
    uint32_t *table = reinterpret_cast<uint32_t *>(dma_->table);
    for (uint32_t i = 0; i < chunks; ++i) {
      const uint32_t length = bytes - i * kAdmaChunk < kAdmaChunk ? bytes - i * kAdmaChunk : kAdmaChunk;
      table[2 * i] = kAdmaValid | kAdmaTran | (i + 1 == chunks ? kAdmaEnd : 0) | (length << 16);
      table[2 * i + 1] = dma_->dataBus + i * kAdmaChunk;
    }
    // The table and the data area go out to memory first, so that no dirty line is written back over the
    // blocks the controller is about to deliver.
    dma_->flush(dma_->table, chunks * 8);
    dma_->flush(dma_->data, bytes);

    ++stats.commands;
    wr32(kRegIntStatus, 0xffffffffu);
    if (v4_) wr32(kRegDmaAddress, 0);  // see readBlockPio: a stale 32-bit block count would win
    wr8(kRegHostControl, (rd8(kRegHostControl) & ~kHc1DmaMask) | kHc1Adma2);
    wr32(kRegAdmaAddress, dma_->tableBus);
    wr32(kRegAdmaAddress + 4, 0);
    wrPair(kRegBlockSize, kSdmaBoundary | kBlockBytes, count);
    wr8(kRegTimeoutControl, kDataTimeoutExponent);
    wr32(kRegArgument, static_cast<uint32_t>(lba));
    wrPair(kRegTransferMode, kTmDmaEnable | kTmBlockCountEnable | kTmAutoCmd12 | kTmRead | kTmMultiBlock,
           (18u << 8) | kCmdData | kCmdCrc | kCmdIndex | kCmdRespShort);

    uint64_t deadline = plat_->nowUs() + kDataTimeoutUs;
    uint32_t status;
    for (;;) {
      status = rd32(kRegIntStatus);
      if (status & kIntErrorMask) {
        if (logBudget()) plat_->log("NativeEmmc: DMA read error, adma-error=0x%08x\n", rd32(kRegAdmaError));
        return failedTransfer(status);
      }
      if (status & kIntDataEnd) break;
      if (plat_->nowUs() > deadline) return failedTimeout(status);
      plat_->delayUs(1);
    }
    wr32(kRegIntStatus, status);
    const uint32_t cardStatus = rd32(kRegResponse);
    // What the CPU read ahead of the controller's writes is dropped before it is used.
    dma_->flush(dma_->data, bytes);
    __builtin_memcpy(out, dma_->data, bytes);
    if ((cardStatus & kCardErrorBits) && logBudget())
      plat_->log("NativeEmmc: CMD18 (DMA) block %llu: card status 0x%08x carries error bits\n",
                 static_cast<unsigned long long>(lba), cardStatus);
    return kOk;
  }

  // One CMD18 with a PIO data phase: `count` blocks (1..kMaxRun) in one command, the controller
  // sends the CMD12 that ends it (Auto CMD12) and raises Transfer Complete afterwards, so the card
  // is back in the transfer state when this returns. The buffer port still holds one block at a
  // time: BUFFER_READ_READY per block, then 128 words, exactly as in a single-block read.
  int readRunPio(uint64_t lba, uint32_t count, uint8_t *out) {
    if (count == 0 || count > kMaxRun || lba + count > 0x100000000ull) return kProtocol;  // 32-bit sector address
    if (waitState(kPsCmdInhibit | kPsDataInhibit, 0, kCommandTimeoutUs) != kOk) return kBusy;
    ++stats.commands;
    wr32(kRegIntStatus, 0xffffffffu);
    if (v4_) wr32(kRegDmaAddress, 0);  // see readBlockPio: a stale 32-bit block count would win
    wrPair(kRegBlockSize, kSdmaBoundary | kBlockBytes, count);
    wr8(kRegTimeoutControl, kDataTimeoutExponent);
    wr32(kRegArgument, static_cast<uint32_t>(lba));
    wrPair(kRegTransferMode, kTmBlockCountEnable | kTmAutoCmd12 | kTmRead | kTmMultiBlock,
           (18u << 8) | kCmdData | kCmdCrc | kCmdIndex | kCmdRespShort);

    uint32_t cardStatus = 0;
    uint32_t status;
    for (uint32_t index = 0; index < count; ++index) {
      uint64_t deadline = plat_->nowUs() + kDataTimeoutUs;  // per block: a long run is not a slow block
      for (;;) {
        status = rd32(kRegIntStatus);
        if (status & kIntErrorMask) return failedTransfer(status);
        if ((status & kIntDataAvailable) && (rd32(kRegPresentState) & kPsDataAvailable)) break;
        if (plat_->nowUs() > deadline) return failedTimeout(status);
        plat_->delayUs(1);
      }
      if (index == 0) cardStatus = rd32(kRegResponse);
      // Acknowledged before the block is read: reading is what lets the controller take the next one,
      // and its READY bit must not be wiped by this acknowledgement.
      wr32(kRegIntStatus, kIntDataAvailable);
      uint32_t words[kBlockBytes / 4];
      for (unsigned i = 0; i < kBlockBytes / 4; ++i) words[i] = rd32(kRegBuffer);
      __builtin_memcpy(out + static_cast<size_t>(index) * kBlockBytes, words, kBlockBytes);
    }
    uint64_t deadline = plat_->nowUs() + kDataTimeoutUs;
    for (;;) {
      status = rd32(kRegIntStatus);
      if (status & kIntErrorMask) return failedTransfer(status);
      if (status & kIntDataEnd) break;
      if (plat_->nowUs() > deadline) return failedTimeout(status);
      plat_->delayUs(1);
    }
    wr32(kRegIntStatus, status);
    if ((cardStatus & kCardErrorBits) && logBudget())
      plat_->log("NativeEmmc: CMD18 block %llu: card status 0x%08x carries error bits\n",
                 static_cast<unsigned long long>(lba), cardStatus);
    return kOk;
  }

  // Poll CMD13 until the card is back in the transfer state with READY_FOR_DATA set. Every answer is
  // looked at for error bits (they are cleared by being read, so one skipped answer would lose them).
  // The first polls spin, then they sleep 1 ms apart: programming a run takes from a few hundred
  // microseconds to many milliseconds, and garbage collection can make it much longer.
  int waitProgrammed(uint32_t timeoutUs) {
    const uint64_t deadline = plat_->nowUs() + timeoutUs;
    uint32_t errors = 0;
    for (unsigned poll = 0;; ++poll) {
      uint32_t status = 0;
      int error = commandRaw(13, rca_ << 16, kCmdRespShort, &status);
      if (error != kOk) return error;
      lastStatus_ = status;
      errors |= status & kCardErrorBits;
      const uint32_t state = (status >> 9) & 0xf;
      if (state == kCardStateTran && (status & kCardReadyForData)) {
        if (errors) {
          if (logBudget()) plat_->log("NativeEmmc: the card reports error bits 0x%08x after programming (status 0x%08x)\n", errors, status);
          return kCardError;
        }
        return kOk;
      }
      if (state != kCardStateTran && state != kCardStatePrg && state != kCardStateRcv && state != kCardStateData) {
        if (logBudget()) plat_->log("NativeEmmc: the card is in state %u (status 0x%08x) while a write should be programming\n", state, status);
        return kProtocol;
      }
      if (plat_->nowUs() > deadline) {
        if (logBudget()) plat_->log("NativeEmmc: the card did not finish programming in %u us (status 0x%08x)\n", timeoutUs, status);
        return kTimeout;
      }
      plat_->delayUs(poll < 16 ? 50 : 1000);
    }
  }

  int writeRunRetry(uint64_t lba, uint32_t count, const uint8_t *in) {
    int result = kIoError;
    for (unsigned attempt = 0; attempt < kRunAttempts; ++attempt) {
      // After a failed attempt the card may still be programming what it got: no command that needs the
      // data lines is legal until it is ready again.
      if (attempt) {
        // Error bits of the failed attempt may still be pending in this answer: they are not a reason to stop.
        const int ready = waitProgrammed(kProgramTimeoutUs);
        if (ready != kOk && ready != kCardError) {
          result = ready;
          break;
        }
      }
      result = writeRunDma(lba, count, in);
      if (result == kOk) {
        ++stats.writeRuns;
        stats.writeBlocks += count;
        return kOk;
      }
      ++stats.writeRetries;
      if (logBudget())
        plat_->log("NativeEmmc: write of %u blocks at %llu failed (%s), attempt %u of %u\n", count, static_cast<unsigned long long>(lba),
                   resultName(result), attempt + 1, kRunAttempts);
      recover(resultName(result));
    }
    ++stats.writeFailures;
    return result;
  }

  // One CMD25 with ADMA2: the CPU copies the run into dma_->data and cleans it out to memory, the controller
  // reads it through the descriptors, Auto CMD12 ends the command, Transfer Complete says the controller is
  // done, and CMD13 polling says the card is.
  int writeRunDma(uint64_t lba, uint32_t count, const uint8_t *in) {
    if (!dma_ || count == 0 || count > kMaxRun || lba + count > 0x100000000ull) return kProtocol;
    if (waitState(kPsCmdInhibit | kPsDataInhibit, 0, kCommandTimeoutUs) != kOk) return kBusy;
    const uint32_t bytes = count * kBlockBytes;
    const uint32_t chunks = (bytes + kAdmaChunk - 1) / kAdmaChunk;
    uint32_t *table = reinterpret_cast<uint32_t *>(dma_->table);
    for (uint32_t i = 0; i < chunks; ++i) {
      const uint32_t length = bytes - i * kAdmaChunk < kAdmaChunk ? bytes - i * kAdmaChunk : kAdmaChunk;
      table[2 * i] = kAdmaValid | kAdmaTran | (i + 1 == chunks ? kAdmaEnd : 0) | (length << 16);
      table[2 * i + 1] = dma_->dataBus + i * kAdmaChunk;
    }
    __builtin_memcpy(dma_->data, in, bytes);
    // Out to memory: the controller reads the table and the data, and does not look into the CPU's caches.
    dma_->flush(dma_->table, chunks * 8);
    dma_->flush(dma_->data, bytes);

    ++stats.commands;
    wr32(kRegIntStatus, 0xffffffffu);
    if (v4_) wr32(kRegDmaAddress, 0);  // see readBlockPio: a stale 32-bit block count would win
    wr8(kRegHostControl, (rd8(kRegHostControl) & ~kHc1DmaMask) | kHc1Adma2);
    wr32(kRegAdmaAddress, dma_->tableBus);
    wr32(kRegAdmaAddress + 4, 0);
    wrPair(kRegBlockSize, kSdmaBoundary | kBlockBytes, count);
    wr8(kRegTimeoutControl, kDataTimeoutExponent);
    wr32(kRegArgument, static_cast<uint32_t>(lba));
    // Not kTmRead: a write. Multi-block, block count enabled, stopped by Auto CMD12.
    wrPair(kRegTransferMode, kTmDmaEnable | kTmBlockCountEnable | kTmAutoCmd12 | kTmMultiBlock,
           (25u << 8) | kCmdData | kCmdCrc | kCmdIndex | kCmdRespShort);

    uint64_t deadline = plat_->nowUs() + kWriteTimeoutUs;
    uint32_t status;
    for (;;) {
      status = rd32(kRegIntStatus);
      if (status & kIntErrorMask) {
        if (logBudget()) plat_->log("NativeEmmc: DMA write error, adma-error=0x%08x\n", rd32(kRegAdmaError));
        return failedTransfer(status);
      }
      if (status & kIntDataEnd) break;
      if (plat_->nowUs() > deadline) return failedTimeout(status);
      plat_->delayUs(1);
    }
    wr32(kRegIntStatus, status);
    const uint32_t cardStatus = rd32(kRegResponse);
    if ((cardStatus & kCardErrorBits) && logBudget())
      plat_->log("NativeEmmc: CMD25 block %llu: card status 0x%08x carries error bits\n", static_cast<unsigned long long>(lba), cardStatus);
    return waitProgrammed(kProgramTimeoutUs);
  }

  // One CMD17 with a PIO data phase.
  int readBlockPio(uint64_t lba, uint8_t *out) {
    if (lba > 0xffffffffull) return kProtocol;  // sector addressing is a 32-bit argument
    if (waitState(kPsCmdInhibit | kPsDataInhibit, 0, kCommandTimeoutUs) != kOk) return kBusy;
    ++stats.commands;
    wr32(kRegIntStatus, 0xffffffffu);
    // In host-v4 mode a non-zero 32-bit block count at 0x00 overrides the 16-bit
    // one. The firmware's last DMA transfer may have left a stale value there.
    if (v4_) wr32(kRegDmaAddress, 0);
    wrPair(kRegBlockSize, kSdmaBoundary | kBlockBytes, 1);
    wr8(kRegTimeoutControl, kDataTimeoutExponent);
    wr32(kRegArgument, static_cast<uint32_t>(lba));
    wrPair(kRegTransferMode, kTmBlockCountEnable | kTmRead,
           (17u << 8) | kCmdData | kCmdCrc | kCmdIndex | kCmdRespShort);

    uint64_t deadline = plat_->nowUs() + kDataTimeoutUs;
    uint32_t status;
    for (;;) {
      status = rd32(kRegIntStatus);
      if (status & kIntErrorMask) return failedTransfer(status);
      if ((status & kIntDataAvailable) && (rd32(kRegPresentState) & kPsDataAvailable)) break;
      if (plat_->nowUs() > deadline) return failedTimeout(status);
      plat_->delayUs(1);
    }
    uint32_t cardStatus = rd32(kRegResponse);
    wr32(kRegIntStatus, kIntDataAvailable);
    uint32_t words[kBlockBytes / 4];
    for (unsigned i = 0; i < kBlockBytes / 4; ++i) words[i] = rd32(kRegBuffer);
    for (;;) {
      status = rd32(kRegIntStatus);
      if (status & kIntErrorMask) return failedTransfer(status);
      if (status & kIntDataEnd) break;
      if (plat_->nowUs() > deadline) return failedTimeout(status);
      plat_->delayUs(1);
    }
    wr32(kRegIntStatus, status);
    if (cardStatus & kCardErrorBits) {
      if (logBudget())
        plat_->log("NativeEmmc: CMD17 block %llu: card status 0x%08x carries error bits\n",
                   static_cast<unsigned long long>(lba), cardStatus);
      // The R1 of a command reports the state the card was in when the command
      // arrived, i.e. the aftermath of the previous one. The data arrived and the
      // controller saw no error, so the block is kept; the line above is the clue.
    }
    __builtin_memcpy(out, words, kBlockBytes);
    return kOk;
  }

  // SDHCI "error interrupt recovery": after a command error the CMD line, and
  // after a data error the DAT lines, must be reset before anything else is sent.
  int failedTransfer(uint32_t status) {
    if (logBudget())
      plat_->log("NativeEmmc: transfer error int-status=0x%08x present=0x%08x\n", status, rd32(kRegPresentState));
    wr32(kRegIntStatus, status);
    if (status & 0x000f0000u) reset(kResetCmd);   // bits 19:16 command timeout/CRC/end bit/index
    if (status & 0x03700000u) reset(kResetData);  // bits 22:20 data timeout/CRC/end bit, 25:24 ADMA / auto CMD12 error
    return (status & (1u << 20)) ? kTimeout : kIoError;
  }

  int failedTimeout(uint32_t status) {
    if (logBudget())
      plat_->log("NativeEmmc: transfer timed out int-status=0x%08x present=0x%08x\n", status, rd32(kRegPresentState));
    reset(kResetCmd | kResetData);
    return kTimeout;
  }
};

}  // namespace emmc
