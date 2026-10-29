// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Host-side test of ./RootManifest.h: the header the kernel driver compiles,
// run against manifests laid out byte for byte like the ones `unixnu build userdata` writes
// (scripts/unixnu/userdata.py of the UniXNU tree: struct '<32sQQII32s40s' entries behind
// "SHNGROOT", version, count, image_bytes, with the SHA-256 of the first 4064 bytes at 4064).
//
//   g++ -std=c++17 -Wall -Wextra -Werror -O1 -I . tests/test-root-manifest.cpp -o /tmp/test-root-manifest
//   /tmp/test-root-manifest                  # synthetic manifests
//   /tmp/test-root-manifest userdata-root.img [partition_bytes]
//                                            # also parse the manifest of a real image
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

// Gpt.h too: the driver includes both, and their helpers must not collide.
#include "Gpt.h"
#include "RootManifest.h"

namespace {

int failures = 0;
#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                        \
      ++failures;                                                                        \
    }                                                                                    \
  } while (0)

// ---- a small, independent SHA-256 (FIPS 180-4), checked against known vectors below -----------
struct Sha256 {
  uint32_t h[8];
  uint8_t buffer[64];
  uint64_t length;
  size_t used;
  static uint32_t rotr(uint32_t x, unsigned n) { return (x >> n) | (x << (32 - n)); }
  void init() {
    static const uint32_t start[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                      0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    memcpy(h, start, sizeof h);
    length = 0;
    used = 0;
  }
  void block(const uint8_t *p) {
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t w[64];
    for (unsigned i = 0; i < 16; ++i)
      w[i] = (static_cast<uint32_t>(p[4 * i]) << 24) | (p[4 * i + 1] << 16) | (p[4 * i + 2] << 8) | p[4 * i + 3];
    for (unsigned i = 16; i < 64; ++i) {
      uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (unsigned i = 0; i < 64; ++i) {
      uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      uint32_t ch = (e & f) ^ (~e & g);
      uint32_t t1 = hh + s1 + ch + k[i] + w[i];
      uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      uint32_t t2 = s0 + maj;
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
  void update(const uint8_t *data, size_t n) {
    length += n;
    while (n) {
      size_t take = 64 - used < n ? 64 - used : n;
      memcpy(buffer + used, data, take);
      used += take;
      data += take;
      n -= take;
      if (used == 64) {
        block(buffer);
        used = 0;
      }
    }
  }
  void final(uint8_t out[32]) {
    uint64_t bits = length * 8;
    uint8_t pad = 0x80;
    update(&pad, 1);
    uint8_t zero = 0;
    while (used != 56) update(&zero, 1);
    uint8_t tail[8];
    for (unsigned i = 0; i < 8; ++i) tail[i] = static_cast<uint8_t>(bits >> (56 - 8 * i));
    update(tail, 8);
    for (unsigned i = 0; i < 8; ++i) {
      out[4 * i] = static_cast<uint8_t>(h[i] >> 24);
      out[4 * i + 1] = static_cast<uint8_t>(h[i] >> 16);
      out[4 * i + 2] = static_cast<uint8_t>(h[i] >> 8);
      out[4 * i + 3] = static_cast<uint8_t>(h[i]);
    }
  }
};

void sha256(const uint8_t *data, size_t length, uint8_t out[32]) {
  Sha256 context;
  context.init();
  context.update(data, length);
  context.final(out);
}

std::string hex(const uint8_t *p, size_t n) {
  static const char digits[] = "0123456789abcdef";
  std::string text;
  for (size_t i = 0; i < n; ++i) {
    text += digits[p[i] >> 4];
    text += digits[p[i] & 15];
  }
  return text;
}

// ---- a manifest laid out like userdata.py ----------------------------------------------------
struct Entry {
  std::string name;
  uint64_t offset, size;
  uint32_t blockSize = 512, flags = 1;
};

void put32(uint8_t *p, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
void put64(uint8_t *p, uint64_t v) {
  for (unsigned i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

std::vector<uint8_t> makeManifest(const std::vector<Entry> &entries, uint64_t imageBytes, uint32_t version = 1,
                                  bool fixDigest = true) {
  std::vector<uint8_t> page(4096, 0);
  memcpy(page.data(), "SHNGROOT", 8);
  put32(&page[8], version);
  put32(&page[12], static_cast<uint32_t>(entries.size()));
  put64(&page[16], imageBytes);
  for (size_t i = 0; i < entries.size(); ++i) {
    uint8_t *entry = &page[24 + 128 * i];
    memcpy(entry, entries[i].name.c_str(), entries[i].name.size() < 32 ? entries[i].name.size() : 32);
    put64(entry + 32, entries[i].offset);
    put64(entry + 40, entries[i].size);
    put32(entry + 48, entries[i].blockSize);
    put32(entry + 52, entries[i].flags);
    for (unsigned b = 0; b < 32; ++b) entry[56 + b] = static_cast<uint8_t>(i * 32 + b);  // a recognisable digest
  }
  if (fixDigest) sha256(page.data(), 4064, &page[4064]);
  return page;
}

constexpr uint64_t MiB = 1ull << 20;
// The layout of this board: OS 6077580800 bytes at 1 MiB, SystemOS behind it, each 1 MiB aligned.
const uint64_t kOsSize = 6077580800ull;
const uint64_t kSysSize = 3303817216ull;

std::vector<Entry> boardEntries() {
  uint64_t osOffset = MiB;
  uint64_t sysOffset = osOffset + (kOsSize + MiB - 1) / MiB * MiB;
  return {{"OS", osOffset, kOsSize}, {"SystemOS", sysOffset, kSysSize}};
}
uint64_t boardImageBytes() {
  auto entries = boardEntries();
  return entries[1].offset + (entries[1].size + MiB - 1) / MiB * MiB;
}
const uint64_t kPartition = 55509ull * MiB;  // userdata of this board

int parse(const std::vector<uint8_t> &page, uint64_t partition, emmc::RootManifest &out) {
  return emmc::parseRootManifest(page.data(), partition, sha256, out);
}

void testSha() {
  uint8_t out[32];
  sha256(reinterpret_cast<const uint8_t *>("abc"), 3, out);
  CHECK(hex(out, 32) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  sha256(nullptr, 0, out);
  CHECK(hex(out, 32) == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  std::string million(1000000, 'a');
  sha256(reinterpret_cast<const uint8_t *>(million.data()), million.size(), out);
  CHECK(hex(out, 32) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

void testGoodManifest() {
  emmc::RootManifest manifest;
  auto page = makeManifest(boardEntries(), boardImageBytes());
  CHECK(parse(page, kPartition, manifest) == emmc::kRootOk);
  CHECK(manifest.count == 2);
  CHECK(manifest.imageBytes == boardImageBytes());
  CHECK(strcmp(manifest.window[0].name, "OS") == 0);
  CHECK(manifest.window[0].offset == MiB && manifest.window[0].size == kOsSize);
  CHECK(strcmp(manifest.window[1].name, "SystemOS") == 0);
  CHECK(manifest.window[1].size == kSysSize);
  CHECK(manifest.window[1].sha256[0] == 32 && manifest.window[1].sha256[31] == 63);
  // The image fits the partition with room to spare, and exactly fits a partition of its own size.
  CHECK(parse(page, boardImageBytes(), manifest) == emmc::kRootOk);
  CHECK(parse(page, boardImageBytes() - 4096, manifest) == emmc::kRootBadBounds);
}

void testRejections() {
  emmc::RootManifest manifest;
  // Not a root image: what the partition holds before one is flashed.
  std::vector<uint8_t> hfs(4096, 0);
  hfs[1024] = 'H';
  hfs[1025] = '+';
  CHECK(parse(hfs, kPartition, manifest) == emmc::kRootNoMagic);
  CHECK(parse(std::vector<uint8_t>(4096, 0), kPartition, manifest) == emmc::kRootNoMagic);

  CHECK(parse(makeManifest(boardEntries(), boardImageBytes(), 2), kPartition, manifest) == emmc::kRootBadVersion);
  CHECK(parse(makeManifest({}, boardImageBytes()), kPartition, manifest) == emmc::kRootBadVersion);
  {
    std::vector<Entry> five;
    for (unsigned i = 0; i < 5; ++i) five.push_back({"W" + std::to_string(i), MiB * (1 + i), MiB});
    CHECK(parse(makeManifest(five, 8 * MiB), kPartition, manifest) == emmc::kRootBadVersion);
  }

  // One flipped bit anywhere in the first 4064 bytes breaks the digest; so does a bad digest.
  for (unsigned at : {8u, 16u, 24u, 40u, 100u, 4063u}) {
    auto page = makeManifest(boardEntries(), boardImageBytes());
    page[at] ^= 1;
    int status = parse(page, kPartition, manifest);
    CHECK(status == emmc::kRootBadDigest || status == emmc::kRootBadVersion);
  }
  {
    auto page = makeManifest(boardEntries(), boardImageBytes());
    page[4064] ^= 0x80;
    CHECK(parse(page, kPartition, manifest) == emmc::kRootBadDigest);
    page = makeManifest(boardEntries(), boardImageBytes(), 1, false);
    CHECK(parse(page, kPartition, manifest) == emmc::kRootBadDigest);
  }

  // Bounds.
  {
    auto entries = boardEntries();
    entries[1].size += MiB;  // runs past image_bytes
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadBounds);
    entries = boardEntries();
    entries[0].offset = 0;  // on top of the manifest block
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadBounds);
    entries = boardEntries();
    entries[0].offset += 512;  // not 4 KiB aligned
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadBounds);
    entries = boardEntries();
    entries[0].size = 0;
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadBounds);
    entries = boardEntries();
    entries[0].size += 100;  // not a whole sector
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadBounds);
    CHECK(parse(makeManifest(boardEntries(), boardImageBytes() + 1), kPartition, manifest) == emmc::kRootBadBounds);
    CHECK(parse(makeManifest(boardEntries(), 0), kPartition, manifest) == emmc::kRootBadBounds);
  }

  // Attributes.
  {
    auto entries = boardEntries();
    entries[0].blockSize = 4096;
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadAttributes);
    entries = boardEntries();
    entries[1].flags = 0;  // writable
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadAttributes);
    entries = boardEntries();
    entries[1].flags = 3;
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadAttributes);
  }

  // Names.
  {
    auto entries = boardEntries();
    entries[1].name = "OS";  // repeated
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadName);
    entries = boardEntries();
    entries[0].name = "";
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadName);
    entries = boardEntries();
    entries[0].name = std::string(32, 'x');  // no terminator
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadName);
    entries = boardEntries();
    entries[0].name = std::string("a\x01") + "b";
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootBadName);
    entries = boardEntries();
    entries[0].name = std::string(31, 'x');  // the longest that fits
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootOk);
  }

  // Overlap.
  {
    auto entries = boardEntries();
    entries[1].offset = entries[0].offset + MiB;
    entries[1].size = MiB;
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootOverlap);
    entries = boardEntries();
    entries[1].offset = entries[0].offset;  // same place
    CHECK(parse(makeManifest(entries, boardImageBytes()), kPartition, manifest) == emmc::kRootOverlap);
    // Adjacent windows do not overlap, and a window may end inside its last 4 KiB sector.
    entries = {{"OS", MiB, 8192 + 512}, {"SystemOS", MiB + 12288, 4096}};
    CHECK(parse(makeManifest(entries, 2 * MiB), kPartition, manifest) == emmc::kRootOk);
    entries = {{"OS", MiB, 8192 + 512}, {"SystemOS", MiB + 8192, 4096}};  // the 4 KiB padding of OS reaches into it
    CHECK(parse(makeManifest(entries, 2 * MiB), kPartition, manifest) == emmc::kRootOverlap);
  }
}

void testPlan() {
  emmc::RootManifest manifest;
  emmc::RootPlan plan;
  CHECK(parse(makeManifest(boardEntries(), boardImageBytes()), kPartition, manifest) == emmc::kRootOk);
  // manifest order is OS, SystemOS; the plan is SystemOS first, OS last
  CHECK(emmc::buildRootPlan(manifest, plan));
  CHECK(plan.count == 2 && plan.order[0] == 1 && plan.order[1] == 0);

  std::vector<Entry> three = {{"Extra", MiB, MiB}, {"OS", 2 * MiB, MiB}, {"SystemOS", 3 * MiB, MiB}};
  CHECK(parse(makeManifest(three, 4 * MiB), kPartition, manifest) == emmc::kRootOk);
  CHECK(emmc::buildRootPlan(manifest, plan));
  CHECK(plan.count == 3 && plan.order[0] == 2 && plan.order[1] == 0 && plan.order[2] == 1);

  std::vector<Entry> osOnly = {{"OS", MiB, MiB}};
  CHECK(parse(makeManifest(osOnly, 2 * MiB), kPartition, manifest) == emmc::kRootOk);
  CHECK(emmc::buildRootPlan(manifest, plan));
  CHECK(plan.count == 1 && plan.order[0] == 0);

  std::vector<Entry> noOs = {{"SystemOS", MiB, MiB}, {"Other", 2 * MiB, MiB}};
  CHECK(parse(makeManifest(noOs, 3 * MiB), kPartition, manifest) == emmc::kRootOk);
  CHECK(!emmc::buildRootPlan(manifest, plan));  // no root volume
  CHECK(!emmc::buildRootPlan(emmc::RootManifest{}, plan));
}

// A real image: print what the driver would see.
int checkFile(const char *path, uint64_t partition) {
  FILE *file = fopen(path, "rb");
  if (!file) {
    printf("cannot open %s\n", path);
    return 1;
  }
  std::vector<uint8_t> page(4096);
  size_t got = fread(page.data(), 1, page.size(), file);
  fseek(file, 0, SEEK_END);
  long long total = ftell(file);
  fclose(file);
  if (got != page.size()) {
    printf("%s: short read\n", path);
    return 1;
  }
  if (!partition) partition = static_cast<uint64_t>(total);
  emmc::RootManifest manifest;
  int status = parse(page, partition, manifest);
  printf("%s: %lld bytes, partition %llu bytes: %s\n", path, total, static_cast<unsigned long long>(partition),
         emmc::rootStatusName(status));
  if (status != emmc::kRootOk) return 1;
  printf("  image_bytes %llu, %u windows\n", static_cast<unsigned long long>(manifest.imageBytes), manifest.count);
  for (unsigned i = 0; i < manifest.count; ++i) {
    printf("  %-10s offset %llu (%#llx) size %llu  sha256 %s\n", manifest.window[i].name,
           static_cast<unsigned long long>(manifest.window[i].offset),
           static_cast<unsigned long long>(manifest.window[i].offset),
           static_cast<unsigned long long>(manifest.window[i].size), hex(manifest.window[i].sha256, 32).c_str());
  }
  emmc::RootPlan plan;
  if (emmc::buildRootPlan(manifest, plan)) {
    printf("  publication order:");
    for (unsigned i = 0; i < plan.count; ++i) printf(" %s", manifest.window[plan.order[i]].name);
    printf("\n");
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc >= 2) return checkFile(argv[1], argc >= 3 ? strtoull(argv[2], nullptr, 0) : 0);
  testSha();
  testGoodManifest();
  testRejections();
  testPlan();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("test-root-manifest: ok\n");
  return 0;
}
