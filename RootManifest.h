// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// RootManifest.h -- the root-image manifest of the `userdata` partition, and the order in which
// its windows are published. Pure functions, no kernel dependency: tests/test-root-manifest.cpp
// runs this very header against manifests laid out like the ones `unixnu build userdata` writes.
//
// `unixnu build userdata` (scripts/unixnu/userdata.py of the UniXNU tree) writes the raw image of
// the partition: a manifest block, then the unchanged OS and SystemOS disk images of the IPSW,
// each a "window" aligned to 1 MiB. The manifest (little endian), 4096 bytes:
//
//     0x000  "SHNGROOT", u32 version = 1, u32 count, u64 image_bytes
//     0x018  count x 128 bytes: char name[32], u64 offset, u64 size, u32 block_size = 512,
//            u32 flags (bit 0 = read-only), u8 sha256[32], u8 reserved[40]
//     0xfe0  SHA-256 of bytes 0x000..0xfdf
//
// The rules below are the ones the reference parser of the UniXNU tree applies (RootLayout.h of
// ShengUfs), so an image that one accepts is an image the other accepts. The partition holds
// 512-byte sectors here (the card's own), not the 4096-byte ones of UFS; the byte offsets in the
// manifest do not depend on that.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace emmc {

constexpr uint32_t kRootManifestBytes = 4096;
constexpr uint32_t kRootHeaderBytes = 24;
constexpr uint32_t kRootEntryBytes = 128;
constexpr uint32_t kRootDigestOffset = 4064;  // SHA-256 of the bytes before it
constexpr uint32_t kRootNameBytes = 32;
constexpr uint32_t kRootMaxWindows = 4;
constexpr uint32_t kRootSectorBytes = 512;    // block_size of every window
constexpr uint32_t kRootReadOnly = 1;

struct RootWindow {
  char name[kRootNameBytes];  // printable ASCII, NUL terminated
  uint64_t offset;            // bytes from the start of the manifest (the start of the image)
  uint64_t size;              // bytes
  uint8_t sha256[32];         // of the whole window (not checked at boot: that would read all of it)
};

struct RootManifest {
  uint32_t count;
  uint64_t imageBytes;
  RootWindow window[kRootMaxWindows];
};

enum RootStatus : int {
  kRootOk = 0,
  kRootNoMagic = 1,       // not a root image: what `userdata` holds until one is flashed
  kRootBadVersion = 2,    // version or window count
  kRootBadDigest = 3,     // the manifest does not match its own SHA-256
  kRootBadBounds = 4,     // image_bytes or a window outside the partition / the image
  kRootBadName = 5,       // empty, unprintable, too long, or repeated
  kRootOverlap = 6,
  kRootBadAttributes = 7  // a block size other than 512, or a window that is not read-only
};

inline const char *rootStatusName(int status) {
  switch (status) {
    case kRootOk: return "ok";
    case kRootNoMagic: return "no SHNGROOT magic";
    case kRootBadVersion: return "unsupported version or window count";
    case kRootBadDigest: return "manifest digest mismatch";
    case kRootBadBounds: return "window outside the image or the partition";
    case kRootBadName: return "bad or repeated window name";
    case kRootOverlap: return "overlapping windows";
    case kRootBadAttributes: return "window is not a read-only 512-byte-sector image";
    default: return "unknown";
  }
}

namespace root_detail {
inline uint32_t le32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline uint64_t le64(const uint8_t *p) { return le32(p) | (static_cast<uint64_t>(le32(p + 4)) << 32); }
inline bool sameText(const char *a, const char *b) {
  while (*a && *a == *b) {
    ++a;
    ++b;
  }
  return *a == *b;
}
constexpr uint64_t roundUp(uint64_t value, uint64_t unit) { return (value + unit - 1) / unit * unit; }
}  // namespace root_detail

// `page` is the first kRootManifestBytes bytes of the partition. `partitionBytes` is the size of
// the partition. `sha256(data, length, out32)` computes a SHA-256. On any status other than
// kRootOk `out` holds nothing usable.
template <class Sha256>
int parseRootManifest(const uint8_t *page, uint64_t partitionBytes, Sha256 &&sha256, RootManifest &out) {
  out = RootManifest{};
  static const char magic[8] = {'S', 'H', 'N', 'G', 'R', 'O', 'O', 'T'};
  for (unsigned i = 0; i < 8; ++i)
    if (page[i] != static_cast<uint8_t>(magic[i])) return kRootNoMagic;
  const uint32_t version = root_detail::le32(page + 8), count = root_detail::le32(page + 12);
  const uint64_t imageBytes = root_detail::le64(page + 16);
  if (version != 1 || !count || count > kRootMaxWindows) return kRootBadVersion;

  uint8_t digest[32];
  sha256(page, kRootDigestOffset, digest);
  for (unsigned i = 0; i < 32; ++i)
    if (digest[i] != page[kRootDigestOffset + i]) return kRootBadDigest;

  if (imageBytes < 4096 || (imageBytes & 4095) || imageBytes > partitionBytes) return kRootBadBounds;
  for (uint32_t index = 0; index < count; ++index) {
    const uint8_t *entry = page + kRootHeaderBytes + kRootEntryBytes * index;
    const uint64_t offset = root_detail::le64(entry + 32), size = root_detail::le64(entry + 40);
    // The window starts after the manifest block, lies inside the image, and so does the sector
    // padding a 4 KiB reader would fetch after its last byte.
    if ((offset & 4095) || offset < 4096 || (size & 511) || !size || offset >= imageBytes || size > imageBytes - offset ||
        root_detail::roundUp(size, 4096) > imageBytes - offset) {
      return kRootBadBounds;
    }
    if (root_detail::le32(entry + 48) != kRootSectorBytes || root_detail::le32(entry + 52) != kRootReadOnly) return kRootBadAttributes;
    unsigned length = 0;
    while (length < kRootNameBytes && entry[length] >= 0x20 && entry[length] <= 0x7e) ++length;
    if (!length || length == kRootNameBytes || entry[length] != 0) return kRootBadName;

    RootWindow &window = out.window[index];
    for (unsigned i = 0; i < kRootNameBytes; ++i) window.name[i] = static_cast<char>(entry[i]);
    window.name[kRootNameBytes - 1] = 0;
    window.offset = offset;
    window.size = size;
    for (unsigned i = 0; i < 32; ++i) window.sha256[i] = entry[56 + i];
    for (uint32_t prior = 0; prior < index; ++prior) {
      const RootWindow &other = out.window[prior];
      if (root_detail::sameText(window.name, other.name)) return kRootBadName;
      if (offset < other.offset + root_detail::roundUp(other.size, 4096) && other.offset < offset + root_detail::roundUp(size, 4096)) return kRootOverlap;
    }
  }
  out.count = count;
  out.imageBytes = imageBytes;
  return kRootOk;
}

// The order in which the windows are published. The root volume lives in "OS"; once it is
// published the kernel mounts it and goes on to look for the SystemOS volume, so "SystemOS" goes
// first and "OS" last, with whatever else there is in between in manifest order.
struct RootPlan {
  unsigned count;
  unsigned order[kRootMaxWindows];
};

inline bool buildRootPlan(const RootManifest &manifest, RootPlan &plan) {
  plan = RootPlan{};
  if (!manifest.count || manifest.count > kRootMaxWindows) return false;
  unsigned system = manifest.count, os = manifest.count;
  for (unsigned i = 0; i < manifest.count; ++i) {
    if (root_detail::sameText(manifest.window[i].name, "SystemOS")) system = i;
    if (root_detail::sameText(manifest.window[i].name, "OS")) os = i;
  }
  if (os == manifest.count) return false;  // no root volume to publish
  unsigned position = 0;
  if (system != manifest.count) plan.order[position++] = system;
  for (unsigned i = 0; i < manifest.count; ++i)
    if (i != system && i != os) plan.order[position++] = i;
  plan.order[position++] = os;
  plan.count = position;
  return true;
}

}  // namespace emmc
