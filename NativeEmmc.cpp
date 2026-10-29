// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// NativeEmmc: the eMMC host of the Rockchip RK3588 EVB10 as an IOKit service, and the block
// devices it publishes.
//
// NativeEmmc attaches to the /arm-io node that dts/evb10.dts of UniXNU-evb10 publishes for the
// DesignWare MSHC at 0xFE2E0000 (compatible "rockchip,rk3588-dwcmshc"), keeps the bus state
// the firmware left, and reads the card through the core in EmmcHost.h. The card is read-only
// except for one window the owner asks for (emmc-data-persist=1, below): the core refuses to
// write a block outside the range it was given, and the range is never anything but that window.
//
// NativeEmmcBlock is a window of the card (the whole disk, or one partition) as a subclass of
// the original IOBlockStorageDevice: 512-byte sectors, read-only (except the Data window),
// synchronous. The original
// IOBlockStorageDriver, the partition schemes and APFS sit on top unchanged. How the subclass
// is bound to the original class is explained in scripts/bind-emmc-storage.py of UniXNU-evb10.
//
// Boot-args:  emmc=0           do not attach
//             emmc=1           probe only (default): dump the controller, read the GPT, check
//                              that reads are stable, publish nothing
//             emmc=2           also publish the whole disk as one read-only block device
//             emmc=3           also publish the partition named "userdata" as one device
//             emmc=4           root image: read the SHNGROOT manifest at the start of `userdata`
//                              (RootManifest.h), and publish its windows, "SystemOS" first and
//                              "OS" last, each as one read-only device of its own. The volume
//                              inside "OS" is what /chosen/root-matching selects as the root.
//             emmc-probe=N     blocks of `userdata` to double-read (default 1024, 0 = none)
//             emmc-multi=0     keep one CMD17 per block (default 1: after the probe has read the same
//                              blocks both ways and the CRCs agree, runs of blocks go out as one
//                              CMD18, see probeUserdata)
//             emmc-dma=0       keep the buffer port (default 1: after the multi-block probe, the same
//                              blocks are read through ADMA2 into a bounce window below 4 GiB, and only
//                              a read that matches the PIO one switches DMA on)
//             emmc-boot-manifest=0  do not publish /chosen/boot-manifest-hash (default 1, see
//                              publishBootManifestHash): the original `mount -P 1` exits 66 without it
//             emmc-root-lba=N  card block of the manifest (default: first block of `userdata`)
//             emmc-root-trust=1  let the kernel accept the root volume without Apple's root
//                              authentication, if it is a read-only volume inside the "OS" window
//                              this driver published (native_emmc_root_trusted, called from
//                              bsd_init by UniXNU-evb10 patches/xnu/evb10-root-trust.patch). Without it the
//                              kernel panics with "rootvp not authenticated after mounting",
//                              because the OS volume of an IPSW is not sealed.
//             wdt-kick=N       keep the DW watchdog at 0xFEAF0000 fed for N seconds. The firmware
//                              arms it (about 89 s) and stops feeding it at ExitBootServices, and
//                              nothing in the kernel does, so without this every run ends there.
//                              The feeding stops after N seconds on purpose: a hung board still
//                              resets by itself. Default 0 = do not feed.
//             pmgr-nub=0       do not publish the ApplePMGRNub that the original AMFI waits for
//                              (default 1, see pmgrNubPublisher). Independent of the card.
//             emmc-data-size=N make the Data RAM disk N MiB (default: as big as the payload's seed). The
//                              seed on the payload only needs to carry the part of the APFS container that is
//                              not zero (TRIM=1 of the data seed tool: 2 MiB of a 512 MiB container); the
//                              rest is zero-filled kernel memory made here (growDataDisk). 32 MiB filled up
//                              within 15 s of the userland starting (a board run: ENOSPC).
//             emmc-data-ram=0  do not publish the Data RAM disk (default 1, see dataDiskPublisher):
//                              the payload's /chosen/memory-map/RAMDisk as a WRITABLE block device
//                              of its own, in front of which Apple's APFS publishes the Data volume
//                              as /dev/diskNsM. Independent of the card.
//             emmc-data-persist=1  keep the Data volume on the card instead of in RAM (default 0; needs emmc=4, DMA, and
//                              the payload's seed, see persistentData). The volume is a window at the END of `userdata`,
//                              a 1 MiB header and then the APFS container the seed describes (its size comes from the
//                              seed's own superblock, not from emmc-data-size), clear of the root image by 64 MiB. The
//                              first boot formats it (zeros, the seed, a read-back, the header's "ready"), later boots
//                              find the header and publish the window as it is; the kernel then copies the OS volume's
//                              /private/var into it only once (UniXNU-evb10 patches/xnu/evb10-userland-mounts.patch). This is the
//                              only place the driver writes the card. If anything about it is wrong the Data RAM disk
//                              is published instead, as without the argument.
//             emmc-data-reset=1    format that window again, whatever it holds (default 0)
//
// The kernel reaches this driver through five C symbols. Three of them refuse unless
// emmc-root-trust=1 and are about read-only volumes inside a window published here:
//   native_emmc_root_trusted(dev)     the root volume is the one of the "OS" window
//   native_emmc_volume_device(uuid, &dev, path, size)
//                                     the BSD device (and its /dev path) of the volume with this
//                                     UUID inside the "SystemOS" window, for the cryptex mount
//   native_emmc_cache_trusted(dev)    that device is the "SystemOS" window's volume, still read-only
// the fourth is about the writable one:
//   native_emmc_data_device(uuid, &dev, path, size)
//                                     the BSD device of the volume with this UUID inside the Data
//                                     RAM disk, for the kernel's mount of it on /private/var. Why not
//                                     /dev/md0: Apple's APFS takes the volume index from the device
//                                     NAME ("diskNsM" is volume M-1, anything else is -1 and
//                                     nx_is_mount_valid refuses it with ENOENT).
// (UniXNU-evb10 patches/xnu/evb10-root-trust.patch and evb10-userland-mounts.patch).
#include <IOKit/IOBSD.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IORegistryEntry.h>
#include <IOKit/IOService.h>
#include <IOKit/storage/IOBlockStorageDevice.h>
#include <kern/clock.h>
#include <kern/task.h>
#include <kern/thread.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSIterator.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSSymbol.h>
#include <libkern/crypto/sha2.h>
#include <pexpert/pexpert.h>

#include "EmmcHost.h"
#include "Gpt.h"
#include "RootManifest.h"

// xnu's own device-memory mapper (osfmk/arm/machine_routines.h), as the platform code uses it
// for the GIC and the UART. Declared here because the driver does not include that header.
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" kern_return_t OSKextLoadKextWithIdentifier(const char *);
// Boot memory (the RAM disk of the payload) is reached through the kernel's static mapping, like
// the original code does; gPhysBase/gPhysSize bound the physical addresses the loader may hand over.
extern "C" vm_offset_t ml_static_ptovirt(vm_offset_t paddr);
extern "C" unsigned long gPhysBase, gPhysSize;

// The sizes of the original classes (scripts/data/evb10-storage-interface-audit.json of UniXNU-evb10). The public
// headers this file compiles against must describe the same objects as the original kernelcache,
// or a subclass would overlay its members on the base class's.
static_assert(sizeof(IOService) == 0x88, "IOService layout differs from the original kernelcache");
static_assert(sizeof(IOBlockStorageDevice) == 0x90, "IOBlockStorageDevice layout differs from the original kernelcache");

extern "C" {
volatile uint32_t native_emmc_ready = 0;          // 1 once the card answered and the GPT parsed
volatile uint64_t native_emmc_userdata_first = 0;
volatile uint64_t native_emmc_userdata_last = 0;
volatile uint32_t native_emmc_blocks_published = 0;
volatile uint64_t native_emmc_block_requests = 0;
volatile uint64_t native_emmc_block_failures = 0;
volatile uint64_t native_emmc_read_calls = 0;     // host reads (one per bounce-buffer chunk, 64 KiB at most)
volatile uint64_t native_emmc_read_blocks = 0;    // blocks the card delivered
volatile uint64_t native_emmc_read_us = 0;        // time spent inside those reads
volatile uint32_t native_emmc_multi_block = 0;    // 1 once reads are runs of blocks (CMD18)
volatile uint32_t native_emmc_dma = 0;            // 1 once those runs arrive by DMA
volatile uint64_t native_emmc_dma_blocks = 0;     // blocks that did
volatile int32_t native_emmc_root_status = -1;    // emmc::RootStatus of the manifest, -1 = not looked at
volatile uint32_t native_emmc_root_windows = 0;   // root windows started by the publisher
volatile uint32_t native_emmc_data_ready = 0;     // 1 once the Data RAM disk is published
volatile uint64_t native_emmc_ram_requests = 0;   // block requests served from the Data RAM disk
volatile uint64_t native_emmc_ram_writes = 0;     // ... of which writes
volatile uint64_t native_emmc_ram_failures = 0;
volatile uint32_t native_emmc_data_persistent = 0; // 1 once the Data volume is a window of the card (not RAM)
volatile uint32_t native_emmc_data_seeded = 0;     // ... and it was already formatted by an earlier boot
volatile uint64_t native_emmc_write_calls = 0;     // host writes (one per bounce-buffer chunk, 64 KiB at most)
volatile uint64_t native_emmc_write_blocks = 0;    // blocks the card acknowledged
volatile uint64_t native_emmc_write_us = 0;
volatile uint64_t native_emmc_write_failures = 0;  // write calls that failed
volatile uint64_t native_emmc_flush_calls = 0;
}

namespace {

constexpr uint32_t kChunkBlocks = 128;                        // one bounce buffer: 64 KiB
constexpr uint32_t kChunkBytes = kChunkBlocks * emmc::kBlockBytes;
constexpr unsigned kRequestLogLines = 48;                     // first block requests that are logged

uint32_t mmioRead(void *base, uint32_t offset, unsigned width) {
  volatile uint8_t *address = static_cast<volatile uint8_t *>(base) + offset;
  switch (width) {
    case 8: return *address;
    case 16: return *reinterpret_cast<volatile uint16_t *>(address);
    default: return *reinterpret_cast<volatile uint32_t *>(address);
  }
}

void mmioWrite(void *base, uint32_t offset, unsigned width, uint32_t value) {
  volatile uint8_t *address = static_cast<volatile uint8_t *>(base) + offset;
  switch (width) {
    case 8: *address = static_cast<uint8_t>(value); break;
    case 16: *reinterpret_cast<volatile uint16_t *>(address) = static_cast<uint16_t>(value); break;
    default: *reinterpret_cast<volatile uint32_t *>(address) = value; break;
  }
  __asm__ volatile("dsb sy" ::: "memory");
}

uint64_t nowUs() {
  uint64_t nanoseconds = 0;
  absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
  return nanoseconds / 1000;
}

// Short waits spin. IODelay() puts the calling thread to sleep on a timer whenever the platform does
// not report a CPU idle latency, and this board's does not: every 1 us poll of the card became a
// context switch and a timer interrupt (72,000 of each per second under a read load, a card read at
// 4 MB/s while the CPU idled). A poll that long is a register read and a spin; only a real wait sleeps.
void delayUs(uint32_t microseconds) {
  if (microseconds >= 1000) {
    IOSleep((microseconds + 999) / 1000);
    return;
  }
  const uint64_t until = nowUs() + microseconds;
  while (nowUs() < until) {
  }
}

const emmc::Platform kPlatform = {mmioRead, mmioWrite, nowUs, delayUs, IOLog};

// The controller does not look into the CPU's caches (the RK3588's eMMC DMA is not cache-coherent and this
// kernel does no maintenance of its own for it): before a run the window's lines are cleaned out to memory and
// dropped, so no dirty line is written over the data later; afterwards they are dropped again, so the CPU reads what
// the controller wrote and not a line it fetched on the way. Clean+invalidate (dc civac) does both, and is harmless
// on a line that is not dirty. The window is only ever written by the CPU before the run starts.
void dmaFlush(void *virt, size_t bytes) {
  uint64_t ctr;
  __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
  const uintptr_t line = static_cast<uintptr_t>(4) << ((ctr >> 16) & 15);
  const uintptr_t end = reinterpret_cast<uintptr_t>(virt) + bytes;
  __asm__ volatile("dsb sy" ::: "memory");
  for (uintptr_t address = reinterpret_cast<uintptr_t>(virt) & ~(line - 1); address < end; address += line)
    __asm__ volatile("dc civac, %0" ::"r"(address) : "memory");
  __asm__ volatile("dsb sy" ::: "memory");
}

emmc::DmaWindow gDmaWindow = {};
IOBufferMemoryDescriptor *gDmaMemory = nullptr;  // for the life of the system

// One physically contiguous, cacheable run of memory below 4 GiB: 64 KiB for the blocks of one command and
// a page behind it for the ADMA2 descriptors. IOKit places it by the mask (bits that may be set in the
// address: below 4 GiB, 64 KiB aligned).
const emmc::DmaWindow *dmaWindow() {
  if (gDmaWindow.data) return &gDmaWindow;
  const size_t bytes = emmc::kMaxRunBytes + 4096;
  IOBufferMemoryDescriptor *memory = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
      kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) | kIOMemoryPhysicallyContiguous,
      bytes, 0x00000000ffff0000ULL);
  if (!memory) {
    IOLog("NativeEmmc: dma: no contiguous memory below 4 GiB for the bounce window\n");
    return nullptr;
  }
  if (memory->prepare() != kIOReturnSuccess) {
    IOLog("NativeEmmc: dma: the bounce window could not be wired\n");
    memory->release();
    return nullptr;
  }
  IOByteCount length = 0;
  const uint64_t bus = memory->getPhysicalSegment(0, &length, kIOMemoryMapperNone);
  uint8_t *virt = static_cast<uint8_t *>(memory->getBytesNoCopy());
  if (!virt || !bus || length < bytes || bus + bytes > 0x100000000ull || (bus & 0xffff)) {
    IOLog("NativeEmmc: dma: window at bus %#llx, %llu contiguous bytes of %zu: not usable\n", static_cast<unsigned long long>(bus),
          static_cast<unsigned long long>(length), bytes);
    (void)memory->complete();
    memory->release();
    return nullptr;
  }
  bzero(virt, bytes);
  dmaFlush(virt, bytes);
  gDmaMemory = memory;
  gDmaWindow.data = virt;
  gDmaWindow.dataBus = static_cast<uint32_t>(bus);
  gDmaWindow.table = virt + emmc::kMaxRunBytes;
  gDmaWindow.tableBus = static_cast<uint32_t>(bus + emmc::kMaxRunBytes);
  gDmaWindow.flush = dmaFlush;
  IOLog("NativeEmmc: dma: bounce window at bus %#llx, kernel %p, %zu bytes (data %u, descriptors behind it)\n",
        static_cast<unsigned long long>(bus), static_cast<void *>(virt), bytes, emmc::kMaxRunBytes);
  return &gDmaWindow;
}

uint32_t bootArg(const char *name, uint32_t fallback) {
  uint32_t value = fallback;
  if (!PE_parse_boot_argn(name, &value, sizeof(value))) return fallback;
  return value;
}

uint64_t bootArg64(const char *name, uint64_t fallback) {
  uint64_t value = fallback;
  if (!PE_parse_boot_argn(name, &value, sizeof(value))) return fallback;
  return value;
}

void sha256(const uint8_t *data, size_t length, uint8_t out[32]) {
  SHA256_CTX context;
  SHA256_Init(&context);
  SHA256_Update(&context, data, length);
  SHA256_Final(out, &context);
}

// Hex of the first bytes of a block, for the log.
void hexLine(const char *label, uint64_t lba, const uint8_t *bytes, unsigned count) {
  static const char digits[] = "0123456789abcdef";
  char text[3 * 32 + 1];
  unsigned at = 0;
  for (unsigned i = 0; i < count && i < 32; ++i) {
    text[at++] = digits[bytes[i] >> 4];
    text[at++] = digits[bytes[i] & 15];
    text[at++] = ' ';
  }
  text[at] = 0;
  IOLog("NativeEmmc: %s lba %llu: %s\n", label, static_cast<unsigned long long>(lba), text);
}

unsigned long long ull(uint64_t value) { return static_cast<unsigned long long>(value); }

}  // namespace

class NativeEmmcBlock;

class NativeEmmc : public IOService {
  OSDeclareDefaultStructors(NativeEmmc)

  IOMemoryMap *mapping = nullptr;
  IOLock *gate = nullptr;
  emmc::Host host;
  uint64_t diskBlocks = 0;
  uint64_t userdataFirst = 0, userdataLast = 0;
  // The root image (emmc=4): where its manifest is, what it says, and the publication order.
  uint64_t rootLba = 0;
  emmc::RootManifest root = {};
  emmc::RootPlan rootPlan = {};

  bool rootReady = false;             // findRoot() found a usable image
  bool scanGpt();
  void probeUserdata();
  bool findRoot();
  // The Data volume on the card (emmc-data-persist=1), see persistentData(). Runs on a thread of its own.
  bool persistentData();
  bool formatData(uint64_t windowFirst, uint64_t dataBlocks, const uint8_t *seed, uint64_t seedBytes, uint32_t seedCrc, uint32_t generation);
  static void dataPublisher(void *self, wait_result_t);
  // `role` is what the kernel will later ask about the window: 0 none, 1 the "OS" window (the root
  // volume, native_emmc_root_trusted), 2 the "SystemOS" window (the cryptex volume,
  // native_emmc_volume_device and native_emmc_cache_trusted).
  NativeEmmcBlock *publishWindow(const char *label, uint64_t first, uint64_t blocks, unsigned role = 0, bool writable = false);
  void publishRoot();
  static void rootPublisher(void *self, wait_result_t);

 public:
  bool start(IOService *provider) override;
  void stop(IOService *provider) override;
  void free() override;

  // Serialised read of whole blocks; the block devices call this.
  IOReturn readBlocks(uint64_t lba, uint32_t count, void *out) {
    if (!gate) return kIOReturnNotReady;
    IOLockLock(gate);
    int result = host.readBlocks(lba, count, out);
    native_emmc_read_calls = native_emmc_read_calls + 1;
    native_emmc_read_blocks = host.stats.blocks;
    native_emmc_read_us = host.stats.readUs;
    native_emmc_dma_blocks = host.stats.dmaBlocks;
    IOLockUnlock(gate);
    if (result == emmc::kOk) return kIOReturnSuccess;
    return result == emmc::kTimeout ? kIOReturnTimeout : kIOReturnIOError;
  }

  // Serialised write of whole blocks, inside the range the card was opened for (enableWrites) and nowhere else:
  // the host core refuses the rest. Returns once the card reports the blocks programmed.
  IOReturn writeBlocks(uint64_t lba, uint32_t count, const void *in) {
    if (!gate) return kIOReturnNotReady;
    IOLockLock(gate);
    const int result = host.writeBlocks(lba, count, in);
    native_emmc_write_calls = native_emmc_write_calls + 1;
    native_emmc_write_blocks = host.stats.writeBlocks;
    native_emmc_write_us = host.stats.writeUs;
    if (result != emmc::kOk) native_emmc_write_failures = native_emmc_write_failures + 1;
    IOLockUnlock(gate);
    if (result == emmc::kOk) return kIOReturnSuccess;
    return result == emmc::kTimeout ? kIOReturnTimeout : kIOReturnIOError;
  }

  // What the card has in its cache goes to the flash (a no-op on a card without one).
  IOReturn flush() {
    if (!gate) return kIOReturnNotReady;
    IOLockLock(gate);
    const int result = host.flushCache();
    native_emmc_flush_calls = native_emmc_flush_calls + 1;
    IOLockUnlock(gate);
    if (result == emmc::kOk) return kIOReturnSuccess;
    return result == emmc::kTimeout ? kIOReturnTimeout : kIOReturnIOError;
  }

  // The one place the driver lets itself write: [first, last]. DMA has to be on.
  bool enableWrites(uint64_t first, uint64_t last) {
    if (!gate) return false;
    IOLockLock(gate);
    host.setWriteRange(first, last);
    const bool ok = host.writable();
    if (!ok) host.clearWriteRange();
    IOLockUnlock(gate);
    return ok;
  }
  void disableWrites() {
    if (!gate) return;
    IOLockLock(gate);
    host.clearWriteRange();
    IOLockUnlock(gate);
  }
};

OSDefineMetaClassAndStructors(NativeEmmc, IOService)

// One window of the card as a block device, read-only unless it is the Data window (configure with writable).
// Sectors are 512 bytes, as the card's own GPT and the disk images of the root volume expect.
//
// The same class also serves the Data RAM disk (configureRam):
// a part of boot memory the payload hands over, read and written by plain copies. Same bindings to
// the original IOBlockStorageDevice, no second subclass to audit.
class NativeEmmcBlock : public IOBlockStorageDevice {
  OSDeclareDefaultStructors(NativeEmmcBlock)

  NativeEmmc *owner = nullptr;      // our provider; it outlives us
  uint64_t first = 0;               // first card block of the window
  uint64_t blocks = 0;              // length of the window in blocks
  char label[32] = {};
  uint8_t *bounce = nullptr;
  IOLock *lock = nullptr;           // the bounce buffer is shared by all requests of this window
  uint8_t *ram = nullptr;           // RAM disk mode: the memory the blocks are, never freed (boot memory)
  bool online = false;
  bool writable = false;            // card window the host core may write (the Data window of emmc-data-persist=1)

 public:
  bool configure(NativeEmmc *controller, const char *name, uint64_t firstBlock, uint64_t blockCount, bool canWrite = false);
  bool configureRam(const char *name, uint8_t *memory, uint64_t bytes);
  const char *windowLabel() const { return label; }
  bool start(IOService *provider) override;
  void free() override;
  IOReturn doEjectMedia() override { return kIOReturnUnsupported; }
  IOReturn doFormatMedia(UInt64) override { return kIOReturnNotWritable; }
  UInt32 doGetFormatCapacities(UInt64 *, UInt32) const override { return 0; }
  char *getVendorString() override { return const_cast<char *>("UniXNU"); }
  char *getProductString() override { return label; }
  char *getRevisionString() override { return const_cast<char *>("1.0"); }
  char *getAdditionalDeviceInfoString() override {
    return const_cast<char *>(ram ? "RAM disk, volatile" : writable ? "RK3588 eMMC, ADMA2 writes" : "RK3588 eMMC, read-only");
  }
  IOReturn reportBlockSize(UInt64 *value) override { *value = emmc::kBlockBytes; return kIOReturnSuccess; }
  IOReturn reportEjectability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportMaxValidBlock(UInt64 *value) override { *value = blocks - 1; return kIOReturnSuccess; }
  IOReturn reportMediaState(bool *present, bool *changed) override {
    *present = online;
    if (changed) *changed = false;
    return kIOReturnSuccess;
  }
  IOReturn reportRemovability(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn reportWriteProtection(bool *value) override { *value = !(ram || writable); return kIOReturnSuccess; }
  IOReturn doSynchronizeCache() override {
    if (!online) return kIOReturnNotReady;
    return writable && owner ? owner->flush() : kIOReturnSuccess;
  }
  IOReturn doSynchronize(UInt64, UInt64, IOOptionBits) override { return doSynchronizeCache(); }
  IOReturn getWriteCacheState(bool *value) override { *value = false; return kIOReturnSuccess; }
  IOReturn setWriteCacheState(bool) override { return kIOReturnUnsupported; }
  IOReturn doAsyncReadWrite(IOMemoryDescriptor *, UInt64, UInt64, IOStorageAttributes *, IOStorageCompletion *) override;
};

OSDefineMetaClassAndStructors(NativeEmmcBlock, IOBlockStorageDevice)

bool NativeEmmcBlock::configure(NativeEmmc *controller, const char *name, uint64_t firstBlock, uint64_t blockCount, bool canWrite) {
  if (!controller || !name || !blockCount || firstBlock > UINT64_MAX - blockCount ||
      blockCount > UINT64_MAX / emmc::kBlockBytes) return false;
  owner = controller;
  first = firstBlock;
  blocks = blockCount;
  writable = canWrite;
  strlcpy(label, name, sizeof(label));
  bounce = static_cast<uint8_t *>(IOMallocData(kChunkBytes));
  lock = IOLockAlloc();
  return bounce && lock;
}

// The Data RAM disk: `memory` is the kernel's static mapping of `bytes` of boot memory (a whole
// number of pages, checked by the caller); the blocks of the device are those bytes.
bool NativeEmmcBlock::configureRam(const char *name, uint8_t *memory, uint64_t bytes) {
  if (!name || !memory || !bytes || (bytes % emmc::kBlockBytes)) return false;
  owner = nullptr;
  first = 0;
  blocks = bytes / emmc::kBlockBytes;
  strlcpy(label, name, sizeof(label));
  ram = memory;
  return true;
}

bool NativeEmmcBlock::start(IOService *provider) {
  if (!(ram || (bounce && lock)) || !IOBlockStorageDevice::start(provider)) return false;
  online = true;
  registerService();
  return true;
}

IOReturn NativeEmmcBlock::doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 count,
                                           IOStorageAttributes *attributes, IOStorageCompletion *completion) {
  if (!buffer || !completion || !completion->action || !count || block >= blocks || count > blocks - block ||
      count > UINT64_MAX / emmc::kBlockBytes || buffer->getLength() < count * emmc::kBlockBytes) {
    return kIOReturnBadArgument;
  }
  if (ram) {
    // kIODirectionIn: the data goes into the buffer (a read of the device); kIODirectionOut: it comes out of it.
    const IODirection direction = buffer->getDirection();
    if (direction != kIODirectionIn && direction != kIODirectionOut) return kIOReturnBadArgument;
    if (!online) return kIOReturnNotReady;
    const uint64_t bytes = count * emmc::kBlockBytes;
    const bool write = direction == kIODirectionOut;
    uint8_t *at = ram + block * emmc::kBlockBytes;
    uint64_t done = write ? buffer->readBytes(0, at, bytes) : buffer->writeBytes(0, at, bytes);
    IOReturn result = done == bytes ? kIOReturnSuccess : kIOReturnIOError;
    uint64_t serial = __atomic_fetch_add(&native_emmc_ram_requests, 1, __ATOMIC_RELAXED);
    if (write) __atomic_fetch_add(&native_emmc_ram_writes, 1, __ATOMIC_RELAXED);
    if (result != kIOReturnSuccess) __atomic_fetch_add(&native_emmc_ram_failures, 1, __ATOMIC_RELAXED);
    if (serial < 16 || result != kIOReturnSuccess) {
      IOLog("NativeEmmcBlock[%s]: ram %s #%llu lba %llu count %llu: %llu bytes, 0x%x\n", label, write ? "write" : "read", ull(serial),
            ull(block), ull(count), ull(done), static_cast<unsigned>(result));
    }
    completion->action(completion->target, completion->parameter, result, done);
    return kIOReturnSuccess;
  }
  const IODirection cardDirection = buffer->getDirection();
  const bool write = cardDirection == kIODirectionOut;
  if (cardDirection != kIODirectionIn && !(writable && write)) return kIOReturnNotWritable;
  if (!online) return kIOReturnNotReady;

  uint64_t serial = __atomic_fetch_add(&native_emmc_block_requests, 1, __ATOMIC_RELAXED);
  if (serial < kRequestLogLines) {
    IOLog("NativeEmmcBlock[%s]: %s #%llu lba %llu count %llu\n", label, write ? "write" : "read", ull(serial), ull(block), ull(count));
  }
  const uint64_t total = count * emmc::kBlockBytes;
  uint64_t done = 0;
  IOReturn result = kIOReturnSuccess;
  IOLockLock(lock);
  while (done < total) {
    uint32_t chunk = static_cast<uint32_t>(total - done > kChunkBytes ? kChunkBlocks : (total - done) / emmc::kBlockBytes);
    const uint64_t at = first + block + done / emmc::kBlockBytes;
    if (write) {
      if (buffer->readBytes(done, bounce, chunk * emmc::kBlockBytes) != chunk * emmc::kBlockBytes) {
        result = kIOReturnIOError;
        break;
      }
      result = owner->writeBlocks(at, chunk, bounce);
      if (result != kIOReturnSuccess) break;
    } else {
      result = owner->readBlocks(at, chunk, bounce);
      if (result != kIOReturnSuccess) break;
      if (buffer->writeBytes(done, bounce, chunk * emmc::kBlockBytes) != chunk * emmc::kBlockBytes) {
        result = kIOReturnIOError;
        break;
      }
    }
    done += chunk * emmc::kBlockBytes;
  }
  // Force unit access: the data has to be in the flash, not only in the card's cache, before the request completes.
  if (write && result == kIOReturnSuccess && attributes && (attributes->options & kIOStorageOptionForceUnitAccess)) {
    result = owner->flush();
  }
  IOLockUnlock(lock);
  if (result != kIOReturnSuccess) {
    uint64_t failures = __atomic_fetch_add(&native_emmc_block_failures, 1, __ATOMIC_RELAXED);
    if (failures < 16) {
      IOLog("NativeEmmcBlock[%s]: %s lba %llu count %llu failed after %llu bytes: 0x%x\n", label, write ? "write" : "read", ull(block),
            ull(count), ull(done), static_cast<unsigned>(result));
    }
  }
  completion->action(completion->target, completion->parameter, result, done);
  return kIOReturnSuccess;
}

void NativeEmmcBlock::free() {
  if (bounce) { IOFreeData(bounce, kChunkBytes); bounce = nullptr; }
  if (lock) { IOLockFree(lock); lock = nullptr; }
  IOBlockStorageDevice::free();
}

namespace {

// One IOMedia as the log shows it. The volumes of a filesystem add their own properties (the
// APFS ones carry FullName, Status and Sealed), shown when present.
void logMedia(const char *prefix, IOService *media) {
  auto text = [media](const char *key) -> const char * {
    OSString *value = OSDynamicCast(OSString, media->getProperty(key));
    return value ? value->getCStringNoCopy() : "-";
  };
  OSNumber *size = OSDynamicCast(OSNumber, media->getProperty("Size"));
  OSBoolean *whole = OSDynamicCast(OSBoolean, media->getProperty("Whole"));
  OSBoolean *writable = OSDynamicCast(OSBoolean, media->getProperty("Writable"));
  IOLog("%s: media %s: content %s, %llu bytes, %s%s uuid %s\n", prefix, text(kIOBSDNameKey), text("Content"),
        size ? ull(size->unsigned64BitValue()) : 0ull, whole && whole->isTrue() ? "whole" : "slice",
        writable && writable->isTrue() ? ", writable" : ", read-only", text("UUID"));
  if (media->getProperty("FullName") || media->getProperty("Sealed") || media->getProperty("Status")) {
    IOLog("%s:   volume \"%s\" status %s sealed %s\n", prefix, text("FullName"), text("Status"), text("Sealed"));
  }
}

// The IOMedia below `window` in the service plane: the one the block driver publishes for it,
// the partitions the partition scheme makes of that, and the volumes the filesystem makes of
// those. With `log` set each one goes to the log.
unsigned mediaBelow(IORegistryEntry *window, bool log) {
  IORegistryIterator *iterator = IORegistryIterator::iterateOver(window, gIOServicePlane, kIORegistryIterateRecursively);
  if (!iterator) return 0;
  unsigned count = 0;
  for (unsigned visited = 0; visited < 512; ++visited) {  // every entry below the window, media or not
    IORegistryEntry *entry = iterator->getNextObject();
    if (!entry) break;
    IOService *media = entry->metaCast("IOMedia") ? OSDynamicCast(IOService, entry) : nullptr;
    if (!media) continue;
    ++count;
    if (log) logMedia("NativeEmmc", media);
  }
  iterator->release();
  return count;
}

// After a window is registered the partition scheme and the filesystem publish their media on
// threads of their own. Wait until the window is quiet and the number of IOMedia below it has
// stopped changing, for at most `seconds`. Returns that number.
unsigned settleMedia(IOService *window, unsigned seconds) {
  window->waitQuiet(static_cast<uint64_t>(seconds) * 1000000000ull);
  unsigned previous = ~0u, stable = 0, count = 0;
  for (unsigned tick = 0; tick < seconds * 10 && stable < 15; ++tick) {
    count = mediaBelow(window, false);
    stable = count && count == previous ? stable + 1 : 0;
    previous = count;
    IOSleep(100);
  }
  return count;
}

// What IOStorageFamily made of the windows: every IOMedia in the registry, once, after the
// count stops growing. Observation only.
void mediaReport(void *, wait_result_t) {
  unsigned previous = ~0u, stable = 0, count = 0;
  for (unsigned tick = 0; tick < 200 && stable < 15; ++tick) {
    OSDictionary *matching = IOService::serviceMatching("IOMedia");
    OSIterator *iterator = matching ? IOService::getMatchingServices(matching) : nullptr;
    count = 0;
    while (iterator && iterator->getNextObject()) ++count;
    if (iterator) iterator->release();
    if (matching) matching->release();
    stable = count && count == previous ? stable + 1 : 0;
    previous = count;
    IOSleep(100);
  }
  IOLog("NativeEmmc: media report: %u IOMedia, %llu block requests, %llu failures\n", count,
        ull(native_emmc_block_requests), ull(native_emmc_block_failures));
  OSDictionary *matching = IOService::serviceMatching("IOMedia");
  OSIterator *iterator = matching ? IOService::getMatchingServices(matching) : nullptr;
  for (unsigned shown = 0; iterator && shown < 64; ++shown) {
    IOService *media = OSDynamicCast(IOService, iterator->getNextObject());
    if (!media) break;
    logMedia("NativeEmmc", media);
  }
  if (iterator) iterator->release();
  if (matching) matching->release();
}

// The "OS" window of the root image, set before that window is started so that the root volume
// cannot be mounted before the trust decision can find it. Never cleared once the window runs.
NativeEmmcBlock *gOsWindow = nullptr;
// The "SystemOS" window (the cryptex volume with the dyld shared cache), set the same way.
NativeEmmcBlock *gSystemOsWindow = nullptr;
// The Data RAM disk (the one writable device), set the same way.
NativeEmmcBlock *gDataWindow = nullptr;

// The IOMedia below `window` whose BSD device number is major,minor (the number the kernel got
// from IOFindBSDRoot). Gives its BSD name, and whether it can be written (unknown counts as yes).
bool mediaByDevice(IORegistryEntry *window, uint32_t major, uint32_t minor, char (&name)[32], bool &writable) {
  IORegistryIterator *iterator = IORegistryIterator::iterateOver(window, gIOServicePlane, kIORegistryIterateRecursively);
  if (!iterator) return false;
  bool found = false;
  for (unsigned visited = 0; visited < 512 && !found; ++visited) {
    IORegistryEntry *entry = iterator->getNextObject();
    if (!entry) break;
    if (!entry->metaCast("IOMedia")) continue;
    OSNumber *entryMajor = OSDynamicCast(OSNumber, entry->getProperty(kIOBSDMajorKey));
    OSNumber *entryMinor = OSDynamicCast(OSNumber, entry->getProperty(kIOBSDMinorKey));
    if (!entryMajor || !entryMinor || entryMajor->unsigned32BitValue() != major || entryMinor->unsigned32BitValue() != minor) continue;
    OSString *bsdName = OSDynamicCast(OSString, entry->getProperty(kIOBSDNameKey));
    OSBoolean *flag = OSDynamicCast(OSBoolean, entry->getProperty("Writable"));
    strlcpy(name, bsdName ? bsdName->getCStringNoCopy() : "-", sizeof(name));
    writable = !flag || flag->isTrue();
    found = true;
  }
  iterator->release();
  return found;
}

// ASCII case-insensitive equality, for UUID strings (the kernel compares them in upper case, the
// property of an IOMedia may be either).
bool sameUuid(const char *a, const char *b) {
  if (!a || !b) return false;
  for (;; ++a, ++b) {
    char x = *a, y = *b;
    if (x >= 'a' && x <= 'z') x = static_cast<char>(x - 'a' + 'A');
    if (y >= 'a' && y <= 'z') y = static_cast<char>(y - 'a' + 'A');
    if (x != y) return false;
    if (!x) return true;
  }
}

// The IOMedia below `window` whose UUID property is `uuid`: its BSD name and device number, and
// whether it can be written (unknown counts as yes). The volumes of a filesystem carry the UUID
// of the volume itself, the one the original kernel code looks the root and the cryptex up by.
bool mediaByUuid(IORegistryEntry *window, const char *uuid, uint32_t &major, uint32_t &minor, char (&name)[32], bool &writable) {
  IORegistryIterator *iterator = IORegistryIterator::iterateOver(window, gIOServicePlane, kIORegistryIterateRecursively);
  if (!iterator) return false;
  bool found = false;
  for (unsigned visited = 0; visited < 512 && !found; ++visited) {
    IORegistryEntry *entry = iterator->getNextObject();
    if (!entry) break;
    if (!entry->metaCast("IOMedia")) continue;
    OSString *entryUuid = OSDynamicCast(OSString, entry->getProperty("UUID"));
    OSNumber *entryMajor = OSDynamicCast(OSNumber, entry->getProperty(kIOBSDMajorKey));
    OSNumber *entryMinor = OSDynamicCast(OSNumber, entry->getProperty(kIOBSDMinorKey));
    OSString *bsdName = OSDynamicCast(OSString, entry->getProperty(kIOBSDNameKey));
    if (!entryUuid || !sameUuid(entryUuid->getCStringNoCopy(), uuid) || !entryMajor || !entryMinor || !bsdName) continue;
    OSBoolean *flag = OSDynamicCast(OSBoolean, entry->getProperty("Writable"));
    major = entryMajor->unsigned32BitValue();
    minor = entryMinor->unsigned32BitValue();
    strlcpy(name, bsdName->getCStringNoCopy(), sizeof(name));
    writable = !flag || flag->isTrue();
    found = true;
  }
  iterator->release();
  return found;
}

// The DW watchdog of the SoC. The firmware sets its period (about 89 s) and feeds it until it
// leaves boot services; restarting the count is writing 0x76 to CRR. The registers other than
// CRR are only read, to log what the hardware did: CCVR is the current count (it counts down
// from 2^(16+TOP)-1, so right after a restart it is close to 0x7FFFFFFF for TOP = 0xF).
constexpr uint64_t kWdtPhys = 0xFEAF0000ull;
constexpr uint32_t kWdtCr = 0x00;
constexpr uint32_t kWdtTorr = 0x04;
constexpr uint32_t kWdtCcvr = 0x08;
constexpr uint32_t kWdtCrr = 0x0C;
constexpr uint32_t kWdtStat = 0x10;
constexpr uint32_t kWdtRestart = 0x76;
constexpr uint32_t kWdtFeedMs = 5000;

void wdtKicker(void *argument, wait_result_t) {
  const uint32_t seconds = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(argument));
  vm_offset_t base = ml_io_map(static_cast<vm_offset_t>(kWdtPhys), 0x1000);
  if (!base) {
    IOLog("NativeEmmc: wdt-kick: cannot map the watchdog at 0x%llx\n", ull(kWdtPhys));
    return;
  }
  volatile uint32_t *regs = reinterpret_cast<volatile uint32_t *>(base);
  IOLog("NativeEmmc: wdt-kick: feeding the watchdog every %u ms for %u s (CR 0x%x TORR 0x%x CCVR 0x%08x STAT 0x%x)\n",
        kWdtFeedMs, seconds, regs[kWdtCr / 4], regs[kWdtTorr / 4], regs[kWdtCcvr / 4], regs[kWdtStat / 4]);
  const uint64_t end = nowUs() + static_cast<uint64_t>(seconds) * 1000000ull;
  for (uint32_t feed = 1; nowUs() < end; ++feed) {
    const uint32_t before = regs[kWdtCcvr / 4];
    regs[kWdtCrr / 4] = kWdtRestart;
    __asm__ volatile("dsb sy" ::: "memory");
    const uint32_t after = regs[kWdtCcvr / 4];
    // The first feeds, then once a minute: enough to see that the count restarts, and the last
    // line before an unexplained reset says how long ago the last feed was.
    if (feed <= 3 || feed % 12 == 0) {
      IOLog("NativeEmmc: wdt-kick: feed %u: count 0x%08x -> 0x%08x (CR 0x%x STAT 0x%x)\n", feed, before, after,
            regs[kWdtCr / 4], regs[kWdtStat / 4]);
    }
    IOSleep(kWdtFeedMs);
  }
  IOLog("NativeEmmc: wdt-kick: %u s are over, not feeding the watchdog any more\n", seconds);
}

// The ApplePMGRNub the original AMFI waits for.
//
// AppleMobileFileIntegrity's configuration (ConfigurationSettings.cpp) finds the power manager with
// IOService::waitForMatchingService(serviceMatching("ApplePMGRNub")), 30 s after it starts, and panics
// "AMFI: No PMGR?" when nothing answers (a board run: launchd had been running for 27 s).
// What it then does with the nub is fixed by the original code (the PMGR audit script of
// UniXNU pins the instructions): read the nub's always-on word through the provider stored at nub+0x88,
// OR bit 2 into it, write it back. The helpers behind that return 0xffffffff and do nothing when the
// provider is NULL, so a REAL nub of the original class without any provider satisfies it: this board
// has no Apple power manager, and nothing here pretends to have one. UniXNU's sirius port found the
// same thing (its docs: "publishing a real but provider-less ApplePMGRNub satisfies the latch").
// Refused, with a log line, unless the class is exactly the original one.

constexpr unsigned kPmgrNubClassSize = 0x740;      // ApplePMGRNub of the pinned kernelcache
constexpr unsigned kPmgrNubProviderOffset = 0x88;  // where the original code keeps the nub's provider

void pmgrNubPublisher(void *, wait_result_t) {
  // The kext is a dependency of AMFI and normally started already; loading it again is a no-op then.
  kern_return_t loaded = OSKextLoadKextWithIdentifier("com.apple.driver.ApplePMGR");
  const OSMetaClass *meta = nullptr;
  for (unsigned tries = 0; tries < 100 && !meta; ++tries) {
    const OSSymbol *name = OSSymbol::withCString("ApplePMGRNub");
    meta = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
    if (name) name->release();
    if (!meta) IOSleep(100);
  }
  if (!meta) {
    IOLog("NativeEmmc: pmgr-nub: no class ApplePMGRNub (loading com.apple.driver.ApplePMGR returned 0x%x)\n", loaded);
    return;
  }
  if (meta->getClassSize() != kPmgrNubClassSize) {
    IOLog("NativeEmmc: pmgr-nub: ApplePMGRNub is 0x%x bytes, expected 0x%x: not the original of the pinned kernelcache, not publishing\n",
          meta->getClassSize(), kPmgrNubClassSize);
    return;
  }
  OSObject *object = meta->alloc();
  IOService *nub = OSDynamicCast(IOService, object);
  IOService *platform = IOService::getPlatform();
  if (!nub || !platform || !nub->init(static_cast<OSDictionary *>(nullptr)) || !nub->attach(platform)) {
    IOLog("NativeEmmc: pmgr-nub: cannot create the nub (object %p, platform %p)\n", object, platform);
    if (object) object->release();
    return;
  }
  uintptr_t provider = 0;
  memcpy(&provider, reinterpret_cast<const char *>(nub) + kPmgrNubProviderOffset, sizeof(provider));
  if (provider) {
    // Not provider-less after all: the layout is not the one this was written against.
    IOLog("NativeEmmc: pmgr-nub: the new nub already has a provider pointer 0x%llx, not publishing\n", ull(provider));
    nub->detach(platform);
    nub->release();
    return;
  }
  nub->registerService(kIOServiceSynchronous);
  IOLog("NativeEmmc: pmgr-nub: published an original ApplePMGRNub without provider (AMFI reads 0xffffffff from it)\n");
}

// The original IOBlockStorageDevice must be the class this file was compiled against, or a
// subclass would overlay its members on the base class's. The audit proves it for the pinned
// kernelcache; this proves it for the kernel that is running.
bool originalStorageClassMatches() {
  const OSMetaClass *base = &IOBlockStorageDevice::gMetaClass;
  const OSMetaClass *service = &IOService::gMetaClass;
  unsigned baseSize = base->getClassSize(), serviceSize = service->getClassSize();
  bool linked = base->getSuperClass() == service;
  IOLog("NativeEmmc: original IOBlockStorageDevice: class size 0x%x (expect 0x%x), IOService 0x%x (expect 0x%x), parent link %s\n",
        baseSize, static_cast<unsigned>(sizeof(IOBlockStorageDevice)), serviceSize, static_cast<unsigned>(sizeof(IOService)),
        linked ? "ok" : "WRONG");
  return baseSize == sizeof(IOBlockStorageDevice) && serviceSize == sizeof(IOService) && linked;
}

}  // namespace

NativeEmmcBlock *NativeEmmc::publishWindow(const char *label, uint64_t first, uint64_t blocks, unsigned role, bool writable) {
  if (!originalStorageClassMatches()) {
    IOLog("NativeEmmc: refusing to publish %s: the original storage classes are not the ones the driver was built for\n", label);
    return nullptr;
  }
  NativeEmmcBlock *device = OSTypeAlloc(NativeEmmcBlock);
  if (!device || !device->init(static_cast<OSDictionary *>(nullptr)) || !device->configure(this, label, first, blocks, writable) ||
      !device->attach(this)) {
    IOLog("NativeEmmc: cannot prepare block device %s\n", label);
    if (device) device->release();
    return nullptr;
  }
  NativeEmmcBlock **slot = role == 1 ? &gOsWindow : role == 2 ? &gSystemOsWindow : role == 3 ? &gDataWindow : nullptr;
  if (slot) __atomic_store_n(slot, device, __ATOMIC_RELEASE);
  if (!device->start(this)) {
    IOLog("NativeEmmc: block device %s did not start\n", label);
    if (slot) __atomic_store_n(slot, static_cast<NativeEmmcBlock *>(nullptr), __ATOMIC_RELEASE);
    device->detach(this);
    device->release();
    return nullptr;
  }
  // Started objects keep their reference: IOMedia clients hold the device for the life of the system.
  __atomic_fetch_add(&native_emmc_blocks_published, 1, __ATOMIC_RELAXED);
  IOLog("NativeEmmc: published %s: lba %llu + %llu blocks (%llu MiB), %s\n", device->windowLabel(), ull(first), ull(blocks),
        ull(blocks >> 11), writable ? "WRITABLE" : "read-only");
  return device;
}

// The Data RAM disk as a writable block device of its own. It hangs off the platform expert, not
// off the card: nothing about it depends on the controller, and a run without a card can still look
// at it. Like the root windows, a started device keeps its reference for the life of the system.
static NativeEmmcBlock *publishRamDisk(IOService *provider, const char *label, uint8_t *memory, uint64_t bytes) {
  if (!originalStorageClassMatches()) {
    IOLog("NativeEmmc: refusing to publish %s: the original storage classes are not the ones the driver was built for\n", label);
    return nullptr;
  }
  NativeEmmcBlock *device = OSTypeAlloc(NativeEmmcBlock);
  if (!device || !device->init(static_cast<OSDictionary *>(nullptr)) || !device->configureRam(label, memory, bytes) ||
      !device->attach(provider)) {
    IOLog("NativeEmmc: cannot prepare RAM block device %s\n", label);
    if (device) device->release();
    return nullptr;
  }
  __atomic_store_n(&gDataWindow, device, __ATOMIC_RELEASE);
  if (!device->start(provider)) {
    IOLog("NativeEmmc: RAM block device %s did not start\n", label);
    __atomic_store_n(&gDataWindow, static_cast<NativeEmmcBlock *>(nullptr), __ATOMIC_RELEASE);
    device->detach(provider);
    device->release();
    return nullptr;
  }
  __atomic_fetch_add(&native_emmc_blocks_published, 1, __ATOMIC_RELAXED);
  native_emmc_data_ready = 1;
  IOLog("NativeEmmc: published %s: RAM %p + %llu bytes (%llu MiB), writable\n", device->windowLabel(), static_cast<void *>(memory), ull(bytes),
        ull(bytes >> 20));
  return device;
}

// The range of the Data seed the payload hands over: /chosen/memory-map/RAMDisk is {physical base,
// size}, both u64, written by the loader (UniXNU's --data-ramdisk). The same bounds check the original
// RamBlock driver makes: page aligned, inside the memory the kernel manages, nowhere else.
static uint8_t *dataDiskRange(uint64_t &bytes) {
  bytes = 0;
  IORegistryEntry *map = IORegistryEntry::fromPath("/chosen/memory-map", gIODTPlane);
  if (!map) {
    IOLog("NativeEmmc: data-ram: no /chosen/memory-map\n");
    return nullptr;
  }
  OSData *range = OSDynamicCast(OSData, map->getProperty("RAMDisk"));
  uint64_t physical = 0, size = 0;
  if (range && range->getLength() == 16) {
    memcpy(&physical, range->getBytesNoCopy(), 8);
    memcpy(&size, static_cast<const uint8_t *>(range->getBytesNoCopy()) + 8, 8);
  }
  map->release();
  if (!size) {
    IOLog("NativeEmmc: data-ram: the payload hands over no RAMDisk\n");
    return nullptr;
  }
  if (size > 0x80000000ull || (physical & 4095) || (size & 4095) || physical < gPhysBase || size > gPhysSize ||
      physical - gPhysBase > gPhysSize - size) {
    IOLog("NativeEmmc: data-ram: RAMDisk %#llx + %#llx is not boot memory the kernel manages (%#lx + %#lx)\n", ull(physical), ull(size),
          gPhysBase, gPhysSize);
    return nullptr;
  }
  uint8_t *memory = reinterpret_cast<uint8_t *>(ml_static_ptovirt(static_cast<vm_offset_t>(physical)));
  if (!memory) {
    IOLog("NativeEmmc: data-ram: no static mapping for %#llx\n", ull(physical));
    return nullptr;
  }
  bytes = size;
  IOLog("NativeEmmc: data-ram: RAMDisk physical %#llx, %llu bytes, at %p; header %02x%02x%02x%02x %02x%02x%02x%02x (NXSB at +32: %02x%02x%02x%02x)\n",
        ull(physical), ull(size), static_cast<void *>(memory), memory[0], memory[1], memory[2], memory[3], memory[4], memory[5], memory[6], memory[7],
        memory[32], memory[33], memory[34], memory[35]);
  return memory;
}

// The seed on the payload is only the front of the Data container: an empty APFS container of 512 MiB has
// 18 blocks that are not zero, all of them in the first 1.7 MiB, and the payload (and the fastboot buffer it
// rides in) has no room for the other 510 MiB of zeros. With emmc-data-size=N the block device is N MiB:
// the seed is copied to the front of a fresh allocation and the rest is zero. The container inside says how
// large it is itself (its superblock), so N has to be the size it was made for.
static uint8_t *growDataDisk(uint8_t *seed, uint64_t &bytes) {
  const uint64_t wanted = static_cast<uint64_t>(bootArg("emmc-data-size", 0)) << 20;
  if (!wanted || wanted == bytes) return seed;
  if (wanted < bytes || wanted > (4ull << 30)) {
    IOLog("NativeEmmc: data-ram: emmc-data-size %llu MiB is not usable with a seed of %llu MiB\n", ull(wanted >> 20), ull(bytes >> 20));
    return seed;
  }
  IOBufferMemoryDescriptor *memory = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, wanted, page_size);
  uint8_t *grown = memory ? static_cast<uint8_t *>(memory->getBytesNoCopy()) : nullptr;
  if (!grown) {
    IOLog("NativeEmmc: data-ram: no memory for %llu MiB\n", ull(wanted >> 20));
    if (memory) memory->release();
    return seed;
  }
  memcpy(grown, seed, bytes);
  bzero(grown + bytes, wanted - bytes);
  IOLog("NativeEmmc: data-ram: %llu MiB of kernel memory at %p: the %llu MiB seed in front, zeros behind\n", ull(wanted >> 20),
        static_cast<void *>(grown), ull(bytes >> 20));
  bytes = wanted;  // the descriptor is kept for the life of the system, like the disk
  return grown;
}

// Runs on a thread of its own: publishing is followed by waiting for the partition scheme and APFS,
// which the thread that starts the controller would be holding up.
static void dataDiskPublisher(void *, wait_result_t) {
  uint64_t bytes = 0;
  uint8_t *memory = dataDiskRange(bytes);
  if (!memory) return;
  memory = growDataDisk(memory, bytes);
  IOService *platform = IOService::getPlatform();
  if (!platform) {
    IOLog("NativeEmmc: data-ram: no platform expert to attach to\n");
    return;
  }
  NativeEmmcBlock *device = publishRamDisk(platform, "Data", memory, bytes);
  if (!device) return;
  unsigned media = settleMedia(device, 20);
  IOLog("NativeEmmc: Data settled with %u IOMedia below it, %llu ram requests (%llu writes), %llu failures\n", media,
        ull(native_emmc_ram_requests), ull(native_emmc_ram_writes), ull(native_emmc_ram_failures));
  mediaBelow(device, true);
}

// ---- the Data volume on the card (emmc-data-persist=1) -----------------------------------------------------
//
// Where: the END of `userdata`, so that nothing else ever has to move. A window of
//     kDataHeaderBlocks (1 MiB) of header, then the APFS container of the payload's seed, whose size is the
//     seed's own (nx_block_size * nx_block_count at +36/+40 of its superblock). It must lie at least 64 MiB behind
//     the end of the root image, and inside `userdata`; otherwise nothing is written and the RAM disk is used.
// What is written, and when: only inside that window, only by this thread and the Data block device, and only
//     after the checks above (NativeEmmc::enableWrites gives the host core that one range, the core refuses
//     the rest). A first boot formats: header "formatting", the first 64 MiB of the container zeroed (the old
//     contents of the window must not look like APFS objects), the seed written over its front, flushed, read
//     back and compared, header "ready". A header that says "formatting" (power loss) is formatted again.
// Header (one 512-byte block, little endian): +0 "EVB10DAT", +8 version, +12 state (1 formatting, 2 ready),
//     +16 container blocks (u64), +24 seed bytes (u64), +32 crc32 of the seed, +36 number of formats so far,
//     +508 crc32 of bytes 0..507.
namespace {

constexpr uint32_t kDataHeaderBlocks = 2048;        // 1 MiB
constexpr uint64_t kDataMarginBlocks = 131072;      // 64 MiB between the root image and the window
constexpr uint64_t kDataZeroBlocks = 131072;        // 64 MiB zeroed in front of the container when formatting
constexpr uint64_t kDataMaxBytes = 8ull << 30;
constexpr uint32_t kDataVersion = 1;
constexpr uint32_t kDataFormatting = 1, kDataReady = 2;
constexpr char kDataMagic[8] = {'E', 'V', 'B', '1', '0', 'D', 'A', 'T'};

struct DataHeader {
  uint32_t version = 0, state = 0;
  uint64_t dataBlocks = 0, seedBytes = 0;
  uint32_t seedCrc = 0, generation = 0;
};

void encodeDataHeader(uint8_t (&block)[emmc::kBlockBytes], const DataHeader &header) {
  bzero(block, sizeof(block));
  memcpy(block, kDataMagic, 8);
  memcpy(block + 8, &header.version, 4);
  memcpy(block + 12, &header.state, 4);
  memcpy(block + 16, &header.dataBlocks, 8);
  memcpy(block + 24, &header.seedBytes, 8);
  memcpy(block + 32, &header.seedCrc, 4);
  memcpy(block + 36, &header.generation, 4);
  const uint32_t crc = emmc::crc32(block, 508);
  memcpy(block + 508, &crc, 4);
}

bool decodeDataHeader(const uint8_t (&block)[emmc::kBlockBytes], DataHeader &header) {
  if (memcmp(block, kDataMagic, 8)) return false;
  uint32_t crc = 0;
  memcpy(&crc, block + 508, 4);
  if (crc != emmc::crc32(block, 508)) return false;
  memcpy(&header.version, block + 8, 4);
  memcpy(&header.state, block + 12, 4);
  memcpy(&header.dataBlocks, block + 16, 8);
  memcpy(&header.seedBytes, block + 24, 8);
  memcpy(&header.seedCrc, block + 32, 4);
  memcpy(&header.generation, block + 36, 4);
  return true;
}

}  // namespace

// Called with nothing held, from dataPublisher. `seed` is `seedBytes` of the payload's RAM disk.
bool NativeEmmc::formatData(uint64_t windowFirst, uint64_t dataBlocks, const uint8_t *seed, uint64_t seedBytes, uint32_t seedCrc,
                            uint32_t generation) {
  const uint64_t begin = nowUs();
  const uint64_t dataFirst = windowFirst + kDataHeaderBlocks;
  alignas(8) uint8_t block[emmc::kBlockBytes];
  uint8_t *buffer = static_cast<uint8_t *>(IOMallocData(kChunkBytes));
  if (!buffer) return false;
  bool ok = true;
  DataHeader header;
  header.version = kDataVersion;
  header.state = kDataFormatting;
  header.dataBlocks = dataBlocks;
  header.seedBytes = seedBytes;
  header.seedCrc = seedCrc;
  header.generation = generation;
  encodeDataHeader(block, header);
  if (writeBlocks(windowFirst, 1, block) != kIOReturnSuccess || flush() != kIOReturnSuccess) {
    IOLog("NativeEmmc: data: cannot write the header of the Data window\n");
    ok = false;
  }
  const uint64_t zeroBlocks = dataBlocks < kDataZeroBlocks ? dataBlocks : kDataZeroBlocks;
  bzero(buffer, kChunkBytes);
  for (uint64_t at = 0; ok && at < zeroBlocks; at += kChunkBlocks) {
    const uint32_t count = static_cast<uint32_t>(zeroBlocks - at < kChunkBlocks ? zeroBlocks - at : kChunkBlocks);
    if (writeBlocks(dataFirst + at, count, buffer) != kIOReturnSuccess) {
      IOLog("NativeEmmc: data: zeroing failed at block %llu of the window\n", ull(at));
      ok = false;
    }
  }
  const uint64_t zeroed = nowUs();
  for (uint64_t at = 0; ok && at < seedBytes; at += kChunkBytes) {
    const uint64_t bytes = seedBytes - at < kChunkBytes ? seedBytes - at : kChunkBytes;
    memcpy(buffer, seed + at, static_cast<size_t>(bytes));
    if (writeBlocks(dataFirst + at / emmc::kBlockBytes, static_cast<uint32_t>(bytes / emmc::kBlockBytes), buffer) != kIOReturnSuccess) {
      IOLog("NativeEmmc: data: writing the seed failed at byte %llu\n", ull(at));
      ok = false;
    }
  }
  if (ok && flush() != kIOReturnSuccess) {
    IOLog("NativeEmmc: data: the flush after the seed failed\n");
    ok = false;
  }
  // What the card holds is what was meant: the seed read back, through the same path the kernel will read it by.
  if (ok) {
    uint32_t crc = 0;
    for (uint64_t at = 0; ok && at < seedBytes; at += kChunkBytes) {
      const uint64_t bytes = seedBytes - at < kChunkBytes ? seedBytes - at : kChunkBytes;
      if (readBlocks(dataFirst + at / emmc::kBlockBytes, static_cast<uint32_t>(bytes / emmc::kBlockBytes), buffer) != kIOReturnSuccess) {
        IOLog("NativeEmmc: data: read-back failed at byte %llu\n", ull(at));
        ok = false;
        break;
      }
      crc = emmc::crc32(buffer, static_cast<size_t>(bytes), crc);
    }
    if (ok && crc != seedCrc) {
      IOLog("NativeEmmc: data: the seed read back from the card differs from what was written (crc32 0x%08x, expected 0x%08x)\n", crc, seedCrc);
      ok = false;
    }
  }
  if (ok) {
    header.state = kDataReady;
    encodeDataHeader(block, header);
    if (writeBlocks(windowFirst, 1, block) != kIOReturnSuccess || flush() != kIOReturnSuccess) {
      IOLog("NativeEmmc: data: cannot write the ready header\n");
      ok = false;
    }
  }
  IOFreeData(buffer, kChunkBytes);
  IOLog("NativeEmmc: data: formatted the Data window at lba %llu (%llu MiB zeroed in %llu ms, seed of %llu KiB, all in %llu ms): %s\n",
        ull(windowFirst), ull(zeroBlocks >> 11), ull((zeroed - begin) / 1000), ull(seedBytes >> 10), ull((nowUs() - begin) / 1000),
        ok ? "ready" : "FAILED");
  return ok;
}

// Decides, formats if needed, and publishes. false = nothing was published (and nothing is writable any more).
bool NativeEmmc::persistentData() {
  uint64_t seedBytes = 0;
  uint8_t *seed = dataDiskRange(seedBytes);
  if (!seed) return false;
  if (!rootReady || !userdataLast) {
    IOLog("NativeEmmc: data: persistent Data needs emmc=4 and a usable root image (rootReady %d)\n", rootReady ? 1 : 0);
    return false;
  }
  if (!native_emmc_dma) {
    IOLog("NativeEmmc: data: persistent Data needs DMA, which is off\n");
    return false;
  }
  // The container the seed describes: nx_superblock_t, nx_magic at +32, nx_block_size at +36, nx_block_count at +40.
  uint32_t apfsBlock = 0;
  uint64_t apfsCount = 0;
  if (seedBytes >= 4096 && !memcmp(seed + 32, "NXSB", 4)) {
    memcpy(&apfsBlock, seed + 36, 4);
    memcpy(&apfsCount, seed + 40, 8);
  }
  if (apfsBlock < 4096 || apfsBlock > 65536 || (apfsBlock & (apfsBlock - 1)) || !apfsCount || apfsCount > kDataMaxBytes / apfsBlock) {
    IOLog("NativeEmmc: data: the seed is no APFS container (block size %u, %llu blocks)\n", apfsBlock, ull(apfsCount));
    return false;
  }
  const uint64_t containerBytes = static_cast<uint64_t>(apfsBlock) * apfsCount;
  if ((containerBytes & 0xfffff) || containerBytes < seedBytes || containerBytes < (64ull << 20)) {
    IOLog("NativeEmmc: data: a container of %llu bytes does not fit a seed of %llu bytes, or is not whole MiB, or is under 64 MiB\n",
          ull(containerBytes), ull(seedBytes));
    return false;
  }
  const uint64_t dataBlocks = containerBytes / emmc::kBlockBytes;
  const uint32_t sizeArg = bootArg("emmc-data-size", 0);
  if (sizeArg && (static_cast<uint64_t>(sizeArg) << 20) != containerBytes) {
    IOLog("NativeEmmc: data: emmc-data-size=%u is not the seed's container (%llu MiB); the container decides\n", sizeArg, ull(containerBytes >> 20));
  }
  const uint64_t total = kDataHeaderBlocks + dataBlocks;
  const uint64_t userdataBlocks = userdataLast - userdataFirst + 1;
  if (total > userdataBlocks) {
    IOLog("NativeEmmc: data: %llu MiB do not fit userdata (%llu MiB)\n", ull(total >> 11), ull(userdataBlocks >> 11));
    return false;
  }
  const uint64_t windowFirst = userdataLast + 1 - total;
  const uint64_t rootEnd = rootLba + (root.imageBytes + emmc::kBlockBytes - 1) / emmc::kBlockBytes;
  if (windowFirst < rootEnd + kDataMarginBlocks) {
    IOLog("NativeEmmc: data: the window at %llu would come closer than 64 MiB to the end of the root image (%llu); not writing\n",
          ull(windowFirst), ull(rootEnd));
    return false;
  }
  IOLog("NativeEmmc: data: window lba %llu..%llu (%llu MiB: 1 MiB header + %llu MiB container of %u-byte blocks), root image ends at %llu\n",
        ull(windowFirst), ull(userdataLast), ull(total >> 11), ull(containerBytes >> 20), apfsBlock, ull(rootEnd));

  const uint32_t seedCrc = emmc::crc32(seed, static_cast<size_t>(seedBytes));
  alignas(8) uint8_t block[emmc::kBlockBytes];
  DataHeader header;
  const bool haveHeader = readBlocks(windowFirst, 1, block) == kIOReturnSuccess && decodeDataHeader(block, header);
  const bool reset = bootArg("emmc-data-reset", 0) != 0;
  const bool keep = haveHeader && !reset && header.version == kDataVersion && header.state == kDataReady && header.dataBlocks == dataBlocks;
  if (haveHeader) {
    IOLog("NativeEmmc: data: header: version %u state %u, %llu blocks, seed crc 0x%08x, formatted %u times%s\n", header.version, header.state,
          ull(header.dataBlocks), header.seedCrc, header.generation, reset ? " (emmc-data-reset=1)" : "");
  } else {
    IOLog("NativeEmmc: data: no header in the window%s: formatting\n", reset ? " (emmc-data-reset=1)" : "");
  }
  if (!enableWrites(windowFirst, userdataLast)) {
    IOLog("NativeEmmc: data: the host core would not open the window for writing\n");
    return false;
  }
  if (keep) {
    if (header.seedCrc != seedCrc) {
      IOLog("NativeEmmc: data: it was formatted from another seed (crc32 0x%08x, this payload's is 0x%08x); the contents are kept\n", header.seedCrc,
            seedCrc);
    }
    native_emmc_data_seeded = 1;
  } else if (!formatData(windowFirst, dataBlocks, seed, seedBytes, seedCrc, haveHeader ? header.generation + 1 : 1)) {
    disableWrites();
    return false;
  }
  native_emmc_data_persistent = 1;
  NativeEmmcBlock *device = publishWindow("Data", windowFirst + kDataHeaderBlocks, dataBlocks, 3, true);
  if (!device) {
    native_emmc_data_persistent = 0;
    native_emmc_data_seeded = 0;
    disableWrites();
    return false;
  }
  native_emmc_data_ready = 1;
  unsigned media = settleMedia(device, 20);
  IOLog("NativeEmmc: Data (on the card) settled with %u IOMedia below it, %llu write calls (%llu blocks), %llu failures\n", media,
        ull(native_emmc_write_calls), ull(native_emmc_write_blocks), ull(native_emmc_write_failures));
  mediaBelow(device, true);
  return true;
}

void NativeEmmc::dataPublisher(void *self, wait_result_t) {
  NativeEmmc *controller = static_cast<NativeEmmc *>(self);
  if (!controller->persistentData()) {
    IOLog("NativeEmmc: data: no persistent Data volume (see above): the Data RAM disk instead\n");
    if (bootArg("emmc-data-ram", 1)) dataDiskPublisher(nullptr, 0);
  }
  controller->release();  // the reference start() took for this thread
}

// The boot-manifest-hash of /chosen. On a phone iBoot publishes it (the SHA-384 of the img4 manifest
// it booted from) and the original `mount -P 1` cannot go on without it: `kern.bootobjectspath` is the
// ENOENT behind "failed to get boot manifest hash", the exit status 66 of that boot task is a critical
// failure, and launchd panics ("mount[3] exited"). UniXNU's own offline runs hit exactly that and
// publish an owner hash from the verified root volumes (patches/xnu/offline-owner-boot-manifest.patch).
// There is no iBoot here either, so the hash names what was booted from the owner's record of it:
// the SHA-256 of the whole OS window and of the whole SystemOS window, as the root manifest lists
// them (the manifest itself is checked against its own SHA-256 before anything is used), role byte
// and digest per volume under a domain of our own, SHA-384 over all of it, the construction of the
// official patch. An identity that is already in /chosen is never replaced.
// boot-arg emmc-boot-manifest=0 leaves it out.
static bool publishBootManifestHash(const emmc::RootManifest &manifest) {
  static const char domain[] = "EmmcRootImage.OwnerBootSet.v1";  // the NUL is part of it
  const emmc::RootWindow *os = nullptr, *system = nullptr;
  for (unsigned i = 0; i < manifest.count; ++i) {
    if (emmc::sameName(manifest.window[i].name, "OS")) os = &manifest.window[i];
    if (emmc::sameName(manifest.window[i].name, "SystemOS")) system = &manifest.window[i];
  }
  if (!os || !system) {
    IOLog("NativeEmmc: boot-manifest-hash: the root image has no %s window, nothing to derive it from\n", os ? "SystemOS" : "OS");
    return false;
  }
  const uint8_t osRole = 1, systemRole = 2;
  uint8_t digest[SHA384_DIGEST_LENGTH];
  SHA384_CTX context;
  SHA384_Init(&context);
  SHA384_Update(&context, domain, sizeof(domain));
  SHA384_Update(&context, &osRole, sizeof(osRole));
  SHA384_Update(&context, os->sha256, sizeof(os->sha256));
  SHA384_Update(&context, &systemRole, sizeof(systemRole));
  SHA384_Update(&context, system->sha256, sizeof(system->sha256));
  SHA384_Final(digest, &context);

  IORegistryEntry *chosen = IORegistryEntry::fromPath("/chosen", gIODTPlane);
  if (!chosen) {
    IOLog("NativeEmmc: boot-manifest-hash: no /chosen\n");
    return false;
  }
  bool published = false;
  if (chosen->getProperty("boot-manifest-hash") || chosen->getProperty("owner-boot-manifest-domain")) {
    IOLog("NativeEmmc: boot-manifest-hash: /chosen already carries an identity, not replaced\n");
  } else {
    OSData *value = OSData::withBytes(digest, sizeof(digest));
    OSData *tag = OSData::withBytes(domain, sizeof(domain));
    published = value && tag && chosen->setProperty("owner-boot-manifest-domain", tag) && chosen->setProperty("boot-manifest-hash", value);
    OSSafeReleaseNULL(tag);
    OSSafeReleaseNULL(value);
  }
  chosen->release();
  if (published) {
    static const char digits[] = "0123456789abcdef";
    char text[2 * sizeof(digest) + 1];
    for (unsigned i = 0; i < sizeof(digest); ++i) {
      text[2 * i] = digits[digest[i] >> 4];
      text[2 * i + 1] = digits[digest[i] & 15];
    }
    text[2 * sizeof(digest)] = 0;
    IOLog("NativeEmmc: boot-manifest-hash published in /chosen: %s (domain %s)\n", text, domain);
  }
  return published;
}

// The manifest of the root image: read at `rootLba` (the start of `userdata` unless a boot-arg
// says otherwise), checked by RootManifest.h, and turned into the order the windows go out in.
// Called with the gate held.
bool NativeEmmc::findRoot() {
  rootLba = bootArg64("emmc-root-lba", userdataFirst);
  if (!userdataLast || rootLba < userdataFirst || rootLba > userdataLast) {
    IOLog("NativeEmmc: no room for a root image (userdata %llu..%llu, manifest at %llu)\n", ull(userdataFirst), ull(userdataLast),
          ull(rootLba));
    return false;
  }
  uint8_t *page = static_cast<uint8_t *>(IOMallocData(emmc::kRootManifestBytes));
  if (!page) return false;
  bool usable = false;
  if (host.readBlocks(rootLba, emmc::kRootManifestBytes / emmc::kBlockBytes, page) != emmc::kOk) {
    IOLog("NativeEmmc: cannot read the root manifest at block %llu\n", ull(rootLba));
  } else {
    const uint64_t partitionBytes = (userdataLast - rootLba + 1) * emmc::kBlockBytes;
    int status = emmc::parseRootManifest(page, partitionBytes, sha256, root);
    native_emmc_root_status = status;
    if (status != emmc::kRootOk) {
      IOLog("NativeEmmc: block %llu holds no usable root image: %s\n", ull(rootLba), emmc::rootStatusName(status));
    } else {
      IOLog("NativeEmmc: root image at block %llu: %u windows, %llu bytes of %llu\n", ull(rootLba), root.count,
            ull(root.imageBytes), ull(partitionBytes));
      for (unsigned i = 0; i < root.count; ++i) {
        IOLog("NativeEmmc:   window \"%s\" offset 0x%llx size %llu bytes (%llu MiB)\n", root.window[i].name, ull(root.window[i].offset),
              ull(root.window[i].size), ull(root.window[i].size >> 20));
      }
      usable = emmc::buildRootPlan(root, rootPlan);
      if (!usable) {
        IOLog("NativeEmmc: the root image has no \"OS\" window\n");
      } else {
        IOLog("NativeEmmc: publication order:");
        for (unsigned i = 0; i < rootPlan.count; ++i) IOLog(" %s", root.window[rootPlan.order[i]].name);
        IOLog("\n");
        if (bootArg("emmc-boot-manifest", 1)) publishBootManifestHash(root);
      }
    }
  }
  IOFreeData(page, emmc::kRootManifestBytes);
  return usable;
}

// Publishes the windows in plan order, each after the previous one has settled: the kernel
// goes looking for the SystemOS volume as soon as it has mounted the root, so by then that one
// must be there. Runs on a thread of its own because settling means waiting for matching
// that the thread starting the controller would be holding up.
void NativeEmmc::publishRoot() {
  for (unsigned position = 0; position < rootPlan.count; ++position) {
    const emmc::RootWindow &window = root.window[rootPlan.order[position]];
    NativeEmmcBlock *device = publishWindow(window.name, rootLba + window.offset / emmc::kBlockBytes,
                                            window.size / emmc::kBlockBytes,
                                            emmc::sameName(window.name, "OS") ? 1u : emmc::sameName(window.name, "SystemOS") ? 2u : 0u);
    if (!device) {
      IOLog("NativeEmmc: root publisher stops: %s did not publish\n", window.name);
      return;
    }
    __atomic_fetch_add(&native_emmc_root_windows, 1, __ATOMIC_RELAXED);
    unsigned media = settleMedia(device, 20);
    IOLog("NativeEmmc: %s settled with %u IOMedia below it, %llu block requests, %llu failures\n", window.name, media,
          ull(native_emmc_block_requests), ull(native_emmc_block_failures));
    mediaBelow(device, true);
  }
  IOLog("NativeEmmc: all %u root windows published\n", rootPlan.count);
}

void NativeEmmc::rootPublisher(void *self, wait_result_t) {
  NativeEmmc *controller = static_cast<NativeEmmc *>(self);
  controller->publishRoot();
  controller->release();  // the reference start() took for this thread
}

// bsd_init asks this after it mounted the root and Apple's FSIOC_KERNEL_ROOTAUTH said no. The
// answer is yes only for the volume the owner put in the "OS" window of the eMMC root image,
// and only if the owner said so on the command line: the trust is the owner's, not the kernel's.
extern "C" int native_emmc_root_trusted(int32_t dev) {
  const uint32_t major = (static_cast<uint32_t>(dev) >> 24) & 0xff, minor = static_cast<uint32_t>(dev) & 0xffffff;
  NativeEmmcBlock *window = __atomic_load_n(&gOsWindow, __ATOMIC_ACQUIRE);
  if (!window) {
    IOLog("NativeEmmc: root device %u,%u is not trusted: this driver published no \"OS\" window\n", major, minor);
    return 0;
  }
  char name[32] = {};
  bool writable = true;
  if (!mediaByDevice(window, major, minor, name, writable)) {
    IOLog("NativeEmmc: root device %u,%u is not trusted: it is no IOMedia below the \"OS\" window\n", major, minor);
    return 0;
  }
  if (writable) {
    IOLog("NativeEmmc: root device %u,%u (%s) is not trusted: the media is writable\n", major, minor, name);
    return 0;
  }
  if (!bootArg("emmc-root-trust", 0)) {
    IOLog("NativeEmmc: root device %u,%u (%s) is a read-only volume of the \"OS\" window, but boot-arg emmc-root-trust=1 is not set: "
          "not trusted\n", major, minor, name);
    return 0;
  }
  IOLog("NativeEmmc: root device %u,%u (%s) is a read-only volume of the \"OS\" window: accepted without Apple's root authentication "
        "(boot-arg emmc-root-trust=1)\n", major, minor, name);
  return 1;
}

// The volume of the "SystemOS" window (the cryptex: dyld shared cache, system libraries) with
// this UUID: its device number and its /dev path, for the kernel's own mount of it over the root
// (UniXNU-evb10 patches/xnu/evb10-userland-mounts.patch, the counterpart of the original owner code's
// native_verified_block_volume_device). Returns 1 when the window has such a read-only volume,
// 0 otherwise. Only the owner's say-so (emmc-root-trust=1) makes it an answer at all.
extern "C" int native_emmc_volume_device(const char *uuid, int32_t *dev, char *path, size_t pathSize) {
  NativeEmmcBlock *window = __atomic_load_n(&gSystemOsWindow, __ATOMIC_ACQUIRE);
  if (!window || !uuid || !dev || !path || pathSize < 16) return 0;
  if (!bootArg("emmc-root-trust", 0)) {
    IOLog("NativeEmmc: SystemOS volume %s not offered: boot-arg emmc-root-trust=1 is not set\n", uuid);
    return 0;
  }
  uint32_t major = 0, minor = 0;
  char name[32] = {};
  bool writable = true;
  if (!mediaByUuid(window, uuid, major, minor, name, writable)) {
    IOLog("NativeEmmc: the \"SystemOS\" window has no volume %s\n", uuid);
    return 0;
  }
  if (writable) {
    IOLog("NativeEmmc: SystemOS volume %s (%s) is writable: not offered\n", uuid, name);
    return 0;
  }
  *dev = static_cast<int32_t>((major << 24) | (minor & 0xffffff));
  snprintf(path, pathSize, "/dev/%s", name);
  IOLog("NativeEmmc: SystemOS volume %s is %s (device %u,%u), read-only\n", uuid, path, major, minor);
  return 1;
}

// May the kernel treat `dev` as the SystemOS volume it mounts? Yes for a read-only IOMedia below the
// "SystemOS" window this driver published, with emmc-root-trust=1: the same decision, and the same
// owner, as native_emmc_root_trusted makes for the root.
extern "C" int native_emmc_cache_trusted(int32_t dev) {
  const uint32_t major = (static_cast<uint32_t>(dev) >> 24) & 0xff, minor = static_cast<uint32_t>(dev) & 0xffffff;
  NativeEmmcBlock *window = __atomic_load_n(&gSystemOsWindow, __ATOMIC_ACQUIRE);
  if (!window || !bootArg("emmc-root-trust", 0)) return 0;
  char name[32] = {};
  bool writable = true;
  if (!mediaByDevice(window, major, minor, name, writable) || writable) return 0;
  return 1;
}

// The volume of the Data RAM disk with this UUID: the BSD device and /dev path the kernel mounts on
// /private/var. Waits for it, for at most `emmc-data-wait` seconds (default 30): the partition scheme
// and APFS publish their media on threads of their own, a little after the RAM disk went out. No trust
// question here, the Data volume is the owner's by construction (it IS the payload's RAM disk); the
// answer is yes only for a WRITABLE volume inside the device this driver published.
extern "C" int native_emmc_data_device(const char *uuid, int32_t *dev, char *path, size_t pathSize) {
  if (!uuid || !dev || !path || pathSize < 16) return 0;
  const uint32_t seconds = bootArg("emmc-data-wait", 30);
  uint32_t major = 0, minor = 0;
  char name[32] = {};
  bool writable = false, found = false;
  NativeEmmcBlock *window = nullptr;
  for (uint32_t tick = 0; tick <= seconds * 10; ++tick) {
    window = __atomic_load_n(&gDataWindow, __ATOMIC_ACQUIRE);
    if (window && mediaByUuid(window, uuid, major, minor, name, writable)) {
      found = true;
      break;
    }
    if (tick < seconds * 10) IOSleep(100);
  }
  if (!window) {
    IOLog("NativeEmmc: no Data RAM disk was published (waited %u s)\n", seconds);
    return 0;
  }
  if (!found) {
    IOLog("NativeEmmc: the Data RAM disk has no volume %s (waited %u s)\n", uuid, seconds);
    mediaBelow(window, true);
    return 0;
  }
  if (!writable) {
    IOLog("NativeEmmc: Data volume %s (%s) is not writable: not offered\n", uuid, name);
    return 0;
  }
  *dev = static_cast<int32_t>((major << 24) | (minor & 0xffffff));
  snprintf(path, pathSize, "/dev/%s", name);
  IOLog("NativeEmmc: Data volume %s is %s (device %u,%u), writable\n", uuid, path, major, minor);
  return 1;
}

bool NativeEmmc::scanGpt() {
  uint8_t *scratch = static_cast<uint8_t *>(IOMallocData(emmc::kGptScratchBytes));
  emmc::GptEntry *entries = static_cast<emmc::GptEntry *>(IOMallocData(sizeof(emmc::GptEntry) * emmc::kGptMaxEntries));
  if (!scratch || !entries) {
    if (scratch) IOFreeData(scratch, emmc::kGptScratchBytes);
    if (entries) IOFreeData(entries, sizeof(emmc::GptEntry) * emmc::kGptMaxEntries);
    return false;
  }
  emmc::GptInfo info;
  int status = emmc::readGpt(
      [this](uint64_t lba, uint32_t count, void *out) { return host.readBlocks(lba, count, out) == emmc::kOk; },
      scratch, info, entries, emmc::kGptMaxEntries);
  if (status == emmc::kGptOk) {
    char guid[37];
    emmc::formatGuid(info.diskGuid, guid);
    diskBlocks = info.altLba + 1;
    IOLog("NativeEmmc: GPT disk %s: my=%llu alt=%llu (%llu blocks, %llu MiB) usable %llu..%llu, %u x %u entries at %llu; "
          "header crc %s, entries crc %s\n",
          guid, ull(info.myLba), ull(info.altLba), ull(diskBlocks), ull(diskBlocks >> 11), ull(info.firstUsable),
          ull(info.lastUsable), info.count, info.entrySize, ull(info.entriesLba), info.headerCrcOk ? "ok" : "BAD",
          info.entriesCrcOk ? "ok" : "BAD");
    for (uint32_t i = 0; i < info.used; ++i) {
      const emmc::GptEntry &entry = entries[i];
      char type[37];
      emmc::formatGuid(entry.typeGuid, type);
      IOLog("NativeEmmc: gpt[%u] \"%s\" lba %llu-%llu (%llu MiB) type %s\n", entry.slot, entry.name, ull(entry.first),
            ull(entry.last), ull(entry.last >= entry.first ? (entry.last - entry.first + 1) >> 11 : 0), type);
      if (emmc::sameName(entry.name, "userdata") && entry.last >= entry.first) {
        userdataFirst = entry.first;
        userdataLast = entry.last;
      }
    }
    IOLog("NativeEmmc: %u partitions\n", info.used);
  } else {
    IOLog("NativeEmmc: no usable GPT (status %d)\n", status);
  }
  IOFreeData(scratch, emmc::kGptScratchBytes);
  IOFreeData(entries, sizeof(emmc::GptEntry) * emmc::kGptMaxEntries);
  return status == emmc::kGptOk;
}

void NativeEmmc::probeUserdata() {
  if (!userdataLast) {
    IOLog("NativeEmmc: no partition named \"userdata\"\n");
    return;
  }
  native_emmc_userdata_first = userdataFirst;
  native_emmc_userdata_last = userdataLast;
  alignas(8) uint8_t block[emmc::kBlockBytes];
  if (host.readBlocks(userdataFirst, 1, block) != emmc::kOk) {
    IOLog("NativeEmmc: cannot read the first block of userdata\n");
    return;
  }
  hexLine("userdata head", userdataFirst, block, 32);
  char magic[9];
  for (unsigned i = 0; i < 8; ++i) magic[i] = block[i] >= 0x20 && block[i] < 0x7f ? static_cast<char>(block[i]) : '.';
  magic[8] = 0;
  IOLog("NativeEmmc: userdata starts with \"%s\" (an UniXNU root image starts with SHNGROOT)\n", magic);
  // An HFS+ volume header lives at byte 1024 of its volume (all fields big endian): signature
  // "H+" or "HX", then blockSize at +40, totalBlocks +44, freeBlocks +48, nextAllocation +52.
  // That is what the earlier iOS port left in `userdata`; the log says how much of it there is.
  if (host.readBlocks(userdataFirst + 2, 1, block) == emmc::kOk) {
    hexLine("userdata +1024", userdataFirst + 2, block, 16);
    auto be32 = [&block](unsigned at) {
      return (static_cast<uint32_t>(block[at]) << 24) | (block[at + 1] << 16) | (block[at + 2] << 8) | block[at + 3];
    };
    if (block[0] == 'H' && (block[1] == '+' || block[1] == 'X')) {
      uint64_t unit = be32(40);
      IOLog("NativeEmmc: userdata holds an HFS%s volume: %u-byte blocks, %u total (%llu MiB), %u free, next allocation %u\n",
            block[1] == '+' ? "+" : "X", be32(40), be32(44), ull(unit * be32(44) >> 20), be32(48), be32(52));
    }
  }

  uint32_t want = bootArg("emmc-probe", 1024);
  if (want == 0) return;
  const uint32_t chunk = 128;  // 64 KiB per read
  uint8_t *buffer = static_cast<uint8_t *>(IOMallocData(chunk * emmc::kBlockBytes));
  if (!buffer) return;
  // `want` blocks from `at`, read the way the kernel will read them: the crc of what came back and
  // how long it took.
  auto pass = [&](uint64_t at, uint32_t &crc, uint64_t &elapsed) {
    crc = 0;
    uint64_t begin = nowUs();
    for (uint32_t done = 0; done < want; done += chunk) {
      uint32_t n = want - done < chunk ? want - done : chunk;
      if (host.readBlocks(at + done, n, buffer) != emmc::kOk) return false;
      crc = emmc::crc32(buffer, n * emmc::kBlockBytes, crc);
    }
    elapsed = nowUs() - begin;
    return true;
  };
  auto rate = [&](uint64_t us) {
    return us ? ull(static_cast<uint64_t>(want) * emmc::kBlockBytes * 1000000ull / 1024 / us) : 0ull;
  };
  uint32_t crcA = 0, crcB = 0, crcC = 0, crcD = 0;
  uint64_t usA = 0, usB = 0, usC = 0, usD = 0;
  host.setMultiBlock(false);
  if (!pass(userdataFirst, crcA, usA) || !pass(userdataFirst, crcB, usB)) {
    IOFreeData(buffer, chunk * emmc::kBlockBytes);
    IOLog("NativeEmmc: probe read of userdata failed\n");
    return;
  }
  IOLog("NativeEmmc: probe %u blocks of userdata, one block per command, twice: crc32 0x%08x / 0x%08x (%s), %llu us / %llu us = %llu KiB/s\n",
        want, crcA, crcB, crcA == crcB ? "stable" : "MISMATCH", ull(usA), ull(usB), rate(usA));
  // The same blocks as runs of blocks (CMD18, one command per 64 KiB), compared with the reads above;
  // then the next range, untouched, for the rate of a card that has not just been asked for the data.
  // Only a probe without a mismatch, a failure or a fallback to single blocks keeps runs switched on.
  if (crcA == crcB && bootArg("emmc-multi", 1)) {
    host.setMultiBlock(true);
    bool ranOk = pass(userdataFirst, crcC, usC);
    bool coldOk = userdataFirst + 2ull * want <= userdataLast + 1 && pass(userdataFirst + want, crcD, usD);
    bool keep = ranOk && crcC == crcA && host.runFailures() == 0;
    IOLog("NativeEmmc: probe %u blocks of userdata in runs of %u blocks: %s, crc32 0x%08x (%s), %llu us = %llu KiB/s; "
          "next range %s %llu us = %llu KiB/s, %u failed runs\n",
          want, chunk, ranOk ? "read" : "FAILED", crcC, crcC == crcA ? "same as single blocks" : "MISMATCH", ull(usC), rate(usC),
          coldOk ? "read" : "skipped", ull(usD), rate(usD), host.runFailures());
    (void)crcD;
    host.setMultiBlock(keep);
    native_emmc_multi_block = keep ? 1 : 0;
    IOLog("NativeEmmc: multi-block reads %s\n", keep ? "ON" : "OFF");

    // Then the same blocks through ADMA2: the CRC has to agree with the single-block reads, every block has to
    // have come by DMA, and nothing may have failed on the way; otherwise DMA stays off.
    if (keep && bootArg("emmc-dma", 1)) {
      const emmc::DmaWindow *window = dmaWindow();
      if (window && host.setDma(window)) {
        const uint64_t dmaBefore = host.stats.dmaBlocks;
        uint32_t crcE = 0, crcF = 0;
        uint64_t usE = 0, usF = 0;
        bool dmaRead = pass(userdataFirst, crcE, usE);
        bool dmaCold = userdataFirst + 3ull * want <= userdataLast + 1 && pass(userdataFirst + 2ull * want, crcF, usF);
        bool good = dmaRead && crcE == crcA && host.dmaFailures() == 0 && host.stats.dmaBlocks >= dmaBefore + want;
        IOLog("NativeEmmc: probe %u blocks of userdata by DMA: %s, crc32 0x%08x (%s), %llu us = %llu KiB/s; "
              "another range %s %llu us = %llu KiB/s, %u failed DMA runs, %llu blocks by DMA\n",
              want, dmaRead ? "read" : "FAILED", crcE, crcE == crcA ? "same as single blocks" : "MISMATCH", ull(usE), rate(usE),
              dmaCold ? "read" : "skipped", ull(usF), rate(usF), host.dmaFailures(), ull(host.stats.dmaBlocks - dmaBefore));
        (void)crcF;
        if (!good) host.setDma(nullptr);
        native_emmc_dma = good ? 1 : 0;
        IOLog("NativeEmmc: DMA reads %s\n", good ? "ON" : "OFF");
      } else {
        IOLog("NativeEmmc: DMA reads OFF (no window, or the controller is in 64-bit addressing mode)\n");
      }
    }
  }
  IOFreeData(buffer, chunk * emmc::kBlockBytes);
}

// The Data RAM disk on a thread of its own. Without emmc-data-persist it is started first thing; with it, the
// persistent publisher decides, and this is what runs instead whenever that one cannot.
static void startDataRamThread() {
  if (!bootArg("emmc-data-ram", 1)) return;
  thread_t thread = nullptr;
  if (kernel_thread_start(dataDiskPublisher, nullptr, &thread) == KERN_SUCCESS) {
    thread_deallocate(thread);
  } else {
    IOLog("NativeEmmc: data-ram: cannot start the thread\n");
  }
}

bool NativeEmmc::start(IOService *provider) {
  if (!IOService::start(provider)) return false;
  // The watchdog has nothing to do with the card; it lives here because this is the one built-in
  // driver of the board, and it must run even when the card does not come up.
  if (uint32_t kick = bootArg("wdt-kick", 0)) {
    thread_t thread = nullptr;
    if (kernel_thread_start(wdtKicker, reinterpret_cast<void *>(static_cast<uintptr_t>(kick)), &thread) == KERN_SUCCESS) {
      thread_deallocate(thread);
    } else {
      IOLog("NativeEmmc: wdt-kick: cannot start the thread\n");
    }
  }
  // Same reason: the power-manager nub that AMFI waits for has nothing to do with the card either.
  if (bootArg("pmgr-nub", 1)) {
    thread_t thread = nullptr;
    if (kernel_thread_start(pmgrNubPublisher, nullptr, &thread) == KERN_SUCCESS) {
      thread_deallocate(thread);
    } else {
      IOLog("NativeEmmc: pmgr-nub: cannot start the thread\n");
    }
  }
  // And the Data RAM disk: the payload's RAMDisk as a writable block device, so that Apple's APFS can
  // publish the Data volume as /dev/diskNsM for the kernel's mount on /private/var. With emmc-data-persist=1
  // the volume is a window of the card instead (persistentData), and this only runs if that cannot, from
  // wherever start() gives up as well.
  const uint32_t mode = bootArg("emmc", 1);
  const bool persist = mode == 4 && bootArg("emmc-data-persist", 0) != 0;
  struct DataFallback {
    bool armed;
    ~DataFallback() {
      if (armed) startDataRamThread();
    }
  } dataFallback = {persist};
  if (!persist) startDataRamThread();
  if (mode == 0) {
    IOLog("NativeEmmc: disabled by boot-arg emmc=0\n");
    return false;
  }
  mapping = provider->mapDeviceMemoryWithIndex(0, kIOMapAnywhere | kIOMapInhibitCache);
  if (!mapping || mapping->getLength() < 0x1000) {
    IOLog("NativeEmmc: cannot map the controller registers\n");
    return false;
  }
  IOLog("NativeEmmc: %s regs phys 0x%llx length 0x%llx\n", provider->getName(), ull(mapping->getPhysicalAddress()),
        ull(mapping->getLength()));
  host.attach(&kPlatform, reinterpret_cast<void *>(static_cast<uintptr_t>(mapping->getVirtualAddress())));
  gate = IOLockAlloc();
  if (!gate) return false;

  host.dump("firmware state");
  int result = host.identify();
  if (result != emmc::kOk) {
    IOLog("NativeEmmc: the card is not usable (%s)\n", emmc::resultName(result));
    host.dump("after failed identify");
    return false;
  }
  IOLockLock(gate);
  bool gpt = scanGpt();
  rootReady = false;
  if (gpt) {
    probeUserdata();
    if (mode == 4) rootReady = findRoot();
  }
  IOLockUnlock(gate);
  const emmc::Stats &stats = host.stats;
  IOLog("NativeEmmc: %llu blocks read, %llu commands, %llu retries, %llu recoveries, %llu failures, %llu host resets, %llu us\n",
        ull(stats.blocks), ull(stats.commands), ull(stats.retries), ull(stats.recoveries), ull(stats.failures),
        ull(stats.hostResets), ull(stats.readUs));
  if (!gpt) return false;
  native_emmc_ready = 1;
  registerService();

  bool published = false;
  if (mode == 2) {
    published = publishWindow("eMMC", 0, diskBlocks) != nullptr;
  } else if (mode == 3) {
    published = userdataLast ? publishWindow("userdata", userdataFirst, userdataLast - userdataFirst + 1) != nullptr : false;
  } else if (mode == 4 && rootReady) {
    // The windows go out from a thread of their own (see publishRoot); it keeps us alive until it is done.
    thread_t thread = nullptr;
    retain();
    if (kernel_thread_start(rootPublisher, this, &thread) == KERN_SUCCESS) {
      thread_deallocate(thread);
    } else {
      IOLog("NativeEmmc: cannot start the root publisher thread\n");
      release();
    }
    if (persist) {
      // The Data window on the card, next to the root windows (it formats first, if it has to, and it only
      // starts once the root image is known to end well in front of it).
      thread_t dataThread = nullptr;
      retain();
      if (kernel_thread_start(dataPublisher, this, &dataThread) == KERN_SUCCESS) {
        thread_deallocate(dataThread);
        dataFallback.armed = false;
      } else {
        IOLog("NativeEmmc: cannot start the Data publisher thread\n");
        release();
      }
    }
  }
  if (published) {
    thread_t thread = nullptr;
    if (kernel_thread_start(mediaReport, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
  }
  return true;
}

void NativeEmmc::stop(IOService *provider) { IOService::stop(provider); }

void NativeEmmc::free() {
  if (mapping) { mapping->release(); mapping = nullptr; }
  if (gate) { IOLockFree(gate); gate = nullptr; }
  IOService::free();
}
