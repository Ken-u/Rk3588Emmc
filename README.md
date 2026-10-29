# Rk3588Emmc

RK3588 eMMC host (Synopsys DesignWare MSHC, `rockchip,rk3588-dwcmshc`) and block
storage for UniXNU: polled SDHCI core with ADMA2 reads and writes, GPT parsing,
and a block device that publishes the root volumes and a persistent Data window.

The repository keeps the runtime class `NativeEmmc`, its source filename and its
IOKit personality (`Info.plist`).

| Path | Contents |
| --- | --- |
| `NativeEmmc.cpp` | IOKit service, block device, root and Data publication |
| `EmmcHost.h` | Controller core: reset, clocks, bus state, PIO/ADMA2 transfers, CMD13 programming wait |
| `Gpt.h` | GPT reader and window lookup |
| `RootManifest.h` | Reader for the `unixnu build userdata` manifest |
| `Info.plist` | Reference personality |
| `tests/` | Host tests against a model of the MSHC and an eMMC |

## Host checks

```sh
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I . tests/test-emmc-host.cpp -o /tmp/test-emmc-host && /tmp/test-emmc-host
g++ -std=c++17 -Wall -Wextra -Werror -O1 -I . tests/test-root-manifest.cpp -o /tmp/test-root-manifest && /tmp/test-root-manifest
```

## Use

`UniXNU-evb10` selects this repository as `package/Rk3588Emmc`. The board supplies
the `/arm-io/emmc` node, the GPT-derived window layout and the boot arguments below;
the kernel patch `rk3588-emmc-personality.patch` of `UniXNU-rk3588-common` registers
the personality. Boot arguments: `emmc-data-size`, `emmc-data-persist`,
`emmc-data-reset` (see `NativeEmmc.cpp`).

Writes are limited to the Data window at the tail of `userdata`; the driver never
writes elsewhere. Build and image commands: [UniXNU building](https://unixnu.com/building).
Device results are recorded in the `UniXNU-evb10` porting record.
