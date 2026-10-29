# Agent instructions

## Responsibility

Maintain the RK3588 eMMC host and block storage driver. Shared XNU patches live in
`UniXNU-rk3588-common`; board layout, partition windows and boot arguments live in
`UniXNU-evb10`.

## Required reading

1. This README and the [driver repositories](https://unixnu.com/drivers/repositories) page.
2. The [component catalog](https://unixnu.com/reference/components) and the selected SoC and OS profiles in UniXNU.
3. [Storage porting](https://unixnu.com/porting/ufs) for the root and Data publication contract.
4. [Device repository](https://unixnu.com/porting/device-repository) and [device tree](https://unixnu.com/porting/device-tree).
5. [Building](https://unixnu.com/building) and [bring-up](https://unixnu.com/porting/bring-up).

## Rules

- Keep the runtime class `NativeEmmc`, `NativeEmmc.cpp` and the personality name.
- Controller register facts in `EmmcHost.h` were measured on hardware. Do not change an access width, an ordering or a timeout without a host test and a device record.
- Writes stay inside the declared Data window. The host tests enforce the command set; keep them passing.
- Inline assembly in Mach-O targets uses `"\n\t"` between instructions; `;` starts a comment.
- Run the two host checks from the README before committing, and report the commands, results and source revisions. Report hardware observations separately from host results.
