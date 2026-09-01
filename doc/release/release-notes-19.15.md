# v19.15.0

> This is a working draft for the up-coming 19.15.0 release.

We are pleased to announce the release of TT System Firmware version 19.15.0 🥳🎉.

Major enhancements with this release include:

## What's Changed

## Blackhole

### Power Management

- Add `TT_SMC_MSG_SET_BOARD_POWER_LIMIT`, allowing the host to temporarily lower the total-board input-power limit and restore the cable- and board-specific default. Firmware rejects limits outside the controller's supported range or above the detected hardware maximum and reports the active value through `TAG_BOARD_POWER_LIMIT`.

## Migration guide

An overview of required and recommended changes to make when migrating from the previous v19.14.0 release can be found in [19.15 Migration Guide](https://github.com/tenstorrent/tt-system-firmware/tree/main/doc/release/migration-guide-19.15.md).

## Full ChangeLog

The full ChangeLog from the previous v19.14.0 release can be found at the link below.

https://github.com/tenstorrent/tt-system-firmware/compare/v19.14.0...v19.15.0
