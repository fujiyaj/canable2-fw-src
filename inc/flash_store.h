#ifndef _FLASH_STORE_H
#define _FLASH_STORE_H

#include <stdbool.h>

#include "params.h" // bus_config_t, motor_config_t, MOTOR_COUNT

// Persists bus_config_t + motor_config_t[MOTOR_COUNT] (PID gains, limits,
// motor/CAN config -- the PARAM groups documented as "persisted on save" in
// params.h) across resets, in the last 2KB page of this board's 128KB flash
// (page 63, 0x0801F800-0x0801FFFF). See STM32G431CBTx_FLASH.ld: the FLASH
// region there is deliberately shrunk to 126K so the linker itself
// guarantees code/rodata/data-load never encroaches on this page --
// flash_store.c hardcodes the address rather than relying on a
// linker-placed section, and the shrunk region is what makes that safe.
//
// On-flash layout (see flash_store.c):
//   u32 magic     a fixed sentinel, not a real checksum -- just "does this
//                 look like our data at all, or blank/garbage flash"
//   u32 version   bumped whenever this layout changes (including the
//                 single-motor -> MOTOR_COUNT-motor change, and any future
//                 MOTOR_COUNT change), so a stale image written by an
//                 older/differently-shaped build is detected and discarded
//                 -- defaults loaded -- instead of being reinterpreted
//                 under today's struct layout
//   bus_config_t bus
//   motor_config_t motor[MOTOR_COUNT]
//   u32 crc32     IEEE CRC-32 (crc32.h) over every byte above
// padded with zero bytes up to a multiple of 8 (STM32G4 flash is
// programmed as double-words) -- the padding is not covered by the CRC
// and its content doesn't matter.

// Reads and validates (magic + version + CRC-32) the stored image. On
// success, *bus_out and motor_out[0..MOTOR_COUNT) are filled in and this
// returns true. On any mismatch (blank/erased flash, corrupt image, or an
// old version from a previous firmware build with a different layout),
// neither output is touched and this returns false -- the caller
// (params_init()) is expected to fall back to compiled-in defaults.
bool flash_store_load(bus_config_t *bus_out, motor_config_t motor_out[MOTOR_COUNT]);

// Erases the config page and programs it with bus + motor[0..MOTOR_COUNT)
// (magic/version/crc32 computed here). Blocking -- a 2K page erase plus
// programming take on the order of tens of milliseconds, during which
// flash code fetches stall (single-bank part, no read-while-write);
// callers must only invoke this while no motor's control loop is running.
// The page is erased before the new image is programmed, so a failure
// partway through (rare -- a real HAL_FLASH error, not something normal
// use should trigger) leaves the page in a state that will fail
// flash_store_load()'s magic/version/CRC check, not a plausible-looking
// stale or torn image. Returns true only if every step succeeded.
bool flash_store_save(const bus_config_t *bus, const motor_config_t motor[MOTOR_COUNT]);

#endif // _FLASH_STORE_H
