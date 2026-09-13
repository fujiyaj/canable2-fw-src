//
// flash_store: bus_config_t + motor_config_t[MOTOR_COUNT] persistence in
// the last 2K page of this board's 128K flash. See inc/flash_store.h for
// the on-flash layout and why the linker script reserves this page.
//

#include <stddef.h>
#include <string.h>

#include "stm32g4xx_hal.h"
#include "crc32.h"
#include "system.h"
#include "flash_store.h"

#define FLASH_STORE_MAGIC   0x43523039u // arbitrary sentinel ("is this even our data"), not a checksum
#define FLASH_STORE_VERSION 2u          // bump on any layout change (v1 was single-motor config_flash_t;
                                         // v2 is bus_config_t + motor_config_t[MOTOR_COUNT] -- an old v1
                                         // image is correctly detected as a version mismatch and discarded,
                                         // falling back to compiled-in defaults, not misinterpreted)

#define FLASH_STORE_PAGE 63u              // last page of 128K @ 2K/page
#define FLASH_STORE_ADDR 0x0801F800u      // = 0x08000000 + 63*0x800 -- must match STM32G431CBTx_FLASH.ld's shrunk FLASH region

typedef struct
{
    uint32_t magic;
    uint32_t version;
    bus_config_t bus;
    motor_config_t motor[MOTOR_COUNT];
    uint32_t crc32;
} flash_store_t;

// STM32G4 flash is programmed 8 bytes (double-word) at a time.
#define STORE_RAW_SIZE (((uint32_t)sizeof(flash_store_t) + 7u) & ~7u)


bool flash_store_load(bus_config_t *bus_out, motor_config_t motor_out[MOTOR_COUNT])
{
    flash_store_t store;
    // Flash is memory-mapped for reads -- no HAL_FLASH_Unlock() needed.
    memcpy(&store, (const void *)FLASH_STORE_ADDR, sizeof(store));

    if (store.magic != FLASH_STORE_MAGIC || store.version != FLASH_STORE_VERSION)
        return false;

    uint32_t crc = crc32_ieee((const uint8_t *)&store, offsetof(flash_store_t, crc32));
    if (crc != store.crc32)
        return false;

    *bus_out = store.bus;
    memcpy(motor_out, store.motor, sizeof(store.motor));
    return true;
}

bool flash_store_save(const bus_config_t *bus, const motor_config_t motor[MOTOR_COUNT])
{
    flash_store_t store;
    memset(&store, 0, sizeof(store));
    store.magic = FLASH_STORE_MAGIC;
    store.version = FLASH_STORE_VERSION;
    store.bus = *bus;
    memcpy(store.motor, motor, sizeof(store.motor));
    store.crc32 = crc32_ieee((const uint8_t *)&store, offsetof(flash_store_t, crc32));

    uint8_t buf[STORE_RAW_SIZE];
    memset(buf, 0, sizeof(buf)); // pad bytes past sizeof(store); not covered by the CRC, content irrelevant
    memcpy(buf, &store, sizeof(store));

    bool ok = true;

    // Global IRQ disable for the whole erase+program sequence: this is a
    // single-bank part, so flash code fetches stall automatically while an
    // erase/program is in flight regardless -- disabling interrupts here
    // just keeps CAN/USB ISRs (which would stall anyway) from being left
    // half-preempted across that stall, matching how this codebase already
    // treats other IRQ-sensitive critical sections (see cdc_transmit()).
    // Bounded to tens of milliseconds; only ever called while no motor's
    // control loop is running, so this is a one-off SAVE-time cost.
    system_irq_disable();

    if (HAL_FLASH_Unlock() != HAL_OK) {
        system_irq_enable();
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_1,
        .Page = FLASH_STORE_PAGE,
        .NbPages = 1,
    };
    uint32_t page_error = 0;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
        ok = false;

    if (ok) {
        for (uint32_t off = 0; off < STORE_RAW_SIZE; off += 8) {
            uint64_t dword;
            memcpy(&dword, buf + off, sizeof(dword));
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_STORE_ADDR + off, dword) != HAL_OK) {
                ok = false;
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    system_irq_enable();

    return ok;
}
