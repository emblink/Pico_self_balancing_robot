#include "string.h"
#include "storage.h"
#include "hardware/flash.h"

#define STORAGE_MAGIC_NUMBER 0xC0FE1234

static StorageData localData = {0};

bool storageInit()
{
    storageRead(&localData);
    if (localData.magicNumber != STORAGE_MAGIC_NUMBER) {
        return false;
    }

    return true;
}

bool storageRead(StorageData* data)
{
    if (NULL == data) {
        return false;
    }

    if (localData.magicNumber == STORAGE_MAGIC_NUMBER) {
        *data = localData;
        return true;
    }

    uintptr_t xip_address = XIP_BASE + STORAGE_BASE_ADDRESS;
    memcpy(&localData, (const void*)xip_address, sizeof(StorageData));
    if (localData.magicNumber != STORAGE_MAGIC_NUMBER) {
        return false;
    }

    *data = localData;
    return true;
}

// This function will be called when it's safe to call flash_range_erase
static void call_flash_range_erase(void *param) {
    uint32_t offset = (uint32_t)param;
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
}

// This function will be called when it's safe to call flash_range_program
static void call_flash_range_program(void *param) {
    uint32_t offset = ((uintptr_t*)param)[0];
    const uint8_t *data = (const uint8_t *)((uintptr_t*)param)[1];
    flash_range_program(offset, data, FLASH_PAGE_SIZE);
}

bool storageWrite(const StorageData* data)
{
    if (NULL == data) {
        return false;
    }

    localData = *data;
    localData.magicNumber = STORAGE_MAGIC_NUMBER;

    // Flash is "execute in place" and so will be in use when any code that is stored in flash runs, e.g. an interrupt handler
    // or code running on a different core.
    // Calling flash_range_erase or flash_range_program at the same time as flash is running code would cause a crash.
    // flash_safe_execute disables interrupts and tries to cooperate with the other core to ensure flash is not in use
    // See the documentation for flash_safe_execute and its assumptions and limitations
    int rc = flash_safe_execute(call_flash_range_erase, (void*) STORAGE_BASE_ADDRESS, UINT32_MAX);
    hard_assert(rc == PICO_OK);

    uintptr_t params[] = { STORAGE_BASE_ADDRESS, (uintptr_t) &localData};
    rc = flash_safe_execute(call_flash_range_program, params, UINT32_MAX);
    hard_assert(rc == PICO_OK);

    return true;
}