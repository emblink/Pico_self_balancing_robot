#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/flash.h"

#define STORAGE_DATA_SIZE    FLASH_SECTOR_SIZE // erase can be only performed in sector size increments
#define STORAGE_BASE_ADDRESS (2 * 1024 * 1024 - STORAGE_DATA_SIZE) // Store at the end of the 2Mb flash memory

typedef struct {
    float kp;
    float ki;
    float kd;
} PIDConfig;

typedef struct {
    uint32_t magicNumber;
    PIDConfig pidConfig;
} StorageData;


bool storageInit(void);
bool storageRead(StorageData* data);
bool storageWrite(const StorageData* data);

