#pragma once
#include <SdFat.h>

// SPI SD card management (from Steigan sdControl)
// Handles SPI bus arbitration with external master (CPAP)

void sd_card_init();
// allow_init=false: never (re-)initialise the card (used by wait loops).
bool sd_card_mount(bool allow_init = true);
bool sd_card_is_initialized();
// ms until the next card init is allowed (0 = now). Only after an attempt.
uint32_t sd_card_init_wait_ms();
uint16_t sd_card_init_count();
uint16_t sd_card_init_fail_count();
// Releases the bus. Returns the SdFat card error code of this session (0 = ok).
uint8_t sd_card_unmount();
bool sd_card_is_mounted();
SdFat &sd_card_get();

// Bus control (for boards shared with a printer/CPAP)
bool sd_card_take_bus();
void sd_card_release_bus();
bool sd_card_bus_available();
