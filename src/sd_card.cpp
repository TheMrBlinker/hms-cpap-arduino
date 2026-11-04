#include "sd_card.h"
#include "config.h"
#include <Arduino.h>
#include <SPI.h>

// Bus sharing via CS_SENSE interrupt + tri-state pin control.
// First mount does full sd.begin() (CMD0..., once per ESP boot).
// Subsequent mounts re-take the SPI pins and drop SdFat's sector cache
// (fast, <1ms, no card commands). See sd_card_mount().
// READ-ONLY: nothing in this firmware may open a file for writing.
//
// Card state we must NOT change behind the CPAP's back:
//  - no CMD59 (USE_SD_CRC stays 0): CRC checking would stay enabled inside the
//    card and make it reject the CPAP's commands/blocks if its driver sends
//    dummy CRCs.
//  - CMD0 + init only once per boot (stock behaviour). An automatic re-init
//    after I/O errors is compiled out unless SD_REINIT_ON_ERROR=1.

#if defined(USE_SD_CRC) && USE_SD_CRC
#error "USE_SD_CRC must stay 0: CMD59 changes card state shared with the CPAP"
#endif

#ifndef SD_REINIT_ON_ERROR
#define SD_REINIT_ON_ERROR 0
#endif

static SdFat sd;
static bool s_mounted = false;
static bool s_card_initialized = false;
static volatile bool s_we_have_bus = false;
// Time of the last CS edge caused by the CPAP (both directions). Compared with
// unsigned subtraction, so it is safe across the 49.7-day millis() wrap.
static volatile uint32_t s_last_cs_edge = 0;
// Counts every CS edge (ours too); used to detect an edge while claiming.
static volatile uint32_t s_cs_edges = 0;

#define SPI_BLOCKOUT_MS  3000

#if SD_REINIT_ON_ERROR
#define REINIT_AFTER_ERR_SESSIONS  3
#define REINIT_MIN_INTERVAL_MS     (5UL * 60UL * 1000UL)
#define REINIT_QUIET_MS            10000
static uint8_t  s_err_sessions = 0;
static uint32_t s_last_init_ms = 0;
static bool     s_reinit_pending = false;
#endif

static IRAM_ATTR void cs_sense_isr() {
    s_cs_edges++;
    if (!s_we_have_bus) {
        s_last_cs_edge = millis();
    }
}

void sd_card_init() {
    pinMode(MISO_PIN, INPUT);
    pinMode(MOSI_PIN, INPUT);
    pinMode(SCLK_PIN, INPUT);
    pinMode(SD_CS_PIN, INPUT);
    pinMode(CS_SENSE_PIN, INPUT_PULLUP);
    s_last_cs_edge = millis();  // require a full quiet period after boot
    // CHANGE: a deselect (rising edge) also restarts the quiet period, so a
    // CPAP that holds CS low for a long transfer is not considered idle the
    // moment it releases.
    attachInterrupt(digitalPinToInterrupt(CS_SENSE_PIN), cs_sense_isr, CHANGE);
    Serial.println(F("[sd] Init — pins tri-stated, CS_SENSE armed"));
}

// CPAP idle = CS deselected (high) now AND no CPAP CS edge for quiet_ms.
static bool bus_idle(uint32_t quiet_ms) {
    return digitalRead(CS_SENSE_PIN) == HIGH &&
           (uint32_t)(millis() - s_last_cs_edge) >= quiet_ms;
}

static void release_bus() {
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(MISO_PIN, INPUT);
    pinMode(MOSI_PIN, INPUT);
    pinMode(SCLK_PIN, INPUT);
    pinMode(SD_CS_PIN, INPUT);
    s_we_have_bus = false;
}

// Atomically check idle and mark the bus as ours, then switch the pins.
// Re-checks after switching: a CPAP edge in that window aborts the claim.
static bool claim_bus(uint32_t quiet_ms) {
    noInterrupts();
    bool idle = bus_idle(quiet_ms);
    if (idle) s_we_have_bus = true;
    uint32_t edges = s_cs_edges;
    interrupts();
    if (!idle) return false;

    digitalWrite(SD_CS_PIN, HIGH);  // latch HIGH first: no low glitch on CS
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(MISO_PIN, SPECIAL);
    pinMode(MOSI_PIN, SPECIAL);
    pinMode(SCLK_PIN, SPECIAL);

    if (s_cs_edges != edges || digitalRead(CS_SENSE_PIN) != HIGH) {
        release_bus();
        s_last_cs_edge = millis();
        return false;
    }
    return true;
}

bool sd_card_take_bus() {
    return claim_bus(SPI_BLOCKOUT_MS);
}

void sd_card_release_bus() {
    release_bus();
}

bool sd_card_bus_available() {
    return bus_idle(SPI_BLOCKOUT_MS);
}

// Freshness strategy (read-only, shared bus, no mux):
//
// The CPAP writes to the card between our requests, so anything SdFat cached
// in a previous session may be stale. There are two ways to fix that:
//   a) re-run sd.begin() on every mount: CMD0/CMD8/ACMD41/CMD58 plus MBR/BPB
//      reads on EVERY request. Lots of extra traffic on a bus the CPAP owns,
//      and CMD0 resets whatever state the CPAP left the card in.
//   b) keep the card initialised and drop SdFat's sector cache on every mount.
// We use (b): sd.cacheClear() is RAM-only (it would only write if the cache
// were dirty, and in this firmware nothing ever dirties it -- all opens are
// O_RDONLY). With USE_SEPARATE_FAT_CACHE == 0 (ESP8266) this single 512-byte
// cache holds both directory and FAT sectors, so after clearing it every
// directory entry, file size and cluster chain is re-read from the card. The
// volume geometry kept from begin() (BPB, FAT location, cluster size) does not
// change unless the card is reformatted (which needs the board to be removed
// from the CPAP, i.e. an ESP reboot).
//
// SdFat's error code is sticky (only begin() clears it), so it is reset in RAM
// at the start of every session; sd_card_unmount() then reports whether THIS
// session hit a card-level error. After an error we do NOT re-initialise the
// card (that would send CMD0 right after CPAP activity); the next session just
// retries the read. Optional, off by default: SD_REINIT_ON_ERROR=1.
bool sd_card_mount() {
    if (s_mounted) return true;

    if (!s_card_initialized) {
        // First mount after boot: full init (CMD0 ...), once.
        if (!claim_bus(SPI_BLOCKOUT_MS)) return false;
        unsigned long t0 = millis();
        bool ok = sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SPI_SPEED));
        if (!ok) {
            Serial.printf("[sd] Card init failed (err=0x%02X data=0x%02X), retry\n",
                          sd.sdErrorCode(), (unsigned)sd.sdErrorData());
            sd.card()->syncDevice();
            release_bus();          // never wait while holding the bus
            delay(100);
            if (!claim_bus(SPI_BLOCKOUT_MS)) return false;  // CPAP became active
            ok = sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SPI_SPEED));
        }
        if (!ok) {
            Serial.printf("[sd] Mount failed (err=0x%02X data=0x%02X)\n",
                          sd.sdErrorCode(), (unsigned)sd.sdErrorData());
            sd.card()->syncDevice();
            release_bus();
            return false;
        }
        s_card_initialized = true;
#if SD_REINIT_ON_ERROR
        s_last_init_ms = millis();
        s_err_sessions = 0;
        s_reinit_pending = false;
#endif
        Serial.printf("[sd] Card init: %lums\n", millis() - t0);
        s_mounted = true;
        return true;
    }

#if SD_REINIT_ON_ERROR
    if (s_reinit_pending && bus_idle(REINIT_QUIET_MS)) {
        // Only after repeated failed sessions, rate-limited, after a long
        // quiet period. Falls back to a normal remount if not quiet enough.
        s_card_initialized = false;
        s_reinit_pending = false;
        Serial.println(F("[sd] Re-initialising card after repeated I/O errors"));
        return sd_card_mount();
    }
#endif

    if (!claim_bus(SPI_BLOCKOUT_MS)) return false;
    // Remount: card stays initialised; forget cached sectors so the current
    // on-card state is read (see comment above). No bus traffic.
    sd.cacheClear();
    sd.card()->error(SD_CARD_ERROR_NONE);  // RAM only: per-session error state
    s_mounted = true;
    return true;
}

uint8_t sd_card_unmount() {
    if (!s_mounted) return 0;
    // A read that failed mid multi-block transfer can leave the card in
    // READ_STATE (still streaming). Send CMD12 in that case so the card is
    // idle before the CPAP gets the bus back. No-op (no bus traffic) when idle.
    sd.card()->syncDevice();
    uint8_t err = sd.sdErrorCode();
    if (err) {
        Serial.printf("[sd] I/O error 0x%02X data=0x%02X this session\n",
                      err, (unsigned)sd.sdErrorData());
    }
#if SD_REINIT_ON_ERROR
    if (err) {
        if (s_err_sessions < 255) s_err_sessions++;
        if (s_err_sessions >= REINIT_AFTER_ERR_SESSIONS &&
            (uint32_t)(millis() - s_last_init_ms) >= REINIT_MIN_INTERVAL_MS) {
            s_reinit_pending = true;
        }
    } else {
        s_err_sessions = 0;
    }
#endif
    s_mounted = false;
    // Do NOT call SPI.end() -- keep SdFat's internal state.
    // Just tri-state the pins so the CPAP can use the bus.
    release_bus();
    return err;
}

bool sd_card_is_mounted() {
    return s_mounted;
}

SdFat &sd_card_get() {
    return sd;
}
