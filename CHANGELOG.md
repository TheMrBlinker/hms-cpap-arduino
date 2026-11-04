# Changelog

## 2026.1.1-lw3 (2026-11-04)

### Fixed

- After the CPAP had been in use, every read failed (`sdErr=0x0C`, CMD18) until the ESP was rebooted: the CPAP re-initialises the card for itself, so the card no longer answers our SPI reads

### Changed

- `SD_REINIT_ON_ERROR=1` is now enabled, with stricter rules: the card is re-initialised only at the start of a user request, after 3 failed read sessions in a row, at least 5 min after the previous init attempt and after 10 s without CPAP bus activity
- A FAILED init is retried under the same rules (previously it was retried on every request after only 3 s of quiet); wait loops inside a request never (re-)initialise the card
- During the pause after a failed init, requests get `503 sd_not_ready` with `Retry-After`
- `/api/status` reports `sd_inits` and `sd_init_fails`

## 2026.1.1-lw2 (2026-11-04)

Fork for the Löwenstein prisma SMART max (folder layout `0040439131/YYYYMMDD/0001`, `Dcm`, `log`). Tested on a FYSETC SD-WiFi v2.1 inside the CPAP.

### Fixed

- `/dir`: listing the root directory (`dir=A:`, `A:/`, `A:\`, `A:%5C`) returned "directory not found"; any folder can now be browsed, links are percent-encoded and HTML-escaped, `..` is rejected
- `/dir`: the 200 header is sent only after the first 2 KB of the listing was read; a card error at the very start is retried once and otherwise answered with a 503 (busy page reloads itself after 5 s) instead of an empty 200 page
- `/download`: the first chunk is read before the header goes out, so an error at the start becomes a 503 instead of a short 200
- Incomplete listings/downloads are cut off instead of looking complete

### Changed

- SD bus is held only while reading (2 KB per session for downloads) and released before every WiFi write; a new session needs CS_SENSE high and 3 s without CPAP activity
- `USE_SD_CRC` removed (CMD59 would change card state shared with the CPAP) and guarded with `#error`; no automatic card re-init (CMD0) after errors
- Landing page links the SD root instead of `DATALOG`
- `/api/status` reports `sd_errors` and `last_error`

### Added

- `/update`: firmware update over WiFi (basic auth user `admin`, CSRF check). Enabled only when a password is set in the untracked `platformio_local.ini` (see `platformio_local.ini.example`) or `OTA_PASSWORD`; the password is not committed

## 2026.1.1 (2026-05-22)

Rebrand and cleanup release. No behavior change for File Server users.

### Changed

- Rebrand SleepLink → CpapDash across source, captive-portal AP name (`CpapDash-XXXX`), mDNS hostname (`cpapdash`), and device serial prefix (`CD-XXXXXXXX`)
- Default API URL updated from defunct `https://api.sleeplinkusa.com` to `https://api.cpapdash.com`
- Embedded firmware version aligned with repo `YYYY.ver.patch` convention (was `1.0.0`)
- Captive portal: removed the File Server / Cloud Push mode picker. Cloud Push is unviable on ESP8285 (BearSSL + WiFi stack exhausts the ~40 KB free heap during chunked HTTPS uploads — see [docs/ESP8285_CONSTRAINTS.md](docs/ESP8285_CONSTRAINTS.md)). File Server mode is now forced on every boot.

### Docs

- README rewritten to match `hms-fysetc` pattern (EXPERIMENTAL banner, badges, ASCII diagram, License section)
- Added MIT `LICENSE`
- Renamed `docs/SLEEPLINK_HARDWARE_OPTIONS.md` → `docs/HARDWARE_OPTIONS.md`

## 2026.1.0 (2026-04-06)

Initial release — ESP8285 dual-mode CPAP data bridge.

### Features

- **File Server mode (default):** ezShare-compatible HTTP server on port 80
  - `GET /dir?dir=A:/DATALOG` — directory listing
  - `GET /download?file=...` — file download with Range header support
  - `GET /api/status` — JSON device status
  - Chunked streaming at ~137 KB/s for multi-MB EDF files
- **Cloud Push mode:** autonomous upload to CpapDash API
  - Sync handshake (`POST /api/v1/data/sync`) — skip already-uploaded files
  - Chunked EDF upload (4 KB chunks) with retry + exponential backoff
  - STR.edf upload
  - Heartbeat with remote reset support
  - EEPROM-based file offset checkpoints
- **WiFiManager captive portal:** dark theme, WiFi scan dropdown
- **NTP time sync**
- **Boot button reset:** hold 5s to clear WiFi + config
- **Device serial:** auto-generated from chip ID (`CD-XXXXXXXX`)

### Hardware

- Board: FYSETC SD-WiFi v2.1 (ESP8285H16, 2 MB flash, 80 KB RAM)
- SD card: SPI mode via SdFat 2.2.2 (FAT16/FAT32, up to 32 GB)
- SPI pins: CS=4, MISO=12, MOSI=13, SCLK=14
- SHARED_SPI mode for reliable mount/unmount cycles

### Build

- PlatformIO + Arduino framework (espressif8266)
- RAM: 48% (39 KB), Flash: 47% (491 KB)
- Libraries: WiFiManager 2.0.17, ArduinoJson 6, ESP8266SdFat 2.2.2, NTPClient
