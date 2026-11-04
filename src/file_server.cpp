#include "file_server.h"
#include "sd_card.h"
#include "config.h"
#include <Arduino.h>

#ifdef HAVE_OTA_SECRET
#include "ota_secret.h"  // generated into .pio/build by scripts/ota_password.py
#endif
#ifdef OTA_PASSWORD
#include <Updater.h>
#endif

// Port of hms-cpap-fysetc/main/file_server.c
// ezShare-compatible endpoints: /dir, /download, /api/status
//
// STRICTLY READ-ONLY: every SD open below uses O_RDONLY.
//
// SHORT BUS SESSIONS: the SD bus is shared with the CPAP without a mux, so it
// is held only while reading from the card and is released (pins tri-stated)
// BEFORE every network write. /download reads FILE_CHUNK_BYTES per session,
// /dir one output buffer's worth of entries. Each new session waits for the
// CPAP to be idle again (sd_card_mount() -> blockout + CS level check). The
// open FatFile objects live in RAM across sessions; closing a read-only file
// causes no card traffic.
//
// FAILED RESPONSES ARE NEVER MADE TO LOOK COMPLETE: if a card error, a bus
// timeout or a short read happens after the 200/206 header went out, the
// connection is closed without the final chunk / without the announced
// Content-Length, so the client sees an incomplete response.

static ESP8266WebServer *s_server = nullptr;

#define BUS_WAIT_MS  10000  // max wait for the CPAP to go idle mid-response

// ---------------------------------------------------------------------------
// Last SD failure, shown in /api/status (the card usually has no serial
// console attached while it sits in the CPAP).
// ---------------------------------------------------------------------------
static char s_last_err[112] = "";
static uint16_t s_err_count = 0;

static void note_error(const char *what, const String &path, const char *reason, uint8_t ec) {
    s_err_count++;
    int n = snprintf(s_last_err, sizeof(s_last_err), "t=%lus %s /%s: %s (sdErr=0x%02X)",
                     millis() / 1000, what, path.c_str(), reason, ec);
    if (n < 0) { s_last_err[0] = '\0'; return; }
    // Keep it JSON-safe: no quotes, backslashes or control characters.
    for (char *p = s_last_err; *p; p++) {
        if (*p == '"' || *p == '\\' || (uint8_t)*p < 0x20) *p = '?';
    }
}

// ---------------------------------------------------------------------------
// Path canonicalisation
//
// Accepts ezShare forms ("A:", "A:\\DATALOG", "A:%5CDATALOG", "A:/DATALOG",
// "/DATALOG", "DATALOG/", ...) and produces "a/b/c" with no leading or
// trailing slash. "" means the root directory. Empty and "." components are
// dropped; a ".." component is rejected (returns false).
// ---------------------------------------------------------------------------
static bool canonical_path(String &p) {
    if (p.length() >= 2 && (p[0] == 'A' || p[0] == 'a') && p[1] == ':') {
        p.remove(0, 2);
    }
    // arg() already URL-decodes once; this handles double-encoded clients.
    p.replace("%5C", "/");
    p.replace("%5c", "/");
    p.replace('\\', '/');

    String out;
    out.reserve(p.length());
    unsigned int start = 0;
    while (start <= p.length()) {
        int slash = p.indexOf('/', start);
        unsigned int end = slash < 0 ? p.length() : (unsigned int)slash;
        unsigned int len = end - start;
        if (len == 0 || (len == 1 && p[start] == '.')) {
            // skip empty / "." component
        } else if (len == 2 && p[start] == '.' && p[start + 1] == '.') {
            return false;
        } else {
            if (out.length()) out += '/';
            out += p.substring(start, end);
        }
        if (slash < 0) break;
        start = end + 1;
    }
    p = out;
    return true;
}

// ---------------------------------------------------------------------------
// Mount the card or send an error response (bus untouched on failure).
// ---------------------------------------------------------------------------
static bool mount_or_respond(const char *what, const String &path) {
    if (sd_card_mount()) return true;
    uint32_t wait_ms = sd_card_is_initialized() ? 0 : sd_card_init_wait_ms();
    if (wait_ms) {
        // An init failed recently: no new attempt before the pause is over.
        char ra[12];
        snprintf(ra, sizeof(ra), "%lu", (unsigned long)(wait_ms / 1000 + 1));
        note_error(what, path, "sd_not_ready (init pause)", 0);
        s_server->sendHeader(F("Retry-After"), ra);
        s_server->send_P(503, PSTR("text/plain"),
                         PSTR("sd_not_ready: card init failed, next attempt after Retry-After seconds"));
    } else if (!sd_card_bus_available()) {
        note_error(what, path, "sd_busy", 0);
        s_server->sendHeader(F("Retry-After"), F("3"));
        s_server->send_P(503, PSTR("text/plain"), PSTR("sd_busy: CPAP is using the card, retry"));
    } else {
        note_error(what, path, "sd_mount_failed", sd_card_get().sdErrorCode());
        s_server->send_P(500, PSTR("text/plain"), PSTR("sd_mount_failed"));
    }
    return false;
}

// Mid-response: wait (bus untouched, yielding) until the CPAP is idle again,
// then start a new bus session. False on timeout or client disconnect.
static bool remount_wait(uint32_t max_ms) {
    uint32_t t0 = millis();
    while (!sd_card_mount(false)) {  // never (re-)initialise from here
        if (!sd_card_is_initialized()) return false;
        if (!s_server->client().connected()) return false;
        if ((uint32_t)(millis() - t0) >= max_ms) return false;
        delay(5);
    }
    return true;
}

// Releases the bus, then reports a failed open as 503 (card I/O error) or 404.
static void respond_open_failure(const char *what, const String &path) {
    SdFat &sd = sd_card_get();
    uint32_t ed = sd.sdErrorData();
    uint8_t ec = sd_card_unmount();  // bus released before any network I/O

    char hdr[64];
    if (ec) {
        snprintf(hdr, sizeof(hdr), "sd_io_error code=0x%02X data=0x%02lX: /",
                 ec, (unsigned long)ed);
    } else {
        snprintf(hdr, sizeof(hdr), "%s not found: /", what);
    }
    Serial.printf("[file_srv] open '/%s' failed: %s\n", path.c_str(), hdr);
    if (ec) note_error(what, path, "open failed", ec);
    String msg(hdr);
    msg += path;
    s_server->send(ec ? 503 : 404, "text/plain", msg);
}

// ---------------------------------------------------------------------------
// Buffered chunked output for the listing (no large Strings in RAM).
// out_flush() ends the current bus session before sending.
//
// The 200 header is DEFERRED until the first flush: a failure while the first
// buffer is still being filled (e.g. the first read after the CPAP touched the
// card) can then still be retried or answered with a proper 503 instead of an
// empty "200 OK" page. 2 KB covers most Löwenstein directories completely.
// ---------------------------------------------------------------------------
static char s_out[2048];
static size_t s_out_len = 0;
static uint8_t s_sess_err = 0;  // first card error of any session of this request
static bool s_hdr_sent = false;

static void end_session() {
    uint8_t e = sd_card_unmount();  // no-op (0) if not mounted
    if (e && !s_sess_err) s_sess_err = e;
}

static void out_flush() {
    end_session();  // never hold the bus during network I/O
    if (!s_hdr_sent) {
        s_server->setContentLength(CONTENT_LENGTH_UNKNOWN);
        s_server->send(200, "text/html", "");
        s_hdr_sent = true;
    }
    if (s_out_len) {
        s_server->sendContent(s_out, s_out_len);
        s_out_len = 0;
    }
}

static void out_write(const char *s, size_t n) {
    while (n) {
        size_t room = sizeof(s_out) - s_out_len;
        size_t k = n < room ? n : room;
        memcpy(s_out + s_out_len, s, k);
        s_out_len += k;
        s += k;
        n -= k;
        if (s_out_len == sizeof(s_out)) out_flush();
    }
}

static void out_str(const char *s) { out_write(s, strlen(s)); }

static void out_char(char c) { out_write(&c, 1); }

// Percent-encode for use inside a query value. Unreserved characters, '/'
// and ':' pass through; everything else (space, '+', '&', '#', '%', UTF-8
// bytes, ...) becomes %XX. The result never needs HTML escaping.
static void out_url(const char *s) {
    static const char hex[] = "0123456789ABCDEF";
    for (; *s; s++) {
        uint8_t c = (uint8_t)*s;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' ||
            c == '~' || c == '/' || c == ':') {
            out_char((char)c);
        } else {
            char e[3] = {'%', hex[c >> 4], hex[c & 0x0F]};
            out_write(e, 3);
        }
    }
}

static void out_html(const char *s) {
    for (; *s; s++) {
        switch (*s) {
            case '&': out_str("&amp;"); break;
            case '<': out_str("&lt;"); break;
            case '>': out_str("&gt;"); break;
            case '"': out_str("&quot;"); break;
            default:  out_char(*s); break;
        }
    }
}

// ---------------------------------------------------------------------------
// GET /dir?dir=A:[/|\|%5C]<path>
//
// Returns ezShare-style HTML directory listing. One line per entry:
//   YYYY-MM-DD   HH:MM:SS         &lt;DIR&gt;   <a href="dir?dir=A:/<path>"> name</a>
//   YYYY-MM-DD   HH:MM:SS       NNNNNKB  <a href="download?file=<path>"> name</a>
// "dir=A:", "A:/", "A:\", "/" or an empty value list the root directory.
// A listing that could not be read completely is cut off (no closing tags,
// no terminating chunk) instead of being sent as a valid-looking 200.
// A failure before anything was sent is retried once from the start; if that
// fails too, the client gets a 503 (with Retry-After) instead of an empty 200.
// ---------------------------------------------------------------------------
static char s_name[256];  // LFN (UTF-8); longer names fall back to the 8.3 alias

static bool open_dir(SdFat &sd, FatFile &dir, const String &path) {
    bool ok = path.length() == 0 ? dir.openRoot(&sd)
                                 : dir.open(&sd, path.c_str(), O_RDONLY);
    if (ok && !dir.isDir()) {
        dir.close();
        ok = false;
    }
    return ok;
}

// Reads all entries of an open directory into the output buffer (flushing as
// it fills). Returns nullptr on success, else the failure reason.
static const char *list_entries(SdFat &sd, FatFile &dir, const String &path,
                                uint16_t &count, uint16_t &skipped) {
    FatFile entry;
    while (true) {
        yield();  // every iteration, including skipped entries
        if (!s_server->client().connected()) return "client gone";
        if (!sd_card_is_mounted() && !remount_wait(BUS_WAIT_MS)) {
            return s_server->client().connected() ? "SD bus busy (timeout)" : "client gone";
        }
        if (s_sess_err) return "SD I/O error";

        // --- all card access for this entry happens here (bus held) ---
        if (!entry.openNext(&dir, O_RDONLY)) {
            if (dir.getError() || sd.sdErrorCode()) return "SD read error";
            return nullptr;  // end of directory
        }
        size_t nlen = entry.getName(s_name, sizeof(s_name));
        bool sfn_fallback = false;
        if (nlen == 0 && !sd.sdErrorCode()) {
            // LFN longer than the buffer: use the 8.3 alias, which SdFat also
            // resolves in open(), so the link still works.
            nlen = entry.getSFN(s_name, sizeof(s_name));
            sfn_fallback = nlen != 0;
        }
        uint16_t fdate = 0, ftime = 0;
        bool have_dt = entry.getModifyDateTime(&fdate, &ftime);
        bool is_dir = entry.isDir();
        uint64_t fsize = entry.fileSize();
        entry.close();  // read-only: no write
        if (sd.sdErrorCode() || dir.getError()) return "SD read error";
        // --- end of card access; output below may release the bus ---

        if (nlen == 0 || strcmp(s_name, ".") == 0 || strcmp(s_name, "..") == 0) {
            if (nlen == 0) skipped++;
            continue;
        }
        if (sfn_fallback) {
            Serial.printf("[file_srv] long name in /%s listed as 8.3 '%s'\n",
                          path.c_str(), s_name);
        }

        if (!have_dt || FS_MONTH(fdate) < 1 || FS_MONTH(fdate) > 12 || FS_DAY(fdate) < 1) {
            fdate = FS_DATE(1980, 1, 1);  // valid placeholder for "no timestamp"
            ftime = 0;
        }
        char head[64];
        int hl = snprintf(head, sizeof(head), "%4u-%2u-%2u   %2u:%02u:%02u",
                          FS_YEAR(fdate), FS_MONTH(fdate), FS_DAY(fdate),
                          FS_HOUR(ftime), FS_MINUTE(ftime), FS_SECOND(ftime));
        out_write(head, hl);

        if (is_dir) {
            out_str("         &lt;DIR&gt;   <a href=\"dir?dir=A:/");
        } else {
            unsigned long size_kb = (unsigned long)((fsize + 1023) / 1024);
            if (size_kb < 1) size_kb = 1;
            hl = snprintf(head, sizeof(head), "       %5luKB  <a href=\"download?file=", size_kb);
            out_write(head, hl);
        }

        if (path.length()) {
            out_url(path.c_str());
            out_char('/');
        }
        out_url(s_name);
        out_str("\"> ");
        out_html(s_name);
        out_str("</a>\r\n");
        count++;
    }
}

static const char DIR_HEAD[] = "<html><body><pre>\r\n";

static void handle_dir() {
    String path = s_server->arg("dir");
    if (!canonical_path(path)) {
        s_server->send_P(400, PSTR("text/plain"), PSTR("invalid path"));
        return;
    }

    if (!mount_or_respond("dir", path)) return;

    SdFat &sd = sd_card_get();
    FatFile dir;
    if (!open_dir(sd, dir, path)) {
        respond_open_failure("directory", path);
        return;
    }

    s_hdr_sent = false;
    uint16_t count = 0, skipped = 0;
    const char *fail = nullptr;
    uint8_t fail_ec = 0;

    for (uint8_t attempt = 0;; attempt++) {
        s_sess_err = 0;
        s_out_len = 0;
        count = skipped = 0;
        out_str(DIR_HEAD);  // buffered only; the header is still deferred

        fail = list_entries(sd, dir, path, count, skipped);
        dir.close();    // read-only; card is idle, so no card traffic even unmounted
        if (!fail && sd_card_is_mounted()) end_session();
        if (!fail && s_sess_err) fail = "SD I/O error";
        if (!fail) break;

        fail_ec = s_sess_err ? s_sess_err : sd.sdErrorCode();
        end_session();
        if (s_hdr_sent || attempt >= 1 || !s_server->client().connected()) break;

        // Nothing sent yet: start over once with a fresh bus session.
        Serial.printf("[file_srv] /dir /%s: %s (sdErr=0x%02X) before output, retrying\n",
                      path.c_str(), fail, fail_ec);
        note_error("dir (retried)", path, fail, fail_ec);
        if (!remount_wait(BUS_WAIT_MS)) {
            fail = s_server->client().connected() ? "SD bus busy (timeout)" : "client gone";
            break;
        }
        if (!open_dir(sd, dir, path)) {
            fail = "reopen failed";
            fail_ec = sd.sdErrorCode();
            end_session();
            break;
        }
    }
    end_session();  // release the bus (if still held) before the final writes

    if (skipped) {
        Serial.printf("[file_srv] /dir /%s: %u entries without readable name skipped\n",
                      path.c_str(), skipped);
    }
    if (fail) {
        Serial.printf("[file_srv] GET /dir /%s FAILED after %u entries: %s (sdErr=0x%02X)%s\n",
                      path.c_str(), count, fail, fail_ec, s_hdr_sent ? ", cut off" : "");
        note_error("dir", path, fail, fail_ec);
        s_out_len = 0;
        if (s_hdr_sent) {
            s_server->client().stop();  // truncated chunked body -> client error
        } else if (s_server->client().connected()) {
            // Nothing sent yet: a proper error page. Busy -> the browser
            // reloads by itself after 5 s; I/O errors are not retried
            // automatically (no endless polling of the card).
            bool busy = strstr(fail, "busy") != nullptr;
            String msg = F("<html><head>");
            if (busy) msg += F("<meta http-equiv=\"refresh\" content=\"5\">");
            msg += F("</head><body><pre>");
            msg += busy ? F("SD card busy (CPAP is using it). Retrying in 5 s ...")
                        : F("SD card read error. Please reload in a few seconds.");
            msg += F("\r\n\r\n(");
            msg += fail;
            msg += F(")</pre></body></html>");
            s_server->sendHeader(F("Retry-After"), busy ? F("5") : F("10"));
            s_server->send(503, "text/html", msg);
        }
        return;
    }

    out_str("</pre></body></html>");
    out_flush();                // sends the deferred header if not done yet
    s_server->sendContent("");  // End chunked

    Serial.printf("[file_srv] GET /dir /%s entries=%u\n", path.c_str(), count);
}

// ---------------------------------------------------------------------------
// GET /download?file=[A:][/|\|%5C]<path>
//
// Streams raw file bytes. Each FILE_CHUNK_BYTES chunk is read in its own short
// bus session; the bus is released before the chunk is sent over WiFi.
// Supports "Range: bytes=N-" for delta downloads / resuming after an abort.
// ---------------------------------------------------------------------------
static uint8_t s_dl_buf[FILE_CHUNK_BYTES];

static void handle_download() {
    String path = s_server->arg("file");
    if (!canonical_path(path)) {
        s_server->send_P(400, PSTR("text/plain"), PSTR("invalid path"));
        return;
    }
    if (path.length() == 0) {
        s_server->send_P(400, PSTR("text/plain"), PSTR("missing file param"));
        return;
    }

    uint32_t offset = 0;
    if (s_server->hasHeader("Range")) {
        String range = s_server->header("Range");
        int eq = range.indexOf("bytes=");
        if (eq >= 0) offset = strtoul(range.c_str() + eq + 6, nullptr, 10);
    }

    if (!mount_or_respond("download", path)) return;

    SdFat &sd = sd_card_get();
    FatFile file;
    if (!file.open(&sd, path.c_str(), O_RDONLY)) {
        respond_open_failure("file", path);
        return;
    }
    if (!file.isFile()) {
        file.close();
        sd_card_unmount();
        s_server->send_P(404, PSTR("text/plain"), PSTR("not a file"));
        return;
    }

    uint32_t file_size = file.fileSize();
    if (offset > file_size) {
        file.close();
        sd_card_unmount();
        char cr[32];
        snprintf(cr, sizeof(cr), "bytes */%lu", (unsigned long)file_size);
        s_server->sendHeader(F("Content-Range"), cr);
        s_server->send_P(416, PSTR("text/plain"), PSTR("range not satisfiable"));
        return;
    }
    if (offset > 0 && !file.seekSet(offset)) {
        uint8_t ec = sd.sdErrorCode();
        file.close();
        sd_card_unmount();
        Serial.printf("[file_srv] seek /%s to %lu failed (sdErr=0x%02X)\n",
                      path.c_str(), (unsigned long)offset, ec);
        s_server->send_P(503, PSTR("text/plain"), PSTR("sd_io_error: seek failed"));
        return;
    }
    uint32_t remaining = file_size - offset;

    // Read the first chunk BEFORE the header goes out, so an error right at
    // the start becomes a proper 503 instead of an empty/short 200.
    int pre = 0;
    if (remaining > 0) {
        uint32_t want = remaining < sizeof(s_dl_buf) ? remaining : sizeof(s_dl_buf);
        pre = file.read(s_dl_buf, want);
    }
    uint8_t ec = sd.sdErrorCode();
    uint8_t uec = sd_card_unmount();  // release the bus before the header goes out
    if (!ec) ec = uec;
    if (ec || (remaining > 0 && pre <= 0)) {
        file.close();
        Serial.printf("[file_srv] /download /%s: sd error 0x%02X at start\n",
                      path.c_str(), ec);
        note_error("download", path, "read failed at start", ec);
        s_server->sendHeader(F("Retry-After"), F("10"));
        s_server->send_P(503, PSTR("text/plain"), PSTR("sd_io_error: please retry"));
        return;
    }

    s_server->setContentLength(remaining);
    if (offset > 0) {
        if (remaining > 0) {
            char cr[48];
            snprintf(cr, sizeof(cr), "bytes %lu-%lu/%lu", (unsigned long)offset,
                     (unsigned long)(file_size - 1), (unsigned long)file_size);
            s_server->sendHeader(F("Content-Range"), cr);
        }
        s_server->send(206, "application/octet-stream", "");
    } else {
        s_server->send(200, "application/octet-stream", "");
    }

    uint32_t total = 0;
    WiFiClient &client = s_server->client();
    const char *fail = nullptr;

    if (pre > 0) {
        if (client.write(s_dl_buf, pre) != (size_t)pre) fail = "client gone";
        else total = pre;
    }

    // Bounded by the size announced in Content-Length, even if the CPAP
    // appends to the file while we stream it.
    while (!fail && total < remaining) {
        if (!remount_wait(BUS_WAIT_MS)) {
            fail = client.connected() ? "SD bus busy (timeout)" : "client gone";
            break;
        }
        uint32_t want = remaining - total;
        if (want > sizeof(s_dl_buf)) want = sizeof(s_dl_buf);
        int n = file.read(s_dl_buf, want);
        ec = sd_card_unmount();  // bus released before client.write
        if (n <= 0 || ec) {
            fail = "SD read error";
            break;
        }
        if (client.write(s_dl_buf, n) != (size_t)n) {
            fail = "client gone";
            break;
        }
        total += n;
        yield();  // Feed watchdog
    }
    file.close();  // read-only; card is idle -> no card traffic

    if (fail) {
        // Short body vs. Content-Length: close now so the client fails fast
        // (and can resume with Range) instead of waiting on keep-alive.
        Serial.printf("[file_srv] GET /download /%s ABORTED at %lu: %s (sdErr=0x%02X)\n",
                      path.c_str(), (unsigned long)(offset + total), fail, ec);
        if (strcmp(fail, "client gone") != 0) note_error("download", path, fail, ec);
        client.stop();
        return;
    }

    Serial.printf("[file_srv] GET /download /%s offset=%lu bytes=%lu/%lu\n",
                  path.c_str(), (unsigned long)offset, (unsigned long)total,
                  (unsigned long)remaining);
}

// ---------------------------------------------------------------------------
// GET /api/status
// ---------------------------------------------------------------------------
static void handle_status() {
    char buf[384];
    unsigned long up = millis() / 1000;
    int secs = up % 60, mins = (up / 60) % 60, hrs = up / 3600;

    snprintf(buf, sizeof(buf),
             "{\"state\":\"FILE_SERVER\",\"wifi\":%s,"
             "\"free_heap\":%u,"
             "\"uptime\":\"%dh%02dm%02ds\","
             "\"fw_version\":\"%s\","
             "\"sd_errors\":%u,"
             "\"sd_inits\":%u,\"sd_init_fails\":%u,"
             "\"last_error\":\"%s\"}",
             WiFi.isConnected() ? "true" : "false",
             ESP.getFreeHeap(),
             hrs, mins, secs,
             APP_VERSION,
             s_err_count, sd_card_init_count(), sd_card_init_fail_count(), s_last_err);

    s_server->send(200, "application/json", buf);
}

// ---------------------------------------------------------------------------
// /update — firmware upload (internal flash only, never touches the SD bus)
//
// Small replacement for ESP8266HTTPUpdateServer:
//  - basic auth checked at upload start AND in the final handler (no cached
//    "authenticated" flag that a later plain POST could reuse),
//  - CSRF guard: if the browser sent Origin/Referer it must match the Host the
//    request was addressed to (curl sends neither),
//  - no CORS headers, no OPTIONS handler, no filesystem update.
// ---------------------------------------------------------------------------
#ifdef OTA_PASSWORD
static bool s_ota_enabled = false;

enum : uint8_t { OTA_IDLE, OTA_RUNNING, OTA_DONE, OTA_FAILED, OTA_REJECTED };
static uint8_t s_ota_state = OTA_IDLE;

static const char OTA_FORM[] PROGMEM =
    "<html><body><h3>Firmware update</h3>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' accept='.bin,.bin.gz' name='firmware'> "
    "<input type='submit' value='Update firmware'></form></body></html>";

static bool ota_auth() {
    return s_server->authenticate("admin", OTA_PASSWORD);
}

static bool ota_same_origin() {
    String self = F("http://");
    self += s_server->hostHeader();
    const String &origin = s_server->header("Origin");
    if (origin.length() && origin != self) return false;
    const String &ref = s_server->header("Referer");
    if (ref.length() && ref != self && !ref.startsWith(self + '/')) return false;
    return true;
}

static void handle_update_form() {
    if (!ota_auth()) return s_server->requestAuthentication();
    s_server->send_P(200, PSTR("text/html"), OTA_FORM);
}

static void handle_update_upload() {
    HTTPUpload &up = s_server->upload();
    switch (up.status) {
        case UPLOAD_FILE_START:
            if (!ota_auth() || !ota_same_origin()) {
                if (Update.isRunning()) Update.end();  // unfinished -> discarded
                s_ota_state = OTA_REJECTED;
                Serial.println(F("[ota] upload rejected (auth/origin)"));
                return;
            }
            if (up.name != "firmware") return;  // ignore other form parts
            if (Update.isRunning()) Update.end();
            {
                uint32_t max_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
                s_ota_state = Update.begin(max_space, U_FLASH) ? OTA_RUNNING : OTA_FAILED;
            }
            Serial.printf("[ota] upload '%s' %s\n", up.filename.c_str(),
                          s_ota_state == OTA_RUNNING ? "started" : "begin failed");
            break;
        case UPLOAD_FILE_WRITE:
            if (s_ota_state == OTA_RUNNING &&
                Update.write(up.buf, up.currentSize) != up.currentSize) {
                s_ota_state = OTA_FAILED;
            }
            break;
        case UPLOAD_FILE_END:
            if (s_ota_state == OTA_RUNNING) {
                s_ota_state = Update.end(true) ? OTA_DONE : OTA_FAILED;
                Serial.printf("[ota] upload end: %u bytes, %s\n", (unsigned)up.totalSize,
                              s_ota_state == OTA_DONE ? "OK" : "FAILED");
            }
            break;
        case UPLOAD_FILE_ABORTED:
            if (Update.isRunning()) Update.end();
            s_ota_state = OTA_FAILED;
            Serial.println(F("[ota] upload aborted"));
            break;
    }
    yield();
}

static void handle_update_done() {
    uint8_t st = s_ota_state;
    s_ota_state = OTA_IDLE;  // never reuse state from an earlier request
    if (!ota_auth()) return s_server->requestAuthentication();
    if (st == OTA_REJECTED || !ota_same_origin()) {
        s_server->send_P(403, PSTR("text/plain"), PSTR("forbidden"));
        return;
    }
    if (st != OTA_DONE) {
        if (Update.isRunning()) Update.end();
        String msg = F("Update failed: ");
        if (st == OTA_IDLE) msg += F("no 'firmware' file part");
        else msg += Update.getErrorString();
        s_server->send(500, "text/plain", msg);
        return;
    }
    s_server->client().setNoDelay(true);
    s_server->send_P(200, PSTR("text/html"),
                     PSTR("<META http-equiv=\"refresh\" content=\"15;URL=/\">"
                          "Update Success! Rebooting..."));
    delay(100);
    s_server->client().stop();
    ESP.restart();
}
#endif  // OTA_PASSWORD

// ---------------------------------------------------------------------------
// GET / — landing page (no SD access)
// ---------------------------------------------------------------------------
static const char ROOT_HTML_HEAD[] PROGMEM =
    "<html><body>"
    "<h2>CpapDash Firmware Lite " APP_VERSION "</h2>"
    "<p><a href=\"/dir?dir=A:/\">Browse SD card</a></p>"
    "<p><a href=\"/api/status\">Status</a></p>";
static const char ROOT_HTML_OTA[] PROGMEM =
    "<p><a href=\"/update\">Firmware update</a></p>";
static const char ROOT_HTML_TAIL[] PROGMEM = "</body></html>";

static void handle_root() {
    s_server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    s_server->send(200, "text/html", "");
    s_server->sendContent_P(ROOT_HTML_HEAD);
#ifdef OTA_PASSWORD
    if (s_ota_enabled) s_server->sendContent_P(ROOT_HTML_OTA);
#endif
    s_server->sendContent_P(ROOT_HTML_TAIL);
    s_server->sendContent("");
}

void file_server_init(ESP8266WebServer &server) {
    s_server = &server;
    server.on("/", HTTP_GET, handle_root);
    server.on("/dir", HTTP_GET, handle_dir);
    server.on("/download", HTTP_GET, handle_download);
    server.on("/api/status", HTTP_GET, handle_status);
    server.collectHeaders("Range", "Origin", "Referer");
    Serial.println(F("[file_srv] Registered: / /dir /download /api/status"));

    // HTTP firmware update (flash only, never touches the SD bus).
    // Fail closed: only registered when a password of >= 8 chars was baked in
    // at build time (see scripts/ota_password.py).
#ifdef OTA_PASSWORD
    if (sizeof(OTA_PASSWORD) - 1 >= 8) {
        server.on("/update", HTTP_GET, handle_update_form);
        server.on("/update", HTTP_POST, handle_update_done, handle_update_upload);
        s_ota_enabled = true;
        Serial.println(F("[file_srv] Registered: /update (basic auth, user admin)"));
    } else {
        Serial.println(F("[file_srv] OTA password too short - /update disabled"));
    }
#else
    (void)ROOT_HTML_OTA;
    Serial.println(F("[file_srv] No OTA password at build time - /update disabled"));
#endif
}
