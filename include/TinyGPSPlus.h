/*
 * TinyGPS++ — compact NMEA-0183 sentence parser for ESP-IDF
 *
 * Original library by Mikal Hart (Copyright 2008-2024).
 * ESP-IDF port: Arduino dependency removed; millis() replaced with
 * esp_timer_get_time() so the library builds without arduino-esp32.
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 */

#pragma once

#include <stdint.h>
#include <limits.h>
#include <esp_timer.h>   // esp_timer_get_time() → microseconds since boot

// ── Library version ───────────────────────────────────────────────────────
#define TINYGPSPLUS_VERSION "1.1.0"

// ── Unit-conversion constants ─────────────────────────────────────────────
#define _GPS_MPH_PER_KNOT      1.15077945
#define _GPS_MPS_PER_KNOT      0.51444444
#define _GPS_KMPH_PER_KNOT     1.852
#define _GPS_MILES_PER_METER   0.00062137112
#define _GPS_KM_PER_METER      0.001
#define _GPS_FEET_PER_METER    3.2808399
#define _GPS_EARTH_MEAN_RADIUS 6371009  ///< metres

// ── Internal constants ────────────────────────────────────────────────────
#define _GPS_MAX_FIELD_SIZE 15

// ── millis() shim ─────────────────────────────────────────────────────────
/// @brief Milliseconds since boot. Wraps after ~49 days (uint32_t rollover).
static inline uint32_t _gps_millis()
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief Raw latitude or longitude in degrees + fractional billionths.
 *
 * Stored this way to avoid accumulating floating-point error during parsing.
 * Convert to double via TinyGPSLocation::lat() / lng().
 */
struct RawDegrees
{
    uint16_t deg;        ///< Integer degrees (0–180)
    uint32_t billionths; ///< Fractional part in billionths of a degree
    bool     negative;   ///< true = South or West

    RawDegrees() : deg(0), billionths(0), negative(false) {}
};

// ── Forward declarations ──────────────────────────────────────────────────
class TinyGPSPlus;
class TinyGPSCustom;

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief Fix location (latitude, longitude, fix quality, positioning mode).
 *
 * Updated on every valid GGA or RMC sentence. Use isValid() to check
 * whether at least one fix has been received and age() to verify it is
 * recent.
 */
struct TinyGPSLocation
{
    friend class TinyGPSPlus;
public:
    /// GGA fix-quality field values (ASCII character codes).
    enum Quality { Invalid='0', GPS='1', DGPS='2', PPS='3',
                   RTK='4', FloatRTK='5', Estimated='6',
                   Manual='7', Simulated='8' };
    /// RMC positioning mode indicator.
    enum Mode { N='N', A='A', D='D', E='E' };

    bool     isValid()   const { return valid; }
    bool     isUpdated() const { return updated; }
    /** @return ms since last commit, or ULONG_MAX if never valid. */
    uint32_t age()       const { return valid ? _gps_millis() - lastCommitTime
                                               : (uint32_t)ULONG_MAX; }

    const RawDegrees &rawLat() { updated = false; return rawLatData; }
    const RawDegrees &rawLng() { updated = false; return rawLngData; }

    /** @return Signed decimal latitude in degrees (negative = South). */
    double lat();
    /** @return Signed decimal longitude in degrees (negative = West). */
    double lng();

    Quality FixQuality() { updated = false; return fixQuality; }
    Mode    FixMode()    { updated = false; return fixMode; }

    TinyGPSLocation()
        : valid(false), updated(false), fixQuality(Invalid), fixMode(N),
          lastCommitTime(0) {}

private:
    bool valid, updated;
    RawDegrees rawLatData, rawLngData, rawNewLatData, rawNewLngData;
    Quality fixQuality, newFixQuality;
    Mode fixMode, newFixMode;
    uint32_t lastCommitTime;
    void commit();
    void setLatitude(const char *term);
    void setLongitude(const char *term);
};

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief UTC date from the RMC sentence (packed as DDMMYY decimal).
 */
struct TinyGPSDate
{
    friend class TinyGPSPlus;
public:
    bool     isValid()   const { return valid; }
    bool     isUpdated() const { return updated; }
    uint32_t age()       const { return valid ? _gps_millis() - lastCommitTime
                                               : (uint32_t)ULONG_MAX; }

    uint32_t value()  { updated = false; return date; } ///< Raw DDMMYY
    uint16_t year();   ///< 4-digit year, e.g. 2025
    uint8_t  month();  ///< 1–12
    uint8_t  day();    ///< 1–31

    TinyGPSDate() : valid(false), updated(false), date(0), newDate(0),
                    lastCommitTime(0) {}

private:
    bool valid, updated;
    uint32_t date, newDate, lastCommitTime;
    void commit();
    void setDate(const char *term);
};

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief UTC time from GGA or RMC (packed as HHMMSSCC decimal centiseconds).
 */
struct TinyGPSTime
{
    friend class TinyGPSPlus;
public:
    bool     isValid()   const { return valid; }
    bool     isUpdated() const { return updated; }
    uint32_t age()       const { return valid ? _gps_millis() - lastCommitTime
                                               : (uint32_t)ULONG_MAX; }

    uint32_t value()        { updated = false; return time; }
    uint8_t  hour();        ///< 0–23
    uint8_t  minute();      ///< 0–59
    uint8_t  second();      ///< 0–59
    uint8_t  centisecond(); ///< 0–99

    TinyGPSTime() : valid(false), updated(false), time(0), newTime(0),
                    lastCommitTime(0) {}

private:
    bool valid, updated;
    uint32_t time, newTime, lastCommitTime;
    void commit();
    void setTime(const char *term);
};

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief Fixed-point signed decimal value stored ×100.
 *
 * Base class for TinyGPSSpeed, TinyGPSCourse, TinyGPSAltitude, TinyGPSHDOP.
 */
struct TinyGPSDecimal
{
    friend class TinyGPSPlus;
public:
    bool     isValid()   const { return valid; }
    bool     isUpdated() const { return updated; }
    uint32_t age()       const { return valid ? _gps_millis() - lastCommitTime
                                               : (uint32_t)ULONG_MAX; }
    int32_t  value()           { updated = false; return val; } ///< raw ×100

    TinyGPSDecimal() : valid(false), updated(false), lastCommitTime(0),
                       val(0), newval(0) {}

private:
    bool valid, updated;
    uint32_t lastCommitTime;
    int32_t val, newval;
    void commit();
    void set(const char *term);
};

// ─────────────────────────────────────────────────────────────────────────
/** @brief Fixed-point unsigned integer. Used for satellite count. */
struct TinyGPSInteger
{
    friend class TinyGPSPlus;
public:
    bool     isValid()   const { return valid; }
    bool     isUpdated() const { return updated; }
    uint32_t age()       const { return valid ? _gps_millis() - lastCommitTime
                                               : (uint32_t)ULONG_MAX; }
    uint32_t value()           { updated = false; return val; }

    TinyGPSInteger() : valid(false), updated(false), lastCommitTime(0),
                       val(0), newval(0) {}

private:
    bool valid, updated;
    uint32_t lastCommitTime;
    uint32_t val, newval;
    void commit();
    void set(const char *term);
};

// ─────────────────────────────────────────────────────────────────────────
/** @brief Ground speed in various units (base value in knots ×100). */
struct TinyGPSSpeed : TinyGPSDecimal
{
    double knots() { return value() / 100.0; }
    double mph()   { return _GPS_MPH_PER_KNOT  * value() / 100.0; }
    double mps()   { return _GPS_MPS_PER_KNOT  * value() / 100.0; }
    double kmph()  { return _GPS_KMPH_PER_KNOT * value() / 100.0; }
};

/** @brief True course over ground in degrees (value ×100). */
struct TinyGPSCourse : public TinyGPSDecimal
{
    double deg() { return value() / 100.0; }
};

/** @brief Altitude above MSL in various units (base value in metres ×100). */
struct TinyGPSAltitude : TinyGPSDecimal
{
    double meters()     { return value() / 100.0; }
    double miles()      { return _GPS_MILES_PER_METER * value() / 100.0; }
    double kilometers() { return _GPS_KM_PER_METER    * value() / 100.0; }
    double feet()       { return _GPS_FEET_PER_METER  * value() / 100.0; }
};

/** @brief Horizontal dilution of precision (value ×100). */
struct TinyGPSHDOP : TinyGPSDecimal
{
    double hdop() { return value() / 100.0; }
};

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief Bind to any arbitrary NMEA field by sentence name + term index.
 *
 * Allows extraction of any field from any sentence without modifying the
 * parser core. TinyGPSCustom objects are registered in a sorted linked list
 * inside TinyGPSPlus and notified on every matching sentence.
 *
 * @code
 * // Extract the true track from GNVTG (term 1)
 * TinyGPSCustom trueTrack(gps, "GNVTG", 1);
 * // … call gps.encode() …
 * if (trueTrack.isUpdated())
 *     ESP_LOGI("gps", "Track: %s°", trueTrack.value());
 * @endcode
 */
class TinyGPSCustom
{
public:
    TinyGPSCustom() : lastCommitTime(0), valid(false), updated(false),
                      sentenceName(nullptr), termNumber(0), next(nullptr)
    {
        stagingBuffer[0] = buffer[0] = '\0';
    }

    /**
     * @param gps          Parser instance to register with.
     * @param sentenceName NMEA sentence identifier, e.g. "GPGGA".
     * @param termNumber   1-based term index within the sentence.
     */
    TinyGPSCustom(TinyGPSPlus &gps, const char *sentenceName, int termNumber);
    void begin(TinyGPSPlus &gps, const char *sentenceName, int termNumber);

    bool        isUpdated() const { return updated; }
    bool        isValid()   const { return valid; }
    uint32_t    age()       const { return valid ? _gps_millis() - lastCommitTime
                                                 : (uint32_t)ULONG_MAX; }
    const char *value()           { updated = false; return buffer; }

private:
    void commit();
    void set(const char *term);

    char          stagingBuffer[_GPS_MAX_FIELD_SIZE + 1];
    char          buffer[_GPS_MAX_FIELD_SIZE + 1];
    uint32_t      lastCommitTime;
    bool          valid, updated;
    const char   *sentenceName;
    int           termNumber;
    friend class  TinyGPSPlus;
    TinyGPSCustom *next;
};

// ─────────────────────────────────────────────────────────────────────────
/**
 * @brief Compact NMEA-0183 parser for ESP-IDF.
 *
 * Feed characters one at a time from a GPS UART via encode(). The parser
 * updates the public member objects (location, date, time, …) on each
 * valid GGA or RMC sentence.
 *
 * ### Minimal usage
 * @code
 * #include <TinyGPSPlus.h>
 * #include <driver/uart.h>
 *
 * static TinyGPSPlus gps;
 *
 * void gps_task(void *)
 * {
 *     // configure UART1 via esp-idf …
 *     uint8_t b;
 *     while (true) {
 *         while (uart_read_bytes(UART_NUM_1, &b, 1, 0) == 1)
 *             gps.encode((char)b);
 *
 *         if (gps.location.isValid() && gps.location.age() < 3000)
 *             ESP_LOGI("gps", "%.5f, %.5f",
 *                      gps.location.lat(), gps.location.lng());
 *
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 * }
 * @endcode
 *
 * ### Diagnostics
 * - `charsProcessed() == 0` after 30 s → no UART data at all
 * - `passedChecksum() == 0` but chars > 0 → baud rate mismatch
 * - `failedChecksum() > 0` → noise or wrong baud rate
 * - `location.isValid() == false` but sentences passing → normal cold start
 */
class TinyGPSPlus
{
public:
    TinyGPSPlus();

    /**
     * @brief Feed one character from the GPS UART.
     * @return true when a complete, valid sentence has just been committed.
     */
    bool encode(char c);

    TinyGPSPlus &operator<<(char c) { encode(c); return *this; }

    // ── Parsed fields ─────────────────────────────────────────────────────
    TinyGPSLocation location;   ///< Lat/lng, fix quality, positioning mode
    TinyGPSDate     date;       ///< UTC date (RMC)
    TinyGPSTime     time;       ///< UTC time (GGA / RMC)
    TinyGPSSpeed    speed;      ///< Ground speed (RMC)
    TinyGPSCourse   course;     ///< True course over ground (RMC)
    TinyGPSAltitude altitude;   ///< MSL altitude in metres (GGA)
    TinyGPSInteger  satellites; ///< Satellites in use (GGA)
    TinyGPSHDOP     hdop;       ///< Horizontal dilution of precision (GGA)

    static const char *libraryVersion() { return TINYGPSPLUS_VERSION; }

    // ── Static utilities ──────────────────────────────────────────────────
    /** @brief Great-circle distance between two WGS-84 positions (metres, ≤0.5% error). */
    static double distanceBetween(double lat1, double lon1,
                                  double lat2, double lon2);
    /** @brief Initial bearing from pos1 to pos2 (degrees; 0=N, 90=E, 270=W). */
    static double courseTo(double lat1, double lon1,
                           double lat2, double lon2);
    /** @brief Convert bearing to 16-point compass label ("N", "NNE", … "NNW"). */
    static const char *cardinal(double course);

    // ── Internal helpers (public for TinyGPSCustom access) ───────────────
    static int32_t parseDecimal(const char *term);
    static void    parseDegrees(const char *term, RawDegrees &deg);

    // ── Diagnostic counters ───────────────────────────────────────────────
    uint32_t charsProcessed()   const { return encodedCharCount; }
    uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
    uint32_t failedChecksum()   const { return failedChecksumCount; }
    uint32_t passedChecksum()   const { return passedChecksumCount; }

private:
    enum { GPS_SENTENCE_GGA, GPS_SENTENCE_RMC, GPS_SENTENCE_OTHER };

    uint8_t parity;
    bool    isChecksumTerm;
    char    term[_GPS_MAX_FIELD_SIZE];
    uint8_t curSentenceType;
    uint8_t curTermNumber;
    uint8_t curTermOffset;
    bool    sentenceHasFix;

    friend class TinyGPSCustom;
    TinyGPSCustom *customElts;
    TinyGPSCustom *customCandidates;
    void insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int index);

    uint32_t encodedCharCount;
    uint32_t sentencesWithFixCount;
    uint32_t failedChecksumCount;
    uint32_t passedChecksumCount;

    int  fromHex(char a);
    bool endOfTermHandler();
};
