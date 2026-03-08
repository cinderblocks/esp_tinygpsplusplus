/*
 * TinyGPS++ — compact NMEA-0183 sentence parser for ESP-IDF
 *
 * Original library by Mikal Hart (Copyright 2008-2024).
 * ESP-IDF port: Arduino dependency removed; uses esp_timer, <cmath>, and
 * the standard C library only.
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 */

#include "TinyGPSPlus.h"

#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cmath>

#define _RMCterm "RMC"
#define _GGAterm "GGA"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define _GPS_RADIANS(x) ((x) * (M_PI / 180.0))
#define _GPS_DEGREES(x) ((x) * (180.0 / M_PI))
#define _GPS_SQ(x)      ((x) * (x))
#define _GPS_TWO_PI     (2.0 * M_PI)

// millis() shim — used internally by commit() methods.
// _gps_millis() is defined in the header (inline, via esp_timer).
static inline uint32_t _millis() { return _gps_millis(); }

// ─────────────────────────────────────────────────────────────────────────
// TinyGPSPlus
// ─────────────────────────────────────────────────────────────────────────

TinyGPSPlus::TinyGPSPlus()
    : parity(0)
    , isChecksumTerm(false)
    , curSentenceType(GPS_SENTENCE_OTHER)
    , curTermNumber(0)
    , curTermOffset(0)
    , sentenceHasFix(false)
    , customElts(nullptr)
    , customCandidates(nullptr)
    , encodedCharCount(0)
    , sentencesWithFixCount(0)
    , failedChecksumCount(0)
    , passedChecksumCount(0)
{
    term[0] = '\0';
}

bool TinyGPSPlus::encode(char c)
{
    ++encodedCharCount;

    switch (c) {
    case ',':
        parity ^= (uint8_t)c;
        /* fall through */
    case '\r':
    case '\n':
    case '*':
    {
        bool isValidSentence = false;
        if (curTermOffset < sizeof(term)) {
            term[curTermOffset] = '\0';
            isValidSentence = endOfTermHandler();
        }
        ++curTermNumber;
        curTermOffset = 0;
        isChecksumTerm = (c == '*');
        return isValidSentence;
    }
    case '$':
        curTermNumber = curTermOffset = 0;
        parity = 0;
        curSentenceType = GPS_SENTENCE_OTHER;
        isChecksumTerm = false;
        sentenceHasFix = false;
        return false;
    default:
        if (curTermOffset < sizeof(term) - 1)
            term[curTermOffset++] = c;
        if (!isChecksumTerm)
            parity ^= (uint8_t)c;
        return false;
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Internal utilities
// ─────────────────────────────────────────────────────────────────────────

int TinyGPSPlus::fromHex(char a)
{
    if (a >= 'A' && a <= 'F') return a - 'A' + 10;
    if (a >= 'a' && a <= 'f') return a - 'a' + 10;
    return a - '0';
}

int32_t TinyGPSPlus::parseDecimal(const char *term)
{
    bool negative = (*term == '-');
    if (negative) ++term;
    int32_t ret = 100 * (int32_t)atol(term);
    while (isdigit((unsigned char)*term)) ++term;
    if (*term == '.' && isdigit((unsigned char)term[1])) {
        ret += 10 * (term[1] - '0');
        if (isdigit((unsigned char)term[2]))
            ret += term[2] - '0';
    }
    return negative ? -ret : ret;
}

void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg)
{
    uint32_t leftOfDecimal = (uint32_t)atol(term);
    uint16_t minutes       = (uint16_t)(leftOfDecimal % 100);
    uint32_t multiplier    = 10000000UL;
    uint32_t tenMillionthsOfMinutes = minutes * multiplier;

    deg.deg = (uint16_t)(leftOfDecimal / 100);
    while (isdigit((unsigned char)*term)) ++term;
    if (*term == '.') {
        while (isdigit((unsigned char)*++term)) {
            multiplier /= 10;
            tenMillionthsOfMinutes += (*term - '0') * multiplier;
        }
    }
    deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
    deg.negative = false;
}

#define COMBINE(sentence_type, term_number) \
    (((unsigned)(sentence_type) << 5) | (term_number))

bool TinyGPSPlus::endOfTermHandler()
{
    if (isChecksumTerm) {
        uint8_t checksum = (uint8_t)(16 * fromHex(term[0]) + fromHex(term[1]));
        if (checksum == parity) {
            ++passedChecksumCount;
            if (sentenceHasFix) ++sentencesWithFixCount;

            switch (curSentenceType) {
            case GPS_SENTENCE_RMC:
                date.commit();
                time.commit();
                if (sentenceHasFix) {
                    location.commit();
                    speed.commit();
                    course.commit();
                }
                break;
            case GPS_SENTENCE_GGA:
                time.commit();
                if (sentenceHasFix) {
                    location.commit();
                    altitude.commit();
                }
                satellites.commit();
                hdop.commit();
                break;
            default: break;
            }

            for (TinyGPSCustom *p = customCandidates;
                 p != nullptr &&
                 strcmp(p->sentenceName, customCandidates->sentenceName) == 0;
                 p = p->next)
            {
                p->commit();
            }
            return true;
        }
        ++failedChecksumCount;
        return false;
    }

    if (curTermNumber == 0) {
        if (term[0] == 'G' && strchr("PNABL", term[1]) != nullptr) {
            if      (!strcmp(term + 2, _RMCterm)) curSentenceType = GPS_SENTENCE_RMC;
            else if (!strcmp(term + 2, _GGAterm)) curSentenceType = GPS_SENTENCE_GGA;
            else                                  curSentenceType = GPS_SENTENCE_OTHER;
        } else {
            curSentenceType = GPS_SENTENCE_OTHER;
        }

        for (customCandidates = customElts;
             customCandidates != nullptr &&
             strcmp(customCandidates->sentenceName, term) < 0;
             customCandidates = customCandidates->next) {}

        if (customCandidates != nullptr &&
            strcmp(customCandidates->sentenceName, term) > 0)
        {
            customCandidates = nullptr;
        }
        return false;
    }

    if (curSentenceType != GPS_SENTENCE_OTHER && term[0]) {
        switch (COMBINE(curSentenceType, curTermNumber)) {
        case COMBINE(GPS_SENTENCE_RMC, 1):
        case COMBINE(GPS_SENTENCE_GGA, 1):  time.setTime(term);                               break;
        case COMBINE(GPS_SENTENCE_RMC, 2):  sentenceHasFix = (term[0] == 'A');                break;
        case COMBINE(GPS_SENTENCE_RMC, 3):
        case COMBINE(GPS_SENTENCE_GGA, 2):  location.setLatitude(term);                       break;
        case COMBINE(GPS_SENTENCE_RMC, 4):
        case COMBINE(GPS_SENTENCE_GGA, 3):  location.rawNewLatData.negative = (term[0]=='S'); break;
        case COMBINE(GPS_SENTENCE_RMC, 5):
        case COMBINE(GPS_SENTENCE_GGA, 4):  location.setLongitude(term);                      break;
        case COMBINE(GPS_SENTENCE_RMC, 6):
        case COMBINE(GPS_SENTENCE_GGA, 5):  location.rawNewLngData.negative = (term[0]=='W'); break;
        case COMBINE(GPS_SENTENCE_RMC, 7):  speed.set(term);                                  break;
        case COMBINE(GPS_SENTENCE_RMC, 8):  course.set(term);                                 break;
        case COMBINE(GPS_SENTENCE_RMC, 9):  date.setDate(term);                               break;
        case COMBINE(GPS_SENTENCE_GGA, 6):
            sentenceHasFix = (term[0] > '0');
            location.newFixQuality = (TinyGPSLocation::Quality)term[0];
            break;
        case COMBINE(GPS_SENTENCE_GGA, 7):  satellites.set(term);                             break;
        case COMBINE(GPS_SENTENCE_GGA, 8):  hdop.set(term);                                   break;
        case COMBINE(GPS_SENTENCE_GGA, 9):  altitude.set(term);                               break;
        case COMBINE(GPS_SENTENCE_RMC, 12):
            location.newFixMode = (TinyGPSLocation::Mode)term[0];
            break;
        }
    }

    for (TinyGPSCustom *p = customCandidates;
         p != nullptr &&
         strcmp(p->sentenceName, customCandidates->sentenceName) == 0 &&
         p->termNumber <= (int)curTermNumber;
         p = p->next)
    {
        if (p->termNumber == (int)curTermNumber) p->set(term);
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────
// Static utilities
// ─────────────────────────────────────────────────────────────────────────

double TinyGPSPlus::distanceBetween(double lat1, double lon1,
                                    double lat2, double lon2)
{
    double delta  = _GPS_RADIANS(lon1 - lon2);
    double sdlong = sin(delta), cdlong = cos(delta);
    lat1 = _GPS_RADIANS(lat1); lat2 = _GPS_RADIANS(lat2);
    double slat1 = sin(lat1), clat1 = cos(lat1);
    double slat2 = sin(lat2), clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = _GPS_SQ(delta) + _GPS_SQ(clat2 * sdlong);
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * _GPS_EARTH_MEAN_RADIUS;
}

double TinyGPSPlus::courseTo(double lat1, double lon1,
                             double lat2, double lon2)
{
    double dlon = _GPS_RADIANS(lon2 - lon1);
    lat1 = _GPS_RADIANS(lat1); lat2 = _GPS_RADIANS(lat2);
    double a1 = sin(dlon) * cos(lat2);
    double a2 = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    a2 = atan2(a1, a2);
    if (a2 < 0.0) a2 += _GPS_TWO_PI;
    return _GPS_DEGREES(a2);
}

const char *TinyGPSPlus::cardinal(double course)
{
    static const char *directions[] = {
        "N","NNE","NE","ENE","E","ESE","SE","SSE",
        "S","SSW","SW","WSW","W","WNW","NW","NNW"
    };
    return directions[(int)((course + 11.25) / 22.5) % 16];
}

// ─────────────────────────────────────────────────────────────────────────
// TinyGPSLocation
// ─────────────────────────────────────────────────────────────────────────

void TinyGPSLocation::commit()
{
    rawLatData = rawNewLatData; rawLngData = rawNewLngData;
    fixQuality = newFixQuality; fixMode    = newFixMode;
    lastCommitTime = _millis();
    valid = updated = true;
}
void TinyGPSLocation::setLatitude(const char *t)  { TinyGPSPlus::parseDegrees(t, rawNewLatData); }
void TinyGPSLocation::setLongitude(const char *t) { TinyGPSPlus::parseDegrees(t, rawNewLngData); }

double TinyGPSLocation::lat()
{
    updated = false;
    double v = rawLatData.deg + rawLatData.billionths / 1000000000.0;
    return rawLatData.negative ? -v : v;
}
double TinyGPSLocation::lng()
{
    updated = false;
    double v = rawLngData.deg + rawLngData.billionths / 1000000000.0;
    return rawLngData.negative ? -v : v;
}

// ─────────────────────────────────────────────────────────────────────────
// TinyGPSDate
// ─────────────────────────────────────────────────────────────────────────

void TinyGPSDate::commit()      { date = newDate; lastCommitTime = _millis(); valid = updated = true; }
void TinyGPSDate::setDate(const char *t) { newDate = (uint32_t)atol(t); }
uint16_t TinyGPSDate::year()  { updated = false; return (uint16_t)(date % 100) + 2000; }
uint8_t  TinyGPSDate::month() { updated = false; return (uint8_t)((date / 100) % 100); }
uint8_t  TinyGPSDate::day()   { updated = false; return (uint8_t)(date / 10000); }

// ─────────────────────────────────────────────────────────────────────────
// TinyGPSTime
// ─────────────────────────────────────────────────────────────────────────

void TinyGPSTime::commit()     { time = newTime; lastCommitTime = _millis(); valid = updated = true; }
void TinyGPSTime::setTime(const char *t) { newTime = (uint32_t)TinyGPSPlus::parseDecimal(t); }
uint8_t TinyGPSTime::hour()        { updated = false; return (uint8_t)(time / 1000000); }
uint8_t TinyGPSTime::minute()      { updated = false; return (uint8_t)((time / 10000) % 100); }
uint8_t TinyGPSTime::second()      { updated = false; return (uint8_t)((time / 100) % 100); }
uint8_t TinyGPSTime::centisecond() { updated = false; return (uint8_t)(time % 100); }

// ─────────────────────────────────────────────────────────────────────────
// TinyGPSDecimal / TinyGPSInteger
// ─────────────────────────────────────────────────────────────────────────

void TinyGPSDecimal::commit()  { val = newval; lastCommitTime = _millis(); valid = updated = true; }
void TinyGPSDecimal::set(const char *t) { newval = TinyGPSPlus::parseDecimal(t); }

void TinyGPSInteger::commit()  { val = newval; lastCommitTime = _millis(); valid = updated = true; }
void TinyGPSInteger::set(const char *t) { newval = (uint32_t)atol(t); }

// ─────────────────────────────────────────────────────────────────────────
// TinyGPSCustom
// ─────────────────────────────────────────────────────────────────────────

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *sn, int tn)
{
    begin(gps, sn, tn);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *sn, int tn)
{
    lastCommitTime = 0;
    updated = valid = false;
    sentenceName = sn;
    termNumber   = tn;
    memset(stagingBuffer, '\0', sizeof(stagingBuffer));
    memset(buffer,        '\0', sizeof(buffer));
    gps.insertCustom(this, sn, tn);
}

void TinyGPSCustom::commit()
{
    strcpy(buffer, stagingBuffer);
    lastCommitTime = _millis();
    valid = updated = true;
}

void TinyGPSCustom::set(const char *term)
{
    strncpy(stagingBuffer, term, sizeof(stagingBuffer) - 1);
    stagingBuffer[sizeof(stagingBuffer) - 1] = '\0';
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sn, int tn)
{
    TinyGPSCustom **pp;
    for (pp = &customElts; *pp != nullptr; pp = &(*pp)->next) {
        int cmp = strcmp(sn, (*pp)->sentenceName);
        if (cmp < 0 || (cmp == 0 && tn < (*pp)->termNumber)) break;
    }
    pElt->next = *pp;
    *pp = pElt;
}
