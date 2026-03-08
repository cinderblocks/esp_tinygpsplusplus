# esp_tinygpsplusplus — ESP-IDF Component

Pure **ESP-IDF** port of [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)
(Mikal Hart, v1.1.0).  The Arduino dependency has been removed entirely:

- `millis()` → `esp_timer_get_time() / 1000`
- Arduino math macros (`radians`, `degrees`, `sq`, `TWO_PI`) → standard C++ `<cmath>`
- No `Arduino.h`, no `HardwareSerial`, no `String`

---

## Component layout

```
esp_tinygpsplusplus/
├── CMakeLists.txt        # idf_component_register
├── idf_component.yml     # IDF Component Manager manifest
├── include/
│   └── TinyGPSPlus.h     # Public API — the only header consumers need
├── src/
│   └── TinyGPSPlus.cpp   # Implementation
└── README.md
```

---

## Adding to a project

### As a local component (recommended for this repo)

The component lives in `components/esp_tinygpsplusplus/`.  Declare the
dependency in your `CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS    "my_source.cpp"
    REQUIRES esp_tinygpsplusplus esp_driver_uart
)
```

Then include the header:

```cpp
#include <TinyGPSPlus.h>
```

---

## Quick start

```cpp
#include <TinyGPSPlus.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "gps";

static constexpr uart_port_t GPS_UART   = UART_NUM_1;
static constexpr int         GPS_RX_PIN = 33;   // ESP32 RX ← module TX
static constexpr int         GPS_TX_PIN = 34;   // ESP32 TX → module RX

void gps_task(void *)
{
    const uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, 512, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    TinyGPSPlus gps;
    uint8_t b;

    while (true) {
        while (uart_read_bytes(GPS_UART, &b, 1, 0) == 1)
            gps.encode((char)b);

        if (gps.location.isValid() && gps.location.age() < 3000) {
            ESP_LOGI(TAG, "%.5f, %.5f  sats=%lu  HDOP=%.1f",
                     gps.location.lat(),
                     gps.location.lng(),
                     (unsigned long)gps.satellites.value(),
                     gps.hdop.hdop());
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## API reference

### `TinyGPSPlus` — main parser class

| Member | Type | Updated by | Description |
|---|---|---|---|
| `location` | `TinyGPSLocation` | GGA, RMC | Lat/lng, fix quality, mode |
| `date` | `TinyGPSDate` | RMC | UTC date |
| `time` | `TinyGPSTime` | GGA, RMC | UTC time |
| `speed` | `TinyGPSSpeed` | RMC | Ground speed |
| `course` | `TinyGPSCourse` | RMC | True course over ground |
| `altitude` | `TinyGPSAltitude` | GGA | MSL altitude |
| `satellites` | `TinyGPSInteger` | GGA | Satellites in use |
| `hdop` | `TinyGPSHDOP` | GGA | Horizontal dilution of precision |

#### `bool encode(char c)`
Feed one byte from the UART.  Returns `true` when a complete, valid
sentence has just been committed to the member objects above.

#### Diagnostic counters

| Method | What it tells you |
|---|---|
| `charsProcessed()` | Must be > 0; if zero after 30 s, no UART data is arriving |
| `passedChecksum()` | Must grow for a fix; zero with chars > 0 = baud rate mismatch |
| `failedChecksum()` | > 0 = noise or wrong baud rate |
| `sentencesWithFix()` | Sentences that contained a valid position fix |

### Common field methods

Every field type (`TinyGPSLocation`, `TinyGPSDate`, `TinyGPSTime`, …) exposes:

| Method | Description |
|---|---|
| `isValid()` | `true` after at least one value has been committed |
| `isUpdated()` | `true` if value changed since last read (cleared by `value()` / `lat()` / etc.) |
| `age()` | ms since last commit; `ULONG_MAX` if never valid |

### `TinyGPSCustom` — arbitrary term binding

```cpp
// Bind to GNVTG term 1 (true track made good, degrees)
TinyGPSCustom trueTrack(gps, "GNVTG", 1);

// … after encode() calls …
if (trueTrack.isUpdated())
    ESP_LOGI(TAG, "Track: %s°", trueTrack.value());
```

### Static utilities

```cpp
// Great-circle distance in metres (≤ 0.5% error)
double d = TinyGPSPlus::distanceBetween(lat1, lon1, lat2, lon2);

// Initial bearing in degrees (0=N, 90=E, 270=W)
double b = TinyGPSPlus::courseTo(lat1, lon1, lat2, lon2);

// 16-point compass label
const char *label = TinyGPSPlus::cardinal(b);  // e.g. "NNE"
```

---

## Diagnostics cheat sheet

| Observation | Likely cause |
|---|---|
| `charsProcessed() == 0` after 30 s | VGNSS rail unpowered, wrong pins, or UART not configured |
| `charsProcessed() > 0`, `passedChecksum() == 0` | Baud rate mismatch — try 9600 |
| Checksums pass, no fix | Normal cold start; needs open sky, allow 1–5 min |
| `failedChecksum() > 0` | Noise or baud mismatch — check wiring length and pull-ups |
| Fix lost immediately after acquire | `age()` threshold too tight — relax to 5000 ms |

---

## `age()` implementation note

`age()` on every field type calls `esp_timer_get_time()` (µs since boot) and
divides by 1000 to obtain milliseconds.  The result is cast to `uint32_t` and
wraps after ~49 days of continuous uptime, matching the original Arduino
`millis()` behaviour.

---

## License

TinyGPS++ core: GNU Lesser General Public License v2.1 (Mikal Hart, 2008-2024).
ESP-IDF port: same license.

---

```zsh
cd components/esp_tinygpsplusplus/src
rm -rf HT_TinyGPS++.h HT_TinyGPS++.cpp driver/ esp32/ esp32c3/ esp32s3/ lora/ loramac/ radio/
```
