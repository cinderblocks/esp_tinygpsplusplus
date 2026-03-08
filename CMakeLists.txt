# esp_tinygpsplusplus — ESP-IDF component
#
# Pure ESP-IDF port of TinyGPS++ (NMEA-0183 parser).
# No Arduino dependency; millis() is implemented via esp_timer_get_time().
#
# To use from another component or main/:
#
#   idf_component_register(
#       ...
#       REQUIRES esp_tinygpsplusplus
#   )
#
#   #include <TinyGPSPlus.h>

cmake_minimum_required(VERSION 3.22)

idf_component_register(
    SRCS         "src/TinyGPSPlus.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     "esp_timer"
)
