# Temperature sensors

A small Arduino sketch for ESP01-S which samples and transmits temperature and
humidity data across MQTT on the local network. Uses timed sampling at a user
configurable interval, persistent config in EEPROM, no-overhead debug logging
and NTP syncronized RTC time.
