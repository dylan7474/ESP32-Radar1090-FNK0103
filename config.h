#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
#define WIFI_SSID "YOUR-SSID"
#define WIFI_PASSWORD "YOUR-PASSWORD"

// -- User Location --
#define USER_LAT 54.0 
#define USER_LON -1.0

// -- Radar & Server Settings --
#define AIRCRAFT_DATA_URL "http://dump1090.dylanjones.org/data/aircraft.json" // Full URL to the dump1090 aircraft.json feed

// Icecast audio stream configuration
#define STREAM_URL "http://audio.dylanjones.org/stream"

// Audio output configuration
// The FNK0103 board uses the onboard amplifier driven from the ESP32's
// internal DAC on GPIO25.  Leave the bit clock and word select pins set to -1
// to target the internal DAC path.  Set them to valid GPIOs if using an
// external I2S amplifier instead.
#define AUDIO_AMP_ENABLE_PIN 4
#define I2S_BCLK_PIN  -1
#define I2S_LRCLK_PIN -1
#define I2S_DOUT_PIN  25

#endif // CONFIG_H
