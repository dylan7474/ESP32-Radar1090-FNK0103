#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
#define WIFI_SSID "YOUR-SSID"
#define WIFI_PASSWORD "YOUR-PASSWORD"

// -- User Location --
#define USER_LAT 54.0 
#define USER_LON -1.0

// -- Radar & Server Settings --
#define DUMP1090_SERVER "192.168.50.100" // IP or hostname of dump1090 server
#define DUMP1090_PORT 8080               // The default web port for dump1090 is often 8080

// Icecast audio stream configuration
#define STREAM_URL "http://radio.dylanjones.org:8000/Audacity"

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
