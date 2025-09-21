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

// I2S pins for MAX98357A audio output (defaults map to the FNK0103 speaker header)
#define I2S_BCLK_PIN  26
#define I2S_LRCLK_PIN 25
#define I2S_DOUT_PIN  27

// Audio stream configuration
#define AUDIO_STREAM_URL "http://192.168.50.4:8000/airband.mp3"
#define AUDIO_STREAM_VOLUME 18  // Valid range 0 (mute) to 21 (max)

#endif // CONFIG_H
