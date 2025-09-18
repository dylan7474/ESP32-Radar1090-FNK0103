#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <cstring>

#include "config.h"

// --- Display & Timing Constants ---
static const uint16_t COLOR_BACKGROUND = TFT_BLACK;
static const uint16_t COLOR_TEXT = TFT_WHITE;
static const uint16_t COLOR_RADAR_OUTLINE = TFT_DARKGREEN;
static const uint16_t COLOR_RADAR_GRID = TFT_DARKGREY;
static const uint16_t COLOR_RADAR_CONTACT = TFT_GREEN;
static const uint16_t COLOR_RADAR_INBOUND = TFT_RED;
static const uint16_t COLOR_RADAR_HOME = TFT_SKYBLUE;
static const int INFO_START_Y = 58;
static const int INFO_LINE_HEIGHT = 26;
static const unsigned long REFRESH_INTERVAL_MS = 5000;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
static const double INBOUND_ALERT_DISTANCE_KM = 5.0;
static const int RADAR_RADIUS = 48;
static const int RADAR_MARGIN = 12;
static const double RADAR_MAX_RANGE_KM = 60.0;
static const int MAX_RADAR_CONTACTS = 40;

TFT_eSPI tft = TFT_eSPI();

struct AircraftInfo {
  String flight;
  double distanceKm;
  int altitude;
  double bearing;
  double groundSpeed;
  double track;
  double minutesToClosest;
  bool inbound;
  bool valid;
};

AircraftInfo closestAircraft;
int aircraftCount = 0;
int inboundAircraftCount = 0;
bool dataConnectionOk = false;
unsigned long lastFetchTime = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastSuccessfulFetch = 0;

// --- Function Prototypes ---
void drawStaticLayout();
void updateDisplay();
void drawInfoLine(int index, const String &text);
void drawStatusLine(int index, const String &text);
void drawRadar();
void resetRadarContacts();
void connectWiFi();
void fetchAircraft();
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double deg2rad(double deg);
String bearingToCardinal(double bearing);
String formatBearingString(double bearing);
String formatTimeAgo(unsigned long ms);

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(1);
  resetRadarContacts();
  drawStaticLayout();

  closestAircraft.valid = false;
  updateDisplay();

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    fetchAircraft();
    lastFetchTime = millis();
  }
}

void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiAttempt > WIFI_RETRY_INTERVAL_MS) {
      connectWiFi();
    }
  }

  if (now - lastFetchTime >= REFRESH_INTERVAL_MS) {
    lastFetchTime = now;
    fetchAircraft();
  }

  delay(50);
}

struct RadarContact {
  double distanceKm;
  double bearing;
  bool inbound;
  String flight;
  bool valid;
};

RadarContact radarContacts[MAX_RADAR_CONTACTS];
int radarContactCount = 0;

void resetRadarContacts() {
  radarContactCount = 0;
  for (int i = 0; i < MAX_RADAR_CONTACTS; ++i) {
    radarContacts[i].valid = false;
    radarContacts[i].flight = "";
  }
}

void drawStaticLayout() {
  tft.fillScreen(COLOR_BACKGROUND);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(3);
  tft.drawString("Closest Plane Radar", tft.width() / 2, 18);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  tft.setTextSize(2);
  tft.drawString("Live ADS-B Monitor", tft.width() / 2, 44);
  tft.drawFastHLine(0, 54, tft.width(), TFT_DARKGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(2);
  drawRadar();
}

void drawInfoLine(int index, const String &text) {
  int y = INFO_START_Y + index * INFO_LINE_HEIGHT;
  String content = text.length() ? text : " ";
  tft.setTextPadding(tft.width() - 20);
  tft.drawString(content, 10, y);
}

void drawStatusLine(int index, const String &text) {
  int baseY = tft.height() - 4 * INFO_LINE_HEIGHT;
  int y = baseY + index * INFO_LINE_HEIGHT;
  String content = text.length() ? text : " ";
  tft.setTextPadding(tft.width() - 20);
  tft.drawString(content, 10, y);
}

void updateDisplay() {
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  if (closestAircraft.valid) {
    String flight = closestAircraft.flight.length() ? closestAircraft.flight : String("(unknown)");
    flight.trim();
    String header = "Flight: " + flight;
    if (closestAircraft.inbound) {
      header += "  (INBOUND)";
    }
    drawInfoLine(0, header);

    String distanceLine = String("Dist: ") + String(closestAircraft.distanceKm, 1) + " km  " +
                          "Bearing: " + formatBearingString(closestAircraft.bearing);
    drawInfoLine(1, distanceLine);

    String altitudeLine = "Alt: ";
    if (closestAircraft.altitude >= 0) {
      altitudeLine += String(closestAircraft.altitude) + " ft";
    } else {
      altitudeLine += "---";
    }
    if (!isnan(closestAircraft.groundSpeed) && closestAircraft.groundSpeed >= 0) {
      altitudeLine += "    Spd: " + String(closestAircraft.groundSpeed, 0) + " kt";
    }
    drawInfoLine(2, altitudeLine);

    String statusLine;
    if (closestAircraft.inbound) {
      if (!isnan(closestAircraft.minutesToClosest) && closestAircraft.minutesToClosest >= 0) {
        statusLine = String("Inbound ETA: ") + String(closestAircraft.minutesToClosest, 1) + " min";
      } else {
        statusLine = "Inbound: ETA --";
      }
    } else if (!isnan(closestAircraft.track)) {
      statusLine = String("Track: ") + formatBearingString(closestAircraft.track);
    } else {
      statusLine = "Track: ---";
    }
    drawInfoLine(3, statusLine);
  } else {
    drawInfoLine(0, "No aircraft in range");
    drawInfoLine(1, " ");
    drawInfoLine(2, " ");
    drawInfoLine(3, " ");
  }

  String seenLine = "Seen aircraft: " + String(aircraftCount);
  if (inboundAircraftCount > 0) {
    seenLine += "  inbound: " + String(inboundAircraftCount);
  }
  drawStatusLine(0, seenLine);

  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    drawStatusLine(1, String("WiFi: connected ") + String(rssi) + " dBm");
  } else {
    drawStatusLine(1, "WiFi: offline");
  }

  String dataLine = String("Data link: ") + (dataConnectionOk ? "OK" : "waiting...");
  drawStatusLine(2, dataLine);

  String updateLine = "Last update: ";
  if (lastSuccessfulFetch > 0) {
    updateLine += formatTimeAgo(millis() - lastSuccessfulFetch);
  } else {
    updateLine += "--";
  }
  drawStatusLine(3, updateLine);

  tft.setTextPadding(0);
  drawRadar();
}

void drawRadar() {
  int centerX = tft.width() - RADAR_MARGIN - RADAR_RADIUS;
  int centerY = tft.height() - RADAR_MARGIN - RADAR_RADIUS;

  tft.fillCircle(centerX, centerY, RADAR_RADIUS, COLOR_BACKGROUND);
  tft.drawCircle(centerX, centerY, RADAR_RADIUS, COLOR_RADAR_OUTLINE);
  tft.drawCircle(centerX, centerY, RADAR_RADIUS / 2, COLOR_RADAR_OUTLINE);

  tft.drawFastHLine(centerX - RADAR_RADIUS, centerY, RADAR_RADIUS * 2, COLOR_RADAR_GRID);
  tft.drawFastVLine(centerX, centerY - RADAR_RADIUS, RADAR_RADIUS * 2, COLOR_RADAR_GRID);

  tft.fillCircle(centerX, centerY, 3, COLOR_RADAR_HOME);

  for (int i = 0; i < radarContactCount; ++i) {
    if (!radarContacts[i].valid) {
      continue;
    }

    double normalized = radarContacts[i].distanceKm / RADAR_MAX_RANGE_KM;
    if (normalized > 1.0) {
      normalized = 1.0;
    } else if (normalized < 0.0 || isnan(normalized)) {
      continue;
    }

    double angleRad = deg2rad(radarContacts[i].bearing);
    double radius = normalized * (RADAR_RADIUS - 3);
    int contactX = centerX + (int)round(sin(angleRad) * radius);
    int contactY = centerY - (int)round(cos(angleRad) * radius);

    uint16_t color = radarContacts[i].inbound ? COLOR_RADAR_INBOUND : COLOR_RADAR_CONTACT;
    tft.fillCircle(contactX, contactY, 3, color);
  }
}

void connectWiFi() {
  lastWifiAttempt = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Connecting to WiFi %s...\n", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    WiFi.setSleep(false);
  } else {
    Serial.println("WiFi connection failed.");
  }

  updateDisplay();
}

void fetchAircraft() {
  if (WiFi.status() != WL_CONNECTED) {
    dataConnectionOk = false;
    closestAircraft.valid = false;
    aircraftCount = 0;
    resetRadarContacts();
    updateDisplay();
    return;
  }

  HTTPClient http;
  char url[160];
  snprintf(url, sizeof(url), "http://%s:%d/dump1090-fa/data/aircraft.json", DUMP1090_SERVER, DUMP1090_PORT);

  if (!http.begin(url)) {
    dataConnectionOk = false;
    closestAircraft.valid = false;
    aircraftCount = 0;
    resetRadarContacts();
    updateDisplay();
    return;
  }

  http.setTimeout(4000);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument doc(16384);
    DeserializationError err = deserializeJson(doc, http.getStream());
    if (!err) {
      dataConnectionOk = true;
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      double bestDistance = 1e12;
      AircraftInfo best;
      best.valid = false;
      best.groundSpeed = NAN;
      best.track = NAN;
      best.minutesToClosest = NAN;
      best.inbound = false;
      aircraftCount = 0;
      resetRadarContacts();
      int localInboundCount = 0;

      for (JsonObject plane : arr) {
        if (!plane.containsKey("lat") || !plane.containsKey("lon")) {
          continue;
        }

        double lat = plane["lat"].as<double>();
        double lon = plane["lon"].as<double>();
        double distance = haversine(USER_LAT, USER_LON, lat, lon);
        if (isnan(distance) || isinf(distance)) {
          continue;
        }

        aircraftCount++;

        double bearingToHome = calculateBearing(USER_LAT, USER_LON, lat, lon);
        double groundSpeed = NAN;
        double track = NAN;
        double minutesToClosest = NAN;
        bool inbound = false;

        if (plane.containsKey("gs")) {
          JsonVariant speedVar = plane["gs"];
          if (speedVar.is<float>() || speedVar.is<double>() || speedVar.is<int>()) {
            groundSpeed = speedVar.as<double>();
          }
        }

        if (plane.containsKey("track")) {
          JsonVariant trackVar = plane["track"];
          if (trackVar.is<float>() || trackVar.is<double>() || trackVar.is<int>()) {
            track = trackVar.as<double>();
          }
        }

        if (!isnan(track) && !isnan(groundSpeed) && groundSpeed > 0) {
          double toBase = fmod(bearingToHome + 180.0, 360.0);
          double angleDiff = fabs(track - toBase);
          if (angleDiff > 180.0) {
            angleDiff = 360.0 - angleDiff;
          }
          double crossTrack = distance * sin(deg2rad(angleDiff));
          double alongTrack = distance * cos(deg2rad(angleDiff));
          if (angleDiff < 90.0 && fabs(crossTrack) <= INBOUND_ALERT_DISTANCE_KM && alongTrack >= 0) {
            inbound = true;
            double speedKmMin = groundSpeed * 1.852 / 60.0;
            if (speedKmMin > 0) {
              minutesToClosest = alongTrack / speedKmMin;
            }
          }
        }

        if (inbound) {
          localInboundCount++;
        }

        if (radarContactCount < MAX_RADAR_CONTACTS) {
          RadarContact &contact = radarContacts[radarContactCount++];
          contact.distanceKm = distance;
          contact.bearing = bearingToHome;
          contact.inbound = inbound;
          contact.valid = true;
          if (plane.containsKey("flight")) {
            const char *flightStr = plane["flight"].as<const char*>();
            contact.flight = flightStr ? String(flightStr) : String();
          } else {
            contact.flight = "";
          }
        }

        if (distance < bestDistance) {
          bestDistance = distance;
          best.valid = true;
          best.distanceKm = distance;
          best.bearing = bearingToHome;
          best.groundSpeed = groundSpeed;
          best.track = track;
          best.minutesToClosest = minutesToClosest;
          best.inbound = inbound;

          if (plane.containsKey("flight")) {
            const char *flightStr = plane["flight"].as<const char*>();
            best.flight = flightStr ? String(flightStr) : String();
          } else {
            best.flight = "";
          }

          if (plane.containsKey("alt_baro")) {
            JsonVariant alt = plane["alt_baro"];
            if (alt.is<int>()) {
              best.altitude = alt.as<int>();
            } else if (alt.is<const char*>() && strcmp(alt.as<const char*>(), "ground") == 0) {
              best.altitude = 0;
            } else {
              best.altitude = -1;
            }
          } else {
            best.altitude = -1;
          }
        }
      }

      closestAircraft = best;
      inboundAircraftCount = localInboundCount;
      if (best.valid) {
        lastSuccessfulFetch = millis();
      }
    } else {
      dataConnectionOk = false;
      closestAircraft.valid = false;
      aircraftCount = 0;
      inboundAircraftCount = 0;
      resetRadarContacts();
    }
  } else {
    dataConnectionOk = false;
    closestAircraft.valid = false;
    aircraftCount = 0;
    inboundAircraftCount = 0;
    resetRadarContacts();
  }

  http.end();
  updateDisplay();
}

double deg2rad(double deg) { return deg * PI / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  const double EARTH_RADIUS_KM = 6371.0;
  return EARTH_RADIUS_KM * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double lonDiff = deg2rad(lon2 - lon1);
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);
  double y = sin(lonDiff) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lonDiff);
  double bearing = atan2(y, x);
  bearing = fmod((bearing * 180.0 / PI + 360.0), 360.0);
  return bearing;
}

String bearingToCardinal(double bearing) {
  if (isnan(bearing)) {
    return String("---");
  }
  static const char *directions[] = {
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
  };
  double normalized = fmod(bearing + 360.0, 360.0);
  int index = (int)round(normalized / 22.5) % 16;
  return String(directions[index]);
}

String formatBearingString(double bearing) {
  if (isnan(bearing)) {
    return String("---");
  }
  double normalized = fmod(bearing + 360.0, 360.0);
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%.0f°", normalized);
  return String(buffer) + " (" + bearingToCardinal(normalized) + ")";
}

String formatTimeAgo(unsigned long ms) {
  unsigned long seconds = ms / 1000;
  if (seconds < 60) {
    return String(seconds) + "s ago";
  }
  unsigned long minutes = seconds / 60;
  seconds %= 60;
  if (minutes < 60) {
    char buffer[24];
    snprintf(buffer, sizeof(buffer), "%lum %02lus ago", minutes, seconds);
    return String(buffer);
  }
  unsigned long hours = minutes / 60;
  minutes %= 60;
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%luh %02lum ago", hours, minutes);
  return String(buffer);
}
