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
static const uint16_t COLOR_INFO_TABLE_BG = TFT_NAVY;
static const uint16_t COLOR_INFO_TABLE_HEADER_BG = TFT_BLUE;
static const uint16_t COLOR_INFO_TABLE_BORDER = TFT_WHITE;
static const int INFO_TEXT_SIZE = 2;
static const unsigned long REFRESH_INTERVAL_MS = 5000;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
static const int RADAR_MARGIN = 12;
static const int RADAR_TOP_PADDING = 24;
static const int MAX_RADAR_CONTACTS = 40;
static const unsigned long RADAR_SWEEP_PERIOD_MS = 4000;
static const unsigned long RADAR_FADE_DURATION_MS = 4000;
static const unsigned long RADAR_FRAME_INTERVAL_MS = 40;
static const double RADAR_SWEEP_WIDTH_DEG = 3.0;
static const uint16_t COLOR_RADAR_SWEEP = TFT_DARKGREEN;
static const uint16_t COLOR_BUTTON_ACTIVE = TFT_DARKGREEN;
static const uint16_t COLOR_BUTTON_INACTIVE = TFT_DARKGREY;
static const int INFO_TABLE_ROW_HEIGHT = 28;
static const int INFO_TABLE_HEADER_HEIGHT = 48;
static const int INFO_TABLE_PADDING = 8;
static const int COMPASS_LABEL_OFFSET = 16;
static const int COMPASS_TEXT_SIZE = 2;

static const int BUTTON_COUNT = 2;
static const int BUTTON_HEIGHT = 48;
static const int BUTTON_SPACING = 12;
static const unsigned long TOUCH_DEBOUNCE_MS = 250;
// Resistive touch calibration bounds; adjust if on-screen touch points do not align.
#ifndef TOUCH_RAW_MIN_X
#define TOUCH_RAW_MIN_X 200
#endif
#ifndef TOUCH_RAW_MAX_X
#define TOUCH_RAW_MAX_X 3900
#endif
#ifndef TOUCH_RAW_MIN_Y
#define TOUCH_RAW_MIN_Y 200
#endif
#ifndef TOUCH_RAW_MAX_Y
#define TOUCH_RAW_MAX_Y 3900
#endif
#ifndef TOUCH_SWAP_XY
#define TOUCH_SWAP_XY 0
#endif
#ifndef TOUCH_INVERT_X
#define TOUCH_INVERT_X 0
#endif
#ifndef TOUCH_INVERT_Y
#define TOUCH_INVERT_Y 1
#endif
static const int WIFI_ICON_BARS = 4;
static const int WIFI_ICON_BAR_WIDTH = 5;
static const int WIFI_ICON_BAR_SPACING = 3;
static const int WIFI_ICON_HEIGHT = 20;

static const double RADAR_RANGE_OPTIONS_KM[] = {5.0, 10.0, 25.0, 50.0, 100.0, 200.0, 300.0};
static const int RADAR_RANGE_OPTION_COUNT = sizeof(RADAR_RANGE_OPTIONS_KM) / sizeof(RADAR_RANGE_OPTIONS_KM[0]);
static const double ALERT_RANGE_OPTIONS_KM[] = {1.0, 3.0, 5.0, 10.0};
static const int ALERT_RANGE_OPTION_COUNT = sizeof(ALERT_RANGE_OPTIONS_KM) / sizeof(ALERT_RANGE_OPTIONS_KM[0]);
static const double DEFAULT_RADAR_RANGE_KM = 25.0;
static const double DEFAULT_ALERT_RANGE_KM = 5.0;

static const int COMPASS_LABEL_COUNT = 4;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite radarSprite = TFT_eSprite(&tft);

bool radarSpriteActive = false;
int radarSpriteWidth = 0;
int radarSpriteHeight = 0;

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
unsigned long radarSweepStart = 0;
unsigned long lastRadarFrameTime = 0;

int radarRangeIndex = 0;
int alertRangeIndex = 0;

int lastWifiBars = -1;
bool lastWifiConnectedState = false;

int radarCenterX = 0;
int radarCenterY = 0;
int radarRadius = 0;
int radarAreaY = 0;
int radarAreaHeight = 0;
int infoAreaX = 0;
int infoAreaY = 0;
int infoAreaWidth = 0;
int infoAreaHeight = 0;
int buttonAreaY = 0;

uint8_t displayRotation = 0;

enum ButtonType {
  BUTTON_RADAR_RANGE,
  BUTTON_ALERT_RANGE,
  BUTTON_UNKNOWN
};

struct TouchButton {
  int x;
  int y;
  int w;
  int h;
  bool state;
  const char *name;
  ButtonType type;
};

TouchButton buttons[BUTTON_COUNT];
unsigned long lastTouchTime = 0;

int activeContactIndex = -1;
bool infoPanelDirty = true;

struct InfoTableRow {
  String label;
  String value;
};

struct InfoPanelCache {
  bool initialized;
  int cachedInfoAreaX;
  int cachedInfoAreaY;
  int cachedInfoAreaWidth;
  int cachedTextAreaHeight;
  int cachedHeaderHeight;
  int cachedTableTop;
  int cachedDividerX;
  int cachedRowCount;
  InfoTableRow cachedRows[10];
};

InfoPanelCache infoPanelCache = {};

struct CompassLabelBounds {
  const char *label;
  int x;
  int y;
  int w;
  int h;
};

CompassLabelBounds compassLabelBounds[COMPASS_LABEL_COUNT];
bool compassLabelBoundsValid = false;

void drawButtons();
void resetRadarContacts();
void updateDisplay();
void fetchAircraft();
void initializeRangeIndices();
double currentRadarRangeKm();
double currentAlertRangeKm();
void cycleRadarRange();
void cycleAlertRange();
void handleRangeButton(ButtonType type);
void renderInfoPanel();
bool setActiveContact(int index);
bool clearActiveContact();
bool ensureActiveContactFresh(unsigned long now);

void initializeRangeIndices() {
  radarRangeIndex = 0;
  alertRangeIndex = 0;

  for (int i = 0; i < RADAR_RANGE_OPTION_COUNT; ++i) {
    if (fabs(RADAR_RANGE_OPTIONS_KM[i] - DEFAULT_RADAR_RANGE_KM) < 0.01) {
      radarRangeIndex = i;
      break;
    }
  }

  for (int i = 0; i < ALERT_RANGE_OPTION_COUNT; ++i) {
    if (fabs(ALERT_RANGE_OPTIONS_KM[i] - DEFAULT_ALERT_RANGE_KM) < 0.01) {
      alertRangeIndex = i;
      break;
    }
  }
}

double currentRadarRangeKm() {
  if (radarRangeIndex < 0 || radarRangeIndex >= RADAR_RANGE_OPTION_COUNT) {
    return RADAR_RANGE_OPTIONS_KM[0];
  }
  return RADAR_RANGE_OPTIONS_KM[radarRangeIndex];
}

double currentAlertRangeKm() {
  if (alertRangeIndex < 0 || alertRangeIndex >= ALERT_RANGE_OPTION_COUNT) {
    return ALERT_RANGE_OPTIONS_KM[0];
  }
  return ALERT_RANGE_OPTIONS_KM[alertRangeIndex];
}

void cycleRadarRange() {
  if (RADAR_RANGE_OPTION_COUNT <= 0) {
    return;
  }
  radarRangeIndex = (radarRangeIndex + 1) % RADAR_RANGE_OPTION_COUNT;
}

void cycleAlertRange() {
  if (ALERT_RANGE_OPTION_COUNT <= 0) {
    return;
  }
  alertRangeIndex = (alertRangeIndex + 1) % ALERT_RANGE_OPTION_COUNT;
}

void handleRangeButton(ButtonType type) {
  if (type == BUTTON_RADAR_RANGE) {
    cycleRadarRange();
  } else if (type == BUTTON_ALERT_RANGE) {
    cycleAlertRange();
  } else {
    return;
  }

  drawButtons();
  resetRadarContacts();
  closestAircraft.valid = false;
  aircraftCount = 0;
  inboundAircraftCount = 0;
  updateDisplay();
  fetchAircraft();
  lastFetchTime = millis();
}

// --- Function Prototypes ---
void drawStaticLayout();
void updateDisplay();
void drawInfoLine(int index, const String &text);
void drawRadar();
void resetRadarContacts();
void setupRadarSprite();
void connectWiFi();
void fetchAircraft();
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double deg2rad(double deg);
String bearingToCardinal(double bearing);
String formatBearingString(double bearing);
String formatTimeAgo(unsigned long ms);
uint16_t fadeColor(uint16_t color, float alpha);
double angularDifference(double a, double b);
void drawStatusBar();
void drawWifiIcon(int x, int y, int barsActive, bool connected);
void configureButtons();
void drawButtons();
void drawButton(int index);
bool readTouchPoint(int &screenX, int &screenY);
void handleTouch();
bool touchInCompassLabel(const char *label, int touchX, int touchY);
void rotateRadarOrientation();

template <typename GFX>
void drawCompassLabels(GFX &gfx, int centerX, int centerY, int radius) {
  static const char *const labels[] = {"N", "E", "S", "W"};
  static const double angles[] = {0.0, 90.0, 180.0, 270.0};
  compassLabelBoundsValid = false;
  if (radius <= 0) {
    return;
  }

  int labelRadius = radius + COMPASS_LABEL_OFFSET;
  gfx.setTextDatum(MC_DATUM);
  gfx.setTextSize(COMPASS_TEXT_SIZE);
  gfx.setTextColor(COLOR_RADAR_GRID, COLOR_BACKGROUND);

  for (int i = 0; i < COMPASS_LABEL_COUNT; ++i) {
    double angleRad = deg2rad(angles[i]);
    int labelX = centerX + (int)round(sin(angleRad) * labelRadius);
    int labelY = centerY - (int)round(cos(angleRad) * labelRadius);
    gfx.drawString(labels[i], labelX, labelY);

    int textWidth = gfx.textWidth(labels[i]);
    if (textWidth <= 0) {
      textWidth = COMPASS_TEXT_SIZE * 6;
    }
    int textHeight = COMPASS_TEXT_SIZE * 8;
    int halfWidth = max(textWidth / 2, 1);
    int halfHeight = max(textHeight / 2, 1);

    compassLabelBounds[i].label = labels[i];
    compassLabelBounds[i].x = labelX - halfWidth;
    compassLabelBounds[i].y = labelY - halfHeight;
    compassLabelBounds[i].w = max(textWidth, 1);
    compassLabelBounds[i].h = max(textHeight, 1);
  }

  compassLabelBoundsValid = true;
  gfx.setTextSize(1);
}

void setup() {
  Serial.begin(115200);
  tft.begin();
  displayRotation = 0;
  tft.setRotation(displayRotation);
  initializeRangeIndices();
  radarSweepStart = millis();
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

  if (now - lastRadarFrameTime >= RADAR_FRAME_INTERVAL_MS) {
    lastRadarFrameTime = now;
    drawRadar();
  }

  handleTouch();

  delay(10);
}

struct RadarContact {
  double distanceKm;
  double bearing;
  bool inbound;
  String flight;
  int altitude;
  double groundSpeed;
  double track;
  double minutesToClosest;
  bool valid;
  unsigned long lastHighlightTime;
  bool stale;
};

RadarContact radarContacts[MAX_RADAR_CONTACTS];
int radarContactCount = 0;

void resetRadarContacts() {
  radarContactCount = 0;
  for (int i = 0; i < MAX_RADAR_CONTACTS; ++i) {
    radarContacts[i].valid = false;
    radarContacts[i].flight = "";
    radarContacts[i].lastHighlightTime = 0;
    radarContacts[i].inbound = false;
    radarContacts[i].distanceKm = 0.0;
    radarContacts[i].bearing = 0.0;
    radarContacts[i].stale = false;
    radarContacts[i].altitude = -1;
    radarContacts[i].groundSpeed = NAN;
    radarContacts[i].track = NAN;
    radarContacts[i].minutesToClosest = NAN;
  }
  activeContactIndex = -1;
  infoPanelDirty = true;
}

void setupRadarSprite() {
  if (radarSpriteActive) {
    radarSprite.deleteSprite();
    radarSpriteActive = false;
  }

  if (radarRadius <= 0) {
    return;
  }

  radarSpriteWidth = radarRadius * 2 + 1;
  radarSpriteHeight = radarSpriteWidth;

  if (radarSpriteWidth <= 0 || radarSpriteHeight <= 0) {
    return;
  }

  radarSprite.setColorDepth(16);
  if (radarSprite.createSprite(radarSpriteWidth, radarSpriteHeight) == nullptr) {
    return;
  }

  radarSprite.fillSprite(COLOR_BACKGROUND);
  radarSpriteActive = true;
}

void drawStaticLayout() {
  tft.fillScreen(COLOR_BACKGROUND);
  radarAreaY = RADAR_TOP_PADDING;
  radarAreaHeight = tft.height() / 2;
  int availableRadarHeight = max(radarAreaHeight - RADAR_MARGIN * 2, 0);
  int radarDiameter = min(tft.width() - RADAR_MARGIN * 2, availableRadarHeight);
  radarRadius = max(radarDiameter / 2, 0);
  radarCenterX = tft.width() / 2;
  radarCenterY = radarAreaY + RADAR_MARGIN + radarRadius;

  infoAreaX = RADAR_MARGIN;
  infoAreaWidth = max(tft.width() - RADAR_MARGIN * 2, 0);
  infoAreaY = radarAreaY + radarAreaHeight + RADAR_MARGIN;
  infoAreaHeight = max(tft.height() - infoAreaY - RADAR_MARGIN, 0);
  int textAreaHeight = max(infoAreaHeight - BUTTON_HEIGHT - BUTTON_SPACING, 0);
  buttonAreaY = infoAreaY + textAreaHeight;
  if (textAreaHeight > 0) {
    buttonAreaY += BUTTON_SPACING;
  }
  int infoAreaBottom = infoAreaY + infoAreaHeight;
  if (buttonAreaY + BUTTON_HEIGHT > infoAreaBottom) {
    buttonAreaY = max(infoAreaBottom - BUTTON_HEIGHT, infoAreaY);
  }

  tft.fillRect(infoAreaX, infoAreaY, infoAreaWidth, infoAreaHeight, COLOR_BACKGROUND);
  infoPanelCache.initialized = false;
  infoPanelCache.cachedRowCount = 0;

  configureButtons();
  setupRadarSprite();

  lastWifiBars = -1;
  lastWifiConnectedState = false;
  activeContactIndex = -1;
  infoPanelDirty = true;

  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  drawRadar();
  drawButtons();
}

void renderInfoPanel() {
  if (!infoPanelDirty) {
    return;
  }

  int textAreaHeight = max(buttonAreaY - infoAreaY, 0);
  if (textAreaHeight <= 0 || infoAreaWidth <= 0) {
    infoPanelDirty = false;
    return;
  }

  tft.setTextSize(INFO_TEXT_SIZE);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);

  const RadarContact *activeContact = nullptr;
  if (activeContactIndex >= 0 && activeContactIndex < radarContactCount) {
    RadarContact &candidate = radarContacts[activeContactIndex];
    unsigned long now = millis();
    if (candidate.valid && candidate.lastHighlightTime != 0 && (now - candidate.lastHighlightTime) <= RADAR_FADE_DURATION_MS) {
      activeContact = &candidate;
    }
  }

  InfoTableRow rows[10];
  int rowCount = 0;
  auto addRow = [&](const String &label, const String &value) {
    if (rowCount < (int)(sizeof(rows) / sizeof(rows[0]))) {
      rows[rowCount].label = label;
      rows[rowCount].value = value;
      ++rowCount;
    }
  };

  if (activeContact != nullptr) {
    String flight = activeContact->flight;
    flight.trim();
    if (!flight.length()) {
      flight = String("(Unknown)");
    }
    addRow("Flight", flight);
    String speedValue = "--";
    if (!isnan(activeContact->groundSpeed) && activeContact->groundSpeed >= 0) {
      speedValue = String(activeContact->groundSpeed, 0) + " kt";
    }
    addRow("Speed", speedValue);
    addRow("Distance", String(activeContact->distanceKm, 1) + " km");
    String altitudeValue = "--";
    if (activeContact->altitude >= 0) {
      altitudeValue = String(activeContact->altitude) + " ft";
    }
    addRow("Altitude", altitudeValue);
    addRow("Bearing", String(activeContact->bearing, 0) + " deg");
    if (!isnan(activeContact->track)) {
      addRow("Track", String(activeContact->track, 0) + " deg");
    }
    if (activeContact->inbound) {
      if (!isnan(activeContact->minutesToClosest) && activeContact->minutesToClosest >= 0) {
        addRow("ETA", String(activeContact->minutesToClosest, 1) + " min");
      } else {
        addRow("ETA", "Approaching");
      }
    }
  } else if (closestAircraft.valid) {
    String flight = closestAircraft.flight;
    flight.trim();
    if (!flight.length()) {
      flight = String("(unknown)");
    }
    addRow("Flight", flight);
    String speedValue = "--";
    if (!isnan(closestAircraft.groundSpeed) && closestAircraft.groundSpeed >= 0) {
      speedValue = String(closestAircraft.groundSpeed, 0) + " kt";
    }
    addRow("Speed", speedValue);
    addRow("Distance", String(closestAircraft.distanceKm, 1) + " km");
    String altitudeValue = "--";
    if (closestAircraft.altitude >= 0) {
      altitudeValue = String(closestAircraft.altitude) + " ft";
    }
    addRow("Altitude", altitudeValue);
    addRow("Bearing", String(closestAircraft.bearing, 0) + " deg");
    if (!isnan(closestAircraft.track)) {
      addRow("Track", String(closestAircraft.track, 0) + " deg");
    }
    if (closestAircraft.inbound && !isnan(closestAircraft.minutesToClosest) && closestAircraft.minutesToClosest >= 0) {
      addRow("ETA", String(closestAircraft.minutesToClosest, 1) + " min");
    }
  } else {
    String statusMessage = dataConnectionOk ? String("Waiting for traffic") : String("Awaiting data link");
    addRow("Flight", statusMessage);
    addRow("Speed", "--");
    addRow("Distance", "--");
    addRow("Altitude", "--");
  }

  if (aircraftCount > 0) {
    String traffic = String(aircraftCount) + " tracked";
    if (inboundAircraftCount > 0) {
      traffic += " / " + String(inboundAircraftCount) + " inbound";
    }
    addRow("Traffic", traffic);
  }

  int headerHeight = 0;
  int availableHeight = textAreaHeight;
  int maxRows = INFO_TABLE_ROW_HEIGHT > 0 ? availableHeight / INFO_TABLE_ROW_HEIGHT : 0;
  if (maxRows < rowCount) {
    rowCount = maxRows;
  }

  int tableTop = infoAreaY + headerHeight;
  int dividerX = infoAreaX + infoAreaWidth / 2;

  bool geometryChanged = !infoPanelCache.initialized || infoPanelCache.cachedInfoAreaX != infoAreaX ||
                         infoPanelCache.cachedInfoAreaY != infoAreaY ||
                         infoPanelCache.cachedInfoAreaWidth != infoAreaWidth ||
                         infoPanelCache.cachedTextAreaHeight != textAreaHeight;

  bool rowStructureChanged = geometryChanged || infoPanelCache.cachedHeaderHeight != headerHeight ||
                             infoPanelCache.cachedRowCount != rowCount;

  if (rowStructureChanged) {
    tft.fillRect(infoAreaX, infoAreaY, infoAreaWidth, textAreaHeight, COLOR_INFO_TABLE_BG);
    tft.drawRect(infoAreaX, infoAreaY, infoAreaWidth, textAreaHeight, COLOR_INFO_TABLE_BORDER);

    if (rowCount > 0) {
      int tableHeight = rowCount * INFO_TABLE_ROW_HEIGHT;
      tft.drawFastVLine(dividerX, tableTop, tableHeight, COLOR_INFO_TABLE_BORDER);
      for (int i = 0; i < rowCount; ++i) {
        int rowY = tableTop + i * INFO_TABLE_ROW_HEIGHT;
        tft.drawFastHLine(infoAreaX, rowY, infoAreaWidth, COLOR_INFO_TABLE_BORDER);
      }
      tft.drawFastHLine(infoAreaX, tableTop + rowCount * INFO_TABLE_ROW_HEIGHT, infoAreaWidth, COLOR_INFO_TABLE_BORDER);
    }
  }

  int textHeight = INFO_TEXT_SIZE * 8;
  int rowsToProcess = max(rowCount, infoPanelCache.cachedRowCount);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_INFO_TABLE_BG);
  for (int i = 0; i < rowsToProcess; ++i) {
    bool rowExists = i < rowCount;
    bool hadRow = i < infoPanelCache.cachedRowCount;
    bool rowChanged = rowStructureChanged || (rowExists && (!hadRow || rows[i].label != infoPanelCache.cachedRows[i].label ||
                                                           rows[i].value != infoPanelCache.cachedRows[i].value)) ||
                      (!rowExists && hadRow);
    if (!rowChanged) {
      continue;
    }

    int rowY = tableTop + i * INFO_TABLE_ROW_HEIGHT;
    int rowFillY = rowY + 1;
    int rowFillH = max(INFO_TABLE_ROW_HEIGHT - 1, 0);
    if (rowFillH > 0) {
      int labelFillX = infoAreaX + 1;
      int labelFillW = max(dividerX - labelFillX - 1, 0);
      if (labelFillW > 0) {
        tft.fillRect(labelFillX, rowFillY, labelFillW, rowFillH, COLOR_INFO_TABLE_BG);
      }
      int valueFillX = dividerX + 1;
      int valueFillW = max(infoAreaX + infoAreaWidth - 1 - valueFillX, 0);
      if (valueFillW > 0) {
        tft.fillRect(valueFillX, rowFillY, valueFillW, rowFillH, COLOR_INFO_TABLE_BG);
      }
    }

    if (rowExists) {
      int textY = rowY + max((INFO_TABLE_ROW_HEIGHT - textHeight) / 2, 0);
      int labelWidth = dividerX - infoAreaX - INFO_TABLE_PADDING * 2;
      if (labelWidth < 0) {
        labelWidth = 0;
      }
      int valueWidth = infoAreaX + infoAreaWidth - dividerX - INFO_TABLE_PADDING * 2;
      if (valueWidth < 0) {
        valueWidth = 0;
      }
      tft.setTextPadding(labelWidth);
      tft.drawString(rows[i].label, infoAreaX + INFO_TABLE_PADDING, textY);
      tft.setTextPadding(valueWidth);
      tft.drawString(rows[i].value, dividerX + INFO_TABLE_PADDING, textY);
    }
  }

  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);

  infoPanelCache.initialized = true;
  infoPanelCache.cachedInfoAreaX = infoAreaX;
  infoPanelCache.cachedInfoAreaY = infoAreaY;
  infoPanelCache.cachedInfoAreaWidth = infoAreaWidth;
  infoPanelCache.cachedTextAreaHeight = textAreaHeight;
  infoPanelCache.cachedHeaderHeight = headerHeight;
  infoPanelCache.cachedTableTop = tableTop;
  infoPanelCache.cachedDividerX = dividerX;
  infoPanelCache.cachedRowCount = rowCount;
  for (int i = 0; i < rowCount; ++i) {
    infoPanelCache.cachedRows[i] = rows[i];
  }
  for (int i = rowCount; i < (int)(sizeof(infoPanelCache.cachedRows) / sizeof(infoPanelCache.cachedRows[0])); ++i) {
    infoPanelCache.cachedRows[i].label = "";
    infoPanelCache.cachedRows[i].value = "";
  }

  infoPanelDirty = false;
}

void updateDisplay() {
  infoPanelDirty = true;
  renderInfoPanel();
  drawRadar();
  tft.setTextSize(1);
}

bool setActiveContact(int index) {
  if (index < 0 || index >= radarContactCount) {
    return clearActiveContact();
  }

  RadarContact &contact = radarContacts[index];
  if (!contact.valid) {
    return clearActiveContact();
  }

  if (activeContactIndex != index) {
    activeContactIndex = index;
    infoPanelDirty = true;
    return true;
  }

  return false;
}

bool clearActiveContact() {
  if (activeContactIndex >= 0) {
    activeContactIndex = -1;
    infoPanelDirty = true;
    return true;
  }
  return false;
}

bool ensureActiveContactFresh(unsigned long now) {
  if (activeContactIndex < 0) {
    return false;
  }
  if (activeContactIndex >= radarContactCount) {
    return clearActiveContact();
  }

  RadarContact &contact = radarContacts[activeContactIndex];
  if (!contact.valid || contact.lastHighlightTime == 0 || (now - contact.lastHighlightTime) > RADAR_FADE_DURATION_MS) {
    return clearActiveContact();
  }

  return false;
}

void drawRadar() {
  unsigned long now = millis();
  lastRadarFrameTime = now;
  compassLabelBoundsValid = false;
  if (radarRadius <= 0) {
    return;
  }

  double radarRangeKm = currentRadarRangeKm();
  if (radarRangeKm <= 0.0) {
    return;
  }

  if (infoPanelDirty) {
    renderInfoPanel();
  }

  bool highlightChanged = false;
  unsigned long sweepElapsed = (now - radarSweepStart) % RADAR_SWEEP_PERIOD_MS;
  double sweepProgress = (double)sweepElapsed / (double)RADAR_SWEEP_PERIOD_MS;
  double sweepAngle = sweepProgress * 360.0;
  double sweepRad = deg2rad(sweepAngle);
  bool flashOn = ((now / 400) % 2) == 0;

  if (radarSpriteActive) {
    radarSprite.fillSprite(COLOR_BACKGROUND);
    int spriteCenter = radarSpriteWidth / 2;

    radarSprite.drawCircle(spriteCenter, spriteCenter, radarRadius, COLOR_RADAR_OUTLINE);
    radarSprite.drawCircle(spriteCenter, spriteCenter, radarRadius / 2, COLOR_RADAR_OUTLINE);
    radarSprite.drawFastHLine(spriteCenter - radarRadius, spriteCenter, radarRadius * 2, COLOR_RADAR_GRID);
    radarSprite.drawFastVLine(spriteCenter, spriteCenter - radarRadius, radarRadius * 2, COLOR_RADAR_GRID);
    radarSprite.fillCircle(spriteCenter, spriteCenter, 3, COLOR_RADAR_HOME);

    int sweepX = spriteCenter + (int)round(sin(sweepRad) * (radarRadius - 1));
    int sweepY = spriteCenter - (int)round(cos(sweepRad) * (radarRadius - 1));
    radarSprite.drawLine(spriteCenter, spriteCenter, sweepX, sweepY, COLOR_RADAR_SWEEP);

    for (int i = 0; i < radarContactCount; ++i) {
      if (!radarContacts[i].valid) {
        continue;
      }

      double normalized = radarContacts[i].distanceKm / radarRangeKm;
      if (normalized > 1.0) {
        normalized = 1.0;
      } else if (normalized < 0.0 || isnan(normalized)) {
        continue;
      }

      double angleRad = deg2rad(radarContacts[i].bearing);
      double radius = normalized * (radarRadius - 3);
      int contactX = spriteCenter + (int)round(sin(angleRad) * radius);
      int contactY = spriteCenter - (int)round(cos(angleRad) * radius);

      double angleDiff = angularDifference(radarContacts[i].bearing, sweepAngle);
      if (!radarContacts[i].stale && angleDiff <= RADAR_SWEEP_WIDTH_DEG) {
        radarContacts[i].lastHighlightTime = now;
        if (setActiveContact(i)) {
          highlightChanged = true;
        }
      }

      if (radarContacts[i].lastHighlightTime == 0) {
        continue;
      }

      unsigned long sinceHighlight = now - radarContacts[i].lastHighlightTime;
      if (sinceHighlight > RADAR_FADE_DURATION_MS) {
        radarContacts[i].valid = false;
        continue;
      }

      float alpha = 1.0f - (float)sinceHighlight / (float)RADAR_FADE_DURATION_MS;
      uint16_t baseColor;
      if (radarContacts[i].inbound) {
        baseColor = radarContacts[i].stale
                        ? COLOR_RADAR_INBOUND
                        : (flashOn ? COLOR_RADAR_INBOUND : COLOR_BACKGROUND);
      } else {
        baseColor = COLOR_RADAR_CONTACT;
      }
      uint16_t fadedColor = fadeColor(baseColor, alpha);
      radarSprite.fillCircle(contactX, contactY, 3, fadedColor);
    }

    int spriteX = radarCenterX - radarRadius;
    int spriteY = radarCenterY - radarRadius;
    radarSprite.pushSprite(spriteX, spriteY);
  } else {
    int centerX = radarCenterX;
    int centerY = radarCenterY;

    int clearWidth = max(tft.width() - RADAR_MARGIN * 2, 0);
    int clearHeight = max(radarAreaHeight, 0);
    if (clearWidth > 0 && clearHeight > 0) {
      tft.fillRect(RADAR_MARGIN, radarAreaY, clearWidth, clearHeight, COLOR_BACKGROUND);
    }

    tft.fillCircle(centerX, centerY, radarRadius, COLOR_BACKGROUND);
    tft.drawCircle(centerX, centerY, radarRadius, COLOR_RADAR_OUTLINE);
    tft.drawCircle(centerX, centerY, radarRadius / 2, COLOR_RADAR_OUTLINE);

    tft.drawFastHLine(centerX - radarRadius, centerY, radarRadius * 2, COLOR_RADAR_GRID);
    tft.drawFastVLine(centerX, centerY - radarRadius, radarRadius * 2, COLOR_RADAR_GRID);
    tft.fillCircle(centerX, centerY, 3, COLOR_RADAR_HOME);

    int sweepX = centerX + (int)round(sin(sweepRad) * (radarRadius - 1));
    int sweepY = centerY - (int)round(cos(sweepRad) * (radarRadius - 1));
    tft.drawLine(centerX, centerY, sweepX, sweepY, COLOR_RADAR_SWEEP);

    for (int i = 0; i < radarContactCount; ++i) {
      if (!radarContacts[i].valid) {
        continue;
      }

      double normalized = radarContacts[i].distanceKm / radarRangeKm;
      if (normalized > 1.0) {
        normalized = 1.0;
      } else if (normalized < 0.0 || isnan(normalized)) {
        continue;
      }

      double angleRad = deg2rad(radarContacts[i].bearing);
      double radius = normalized * (radarRadius - 3);
      int contactX = centerX + (int)round(sin(angleRad) * radius);
      int contactY = centerY - (int)round(cos(angleRad) * radius);

      double angleDiff = angularDifference(radarContacts[i].bearing, sweepAngle);
      if (!radarContacts[i].stale && angleDiff <= RADAR_SWEEP_WIDTH_DEG) {
        radarContacts[i].lastHighlightTime = now;
        if (setActiveContact(i)) {
          highlightChanged = true;
        }
      }

      if (radarContacts[i].lastHighlightTime == 0) {
        continue;
      }

      unsigned long sinceHighlight = now - radarContacts[i].lastHighlightTime;
      if (sinceHighlight > RADAR_FADE_DURATION_MS) {
        radarContacts[i].valid = false;
        continue;
      }

      float alpha = 1.0f - (float)sinceHighlight / (float)RADAR_FADE_DURATION_MS;
      uint16_t baseColor;
      if (radarContacts[i].inbound) {
        baseColor = radarContacts[i].stale
                        ? COLOR_RADAR_INBOUND
                        : (flashOn ? COLOR_RADAR_INBOUND : COLOR_BACKGROUND);
      } else {
        baseColor = COLOR_RADAR_CONTACT;
      }
      uint16_t fadedColor = fadeColor(baseColor, alpha);
      tft.fillCircle(contactX, contactY, 3, fadedColor);
    }

  }

  bool cleared = ensureActiveContactFresh(now);
  if (highlightChanged || cleared) {
    renderInfoPanel();
  }

  drawCompassLabels(tft, radarCenterX, radarCenterY, radarRadius);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);

  drawStatusBar();
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

  double radarRangeKm = currentRadarRangeKm();
  double alertRangeKm = currentAlertRangeKm();
  if (radarRangeKm <= 0.0) {
    radarRangeKm = 0.1;
  }
  if (alertRangeKm <= 0.0) {
    alertRangeKm = 0.1;
  }

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
      unsigned long fetchTime = millis();
      RadarContact previousContacts[MAX_RADAR_CONTACTS];
      bool previousMatched[MAX_RADAR_CONTACTS];
      int previousCount = radarContactCount;
      int previousActiveIndex = activeContactIndex;
      for (int i = 0; i < MAX_RADAR_CONTACTS; ++i) {
        previousContacts[i] = radarContacts[i];
        previousMatched[i] = false;
      }

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

        if (distance > radarRangeKm) {
          continue;
        }

        aircraftCount++;

        double bearingToHome = calculateBearing(USER_LAT, USER_LON, lat, lon);
        double groundSpeed = NAN;
        double track = NAN;
        double minutesToClosest = NAN;
        bool inbound = false;

        if (distance <= alertRangeKm) {
          inbound = true;
          minutesToClosest = 0.0;
        }

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

        if (!inbound && !isnan(track) && !isnan(groundSpeed) && groundSpeed > 0) {
          double toBase = fmod(bearingToHome + 180.0, 360.0);
          double angleDiff = fabs(track - toBase);
          if (angleDiff > 180.0) {
            angleDiff = 360.0 - angleDiff;
          }
          double crossTrack = distance * sin(deg2rad(angleDiff));
          double alongTrack = distance * cos(deg2rad(angleDiff));
          if (angleDiff < 90.0 && fabs(crossTrack) <= alertRangeKm && alongTrack >= 0) {
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

        String flight;
        if (plane.containsKey("flight")) {
          const char *flightStr = plane["flight"].as<const char*>();
          if (flightStr != nullptr) {
            flight = String(flightStr);
            flight.trim();
          }
        }

        int altitude = -1;
        if (plane.containsKey("alt_baro")) {
          JsonVariant alt = plane["alt_baro"];
          if (alt.is<int>()) {
            altitude = alt.as<int>();
          } else if (alt.is<const char*>()) {
            const char *altStr = alt.as<const char*>();
            if (altStr != nullptr && strcmp(altStr, "ground") == 0) {
              altitude = 0;
            }
          }
        }

        int matchIndex = -1;
        if (previousCount > 0) {
          if (flight.length() > 0) {
            for (int j = 0; j < previousCount; ++j) {
              if (previousMatched[j] || !previousContacts[j].valid) {
                continue;
              }
              String prevFlight = previousContacts[j].flight;
              prevFlight.trim();
              if (prevFlight.length() == 0) {
                continue;
              }
              if (prevFlight.equalsIgnoreCase(flight)) {
                matchIndex = j;
                break;
              }
            }
          }

          if (matchIndex < 0) {
            for (int j = 0; j < previousCount; ++j) {
              if (previousMatched[j] || !previousContacts[j].valid) {
                continue;
              }
              double distanceDiff = fabs(previousContacts[j].distanceKm - distance);
              double bearingDiff = angularDifference(previousContacts[j].bearing, bearingToHome);
              if (distanceDiff <= 1.0 && bearingDiff <= 12.0) {
                matchIndex = j;
                break;
              }
            }
          }
        }

        if (radarContactCount < MAX_RADAR_CONTACTS) {
          RadarContact &contact = radarContacts[radarContactCount++];
          contact.distanceKm = distance;
          contact.bearing = bearingToHome;
          contact.inbound = inbound;
          contact.valid = true;
          contact.stale = false;
          contact.lastHighlightTime = 0;
          contact.altitude = altitude;
          contact.groundSpeed = groundSpeed;
          contact.track = track;
          contact.minutesToClosest = minutesToClosest;
          if (matchIndex >= 0) {
            unsigned long previousHighlight = previousContacts[matchIndex].lastHighlightTime;
            if (previousHighlight != 0 && (fetchTime - previousHighlight) < RADAR_FADE_DURATION_MS) {
              contact.lastHighlightTime = previousHighlight;
            }
            previousMatched[matchIndex] = true;
            if (matchIndex == previousActiveIndex && contact.lastHighlightTime != 0) {
              setActiveContact(radarContactCount - 1);
            }
          }
          contact.flight = flight;
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

          best.altitude = altitude;
        }
      }

      for (int i = 0; i < previousCount && radarContactCount < MAX_RADAR_CONTACTS; ++i) {
        if (previousMatched[i] || !previousContacts[i].valid) {
          continue;
        }
        unsigned long lastHighlight = previousContacts[i].lastHighlightTime;
        if (lastHighlight == 0 || (fetchTime - lastHighlight) > RADAR_FADE_DURATION_MS) {
          continue;
        }
        RadarContact &contact = radarContacts[radarContactCount++];
        contact = previousContacts[i];
        contact.valid = true;
        contact.stale = true;
        if (i == previousActiveIndex && contact.lastHighlightTime != 0) {
          setActiveContact(radarContactCount - 1);
        }
      }

      closestAircraft = best;
      inboundAircraftCount = localInboundCount;
      if (best.valid) {
        lastSuccessfulFetch = fetchTime;
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

void drawStatusBar() {
  int iconWidth = WIFI_ICON_BARS * WIFI_ICON_BAR_WIDTH + (WIFI_ICON_BARS - 1) * WIFI_ICON_BAR_SPACING;
  int iconX = tft.width() - RADAR_MARGIN - iconWidth;
  int iconY = RADAR_MARGIN / 2;

  bool connected = WiFi.status() == WL_CONNECTED;
  int rssi = connected ? WiFi.RSSI() : -100;
  int barsActive = 0;

  if (connected) {
    if (rssi >= -55) {
      barsActive = 4;
    } else if (rssi >= -65) {
      barsActive = 3;
    } else if (rssi >= -75) {
      barsActive = 2;
    } else if (rssi >= -85) {
      barsActive = 1;
    } else {
      barsActive = 0;
    }
  }

  if (connected != lastWifiConnectedState || barsActive != lastWifiBars) {
    drawWifiIcon(iconX, iconY, barsActive, connected);
    lastWifiConnectedState = connected;
    lastWifiBars = barsActive;
  }
}

void drawWifiIcon(int x, int y, int barsActive, bool connected) {
  int iconWidth = WIFI_ICON_BARS * WIFI_ICON_BAR_WIDTH + (WIFI_ICON_BARS - 1) * WIFI_ICON_BAR_SPACING;
  tft.fillRect(x, y, iconWidth, WIFI_ICON_HEIGHT, COLOR_BACKGROUND);

  uint16_t activeColor = connected ? TFT_SKYBLUE : COLOR_RADAR_GRID;
  for (int i = 0; i < WIFI_ICON_BARS; ++i) {
    int barHeight = 6 + i * 3;
    int barX = x + i * (WIFI_ICON_BAR_WIDTH + WIFI_ICON_BAR_SPACING);
    int barY = y + WIFI_ICON_HEIGHT - barHeight;
    uint16_t color = (i < barsActive) ? activeColor : COLOR_RADAR_GRID;
    tft.fillRect(barX, barY, WIFI_ICON_BAR_WIDTH, barHeight, color);
  }
}

void configureButtons() {
  int buttonWidth = 0;
  if (BUTTON_COUNT > 0) {
    buttonWidth = (infoAreaWidth - BUTTON_SPACING * (BUTTON_COUNT - 1)) / BUTTON_COUNT;
    if (buttonWidth < 0) {
      buttonWidth = 0;
    }
  }

  int buttonY = buttonAreaY;
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    TouchButton &btn = buttons[i];
    btn.x = infoAreaX + i * (buttonWidth + BUTTON_SPACING);
    btn.y = buttonY;
    btn.w = buttonWidth;
    btn.h = BUTTON_HEIGHT;
    btn.state = false;
    if (i == 0) {
      btn.name = "Radar";
      btn.type = BUTTON_RADAR_RANGE;
    } else if (i == 1) {
      btn.name = "Alert";
      btn.type = BUTTON_ALERT_RANGE;
    } else {
      btn.name = "Button";
      btn.type = BUTTON_UNKNOWN;
    }
  }
}

void drawButtons() {
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    drawButton(i);
  }
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
}

void drawButton(int index) {
  if (index < 0 || index >= BUTTON_COUNT) {
    return;
  }

  TouchButton &btn = buttons[index];
  uint16_t fillColor = COLOR_BUTTON_INACTIVE;
  tft.fillRoundRect(btn.x, btn.y, btn.w, btn.h, 8, fillColor);
  tft.drawRoundRect(btn.x, btn.y, btn.w, btn.h, 8, COLOR_RADAR_OUTLINE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_TEXT, fillColor);
  tft.setTextSize(2);

  int centerX = btn.x + btn.w / 2;
  int centerY = btn.y + btn.h / 2;
  String title = btn.name ? String(btn.name) : String("Button");
  String value;

  if (btn.type == BUTTON_RADAR_RANGE) {
    value = String(currentRadarRangeKm(), 0) + " km";
  } else if (btn.type == BUTTON_ALERT_RANGE) {
    value = String(currentAlertRangeKm(), 0) + " km";
  } else {
    value = btn.state ? String("ON") : String("OFF");
  }

  tft.drawString(title, centerX, centerY - 10);
  tft.drawString(value, centerX, centerY + 12);
  tft.setTextSize(1);
}

bool readTouchPoint(int &screenX, int &screenY) {
  uint16_t rawX = 0;
  uint16_t rawY = 0;
  if (!tft.getTouchRaw(&rawX, &rawY)) {
    return false;
  }

  long calMinX = TOUCH_RAW_MIN_X;
  long calMaxX = TOUCH_RAW_MAX_X;
  long calMinY = TOUCH_RAW_MIN_Y;
  long calMaxY = TOUCH_RAW_MAX_Y;

#if TOUCH_SWAP_XY
  uint16_t rawSwap = rawX;
  rawX = rawY;
  rawY = rawSwap;
  long calSwap = calMinX;
  calMinX = calMinY;
  calMinY = calSwap;
  calSwap = calMaxX;
  calMaxX = calMaxY;
  calMaxY = calSwap;
#endif

#if TOUCH_INVERT_X
  long calSwap = calMinX;
  calMinX = calMaxX;
  calMaxX = calSwap;
#endif

#if TOUCH_INVERT_Y
  long calSwap = calMinY;
  calMinY = calMaxY;
  calMaxY = calSwap;
#endif

  long mappedX = rawX;
  long mappedY = rawY;

  if (calMaxX != calMinX) {
    mappedX = map((long)rawX, calMinX, calMaxX, 0, (long)tft.width() - 1);
  } else {
    mappedX = rawX % tft.width();
  }

  if (calMaxY != calMinY) {
    mappedY = map((long)rawY, calMinY, calMaxY, 0, (long)tft.height() - 1);
  } else {
    mappedY = rawY % tft.height();
  }

  mappedX = constrain(mappedX, 0, (long)tft.width() - 1);
  mappedY = constrain(mappedY, 0, (long)tft.height() - 1);

  screenX = (int)mappedX;
  screenY = (int)mappedY;
  return true;
}

bool touchInCompassLabel(const char *label, int touchX, int touchY) {
  if (!compassLabelBoundsValid || label == nullptr) {
    return false;
  }

  for (int i = 0; i < COMPASS_LABEL_COUNT; ++i) {
    const CompassLabelBounds &bounds = compassLabelBounds[i];
    if (bounds.label == nullptr || strcmp(bounds.label, label) != 0) {
      continue;
    }

    int right = bounds.x + bounds.w - 1;
    int bottom = bounds.y + bounds.h - 1;
    if (touchX >= bounds.x && touchX <= right && touchY >= bounds.y && touchY <= bottom) {
      return true;
    }
  }

  return false;
}

void rotateRadarOrientation() {
  displayRotation = (displayRotation + 1) % 4;
  tft.setRotation(displayRotation);
  compassLabelBoundsValid = false;
  drawStaticLayout();
}

void handleTouch() {
  int touchX = 0;
  int touchY = 0;
  if (!readTouchPoint(touchX, touchY)) {
    return;
  }

  unsigned long now = millis();
  if (now - lastTouchTime < TOUCH_DEBOUNCE_MS) {
    return;
  }
  lastTouchTime = now;

  bool handled = false;
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    TouchButton &btn = buttons[i];
    if (touchX >= btn.x && touchX <= btn.x + btn.w && touchY >= btn.y && touchY <= btn.y + btn.h) {
      handleRangeButton(btn.type);
      handled = true;
      break;
    }
  }

  if (!handled && touchInCompassLabel("N", touchX, touchY)) {
    rotateRadarOrientation();
    return;
  }
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
}

double angularDifference(double a, double b) {
  double diff = fabs(fmod(a - b + 540.0, 360.0) - 180.0);
  return diff;
}

uint16_t fadeColor(uint16_t color, float alpha) {
  if (alpha <= 0.0f) {
    return COLOR_BACKGROUND;
  }
  if (alpha >= 1.0f) {
    return color;
  }

  uint8_t r = (color >> 11) & 0x1F;
  uint8_t g = (color >> 5) & 0x3F;
  uint8_t b = color & 0x1F;

  r = (uint8_t)round(r * alpha);
  g = (uint8_t)round(g * alpha);
  b = (uint8_t)round(b * alpha);

  return (uint16_t)((r << 11) | (g << 5) | b);
}
