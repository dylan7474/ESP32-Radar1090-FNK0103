#pragma once

#include <ArduinoJson.h>
#include <ctype.h>
#include <string.h>

#include "aircraft_icons.h"

// Helpers to classify aircraft data from dump1090 into icon categories.
inline bool aircraftStringContainsIgnoreCase(const char *haystack, const char *needle) {
  if (!haystack || !needle || *needle == '\0') {
    return false;
  }
  size_t needleLen = strlen(needle);
  for (const char *p = haystack; *p; ++p) {
    size_t i = 0;
    while (i < needleLen && p[i] && toupper((unsigned char)p[i]) == toupper((unsigned char)needle[i])) {
      ++i;
    }
    if (i == needleLen) {
      return true;
    }
  }
  return false;
}

inline bool aircraftIconIsSpecial(AircraftIconId icon) {
  return icon == AIRCRAFT_ICON_ROTOR || icon == AIRCRAFT_ICON_GLIDER ||
         icon == AIRCRAFT_ICON_LIGHTERTHANAIR || icon == AIRCRAFT_ICON_DRONEUAV;
}

inline AircraftIconId aircraftIconFromWtc(JsonVariantConst wtcVar) {
  if (wtcVar.isNull()) {
    return AIRCRAFT_ICON_MEDIUM;
  }
  if (wtcVar.is<int>() || wtcVar.is<long>() || wtcVar.is<unsigned int>() || wtcVar.is<unsigned long>()) {
    int value = wtcVar.as<int>();
    if (value <= 0) {
      return AIRCRAFT_ICON_MEDIUM;
    }
    switch (value) {
      case 1:
        return AIRCRAFT_ICON_LIGHT;
      case 2:
        return AIRCRAFT_ICON_MEDIUM;
      case 3:
        return AIRCRAFT_ICON_HEAVY;
      default:
        return AIRCRAFT_ICON_MEDIUM;
    }
  }
  if (wtcVar.is<const char*>()) {
    const char *wtcStr = wtcVar.as<const char*>();
    if (!wtcStr || !*wtcStr) {
      return AIRCRAFT_ICON_MEDIUM;
    }
    char c = toupper((unsigned char)wtcStr[0]);
    if (c == 'L') {
      return AIRCRAFT_ICON_LIGHT;
    }
    if (c == 'H' || c == 'J') {
      return AIRCRAFT_ICON_HEAVY;
    }
    if (c == 'M') {
      return AIRCRAFT_ICON_MEDIUM;
    }
  }
  return AIRCRAFT_ICON_MEDIUM;
}

inline AircraftIconId aircraftIconFromCategory(const char *category) {
  if (!category || !*category) {
    return AIRCRAFT_ICON_MEDIUM;
  }
  char major = toupper((unsigned char)category[0]);
  char minor = category[1];
  if (major == 'A') {
    switch (minor) {
      case '1':
      case '2':
        return AIRCRAFT_ICON_LIGHT;
      case '3':
        return AIRCRAFT_ICON_MEDIUM;
      case '4':
      case '5':
        return AIRCRAFT_ICON_HEAVY;
      case '7':
        return AIRCRAFT_ICON_ROTOR;
      default:
        return AIRCRAFT_ICON_MEDIUM;
    }
  } else if (major == 'B') {
    switch (minor) {
      case '1':
        return AIRCRAFT_ICON_GLIDER;
      case '2':
        return AIRCRAFT_ICON_LIGHTERTHANAIR;
      case '4':
        return AIRCRAFT_ICON_DRONEUAV;
      case '6':
        return AIRCRAFT_ICON_LIGHT;
      default:
        return AIRCRAFT_ICON_MEDIUM;
    }
  }
  return AIRCRAFT_ICON_MEDIUM;
}

inline AircraftIconId aircraftIconFromDescriptor(const char *text) {
  if (!text || !*text) {
    return AIRCRAFT_ICON_MEDIUM;
  }
  if (aircraftStringContainsIgnoreCase(text, "HELI") ||
      aircraftStringContainsIgnoreCase(text, "ROTOR")) {
    return AIRCRAFT_ICON_ROTOR;
  }
  if (aircraftStringContainsIgnoreCase(text, "GLIDER") ||
      aircraftStringContainsIgnoreCase(text, "SAILPLANE")) {
    return AIRCRAFT_ICON_GLIDER;
  }
  if (aircraftStringContainsIgnoreCase(text, "BALLOON") ||
      aircraftStringContainsIgnoreCase(text, "AIRSHIP") ||
      aircraftStringContainsIgnoreCase(text, "BLIMP")) {
    return AIRCRAFT_ICON_LIGHTERTHANAIR;
  }
  if (aircraftStringContainsIgnoreCase(text, "DRONE") ||
      aircraftStringContainsIgnoreCase(text, "UAV") ||
      aircraftStringContainsIgnoreCase(text, "UAS") ||
      aircraftStringContainsIgnoreCase(text, "UNMANNED")) {
    return AIRCRAFT_ICON_DRONEUAV;
  }
  if (aircraftStringContainsIgnoreCase(text, "ULTRA")) {
    return AIRCRAFT_ICON_LIGHT;
  }
  if (aircraftStringContainsIgnoreCase(text, "HEAVY") ||
      aircraftStringContainsIgnoreCase(text, "SUPER")) {
    return AIRCRAFT_ICON_HEAVY;
  }
  return AIRCRAFT_ICON_MEDIUM;
}

inline AircraftIconId determineAircraftIcon(JsonObjectConst plane) {
  AircraftIconId icon = aircraftIconFromWtc(plane["wtc"]);

  const char *categoryStr = plane["category"].is<const char*>() ? plane["category"].as<const char*>() : nullptr;
  AircraftIconId categoryIcon = aircraftIconFromCategory(categoryStr);
  if (categoryIcon != AIRCRAFT_ICON_MEDIUM) {
    if (aircraftIconIsSpecial(categoryIcon)) {
      icon = categoryIcon;
    } else if (categoryIcon == AIRCRAFT_ICON_HEAVY) {
      icon = AIRCRAFT_ICON_HEAVY;
    } else if (icon == AIRCRAFT_ICON_MEDIUM) {
      icon = categoryIcon;
    }
  }

  AircraftIconId descriptorIcon = aircraftIconFromDescriptor(plane["type"].is<const char*>() ? plane["type"].as<const char*>() : nullptr);
  if (descriptorIcon == AIRCRAFT_ICON_MEDIUM) {
    descriptorIcon = aircraftIconFromDescriptor(plane["desc"].is<const char*>() ? plane["desc"].as<const char*>() : nullptr);
  }
  if (descriptorIcon != AIRCRAFT_ICON_MEDIUM) {
    icon = descriptorIcon;
  }

  if (icon >= AIRCRAFT_ICON_COUNT) {
    icon = AIRCRAFT_ICON_MEDIUM;
  }
  return icon;
}

inline const AircraftIcon &aircraftIconForId(AircraftIconId id) {
  uint8_t index = static_cast<uint8_t>(id);
  if (index >= AIRCRAFT_ICON_COUNT) {
    index = static_cast<uint8_t>(AIRCRAFT_ICON_MEDIUM);
  }
  return AIRCRAFT_ICONS[index];
}
