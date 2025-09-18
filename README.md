# ClosestPlane ESP32

An Arduino-compatible sketch for the Freenove ESP32 4.0" display board (FNK0103) that connects to a dump1090 server and renders a simple radar status panel on the integrated TFT. The display shows the closest aircraft, distance, bearing, altitude, track, and connection status so you can keep an eye on nearby traffic at a glance.

## Required Libraries

Install the following libraries in the Arduino IDE (or via `arduino-cli`) before compiling:

- **ESP32 board support** (`esp32` by Espressif) – provides WiFi and HTTPClient functionality.
- **ArduinoJson** – parses the `aircraft.json` feed from dump1090.
- **TFT_eSPI** – drives the 4" IPS TFT included with the Freenove board. Configure the library's `User_Setup_Select.h`/`User_Setup.h` files for the FNK0103 before compiling.

## Hardware Connections

The sketch targets the Freenove FNK0103 kit where the ESP32, TFT backplane and touch buttons share a common PCB. No external wiring is required beyond providing power and ensuring the TFT_eSPI library is configured for the panel that ships with the kit. If you adapt the sketch to another ESP32 + TFT combination, update the TFT_eSPI pin definitions accordingly.

## Setup
1. Update `config.h` with your WiFi credentials, dump1090 server address and your latitude/longitude.
2. Ensure the libraries above are installed in the Arduino IDE and that TFT_eSPI is configured for the Freenove display.
3. Open `freenove.ino` in the Arduino IDE, select your ESP32 board and the correct port.
4. Compile and upload.

## Operation
- The TFT lists the closest aircraft detected within the configured radius and highlights whether it is inbound.
- Connection, WiFi strength and fetch timing are summarised near the bottom of the display.
- Aircraft distance, bearing, altitude, track and estimated arrival (for inbound flights) refresh every five seconds while the ESP32 maintains a WiFi link to the dump1090 server.

## Building in a Codex/Codespace Environment

The following setup script prepares a Codex/Codespace container with all required tools and
libraries for this project. Run it in your container before compiling:
```bash
#!/bin/bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "==> Base packages"
apt-get update
apt-get install -y --no-install-recommends \
  curl ca-certificates git python3 tar xz-utils

# --- Proxy handling ---
# Mirror the proxy you see in Codex logs (http://proxy:8080) for all tools.
HTTP_PROXY="${HTTP_PROXY:-${http_proxy:-}}"
HTTPS_PROXY="${HTTPS_PROXY:-${https_proxy:-$HTTP_PROXY}}"
NO_PROXY="${NO_PROXY:-${no_proxy:-localhost,127.0.0.1,::1}}"
export HTTP_PROXY HTTPS_PROXY NO_PROXY
# Some tools only read lowercase:
export http_proxy="${HTTP_PROXY:-}"
export https_proxy="${HTTPS_PROXY:-}"
export no_proxy="${NO_PROXY:-}"

echo "HTTP_PROXY=${HTTP_PROXY:-<unset>}"
echo "HTTPS_PROXY=${HTTPS_PROXY:-<unset>}"
echo "NO_PROXY=${NO_PROXY}"

# --- Prefer IPv4 to avoid IPv6 'network is unreachable' ---
if ! grep -q '^precedence ::ffff:0:0/96 100' /etc/gai.conf 2>/dev/null; then
  echo 'precedence ::ffff:0:0/96 100' >> /etc/gai.conf
fi

# --- Helper: retry with backoff ---
retry () {
  local attempts="$1"; shift
  local sleep_s="$1"; shift
  local n=1
  until "$@"; do
    if [[ $n -ge $attempts ]]; then
      echo "Command failed after $n attempts: $*" >&2
      return 1
    fi
    echo "Retry $n/$attempts failed. Sleeping ${sleep_s}s…"
    n=$((n+1))
    sleep "$sleep_s"
  done
}

echo "==> Installing arduino-cli"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
install -m 0755 bin/arduino-cli /usr/local/bin/arduino-cli
rm -rf bin
arduino-cli version

echo "==> arduino-cli config"
arduino-cli config init --overwrite

# Boards index + proxy (this is the only network key we need)
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

if [[ -n "${HTTPS_PROXY}" || -n "${HTTP_PROXY}" ]]; then
  arduino-cli config set network.proxy "${HTTPS_PROXY:-$HTTP_PROXY}"
fi

# Optional: slightly longer socket timeout (older CLI versions may ignore this; safe if it errors)
arduino-cli config set network.socket_timeout 60s || true

echo "==> Updating indexes (via proxy, with retries)"
retry 5 5 arduino-cli core update-index

echo "==> Installing ESP32 core"
retry 5 5 arduino-cli core install esp32:esp32

echo "==> Installing libraries"
retry 5 5 arduino-cli lib install "ArduinoJson"
retry 5 5 arduino-cli lib install "TFT_eSPI"

echo "✅ Setup complete. Compile with:"
echo "   arduino-cli compile --fqbn esp32:esp32:esp32 sketches/freenove"
```

The command completes successfully when the required libraries (`ArduinoJson`, `TFT_eSPI`, etc.) are installed. A thin wrapper
sketch lives in `sketches/freenove` so that `arduino-cli` sees a folder/file pair with matching names while still keeping the
primary source at the repository root for IDE users.


