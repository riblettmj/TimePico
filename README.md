# TimePico — GPS/PPS NTP Server + Mini-Rack Clock (Raspberry Pi Pico W / Pico 2 W)

TimePico is a self-contained, **fully local** network time appliance built for air‑gapped or “no cloud” environments. It uses a GNSS receiver (GPS) — optionally disciplined with **PPS** — to keep accurate time, serves **NTP on UDP/123**, and displays the current time on a compact **4‑digit HT16K33 7‑segment display**.

It also includes a lightweight configuration portal (with a short availability window in STA mode + AP fallback) and a safe “staged” OTA update mechanism.

## Inspiration & credit

This project was inspired by (and built in the spirit of) these excellent projects/videos:

- Jeff Geerling — **Raspberry Pi Pico Mini Rack GPS Clock** (blog):  
  https://www.jeffgeerling.com/blog/2026/pico-gps-clock-mini-rack/  
- Jeff Geerling — **The most accurate clock in a mini rack** (YouTube):  
  https://www.youtube.com/watch?v=E5qA4fgdS28  
- Jeff Geerling — Time Pi repository:  
  https://github.com/geerlingguy/time-pi  

Additional inspiration for the “Pico + GPS as a stratum‑1 style server” idea:

- Gary Explains — **Build a Stratum 1 Time Server Using a Raspberry Pi Pico** (YouTube):  
  https://www.youtube.com/watch?v=pyVCHX4H7bM  
- Gary Explains examples repo (includes `ublox_tinygps_ntp_server_pico`):  
  https://github.com/garyexplains/examples/tree/master/ublox_tinygps_ntp_server_pico  

## Features (high level)

- **GPS time acquisition** via NMEA over UART
- Optional **PPS input** for tighter alignment and better stability
- **NTP server (UDP/123)** with:
  - token‑bucket rate limiting (supports common client “bursts” like iBurst)
  - optional Kiss‑o’-Death RATE responses when limited
  - optional CIDR allowlist
  - safety gating: the server only responds when the clock is considered stable and not “suspect”
- **Peer cross-check (“voting”)** between multiple TimePico units:
  - authenticated peer messages (pre‑shared key)
  - outlier detection + strike counting
  - marks a unit “suspect” if it diverges beyond thresholds
- **7‑segment time display** with status indicators:
  - `----` while time is not yet stabilized
  - blinking colon when GPS+PPS are locked
  - solid colon when running without a current lock
  - decimal‑point “throbber” during GPS acquisition/stabilization
  - right‑most decimal flashes quickly on each NTP query (optional debug)
- **Config portal**
  - STA (WiFi connected): portal enabled for the first 5 minutes after boot (then shuts off until reboot)
  - WiFi failure: AP fallback (open network) with a short PIN shown on the display
- **Safe OTA updates**
  - staged upload to `app_next.py`
  - previous version preserved as `app_prev.py`
  - the old version is not removed unless the new version starts successfully
  - optional “Fetch from URL” workflow (your browser fetches a GitHub raw URL, then uploads to the device)

## Repository layout

This repo is designed to use a tiny stable loader + a replaceable application:

- `main.py` — **stable loader**
- `app.py` — **TimePico application** (the NTP server, portal, GPS, display, etc.)

On the Pico filesystem, place both files in the root.

The portal will also create/update:
- `timepico_cfg.json` — configuration file

## Hardware

Typical build:

- Raspberry Pi **Pico W** or **Pico 2 W**
- GNSS/GPS module (UART NMEA) with optional PPS output (or a board where PPS can be wired)
- HT16K33 4‑digit 7‑segment I2C display (e.g., Adafruit 0.56" backpack)
- External GPS antenna (recommended for indoor/rack use)

### Wiring (defaults in firmware)

| Function | Pico GPIO | Notes |
|---|---:|---|
| I2C SDA | GPIO4 | HT16K33 display |
| I2C SCL | GPIO5 | HT16K33 display |
| GPS UART TX (Pico → GPS RX) | GPIO0 | UART0 TX |
| GPS UART RX (Pico ← GPS TX) | GPIO1 | UART0 RX |
| PPS input | GPIO16 | optional (enable/disable in firmware) |

> PPS note: not all GPS “HATs” expose PPS to a header pin. If your board doesn’t provide PPS, set `ENABLE_PPS = False` in the code.

## Getting started

### 1) Install MicroPython on the Pico

Official Raspberry Pi docs (drag‑and‑drop UF2):
- https://www.raspberrypi.com/documentation/microcontrollers/micropython.html

You can also download UF2s directly from MicroPython:
- Pico W: https://micropython.org/download/RPI_PICO_W/
- Pico 2 W: https://micropython.org/download/RPI_PICO2_W/

Basic UF2 install steps:
1. Hold **BOOTSEL** while plugging the Pico into USB.
2. The Pico appears as a drive named **RPI-RP2**.
3. Drag the MicroPython `.uf2` onto that drive; the Pico reboots into MicroPython.

### 2) Copy files to the Pico using Thonny

Raspberry Pi’s “Getting started with Pico” + Thonny guide:
- https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico

Steps:
1. Install/open **Thonny**
2. In Thonny: **Tools → Options → Interpreter**
3. Select a Pico interpreter (e.g., “MicroPython (Raspberry Pi Pico)”)
4. Open Thonny’s **Files** pane and copy:
   - `main.py` (loader)
   - `app.py` (application)
   to the Pico’s filesystem root.
5. Reboot/power-cycle the Pico.

## First boot & configuration portal

### Normal (WiFi STA) boot
- The device attempts to connect to WiFi using stored credentials.
- If it connects successfully:
  - the **config portal is available for 5 minutes after boot**
  - browse to: `http://<device-ip>/`
  - the device will show a **4‑digit PIN** on the display when the portal is accessed (PIN is required by default).

### AP fallback (WiFi failed)
If WiFi fails shortly after boot, the device starts an **open AP**:
- SSID: `TimePico-Setup-####`
- Device IP: `192.168.4.1`
- Browse to: `http://192.168.4.1/`
- A 4‑digit PIN is shown on the display.

> The AP is open (no password) on purpose to keep memory usage low. The PIN is intended to prevent casual drive‑by edits, not to be strong security.

## Configuration settings (via portal / `timepico_cfg.json`)

The portal writes `timepico_cfg.json` and overrides firmware defaults.

### Key settings + defaults

| Setting | Key in `timepico_cfg.json` | Default |
|---|---|---|
| Hostname | `hostname` | `"timepico"` |
| 24h / 12h display | `time_display_24h` | `true` |
| UTC offset hours | `tz_offset_hours` | `0` |
| Day brightness (06:00–17:59) | `brightness_day` | `2` |
| Night brightness (18:00–05:59) | `brightness_night` | `0` |
| WiFi SSID | `wifi_ssid` | `"YOUR_SSID"` |
| WiFi password | `wifi_password` | `"YOUR_PASSWORD"` |
| Use static IP | `use_static_ip` | `true` |
| Static IP tuple | `static_ip` | `["192.168.10.10","255.255.255.0","192.168.10.1","192.168.10.1"]` |
| Allowed client CIDRs | `allowed_cidrs` | `["192.168.10.0/24"]` |
| Peer TimePicos | `peers` | `["192.168.10.11","192.168.10.12"]` |
| Drop NTP replies when unstable | `drop_unstable_ntp` | `true` |

### Example `timepico_cfg.json`

```json
{
  "net_mode": "wifi",
  "wifi_ssid": "MyLabWiFi",
  "wifi_password": "correct-horse-battery-staple",
  "use_static_ip": true,
  "static_ip": ["192.168.10.10", "255.255.255.0", "192.168.10.1", "192.168.10.1"],

  "hostname": "timepico-rack1",
  "time_display_24h": true,
  "tz_offset_hours": -6,

  "brightness_day": 4,
  "brightness_night": 0,

  "allowed_cidrs": ["192.168.10.0/24", "192.168.20.0/24"],
  "peers": ["192.168.10.11", "192.168.10.12"],

  "drop_unstable_ntp": true
}
