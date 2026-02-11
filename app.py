# TimePico PPS NTP Server (Hardened) + Peer Voting + 7-seg Clock
# Copyright 2026 - Matthew J. Riblett
#
# MicroPython (RP2040 / RP2350), intended for Raspberry Pi Pico W / Pico 2 W
#
# Minimal dependencies: MicroPython stdlib only.
#
# IMPORTANT:
# - This file assumes BLE is NOT used; it is your PPS/GPS NTP appliance.
# - For wired Ethernet W5x00, AP fallback does not apply; portal still opens for the first window.
#
# File on Pico:
#   app.py
# Config file (created/updated by portal):
#   timepico_cfg.json

# ---------------------------- DEFAULT CONFIG -----------------------------
# (Values below act as defaults; timepico_cfg.json overrides them.)

# Display (HT16K33 4-digit backpack)
I2C_ID = 0
I2C_SDA = 4
I2C_SCL = 5
HT16K33_ADDR = 0x70
DISPLAY_BRIGHTNESS_DAY = 2       # 0..15, used 06:00-17:59 (local)
DISPLAY_BRIGHTNESS_NIGHT = 0     # 0..15, used 18:00-05:59 (local)

# Firmware version (shown on portal pages)
TIMEPICO_VERSION = "v16.6.6"

# Default URL used to pre-populate the "Update from URL" field (Official Github Repository)
DEFAULT_UPDATE_URL = "https://raw.githubusercontent.com/riblettmj/TimePico/refs/heads/master/app.py"

# Time display format
TIME_DISPLAY_24H = True          # True = 24-hour; False = 12-hour (no AM/PM indicator)

# Hostname advertised on the network (best-effort DHCP hostname)
HOSTNAME = "timepico"           # portal can override; blank => auto-generate

# Time zone (numeric UTC offset; DST is not automatically handled)
# Examples: 0, -6, +1, 5.5 (for UTC+05:30). Stored in config as tz_offset_hours.
TZ_OFFSET_HOURS = 0
TZ_OFFSET_S = 0

# GPS UART (Waveshare uses UART0 by default: GPIO0/1)
GPS_UART_ID = 0
GPS_BAUD = 9600
GPS_TX = 0                       # Pico GPIO0 -> GPS RX
GPS_RX = 1                       # Pico GPIO1 <- GPS TX

# PPS input
ENABLE_PPS = True
PPS_GPIO = 16                    # Pico GPIO16 (physical pin 21)
PPS_PULLDOWN = True

# Network interface
NET_MODE = "wifi"                # "wifi" (Pico W/2W) or "wiznet" (W5x00)

# WiFi defaults (override via portal)
WIFI_SSID = "YOUR_SSID"
WIFI_PASSWORD = "YOUR_PASSWORD"
USE_STATIC_IP = False
STATIC_IP = ("127.0.0.1", "255.255.255.0", "127.0.0.1", "127.0.0.1")  # ip,mask,gw,dns

# WIZnet defaults (override via portal if you wish; portal primarily targets wifi)
WIZ_SPI_ID = 0
WIZ_SCK = 18
WIZ_MOSI = 19
WIZ_MISO = 20                  # NOTE: avoid conflict with PPS on GPIO16
WIZ_CS = 17
WIZ_RST = 15                   # wire W5x00 reset here (or change)

# NTP
NTP_PORT = 123
ALLOWED_CIDRS = ["192.168.0.0/24"]   # empty list => allow all
# Token-bucket rate limiting (permits iBurst while limiting abuse)
NTP_RL_BUCKET_CAP = 12            # allow bursts up to 12 replies per client
NTP_RL_REFILL_MS = 150            # refill 1 token per 150ms (~6.6 replies/sec sustained)
NTP_SEND_KOD_ON_LIMIT = True      # send Kiss-o'-Death RATE when limited (else drop)
NTP_MIN_INTERVAL_MS = 0           # legacy; unused when token bucket enabled
DROP_UNSTABLE_NTP = True

# NTP debug indicator: flash right-most decimal point on each client request
NTP_DEBUG_FLASH_ENABLE = True
NTP_DEBUG_FLASH_STEP_MS = 80       # toggle interval; 6 toggles => 3 flashes
NTP_DEBUG_FLASH_TOGGLES = 6
NTP_DEBUG_FLASH_QUEUE_CAP = 6
              # if False: reply stratum-16 alarm instead of dropping

# Peer cross-check protocol
PEER_PORT = 4567
PEERS = ["192.168.0.101", "192.168.0.102"]
PEER_PSK = b"CHANGE_ME_TO_A_RANDOM_32BYTE_KEY________"
PEER_PING_INTERVAL_MS = 5000
PEER_SAMPLE_MAX_AGE_MS = 20000
PEER_MAX_DELAY_US = 50_000

# Throbber indicator (GPS acquisition / stabilization)
THROBBER_STEP_MS = 200

# Voting / interference thresholds
MIN_PEERS_FOR_VOTE = 2                # for 3-node cluster, keep 2
VOTE_CONFIRM_US = 250_000
ALERT_OFFSET_US = 250_000
PEER_OUTLIER_US = 500_000
OUTLIER_STRIKES = 3
SUSPECT_CLEAR_GOOD_S = 60

# PPS / stabilization thresholds
PPS_TIMEOUT_MS = 1500
GPS_RMC_TIMEOUT_MS = 3000
RMC_ALIGN_WINDOW_US = 900_000
PPS_PERIOD_VALID_MIN_US = 900_000
PPS_PERIOD_VALID_MAX_US = 1_100_000
PPS_LOCK_TOL_US = 800
PPS_LOCK_COUNT = 8
GPS_STABLE_S = 30
GPS_MAX_STEP_US = 1_000_000

# Holdover policy
MAX_HOLDOVER_S = 6 * 3600
MAX_HOLDOVER_UNCERTAINTY_US = 2_000_000
HOLDOVER_PPM_MIN = 20
HOLDOVER_PPM_MARGIN = 10

# Config portal
ENABLE_CONFIG_PORTAL = True
# Require a 4-digit PIN for portal access in STA/wired mode (recommended).
PORTAL_PIN_REQUIRED_STA = True
# Require a 4-digit PIN for portal access in AP fallback mode.
PORTAL_PIN_REQUIRED_AP = True
# Show PIN on the display for this long after a portal connection is attempted.
PORTAL_PIN_DISPLAY_ON_ACCESS_MS = 30000
# Show PIN briefly at boot when portal starts (helps initial provisioning).
PORTAL_PIN_DISPLAY_ON_BOOT_MS = 8000
CONFIG_PORTAL_PORT = 80
CONFIG_PORTAL_WINDOW_S = 300          # 5 minutes after boot (STA/wired)
WIFI_CONNECT_TIMEOUT_S = 12           # "shortly after boot"
AP_FALLBACK_ENABLE = True
AP_SSID_PREFIX = "TimePico-"
AP_IP = ("192.168.4.1", "255.255.255.0", "192.168.4.1", "192.168.4.1")


# Portal HTML/CSS (compact to reduce RAM/flash usage)
PORTAL_CSS = b"body{margin:0;font:16px Arial,sans-serif;background:#f5f7fb;color:#111}"\
b".bar{background:#0b3d91;color:#fff;padding:10px 12px;font-weight:700}"\
b".v{float:right;font-size:12px;opacity:.85;font-weight:600}"\
b".w{padding:12px}.c{max-width:740px;background:#fff;border:1px solid #d7e0ee;border-radius:8px;padding:12px}"\
b"label{display:block;font-weight:700;margin:10px 0 4px}"\
b"input,select{width:100%;max-width:520px;padding:8px 10px;border:1px solid #b8c6d8;border-radius:6px;font-size:16px}"\
b"input[disabled],select[disabled]{opacity:.55}"\
b".b{padding:10px 12px;border:0;border-radius:6px;color:#fff;font-weight:800;font-size:16px}"\
b".blue{background:#0b3d91}.red{background:#b00020}"\
b".m{color:#334155;font-size:13px;margin:0 0 10px}.s{color:#475569;font-size:12px;margin-top:10px}.e{color:#b00020;font-weight:700}"

# Common HTML fragments
PORTAL_H1 = b"<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
PORTAL_T1 = b"<title>"
PORTAL_T2 = b"</title><style>"
PORTAL_H2 = b"</style></head><body><div class='bar'>"
PORTAL_V1 = b"<span class='v'>"
PORTAL_H3 = b"</span></div><div class='w'><div class='c'>"
PORTAL_END = b"</div></div></body></html>"

# Firmware update (OTA) - upload new app.py via portal (staged + safe swap on reboot)
FW_UPDATE_ENABLE = True
FW_CANDIDATE_FILE = "app_next.py"
FW_PENDING_FILE = "update_pending.txt"
FW_MAX_UPLOAD_BYTES = 300000   # safety cap; set 0 to disable cap


# Watchdog
ENABLE_WDT = True
WDT_TIMEOUT_MS = 8000

DEBUG = True
# ------------------------- END DEFAULT CONFIG ----------------------------

import gc, struct, socket, time

try:
    import network
except ImportError:
    network = None

try:
    import hashlib
except ImportError:
    import uhashlib as hashlib  # type: ignore

try:
    import ubinascii
except ImportError:
    import binascii as ubinascii  # type: ignore

import machine
from machine import Pin, I2C, UART
try:
    from machine import WDT
except ImportError:
    WDT = None

NTP_DELTA = 2208988800  # 1900->1970
CFG_FILE = "timepico_cfg.json"

def log(*a):
    if DEBUG:
        print("[timepico]", *a)

def ticks_us(): return time.ticks_us()
def ticks_ms(): return time.ticks_ms()
def ticks_diff(a,b): return time.ticks_diff(a,b)
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def median(vals):
    vals = list(vals)
    if not vals: return None
    vals.sort()
    n = len(vals); m = n//2
    return vals[m] if (n & 1) else (vals[m-1] + vals[m])//2

def unique_u32():
    try: uid = machine.unique_id()
    except Exception: uid = b"fallback"
    return ubinascii.crc32(uid) & 0xFFFFFFFF

def rand_u64():
    try:
        import urandom
        return (urandom.getrandbits(32) << 32) | urandom.getrandbits(32)
    except Exception:
        t = ticks_us() & 0xFFFFFFFF
        return (t << 32) | (ticks_ms() & 0xFFFFFFFF)

# ---------------- Config load/save ----------------
def _load_cfg():
    try:
        import json
        with open(CFG_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return {}

def _save_cfg(cfg: dict):
    try:
        import json
        tmp = CFG_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cfg, f)
        # Best-effort atomic replace
        try:
            import os
            os.replace(tmp, CFG_FILE)
        except Exception:
            with open(CFG_FILE, "w") as f:
                json.dump(cfg, f)
    except Exception as e:
        log("cfg save failed:", e)

CFG = _load_cfg()

def cfg_get(key, default):
    try:
        if key in CFG:
            return CFG[key]
    except Exception:
        pass
    return default

# Apply cfg overrides
NET_MODE = str(cfg_get("net_mode", NET_MODE))

WIFI_SSID = str(cfg_get("wifi_ssid", WIFI_SSID))
WIFI_PASSWORD = str(cfg_get("wifi_password", WIFI_PASSWORD))
USE_STATIC_IP = bool(cfg_get("use_static_ip", USE_STATIC_IP))
STATIC_IP = tuple(cfg_get("static_ip", STATIC_IP))

# Hostname / time format / timezone selection
HOSTNAME = str(cfg_get("hostname", HOSTNAME)).strip()
TIME_DISPLAY_24H = bool(cfg_get("time_display_24h", TIME_DISPLAY_24H))

# Display brightness schedule
def _parse_brightness(v, default):
    try:
        b = int(float(str(v).strip()))
    except Exception:
        b = int(default)
    return clamp(b, 0, 15)

DISPLAY_BRIGHTNESS_DAY = _parse_brightness(cfg_get("brightness_day", DISPLAY_BRIGHTNESS_DAY), DISPLAY_BRIGHTNESS_DAY)
DISPLAY_BRIGHTNESS_NIGHT = _parse_brightness(cfg_get("brightness_night", DISPLAY_BRIGHTNESS_NIGHT), DISPLAY_BRIGHTNESS_NIGHT)

# Timezone offset (hours; may be fractional). DST is not automatic.
def _parse_tz_offset_hours(v):
    try:
        if isinstance(v, (int, float)):
            return float(v)
        s = str(v).strip()
        if not s:
            return 0.0
        # accept formats like '+05:30' or '-03:45'
        if ':' in s:
            sign = -1 if s.startswith('-') else 1
            s2 = s.lstrip('+-')
            hh, mm = s2.split(':', 1)
            return sign * (int(hh) + (int(mm) / 60.0))
        return float(s)
    except Exception:
        return 0.0

TZ_OFFSET_HOURS = _parse_tz_offset_hours(cfg_get("tz_offset_hours", TZ_OFFSET_HOURS))
TZ_OFFSET_S = int(TZ_OFFSET_HOURS * 3600)

# Auto-generate hostname if blank
def _sanitize_hostname(h):
    h = (h or "").strip().lower()
    out = []
    for ch in h:
        if ('a' <= ch <= 'z') or ('0' <= ch <= '9') or ch == '-':
            out.append(ch)
    h = ''.join(out)
    # must start/end alnum
    while h and not (('a' <= h[0] <= 'z') or ('0' <= h[0] <= '9')):
        h = h[1:]
    while h and not (('a' <= h[-1] <= 'z') or ('0' <= h[-1] <= '9')):
        h = h[:-1]
    if len(h) > 32:
        h = h[:32]
    return h

HOSTNAME = _sanitize_hostname(HOSTNAME)
if not HOSTNAME:
    HOSTNAME = "timepico-%04d" % (unique_u32() % 10000)

ALLOWED_CIDRS = list(cfg_get("allowed_cidrs", ALLOWED_CIDRS))
DROP_UNSTABLE_NTP = bool(cfg_get("drop_unstable_ntp", DROP_UNSTABLE_NTP))

PEERS = list(cfg_get("peers", PEERS))

# ---------------- CIDR allowlist ----------------
def ipv4_to_u32(ip):
    p = ip.split(".")
    v = 0
    for x in p:
        v = (v << 8) | int(x)
    return v & 0xFFFFFFFF

def parse_cidr(c):
    ip_s, pref_s = c.split("/")
    ip = ipv4_to_u32(ip_s)
    pref = int(pref_s)
    mask = (0xFFFFFFFF << (32 - pref)) & 0xFFFFFFFF if pref else 0
    return (ip & mask, mask)

CIDRS = []
try:
    CIDRS = [parse_cidr(c) for c in ALLOWED_CIDRS]
except Exception:
    CIDRS = []

def ip_allowed(ip):
    if not CIDRS: return True
    try: v = ipv4_to_u32(ip)
    except Exception: return False
    for net,mask in CIDRS:
        if (v & mask) == net: return True
    return False

# ---- timekeeping: civil->unix ----
def _days_from_civil(y,m,d):
    if m <= 2: y -= 1; m += 12
    era = y//400
    yoe = y - era*400
    doy = (153*(m-3)+2)//5 + (d-1)
    doe = yoe*365 + yoe//4 - yoe//100 + doy
    return era*146097 + doe

_DAYS_1970_01_01 = _days_from_civil(1970,1,1)

def unix_us_from_ymdhms(y,mo,d,hh,mm,ss,us=0):
    days = _days_from_civil(y,mo,d) - _DAYS_1970_01_01
    return (((days*24+hh)*60+mm)*60+ss)*1_000_000 + us

def ntp_from_unix_us(unix_us):
    sec = unix_us//1_000_000 + NTP_DELTA
    frac = ((unix_us % 1_000_000) << 32) // 1_000_000
    return sec & 0xFFFFFFFF, frac & 0xFFFFFFFF

def set_machine_rtc_from_unix(unix_us):
    try:
        rtc = machine.RTC()
        unix_s = int(unix_us // 1_000_000)
        tm = time.gmtime(unix_s)
        rtc.datetime((tm[0], tm[1], tm[2], tm[6], tm[3], tm[4], tm[5], 0))
    except Exception:
        pass

# ---------------- Clock discipline with PPS + holdover + gating ---------
FLAG_GPS_LOCK     = 0x01
FLAG_SUSPECT      = 0x02
FLAG_STABILIZED   = 0x04
FLAG_STABLE_NOW   = 0x08
FLAG_PPS_LOCK     = 0x10

class ClockDiscipline:
    def __init__(self):
        self._base_unix_us = None
        self._base_ticks_us = None
        self._last_served = 0
        self._corr_q32 = 1 << 32

        self.gps_valid = False
        self.gps_fix_type = 0
        self.gps_lock = False
        self.last_gps_ms = 0

        self.last_pps_ms = 0
        self.last_pps_tick_us = None
        self.pps_period_avg_us = None
        self.pps_good = 0
        self.pps_locked = False

        self.pps_anchor_unix_s = None
        self.pps_anchor_tick_us = None

        self.ever_stabilized = False
        self._stable_run_s = 0
        self.last_trusted_ref_ms = 0

        self.suspect = False
        self.suspect_clear_ms = 0

        self.peer_ok = False
        self.peer_median_offset_us = 0
        self.peer_sample_count = 0

    def _set_base(self, unix_us, base_tick_us=None):
        self._base_unix_us = int(unix_us)
        self._base_ticks_us = ticks_us() if base_tick_us is None else int(base_tick_us)
        if self._base_unix_us > self._last_served:
            self._last_served = self._base_unix_us

    def now_unix_us(self):
        if self._base_unix_us is None or self._base_ticks_us is None:
            return None
        dt = ticks_diff(ticks_us(), self._base_ticks_us)
        dt_corr = (int(dt) * int(self._corr_q32)) >> 32
        est = int(self._base_unix_us) + int(dt_corr)
        if est < self._last_served:
            est = self._last_served
        else:
            self._last_served = est
        return est

    def flags(self):
        f = 0
        if self.gps_lock: f |= FLAG_GPS_LOCK
        if self.suspect: f |= FLAG_SUSPECT
        if self.ever_stabilized: f |= FLAG_STABILIZED
        if self.is_stable_now(): f |= FLAG_STABLE_NOW
        if self.pps_locked: f |= FLAG_PPS_LOCK
        return f

    def _pps_age_ms(self):
        if self.last_pps_ms == 0: return 0x7FFFFFFF
        return ticks_diff(ticks_ms(), self.last_pps_ms)

    def _gps_age_ms(self):
        if self.last_gps_ms == 0: return 0x7FFFFFFF
        return ticks_diff(ticks_ms(), self.last_gps_ms)

    def pps_present(self):
        age = self._pps_age_ms()
        return age >= 0 and age <= PPS_TIMEOUT_MS

    def gps_recent(self):
        age = self._gps_age_ms()
        return age >= 0 and age <= GPS_RMC_TIMEOUT_MS

    def _update_corr_from_period(self, period_us):
        if period_us <= 0: return
        self._corr_q32 = (1_000_000 << 32) // int(period_us)

    def _holdover_ppm(self):
        if self.pps_period_avg_us is None:
            return HOLDOVER_PPM_MIN
        ppm_meas = abs(int(self.pps_period_avg_us) - 1_000_000)
        return max(HOLDOVER_PPM_MIN, ppm_meas + HOLDOVER_PPM_MARGIN)

    def holdover_uncertainty_us(self):
        if self.last_trusted_ref_ms == 0:
            return 0x7FFFFFFF
        age_ms = ticks_diff(ticks_ms(), self.last_trusted_ref_ms)
        if age_ms < 0:
            return 0x7FFFFFFF
        age_s = age_ms // 1000
        return int(age_s) * int(self._holdover_ppm())

    def is_stable_now(self):
        if self.suspect: return False
        if self._base_unix_us is None: return False
        if not self.ever_stabilized: return False

        if self.gps_lock and self.pps_locked and self.pps_present():
            return True

        if self.peer_ok and self.peer_sample_count >= MIN_PEERS_FOR_VOTE:
            if abs(int(self.peer_median_offset_us)) <= VOTE_CONFIRM_US:
                return True

        age_us = self.holdover_uncertainty_us()
        age_ms = ticks_diff(ticks_ms(), self.last_trusted_ref_ms) if self.last_trusted_ref_ms else 0x7FFFFFFF
        if age_ms >= 0 and (age_ms // 1000) <= MAX_HOLDOVER_S and age_us <= MAX_HOLDOVER_UNCERTAINTY_US:
            return True

        return False

    def can_serve_ntp(self):
        return self.is_stable_now() and (not self.suspect)

    def ntp_stratum(self):
        if not self.is_stable_now(): return 16
        if self.gps_lock and self.pps_locked: return 1
        if self.peer_ok: return 2
        return 3

    def ntp_refid(self):
        if self.suspect: return b"BAD!"
        if self.gps_lock and self.pps_locked: return b"GPS\0"
        if self.peer_ok: return b"P2P\0"
        return b"LOCL"

    def ntp_leap(self):
        return 0 if self.ntp_stratum() < 16 else 3

    def pps_pulse(self, pps_tick_us):
        nowms = ticks_ms()
        self.last_pps_ms = nowms

        if self.last_pps_tick_us is not None:
            period = int(ticks_diff(int(pps_tick_us), int(self.last_pps_tick_us)))
            if PPS_PERIOD_VALID_MIN_US <= period <= PPS_PERIOD_VALID_MAX_US:
                if self.pps_period_avg_us is None:
                    self.pps_period_avg_us = period
                else:
                    self.pps_period_avg_us = (int(self.pps_period_avg_us) * 7 + period) // 8

                self._update_corr_from_period(self.pps_period_avg_us)

                if abs(int(self.pps_period_avg_us) - 1_000_000) <= PPS_LOCK_TOL_US:
                    self.pps_good += 1
                else:
                    self.pps_good = 0
                self.pps_locked = (self.pps_good >= PPS_LOCK_COUNT)
            else:
                self.pps_period_avg_us = None
                self.pps_good = 0
                self.pps_locked = False

        self.last_pps_tick_us = int(pps_tick_us)

        if self.pps_anchor_unix_s is not None:
            self.pps_anchor_unix_s = int(self.pps_anchor_unix_s) + 1
            self.pps_anchor_tick_us = int(pps_tick_us)
            self._set_base(int(self.pps_anchor_unix_s) * 1_000_000, base_tick_us=self.pps_anchor_tick_us)

    def gps_rmc(self, unix_us, valid, fix_type, rmc_rx_tick_us):
        nowms = ticks_ms()
        self.gps_valid = bool(valid)
        self.gps_fix_type = int(fix_type) if fix_type else 0
        self.gps_lock = bool(self.gps_valid and self.gps_fix_type == 3)
        self.last_gps_ms = nowms

        gps_unix_s = int((int(unix_us) + 500_000) // 1_000_000)

        aligned = False
        if self.last_pps_tick_us is not None:
            skew = abs(int(ticks_diff(int(rmc_rx_tick_us), int(self.last_pps_tick_us))))
            if skew <= RMC_ALIGN_WINDOW_US:
                self.pps_anchor_unix_s = gps_unix_s
                self.pps_anchor_tick_us = int(self.last_pps_tick_us)
                self._set_base(gps_unix_s * 1_000_000, base_tick_us=self.pps_anchor_tick_us)
                aligned = True

        if (self._base_unix_us is None) and (not aligned):
            self._set_base(int(unix_us), base_tick_us=int(rmc_rx_tick_us))

        if self.gps_lock and self.pps_locked and (self.pps_anchor_unix_s is not None) and self.pps_present():
            self.last_trusted_ref_ms = nowms

            if self.ever_stabilized:
                local_now = self.now_unix_us()
                if local_now is not None:
                    err = int((gps_unix_s * 1_000_000) - local_now)
                    if abs(err) > GPS_MAX_STEP_US:
                        self.suspect = True
                        log("ALERT: GPS step too large after stabilization:", err, "us -> SUSPECT")
                        return

            self._stable_run_s += 1
            if (not self.ever_stabilized) and self._stable_run_s >= GPS_STABLE_S:
                self.ever_stabilized = True
                set_machine_rtc_from_unix(gps_unix_s * 1_000_000)
                log("Stabilized: GPS+PPS locked for", GPS_STABLE_S, "seconds -> eligible voter / can serve if stable.")
        else:
            self._stable_run_s = 0

    def consider_peers(self, med_off, good, count):
        self.peer_sample_count = int(count)
        self.peer_ok = bool(good and med_off is not None)
        if med_off is not None:
            self.peer_median_offset_us = int(med_off)

        if self.is_stable_now() and self.peer_ok and self.peer_sample_count >= MIN_PEERS_FOR_VOTE:
            if abs(self.peer_median_offset_us) > ALERT_OFFSET_US and not self.suspect:
                self.suspect = True
                self.suspect_clear_ms = 0
                log("ALERT: local deviates from peers by", self.peer_median_offset_us, "us -> SUSPECT")
            elif self.suspect:
                if abs(self.peer_median_offset_us) <= (ALERT_OFFSET_US // 2):
                    if self.suspect_clear_ms == 0:
                        self.suspect_clear_ms = ticks_ms()
                    elif ticks_diff(ticks_ms(), self.suspect_clear_ms) > SUSPECT_CLEAR_GOOD_S * 1000:
                        self.suspect = False
                        self.suspect_clear_ms = 0
                        log("Suspect cleared after sustained peer agreement")
                else:
                    self.suspect_clear_ms = 0

        if (not self.gps_lock) and self.peer_ok and (not self.suspect) and self.peer_sample_count >= MIN_PEERS_FOR_VOTE:
            if abs(self.peer_median_offset_us) <= VOTE_CONFIRM_US:
                self.last_trusted_ref_ms = ticks_ms()

CLOCK = ClockDiscipline()

# ---------------- GPS NMEA parser ----------------
class NMEAParser:
    def __init__(self):
        self._buf = bytearray()
        self.fix_type = 0
        self.last_rmc_unix_us = None
        self.last_rmc_valid = False

    def _checksum_ok(self, line):
        try:
            star = line.rfind(b"*")
            if star == -1: return False
            given = int(line[star+1:star+3], 16)
            calc = 0
            for b in line[1:star]:
                calc ^= b
            return (calc & 0xFF) == given
        except Exception:
            return False

    def _parse_hhmmss(self, s):
        if len(s) < 6: return None
        hh = int(s[0:2]); mm = int(s[2:4]); ss = int(s[4:6])
        us = 0
        dot = s.find(b".")
        if dot != -1 and dot + 1 < len(s):
            frac = s[dot+1:dot+4]
            while len(frac) < 3: frac += b"0"
            us = int(frac) * 1000
        return hh, mm, ss, us

    def _parse_ddmmyy(self, s):
        if len(s) != 6: return None
        dd = int(s[0:2]); mo = int(s[2:4]); yy = int(s[4:6])
        return 2000 + yy, mo, dd

    def _handle_rmc(self, f):
        if len(f) < 10: return
        t = f[1]; st = f[2]; d = f[9]
        if not t or not d: return
        pt = self._parse_hhmmss(t)
        pd = self._parse_ddmmyy(d)
        if pt is None or pd is None: return
        hh, mm, ss, us = pt
        y, mo, dd = pd
        self.last_rmc_unix_us = unix_us_from_ymdhms(y, mo, dd, hh, mm, ss, us)
        self.last_rmc_valid = (st == b"A")

    def _handle_gsa(self, f):
        if len(f) < 3: return
        try:
            self.fix_type = int(f[2])
        except Exception:
            pass

    def feed(self, b):
        for ch in b:
            if ch == 10:
                line = bytes(self._buf).strip()
                self._buf = bytearray()
                if not line or line[0:1] != b"$": continue
                if not self._checksum_ok(line): continue
                star = line.rfind(b"*")
                core = line[1:star]
                f = core.split(b",")
                if not f: continue
                t = f[0]
                if t.endswith(b"RMC"):
                    self._handle_rmc(f)
                elif t.endswith(b"GSA"):
                    self._handle_gsa(f)
            elif ch != 13:
                if len(self._buf) < 128:
                    self._buf.append(ch)

GPS = NMEAParser()

# ---------------- HT16K33 7-seg ----------------
class HT16K33_7Seg:
    DIGITS = (0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F)
    GLYPH_MINUS = 0x40
    GLYPH_BLANK = 0x00
    GLYPH_E = 0x79
    GLYPH_r = 0x50
    DP = 0x80

    def __init__(self, i2c, addr=0x70, brightness=2):
        self.i2c = i2c
        self.addr = addr
        self.buf = [0] * 8
        self._cmd(0x21)
        self.set_blink_rate(0)
        self.set_brightness(brightness)
        self.clear()
        self.draw()

    def _cmd(self, c):
        self.i2c.writeto(self.addr, bytes([c & 0xFF]))

    def set_brightness(self, b):
        self._cmd(0xE0 | clamp(int(b), 0, 15))

    def set_blink_rate(self, r):
        self._cmd(0x80 | 0x01 | (clamp(int(r), 0, 3) << 1))

    def clear(self):
        for i in range(8):
            self.buf[i] = 0
        self.buf[2] = 0

    def _loc(self, idx):
        locs = (0, 1, 3, 4)
        return locs[idx]

    def set_colon(self, on):
        self.buf[2] = 0x02 if on else 0

    def set_digit(self, idx, val, leading_blank=False):
        loc = self._loc(idx)
        if leading_blank and val == 0:
            glyph = self.GLYPH_BLANK
        else:
            glyph = self.DIGITS[val]
        # Preserve DP bit if already set
        dp = self.buf[loc] & self.DP
        self.buf[loc] = glyph | dp

    def clear_dps(self):
        for i in range(4):
            loc = self._loc(i)
            self.buf[loc] &= 0x7F

    def set_dp(self, idx, on):
        loc = self._loc(idx)
        if on:
            self.buf[loc] |= self.DP
        else:
            self.buf[loc] &= 0x7F

    def show_dashes(self):
        for i in range(4):
            loc = self._loc(i)
            self.buf[loc] = self.GLYPH_MINUS
        self.set_colon(False)

    def show_error(self):
        self.buf[self._loc(0)] = self.GLYPH_E
        self.buf[self._loc(1)] = self.GLYPH_r
        self.buf[self._loc(2)] = self.GLYPH_r
        self.buf[self._loc(3)] = self.GLYPH_BLANK
        self.set_colon(False)

    def show_hhmm(self, hh, mm, colon_on):
        hh %= 24; mm %= 60
        self.set_digit(0, hh // 10, leading_blank=True)
        self.set_digit(1, hh % 10)
        self.set_digit(2, mm // 10)
        self.set_digit(3, mm % 10)
        self.set_colon(colon_on)

    def show_number4(self, n):
        # Show 4 digits with leading zeros (e.g., PIN)
        try:
            n = int(n) % 10000
        except Exception:
            n = 0
        d0 = (n // 1000) % 10
        d1 = (n // 100) % 10
        d2 = (n // 10) % 10
        d3 = n % 10
        # Preserve decimal points if already set
        for idx, val in enumerate((d0, d1, d2, d3)):
            loc = self._loc(idx)
            dp = self.buf[loc] & self.DP
            self.buf[loc] = self.DIGITS[val] | dp
        self.set_colon(False)

    def draw(self):
        out = bytearray(17)
        out[0] = 0x00
        j = 1
        for w in self.buf:
            out[j] = w & 0xFF
            out[j+1] = (w >> 8) & 0xFF
            j += 2
        self.i2c.writeto(self.addr, out)

# ---------------- HMAC (truncated) ----------------
def hmac_sha256_trunc16(key, msg):
    block = 64
    if len(key) > block:
        key = hashlib.sha256(key).digest()
    if len(key) < block:
        key += b"\x00" * (block - len(key))
    o = bytes((b ^ 0x5C) for b in key)
    i = bytes((b ^ 0x36) for b in key)
    inner = hashlib.sha256(i + msg).digest()
    return hashlib.sha256(o + inner).digest()[:16]

# ---------------- NTP request debug flash (DP3) ----------------
_ntp_flash_queue = 0
_ntp_flash_remaining = 0
_ntp_flash_on = False
_ntp_flash_next_ms = 0
_ntp_flash_dirty = False

def ntp_debug_flash_trigger():
    # Called on each valid NTP client request.
    # If we're already flashing, queue one more flash sequence (capped).
    global _ntp_flash_queue, _ntp_flash_remaining
    if not NTP_DEBUG_FLASH_ENABLE:
        return
    now_ms = ticks_ms()
    if _ntp_flash_remaining > 0:
        if _ntp_flash_queue < int(NTP_DEBUG_FLASH_QUEUE_CAP):
            _ntp_flash_queue += 1
    else:
        _start_ntp_flash(now_ms)

def _start_ntp_flash(now_ms):
    global _ntp_flash_remaining, _ntp_flash_on, _ntp_flash_next_ms
    _ntp_flash_remaining = int(NTP_DEBUG_FLASH_TOGGLES)
    _ntp_flash_on = False
    _ntp_flash_next_ms = int(now_ms)

def ntp_flash_update(now_ms):
    # Returns True if flash is active or queued.
    global _ntp_flash_queue, _ntp_flash_remaining, _ntp_flash_on, _ntp_flash_next_ms, _ntp_flash_dirty
    now_ms = int(now_ms)
    if _ntp_flash_remaining <= 0:
        if _ntp_flash_queue > 0:
            _ntp_flash_queue -= 1
            _start_ntp_flash(now_ms)
        else:
            return False
    # time to toggle?
    if ticks_diff(now_ms, _ntp_flash_next_ms) >= 0:
        _ntp_flash_on = not _ntp_flash_on
        _ntp_flash_remaining -= 1
        _ntp_flash_next_ms = now_ms + int(NTP_DEBUG_FLASH_STEP_MS)
        _ntp_flash_dirty = True
    return True

def ntp_flash_dp3_state():
    # True if DP3 should be forced ON right now
    return bool(_ntp_flash_on) and (_ntp_flash_remaining > 0)

# ---------------- Peer protocol ----------------
PEER_MAGIC = b"TPv2"
MSG_PING = 1
MSG_PONG = 2
PEER_FMT = "!4sBBHIQQQQHBB"
PEER_LEN = struct.calcsize(PEER_FMT)
PEER_TOTAL = PEER_LEN + 16

class PeerState:
    def __init__(self, ip):
        self.ip = ip
        self.last_offset_us = None
        self.last_delay_us = None
        self.last_sample_ms = 0
        self.strikes = 0
        self.suspect = False
        self.last_stratum = 16
        self.last_flags = 0

class PeerMonitor:
    def __init__(self, peers):
        self.sender_id = unique_u32()
        self.seq = 0
        self.peers = {ip: PeerState(ip) for ip in peers}
        self.pending = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0", PEER_PORT))
        self.last_ping_ms = 0
        self.last_activity_ms = 0

    def _touch(self):
        self.last_activity_ms = ticks_ms()

    def _gps_age_s(self):
        if CLOCK.last_gps_ms == 0:
            return 65535
        age = ticks_diff(ticks_ms(), CLOCK.last_gps_ms)
        return 65535 if age < 0 else clamp(age // 1000, 0, 65535)

    def _pack(self, typ, flags, seq, sender_id, t1, t2, t3, nonce, gps_age_s, stratum):
        base = struct.pack(
            PEER_FMT, PEER_MAGIC, typ & 0xFF, flags & 0xFF, seq & 0xFFFF, sender_id & 0xFFFFFFFF,
            t1 & 0xFFFFFFFFFFFFFFFF, t2 & 0xFFFFFFFFFFFFFFFF, t3 & 0xFFFFFFFFFFFFFFFF, nonce & 0xFFFFFFFFFFFFFFFF,
            gps_age_s & 0xFFFF, stratum & 0xFF, 0
        )
        return base + hmac_sha256_trunc16(PEER_PSK, base)

    def _unpack(self, pkt):
        if len(pkt) != PEER_TOTAL:
            return None
        base = pkt[:PEER_LEN]
        mac = pkt[PEER_LEN:]
        if hmac_sha256_trunc16(PEER_PSK, base) != mac:
            return None
        f = struct.unpack(PEER_FMT, base)
        if f[0] != PEER_MAGIC:
            return None
        return f

    def send_pings_if_due(self):
        now = ticks_ms()
        # Update NTP request flash state machine
        try:
            ntp_flash_update(now)
        except Exception:
            pass
        if ticks_diff(now, self.last_ping_ms) < PEER_PING_INTERVAL_MS:
            return
        self.last_ping_ms = now

        t1 = CLOCK.now_unix_us()
        if t1 is None:
            return

        flags = CLOCK.flags()
        for ip in list(self.peers.keys()):
            self.seq = (self.seq + 1) & 0xFFFF
            nonce = rand_u64()
            pkt = self._pack(MSG_PING, flags, self.seq, self.sender_id, t1, 0, 0, nonce, self._gps_age_s(), CLOCK.ntp_stratum())
            try:
                self.sock.sendto(pkt, (ip, PEER_PORT))
                self._touch()
                self.pending[nonce] = (ip, t1, now)
            except Exception:
                pass

        for nonce, (ip, _, sent) in list(self.pending.items()):
            if ticks_diff(now, sent) > PEER_SAMPLE_MAX_AGE_MS:
                self.pending.pop(nonce, None)

    def handle_incoming(self):
        while True:
            try:
                pkt, addr = self.sock.recvfrom(256)
            except OSError:
                break
            ip = addr[0]
            if ip not in self.peers:
                continue
            f = self._unpack(pkt)
            if not f:
                continue
            self._touch()
            _, typ, flags, seq, sender_id, t1, t2, t3, nonce, gps_age_s, stratum, _ = f

            st = self.peers[ip]
            st.last_flags = int(flags)
            st.last_stratum = int(stratum)

            if typ == MSG_PING:
                now = CLOCK.now_unix_us()
                if now is None:
                    continue
                recv_us = now
                send_us = CLOCK.now_unix_us() or recv_us
                rflags = CLOCK.flags()
                resp = self._pack(MSG_PONG, rflags, seq, self.sender_id, t1, recv_us, send_us, nonce, self._gps_age_s(), CLOCK.ntp_stratum())
                try:
                    self.sock.sendto(resp, (ip, PEER_PORT))
                    self._touch()
                except Exception:
                    pass

            elif typ == MSG_PONG:
                pend = self.pending.pop(nonce, None)
                if not pend:
                    continue
                p_ip, p_t1, p_sent = pend
                if p_ip != ip:
                    continue
                t4 = CLOCK.now_unix_us()
                if t4 is None:
                    continue

                delay = int((t4 - p_t1) - (t3 - t2))
                offset = int(((t2 - p_t1) + (t3 - t4)) // 2)

                st.last_sample_ms = ticks_ms()
                st.last_offset_us = offset
                st.last_delay_us = delay

    def _peer_eligible_voter(self, st):
        if st.suspect: return False
        if st.last_offset_us is None or st.last_delay_us is None: return False
        if ticks_diff(ticks_ms(), st.last_sample_ms) > PEER_SAMPLE_MAX_AGE_MS: return False
        if abs(int(st.last_delay_us)) > PEER_MAX_DELAY_US: return False
        if st.last_stratum >= 16: return False
        if (st.last_flags & FLAG_STABILIZED) == 0: return False
        if (st.last_flags & FLAG_STABLE_NOW) == 0: return False
        if (st.last_flags & FLAG_SUSPECT) != 0: return False
        return True

    def evaluate(self):
        samples = []
        for ip, st in self.peers.items():
            if not self._peer_eligible_voter(st):
                continue
            samples.append((ip, int(st.last_offset_us)))

        if not samples:
            return None, False, 0

        off = [o for _, o in samples]
        med = median(off)

        if med is not None:
            for ip, o in samples:
                st = self.peers[ip]
                if abs(o - med) > PEER_OUTLIER_US:
                    st.strikes += 1
                    if st.strikes >= OUTLIER_STRIKES:
                        st.suspect = True
                        log("Peer", ip, "flagged SUSPECT (outlier)", o, "median", med)
                else:
                    st.strikes = 0

        return med, True, len(samples)

# ---------------- NTP server ----------------
class NTPServer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0", NTP_PORT))
        # per-client token bucket: ip -> (tokens, last_ms)
        self.ratelimit = {}
        log("NTP server listening on UDP/%d" % NTP_PORT)

    def _rate_ok(self, ip):
        # Token bucket that allows iBurst (multiple quick requests) while limiting abuse.
        # Cap tokens; refill at 1 token every NTP_RL_REFILL_MS.
        now = ticks_ms()
        # Update NTP request flash state machine
        try:
            ntp_flash_update(now)
        except Exception:
            pass
        st = self.ratelimit.get(ip)
        if st is None:
            tokens = int(NTP_RL_BUCKET_CAP)
            last = now
        else:
            tokens, last = st

        dt = ticks_diff(now, last)
        if dt > 0:
            add = dt // int(NTP_RL_REFILL_MS) if int(NTP_RL_REFILL_MS) > 0 else 0
            if add > 0:
                tokens = min(int(NTP_RL_BUCKET_CAP), int(tokens) + int(add))
                # advance last by the amount we accounted for (keeps remainder)
                last = last + int(add) * int(NTP_RL_REFILL_MS)

        if tokens <= 0:
            # keep updated last to avoid unbounded dt growth
            self.ratelimit[ip] = (0, now)
            return False

        tokens -= 1
        self.ratelimit[ip] = (tokens, last)

        # simple cache trim
        if len(self.ratelimit) > 128:
            for k in list(self.ratelimit.keys())[:32]:
                self.ratelimit.pop(k, None)
        return True

    def _req_version(self, req0):
        # Return requested VN, clamped to 3..4 (best compatibility with common clients).
        try:
            vn = (req0 >> 3) & 0x07
        except Exception:
            return 4
        if vn < 3:
            return 3
        if vn > 4:
            return 4
        return vn

    def _build_reply(self, req, t_recv_us, t_xmt_us):
        # Basic NTP server response compatible with SNTP/NTPv3/NTPv4 clients.
        # Echo client transmit timestamp into originate timestamp.
        try:
            org_sec, org_frac = struct.unpack_from("!II", req, 40)
        except Exception:
            org_sec, org_frac = 0, 0

        vn = self._req_version(req[0] if req else 0)
        li = CLOCK.ntp_leap()
        mode = 4  # server
        first = (li << 6) | (vn << 3) | mode

        stratum = CLOCK.ntp_stratum()
        # Echo client poll if present (improves compatibility with some stacks)
        poll = req[2] if len(req) >= 3 else 4
        precision = -20

        root_delay = 0
        # Root dispersion (16.16). Keep small but non-zero.
        root_disp = 0x00002000  # ~0.125s
        ref_id = CLOCK.ntp_refid()

        # Reference timestamp: best-effort use current PPS-anchored second.
        ref_us = t_xmt_us
        if CLOCK.pps_anchor_unix_s is not None:
            ref_us = int(CLOCK.pps_anchor_unix_s) * 1_000_000

        ref_sec, ref_frac = ntp_from_unix_us(ref_us)
        rec_sec, rec_frac = ntp_from_unix_us(t_recv_us)
        xmt_sec, xmt_frac = ntp_from_unix_us(t_xmt_us)

        pkt = bytearray(48)
        struct.pack_into("!BBBB", pkt, 0, first & 0xFF, stratum & 0xFF, poll & 0xFF, precision & 0xFF)
        struct.pack_into("!II", pkt, 4, root_delay & 0xFFFFFFFF, root_disp & 0xFFFFFFFF)
        pkt[12:16] = ref_id[0:4].ljust(4, b"\x00")
        struct.pack_into("!II", pkt, 16, ref_sec, ref_frac)         # Reference Timestamp
        struct.pack_into("!II", pkt, 24, org_sec, org_frac)         # Originate Timestamp (client xmit)
        struct.pack_into("!II", pkt, 32, rec_sec, rec_frac)         # Receive Timestamp
        struct.pack_into("!II", pkt, 40, xmt_sec, xmt_frac)         # Transmit Timestamp
        return bytes(pkt)

    def _build_kod(self, req, t_recv_us, t_xmt_us, code=b"RATE"):
        # Kiss-o'-Death packet: stratum 0, refid = 4-char code (e.g., RATE, DENY)
        try:
            org_sec, org_frac = struct.unpack_from("!II", req, 40)
        except Exception:
            org_sec, org_frac = 0, 0

        vn = self._req_version(req[0] if req else 0)
        li = 0
        mode = 4
        first = (li << 6) | (vn << 3) | mode

        stratum = 0
        poll = req[2] if len(req) >= 3 else 4
        precision = -20

        root_delay = 0
        root_disp = 0

        ref_sec, ref_frac = ntp_from_unix_us(t_xmt_us)
        rec_sec, rec_frac = ntp_from_unix_us(t_recv_us)
        xmt_sec, xmt_frac = ntp_from_unix_us(t_xmt_us)

        pkt = bytearray(48)
        struct.pack_into("!BBBB", pkt, 0, first & 0xFF, stratum & 0xFF, poll & 0xFF, precision & 0xFF)
        struct.pack_into("!II", pkt, 4, root_delay & 0xFFFFFFFF, root_disp & 0xFFFFFFFF)
        pkt[12:16] = code[0:4].ljust(4, b"\x00")                    # Kiss code
        struct.pack_into("!II", pkt, 16, ref_sec, ref_frac)
        struct.pack_into("!II", pkt, 24, org_sec, org_frac)
        struct.pack_into("!II", pkt, 32, rec_sec, rec_frac)
        struct.pack_into("!II", pkt, 40, xmt_sec, xmt_frac)
        return bytes(pkt)

    def serve_once(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
            except OSError:
                break

            ip = addr[0]
            if not ip_allowed(ip):
                continue

            # Must be at least a full header
            if not data or len(data) < 48:
                continue

            # Mode check: accept client mode (3). Some stacks may send mode 1 (sym active) for testing;
            # we ignore non-client requests to keep attack surface small.
            mode = data[0] & 0x07
            if mode != 3:
                continue


            # Debug: flash DP3 on any valid NTP client request
            try:
                ntp_debug_flash_trigger()
            except Exception:
                pass

            now1 = CLOCK.now_unix_us()
            if now1 is None:
                continue
            t_recv = now1

            # Rate limit (but allow iBurst)
            if not self._rate_ok(ip):
                if NTP_SEND_KOD_ON_LIMIT:
                    try:
                        t_xmt = CLOCK.now_unix_us() or t_recv
                        self.sock.sendto(self._build_kod(data, t_recv, t_xmt, b"RATE"), addr)
                    except Exception:
                        pass
                continue

            # Stability gate (your hardened requirement)
            if not CLOCK.can_serve_ntp():
                if DROP_UNSTABLE_NTP:
                    continue
                # else: respond but stratum will be 16 and LI alarm via CLOCK

            t_xmt = CLOCK.now_unix_us() or t_recv
            try:
                self.sock.sendto(self._build_reply(data, t_recv, t_xmt), addr)
            except Exception:
                pass


# ---------------- Config Portal ----------------
def _html_escape(s):
    s = str(s)
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")

def _urldecode(s):
    # Minimal x-www-form-urlencoded decoder
    out = bytearray()
    i = 0
    s = s.replace("+", " ")
    while i < len(s):
        ch = s[i]
        if ch == "%" and i + 2 < len(s):
            try:
                out.append(int(s[i+1:i+3], 16))
                i += 3
                continue
            except Exception:
                pass
        out.append(ord(ch))
        i += 1
    try:
        return out.decode("utf-8")
    except Exception:
        return out.decode()

def _parse_qs(body):
    # returns dict of key->value
    res = {}
    for part in body.split("&"):
        if not part:
            continue
        if "=" in part:
            k, v = part.split("=", 1)
        else:
            k, v = part, ""
        res[_urldecode(k)] = _urldecode(v)
    return res

def _fs_free_bytes():
    # Best-effort free space on filesystem (bytes). Returns 0 if unavailable.
    try:
        st = os.statvfs('/')
        # (bsize, frsize, blocks, bfree, bavail, ...)
        bsize = int(st[0])
        bfree = int(st[3])
        return bsize * bfree
    except Exception:
        return 0

class ConfigPortal:
    def __init__(self, boot_ms):
        self.boot_ms = boot_ms
        self.sock = None
        self.enabled_until_ms = boot_ms + int(CONFIG_PORTAL_WINDOW_S * 1000)
        self.ap_mode = False
        self.started = False

        self.require_pin = False
        self.pin = None

        # Controls when to show PIN on the display (STA/wired mode); AP shows PIN continuously.
        self.pin_display_until_ms = 0

    def _gen_pin(self):
        # Best-effort 4-digit PIN (not cryptographic; prevents casual access).
        try:
            return int((rand_u64() ^ (unique_u32() << 16) ^ ticks_us()) % 10000)
        except Exception:
            return int((ticks_ms() ^ unique_u32()) % 10000)

    def start(self, ap_mode=False):
        if not ENABLE_CONFIG_PORTAL:
            return
        self.ap_mode = bool(ap_mode)

        # Decide if PIN is required
        if self.ap_mode:
            self.require_pin = bool(PORTAL_PIN_REQUIRED_AP)
        else:
            self.require_pin = bool(PORTAL_PIN_REQUIRED_STA)

        # Create/refresh PIN if required
        if self.require_pin:
            self.pin = self._gen_pin()
            # show briefly at boot
            self.pin_display_until_ms = ticks_ms() + int(PORTAL_PIN_DISPLAY_ON_BOOT_MS)
            log("Config portal PIN=%04d (mode=%s)" % (int(self.pin), "AP" if self.ap_mode else "STA"))
        else:
            self.pin = None
            self.pin_display_until_ms = 0

        if self.ap_mode:
            # In AP fallback: keep portal available until reboot/config-save
            self.enabled_until_ms = 0
        else:
            # In STA/wired: time-limited window after boot
            self.enabled_until_ms = self.boot_ms + int(CONFIG_PORTAL_WINDOW_S * 1000)

        try:
            s = socket.socket()
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except Exception:
                pass
            s.setblocking(False)
            s.bind(("0.0.0.0", CONFIG_PORTAL_PORT))
            s.listen(2)
            self.sock = s
            self.started = True
            log("Config portal listening on TCP/%d (ap_mode=%s)" % (CONFIG_PORTAL_PORT, self.ap_mode))
        except Exception as e:
            log("Config portal failed to start:", e)
            self.sock = None
            self.started = False

    def active(self):
        if not self.started or self.sock is None:
            return False
        if self.ap_mode:
            return True
        return ticks_diff(ticks_ms(), self.enabled_until_ms) < 0

    def stop_if_expired(self):
        if self.ap_mode:
            return
        if self.sock is None:
            return
        if ticks_diff(ticks_ms(), self.enabled_until_ms) >= 0:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            log("Config portal disabled (window expired)")

    def touch_access(self):
        # Called when an HTTP connection is accepted (someone is trying to use the portal).
        if self.require_pin and self.pin is not None and (not self.ap_mode):
            self.pin_display_until_ms = ticks_ms() + int(PORTAL_PIN_DISPLAY_ON_ACCESS_MS)

    def should_show_pin(self):
        if not self.require_pin or self.pin is None:
            return False
        if self.ap_mode:
            return True
        return ticks_diff(ticks_ms(), int(self.pin_display_until_ms)) < 0

    def current_pin(self):
        return int(self.pin) if self.pin is not None else None

    # ---------- low-memory response helpers ----------
    def _send_all(self, conn, data):
        # Accept str/bytes and send reliably in small chunks.
        if data is None:
            return
        try:
            if isinstance(data, str):
                data = data.encode("utf-8")
        except Exception:
            try:
                data = str(data).encode("utf-8")
            except Exception:
                return

        try:
            mv = memoryview(data)
            while mv:
                n = conn.send(mv)
                if n is None:
                    break
                mv = mv[n:]
        except Exception:
            # Try write() if send() fails on some ports
            try:
                conn.write(data)
            except Exception:
                pass

    def _http_header(self, code="200 OK", ctype="text/html; charset=utf-8"):
        return "HTTP/1.0 %s\r\nContent-Type: %s\r\nConnection: close\r\n\r\n" % (code, ctype)

    def _page_begin(self, conn, title, bar_title, refresh_s=None, refresh_url="/"):
        # Send common HTML head + styles and open wrapper/card.
        self._send_all(conn, self._http_header("200 OK", "text/html; charset=utf-8"))
        self._send_all(conn, PORTAL_H1)
        if refresh_s is not None:
            try:
                self._send_all(conn, "<meta http-equiv='refresh' content='%d; url=%s'>" % (int(refresh_s), refresh_url))
            except Exception:
                pass
        self._send_all(conn, PORTAL_T1)
        self._send_all(conn, title)
        self._send_all(conn, PORTAL_T2)
        self._send_all(conn, PORTAL_CSS)
        self._send_all(conn, PORTAL_H2)
        self._send_all(conn, bar_title)
        self._send_all(conn, PORTAL_V1)
        self._send_all(conn, TIMEPICO_VERSION)
        self._send_all(conn, PORTAL_H3)

    def _page_end(self, conn):
        self._send_all(conn, PORTAL_END)

    def _render_unlock(self, conn, nic_mode, ip_info, msg=""):
        # PIN entry page (compact)
        gc.collect()
        self._page_begin(conn, "Unlock", "TimePico Portal")
        # Hide network details on PIN page: only show hostname + mode
        self._send_all(conn, "<p class='m'><b>Hostname:</b> %s<br><b>Mode:</b> %s</p>" %
                             (_html_escape(HOSTNAME), _html_escape(nic_mode)))
        if msg:
            self._send_all(conn, "<div class='e'>%s</div>" % _html_escape(msg))
        self._send_all(conn, "<p>Enter the 4-digit PIN shown on the device display.</p>")
        self._send_all(conn, "<form method='GET' action='/'><label for='pin'>PIN</label>")
        self._send_all(conn, "<input id='pin' name='pin' inputmode='numeric' pattern='[0-9]*' maxlength='4' autofocus>")
        self._send_all(conn, "<div style='height:10px'></div><button class='b blue' type='submit'>Unlock</button></form>")
        self._send_all(conn, "<div class='s'>Refresh this page to re-show the PIN on the display (STA mode).</div>")
        self._page_end(conn)

    def _render_config(self, conn, nic_mode, ip_info):
        # Config form (compact)
        gc.collect()
        peers_str = ",".join(PEERS) if isinstance(PEERS, list) else str(PEERS)
        cidr_str = ",".join(ALLOWED_CIDRS) if isinstance(ALLOWED_CIDRS, list) else str(ALLOWED_CIDRS)
        static_checked = "checked" if USE_STATIC_IP else ""
        ip, mask, gw, dns = STATIC_IP if isinstance(STATIC_IP, tuple) and len(STATIC_IP) == 4 else ("", "", "", "")

        self._page_begin(conn, "Config", "TimePico Config")
        self._send_all(conn, "<p class='m'><b>Hostname:</b> %s<br><b>Mode:</b> %s<br><b>IP:</b> %s</p>" %
                             (_html_escape(HOSTNAME), _html_escape(nic_mode), _html_escape(ip_info)))

        self._send_all(conn, "<hr><b>WiFi &amp; Identity</b>")
        self._send_all(conn, "<form method='POST' action='/save'>")
        if self.require_pin and (self.pin is not None):
            self._send_all(conn, "<input type='hidden' name='pin' value='%04d'>" % int(self.pin))

        # WiFi + identity
        self._send_all(conn, "<label for='wifi_ssid'>WiFi SSID</label><input id='wifi_ssid' name='wifi_ssid' value='%s'>" % _html_escape(WIFI_SSID))
        self._send_all(conn, "<label for='wifi_password'>WiFi Password</label><input id='wifi_password' name='wifi_password' value='%s' type='password'>" % _html_escape(WIFI_PASSWORD))
        self._send_all(conn, "<label for='hostname'>Hostname</label><input id='hostname' name='hostname' value='%s'>" % _html_escape(HOSTNAME))

        # Time format        self._send_all(conn, "<hr><b>Time &amp; Display</b>")

        # Time format

        opt24 = "selected" if TIME_DISPLAY_24H else ""
        opt12 = "selected" if (not TIME_DISPLAY_24H) else ""
        self._send_all(conn, "<label for='time_display'>Time display</label><select id='time_display' name='time_display'>")
        self._send_all(conn, "<option value='24' %s>24-hour</option><option value='12' %s>12-hour</option></select>" % (opt24, opt12))

        # Time zone offset (numeric)
        self._send_all(conn, "<label for='tz_offset_hours'>UTC offset (hours)</label><input id='tz_offset_hours' name='tz_offset_hours' value='%s'>" % _html_escape(TZ_OFFSET_HOURS))
        self._send_all(conn, "<div class='s'>Examples: -6, 0, +1, 5.5. DST is not automatic.</div>")

        # Brightness schedule
        self._send_all(conn, "<label for='brightness_day'>Day brightness (06:00-17:59)</label><input id='brightness_day' name='brightness_day' value='%d'>" % int(DISPLAY_BRIGHTNESS_DAY))
        self._send_all(conn, "<label for='brightness_night'>Night brightness (18:00-05:59)</label><input id='brightness_night' name='brightness_night' value='%d'>" % int(DISPLAY_BRIGHTNESS_NIGHT))
        self._send_all(conn, "<div class='s'>Brightness range is 0..15.</div>")

        self._send_all(conn, "<hr><b>Network</b>")

        # Static IP toggle + fields
        self._send_all(conn, "<label><input id='use_static_ip' type='checkbox' name='use_static_ip' value='1' %s> Use static IP</label>" % static_checked)
        self._send_all(conn, "<label for='ip'>IP</label><input id='ip' name='ip' value='%s'>" % _html_escape(ip))
        self._send_all(conn, "<label for='mask'>Netmask</label><input id='mask' name='mask' value='%s'>" % _html_escape(mask))
        self._send_all(conn, "<label for='gw'>Gateway</label><input id='gw' name='gw' value='%s'>" % _html_escape(gw))
        self._send_all(conn, "<label for='dns'>DNS</label><input id='dns' name='dns' value='%s'>" % _html_escape(dns))

        self._send_all(conn, "<hr><b>Peers &amp; Access</b>")

        # Cluster settings
        self._send_all(conn, "<label for='peers'>Peers</label><input id='peers' name='peers' value='%s'>" % _html_escape(peers_str))
        self._send_all(conn, "<label for='allowed_cidrs'>Allowed CIDRs</label><input id='allowed_cidrs' name='allowed_cidrs' value='%s'>" % _html_escape(cidr_str))

        self._send_all(conn, "<div style='height:12px'></div><button class='b red' type='submit'>Save &amp; Reboot</button></form>")

        # JS to disable IP fields when static unchecked (kept tiny; placed at end for captive portal friendliness)
        self._send_all(conn,
            "<script>"
            "function t(){var d=!document.getElementById('use_static_ip').checked;"
            "['ip','mask','gw','dns'].forEach(function(i){var e=document.getElementById(i);if(e)e.disabled=d;});}"
            "var c=document.getElementById('use_static_ip');if(c)c.onchange=t;t();"
            "</script>"
        )

        if FW_UPDATE_ENABLE:
            pin_q = ""
            if self.require_pin and (self.pin is not None):
                try:
                    pin_q = "?pin=%04d" % int(self.pin)
                except Exception:
                    pin_q = ""
            self._send_all(conn, "<hr><b>Firmware</b>")
            self._send_all(conn, "<div class='s'><a href='/u%s'>Upload update</a></div>" % pin_q)

        # Maintenance
        pin_q2 = ""
        if self.require_pin and (self.pin is not None):
            try:
                pin_q2 = "?pin=%04d" % int(self.pin)
            except Exception:
                pin_q2 = ""
        self._send_all(conn, "<hr><b>Maintenance</b>")
        self._send_all(conn, "<form method='POST' action='/rmold%s'><button class='b red' type='submit'>Delete old app files</button></form>" % pin_q2)
        self._send_all(conn, "<form method='POST' action='/rmtmp%s'><button class='b red' type='submit'>Clean .tmp files</button></form>" % pin_q2)

        self._send_all(conn, "<div class='s'>Portal window: %ds after boot on STA/wired; AP stays up until reboot/save.</div>" % int(CONFIG_PORTAL_WINDOW_S))
        self._page_end(conn)

    def _render_update(self, conn, nic_mode, ip_info):
        # Firmware update page (uploads raw .py as application candidate)
        gc.collect()
        self._page_begin(conn, "Update", "TimePico Update")
        self._send_all(conn, "<p class='m'><b>Hostname:</b> %s<br><b>Mode:</b> %s<br><b>IP:</b> %s</p>" %
                     (_html_escape(HOSTNAME), _html_escape(nic_mode), _html_escape(ip_info)))
        pin_q = ""
        pin_val = ""
        if self.require_pin and (self.pin is not None):
            try:
                pin_val = "%04d" % int(self.pin)
                pin_q = "?pin=%s" % pin_val
            except Exception:
                pin_val = ""
                pin_q = ""
        self._send_all(conn, "<hr><b>Firmware update</b>")
        self._send_all(conn, "<div class='s'>Upload a new application file (MicroPython .py) to stage an update. The device will reboot twice.</div>")
        self._send_all(conn, "<div style='height:8px'></div><input type='file' id='f' accept='.py'>")
        self._send_all(conn, "<div style='height:10px'></div><button class='b red' type='button' onclick='u()'>Upload &amp; Apply</button><hr><b>Update from URL</b><div class='s'>Optionally fetch a raw <code>app.py</code> from GitHub (or any HTTPS URL) using your browser, then upload it to this device. This avoids the Pico needing to make outbound TLS connections.</div><div style='height:8px'></div><input id='url' placeholder='https://raw.githubusercontent.com/ORG/REPO/BRANCH/app.py' style='width:100%;max-width:520px'><div style='height:10px'></div><button class='b blue' type='button' onclick='g()'>Fetch &amp; Apply</button>")
        self._send_all(conn, "<pre id='o' class='s'></pre>")
        # Tiny JS uploader: sends file as raw body (not multipart) to keep server parsing simple
        self._send_all(conn,
            "<script>"
            "var cur='%s',pin='%s',durl='%s';""try{var e=document.getElementById('url');if(e&&!e.value)e.value=durl;}catch(e){}"
            "function u(){var o=document.getElementById('o');var f=document.getElementById('f').files[0];if(!f){o.textContent='Select a .py file first.';return;}o.textContent='Uploading...';fetch('/fw?pin='+pin,{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:f}).then(r=>r.text()).then(t=>{o.textContent=t;}).catch(e=>{o.textContent='Error: '+e;});}"
            "function g(){var o=document.getElementById('o');var url=(document.getElementById('url').value||'').trim();if(!url){o.textContent='Enter a raw URL first.';return;}o.textContent='Fetching...';fetch(url).then(r=>{if(!r.ok)throw new Error('HTTP '+r.status);return r.blob();}).then(b=>b.slice(0,8192).text().then(h=>({blob:b,head:h}))).then(obj=>{var rv='(unknown)';var s=obj.head||'';var i=s.indexOf('TIMEPICO_VERSION');if(i>=0){var dq=String.fromCharCode(34),sq=String.fromCharCode(39);var j=s.indexOf(dq,i),k=s.indexOf(dq,j+1);if(j>=0&&k>j){rv=s.substring(j+1,k);}else{j=s.indexOf(sq,i);k=s.indexOf(sq,j+1);if(j>=0&&k>j)rv=s.substring(j+1,k);}}o.textContent='Current: '+cur+'\\nRemote: '+rv+'\\nUploading...';return fetch('/fw?pin='+pin,{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:obj.blob}).then(r=>r.text()).then(t=>{o.textContent='Current: '+cur+'\\nRemote: '+rv+'\\n'+t;});}).catch(e=>{o.textContent='Error: '+e;});}"
            "</script>" % (_html_escape(TIMEPICO_VERSION), _html_escape(pin_val), _html_escape(DEFAULT_UPDATE_URL))
        )
        self._send_all(conn, "<div class='s'><a href='/%s'>Back to config</a></div>" % pin_q)
        self._page_end(conn)

    def _handle_fw_upload(self, conn, lines, body0, clen, q):
        # Stream firmware upload to FW_CANDIDATE_FILE without buffering entire body.
        if not FW_UPDATE_ENABLE:
            self._render_text(conn, "404 Not Found", "Not Found\n")
            return
        # Auth
        if self.require_pin:
            pin_in = (q.get('pin', '') or '')
            if not self._pin_ok(pin_in):
                self._render_text(conn, "403 Forbidden", "Forbidden\n")
                return

        if clen <= 0:
            self._render_text(conn, "400 Bad Request", "Missing Content-Length\n")
            return
        if FW_MAX_UPLOAD_BYTES and (clen > int(FW_MAX_UPLOAD_BYTES)):
            self._render_text(conn, "413 Payload Too Large", "Too large\n")
            return

        free_b = _fs_free_bytes()
        # leave a little margin for filesystem metadata
        if free_b and (clen > (free_b - 4096)):
            self._render_text(conn, "507 Insufficient Storage", "Not enough free space\n")
            return

        # Remove any prior candidate
        try:
            os.remove(FW_CANDIDATE_FILE)
        except Exception:
            pass

        # Optional hash
        h = None
        try:
            import uhashlib
            h = uhashlib.sha256()
        except Exception:
            h = None

        written = 0
        try:
            f = open(FW_CANDIDATE_FILE, 'wb')
        except Exception:
            self._render_text(conn, "500 Internal Server Error", "Cannot open file\n")
            return

        try:
            if body0:
                f.write(body0)
                written = len(body0)
                if h:
                    try:
                        h.update(body0)
                    except Exception:
                        pass
            remaining = clen - written
            while remaining > 0:
                try:
                    chunk = conn.recv(1024 if remaining > 1024 else remaining)
                except Exception:
                    chunk = b''
                if not chunk:
                    break
                f.write(chunk)
                written += len(chunk)
                remaining -= len(chunk)
                if h:
                    try:
                        h.update(chunk)
                    except Exception:
                        pass
                if (written & 0x1FFF) == 0:
                    gc.collect()
        finally:
            try:
                f.close()
            except Exception:
                pass

        if written != clen:
            try:
                os.remove(FW_CANDIDATE_FILE)
            except Exception:
                pass
            self._render_text(conn, "400 Bad Request", "Upload incomplete\n")
            return

        digest = ""
        if h:
            try:
                digest = ubinascii.hexlify(h.digest()).decode('utf-8')
            except Exception:
                digest = ""

        # Mark pending update for loader
        try:
            pf = open(FW_PENDING_FILE, 'w')
            pf.write(FW_CANDIDATE_FILE + "\n")
            pf.write(str(int(clen)) + "\n")
            pf.write(digest + "\n")
            pf.close()
        except Exception:
            pass
        msg = "OK staged update (%d bytes)\n" % int(clen)
        if digest:
            msg += "SHA256: %s\n" % digest
        else:
            msg += "SHA256: (unavailable)\n"
        msg += "Rebooting...\n"
        self._render_text(conn, "200 OK", msg)
        try:
            conn.close()
        except Exception:
            pass
        time.sleep_ms(600)
        machine.reset()


    def _handle_rmold(self, conn, q):
        # Remove old app versions / OTA staging files (manual maintenance).
        # Files are removed only if they exist.
        if self.require_pin:
            pin_in = (q.get('pin', '') or '')
            if not self._pin_ok(pin_in):
                self._render_text(conn, "403 Forbidden", "Forbidden\n")
                return

        try:
            import os
        except Exception:
            self._render_text(conn, "500 Internal Server Error", "OS unavailable\n")
            return

        targets = ["app_prev.py", "app_bad.py", "app_next.py", "update_pending.txt"]
        removed = []
        kept = []
        for fn in targets:
            try:
                os.remove(fn)
                removed.append(fn)
            except Exception:
                kept.append(fn)

        free_b = _fs_free_bytes()
        msg = "Cleanup complete.\n"
        msg += "Removed: %s\n" % (", ".join(removed) if removed else "(none)")
        msg += "Remaining (not found / could not remove): %s\n" % (", ".join(kept) if kept else "(none)")
        if free_b is not None:
            msg += "Free bytes: %d\n" % int(free_b)
        self._render_text(conn, "200 OK", msg)

    def _handle_rmtmp(self, conn, q):
        # Remove any lingering *.tmp files (e.g., config write temp files).
        if self.require_pin:
            pin_in = (q.get('pin', '') or '')
            if not self._pin_ok(pin_in):
                self._render_text(conn, "403 Forbidden", "Forbidden\n")
                return

        try:
            import os
        except Exception:
            self._render_text(conn, "500 Internal Server Error", "OS unavailable\n")
            return

        removed = []
        try:
            it = None
            try:
                it = os.ilistdir()
            except Exception:
                it = os.listdir()
            for e in it:
                fn = e[0] if isinstance(e, tuple) else e
                try:
                    if fn and fn.endswith(".tmp"):
                        try:
                            os.remove(fn)
                            removed.append(fn)
                        except Exception:
                            pass
                except Exception:
                    pass
        except Exception:
            pass

        free_b = _fs_free_bytes()
        msg = "TMP cleanup complete.\n"
        msg += "Removed: %s\n" % (", ".join(removed) if removed else "(none)")
        if free_b is not None:
            msg += "Free bytes: %d\n" % int(free_b)
        self._render_text(conn, "200 OK", msg)
    def _render_text(self, conn, code="200 OK", body="OK\n"):
        self._send_all(conn, self._http_header(code, "text/plain; charset=utf-8"))
        self._send_all(conn, body)


    def _render_saved_html(self, conn, nic_mode, ip_info):
        # Confirmation page that auto-loads '/' after 30 seconds.
        gc.collect()
        self._page_begin(conn, "Saved", "TimePico Config", refresh_s=30, refresh_url="/")
        self._send_all(conn, "<p class='m'><b>Hostname:</b> %s<br><b>Mode:</b> %s</p>" %
                             (_html_escape(HOSTNAME), _html_escape(nic_mode)))
        self._send_all(conn, "<div><b>Saved.</b> Rebooting...</div>")
        self._send_all(conn, "<div class='s'>This page will try to open the portal again in ~30 seconds.</div>")
        self._page_end(conn)

    def _pin_ok(self, pin_in: str) -> bool:
        if not self.require_pin:
            return True
        if self.pin is None:
            return False
        try:
            return str(pin_in).strip() == ("%04d" % int(self.pin))
        except Exception:
            return False

    def poll(self, nic_mode="unknown", ip_info=""):
        if not self.active():
            self.stop_if_expired()
            return

        try:
            conn, addr = self.sock.accept()
        except Exception:
            return

        # Someone is using the portal; show PIN for a while (STA/wired).
        self.touch_access()

        try:
            gc.collect()
            try:
                conn.settimeout(1.0)
            except Exception:
                pass

            req = conn.recv(2048)
            if not req:
                try:
                    conn.close()
                except Exception:
                    pass
                return

            # Decode head safely (avoid keyword args; some MicroPython ports reject them)
            parts = req.split(b"\r\n\r\n", 1)
            head = parts[0].decode("utf-8", "ignore")
            body0 = parts[1] if len(parts) > 1 else b""
            body = body0.decode("utf-8", "ignore") if body0 else ""
            lines = head.split("\r\n")
            if not lines:
                self._render_text(conn, "400 Bad Request", "Malformed\n")
                return

            first = lines[0].split()
            if len(first) < 2:
                self._render_text(conn, "400 Bad Request", "Malformed\n")
                return

            method = first[0].upper()
            path = first[1]

            # Parse query string for GET /?pin=####
            qs = ""
            if "?" in path:
                path, qs = path.split("?", 1)
            q = _parse_qs(qs) if qs else {}

            # content-length if POST
            clen = 0
            if method == "POST":
                for ln in lines[1:]:
                    if ln.lower().startswith("content-length:"):
                        try:
                            clen = int(ln.split(":", 1)[1].strip())
                        except Exception:
                            clen = 0
                # For small form posts (/save), read the remainder (bounded).
                if path == "/save" and clen and (len(body0) < clen) and (clen < 4096):
                    need = clen - len(body0)
                    while need > 0:
                        try:
                            more = conn.recv(512 if need > 512 else need)
                        except Exception:
                            break
                        if not more:
                            break
                        body0 += more
                        need -= len(more)
                    body = body0.decode("utf-8", "ignore") if body0 else ""

            if method == "GET" and path == "/":
                if self.require_pin:
                    pin_in = (q.get("pin", "") or "")
                    if not self._pin_ok(pin_in):
                        self._render_unlock(conn, nic_mode, ip_info)
                    else:
                        self._render_config(conn, nic_mode, ip_info)
                else:
                    self._render_config(conn, nic_mode, ip_info)

            elif method == "POST" and path == "/save":
                form = _parse_qs(body)
                if self.require_pin and (not self._pin_ok(form.get("pin", ""))):
                    self._render_text(conn, "403 Forbidden", "Forbidden\n")
                    return

                # Update CFG dict
                CFG["net_mode"] = "wifi"  # portal config targets wifi
                CFG["wifi_ssid"] = form.get("wifi_ssid", WIFI_SSID)
                CFG["wifi_password"] = form.get("wifi_password", WIFI_PASSWORD)

                CFG["use_static_ip"] = ("use_static_ip" in form)

                ip = form.get("ip", STATIC_IP[0] if len(STATIC_IP)==4 else "")
                mask = form.get("mask", STATIC_IP[1] if len(STATIC_IP)==4 else "")
                gw = form.get("gw", STATIC_IP[2] if len(STATIC_IP)==4 else "")
                dns = form.get("dns", STATIC_IP[3] if len(STATIC_IP)==4 else "")
                CFG["static_ip"] = [ip.strip(), mask.strip(), gw.strip(), dns.strip()]

                peers = form.get("peers", "")
                peer_list = []
                for p in peers.replace(";", ",").replace(" ", ",").split(","):
                    p = p.strip()
                    if p:
                        peer_list.append(p)
                CFG["peers"] = peer_list

                cidrs = form.get("allowed_cidrs", "")
                cidr_list = []
                for c in cidrs.replace(";", ",").replace(" ", ",").split(","):
                    c = c.strip()
                    if c:
                        cidr_list.append(c)
                CFG["allowed_cidrs"] = cidr_list

                # Hostname
                hn = form.get("hostname", "").strip().lower()
                hn = _sanitize_hostname(hn)
                if not hn:
                    hn = "timepico-%04d" % (unique_u32() % 10000)
                CFG["hostname"] = hn

                # Time display format
                tf = (form.get("time_display", "24") or "24").strip()
                CFG["time_display_24h"] = (tf != "12")                # Timezone offset hours (numeric; may be fractional)
                tz = (form.get("tz_offset_hours", "") or "").strip()
                if tz:
                    CFG["tz_offset_hours"] = tz
                else:
                    CFG["tz_offset_hours"] = "0"


                # Brightness schedule
                try:
                    bd = int(float((form.get("brightness_day", "") or "").strip() or str(DISPLAY_BRIGHTNESS_DAY)))
                except Exception:
                    bd = int(DISPLAY_BRIGHTNESS_DAY)
                try:
                    bn = int(float((form.get("brightness_night", "") or "").strip() or str(DISPLAY_BRIGHTNESS_NIGHT)))
                except Exception:
                    bn = int(DISPLAY_BRIGHTNESS_NIGHT)
                CFG["brightness_day"] = clamp(bd, 0, 15)
                CFG["brightness_night"] = clamp(bn, 0, 15)

                _save_cfg(CFG)
                self._render_saved_html(conn, nic_mode, ip_info)
                try:
                    conn.close()
                except Exception:
                    pass
                time.sleep_ms(800)
                machine.reset()

            
            elif method == "GET" and path == "/u":
                if self.require_pin:
                    pin_in = (q.get("pin", "") or "")
                    if not self._pin_ok(pin_in):
                        self._render_unlock(conn, nic_mode, ip_info)
                    else:
                        self._render_update(conn, nic_mode, ip_info)
                else:
                    self._render_update(conn, nic_mode, ip_info)

            elif method == "POST" and path == "/fw":
                # Firmware upload (raw body)
                self._handle_fw_upload(conn, lines, body0, clen, q)

            elif method == "POST" and path == "/rmold":
                self._handle_rmold(conn, q)

            elif method == "POST" and path == "/rmtmp":
                self._handle_rmtmp(conn, q)

            else:
                # fast 404 (also handles /favicon.ico)
                self._render_text(conn, "404 Not Found", "Not Found\n")

        except Exception as e:
            # Print error to serial for debugging, and return a simple page.
            try:
                log("Portal error:", repr(e))
            except Exception:
                pass
            try:
                self._render_text(conn, "500 Internal Server Error", "Error\n")
            except Exception:
                pass
        finally:
            try:
                conn.close()
            except Exception:
                pass


def _apply_hostname_to_wlan(wlan):
    # Best-effort DHCP hostname/stack hostname (varies by MicroPython build)
    try:
        if hasattr(network, 'hostname'):
            try:
                network.hostname(HOSTNAME)
                return
            except Exception:
                pass
    except Exception:
        pass
    try:
        wlan.config(dhcp_hostname=HOSTNAME)
    except Exception:
        pass

# ---------------- Network init (STA + AP fallback) ----------------
def network_init_with_portal(boot_ms):
    if network is None:
        raise RuntimeError("network module not available")

    portal = ConfigPortal(boot_ms)

    if NET_MODE == "wifi":
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        _apply_hostname_to_wlan(wlan)
        ok = False
        if WIFI_SSID and WIFI_SSID != "YOUR_SSID":
            try:
                if not wlan.isconnected():
                    log("Connecting WiFi STA...")
                    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
                t0 = ticks_ms()
                while not wlan.isconnected():
                    if ticks_diff(ticks_ms(), t0) > int(WIFI_CONNECT_TIMEOUT_S * 1000):
                        break
                    time.sleep_ms(200)
                ok = wlan.isconnected()
            except Exception:
                ok = False

        if ok:
            if USE_STATIC_IP:
                try:
                    wlan.ifconfig(STATIC_IP)
                except Exception as e:
                    log("Static IP set failed:", e)
            log("WiFi STA up:", wlan.ifconfig())
            portal.start(ap_mode=False)
            return wlan, "sta", portal

        # STA failed -> AP fallback
        if AP_FALLBACK_ENABLE:
            try:
                wlan.active(False)
            except Exception:
                pass
            ap = network.WLAN(network.AP_IF)
            ap.active(True)
            ssid = "%s-%04d" % (AP_SSID_PREFIX, unique_u32() % 10000)
            try:
                # Force OPEN AP (no password). Portal access is protected by a 4-digit PIN.
                try:
                    # Most MicroPython ports: AUTH_OPEN constant exists
                    ap.config(essid=ssid, authmode=getattr(network, 'AUTH_OPEN', 0))
                except Exception:
                    try:
                        # Some ports use 'security' field (0=open)
                        ap.config(essid=ssid, security=0)
                    except Exception:
                        # Fallback: set ESSID and try to clear password
                        ap.config(essid=ssid)
                        try:
                            ap.config(password=None)
                        except Exception:
                            try:
                                ap.config(password='')
                            except Exception:
                                pass
            except Exception:
                pass
            try:
                ap.ifconfig(AP_IP)
            except Exception:
                pass
            log("AP fallback up:", ap.ifconfig(), "SSID:", ssid)
            portal.start(ap_mode=True)
            return ap, "ap", portal

        raise RuntimeError("WiFi connect failed and AP fallback disabled")

    if NET_MODE == "wiznet":
        spi = machine.SPI(
            WIZ_SPI_ID,
            baudrate=20_000_000,
            sck=Pin(WIZ_SCK),
            mosi=Pin(WIZ_MOSI),
            miso=Pin(WIZ_MISO),
        )
        cs = Pin(WIZ_CS, Pin.OUT)
        rst = Pin(WIZ_RST, Pin.OUT)
        nic = network.WIZNET5K(spi, cs, rst)
        nic.active(True)
        if USE_STATIC_IP:
            nic.ifconfig(STATIC_IP)
        log("Wiznet up:", nic.ifconfig())
        portal.start(ap_mode=False)
        return nic, "wired", portal

    raise ValueError("NET_MODE must be 'wifi' or 'wiznet'")

def init_led():
    try:
        return Pin("LED", Pin.OUT)
    except Exception:
        return Pin(25, Pin.OUT)

# ---------------- PPS IRQ plumbing ----------------
_pps_tick = 0
_pps_flag = False

def _pps_irq(pin):
    global _pps_tick, _pps_flag
    _pps_tick = ticks_us()
    _pps_flag = True

def setup_pps():
    if not ENABLE_PPS:
        return None
    pull = Pin.PULL_DOWN if PPS_PULLDOWN else None
    p = Pin(PPS_GPIO, Pin.IN, pull)
    p.irq(trigger=Pin.IRQ_RISING, handler=_pps_irq)
    log("PPS IRQ enabled on GPIO", PPS_GPIO)
    return p

# ---------------- Main ----------------
def main():
    global _ntp_flash_dirty
    gc.collect()
    boot_ms = ticks_ms()
    led = init_led()
    led.value(0)

    wdt = None
    if ENABLE_WDT and WDT is not None:
        try:
            wdt = WDT(timeout=WDT_TIMEOUT_MS)
            log("WDT enabled:", WDT_TIMEOUT_MS, "ms")
        except Exception as e:
            log("WDT not available:", e)

    i2c = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
    disp = HT16K33_7Seg(i2c, HT16K33_ADDR, DISPLAY_BRIGHTNESS_DAY)
    current_brightness = DISPLAY_BRIGHTNESS_DAY
    disp.show_dashes()
    disp.draw()

    uart = UART(GPS_UART_ID, baudrate=GPS_BAUD, tx=Pin(GPS_TX), rx=Pin(GPS_RX))
    uart.init(bits=8, parity=None, stop=1)

    setup_pps()

    nic, nic_mode, portal = network_init_with_portal(boot_ms)

    ntp = NTPServer()
    peers = PeerMonitor(PEERS)

    last_display_ms = 0
    colon = False
    last_colon_ms = 0

    # Throbber state
    throb_idx = 0
    throb_last_ms = 0

    while True:
        if wdt:
            wdt.feed()

        now = ticks_ms()
        # Update NTP request flash state machine
        try:
            ntp_flash_update(now)
        except Exception:
            pass

        # LED status: suspect fast blink, stable serve slow blink
        if CLOCK.suspect:
            led.value(1 if (now // 150) % 2 else 0)
        elif CLOCK.can_serve_ntp():
            led.value(1 if (now // 500) % 2 else 0)
        else:
            led.value(0)

        # Config portal (time-limited on STA/wired)
        portal.poll(nic_mode=nic_mode, ip_info=str(getattr(nic, "ifconfig", lambda: ("", "", "", ""))()))
        portal.stop_if_expired()

        # PPS event
        global _pps_flag, _pps_tick
        if _pps_flag:
            _pps_flag = False
            CLOCK.pps_pulse(_pps_tick)

        # GPS UART feed
        n = uart.any()
        if n:
            d = uart.read(n)
            if d:
                GPS.feed(d)
                if GPS.last_rmc_unix_us is not None:
                    unix_us = GPS.last_rmc_unix_us
                    valid = GPS.last_rmc_valid
                    fix_type = GPS.fix_type
                    GPS.last_rmc_unix_us = None
                    CLOCK.gps_rmc(unix_us, valid, fix_type, ticks_us())

        # Peer protocol
        peers.handle_incoming()
        peers.send_pings_if_due()
        med_off, good, count = peers.evaluate()
        CLOCK.consider_peers(med_off, good, count)

        # NTP
        ntp.serve_once()

        # Display update (also refresh during NTP debug flash toggles)
        if (ticks_diff(now, last_display_ms) > 200) or (_ntp_flash_dirty):
            last_display_ms = now

            # If portal PIN should be displayed (AP always; STA briefly after access), show it.
            show_pin = False
            pin_val = None
            try:
                show_pin = portal.should_show_pin()
                pin_val = portal.current_pin() if show_pin else None
            except Exception:
                show_pin = False
                pin_val = None

            # Render either PIN (when portal is being used) or normal time display
            if show_pin and (pin_val is not None):
                # Show PIN without colon; keep decimal points dark unless NTP debug flash is active.
                try:
                    disp.clear_dps()
                    disp.show_number4(int(pin_val))
                except Exception:
                    pass
                # No throbber while showing PIN.
                throbber_active = False
            else:
                # throbber update (GPS acquisition / stabilization)
                # Run ONLY while we are acquiring/trying to re-stabilize GPS+PPS; otherwise keep all decimal points dark.
                full_lock = (CLOCK.gps_lock and CLOCK.pps_locked and CLOCK.pps_present())
                throbber_active = (not CLOCK.suspect) and ((not CLOCK.ever_stabilized) or (not full_lock))

                if ticks_diff(now, throb_last_ms) > THROBBER_STEP_MS:
                    throb_last_ms = now
                    throb_idx = (throb_idx + 1) % 4

                # base display content
                now_us = CLOCK.now_unix_us()
                if CLOCK.suspect:
                    if current_brightness != DISPLAY_BRIGHTNESS_DAY:
                        try:
                            disp.set_brightness(DISPLAY_BRIGHTNESS_DAY)
                        except Exception:
                            pass
                        current_brightness = DISPLAY_BRIGHTNESS_DAY
                    disp.show_error()
                elif now_us is None or (not CLOCK.ever_stabilized):
                    # Default to day brightness when time is unknown
                    if current_brightness != DISPLAY_BRIGHTNESS_DAY:
                        try:
                            disp.set_brightness(DISPLAY_BRIGHTNESS_DAY)
                        except Exception:
                            pass
                        current_brightness = DISPLAY_BRIGHTNESS_DAY
                    disp.show_dashes()
                else:
                    unix_s = (now_us // 1_000_000) + int(TZ_OFFSET_S)
                    day_s = unix_s % 86400
                    hour24 = day_s // 3600
                    hh = hour24
                    mm = (day_s % 3600) // 60

                    # Brightness schedule (based on local 24h time)
                    try:
                        desired_b = DISPLAY_BRIGHTNESS_DAY if (6 <= int(hour24) < 18) else DISPLAY_BRIGHTNESS_NIGHT
                    except Exception:
                        desired_b = DISPLAY_BRIGHTNESS_DAY
                    if desired_b != current_brightness:
                        try:
                            disp.set_brightness(desired_b)
                        except Exception:
                            pass
                        current_brightness = desired_b

                    if not TIME_DISPLAY_24H:
                        # 12-hour format (no AM/PM indicator)
                        hh = int(hh) % 12
                        if hh == 0:
                            hh = 12

                    # Colon behavior unchanged
                    if CLOCK.gps_lock and CLOCK.pps_locked:
                        if ticks_diff(now, last_colon_ms) > 500:
                            last_colon_ms = now
                            colon = not colon
                    else:
                        colon = True
                    disp.show_hhmm(int(hh), int(mm), colon)

                # apply throbber to decimal points (only when acquiring)
                disp.clear_dps()
                if throbber_active:
                    disp.set_dp(throb_idx, True)

            # Overlay: NTP debug flash on right-most decimal point (DP3)
            try:
                if ntp_flash_dp3_state():
                    disp.set_dp(3, True)
            except Exception:
                pass

            disp.draw()
            # clear flash dirty flag after pushing to display
            _ntp_flash_dirty = False
        if (now % 5000) < 20:
            gc.collect()

        time.sleep_ms(5)

if __name__ == "__main__":
    main()
