"""
Servo calibration tool.

Drives the pan/tilt servos to specific angles from the terminal over plain
HTTP (GET /control?panabs=...&tiltabs=...), centering both to 90 on start so
you can mount the servo horns straight before final assembly.

Usage:
    python calibrate_servos.py            # uses ESP32_IP below
    python calibrate_servos.py 192.168.1.15

Commands (type one per line):
    <enter>     center both servos (90, 90)
    p <deg>     set pan angle 0-180, e.g. "p 45"
    t <deg>     set tilt angle 0-125, e.g. "t 120" (larger = further down)
    p+ / p-     nudge pan +/-5 degrees
    t+ / t-     nudge tilt +/-5 degrees
    q           quit
"""

import http.client
import sys
import urllib.parse

ESP32_IP = <esp32-ip>  # printed on the OLED / serial monitor after boot
CONTROL_PORT = 80

# Per-axis limits, mirroring include/wifi_config.h. Tilt stops at 125 because
# the head fouls the body past that 
PAN_MIN, PAN_MAX, PAN_CENTER = 0, 180, 90
TILT_MIN, TILT_MAX, TILT_CENTER = 0, 125, 90
NUDGE_DEG = 5

_conn = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def send_angles(pan, tilt):
    """Send both absolute angles. Reopens the connection if it went stale."""
    global _conn

    query = urllib.parse.urlencode({"panabs": int(pan), "tiltabs": int(tilt)})
    for attempt in (1, 2):
        try:
            if _conn is None:
                _conn = http.client.HTTPConnection(ESP32_IP, CONTROL_PORT, timeout=3)
            _conn.request("GET", f"/control?{query}")
            _conn.getresponse().read()
            print(f"-> pan={int(pan)} tilt={int(tilt)}")
            return
        except Exception as exc:
            try:
                if _conn is not None:
                    _conn.close()
            except Exception:
                pass
            _conn = None
            if attempt == 2:
                print(f"send failed: {exc}")


def main():
    global ESP32_IP
    if len(sys.argv) > 1:
        ESP32_IP = sys.argv[1]

    print(f"Controlling ESP32-CAM at http://{ESP32_IP}:{CONTROL_PORT}/control")

    pan, tilt = PAN_CENTER, TILT_CENTER
    send_angles(pan, tilt)
    print(__doc__)

    while True:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break

        if cmd == "":
            pan, tilt = PAN_CENTER, TILT_CENTER
        elif cmd == "q":
            break
        elif cmd == "p+":
            pan = clamp(pan + NUDGE_DEG, PAN_MIN, PAN_MAX)
        elif cmd == "p-":
            pan = clamp(pan - NUDGE_DEG, PAN_MIN, PAN_MAX)
        elif cmd == "t+":
            tilt = clamp(tilt + NUDGE_DEG, TILT_MIN, TILT_MAX)
        elif cmd == "t-":
            tilt = clamp(tilt - NUDGE_DEG, TILT_MIN, TILT_MAX)
        elif cmd.startswith("p "):
            try:
                pan = clamp(int(cmd.split()[1]), PAN_MIN, PAN_MAX)
            except (IndexError, ValueError):
                print(f"usage: p <{PAN_MIN}-{PAN_MAX}>")
                continue
        elif cmd.startswith("t "):
            try:
                tilt = clamp(int(cmd.split()[1]), TILT_MIN, TILT_MAX)
            except (IndexError, ValueError):
                print(f"usage: t <{TILT_MIN}-{TILT_MAX}>")
                continue
        else:
            print("unknown command - see usage above")
            continue

        send_angles(pan, tilt)

    if _conn is not None:
        try:
            _conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
