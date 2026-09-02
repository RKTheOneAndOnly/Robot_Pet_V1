"""
Robot Pet tracker.

Plain HTTP throughout, matching the Espressif CameraWebServer layout:

  * Video   - GET http://<ip>:81/stream   (multipart MJPEG)
  * Control - GET http://<ip>:80/control?pan=-2&tilt=1&mood=happy

Relative commands carry a signed DELTA IN DEGREES. This side never needs to
know the absolute angle - it only says "turn this far, this way" - but it does
scale the step to how far off-centre the target is. A fixed step regardless of
error is what makes the head hunt back and forth around the centre. The ESP32
still owns the angle and clamps to each axis's limits.

Two tracking modes:

  face   - Haar cascade, the default
  color  - HSV colour blob, for holding up an apple or any coloured object.
           Click the object in the video window to sample its colour.

Tuning lives in the constants below. The live slider window is still in the
code but switched off - set USE_SLIDERS = True to bring it back.

LATENCY MATTERS MORE THAN ANYTHING ELSE HERE. A tracker steering from stale
frames chases where the target *was*, overshoots, and corrects late, which
reads as lag no matter how the gains are set. The pipeline is built to keep
the newest frame and throw everything else away:

  * detection runs on a downscaled copy (DETECT_WIDTH), which is where most
    of the per-frame time went
  * control requests go out on their own thread, so waiting for the ESP32 to
    answer never stalls tracking
  * the stream reader takes whatever bytes are available rather than blocking
    for a fixed-size read, and keeps only the last complete frame

The HUD shows fps and frame age in ms - watch those while tuning.

Usage:
    pip install -r requirements.txt
    python track.py                       # face mode, uses ESP32_IP below
    python track.py 192.168.1.15          # face mode, explicit IP
    python track.py 192.168.1.15 color    # colour-blob mode

Keys:  f = face mode   c = colour mode   q = quit
       click on the video to sample the colour under the cursor

To sanity-check the camera on its own, open http://<esp32-ip>:81/stream in a
browser - that path is independent of this script entirely.
"""

import http.client
import random
import sys
import threading
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------
# ESP32_IP = <esp32-ip>      # printed on the OLED / serial monitor after boot
ESP32_IP = "192.168.1.41"     # printed on the OLED / serial monitor after boot
CONTROL_PORT = 80          # GET /control?pan=-2&tilt=1&mood=happy
STREAM_PORT = 81           # GET /stream (multipart MJPEG)

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
DEADZONE_PX = 30      # ignore errors smaller than this (avoids hunting)
KP = 10               # degrees of servo movement per pixel of error
MAX_STEP_DEG = 4      # ceiling on a single move, whatever the error
SMOOTHING = 0.55      # centroid EMA: 0 = raw and jumpy, 0.9 = heavy and laggy

SEND_INTERVAL_S = 0.08

# Minimum gap between mood updates
MOOD_INTERVAL_S = 2.0

# Expressions the firmware understands (MOOD_NAMES in src/main.cpp):
#   neutral happy curious angry sleepy surprised
#   love sad suspicious dizzy wink bored excited

QUIRKS_WHEN_TRACKING = ["love", "excited", "wink", "surprised", "happy"]
QUIRKS_WHEN_IDLE = ["bored", "suspicious", "sad", "dizzy", "neutral"]

QUIRK_MIN_GAP_S = 8.0     # earliest a spontaneous expression can appear
QUIRK_MAX_GAP_S = 22.0    # latest
QUIRK_HOLD_S = 4.0        # how long it holds before reverting to the base


# Set True to get the interactive slider window back.
USE_SLIDERS = False

PAN_MIN, PAN_MAX, PAN_CENTER = 0, 180, 90
TILT_MIN, TILT_MAX, TILT_CENTER = 0, 125, 90

# Flip if an axis chases the target the wrong way.
PAN_INVERT = False
TILT_INVERT = False

# After this many consecutive frames with no target, ease back to center.
LOST_FRAMES_BEFORE_RECENTER = 30

# Colour mode thresholds (used when USE_SLIDERS is False).
HUE_TOL = 10
SAT_MIN = 90
VAL_MIN = 60
MIN_BLOB_AREA = 120   # in downscaled pixels - ignore specks smaller than this

RECONNECT_DELAY_S = 2.0

VIDEO_WIN = "Robot Pet Tracker"
TUNE_WIN = "Tuning"

# ---------------------------------------------------------------------------
# Shared state between the threads and the main loop.
# ---------------------------------------------------------------------------
_frame_lock = threading.Lock()
_latest_jpeg = None      # (bytes, captured_at) - newest frame only
_stop = threading.Event()

_track_mode = "face"
_picked_hue = 0          # colour mode: hue sampled by clicking
_picked = False
_click = None            # pending click coords, consumed by the main loop


# ---------------------------------------------------------------------------
# Video: MJPEG reader thread
# ---------------------------------------------------------------------------
def mjpeg_reader(url):
    """Pull the multipart MJPEG stream and keep only the newest frame.
    """
    global _latest_jpeg

    while not _stop.is_set():
        try:
            print(f"Opening video stream {url} ...")
            with urllib.request.urlopen(url, timeout=10) as resp:
                print("Video stream connected")
                buf = b""
                while not _stop.is_set():
                    chunk = resp.read1(65536)
                    if not chunk:
                        break
                    buf += chunk
                    newest = None
                    while True:
                        start = buf.find(b"\xff\xd8")
                        if start == -1:
                            break
                        end = buf.find(b"\xff\xd9", start + 2)
                        if end == -1:
                            break
                        newest = buf[start:end + 2]
                        buf = buf[end + 2:]

                    if newest is not None:
                        with _frame_lock:
                            _latest_jpeg = (newest, time.time())
                    elif len(buf) > 512 * 1024:
                        buf = b""  # never seen a complete frame - don't grow forever
        except Exception as exc:
            if not _stop.is_set():
                print(f"Video stream error ({exc}); retrying in {RECONNECT_DELAY_S}s")
        if not _stop.is_set():
            time.sleep(RECONNECT_DELAY_S)


# ---------------------------------------------------------------------------
# Control: plain HTTP GETs to /control, issued off the tracking thread
# ---------------------------------------------------------------------------
# One reused keep-alive connection, so a command costs a single small request
# rather than a fresh TCP handshake. There is no session to lose: if the
# connection breaks we simply open another one on the next command.
#
# The send happens on its own thread because a request costs a full round trip
# to the ESP32. Doing that inline stalled the tracking loop on every command -
# frames piled up while we waited for "ok", and the tracker then steered from
# whatever had gone stale in the meantime.
_conn = None
_control_ok = False

_cmd_lock = threading.Lock()
_pending = None
_cmd_ready = threading.Event()


def queue_cmd(params):
    """Hand a command to the sender thread. Never blocks."""
    global _pending

    if not params:
        return
    with _cmd_lock:
        if _pending is None:
            _pending = {}
        for key, value in params.items():
            if key in ("pan", "tilt"):
                _pending[key] = _pending.get(key, 0) + value
            else:
                _pending[key] = value
    _cmd_ready.set()


def _do_send(params):
    global _conn, _control_ok

    query = urllib.parse.urlencode(params)
    for attempt in (1, 2):  # one retry: the kept-alive socket may be stale
        try:
            if _conn is None:
                _conn = http.client.HTTPConnection(ESP32_IP, CONTROL_PORT, timeout=2)
            _conn.request("GET", f"/control?{query}")
            _conn.getresponse().read()
            _control_ok = True
            return
        except Exception:
            try:
                if _conn is not None:
                    _conn.close()
            except Exception:
                pass
            _conn = None
            if attempt == 2:
                _control_ok = False


def sender_thread():
    global _pending

    while not _stop.is_set():
        _cmd_ready.wait(0.1)
        _cmd_ready.clear()
        with _cmd_lock:
            params = _pending
            _pending = None
        if params:
            _do_send(params)

    if _conn is not None:
        try:
            _conn.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Tuning sliders (off by default - set USE_SLIDERS = True to re-enable)
# ---------------------------------------------------------------------------
def _noop(_):
    pass


def build_tuning_window():
    cv2.namedWindow(TUNE_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TUNE_WIN, 420, 300)
    cv2.createTrackbar("deadzone px", TUNE_WIN, DEADZONE_PX, 120, _noop)
    cv2.createTrackbar("KP x1000", TUNE_WIN, int(KP * 1000), 300, _noop)
    cv2.createTrackbar("max step deg", TUNE_WIN, MAX_STEP_DEG, 15, _noop)
    cv2.createTrackbar("smooth x100", TUNE_WIN, int(SMOOTHING * 100), 95, _noop)
    cv2.createTrackbar("send ms", TUNE_WIN, int(SEND_INTERVAL_S * 1000), 300, _noop)
    cv2.createTrackbar("hue tol", TUNE_WIN, HUE_TOL, 60, _noop)
    cv2.createTrackbar("sat min", TUNE_WIN, SAT_MIN, 255, _noop)
    cv2.createTrackbar("val min", TUNE_WIN, VAL_MIN, 255, _noop)


_STATIC_TUNING = {
    "deadzone": DEADZONE_PX,
    "kp": KP,
    "max_step": MAX_STEP_DEG,
    "smooth": SMOOTHING,
    "send_s": SEND_INTERVAL_S,
    "hue_tol": HUE_TOL,
    "sat_min": SAT_MIN,
    "val_min": VAL_MIN,
}


def tuning():
    """Current tuning values - from the sliders if they're enabled."""
    if not USE_SLIDERS:
        return _STATIC_TUNING
    return {
        "deadzone": cv2.getTrackbarPos("deadzone px", TUNE_WIN),
        "kp": cv2.getTrackbarPos("KP x1000", TUNE_WIN) / 1000.0,
        "max_step": max(1, cv2.getTrackbarPos("max step deg", TUNE_WIN)),
        "smooth": cv2.getTrackbarPos("smooth x100", TUNE_WIN) / 100.0,
        "send_s": max(0.01, cv2.getTrackbarPos("send ms", TUNE_WIN) / 1000.0),
        "hue_tol": cv2.getTrackbarPos("hue tol", TUNE_WIN),
        "sat_min": cv2.getTrackbarPos("sat min", TUNE_WIN),
        "val_min": cv2.getTrackbarPos("val min", TUNE_WIN),
    }


def on_mouse(event, x, y, flags, param):
    global _click
    if event == cv2.EVENT_LBUTTONDOWN:
        _click = (x, y)


# ---------------------------------------------------------------------------
# Detectors. Both run on the DOWNSCALED frame and return (cx, cy, w, h) in
# that frame's coordinates, or None.
# ---------------------------------------------------------------------------
def detect_face(small):
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    # scaleFactor 1.2 rather than 1.1: fewer scales to search, noticeably
    # faster, and at this resolution the accuracy difference is negligible.
    faces = _face_cascade.detectMultiScale(
        gray, scaleFactor=1.2, minNeighbors=4, minSize=(24, 24)
    )
    if len(faces) == 0:
        return None
    x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
    return (x + w // 2, y + h // 2, w, h)


def color_mask(hsv, t):
    """Threshold around the picked hue, handling the red wrap-around.

    Red sits at both ends of the hue circle, so an apple needs two ranges
    stitched together rather than one.
    """
    lo_h = _picked_hue - t["hue_tol"]
    hi_h = _picked_hue + t["hue_tol"]
    smin, vmin = t["sat_min"], t["val_min"]

    if lo_h < 0:
        m1 = cv2.inRange(hsv, (0, smin, vmin), (hi_h, 255, 255))
        m2 = cv2.inRange(hsv, (180 + lo_h, smin, vmin), (179, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
    elif hi_h > 179:
        m1 = cv2.inRange(hsv, (lo_h, smin, vmin), (179, 255, 255))
        m2 = cv2.inRange(hsv, (0, smin, vmin), (hi_h - 180, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv, (lo_h, smin, vmin), (hi_h, 255, 255))

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_color(small, t):
    if not _picked:
        return None, None
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, t)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < MIN_BLOB_AREA:
        return None, mask
    x, y, w, h = cv2.boundingRect(biggest)
    return (x + w // 2, y + h // 2, w, h), mask


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    global ESP32_IP, _latest_jpeg, _track_mode, _picked_hue, _picked, _click

    args = list(sys.argv[1:])
    if args:
        ESP32_IP = args[0]
    if len(args) > 1 and args[1] in ("face", "color", "colour"):
        _track_mode = "color" if args[1].startswith("colo") else "face"

    global _face_cascade
    _face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    stream_url = f"http://{ESP32_IP}:{STREAM_PORT}/stream"
    threading.Thread(target=mjpeg_reader, args=(stream_url,), daemon=True).start()
    threading.Thread(target=sender_thread, daemon=True).start()

    cv2.namedWindow(VIDEO_WIN)
    cv2.setMouseCallback(VIDEO_WIN, on_mouse)
    if USE_SLIDERS:
        build_tuning_window()

    print(__doc__)

    # Local shadow of the servo angles, used for the recenter logic and the
    # readout. Accurate, because the deltas we send are real degrees.
    pan_shadow, tilt_shadow = float(PAN_CENTER), float(TILT_CENTER)
    lost_frames = 0
    mood = None
    smooth_x = smooth_y = None
    last_send = 0.0
    last_mood_send = 0.0
    fps = 0.0
    last_frame_time = time.time()

    try:
        while True:
            with _frame_lock:
                item = _latest_jpeg
                _latest_jpeg = None  # only ever process the newest frame

            if item is None:
                time.sleep(0.002)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            jpeg, captured_at = item
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            now = time.time()
            dt = now - last_frame_time
            last_frame_time = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)  # smoothed, so it's readable
            age_ms = (now - captured_at) * 1000.0

            t = tuning()
            h, w = frame.shape[:2]
            center_x, center_y = w / 2, h / 2

            # Downscaled copy for detection - the expensive part of the loop.
            if DETECT_WIDTH and w > DETECT_WIDTH:
                scale = DETECT_WIDTH / float(w)
                small = cv2.resize(frame, (DETECT_WIDTH, int(h * scale)),
                                   interpolation=cv2.INTER_AREA)
            else:
                scale = 1.0
                small = frame

            # Consume a pending click: sample the hue under the cursor and
            # switch to colour mode, so pointing at an apple just works.
            if _click is not None:
                cx, cy = _click
                _click = None
                if 0 <= cx < w and 0 <= cy < h:
                    hsv_px = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[cy, cx]
                    _picked_hue = int(hsv_px[0])
                    _picked = True
                    _track_mode = "color"
                    print(f"Sampled hue={_picked_hue} sat={hsv_px[1]} "
                          f"val={hsv_px[2]} -> colour mode")

            mask = None
            if _track_mode == "color":
                target, mask = detect_color(small, t)
            else:
                target = detect_face(small)

            if target is not None and scale != 1.0:
                target = tuple(int(v / scale) for v in target)

            cmd = {}

            if target is not None:
                lost_frames = 0
                tx, ty, tw, th = target

                # Smooth the centroid before it reaches the controller. Haar
                # boxes and blob edges jitter frame to frame even on a still
                # object, and that noise would otherwise be amplified straight
                # into servo movement.
                a = t["smooth"]
                if smooth_x is None:
                    smooth_x, smooth_y = float(tx), float(ty)
                else:
                    smooth_x = a * smooth_x + (1.0 - a) * tx
                    smooth_y = a * smooth_y + (1.0 - a) * ty

                error_x = smooth_x - center_x
                error_y = smooth_y - center_y

                # Proportional step, rounded to whole degrees
                if abs(error_x) > t["deadzone"]:
                    step = clamp(t["kp"] * error_x, -t["max_step"], t["max_step"])
                    if PAN_INVERT:
                        step = -step
                    step = int(round(step))
                    if step:
                        cmd["pan"] = step
                        pan_shadow = clamp(pan_shadow + step, PAN_MIN, PAN_MAX)

                if abs(error_y) > t["deadzone"]:
                    # error_y > 0 means the target is BELOW frame centre
                    step = clamp(t["kp"] * error_y, -t["max_step"], t["max_step"])
                    if TILT_INVERT:
                        step = -step
                    step = int(round(step))
                    if step:
                        cmd["tilt"] = step
                        tilt_shadow = clamp(tilt_shadow + step, TILT_MIN, TILT_MAX)

                new_mood = "happy" if not cmd else "curious"

                cv2.rectangle(frame, (tx - tw // 2, ty - th // 2),
                              (tx + tw // 2, ty + th // 2), (0, 255, 0), 2)
                cv2.circle(frame, (int(smooth_x), int(smooth_y)), 4, (0, 255, 255), -1)
            else:
                lost_frames += 1
                smooth_x = smooth_y = None
                new_mood = ("neutral" if lost_frames <= LOST_FRAMES_BEFORE_RECENTER
                            else "sleepy")
                if lost_frames > LOST_FRAMES_BEFORE_RECENTER:
                    if abs(pan_shadow - PAN_CENTER) >= 1:
                        step = int(round(clamp(PAN_CENTER - pan_shadow, -2, 2)))
                        if step:
                            cmd["pan"] = step
                            pan_shadow = clamp(pan_shadow + step, PAN_MIN, PAN_MAX)
                    if abs(tilt_shadow - TILT_CENTER) >= 1:
                        step = int(round(clamp(TILT_CENTER - tilt_shadow, -2, 2)))
                        if step:
                            cmd["tilt"] = step
                            tilt_shadow = clamp(tilt_shadow + step, TILT_MIN, TILT_MAX)

            # Mood goes out on its own slow cadence: on change, or as a
            # keepalive while a target is visible. `mood` tracks what the pet
            # is actually showing, which is what the HUD should report.
            if (now - last_mood_send) >= MOOD_INTERVAL_S and (
                new_mood != mood or target is not None
            ):
                mood = new_mood
                cmd["mood"] = mood
                last_mood_send = now

            dz = t["deadzone"]
            cv2.rectangle(frame,
                          (int(center_x - dz), int(center_y - dz)),
                          (int(center_x + dz), int(center_y + dz)),
                          (80, 80, 80), 1)
            cv2.drawMarker(frame, (int(center_x), int(center_y)), (0, 0, 255),
                           markerType=cv2.MARKER_CROSS, markerSize=16)

            # Rate-limit movement. A mood change goes out immediately since it
            # costs the servos nothing.
            movement = {k: v for k, v in cmd.items() if k in ("pan", "tilt")}
            if movement and (now - last_send) < t["send_s"]:
                cmd = {k: v for k, v in cmd.items() if k == "mood"}
            elif movement:
                last_send = now

            if cmd:
                queue_cmd(cmd)

            link = "ctrl" if _control_ok else "NO CTRL"
            hud = (f"{_track_mode} pan~{int(pan_shadow)} tilt~{int(tilt_shadow)} "
                   f"{mood} [{link}]")
            cv2.putText(frame, hud, (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv2.putText(frame, f"{fps:4.1f} fps  age {age_ms:3.0f} ms", (5, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            if _track_mode == "color" and not _picked:
                cv2.putText(frame, "click the object to sample its colour",
                            (5, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 255), 1)

            cv2.imshow(VIDEO_WIN, frame)
            if mask is not None:
                cv2.imshow("mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("f"):
                _track_mode = "face"
                smooth_x = smooth_y = None
                try:
                    cv2.destroyWindow("mask")
                except cv2.error:
                    pass
            elif key == ord("c"):
                _track_mode = "color"
                smooth_x = smooth_y = None
    finally:
        _stop.set()
        _cmd_ready.set()  # let the sender thread notice and exit
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()