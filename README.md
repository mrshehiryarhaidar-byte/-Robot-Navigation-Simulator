# 🔥 Naruto Jutsu Hand Sign Recognition System

> *"I am going to become Hokage!"* — detect jutsu gestures with your webcam using real-time computer vision.

```
╔══════════════════════════════════════════════════════════════╗
║        NARUTO JUTSU HAND SIGN RECOGNITION SYSTEM            ║
║        OpenCV  ·  MediaPipe  ·  PyGame  ·  NumPy           ║
╚══════════════════════════════════════════════════════════════╝
```

---

## ⚡ Quick Start

### 1 — Install Dependencies

```bash
pip install opencv-python mediapipe pygame numpy
```

> **Python 3.9 – 3.11** recommended (MediaPipe compatibility).  
> On Apple Silicon use `pip install mediapipe-silicon` if the standard build fails.

### 2 — Run

```bash
python naruto_jutsu_system.py
```

A webcam window opens instantly. Make a gesture, hold it for ~8 frames, and the jutsu fires. Press **Q** or **Esc** to quit.

---

## 🥷 Gesture → Jutsu Map

| Gesture | Jutsu | Visual Effect |
|---------|-------|---------------|
| ✊ **Fist** | Fire Style: Fireball Jutsu | Red/orange glow + embers |
| ✌️ **Peace / Two Fingers** | Shadow Clone Jutsu | Ghost silhouettes slide out |
| 🖐 **Open Palm** | Wind Style: Great Breakthrough | Spinning ellipse rings |
| 👍 **Thumb Up** | Lightning Style: Chidori | Electric bolt spray + flash |
| 👌 **OK Sign** | Water Style: Water Dragon | Ripple rings + spiral droplets |

---

## 🎵 Custom Sound Effects

Drop `.wav` or `.mp3` files into a `sounds/` folder next to the script:

```
sounds/
  fire.wav
  shadow.wav
  wind.wav
  lightning.wav
  water.wav
```

The engine loads them automatically. If a file is missing, a synthesised sound is generated in Python so the app always has audio.

**Good free sources:**
- [freesound.org](https://freesound.org) — search "fire whoosh", "thunder crack", "water splash"
- [zapsplat.com](https://zapsplat.com)
- The Naruto OST SFX packs on YouTube (download + convert with `ffmpeg`)

---

## ⚙️ Configuration

Edit the `CONFIG` dict at the top of the script:

| Key | Default | Description |
|-----|---------|-------------|
| `camera_index` | `0` | Webcam index (try `1` for external cameras) |
| `cooldown_seconds` | `2.0` | Seconds between consecutive jutsu triggers |
| `detection_hold_frames` | `8` | Frames a pose must be held before firing |
| `show_landmarks` | `True` | Toggle MediaPipe skeleton overlay |
| `show_fps` | `True` | FPS counter in top-right corner |
| `effect_duration` | `1.8` | How long each visual effect lasts (seconds) |

---

## 🏗️ Architecture

```
NarutoJutsuApp
├── GestureRecogniser   — MediaPipe Hands wrapper + finger-state logic
├── EffectRenderer      — OpenCV per-jutsu animations (fire, wind, lightning…)
├── SoundEngine         — PyGame mixer, file loader, NumPy synthesiser fallback
└── HUD                 — Scanline overlay, jutsu name flash, cooldown bar, FPS
```

### Gesture Recognition Pipeline

```
Webcam frame
  └─► MediaPipe (21 landmarks per hand)
        └─► finger_states()  → [thumb, index, middle, ring, pinky] booleans
              └─► classify() → jutsu key
                    └─► hold N frames → trigger!
```

---

## 🛠️ Troubleshooting

| Problem | Fix |
|---------|-----|
| `Cannot open webcam` | Change `camera_index` to `1` or `2` |
| Gestures misfire | Ensure good lighting; keep hand 30–70 cm from camera |
| No sound | `pip install pygame`; some Linux distros need `sudo apt install libasound2-dev` |
| Very low FPS | Lower camera resolution in `cap.set()` calls (e.g. 640×480) |
| MediaPipe install fails | Try `pip install mediapipe==0.10.9` or use Python 3.10 |

---

## 🌿 Adding Custom Jutsus

1. Add a new entry to `JUTSU_META` and `GESTURE_GUIDE`
2. Implement recognition logic in `GestureRecogniser.classify()`
3. Add a `_draw_<name>()` method to `EffectRenderer`
4. Add synthesis in `SoundEngine` and a `_load_or_synth()` call

---

*Built with 💙 using Python + OpenCV + MediaPipe + PyGame*
