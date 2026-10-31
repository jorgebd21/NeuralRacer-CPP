# Simulator Media & Documentation Asset Capture Guide

This guide details the standard procedure for capturing high-fidelity screenshots, telemetry graphs, and animated GIFs for the **NeuralRacer-CPP** documentation and release notes.

---

## 1. Simulation Viewport & Resolution Configuration

Before capturing assets, ensure the simulation viewport and rendering parameters are set consistently:

* **Primary Simulation Canvas:** $2400 \times 1600\text{ px}$ (Internal coordinate space defined in `include/Config.h`).
* **Display Window Resolution:** $1280 \times 800\text{ px}$ (Default Raylib viewport window) or $1920 \times 1080\text{ px}$ for high-DPI displays.
* **Target Frame Rate:** 60 FPS (Fixed physics and rendering tick rate via `SetTargetFPS(60)`).

> [!TIP]
> For cleanest visual captures, maximize the simulation window or run in borderless fullscreen mode to avoid desktop environment window frame artifacts.

---

## 2. HUD & Overlay Keybindings for Clean Captures

Use the engine's built-in toggle shortcuts to customize the level of information visible in each capture:

| Shortcut | Description | Recommended Usage |
| :---: | :--- | :--- |
| `[ H ]` | Toggle Telemetry Dashboard & Heatmaps | Enable for telemetry showcase; disable for pure driving aesthetics. |
| `[ C ]` | Toggle Checkpoints & Gate Lines | Disable for clean racing captures; enable for algorithmic debugging. |
| `[ V ]` | Toggle Camera View (`CAM_FOLLOW_LEADER` vs `CAM_OVERVIEW`) | Use `CAM_FOLLOW_LEADER` for dynamic chase shots; `CAM_OVERVIEW` for full circuit captures. |
| `[ G ]` | Toggle Semi-Transparent Ghost Car | Enable in Replay and Test AI modes to display lap comparison lines. |
| `[ SPACE ]` | Toggle Speed Multiplier (1x / 50x) | Set to Normal (1x) during recording to maintain 60 FPS visual smoothness. |

---

## 3. High-Quality Animated GIF Generation (`docs/Training.gif`)

To create compact, artifact-free animated GIFs for the `README.md` hero section, follow the two-pass `ffmpeg` workflow using custom palette optimization:

### Step 1: Record 60 FPS Source Video
Record a 5–10 second driving sequence at 60 FPS using OBS Studio, GPU screen recorder, or `ffmpeg` X11 capture:
```bash
# Example recording with ffmpeg directly from X11 display:
ffmpeg -f x11grab -video_size 1280x800 -framerate 60 -i :0.0+0,0 -c:v libx264 -crf 18 -preset ultrafast raw_capture.mp4
```

### Step 2: Generate Optimized Color Palette
```bash
ffmpeg -y -i raw_capture.mp4 -vf "fps=30,scale=800:-1:flags=lanczos,palettegen=stats_mode=diff" palette.png
```

### Step 3: Render Final GIF with Palette Mapping
```bash
ffmpeg -y -i raw_capture.mp4 -i palette.png -filter_complex "fps=30,scale=800:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=3" docs/Training.gif
```

> [!NOTE]
> Keep the final GIF under 5 MB to guarantee rapid loading across all GitHub desktop and mobile web interfaces.

---

## 4. Static Mode Screenshots

Static screenshots should be captured in lossless PNG format and saved directly to `docs/`:

1. **Training Mode (`docs/Training.png`):** Capture during active generation (e.g., Gen 100+) with the population distributed, telemetry dashboard active, and neural network weights highlighted.
2. **Exhibition Mode (`docs/Exhibition.png`):** Capture side-by-side racing between the player vehicle (blue) and the elite neural brain (red).
3. **Test AI Mode (`docs/TestAI.png`):** Capture a single elite vehicle navigating a technical chicane or hairpin in full overview mode (`[ V ]`).

```bash
# Optimize PNG file size losslessly before committing:
optipng -o7 docs/*.png 2>/dev/null || true
```
