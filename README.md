# FPV Sim (Unreal Engine 5)

An FPV drone simulator built in Unreal Engine 5: a 5" freestyle quad flown in acro, with its physics written from scratch
in C++. Plug in a real ELRS / EdgeTX radio over USB and fly a practice field, or real places anywhere on Earth streamed as
photoreal 3D (Google Photorealistic 3D Tiles via Cesium for Unreal).

## Features

* **Physics** (`Source/FpvSim/QuadSim.h`, plain C++ with no engine types):
  * 2 kHz fixed-step rigid body.
  * Motor rpm lag and rotor inertia, so throttle changes give a yaw kick.
  * Prop thrust drops with inflow (climb rate, and roll/pitch rate at each arm).
  * Quadratic body drag, rotor H-force and propwash shake.
  * 6S battery with voltage sag; you can turn on an infinite battery.
  * Spring-damper ground and crash contacts.
* **Flight controller:** Betaflight-style "Actual" rates, PID with D-term and feedforward, gyro filtering, an airmode
  mixer, arming checks and turtle mode (flip over after a crash).
* **Radio:** any EdgeTX / ELRS handset in USB joystick mode, read with DirectInput (8 channels, 32 buttons, no drivers).
  A guided calibration finds your sticks and switches. Arm, turtle, reset and camera switches are assignable.
* **Views:** FPV camera with adjustable uptilt and field of view, a chase camera and a line-of-sight camera. Modelled
  5" tri-blade props that blur when spinning. A Betaflight-style OSD.
* **Settings menu** (Esc): camera, weight, thrust, top speed, motor response, drag, propwash, crash speed, battery,
  rates, airmode and physics rate. Changes apply live and can be saved.
* **Worlds:** a procedural practice field with a gate track, trees, an open parking structure and a lattice tower.
  Real-world places (see below), including landmark spawn points for filming.
* **Filming:** press **O** for a clean view (no OSD; the map credits stay on screen). Air-start landmarks hold the quad
  in place until you arm.

## Requirements

* Windows 10/11, Unreal Engine 5.8, Visual Studio 2022 ("Game development with C++").
* For real-world flying: the [Cesium for Unreal](https://github.com/CesiumGS/cesium-unreal/releases) plugin for UE 5.8,
  unzipped into `Plugins/` (not included). You also need your own Cesium ion access token (below).

## Build and run

```powershell
powershell -ExecutionPolicy Bypass -File build.ps1
powershell -ExecutionPolicy Bypass -File run.ps1          # add -Fullscreen for full screen
```

The first launch compiles shaders and takes a few minutes. There is no editor content: the level, sky and quad are all
built in code.

## Radio

Plug in the handset over USB and choose **USB Joystick (HID)** on EdgeTX. In the sim, press **C** and follow the
calibration (Enter after each step). Arm with throttle low. No radio? Press **K** for keyboard flying (Space arm, W/S
throttle, A/D yaw, arrows pitch/roll).

## Keys

| Key | |
|---|---|
| Esc / M | settings menu (pauses the sim; Quit is in there) |
| C | radio calibration |
| R | reset to the start |
| T | turtle mode (if you have no turtle switch) |
| V | camera: FPV / chase / line of sight |
| O | clean recording view |
| H | stick overlay |
| K | keyboard flying |
| F5 | reload `fpv_settings.json` |

## Real-world flying

Real places come from Google Photorealistic 3D Tiles, streamed through Cesium ion. Access goes through your own Cesium
ion account and access token, which is not included here. Make a free account at [cesium.com/ion](https://ion.cesium.com),
add Google Photorealistic 3D Tiles to your assets, and put your token in a file named `cesium_ion_token.txt` next to
`FpvSim.uproject` (it is git-ignored). See [Cesium's documentation](https://cesium.com/learn/unreal/) for details. Then:
Esc, WORLD, turn on Real world, pick a Location (including the landmarks), and choose Go.

Add your own places in `fpv_settings.json` under `"places"`: `name`, `lat`, `lon`, and an optional `alt` (metres, for an
air start). Keep the on-screen Google / Cesium credits visible, including in videos; Google's terms require them.

## Physics check (no Unreal needed)

```powershell
powershell -ExecutionPolicy Bypass -File tools\run_harness.ps1          # add "tune" to re-run the PID gain search
```

| Check | Result |
|---|---|
| Hover | ~28 % throttle, thrust/weight 8.5 |
| Rate steps | 90 % of the commanded rate in ~30 ms, 6–8 % overshoot, at any throttle |
| Top speed | ~140 km/h level |
| Turtle | flips back over in ~0.3 s |

The airframe numbers are typical of a 5" 6S freestyle quad, not a measured build.

## Licence

MIT. See [LICENSE](LICENSE). Cesium for Unreal, Unreal Engine and the Google / Cesium map data have their own licences
and terms.
