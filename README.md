# zabidou_pc_sync

This repository contains a C++ calibration sub-project under `calibration/` for intrinsic and extrinsic calibration between Lucid Phoenix RGB and Helios2 depth cameras.

## Dependencies

The calibration project uses CMake and expects:

- `cmake` 3.20+
- a C++17 compiler
- OpenCV
- Eigen3
- Lucid Arena SDK for live-camera support

Dataset-only builds do not require Arena SDK.

## Arena SDK Setup

Lucid's official getting-started guide says Arena SDK must be downloaded from Lucid, and that downloading it requires a Lucid account:

- Getting started: <https://support.thinklucid.com/getting-started/>
- Arena SDK docs landing page: <https://support.thinklucid.com/documentation/welcome-to-arena-sdk-documentation/>

Recommended setup flow on Linux:

1. Create or sign in to your Lucid account.
2. Download the Linux Arena SDK package from Lucid.
3. Extract or install it to a stable location, for example `~/ArenaSDK_Linux_x64` or `/opt/lucid/ArenaSDK`.
4. Export `ARENA_ROOT` to that install directory before running CMake.

Example:

```bash
export ARENA_ROOT=$HOME/ArenaSDK_Linux_x64
```

Quick verification:

```bash
find "$ARENA_ROOT" -name ArenaApi.h -o -name 'libArena*.so'
```

If that command finds the header and shared libraries, the custom `FindArenaSDK.cmake` should have a reasonable chance of working.

## Build

### Dataset-only build

Use this when you want offline calibration from saved `rgb_*.png` and `depth_*.png` pairs and do not need Lucid hardware access:

```bash
cmake -S calibration -B calibration/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ARENA_SDK=OFF

cmake --build calibration/build -j
```

### Live-camera build

Use this when you are preparing for Phoenix + Helios2 live capture through Arena SDK:

```bash
export ARENA_ROOT=$HOME/ArenaSDK_Linux_x64

cmake -S calibration -B calibration/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ARENA_SDK=ON

cmake --build calibration/build -j
```

Optional build flags:

- `-DBUILD_APPS=ON` to build runnable apps
- `-DBUILD_TESTS=ON` to build tests
- `-DBUILD_PYTHON_BINDINGS=ON` to enable the Python placeholder target
- `-DBUILD_ROS2_NODE=ON` to enable the ROS2 placeholder target

## Run

The main executable is:

```bash
./calibration/build/apps/run_calibration --help
```

Offline dataset calibration:

```bash
./calibration/build/apps/run_calibration \
  --backend opencv \
  --dataset /path/to/dataset \
  --rows 9 \
  --cols 6 \
  --square-size 0.025 \
  --out calibration_result.yaml
```

Live camera calibration:

```bash
./calibration/build/apps/run_calibration \
  --backend opencv \
  --rows 9 \
  --cols 6 \
  --square-size 0.025 \
  --max-frames 50 \
  --out calibration_result.yaml
```

Current CLI flags:

- `--backend opencv|rgbd`
- `--dataset PATH`
- `--rows N`
- `--cols N`
- `--square-size METERS`
- `--max-frames N`
- `--out PATH`
- `--preview`

## Test

Build with tests enabled, then run:

```bash
ctest --test-dir calibration/build -V
```

The current test target is a smoke test for the OpenCV calibration path.

## Current Notes

- `opencv` is the working backend today.
- `rgbd` is a stub and currently throws `not implemented`.
- `--preview` is accepted but is not wired to visualization yet.
- `DatasetFrameSource` currently expects paired files in one directory using names like `rgb_*.png` and `depth_*.png`.
- If Arena SDK is not found, CMake now allows a dataset-only build to continue.
- `Helios2` and `Phoenix` are currently placeholder classes, so the project does not link Arena SDK yet even if it is detected.
- If you previously configured the build while Arena link dependencies were enabled, reconfigure from a clean build directory to flush cached link settings:

```bash
rm -rf calibration/build
cmake -S calibration -B calibration/build -DCMAKE_BUILD_TYPE=Release
cmake --build calibration/build -j
```
