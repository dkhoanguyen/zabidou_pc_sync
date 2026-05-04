# zabidou_pc_sync

This repository contains a C++ calibration sub-project under `calibration/` for intrinsic and extrinsic calibration between Lucid Phoenix RGB and Helios2 depth cameras, plus Python utilities under `pc_generation/` for reconstructing and visualizing colored point clouds from the calibration outputs.

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

Calibration outputs should be written under `calibration/results/`. The app-based mono and stereo calibration tools already default to that location, for example `calibration/results/combined_all_datasets/`.

Offline dataset calibration:

```bash
./calibration/build/apps/run_calibration \
  --backend opencv \
  --dataset /path/to/dataset \
  --rows 9 \
  --cols 6 \
  --square-size 0.025 \
  --out calibration/results/calibration_result.yaml
```

Live camera calibration:

```bash
./calibration/build/apps/run_calibration \
  --backend opencv \
  --rows 9 \
  --cols 6 \
  --square-size 0.025 \
  --max-frames 50 \
  --out calibration/results/calibration_result.yaml
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

## Calibration Workflow Notes

For the current app-based workflow in `calibration/apps/`:

- Mono RGB calibration writes `mono_rgb_calibration.yaml` under `calibration/results/...`.
- Mono depth calibration writes `mono_depth_calibration.yaml` under `calibration/results/...`.
- Stereo calibration writes `stereo_calibration.yaml` under `calibration/results/...`.
- Stereo calibration uses the manually calibrated RGB intrinsics from `mono_rgb_calibration.yaml`.
- Stereo calibration uses the Helios ToF factory intrinsics read from the device for the depth camera intrinsics.

Typical app-based commands are:

```bash
./calibration/build/apps/mono_calibration_app \
  --combine-all-datasets \
  --modality rgb

./calibration/build/apps/mono_calibration_app \
  --combine-all-datasets \
  --modality depth

./calibration/build/apps/stereo_calibration_app \
  --combine-all-datasets
```

That produces a result set like:

- `calibration/results/combined_all_datasets/mono_rgb_calibration.yaml`
- `calibration/results/combined_all_datasets/mono_depth_calibration.yaml`
- `calibration/results/combined_all_datasets/stereo_calibration.yaml`

## Point Cloud Reconstruction

The `pc_generation/` utilities consume the stereo calibration result, typically:

`calibration/results/combined_all_datasets/stereo_calibration.yaml`

Useful scripts for reconstructing or viewing a colored Helios point cloud are:

- `pc_generation/colorize_helios_point_cloud.py`: captures one RGB + Helios pair, colorizes the Helios point cloud, and saves a `.ply`.
- `pc_generation/show_stereo_point_cloud_open3d.py`: captures one pair or streams continuously and shows the colored point cloud in Open3D.
- `pc_generation/view_colored_pointcloud.py`: captures one pair and shows the colored point cloud with Matplotlib.
- `pc_generation/show_white_plane_point_cloud_open3d.py`: Open3D viewer variant for white-plane inspection/debugging.

Example commands from the repo root:

```bash
python3 pc_generation/colorize_helios_point_cloud.py \
  --stereo-yaml calibration/results/combined_all_datasets/stereo_calibration.yaml \
  --output calibration/results/combined_all_datasets/helios_colored_point_cloud.ply \
  --once
```

```bash
python3 pc_generation/show_stereo_point_cloud_open3d.py \
  --stereo-yaml calibration/results/combined_all_datasets/stereo_calibration.yaml \
  --preview
```

```bash
python3 pc_generation/view_colored_pointcloud.py \
  --stereo-yaml calibration/results/combined_all_datasets/stereo_calibration.yaml
```

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

## Building Sfm on Jetson and running example dataset
The instructions for cloning, building and running nvblox with a jetson can be found here
https://github.com/valtsblukis/nvblox
