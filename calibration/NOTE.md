# ArenaSDK Setup — What Went Wrong & How to Fix It

## What the SDK installer did wrong

Running `sudo sh Arena_SDK_Linux_x64_AVMP.conf` registered **all five** ArenaSDK
library directories in `/etc/ld.so.conf.d/Arena_SDK.conf`:

```
/opt/lucid/ArenaSDK/lib64
/opt/lucid/ArenaSDK/GenICam/library/lib/Linux64_x64
/opt/lucid/ArenaSDK/ffmpeg
/opt/lucid/ArenaSDK/Metavision/lib      ← problem
/opt/lucid/ArenaSDK/OpenCV/lib          ← problem
```

`Metavision/lib` and `OpenCV/lib` contain hundreds of bundled third-party
libraries (GDCM, GDAL deps, libxerces, libpq, etc.) built against old ABI
versions (ICU 70, OpenLDAP 2.5, libtiff 5). On Ubuntu 24.04 these old versions
do not exist. Registering those paths globally caused the system's own
`libgdal.so.34` to pick up Metavision's GDCM 3.0 libs *instead* of the system
package, which then pulled in the broken ICU/tiff/ldap chain at runtime.

Additionally, the Metavision OpenCV files were installed with `rwx------`
(root-only) permissions, so the dynamic linker could not open them when running
as a normal user.

---

## Full fix, step by step

### 1. Fix permissions on the Metavision OpenCV stubs

```bash
sudo chmod a+rx \
  /opt/lucid/ArenaSDK/Metavision/lib/libopencv_core.so \
  /opt/lucid/ArenaSDK/Metavision/lib/libopencv_highgui.so \
  /opt/lucid/ArenaSDK/Metavision/lib/libopencv_imgcodecs.so \
  /opt/lucid/ArenaSDK/Metavision/lib/libopencv_imgproc.so \
  /opt/lucid/ArenaSDK/Metavision/lib/libopencv_videoio.so
```

### 2. Install patchelf

```bash
sudo apt install patchelf
```

### 3. Remove Metavision NEEDED entries from libarena.so

`libarena.so` and `libgentl.so` both have hard `DT_NEEDED` entries for
`libmetavision_sdk_core.so.4` and `libmetavision_sdk_base.so.4`. These event-
camera libs have a broken transitive dependency chain on this OS. Since only
Lucid RGB/depth cameras (Phoenix, Helios) are used, those entries can be safely
removed.

```bash
# Backup originals first
sudo cp /opt/lucid/ArenaSDK/lib64/libarena.so  /opt/lucid/ArenaSDK/lib64/libarena.so.bak
sudo cp /opt/lucid/ArenaSDK/lib64/libgentl.so  /opt/lucid/ArenaSDK/lib64/libgentl.so.bak

# Strip the Metavision NEEDED entries
sudo patchelf --remove-needed libmetavision_sdk_core.so.4 /opt/lucid/ArenaSDK/lib64/libarena.so
sudo patchelf --remove-needed libmetavision_sdk_base.so.4 /opt/lucid/ArenaSDK/lib64/libarena.so
sudo patchelf --remove-needed libmetavision_sdk_core.so.4 /opt/lucid/ArenaSDK/lib64/libgentl.so
sudo patchelf --remove-needed libmetavision_sdk_base.so.4 /opt/lucid/ArenaSDK/lib64/libgentl.so
```

Both files are symlinks to their `*.so.1.0.0` counterparts — patchelf follows
the symlink and patches the real file, so all versioned aliases are fixed in one
step.

### 4. Restrict ldconfig to the two safe Arena lib directories

Overwrite the conf to include only the dirs that do not contain conflicting
third-party libraries:

```bash
sudo tee /etc/ld.so.conf.d/Arena_SDK.conf << 'EOF'
/opt/lucid/ArenaSDK/lib64
/opt/lucid/ArenaSDK/GenICam/library/lib/Linux64_x64
EOF
sudo ldconfig
```

### 5. Fix the CMake finder and project build files

Three bugs existed in `cmake/FindArenaSDK.cmake` and `CMakeLists.txt`:

| Bug | Effect | Fix |
|-----|--------|-----|
| OpenCV version check required exactly `4.0.x` | `ArenaSDK_FOUND=FALSE` with system OpenCV 4.6 | Removed the check entirely |
| Metavision ABI checks (ICU 70, tiff 5, ldap 2.5) set `ArenaSDK_FOUND=FALSE` | No Arena support in any build | Made Metavision optional; only sets `ArenaSDK_HAS_METAVISION` |
| No RPATH on the imported target | Executables couldn't find Arena .so files at runtime | Added `INTERFACE_LINK_OPTIONS` with `-Wl,-rpath` for `lib64` and GenICam paths |
| `libarena.so` transitive Metavision deps cause link-time ABI errors | Executables fail to link | Added `-Wl,--allow-shlib-undefined` as a PUBLIC link option on the `calibration` target |

### 6. Rebuild the project

The build directory generated before the CMake fixes is stale — reconfigure it:

```bash
cd /path/to/calibration
cmake -B build -DENABLE_ARENA_SDK=ON
cmake --build build
```

---

## Verification

```bash
# No "not found" entries should appear
ldd build/apps/camera_test | grep "not found"

# Should show libarena.so.1 and GenICam libs with no Metavision entries
patchelf --print-needed /opt/lucid/ArenaSDK/lib64/libarena.so
patchelf --print-needed /opt/lucid/ArenaSDK/lib64/libgentl.so
```

---

## Notes on Metavision (event cameras)

The Metavision SDK bundled with this ArenaSDK version requires ICU 70,
OpenLDAP 2.5, and libtiff 5 — none of which ship with Ubuntu 24.04. Metavision
event-camera support is therefore non-functional on this machine. The
`CALIBRATION_HAS_ARENA_SDK` flag is still set to `1`; only the
`ArenaSDK_HAS_METAVISION` flag is `FALSE`. Phoenix and Helios cameras are fully
supported.

If a future ArenaSDK release fixes the Metavision packaging, restore the
original libraries from the `.bak` files before applying the updated SDK.
