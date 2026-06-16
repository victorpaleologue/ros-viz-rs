#!/usr/bin/env bash
# Smoke-test the debug APK on a running emulator: install, launch, and fail
# (printing the crash buffer) if the app crashes or isn't alive shortly after.
# Invoked as a single line from the emulator-runner step, which executes its
# `script:` input line by line — so all multi-line logic must live here.
set -uo pipefail

PKG=eu.palaio.rosvizrs
ACT=com.google.androidgamesdk.GameActivity
APK="${GITHUB_WORKSPACE:-.}/ros-viz-rs-debug.apk"

echo "Installing $APK"
adb install -r "$APK"
adb logcat -c
adb shell am start -n "$PKG/$ACT"

# Watch up to ~40s. A startup crash usually lands in the crash buffer; also
# watch the main buffer for native fatals / missing-library errors.
crashed=""
for _ in $(seq 1 20); do
  sleep 2
  if adb logcat -d -b crash | grep -qE "$PKG|libros_viz_rs|RustStdoutStderr"; then
    crashed="crash-buffer"
    break
  fi
  if adb logcat -d | grep -qE "FATAL EXCEPTION|Fatal signal|UnsatisfiedLinkError"; then
    crashed="main-buffer"
    break
  fi
done

pid="$(adb shell pidof "$PKG" | tr -d '\r')"

echo "==================== CRASH BUFFER ===================="
adb logcat -d -b crash | tail -n 150
echo "==================== APP / NATIVE LOG (filtered) ===================="
adb logcat -d \
  | grep -iE "$PKG|ros_viz|RustStdoutStderr|bevy|wgpu|winit|panicked|GameActivity|UnsatisfiedLink|Fatal signal|dlopen" \
  | tail -n 150
echo "====================================================================="

if [ -n "$crashed" ] || [ -z "$pid" ]; then
  echo "::error::App crashed at startup (crashed='$crashed', pid='$pid')"
  exit 1
fi
echo "Smoke test passed: $PKG is running (pid $pid)."
