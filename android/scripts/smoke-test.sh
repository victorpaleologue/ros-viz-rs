#!/usr/bin/env bash
# Smoke-test the debug APK on a running emulator: install, launch, and fail
# (printing the crash buffer) if the app crashes or isn't alive shortly after.
# Invoked as a single line from the emulator-runner step, which executes its
# `script:` input line by line — so all multi-line logic must live here.
set -uo pipefail

PKG=eu.palaio.rosvizrs
ACT=android.app.NativeActivity
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

# Grab a framebuffer screenshot so the rendered UI (the egui connection bar,
# which can't be seen on a headless CI run otherwise) is reviewable as an
# artifact. The activity gets stopped during the headless watch above (winit
# logs onStop/onDestroy), so bring it back to the foreground and let it render
# a few frames first, or we'd just photograph the launcher. Best-effort: never
# fail the smoke test over a missing screenshot.
adb shell am start -n "$PKG/$ACT" >/dev/null 2>&1 || true
shot="${GITHUB_WORKSPACE:-.}/app-screenshot.png"
best=0
for _ in $(seq 1 4); do
  # A tap generates input so the reactive (battery-saving) renderer draws a
  # fresh frame; the headless emulator tears the activity down a few seconds
  # after launch, so grab quickly and keep the largest (least-blank) frame.
  adb shell input tap 160 320 >/dev/null 2>&1 || true
  sleep 1
  tmp="$(mktemp)"
  if adb exec-out screencap -p > "$tmp" 2>/dev/null; then
    sz="$(wc -c < "$tmp")"
    if [ "$sz" -gt "$best" ]; then best="$sz"; mv "$tmp" "$shot"; else rm -f "$tmp"; fi
  else
    rm -f "$tmp"
  fi
done
if [ -s "$shot" ]; then
  echo "Captured screenshot -> $shot ($best bytes)"
  # Also emit it base64 between markers: CI artifacts live on a storage host
  # that egress policies often block, but job logs are always fetchable, so
  # this lets the rendered frame be reviewed straight from the log.
  echo "----BEGIN_SCREENSHOT_B64----"
  base64 -w0 "$shot" 2>/dev/null || base64 "$shot"
  echo
  echo "----END_SCREENSHOT_B64----"
else
  echo "::warning::Could not capture a screenshot"
  rm -f "$shot"
fi

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
