#!/bin/bash
# Bundle target/release/ros-viz-rs into ros-viz-rs.app and a drag-to-install
# .dmg (custom background, Applications symlink, volume + app icon).
#
# Usage: scripts/package_macos.sh <version> [arch-label]
# Produces: ros-viz-rs-<version>-macos-<arch>.dmg in the working directory.
# Requires: create-dmg (brew install create-dmg), a built release binary.
set -euo pipefail

version="${1:?usage: package_macos.sh <version> [arch-label]}"
arch="${2:-$(uname -m | sed 's/x86_64/intel/; s/arm64/arm64/')}"
root="$(cd "$(dirname "$0")/.." && pwd)"
binary="$root/target/release/ros-viz-rs"
[ -x "$binary" ] || { echo "error: build first: cargo build --release" >&2; exit 1; }

staging="$(mktemp -d)"
trap 'rm -rf "$staging"' EXIT
app="$staging/ros-viz-rs.app"

mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"
cp "$binary" "$app/Contents/MacOS/ros-viz-rs"
cp "$root/assets/macos/AppIcon.icns" "$app/Contents/Resources/AppIcon.icns"
cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key><string>ros-viz-rs</string>
  <key>CFBundleIdentifier</key><string>eu.palaio.ros-viz-rs</string>
  <key>CFBundleName</key><string>ros-viz-rs</string>
  <key>CFBundleDisplayName</key><string>ros-viz-rs</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundleShortVersionString</key><string>$version</string>
  <key>CFBundleVersion</key><string>$version</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>LSMinimumSystemVersion</key><string>12.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
PLIST

dmg="ros-viz-rs-$version-macos-$arch.dmg"
rm -f "$dmg"
# The Finder-driven layout step occasionally times out (AppleEvent -1712,
# a known create-dmg flake); retry once before giving up.
attempt() {
create-dmg \
  --volname "ros-viz-rs" \
  --volicon "$root/assets/macos/AppIcon.icns" \
  --background "$root/assets/macos/dmg-background.tiff" \
  --window-pos 200 120 \
  --window-size 660 420 \
  --icon-size 128 \
  --icon "ros-viz-rs.app" 165 220 \
  --app-drop-link 495 220 \
  --no-internet-enable \
  "$dmg" "$staging"
}
attempt || { echo "create-dmg failed once; retrying…" >&2; rm -f rw.*."$dmg"; attempt; }

echo "built: $dmg"
