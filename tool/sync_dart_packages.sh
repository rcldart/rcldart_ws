#!/usr/bin/env bash
# sync_dart_packages.sh — one command to make custom interface packages in src/
# usable by every app in apps/.
#
#   1. colcon build  (src/<pkg> → install/<pkg>/share/**/*.idl)
#   2. gen_ros_dart_ws.py  (install → dart/<pkg> Dart packages, deps resolved)
#   3. sync_app_overrides.py  (inject path dependency_overrides into apps/*)
#
# After this, `flutter pub get` in an app picks up the new/updated message types
# with no manual pubspec editing.
#
# Usage: tool/sync_dart_packages.sh                 # build+gen all src pkgs, sync all apps
#        tool/sync_dart_packages.sh <pkg>...        # only these src packages
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
GEN="$(cd "$WS/.." && pwd)/rosidl_generator_dart/bin/gen_ros_dart_ws.py"
cd "$WS"

# Which interface packages to (re)generate: args, else everything in src/.
if [ "$#" -gt 0 ]; then
  PKGS=("$@")
else
  PKGS=()
  for d in src/*/; do [ -f "$d/package.xml" ] && PKGS+=("$(basename "$d")"); done
fi
[ "${#PKGS[@]}" -gt 0 ] || { echo "no interface packages in src/"; exit 1; }
echo "== packages: ${PKGS[*]} =="

echo "== 1. colcon build =="
if command -v colcon >/dev/null; then
  colcon build --packages-select "${PKGS[@]}" 2>&1 | tail -5
else
  echo "  colcon not found — assuming install/ is already built"
fi

echo "== 2. generate Dart packages (deps auto-resolved) =="
# Search the workspace's built packages (install/<pkg>/share) first, then the
# system distro — so custom src/ packages AND upstream deps both resolve.
DISTRO_SHARE="/opt/ros/${ROS_DISTRO:-jazzy}/share"
WS_SHARES="$(printf '%s:' "$WS"/install/*/share 2>/dev/null)"
export SHARE_ROOT="${WS_SHARES}${DISTRO_SHARE}"
python3 "$GEN" "$WS/dart" "${PKGS[@]}"

echo "== 3. inject dependency_overrides into apps/* =="
python3 "$WS/tool/sync_app_overrides.py"

echo "== done. In an app: flutter pub get =="
