#!/bin/bash
# Builds the given RubyFPV binaries for the Radxa platform and packages them into a
# flat tar.gz update archive, ready to be copied onto a vehicle or controller and
# deployed manually (or dropped into FOLDER_UPDATES/bin/<board>/ for the official
# controller-driven "Update Vehicle" flow).
#
# Usage:
#   ./package_update_radxa.sh vehicle    # ruby_rt_vehicle, ruby_tx_telemetry, ruby_start
#   ./package_update_radxa.sh central    # ruby_central
#   ./package_update_radxa.sh all        # both of the above
#
# Requires a working Radxa (aarch64) cross build environment (drm, cairo, SDL2, i2c,
# gpiod dev libraries for the target arch) - this will NOT produce working binaries
# on a plain x86_64 dev machine without a proper cross toolchain. Best run directly
# on the Radxa board, or in the same environment used by make_radxa.sh.
#
# Deployment on the target device (no signature/checksum check is performed by
# RubyFPV, this is a straight file replacement):
#   scp ruby_update_<component>_<timestamp>.tar.gz radxa@<ip>:/home/radxa/ruby/
#   ssh radxa@<ip>
#   cd /home/radxa/ruby/
#   tar -xzf ruby_update_<component>_<timestamp>.tar.gz -C /tmp/ruby_update_staging
#   cp -f /tmp/ruby_update_staging/* /home/radxa/ruby/
#   chmod 777 /home/radxa/ruby/ruby_*
#   sudo reboot   # or manually restart the affected ruby_* processes

set -e

COMPONENT="${1:-all}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUT_DIR="releases"
STAGE_DIR=$(mktemp -d)

VEHICLE_TARGETS="ruby_rt_vehicle ruby_tx_telemetry ruby_start"
CENTRAL_TARGETS="ruby_central"

cleanup() {
   rm -rf "$STAGE_DIR"
}
trap cleanup EXIT

build_and_package() {
   local NAME="$1"
   shift
   local TARGETS="$@"

   echo "=== Building [$NAME]: $TARGETS ==="
   make $TARGETS RUBY_BUILD_ENV=radxa

   local PKG_STAGE="$STAGE_DIR/$NAME"
   mkdir -p "$PKG_STAGE"
   for T in $TARGETS; do
      if [ ! -f "$T" ]; then
         echo "ERROR: expected binary '$T' was not produced by the build." >&2
         exit 1
      fi
      cp -f "$T" "$PKG_STAGE/"
   done

   mkdir -p "$OUT_DIR"
   local OUT_FILE="$OUT_DIR/ruby_update_${NAME}_${TIMESTAMP}.tar.gz"
   tar -czf "$OUT_FILE" -C "$PKG_STAGE" .
   echo "Created $OUT_FILE"
   tar -tzf "$OUT_FILE"
}

case "$COMPONENT" in
   vehicle)
      build_and_package vehicle $VEHICLE_TARGETS
      ;;
   central)
      build_and_package central $CENTRAL_TARGETS
      ;;
   all)
      build_and_package vehicle $VEHICLE_TARGETS
      build_and_package central $CENTRAL_TARGETS
      ;;
   *)
      echo "Usage: $0 [vehicle|central|all]" >&2
      exit 1
      ;;
esac
