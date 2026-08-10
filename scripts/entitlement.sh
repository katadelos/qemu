#!/bin/sh -e
#
# Helper script for the build process to apply entitlements

in_place=:
if [ "$1" = --install ]; then
  shift
  in_place=false
fi

DST="$1"
SRC="$2"
ICON="$3"
ENTITLEMENT="$4"

if $in_place; then
  trap 'rm "$DST.tmp"' exit
  cp -pPf "$SRC" "$DST.tmp"
  SRC="$DST.tmp"
else
  cd "$MESON_INSTALL_DESTDIR_PREFIX"
fi

if test -n "$ENTITLEMENT"; then
  # Current macOS code signing rejects resource forks and Finder metadata on
  # signed command-line executables.  Adding the legacy QEMU icon after
  # signing also makes AMFI kill Cocoa QEMU at launch, so omit that cosmetic
  # metadata for entitled binaries.
  xattr -c "$SRC"
  codesign --entitlements "$ENTITLEMENT" --force -s - "$SRC"
else
  # Add the QEMU icon to unsigned binaries on older macOS configurations.
  Rez -append "$ICON" -o "$SRC"
  SetFile -a C "$SRC"
fi

mv -f "$SRC" "$DST"
trap '' exit
