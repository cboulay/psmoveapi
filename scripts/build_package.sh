#!/bin/bash

set -e
set -x

BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../"
cd "$BASE"

pkg_tarball() {
    PLATFORM_PKG="tar czvf"
    PLATFORM_PKG_EXT=".tar.gz"
}

pkg_zipfile_7z() {
    PLATFORM_PKG="7z a"
    PLATFORM_PKG_EXT=".zip"
}

pkg_zipfile_zip() {
    PLATFORM_PKG="zip -r"
    PLATFORM_PKG_EXT=".zip"
}

if [ -z "$BUILD_TYPE" ]; then
    echo "\$BUILD_TYPE is not set, trying to auto-detect"
    UNAME=$(uname)
    case "$UNAME" in
        Darwin)
            BUILD_TYPE="macos-native-clang"
            ;;
        Linux)
            BUILD_TYPE="linux-native-clang"
            ;;
        *)
            echo "Could not auto-detect build type, please set \$BUILD_TYPE"
            exit 1
            ;;
    esac
fi

case "$BUILD_TYPE" in
    linux-native-clang)
        PLATFORM_BIN="
        build/psmove
        build/test_tracker
        "
        PLATFORM_LIB="
        build/libpsmoveapi.so
        build/libpsmoveapi_tracker.so
        "
        pkg_tarball

        PLATFORM_NAME="linux"
        bash -e -x scripts/linux/build-debian
        ;;
    linux-cross-mingw*)
        PLATFORM_BIN="
        build/psmove.exe
        build/test_tracker.exe
        "
        PLATFORM_LIB="
        build/libpsmoveapi.dll
        build/libpsmoveapi_tracker.dll
        "
        pkg_zipfile_zip

        case "$BUILD_TYPE" in
            linux-cross-mingw64)
                PLATFORM_NAME="mingw64"
                bash -e -x scripts/mingw64/cross-compile x86_64-w64-mingw32
                ;;
            linux-cross-mingw32)
                PLATFORM_NAME="mingw32"
                bash -e -x scripts/mingw64/cross-compile i686-w64-mingw32
                ;;
            *)
                echo "Invalid \$BUILD_TYPE: '$BUILD_TYPE'"
                exit 1
                ;;
        esac
        ;;
    macos-native-clang)
        PLATFORM_BIN="
        build/psmove
        build/test_tracker
        "
        PLATFORM_LIB="
        build/libpsmoveapi.dylib
        build/libpsmoveapi_tracker.dylib
        "
        pkg_tarball

        PLATFORM_NAME="macos"
        bash -e -x scripts/macos/build-macos
        ;;
    windows-native-msvc)
        PLATFORM_BIN="
        build/Release/psmove.exe
        build/Release/test_tracker.exe
        "
        PLATFORM_LIB="
        build/Release/psmoveapi.dll
        build/Release/psmoveapi_tracker.dll
        "
        pkg_zipfile_7z

        PLATFORM_NAME="win64-msvc2015"
        chmod +x ./scripts/visualc/build_msvc_2015.bat
        ./scripts/visualc/build_msvc_2015.bat
        ;;
    *)
        echo "Invalid/unknown \$BUILD_TYPE value: '$BUILD_TYPE'"
        exit 1
        ;;
esac

if [ ! -z "$PSMOVEAPI_CUSTOM_PLATFORM_NAME" ]; then
    PLATFORM_NAME="$PSMOVEAPI_CUSTOM_PLATFORM_NAME"
fi

# Git revision identifier
PSMOVEAPI_REVISION=$(git describe --tags)

DEST="psmoveapi-${PSMOVEAPI_REVISION}-${PLATFORM_NAME}"
mkdir -p "$DEST"

cp -v README.md COPYING "$DEST/"

mkdir -p "$DEST/include"
cp -v include/*.h build/psmove_config.h "$DEST/include/"

mkdir -p "$DEST/bindings/python"
cp -rv bindings/python/psmoveapi.py "$DEST/bindings/python/"

if [ ! -z "$PLATFORM_BIN" ]; then
    mkdir -p "$DEST/bin"
    cp -v $PLATFORM_BIN "$DEST/bin/"
fi

if [ ! -z "$PLATFORM_LIB" ]; then
    mkdir -p "$DEST/lib"
    cp -v $PLATFORM_LIB "$DEST/lib/"
fi

mkdir -p upload
$PLATFORM_PKG "upload/${DEST}${PLATFORM_PKG_EXT}" "$DEST"
rm -rf "$DEST"
