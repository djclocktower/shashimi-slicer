# Building Shashimi Slicer

Shashimi Slicer builds exactly like upstream OrcaSlicer: first the dependencies (`deps/`), then the
application. The CAD workspace is on by default (`SLIC3R_CAD=ON` in both `deps/CMakeLists.txt` and
the top-level `CMakeLists.txt`; the two must match). With it on, the dependency build also builds
the SolveSpace solver and OCCT's `ModelingAlgorithms` module, so a deps tree built by an upstream
OrcaSlicer checkout with `SLIC3R_CAD=OFF` cannot be reused.

The CMake target is still called `OrcaSlicer`; only the produced binary is renamed.

## Windows

Prerequisites: Visual Studio 2022 (or 2019 / 2026) with the "Desktop development with C++"
workload (MSVC x64 tools, Windows 11 SDK, C++ CMake tools), CMake, Git and Strawberry Perl
(needed by the OpenSSL dependency build). `build_win.bat -u` installs CMake, Perl and Git with
WinGet; add `--install-vs buildtools` (or `ide`) to install Visual Studio too.

From a regular command prompt in the repository root:

```bat
build_win.bat -d        :: dependencies, once (into deps\build)
build_win.bat -s        :: the application (into build)
build_win.bat -s -i     :: optional: also install a runnable tree into build\OrcaSlicer
```

`build_win.bat -h` lists every option (`--config debug`, `--arch arm64`, `-x` for Ninja, `-l` for
clang-cl, `--tests`, `-j N`, ...). The executable lands in `build\src\Release\ShashimiSlicer.exe`,
or `build\OrcaSlicer\ShashimiSlicer.exe` with `-i`. The summary printed at the end names the exact
path and the Visual Studio solution (`build\ShashimiSlicer.sln`).

## macOS

Prerequisites: Xcode (command line tools), CMake, Git, and `brew install libtool` (what CI
installs).

```sh
./build_release_macos.sh -d -a arm64     # dependencies (x86_64 or universal also accepted)
./build_release_macos.sh -s -a arm64     # the application
```

Add `-x` to use Ninja instead of Xcode and `-h` for all options. The app bundle is
`build/<arch>/OrcaSlicer/OrcaSlicer.app` (shown as "Shashimi Slicer").

## Linux

```sh
./build_linux.sh -u      # install system packages (asks for sudo)
./build_linux.sh -dsi    # dependencies, application and AppImage
```

`-d`/`-s`/`-i` can be run separately; `-t` builds the tests, `-g` runs the build in an Ubuntu 24.04
container, `-h` lists the rest. The binary is `build/src/Release/shashimi-slicer`; the runnable
package is `build/package/shashimi-slicer`, and `-i` writes `build/ShashimiSlicer_Linux_V<version>.AppImage`.

## Networks that block GitHub archive downloads

Some networks allow `git clone` from github.com but block its source-archive and release-asset
downloads. The dependency build can fetch those sources with git instead:
`-DORCA_DEPS_GITHUB_VIA_GIT=ON` (off by default; see the comment at the option in
`deps/CMakeLists.txt`).

```sh
DEPS_EXTRA_BUILD_ARGS="-DORCA_DEPS_GITHUB_VIA_GIT=ON" ./build_linux.sh -d   # Linux
```

```bat
build_win.bat -d --deps-args "-DORCA_DEPS_GITHUB_VIA_GIT=ON"                 :: Windows
```

`build_release_macos.sh` has no deps-argument option; configure the deps tree once with
`cmake -S deps -B deps/build/<arch> -DORCA_DEPS_GITHUB_VIA_GIT=ON` before running it. The value
stays in that tree's CMake cache.
