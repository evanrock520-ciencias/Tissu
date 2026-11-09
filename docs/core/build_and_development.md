# Build and Development

Tissu uses CMake presets for local builds and CI. Each preset has its own build
folder under `build/`, so Release, Debug and viewer builds can coexist.

## Prerequisites

- **CMake >= 3.29** and **Ninja**.
- **Python >= 3.10**, including development headers for native extensions.
- A **C++17 compiler**: GCC, Clang, Apple Clang or MSVC.
- **OpenMP**: included with GCC and MSVC; install `libomp` for Apple Clang.
- **Git** to download CMake's third-party dependencies.
- **ccache** for `linux-release` and `macos-release`.
- **mold** for `linux-release` (Linux only).

The portable `headless-release` preset uses the system linker and does not
require ccache. CI uses CMake 4.3.0, Ninja 1.13.2 and Python 3.12.

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build python3-dev python3-venv git ccache mold
```

Check `cmake --version`: if your distribution provides an older version, install
CMake >= 3.29 before configuring.

### Fedora

```bash
sudo dnf install -y gcc gcc-c++ cmake ninja-build python3-devel python3-pip git ccache mold
```

### macOS

```bash
xcode-select --install
brew install cmake ninja python git libomp ccache
```

### Windows

Install Visual Studio 2022 with the **Desktop development with C++** workload,
CMake, Ninja, Python and Git. Run the commands below from an **x64 Native Tools
Command Prompt** or Developer PowerShell so Ninja can find `cl.exe` and the
Windows SDK. CI configures this environment with `ilammy/msvc-dev-cmd`.

## Build with presets

```bash
git clone https://github.com/evanrock520-ciencias/Tissu.git
cd Tissu
python -m venv .venv
```

Activate the environment (`source .venv/bin/activate` on Linux/macOS,
`.venv\Scripts\Activate.ps1` in PowerShell), then install the development tools:

```bash
python -m pip install --upgrade pip
python -m pip install mypy pytest jinja2
cmake --list-presets
```

Choose the preset for your platform:

| Preset | Configuration | Compiler cache | Linker |
| --- | --- | --- | --- |
| `linux-release` | Release, headless | ccache | mold |
| `macos-release` | Release, headless | ccache | Apple default |
| `windows-release` | Release, headless, MSVC | None | MSVC default |
| `headless-release` | Release, headless, portable | None | System default |
| `viewer-release` | Release with OpenGL viewer | None | System default |
| `debug` | Debug, headless | None | System default |
| `benchmarks` | Release, benchmarks only, no tests | None | System default |

For example, on Linux:

```bash
cmake --preset linux-release -DPython_EXECUTABLE="$(python -c 'import sys; print(sys.executable)')"
cmake --build --preset linux-release --parallel 2
python -m pip install -e '.[dev]'
ctest --preset linux-release
python -m pytest tests/python
```

On macOS, substitute `macos-release`. In PowerShell use:

```powershell
cmake --preset windows-release "-DPython_EXECUTABLE=$((Get-Command python).Source)"
cmake --build --preset windows-release --parallel 2
python -m pip install -e '.[dev]'
ctest --preset windows-release
python -m pytest tests/python
```

Selecting the interpreter explicitly ensures that CMake builds the extension
for the same Python environment that runs the tests. The build copies the
extension into `python/tissu/` and `blender/libs/`; build one Python ABI at a time
in a checkout.

### Optional components

Presets disable Tracy, benchmarks and examples unless explicitly requested.
Override cache variables when configuring:

```bash
cmake --preset linux-release -DTISSU_BUILD_EXAMPLES=ON -DTISSU_BUILD_BENCHMARKS=ON
cmake --build --preset linux-release --parallel 2
```

Available switches:

- `BUILD_TESTING`: build and register Tissu's C++ tests.
- `TISSU_BUILD_BENCHMARKS`: download Google Benchmark and build `tissu_benchmarks`.
- `TISSU_BUILD_EXAMPLES`: build the C++ examples.
- `TISSU_BUILD_VIEWER`: download and build the OpenGL viewer dependencies.
- `TISSU_ENABLE_TRACY`: enable profiling instrumentation.
- `TISSU_USE_CCACHE`: require ccache and use it as the compiler launcher.
- `CMAKE_LINKER_TYPE`: select a linker, e.g. `MOLD` on Linux or `DEFAULT`.

Viewer builds also require OpenGL development libraries and GLFW's platform
dependencies. On Ubuntu install these before using `viewer-release`:

```bash
sudo apt-get install -y libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

Use an ignored `CMakeUserPresets.json` for personal compiler paths and local
presets. Do not edit the shared presets to accommodate a single workstation.

### Configure, build and test with one command

Each development preset has a matching workflow preset:

```bash
cmake --workflow --preset linux-release
```

This runs CMake configuration, compilation and the C++ tests in sequence.
Choose `macos-release` or `windows-release` on those platforms. Activate your
Python environment before the first configuration. To select an interpreter
explicitly, configure once with `-DPython_EXECUTABLE=...` as shown above; the
workflow reuses that selection in the preset's build directory.

Run the Python tests separately in the same environment:

```bash
uv run --no-project python -m pytest tests/python
```

Set `CMAKE_BUILD_PARALLEL_LEVEL` to limit compilation concurrency, e.g.
`CMAKE_BUILD_PARALLEL_LEVEL=2 cmake --workflow --preset linux-release` on Unix.
The `benchmarks` workflow preset configures and builds its executable without
running tests or benchmarks automatically.

### Benchmarks

```bash
cmake --preset benchmarks
cmake --build --preset benchmarks --parallel 2
./build/benchmarks/bin/tissu_benchmarks --benchmark_out=build/benchmarks/results.json --benchmark_out_format=json
```

On Windows the executable has the `.exe` suffix. For Linux compiler caching and
mold, append `-DTISSU_USE_CCACHE=ON -DCMAKE_LINKER_TYPE=MOLD` when configuring.
The manual **Benchmarks** workflow uploads JSON results; it does not generate
plots or enforce performance thresholds.

## Tests and CI

The **CI** workflow runs on pushes to `main`, pull requests targeting `main`,
and manual dispatch. Feature-branch pushes are checked through their pull
request, so an update to a PR runs CI once. CI runs actionlint, Ruff and
clang-format once, builds and tests Release on Linux, macOS and Windows, and
checks that the benchmarks compile on Linux.
`astral-sh/setup-uv` installs uv 0.13.7, selects Python 3.12 and activates a
`.venv` shared by dependency installation, CMake and the Python tests. The action
persists uv's download cache using `pyproject.toml` and the build configuration
as cache inputs. Dependencies are installed with `uv pip install '.[dev]'`;
tests run with `uv run --no-project python -m pytest tests/python` in that
environment. The release workflow also uses uv, with its Python version matrix.

`lukka/get-cmake` installs CMake and Ninja, `threeal/cmake-action` configures and
builds using the shared presets, and CTest executes the corresponding test
preset. `rui314/setup-mold` installs mold on Linux with `make-default: false`;
the Linux preset explicitly selects it through `CMAKE_LINKER_TYPE=MOLD`.
`hendrikmuhs/ccache-action` installs and persists ccache on Unix.
Compiler caches are separated by configuration, operating system, architecture
and Python version. Generated FetchContent build trees are not restored across
runners.

CTest presets run sequentially because some tests share temporary file names.
They set `OMP_NUM_THREADS=2` to limit OpenMP overhead and fail when no tests are
registered. CI uploads C++ and Python test reports and cancels superseded runs.
Actions use explicit version tags or commit SHAs; Dependabot proposes grouped
weekly updates.

### Code style

```bash
python -m pip install prek
prek run --all-files
```

The hook versions are defined in `.pre-commit-config.yaml` and used by the
`j178/prek-action` CI job.

### Releases

The release workflow reuses the native CI setup and platform presets for each
Python version, with `BUILD_TESTING=OFF` for packaging. It builds the extension
before creating wheels, repairs native dependencies, and installs each repaired
wheel in an isolated uv environment to run a small simulation. The source
distribution is built and uploaded once on Linux with Python 3.12.

Manual dispatch builds and checks distributions without publishing them. A
published GitHub release triggers the existing PyPI publishing job.

### Docker

The Dockerfile uses Python 3.12 on Debian Bookworm, uv 0.13.7 and CMake 4.3.0.
It configures `headless-release` with `BUILD_TESTING=OFF`, builds the native
extension, and installs the Python package using uv. Host binaries and local
user presets are excluded from the Docker context.

```bash
docker build -t tissu:local .
docker run --rm tissu:local python -c "from tissu import Simulation; print(Simulation())"
```

Build and test orchestration is provided by CMake presets, CTest and pytest;
the former Python build/test helper scripts have been removed.
`scripts/patch_alembic.py` remains a required dependency patch used by CMake.
