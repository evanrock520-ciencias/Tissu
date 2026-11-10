# Python API

**Tissu** is available from both this repo and PyPI.

## Supported Python Versions and Platforms

Tissu 1.1.0 provides headless wheels for CPython 3.10–3.14. You can install it using pip:

```bash
pip install pytissu
```

The uploaded wheels for version 1.1.0 have the following compatibility:

| Python versions | Platform | Minimum OS requirement |
|-----------------|----------|------------------------|
| 3.10–3.14       | Linux x86_64 | glibc 2.39 (`manylinux_2_39`) |
| 3.10–3.14       | Windows AMD64 | 64-bit Windows |
| 3.10–3.14       | macOS ARM64 | macOS 26.0 |

For platforms without a matching wheel, or for viewer support, build from source:

```bash
pip install -e .
```

> [!NOTE]
> To use the repo version of Tissu, see [Installation](../core/build_and_development.md).

## Content

- [Simulation](simulation.md)
