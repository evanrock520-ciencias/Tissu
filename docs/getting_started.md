# Getting started

This guide runs a small cloth simulation and exports an OBJ mesh. It works
without a display or the optional OpenGL viewer.

## Install

Use CPython 3.10–3.14 and check the [platform compatibility table](python/index.md)
for an available wheel. Create a virtual environment:

```bash
python -m venv .venv
```

Activate it with `source .venv/bin/activate` on Linux/macOS or
`.venv\Scripts\Activate.ps1` in PowerShell, then install the precompiled package:

```bash
python -m pip install --only-binary=:all: pytissu
```

The distribution is named **pytissu**; the Python import is **tissu**. If pip
cannot find a matching wheel, check the compatibility table or follow the
[source build guide](core/build_and_development.md).

## Run a simulation

Save the following as `quickstart.py`:

```python
from tissu import Simulation

sim = Simulation(substeps=5, iterations=2)
cloth = sim.create_grid(
    name="cloth",
    rows=8,
    cols=8,
    spacing=0.1,
    material="cotton",
)
cloth.pin_top_corners()

sim.simulate(frames=60)
sim.save_snapshot(path="cloth.obj", fabric_name="cloth")

print(f"Exported {sim.positions.shape[0]} particles to cloth.obj")
```

Run it in the activated environment:

```bash
python quickstart.py
```

The script writes `cloth.obj` in the current directory and reports 64 particles.
You can open the mesh in a 3D application or use it as input for another tool.

## Export an animation

To export an Alembic animation, replace the `simulate`, `save_snapshot` and
`print` calls with:

```python
sim.bake_alembic(path="cloth.abc", start_frame=0, end_frame=60, fps=60)
print("Exported cloth.abc")
```

This runs the simulation and writes `cloth.abc`. For parameters and the optional
viewer, see the [Simulation reference](python/simulation.md).
