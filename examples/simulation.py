from python.tissu import Simulation, Material

def falling():
    sim = Simulation(substeps=15, iterations=2, gravity=-9.81, thickness=0.05)
    sim.wind = [0.0, 0.0, 0.0]
    sim.add_floor(height=0.0, friction=0.5)

    curtain = sim.create_grid("Curtain", rows=80, cols=80, spacing=0.05, material="silk")
    curtain.pin_top_corners()

    sim.view()

if __name__ == "__main__":
    falling()