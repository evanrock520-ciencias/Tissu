from python.tissu import Simulation

def curtain():
    sim = Simulation(substeps=10, iterations=2, gravity=-9.81, thickness=0.005)
    sim.wind = [3.0, 0.0, 2.0]
    
    sim.add_floor()
    curtain = sim.create_grid(
        name="curtain",
        rows=50,
        cols=50,
        spacing=0.01,
        material="silk"
    )
    
    curtain.pin_top_corners()
    sim.view()
    
    
def pillow():
    sim = Simulation(substeps=40, iterations=2, gravity=-9.81, thickness=0.002)
    sim.wind = [0.0, 0.0, 0.0]
    
    sim.add_floor(height=-1.0, friction=1.0)
    sim.add_sphere(name="sphere", center=[0.0, 0.0, 0.0], radius=0.8)

    pillow = sim.create_from_obj(
        name="pillow",
        material="cotton",
        path="data/models/pillow.obj",
    )
    
    pillow.update_material(
        structural=1e-7,  
        bending=1e-5     
    )
    
    rest_vol = pillow.enable_volume_preservation(compliance=0.0)
    
    sim.view()

if __name__ == "__main__":
    match 0:
        case 0 : curtain()
        case 1 : pillow()