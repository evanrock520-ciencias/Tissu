from python.tissu import Simulation

def pillow():
    sim = Simulation(substeps=40, iterations=2, gravity=-9.81, thickness=0.002)
    sim.wind = [0.0, 0.0, 0.0]
    
    sim.add_floor(height=-1.0, friction=1.0)

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
    
    sim.bake_alembic(
        filepath="data/animations/pillow.abc",
        start_frame=0,
        end_frame=120,
        fps=60
    )

if __name__ == "__main__":
    pillow()