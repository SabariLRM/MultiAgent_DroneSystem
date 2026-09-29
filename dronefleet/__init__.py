"""Multi-agent drone fleet coordination for package delivery.

Public entry points::

    from dronefleet import SimConfig, Simulation
    sim = Simulation(SimConfig(n_drones=10, seed=3))
    metrics = sim.run()
"""

from .config import SimConfig
from .simulation import Simulation

__all__ = ["SimConfig", "Simulation"]
__version__ = "1.0.0"
