from psim import Configuration, sims, Simulation

import lin
import pytest


def test_orbit_controller():
    """Test the orbit controller.
    This boots the simulation starting in standby (after detumbling)
    """
    configs = ['sensors/base', 'truth/base', 'fc/base', 'truth/standby']
    configs = ['config/parameters/' + f + '.txt' for f in configs]
    print(configs)
    config = Configuration(configs)
    sim = Simulation(sims.OrbitControllerTest, config)

    #The spacecrafts are given 30 days to rendezvous
    timeout = 30 * 24 * 3600 * 1000000000
    threshold = 0.5
    sim.step()

    while sim['truth.leader.hill.dr.norm'] > threshold:
        assert sim['truth.t.ns'] < timeout , 'Spacecrafts failed to rendezvous in alloted time'
        sim.step()