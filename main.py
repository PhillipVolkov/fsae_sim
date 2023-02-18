from sim import Sim

sim = Sim()
while sim.running:
  sim.update()
  sim.render()
