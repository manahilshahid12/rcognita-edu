import numpy as np


class TrajectoryGenerator:
    def _state_dyn(self, t, state, action, observation):
        R=action[0]/action[1]
        x=R*np.cos(observation[2])
        y=R*np.sin(observation[2])
        theta=np.atan((y[t]-y[t-1])/x[t]-x[t-1])

    




