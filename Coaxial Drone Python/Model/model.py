import sys
import torch
from Model import cartpole
from Model import coaxial
import numpy as np


class Model():

    def __init__(self, platform="Coaxial"):
        try:
            eval(platform.lower() + '.Init' + platform + '(self)')
            print("Model Initialized! - " + platform)
        except:
            print("Wrong Platform Model Select !")
            sys.exit(1)

    def Dynamics(self, X, U, GE_modelNum):
        XDot = self.f(self, X) + self.G(self, X, U, GE_modelNum)

        return XDot

    def Motor(self, UCmd, UActual, dt):
        UActual = self.motor(self, UCmd, UActual, dt)

        # constraint Saturation 
        UActual[0, :] =   torch.minimum(torch.maximum(UActual[0, :], torch.tensor(0.1)), torch.tensor(self.thrustMax))
        UActual[1, :] =   torch.minimum(torch.maximum(UActual[1, :], torch.tensor(0.1)), torch.tensor(self.thrustMax))
        UActual[2, :] =   finSaturation(self.finmax, UActual[2, :])
        UActual[3, :] =   finSaturation(self.finmax, UActual[3, :])
        UActual[4, :] =   finSaturation(self.finmax, UActual[4, :])
        UActual[5, :] =   finSaturation(self.finmax, UActual[5, :])

        return UActual

def finSaturation(finMax, alpha):
    
    if (alpha >= finMax):
        alpha = finMax

    elif (alpha <= -finMax):
        alpha = -finMax

    else:
        alpha = alpha

    return alpha