import sys
import torch 
import numpy as np 

from Controller import pid_atti
from Controller import pid_nn

class ControllerModel():
    def __init__(self, controller="PID_Atti"):
        try:
            eval(controller.lower() + '.Init' + controller + '(self)')
            print("Controller Initialized! - " + controller)
        except:
            print("Wrong Controller Model Select !")
            sys.exit(1)

    def Controller(self, model, X, XCmd, Uprev, controlHz, dev):
        XCmd, U, FMHist = self.Allocator(self, model, X, XCmd, Uprev, controlHz, dev)

        return XCmd, U, FMHist 