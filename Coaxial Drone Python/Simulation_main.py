import torch
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import pandas as pd
import os 

from Model.model import Model
from Controller.controller import ControllerModel
from Simulation.simulation import Sim
# from Simulation.visualization import Plot

if __name__ == '__main__':

    # select device 
    CPU_Select   =  True
    if CPU_Select is not True:
        dev = torch.device("cuda:0" if (torch.cuda.is_available()) else "cpu")
        print("Device : ", dev)
    else:
        dev = torch.device("cpu")
        print("Device : ", dev)

    # select model & simulation setting
    model       =   Model(platform='Coaxial')
    sinario     =   1  # reference command sinario
    sim         =   Sim(model, dev, sinario, datasave=True)
    # sim         =   Sim(model, dev, sinario, datasave=False)
    GE_modelNum =   1  # ground effect model selection

    # control settings
    counter     =   0
    controlHz   =   1/200
    # controller  =   ControllerModel(controller="PID_Atti")     
    controller  =   ControllerModel(controller="PID_NN")

    # plot        =   Plot(object='coaxial')
    print("Simulation Start!!")
    start = time.time()
    for i in range(len(sim.t)):
        
        # reference command
        sim.XCmd[:, 0]  =   sim.XCmdHist[:,i]
        

        # controller 
        if counter % 5 == 0:
            sim.XCmd, sim.U, sim.FMbControl    =   controller.Controller(model=model, X=sim.X, XCmd=sim.XCmd, Uprev=sim.U ,controlHz=controlHz, dev=dev)
            counter     =   0
        else:
            counter     +=  1

        sim.UActual =   model.Motor(sim.U, sim.UActual, sim.dt)
        sim.XDot    =   model.Dynamics(sim.X, sim.UActual, GE_modelNum)
        sim.X       =   sim.X + sim.XDot * sim.dt
        
        # save data
        sim.XHist[:, i]     =   sim.X.T
        sim.XDotHist[:, i]  =   sim.XDot.T
        sim.XCmdHist[:,i]   =   sim.XCmd.T
        sim.UHist[:, i]     =   sim.U.T
        sim.UActualHist[:,i] =  sim.UActual.T
        sim.FMbControlHist[:,i]    =   sim.FMbControl.T
    
        # plot.UpdatePlot(sim.X[0,0].cpu().numpy(),sim.X[1,0].cpu().numpy(),sim.X[2,0].cpu().numpy(),sim.X[6,0].cpu().numpy(),sim.X[7,0].cpu().numpy(),sim.X[8,0].cpu().numpy())

    # plot.EndPlot()
    elapsed = (time.time() - start)
    print("Simulation Done!! Time : {}".format(elapsed))

    simTime         =   sim.t.cpu()
    simXHist        =   sim.XHist.cpu()
    simXDotHist     =   sim.XDotHist.cpu()
    simUHist        =   sim.UHist.cpu()
    simUAHist       =   sim.UActualHist.cpu()
    simXcmdHist     =   sim.XCmdHist.cpu()
    simFMbControlHist  =   sim.FMbControlHist.cpu()

    # calculate Traing Target Data
    TargetFz        =   (model.m)*(simXDotHist[5,:] - torch.mul(simXHist[3,:],simXHist[10,:]) + torch.mul(simXHist[4,:],simXHist[9,:])) - (model.m*model.g*torch.cos(simXHist[6,:])*torch.cos(simXHist[7,:])-simUHist[0,:]-simUHist[1,:]) 
    TargetMz        =   (model.Iz)*(simXDotHist[11,:] - (model.Ix-model.Iy)*torch.mul(simXHist[9,:],simXHist[10,:])) - (-model.kQ*simUHist[0,:] + model.kQ*simUHist[1,:])    
    
    # TargetFz        =   (model.m)*(simXDotHist[5,:] - torch.mul(simXHist[3,:],simXHist[10,:]) + torch.mul(simXHist[4,:],simXHist[9,:])) - (model.m*model.g*torch.cos(simXHist[6,:])*torch.cos(simXHist[7,:])-simUAHist[0,:]-simUAHist[1,:]) 
    # TargetMz        =   (model.Iz)*(simXDotHist[11,:] - (model.Ix-model.Iy)*torch.mul(simXHist[9,:],simXHist[10,:])) - (-model.kQ*simUAHist[0,:] + model.kQ*simUAHist[1,:])    
    if(sim.datasave == True):
        # Data save as xlsx
        start           =   time.time()
        print("Data Saving......")
        simResultData   =   pd.DataFrame({"Time"   : simTime, 
                                          
                                            "N"      : simXHist[0,:],
                                            "E"      : simXHist[1,:],
                                            "D"      : simXHist[2,:],
                                            "u"      : simXHist[3,:],
                                            "v"      : simXHist[4,:],
                                            "w"      : simXHist[5,:],
                                            "roll"   : simXHist[6,:],
                                            "pitch"  : simXHist[7,:],
                                            "yaw"    : simXHist[8,:],
                                            "p"      : simXHist[9,:],
                                            "q"      : simXHist[10,:],
                                            "r"      : simXHist[11,:],

                                            "NCmd"   : simXcmdHist[0,:],
                                            "ECmd"   : simXcmdHist[1,:],
                                            "DCmd"   : simXcmdHist[2,:],
                                            "uCmd"   : simXcmdHist[3,:],
                                            "vCmd"   : simXcmdHist[4,:],
                                            "wCmd"   : simXcmdHist[5,:],
                                            "rollCmd": simXcmdHist[6,:],
                                            "pitchCmd": simXcmdHist[7,:],
                                            "yawCmd" : simXcmdHist[8,:],
                                            "pCmd"   : simXcmdHist[9,:],
                                            "qCmd"   : simXcmdHist[10,:],
                                            "rCmd"   : simXcmdHist[11,:],

                                            "Tu"     : simUAHist[0,:],
                                            "Tl"     : simUAHist[1,:],
                                            "fin1"   : simUAHist[2,:],
                                            "fin2"   : simUAHist[3,:],
                                            "fin3"   : simUAHist[4,:],
                                            "fin4"   : simUAHist[5,:],

                                            "TuCmd"  : simUHist[0,:],
                                            "TlCmd"  : simUHist[1,:],
                                            "fin1Cmd": simUHist[2,:],
                                            "fin2Cmd": simUHist[3,:],
                                            "fin3Cmd": simUHist[4,:],
                                            "fin4Cmd": simUHist[5,:],

                                            "Fx"     : simFMbControlHist[0,:],
                                            "Fy"     : simFMbControlHist[1,:],
                                            "Fz"     : simFMbControlHist[2,:],
                                            "Mx"     : simFMbControlHist[3,:],
                                            "My"     : simFMbControlHist[4,:],
                                            "Mz"     : simFMbControlHist[5,:],

                                            "TargetFz": TargetFz,
                                            "TargetMz": TargetMz,}).astype("Float32")
        
        os.chdir("./SimulationData")
        timestr                 =   time.strftime("%Y%m%d-%H%M%S")
        simResultDataFilename   =   "results_"+ timestr + ".xlsx"
        
        simResultData.to_excel(simResultDataFilename, index=False)
        lapsed                  =   (time.time() - start)

        print("Data Save Done !! Time : {}".format(elapsed))
    
    else:
        print("Datasave option is OFF!!!")

    # Plot properties 
    fontdict={'fontname': 'Times New Roman',
     'fontsize': 25,
     'style': 'italic', # 'oblique'
      'fontweight': 'bold'}  # 'heavy', 'light', 'ultrabold', 'ultralight'
    plt.rc('legend', fontsize=20)
    
    # Plot Results
    print("Plot Figure Making...")
    # Altitude
    plt.figure(figsize=(15,15), dpi=80)
    plt.plot(simTime, -simXHist[2,:], linewidth=3.0, label='State')
    plt.plot(simTime, -simXcmdHist[2,:], linewidth=3.0, linestyle='--', label='Command')
    plt.xlabel("Time [s]", **fontdict)
    plt.ylabel("Height [m]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.title("Altitude", **fontdict)
    plt.grid(True, linestyle='--', linewidth=0.5)
    # Attitude
    # Roll
    plt.figure(figsize=(15,15), dpi=80)
    plt.subplot(3,1,1)
    plt.plot(simTime, simXHist[6,:] * 180/math.pi, linewidth=3.0, label='State')
    plt.plot(simTime, simXcmdHist[6,:] * 180/math.pi, linewidth=3.0, linestyle='--', label='Command')
    plt.ylabel("Roll [deg]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    # Ptich
    plt.subplot(3,1,2)
    plt.plot(simTime, simXHist[7,:] * 180/math.pi, linewidth=3.0, label='State')
    plt.plot(simTime, simXcmdHist[7,:] * 180/math.pi, linewidth=3.0, linestyle='--', label='Command')
    plt.ylabel("Pitch [deg]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    # Yaw
    plt.subplot(3,1,3)
    plt.plot(simTime, simXHist[8,:] * 180/math.pi, linewidth=3.0, label='State')
    plt.plot(simTime, simXcmdHist[8,:] * 180/math.pi, linewidth=3.0, linestyle='--', label='Command')
    plt.xlabel("Time [s]", **fontdict)
    plt.ylabel("Yaw [deg]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.suptitle("Attitude", **fontdict)
    plt.tight_layout()
    # Rate
    # P
    plt.figure(figsize=(15,15), dpi=80)
    plt.subplot(3,1,1)
    plt.plot(simTime, simXHist[9,:] * 180/math.pi, linewidth=3.0, label='State')
    plt.plot(simTime, simXcmdHist[9,:] * 180/math.pi, linewidth=3.0, linestyle='--', label='Command')
    plt.ylabel("Roll Rate [deg/s]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    # Q
    plt.subplot(3,1,2)
    plt.plot(simTime, simXHist[10,:] * 180/math.pi, linewidth=3.0, label='State')
    plt.plot(simTime, simXcmdHist[10,:] * 180/math.pi, linewidth=3.0, linestyle='--', label='Command')
    plt.ylabel("Pitch Rate [deg/s]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    # R
    plt.subplot(3,1,3)
    plt.plot(simTime, simXHist[11,:] * 180/math.pi, linewidth=3.0, label='State')
    plt.plot(simTime, simXcmdHist[11,:] * 180/math.pi, linewidth=3.0, linestyle='--', label='Command')
    plt.xlabel("Time [s]", **fontdict)
    plt.ylabel("Yaw Rate [deg/s]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.suptitle("Rate", **fontdict)
    plt.tight_layout()
    # Control input
    # Motor 
    plt.figure(figsize=(15,15), dpi=80)
    plt.subplot(2,1,1)
    plt.plot(simTime, simUHist[0,:].cpu(), linewidth=3.0, label='Upper Motor')
    plt.plot(simTime, simUHist[1,:].cpu(), linewidth=3.0, label='Lower Motor')
    plt.ylabel("Thrust [N]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    # control fin
    plt.subplot(2,1,2)
    plt.plot(simTime, simUHist[2,:] * 180/math.pi, linewidth=3.0, label='fin1')
    plt.plot(simTime, simUHist[3,:] * 180/math.pi, linewidth=3.0, label='fin2')
    plt.plot(simTime, simUHist[4,:] * 180/math.pi, linewidth=3.0, label='fin3')
    plt.plot(simTime, simUHist[5,:] * 180/math.pi, linewidth=3.0, label='fin4')
    plt.xlabel("Time [s]", **fontdict)
    plt.ylabel("Control Surface [deg]", **fontdict)
    plt.legend()
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.suptitle("Control Input", **fontdict)
    plt.tight_layout()
    plt.show()