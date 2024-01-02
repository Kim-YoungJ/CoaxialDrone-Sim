import torch
import math 

def InitPID_Atti(controller):
    # tau_D                   =   0.6
    # tau_velD                =   0.1

    tau_D                   =   0.4
    tau_velD                =   0.1

    tau_roll                =   0.9
    tau_p                   =   0.3

    tau_pitch               =   0.9
    tau_q                   =   0.3

    tau_yaw                 =   0.4
    tau_r                   =   0.1

    controller.K_D          =   1 / tau_D
    controller.K_velD       =   1 / tau_velD 
    controller.K_roll       =   1 / tau_roll
    controller.K_pitch      =   1 / tau_pitch
    controller.K_yaw        =   1 / tau_yaw
    controller.K_p          =   1 / tau_p
    controller.K_q          =   1 / tau_q
    controller.K_r          =   1 / tau_r
    controller.Igain_velD   =   (1 / tau_velD) * 0.1
    controller.Igain_p      =   (1 / tau_p) * 0.1
    controller.Igain_q      =   (1 / tau_q) * 0.1
    controller.Igain_r      =   (1 / tau_r) * 0.1
    
    # I_term static
    controller.IvelD        =   0.0
    controller.Ip           =   0.0
    controller.Iq           =   0.0
    controller.Ir           =   0.0

    controller.Allocator    =   Allocator

    controller.controller   = "PID_Atti"


def AltitudeController(controller, model, X, XCmd, controlHz):
    
    # state information
    d           =   X[2, :]
    u           =   X[3, :]
    v           =   X[4, :]
    w           =   X[5, :]
    
    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]
    
    uvw         =   torch.tensor([[u],[v],[w]])
    velNed      =   torch.matmul(EulerDCM(roll, pitch, yaw),uvw)
    velD        =   velNed[2, 0]

    # command information
    dCmd        =   XCmd[2,:]

    # P-PI controller
    errorD      =   dCmd - d
    
    velDCmd     =   (controller.K_D * errorD)

    errorVelD   =   velDCmd - velD
    
    controller.IvelD    =   controller.IvelD + controller.Igain_velD * errorVelD * controlHz

    # Fz trim term
    Fz_trim     =   -(model.m * model.g) / (torch.cos(pitch) * torch.cos(roll))

    FzControl   =   (model.m / (torch.cos(pitch) * torch.cos(roll))) * (controller.K_velD * errorVelD + controller.IvelD) + Fz_trim
    FzControl  =   torch.maximum(torch.minimum(FzControl, torch.tensor(0.0)), torch.tensor(-model.thrustMax * 2))
    
    XCmd[5, :]  =   (1/(torch.cos(pitch) * torch.cos(roll))) * velDCmd

    return XCmd, FzControl


def AttiThrController(controller, X, XCmd):

    # state information 
    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]
    p           =   X[9, :]
    q           =   X[10, :]
    r           =   X[11, :]

    # command information 
    rollCmd      =   XCmd[6, :]
    pitchCmd     =   XCmd[7, :]
    yawCmd       =   XCmd[8, :]

    # Atti trim 
    P_trim       =   -(torch.sin(roll)*torch.tan(pitch)*q + torch.cos(roll)*torch.tan(pitch)*r)
    Q_trim       =   r*torch.sin(roll)
    R_trim       =   -q*torch.tan(roll)

    # P controller
    errorRoll    =   rollCmd - roll
    errorPitch   =   pitchCmd - pitch
    errorYaw     =   yawCmd - yaw

    pCmd         =   controller.K_roll * errorRoll + P_trim  
    qCmd         =   (controller.K_pitch / torch.cos(roll)) * errorPitch + Q_trim
    rCmd         =   (controller.K_yaw * (torch.cos(pitch)/torch.cos(roll))) * errorYaw + R_trim

    XCmd[9, :]   =   pCmd
    XCmd[10, :]  =   qCmd
    XCmd[11, :]  =   rCmd

    return XCmd

 
def RateController(controller, model, X, XCmd, controlHz):

    # state information 
    Ix          =   model.Ix
    Iy          =   model.Iy
    Iz          =   model.Iz

    p           =   X[9, :]
    q           =   X[10, :]
    r           =   X[11, :]

    # command information 
    pCmd        =   XCmd[9, :]
    qCmd        =   XCmd[10, :]
    rCmd        =   XCmd[11, :]

    # Trim term
    L_trim      =   (Iz - Iy) * q * r
    M_trim      =   (Ix - Iz) * p * r
    N_trim      =   (Iy - Ix) * q * p

    # PI controller 
    errorP      =   pCmd - p
    errorQ      =   qCmd - q
    errorR      =   rCmd - r 

    controller.Ip = controller.Ip + controller.Igain_p * errorP * controlHz
    controller.Iq = controller.Iq + controller.Igain_q * errorQ * controlHz
    controller.Ir = controller.Ir + controller.Igain_r * errorR * controlHz

    LControl    =   Ix * (controller.K_p * errorP + controller.Ip) + L_trim
    MControl    =   Iy * (controller.K_q * errorQ + controller.Iq) + M_trim
    NControl    =   Iz * (controller.K_r * errorR + controller.Ir) + N_trim
    
    return XCmd, LControl, MControl, NControl, 


def Allocator(controller, model, X, XCmd, Uprev, controlHz, dev):
    
    # Control
    XCmd, FzControl                    = AltitudeController(controller, model, X, XCmd, controlHz)
    XCmd                               = AttiThrController(controller, X, XCmd)
    XCmd, LControl, MControl, NControl = RateController(controller, model, X, XCmd, controlHz)
    
    Vs      =   math.sqrt(-FzControl / (2 * 1.225 * math.pi * (model.rprop**2)))
    q       =   0.5 * 1.225 * model.finA * (Vs**2)
    k1      =   model.cL * q * model.dz
   
    kQ      =   model.kQ

    CA      =   torch.Tensor([[-1, -1, 0, 0, 0 ,0],
                              [ 0, 0, -k1, 0, k1, 0],
                              [0, 0, 0, k1, 0, -k1],
                              [-kQ, kQ, 0, 0, 0, 0]])
    
    # k2      =   model.cL * q * model.d
    # CA      =   torch.Tensor([[-1, -1, 0, 0, 0 ,0],
    #                           [ 0, 0, -k1, 0, k1, 0],
    #                           [0, 0, 0, k1, 0, -k1],
    #                           [-kQ, kQ, k2, k2, k2, k2]])
    
    CMatrix =   torch.Tensor([[FzControl],[LControl],[MControl],[NControl]])
    
    CMatrixHist =   torch.Tensor([[0.0],[0.0],[FzControl],[LControl],[MControl],[NControl]]).to(dev)

    U       =   torch.matmul(torch.linalg.pinv(CA), CMatrix).to(dev)
    
    # constraint Saturation 
    U[0, :] =   torch.minimum(torch.maximum(U[0, :], torch.tensor(0.1)), torch.tensor(model.thrustMax))
    U[1, :] =   torch.minimum(torch.maximum(U[1, :], torch.tensor(0.1)), torch.tensor(model.thrustMax))
    U[2, :] =   finSaturation(model.finmax, U[2, :])
    U[3, :] =   finSaturation(model.finmax, U[3, :])
    U[4, :] =   finSaturation(model.finmax, U[4, :])
    U[5, :] =   finSaturation(model.finmax, U[5, :])
    
    # print(XCmd)
    # print(U)
    # print(CMatrixHist)
    return XCmd, U, CMatrixHist 


def EulerDCM(roll, pitch, yaw):
    R       =   torch.zeros((3,3))

    cr      =   torch.cos(roll)
    sr      =   torch.sin(roll)
    cp      =   torch.cos(pitch)
    sp      =   torch.sin(pitch)
    cy      =   torch.cos(yaw)
    sy      =   torch.sin(yaw)

    R[0, 0] = cy*cp
    R[0, 1] = -sy*cr + cy*sp*sr
    R[0, 2] = sy*sr + cy*sp*cr

    R[1, 0] = sy*cp
    R[1, 1] = cy*cr + sy*sp*sr
    R[1, 2] = -cy*sr + sy*sp*cr

    R[2, 0] = -sp
    R[2, 1] = cp*sr
    R[2, 2] = cp*cr

    return R

def finSaturation(finMax, alpha):
    
    if (alpha >= finMax):
        alpha = finMax

    elif (alpha <= -finMax):
        alpha = -finMax

    else:
        alpha = alpha

    return alpha