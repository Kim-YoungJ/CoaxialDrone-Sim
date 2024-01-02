import torch
import math

def InitCoaxial(model):
    # Platform
    model.nx        =   12
    model.nu        =   6
    model.m         =   1.585
    model.g         =   9.81
    model.dz        =   0.2
    model.d         =   0.1
    model.finA      =   9.282e-2
    model.Ix        =   1.08e-2
    model.Iy        =   1.08e-2
    model.Iz        =   3.05e-2
    model.thrustMax =   10.0
    model.cT        =   1.1859e-7
    model.cM        =   2.4167e-9
    model.kQ        =   model.cM / model.cT
    model.cL        =   2 * (math.pi)
    model.lprop     =   0.10
    model.rprop     =   0.2794
    model.finmax    =   math.radians(30.0)

    model.f         =   StateDynamicsCoaxial
    model.G         =   ControlDynamicsCoaxial

    # Motor
    model.tauMotor  =   0.05
    model.motor     =   MotorDynamics

    model.platform  =   "Coaxial"


def StateDynamicsCoaxial(model, X):
    f           =   torch.zeros((X.shape[0],X.shape[1]), device = X.device)

    # moddel prameter
    m           =   model.m
    g           =   model.g
    Ix          =   model.Ix
    Iy          =   model.Iy
    Iz          =   model.Iz

    # state information
    N           =   X[0, :]
    E           =   X[1, :]
    D           =   X[2, :]
    u           =   X[3, :]
    v           =   X[4, :]
    w           =   X[5, :]

    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]
    p           =   X[9, :]
    q           =   X[10, :]
    r           =   X[11, :]

    uvw         =   torch.tensor([[u],[v],[w]])
    pqr         =   torch.tensor([[p],[q],[r]])

    velNed      =   torch.matmul(EulerDCM(roll, pitch, yaw),uvw)
    eulerDot    =   torch.matmul(PqrDCM(p, q, r), pqr)

    f[0,:]      =   velNed[0, 0]
    f[1,:]      =   velNed[1, 0]
    f[2,:]      =   velNed[2, 0]

    f[3,:]      =   v*r - w*q
    f[4,:]      =   w*p - u*r
    f[5,:]      =   u*q - v*p

    f[6,:]      =   eulerDot[0, 0]
    f[7,:]      =   eulerDot[1, 0]
    f[8,:]      =   eulerDot[2, 0]

    f[9,:]      =   ((Iy - Iz) / Ix) * q * r
    f[10,:]     =   ((Iz - Ix) / Iy) * r * p
    f[11,:]     =   ((Ix - Iy) / Iz) * p * q

    return f


def ControlDynamicsCoaxial(model, X, U, GE_modelNum):
    G           =   torch.zeros((X.shape[0],X.shape[1]), device = X.device)

    m           =   model.m
    Ix          =   model.Ix
    Iy          =   model.Iy
    Iz          =   model.Iz

    Fx, Fy, Fz, L, M, N     =   CoaxialForceMomentGen(model, X, U, GE_modelNum)

    G[3,:]      =   Fx / m
    G[4,:]      =   Fy / m
    G[5,:]      =   Fz / m

    G[9,:]      =   L / Ix
    G[10,:]     =   M / Iy
    G[11,:]     =   N / Iz

    return G

def CoaxialForceMomentGen(model, X, U, GE_modelNum):
    m           =   model.m
    g           =   model.g
    dz          =   model.dz
    d           =   model.d
    finA        =   model.finA
    cT          =   model.cT
    cM          =   model.cM
    kQ          =   model.kQ
    cL          =   model.cL
    rprop       =   model.rprop

    N           =   X[0, :]
    E           =   X[1, :]
    D           =   X[2, :]
    u           =   X[3, :]
    v           =   X[4, :]
    w           =   X[5, :]

    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]
    p           =   X[9, :]
    q           =   X[10, :]
    r           =   X[11, :]

    T1Cmd       =   U[0,:]
    T2Cmd       =   U[1, :]
    del1Cmd     =   U[2, :]
    del2Cmd     =   U[3, :]
    del3Cmd     =   U[4, :]
    del4Cmd     =   U[5, :]

    # gravity zone
    Grav        =   torch.tensor([[0],[0],[g]])
    R           =   EulerDCM(roll, pitch, yaw)

    FGrav       =   torch.matmul(R.T, m*Grav)
    
    FxGrav      =   FGrav[0, 0]
    FyGrav      =   FGrav[1, 0]
    FzGrav      =   FGrav[2, 0]
    
    LGrav       =   0
    MGrav       =   0
    NGrav       =   0

    # Thrust zone
    ThrustArea  =   (math.pi) * (rprop**2)
    rho         =   1.225
    
    Vu          =   math.sqrt((T1Cmd)/(2*rho*ThrustArea))
    Vl          =   math.sqrt((T2Cmd) / (2*rho*ThrustArea))
    Vs          =   (Vl+Vu) + math.sqrt(((Vl+Vu)**2) - (Vu**2)*(2-(Vu/(Vu+Vl))))
    Tu          =   T1Cmd
    Tl          =   rho*ThrustArea*(Vu+Vl)*Vs - rho*ThrustArea*(Vu**2)

    FxThrust    =   0
    FyThrust    =   0
    FzThrust    =   -Tu-Tl
    
    LThrust     =   0
    MThrust     =   0
    NThrust     =   -Tu*kQ + Tl*kQ
    
    if (GE_modelNum != 0):
        FzThrust  = GroundEffectModel(GE_modelNum, rprop, D, FzThrust)

    # Aerodynamic zone
    q           =   0.5*1.225*finA*(Vs**2)

    FxAero      =   q*cL*(del2Cmd - del4Cmd)
    FyAero      =   q*cL*(del1Cmd - del3Cmd)
    FzAero      =   0
    
    LAero       =   q*cL*(-del1Cmd + del3Cmd)*dz
    MAero       =   q*cL*(-del4Cmd + del2Cmd)*dz
    NAero       =   q*cL*(del1Cmd + del2Cmd + del3Cmd + del4Cmd)*d

    Fx          =   FxGrav + FxAero + FxThrust
    Fy          =   FyGrav + FyAero + FyThrust
    Fz          =   FzGrav + FzAero + FzThrust

    L           =   LGrav + LAero + LThrust
    M           =   MGrav + MAero + MThrust
    N           =   NGrav + NAero + NThrust

    return Fx, Fy, Fz, L, M, N

def MotorDynamics(model, UCmd, Uactual, dt):

    Uactual[0,:]    =   Uactual[0,:] + (1/model.tauMotor)*(UCmd[0,:]-Uactual[0,:])*dt
    Uactual[1,:]    =   Uactual[1,:] + (1/model.tauMotor)*(UCmd[1,:]-Uactual[1,:])*dt

    return Uactual

def GroundEffectModel(Num, rprop, D, Thrust):
    if (Num == 1):
        rho     =   5
        Thrust  =   (1 / (1 - rho*((rprop / (4 * max(-D, 0.2))))**2))*Thrust

    return Thrust


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


def PqrDCM(roll, pitch, yaw):
    W       =   torch.zeros((3,3))
    
    W[0,0]  =   1
    W[0,1]  =   torch.sin(roll)*torch.tan(pitch)
    W[0,2]  =   torch.cos(roll)*torch.tan(pitch)

    W[1,0]  =   0
    W[1,1]  =   torch.cos(roll)
    W[1,2]  =   -torch.sin(roll)

    W[2,0]  =   0
    W[2,1]  =   torch.sin(roll)/torch.cos(pitch)
    W[2,2]  =   torch.cos(roll)/torch.cos(pitch)

    return W