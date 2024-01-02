import torch

def InitCartpole(model):
    model.nx     =   5
    model.nu     =   1
    model.mc     =   1
    model.mp     =   0.01
    model.l      =   1
    model.g      =   9.81
    model.tau    =   0.05
    model.Umax   =   5
    model.f      =   StateDynamicsCartpole
    model.G      =   ControlDynamicsCartpole
    model.Gc     =   ControlOnlyDynamicsCartpole
    model.L      =   RunningCostCartpole
    model.platform  =   "Cartpole"


def StateDynamicsCartpole(model, X):
    f         =   torch.zeros((X.shape[0],X.shape[1]), device = X.device)

    mc        =   model.mc
    mp        =   model.mp
    l         =   model.l
    g         =   model.g
    tau       =   model.tau

    x         =   X[0,:]
    theta     =   X[1,:]
    xDot      =   X[2,:]
    thetaDot  =   X[3,:]
    F         =   X[4,:]

    f[0,:]    =   xDot
    f[1,:]    =   thetaDot
    f[2,:]    =   (1/(mc+mp*torch.sin(theta)**2))*(F + (mp*torch.sin(theta)*(l*thetaDot**2 + g*torch.cos(theta))))
    f[3,:]    =   (1/(l*(mc+mp*torch.sin(theta)**2)))*(-F*torch.cos(theta) - mp*l*(thetaDot**2)*torch.cos(theta)*torch.sin(theta) - (mc+mp)*g*torch.sin(theta))
    f[4,:]    =   -1/tau*F

    return f


def ControlDynamicsCartpole(model, X):
    G         =   torch.zeros((model.nx,model.nu), device = X.device)
    mc        =   model.mc
    mp        =   model.mp
    l         =   model.l
    g         =   model.g
    tau       =   model.tau

    x         =   X[0,:]
    theta     =   X[1,:]
    xDot      =   X[2,:]
    thetaDot  =   X[3,:]
    F         =   X[4,:]

    for i in range(model.nu):
      G[model.nx-model.nu+i,i] = 1/tau

    return G


def ControlOnlyDynamicsCartpole(model, X):
    Gc        =   torch.zeros((model.nu,model.nu), device = X.device)

    mc        =   model.mc
    mp        =   model.mp
    l         =   model.l
    g         =   model.g
    tau       =   model.tau

    for i in range(model.nu):
      Gc[i,i] = 1/tau

    return Gc


def RunningCostCartpole(model,X):

    # L         =   torch.zeros(1,X.shape[-1]).to(X.device)

    k1        =   100
    k2        =   500
    k3        =   1
    k4        =   1

    x         =   X[0,:]
    theta     =   X[1,:]
    xDot      =   X[2,:]
    thetaDot  =   X[3,:]

    L         =   k1*x**2 + k2*(1+torch.cos(theta))**2 + k3*xDot**2 + k4*thetaDot**2

    return L