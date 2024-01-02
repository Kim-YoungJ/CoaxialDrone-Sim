import sys
import torch
import math 
import random

class Sim():
  def __init__(self, model, dev, sinario, datasave):

    try:
      # simulation setting
      self.datasave =   datasave
      self.device   =   dev
      self.nx       =   model.nx
      self.nu       =   model.nu

      self.t0       =   0
      self.tf       =   40
      self.dt       =   0.001

      # state
      self.X0       =   torch.zeros((self.nx, 1), device = self.device)
      # inital pos
      self.X0[0, :] = 0.0
      self.X0[1, :] = 0.0
      self.X0[2, :] = -0.8
      #initial atti
      self.X0[6,:]  = 0.0
      self.X0[7,:]  = 0.0
      self.X0[8,:]  = math.radians(0.0)
      
      # control 
      self.U0       = torch.zeros((self.nu, 1), device = self.device)
      self.U0[0, :] = ((model.m)*(model.g))/2
      self.U0[1, :] = ((model.m)*(model.g))/2

      print("Simulation Time - " + str(self.tf)+"sec")
      print("Simulation Initialized! - " + model.platform)

    except:
      print("Simulation Initialized Wrong!")
      sys.exit(1)


    self.t          =   torch.arange(self.t0, self.tf+self.dt, self.dt, device = self.device)
    self.nT         =   self.t.shape[0]
    
    self.X          =   self.X0
    self.XDot       =   torch.zeros((self.nx, 1), device = self.device)
    self.XCmd       =   torch.zeros((self.nx, 1), device = self.device)
    self.U          =   self.U0
    self.UActual    =   self.U0
    self.FMbControl =   torch.zeros((6, 1), device = self.device)

    self.XHist      =   torch.zeros((self.nx, self.nT), device = self.device)
    self.UHist      =   torch.zeros((self.nu, self.nT), device = self.device)
    self.UActualHist=   torch.zeros((self.nu, self.nT), device = self.device)
    self.XDotHist   =   torch.zeros((self.nx, self.nT), device = self.device)
    self.FMbControlHist =   torch.zeros((6, self.nT), device = self.device)
    # self.costHist   =   torch.zeros((1, self.nT), device = self.device)

    self.XHist[:,0] =   self.X0.T
    self.UHist[:,0] =   self.U0.T
    self.UActualHist[:,0] = self.U0.T

    # reference command 
    self.XCmdHist   =   referenceCmd(self, sinario)
    
    

def referenceCmd(self, sinario):
  time = torch.arange(self.t0, self.tf+self.dt, self.dt)
  XCmdHist = torch.zeros((self.nx, self.nT), device=self.device)
  
  if sinario == 1:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<10):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=10 and time[i]<20):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=20 and time[i]<30):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=30 and time[i]<=40):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))   

  elif sinario == 2:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<5):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=5 and time[i]<35):
        XCmdHist[2,i] = (0.6/30) * (time[i]-5)-0.8
        XCmdHist[8,i].fill_(math.radians(0.0))
      else:
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
       

  elif sinario == 3:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<10):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(5.0))
      elif (time[i]>=10 and time[i]<20):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=20 and time[i]<30):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(-5.0))
      elif (time[i]>=30 and time[i]<=40):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))  

  elif sinario == 4:
    for i in range(len(time)):
      XCmdHist[2,i] = -0.8 + 0.6 * torch.sin((2*torch.pi/30)*time[i])
                         

  elif sinario == 5:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<10):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/5)*time[i])
      elif (time[i]>=10 and time[i]<20):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/5)*time[i])
      elif (time[i]>=20 and time[i]<30):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/6)*time[i])
      elif (time[i]>=30 and time[i]<40):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/6)*time[i])
      elif (time[i]>=40 and time[i]<50):  
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/7)*time[i])
      elif (time[i]>=50 and time[i]<60):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/7)*time[i])
      elif (time[i]>=60 and time[i]<70): 
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/8)*time[i])
      elif (time[i]>=70 and time[i]<80):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/8)*time[i])
      elif (time[i]>=80 and time[i]<90):   
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/9)*time[i])
      elif (time[i]>=90 and time[i]<100):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/9)*time[i])
      else:
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]/2) * torch.sin((2*torch.pi/10)*time[i])
  
  elif sinario == 6:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<30):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp(time[i]*0.01)) * torch.sin((2*torch.pi/3)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=30 and time[i]<60):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-30)*0.01)) * torch.sin((2*torch.pi/4)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=60 and time[i]<90):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-60)*0.01)) * torch.sin((2*torch.pi/5)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=90 and time[i]<120):  
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-90)*0.01)) * torch.sin((2*torch.pi/6)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=120 and time[i]<150):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-120)*0.01)) * torch.sin((2*torch.pi/7)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=150 and time[i]<180):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-150)*0.01)) * torch.sin((2*torch.pi/8)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=180 and time[i]<210):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-180)*0.01)) * torch.sin((2*torch.pi/9)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      elif (time[i]>=210 and time[i]<240):
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-210)*0.01)) * torch.sin((2*torch.pi/10)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05
      else:
        XCmdHist[2,i] = self.X0[2,:] + ((torch.abs(self.X0[2,:]))*torch.exp((time[i]-240)*0.01)) * torch.sin((2*torch.pi/11)*time[i])
        if XCmdHist[2,i] > 0:
          XCmdHist[2,i] = -0.05

  elif sinario == 7:
    for i in range(len(time)):
        XCmdHist[2,i] = self.X0[2,:] +(self.X0[2,:]) * torch.sin((2*torch.pi/30)*time[i])

  elif sinario == 8:
    for i in range(len(time)):
      XCmdHist[2,i] = -0.8 + 0.8 * torch.sin((2*torch.pi/30)*time[i])
      # XCmdHist[8,i] = 0 + 5 * torch.sin((2*torch.pi/60)*time[i])
      XCmdHist[8,i] = 0 + 5* (math.pi/180) * torch.sin((2*torch.pi/30)*time[i])

  elif sinario == 9:
    data1 = -0.8
    data2 = 0
    for i in range(len(time)):
      XCmdHist[2,i].fill_(data1)
      XCmdHist[8,i].fill_(data2)
      if time[i] == 0:
        pass
      else:
        if (time[i] % 10) == 0:
          data1 = -random.randint(2,12) / 10 
          # XCmdHist[8,i] = 0 + 5 * torch.sin((2*torch.pi/60)*time[i])
        if (time[i] % 15) == 0:
          data2 = -5*(math.pi/180) + 10*(math.pi/180) * random.randint(0,10) / 10
      
  
  elif sinario == 10:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<10):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=10 and time[i]<20):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=20 and time[i]<30):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=30 and time[i]<40):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=40 and time[i]<50):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=50 and time[i]<60):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=60 and time[i]<65):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=65 and time[i]<95):
        XCmdHist[2,i] = (0.6/30) * (time[i]-65)-0.8
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=95 and time[i]<100):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=100 and time[i]<110):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(5.0))
      elif (time[i]>=110 and time[i]<120):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=120 and time[i]<130):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(-5.0))
        # Sex Sex Sex Sex Sex Sex Sex
      elif (time[i]>=130 and time[i]<=140):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=140 and time[i]<=150):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(5.0))
      elif (time[i]>=150 and time[i]<=160):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=160 and time[i]<=170):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(-5.0))
      else:
        XCmdHist[2,i] = -0.8 + 0.6 * torch.sin((2*torch.pi/30)*time[i])
        XCmdHist[8,i] = 0 + 5* (math.pi/180) * torch.sin((2*torch.pi/30)*time[i])
  
  elif sinario == 11:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<10):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=10 and time[i]<20):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=20 and time[i]<30):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=30 and time[i]<40):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=40 and time[i]<50):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=50 and time[i]<60):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=60 and time[i]<70):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=70 and time[i]<80):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=80 and time[i]<90):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=90 and time[i]<100):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=100 and time[i]<110):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=110 and time[i]<=120):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))

  elif sinario == 12:
    for i in range(len(time)):
      XCmdHist[2,i] = -0.4 + 0.4 * torch.sin((2*torch.pi/25)*time[i])
      # XCmdHist[8,i] = 0 + 5 * torch.sin((2*torch.pi/60)*time[i])
      XCmdHist[8,i] = 0 + 3* (math.pi/180) * torch.sin((2*torch.pi/25)*time[i])     

  elif sinario == 13:
    for i in range(len(time)):
      if (time[i]>=0 and time[i]<10):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(5.0))
      elif (time[i]>=10 and time[i]<20):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))
      elif (time[i]>=20 and time[i]<30):
        XCmdHist[2,i].fill_(-0.2)
        XCmdHist[8,i].fill_(math.radians(-5.0))
      elif (time[i]>=30 and time[i]<=40):
        XCmdHist[2,i].fill_(-0.8)
        XCmdHist[8,i].fill_(math.radians(0.0))    
  
  else:
    sys.exit("!!!Select the appropriate scenario!!!")

  return XCmdHist
