import sys
import os 
import time 
import pandas as pd
import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
from torch.nn.utils import spectral_norm


# Setting Parameters
timestr = time.strftime("%Y%m%d-%H%M%S")
def Params(batch_size=16, epoch=100, 
           shuffle_train=True, drop_last=True, num_workers=4, 
           pin_memory=True, persistent_workers=False):
    
    params = {'batch_size': batch_size,
            'shuffle_train': shuffle_train,
            'drop_last': drop_last,
            'epoch': epoch,
            'num_workers' : num_workers,
            'pin_memory' : pin_memory,
            'persistent_workers' : persistent_workers
            }

    return params

# DATASETS for read Xlxs
class ShieldDroneData(Dataset):
    def __init__(self, dataFile, dev):
        super(ShieldDroneData, self).__init__()
        
        df = pd.read_excel(dataFile)
        # print(df)
        # print(df.Time)
        
        X_temp = df.loc[:,['D','w','yaw','r','TlCmd','TuCmd']]
        Y_temp = df.loc[:,['TargetFz','TargetMz']]
        # print(X_temp)
        
        Xdata = torch.tensor(X_temp.values, dtype=torch.float32)
        Ydata = torch.tensor(Y_temp.values, dtype=torch.float32)
        # print(Xdata)
        # print(Ydata)
        
        self.datalen = Xdata.shape[0]
        self.Xdata = Xdata.to(dev)
        self.Ydata = Ydata.to(dev)
        
    def __len__(self):

        return self.datalen
    
    def __getitem__(self, idx):
        
        return self.Xdata[idx,:], self.Ydata[idx,:]
    

def Train_Dataloader(train_data, params):
    dataloader  = DataLoader(train_data,
                            batch_size=params['batch_size'],
                            shuffle=params['shuffle_train'],
                            drop_last=params['drop_last'],
                            num_workers=params['num_workers'],
                            pin_memory= params['pin_memory'],
                            persistent_workers=params['persistent_workers'])

    return dataloader


# DNN Models
inputSize = 6
outputSize = 2
class DNN_Compensetor(nn.Module):
    def __init__(self):
        super(DNN_Compensetor, self).__init__()

        self.fc1 = nn.Linear(inputSize, 10)
        # self.fc1 = spectral_norm(nn.Linear(inputSize, 10))
        self.activate1 = nn.ReLU() # nn.Misth()
        # self.activate1 = nn.Mish()
        
        self.fc2 = nn.Linear(10, 30)
        # self.fc2 = spectral_norm(nn.Linear(10, 30))
        self.activate2 = nn.ReLU() # nn.Misth()
        # self.activate2 = nn.Mish()
        
        self.fc3 = nn.Linear(30, 12)
        # self.fc3 = spectral_norm(nn.Linear(30, 12))
        self.activate3 = nn.ReLU() # nn.Misth()
        # self.activate3 = nn.Mish()
        
        self.fc4 = nn.Linear(12, outputSize)
        # self.fc4 = spectral_norm(nn.Linear(12, outputSize))
    
    def forward(self, x): 
        x = self.activate1(self.fc1(x))
        x = self.activate2(self.fc2(x))
        x = self.activate3(self.fc3(x))
        x = self.fc4(x)

        return x 
    
# Training 
def train(model, optimizer, lossfunction, train_dataloader, train_loss_list, epoch):
    
    train_loss = 0.0
    total_batch = len(train_dataloader)
    
    epoch_time = time.time()
    
    for batch_idx, (x,y) in enumerate(train_dataloader):
            
        # Data 
        X, Y = x, y
    
        # print(X)
        # print(Y)
        # Forward Propagation
        Y_pred = model(X)
        loss = lossfunction(Y_pred, Y)

        # set optimizer to zero gradient to remove previous epoch gradients
        optimizer.zero_grad()

        # backward propagtion 
        loss.backward()
            
        # optimize 
        optimizer.step()

        #Lip
        for param in model.parameters():
            M = param.detach().cpu().numpy()
            if M.ndim > 1:
                u, s, vh = np.linalg.svd(M)
                if s[0] > 1.0:
                    param.data = ((param / s[0]) * 1.0)

        train_loss += loss.item()

    epoch_elasped = time.time() - epoch_time

    train_loss = loss.item() / total_batch
    print("\nEpoch Training Time : {}".format(epoch_elasped))
    print("Epoch %d: loss = %4e" %(epoch, train_loss))
    train_loss_list.append(train_loss)

# Test
def Test(model, test_dataloader, lossfunction, truelist, predList):
    test_loss = 0.0
    total_loss = 0.0

    model.eval()
    for batch_idx, (x,y) in enumerate(test_dataloader):

        # Data 
        X, Y = x, y
        
        # prediction
        Y_pred = model(X)
        loss = lossfunction(Y_pred, Y)
        
        test_loss = loss.item()
        
        total_loss += test_loss
        
        truelist.append(Y)
        predList.append(Y_pred)
    
    print("Total loss : %4e" %(total_loss))


# save model(.pt file)
def save_model_pt(model_, optimizer_, epoch_, ori_path_):
    try : 
        os.chdir(ori_path_)
        data_path = "./NetworkPytorch/model_pt_file"
        os.chdir(data_path)
    except:
        print("Check path!!")
        sys.exit(0)

    save_as = timestr + '_' + str(epoch_) + ".pt"
    best_model = {'model':model_.state_dict(),
                  'optimizer':optimizer_.state_dict(),
                  'epoch':epoch_}
    torch.save(best_model, save_as)
