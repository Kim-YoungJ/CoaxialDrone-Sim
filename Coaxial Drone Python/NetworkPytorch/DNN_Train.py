import os
import sys
import time 
import torch.optim as optim
import pandas as pd
import matplotlib.pyplot as plt
from torch.utils.data import random_split
from DNN_Func import * 

def recent_file_search(file_path):
        file_name_and_time_lst = []
        # 해당 경로에 있는 파일들의 생성시간을 함께 리스트로 넣어줌. 
        for f_name in os.listdir(f"{file_path}"):
            written_time = os.path.getctime(f"{file_path}{f_name}")
            file_name_and_time_lst.append((f_name, written_time))
        # 생성시간 역순으로 정렬하고, 
        sorted_file_lst = sorted(file_name_and_time_lst, key=lambda x: x[1], reverse=True)
        # 가장 앞에 이는 놈을 넣어준다.
        recent_file = sorted_file_lst[0]
        recent_file_name = recent_file[0]

        return recent_file_name

if __name__ == "__main__":
    
    # Select Device 'CPU' , 'GPU'
    dev         =   torch.device("cuda:0" if (torch.cuda.is_available()) else "cpu")
    print("Device : ", dev)

    # Parameters
    params      =   Params(batch_size=128, epoch=150, 
                            shuffle_train=True, drop_last=True, num_workers=0, 
                            pin_memory=False, persistent_workers=False)
    # Pretrained model
    pretrained_model_use = False

    # DataFolder & File
    ori_path = os.getcwd()
    try: 
        data_path = "./SimulationData/"
        os.chdir(data_path)
    except:
        print("Check path! current : " + ori_path)
        sys.exit(0)
    
    dataFile = 'Train_pid test.xlsx'

    # Train & valid Data
    Datasets        =   ShieldDroneData(dataFile, dev)
    dataset_size    =   len(Datasets)
    train_size      =   int(dataset_size * 0.9)
    test_size       =   dataset_size - train_size

    trainDataset, testDataset   =   random_split(Datasets, [train_size, test_size])
    
    # Dataloader 
    train_dataloader    =   Train_Dataloader(trainDataset, params)
    test_dataloader     =   DataLoader(testDataset, batch_size=test_size, shuffle=True, drop_last=True, num_workers=0)
    
    # Network & Optimizer & loss function
    if pretrained_model_use:
        os.chdir(ori_path)
        model_path  =   './NetworkPytorch/model_pt_file/'
        model_file  =   '20231214-214547_25.pt'
        os.chdir(model_path)
        Network = DNN_Compensetor().to(dev)
        model_dict = torch.load(model_file)
        Network.load_state_dict(model_dict['model'])
    else:
        Network     =   DNN_Compensetor().to(dev)
    
    optimizer   =   optim.Adam(Network.parameters(), lr=0.0005)
    criterion   =   nn.MSELoss()
     
    # Training & Validation 
    start           =   time.time()
    train_loss_list =   []

    # Early stop setting 
    patience_limit = 20
    patience_check = 0

    for epoch in range(1, params['epoch']+1):
        print("\nepoch_start : ", epoch)
        train(model=Network, optimizer=optimizer, lossfunction=criterion, train_dataloader=train_dataloader, train_loss_list=train_loss_list, epoch=epoch)

        # Saving best trained weight
        save_condition = (train_loss_list[-1] == min(train_loss_list))
        if save_condition:
            save_model_pt(model_=Network, optimizer_=optimizer, epoch_=epoch, ori_path_=ori_path)
            print("model saved(epoch) : " + str(epoch))
            patience_check = 0
        else:
             patience_check += 1
             if patience_check > patience_limit:
                  break

    elapsed         =   (time.time() - start)
    print("Train/Valid Done!! Time : {}".format(elapsed))

    # Test & Visualization
    trueList    =   []
    predList    =   []

    with torch.no_grad():
        os.chdir(ori_path)
        model_path = './NetworkPytorch/model_pt_file/'
        model_file = recent_file_search(model_path)
        os.chdir(model_path)
        model_dict = torch.load(model_file)
        Network.load_state_dict(model_dict['model'])
        
        Test(Network, test_dataloader, criterion, trueList, predList)
    
    trueList_numpy = trueList[0].cpu().detach().numpy()
    predList_numpy = predList[0].cpu().detach().numpy()
    
    
    plt.figure(figsize=(15,15), dpi=80)
    plt.scatter(trueList_numpy[:,0], predList_numpy[:,0], label="Fz")
    plt.xlabel("True Fz")
    plt.ylabel("Pred Fz")
    plt.legend()

    plt.figure(figsize=(15,15), dpi=80)
    plt.scatter(trueList_numpy[:,1], predList_numpy[:,1], label="Mz")
    plt.xlabel("True Mz")
    plt.ylabel("Pred Mz")
    plt.legend()
    plt.show()

    
