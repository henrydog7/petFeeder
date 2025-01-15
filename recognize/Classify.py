import numpy as np
import os
import torch
from torch import nn
import torch.optim as optim
import torch.utils
import torchvision
from torchvision import transforms, models, datasets
import imageio
import time
import warnings
import random
import sys
import copy
import json

#import the dataset
if(os.path.exists("C:\\Users\\balog\\Documents\\classify\\dataset")):
    data_path= "C:\\Users\\balog\\Documents\\classify\\dataset"
elif (os.path.exists("C:\\Users\\kingp\\Documents\\classify\\dataset")):
    data_path= "C:\\Users\\balog\\kingp\\classify\\dataset"
      
train_path = data_path + '/train'
valid_path = data_path + '/valid'

#Data Preprocessing
#Data augmentation, for more data

data_transform = {
    'train' : transforms. Compose([transforms.RandomRotation(45), 
                                   transforms.RandomCrop(224), #resize data
                                   transforms.RandomHorizontalFlip(p=0.5), # 5050 change of flipping
                                   transforms.RandomVerticalFlip(p=0.5),
                                   transforms.ColorJitter(brightness=0.2,contrast=0.1, saturation= 0.1, hue=0.1),
                                   transforms.RandomGrayscale(p=0.025),
                                   transforms.ToTensor(), #Covert to tensor
                                   transforms.Normalize([0.485,0.456,0.406],[0.229,0.224,0.225])]), #Normalize values from ImageNet
    'valid': transforms. Compose([transforms.Resize(256),
                                  transforms.ToTensor(),#Covert to tensor
                                  transforms.Normalize([0.485,0.456,0.406],[0.229,0.224,0.225])]),
}

batch_size= 8

dataset = { x: datasets.ImageFolder (os.path.join(data_path, x), data_transform[x] ) for x in ['train','valid']} #Create dataset
dataloader = { x: torch.utils.data.DataLoader(dataset[x], batch_size = batch_size, shuffle=True) for x in ['train','valid'] } #laod in data with Dataloader function from torch
dataset_size = { x: len(dataset[x] )for x in ['train','valid'] }
classname = dataset['train'].classes

#Key value pairs from .json file
with open('Dictionary.json','r') as f:
    Dictionary = json.load(f)
#function to reverse image from tensor back to numpy array, and  also reverse changes in data augmentation step 
def tensor_to_numpy (tensor):
    image=tensor.to("cpu").clone().detach()
    image=image.numpy().squeeze()
    image=image.transpose(1,2,0)
    image=image*np.array((0.229,0.224,0.225)+np.array(0.485,0.456,0.406))
    image=image.clip(0,1)
    return image

#Load in classification model from torchvision 
model_name='resnet'
feature_extract=True
if torch.cuda.is_available():
    device=torch.device("cuda:0")
else:
    device=torch.device("cpu")

def set_param_grad (model, feature_extracting): #avoid change parameters of pretrained model
    if feature_extracting:
        for param in model.parameters():
            param.requires_grad = False

model_pt=models.resnet152() #select resnet152

#use example code from torchvision to load in pretrained model
def init_model ( num_classes, feature_extract, use_pretrained=True):
    model_pt=None
    input_size=0

    model_pt = models.resnet152(pretrained=use_pretrained)# download model to checkpoints file
    set_param_grad(model_pt, feature_extract)
    num_ftrs=model_pt.fc.in_features
    model_pt.fc = nn.Sequential(nn.Linear(num_ftrs,num_classes),nn.LogSoftmax(dim=1))

    input_size=224
    return model_pt, input_size

model_pt, input_size=init_model(37,feature_extract, use_pretrained=True)
model_pt=model_pt.to(device) #use gpu to train model
if(os.path.exists("C:\\Users\\balog\\.cache\\torch\\hub\\checkpoints")):
    filename= "C:\\Users\\balog\\.cache\\torch\\hub\\checkpoints"
elif (os.path.exists("C:\\Users\\kingp\\.cache\\torch\\hub\\checkpoints")):
    filename= "C:\\Users\\balog\\kingp\\.cache\\torch\\hub\\checkpoints"


params_update=model_pt.parameters()
#avoid changing all parameters of the pretrained model, only change weight and bias parameters of specific classifying task
params_update=[]
for name,param in model_pt.named_parameters():
    if param.requires_grad==True:
        params_update.append(param)

optimizer=optim.Adam(params_update, lr=1e-3) # set learning rate to start at 0.001
decay= optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.1) #learning rate decay
loss_fn= nn.NLLLoss() #log_softmax function applied in init function

def train (model,dataloader, loss_fn, optimizer, decay, file_name, num_epochs):
    best_acc=0 
    start=time.time()

    model.to(device) # load model to gpu
    val_acc_history=[] #array to save old accuracy record
    valid_losses=[]
    train_acc_history=[] 
    train_losses=[]
    LR=[optimizer.param_groups[0]['lr']]
    best_model=copy.deepcopy(model.state_dict())

    for epoch in range(num_epochs):
        print('Epoch {}/{}'.format(epoch, num_epochs-1))
        print('-'*10)
        
        for phase in ['train','vaild']: # training and validation 
            if phase =='train':
                model.train()
            else:
                model.eval()
            
            running_loss = 0.0
            running_corrects=0

            for inputs, labels in dataloader[phase].dataset:
                inputs=inputs.to(device) #load to gpu
                labels=labels.to(device)

                optimizer.zero_grad() #zero the parameter gradients

                with torch.set_grad_enabled(phase=='train'):
                    outputs = model(inputs)
                    loss = loss_fn(outputs, labels)
                    _, preds = torch.max(outputs,1)

                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                running_loss += loss.item() * inputs.size(0) #calculate losses
                running_corrects += torch.sum(preds == labels.data)

            epoch_loss = running_loss/len(dataloader[phase].dataset)
            epoch_acc = running_corrects.double() / len(dataloader[phase].dataset)

            time_elapsed=time.time()-start  #calculate used time
            print ('time elapsed {:.0f}m {:.0f}s'.format(time_elapsed//60,time_elapsed%60))
            print ('{} Loss: {:.4f} Acc:{:.4f}'.format(phase, epoch_loss, epoch_acc))

            if phase == 'valid' and epoch_acc > best_acc: #update best accuracy
                best_acc = epoch_acc
                best_model = copy.deepcopy(model.state_dict())
                state = {
                    'state_dict': model.state_dict(),
                    'best_acc':best_acc,
                    'optimizer':optimizer.state_dict(),
                }
                torch.save(state, file_name)
            if phase =='train':   #record to history
                train_acc_history.append(epoch_acc)
                train_losses.append(epoch_loss)
        
            if phase =='valid':
                val_acc_history.append(epoch_acc)
                valid_losses.append(epoch_loss)
                decay.step(epoch_loss)

        print('Optimizer learning rate : {:.7f}'.format(optimizer.param_groups[0]['lr']))
        LR.append(optimizer.params_groups[0]['lr'])
        print()
    time_elapsed=time.time()-start #calculate used time
    print ('Training time: {:.0f}m {:.0f}s'.format(time_elapsed//60,time_elapsed%60))
    print ('Best val Acc: {:.4f} Acc:{:.4f}'.format(best_acc))

    model.load_state_dict(best_model) #load best result
    return model, val_acc_history, train_acc_history, valid_losses, train_losses, LR

model_pt, val_acc_history, train_acc_history, valid_losses, train_losses, LR=train(model_pt,dataloader,loss_fn,optimizer,decay,filename,50)
