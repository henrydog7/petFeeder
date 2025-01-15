import numpy as np
import os
import torch
from torch import nn
import torch.optim as optim
import torch.utils
import torchvision
from torchvision import transforms, models, datasets
from PIL import Image
import time
import copy
import json

#copied from training code
with open(r"C:\Users\kingp\Documents\classify\Dictionary.json", 'r') as f:
    Dictionary = json.load(f)

model_name='resnet'
feature_extract=True
device= torch.device("cuda:0")

def set_param_grad (model, feature_extracting): #avoid change parameters of pretrained model
    if feature_extracting:
        for param in model.parameters():
            param.requires_grad = False


def init_model ( num_classes, feature_extract, use_pretrained=True):
    model_pt=None
    input_size=0

    model_pt = models.resnet152(pretrained=use_pretrained)# download model to checkpoints file
    set_param_grad(model_pt, feature_extract)
    num_ftrs=model_pt.fc.in_features
    model_pt.fc = nn.Sequential(nn.Linear(num_ftrs,num_classes),nn.LogSoftmax(dim=1))

    input_size=224
    return model_pt, input_size

#load in trained model
model_pt, input_size=init_model(37,feature_extract, use_pretrained=True)
model_pt=model_pt.to(device)

filename="C:\\Users\\kingp\\Documents\\classify\\model_checkpoint.pth"

checkpoint=torch.load(filename)
best_acc=checkpoint['best_acc']
model_pt.load_state_dict(checkpoint['state_dict'])
#do same transforms with input image
data_transform = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

image_path = #path of image to classify
image = Image.open(image_path)
image = data_transform(image)

model_pt.eval()
image = image.unsqueeze(0)# model expects batches

image = image.to(device)
model_pt = model_pt.to(device)

with torch.no_grad(): 
    outputs = model_pt(image)
    _, preds = torch.max(outputs, 1)

predicted_breed = Dictionary[str(preds.item()+1)]
print(f"Predicted breed: {predicted_breed}")