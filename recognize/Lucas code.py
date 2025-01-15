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
with open(r"C:/Users/balog/OneDrive - Queen's University/2024-2025/MREN 318/Project/recognize/Dictionary.json", 'r') as f:
    Dictionary = json.load(f)

model_name='resnet'
feature_extract=True
device= torch.device("cpu")

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


filename="C:/Users/balog/OneDrive - Queen's University/2024-2025/MREN 318/Project/recognize/model_checkpoint.pth"

checkpoint=torch.load(filename, map_location=torch.device('cpu'))
best_acc=checkpoint['best_acc']
model_pt.load_state_dict(checkpoint['state_dict'])
#do same transforms with input image
data_transform = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

image_path = "C:/Users/balog/OneDrive - Queen's University/2024-2025/MREN 318/Project/recognize/" + input("Image: ")
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



# import paho.mqtt.publish as publish
# MQTT_SERVER = "localhost"  #Write Server IP Address

import pyserial

ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)


publish.single("breed", predicted_breed, port=5051, hostname=MQTT_SERVER)

import time
import logging
from time import strftime
from arduino_iot_cloud import ArduinoCloudClient
from arduino_iot_cloud import Location
from arduino_iot_cloud import Schedule
from arduino_iot_cloud import ColoredLight
from arduino_iot_cloud import Task
from random import uniform
import argparse
import ssl # noqa
# Switch callback, toggles the LED.
def on_switch_changed(client, value):
    # Note the client object passed to this function can be used to access
    # and modify any registered cloud object. The following line updates
    # the LED value.
    client["led"] = value

# 1. Create a client object, which is used to connect to the IoT cloud and link local
# objects to cloud objects. Note a username and password can be used for basic authentication
# on both CPython and MicroPython. For more advanced authentication methods, please see the examples.

parser = argparse.ArgumentParser(description="arduino_iot_cloud.py")
parser.add_argument("-d", "--debug", action="store_true", help="Enable debugging messages")
parser.add_argument("-s", "--sync", action="store_true", help="Run in synchronous mode")
args = parser.parse_args()

    # Assume the host has an active Internet connection.

    # Configure the logger.
    # All message equal or higher to the logger level are printed.
    # To see more debugging messages, pass --debug on the command line.
logging.basicConfig(
    datefmt="%H:%M:%S",
    format="%(asctime)s.%(msecs)03d %(message)s",
    level=logging.DEBUG if args.debug else logging.INFO,
    )

    # Create a client object to connect to the Arduino IoT cloud.
    # The most basic authentication method uses a username and password. The username is the device
    # ID, and the password is the secret key obtained from the IoT cloud when provisioning a device.


client = ArduinoCloudClient(device_id="24aa9972-4346-4713-aa40-911c27e2e508", username="24aa9972-4346-4713-aa40-911c27e2e508", password="@E2#C3!PdyCkhewZ7RybVxYaM", sync_mode=args.sync)
print("connected")
# 2. Register cloud objects.
# Note: The following objects must be created first in the dashboard and linked to the device.
# When the switch is toggled from the dashboard, the on_switch_changed function is called with
# the client object and new value args.
#client.register("sw1", value=None, on_write=on_switch_changed)

# The LED object is updated in the switch's on_write callback.
client.register("isCat")  
client["isCat"] = True
client.register("isCat", value=None)
    
client.start()