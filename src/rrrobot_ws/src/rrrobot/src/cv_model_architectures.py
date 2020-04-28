import torch, torchvision
import numpy as np
import torch.nn.functional as F
from torch import nn


class Model_resnet(nn.Module):
    def __init__(self, freeze=False, pretrained=True):
        super(Model_resnet, self).__init__()
        if pretrained:
            self.resnet = torchvision.models.resnet34(pretrained=True)
        else:
            self.resnet = torchvision.models.resnet34(pretrained=False)
        if freeze:
            for param in self.resnet.parameters():
                param.requires_grad = False
        self.resnet.fc = torch.nn.Linear(512, 1024)
        self.addition = nn.Sequential(
            nn.ReLU(),
            nn.BatchNorm1d(1024),
            nn.Dropout(p=0.5),
            nn.Linear(1024, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(p=0.5),
            nn.Linear(512, 6))

    def forward(self, image):
        z = self.resnet(image)
        z = self.addition(z)
        return z



class Model_inception(nn.Module):
    def __init__(self, freeze=False, pretrained=True):
        super(Model_inception, self).__init__()
        if pretrained:
            self.inceptionnet = torchvision.models.inception_v3(pretrained=True)
        else:
            self.inceptionnet = torchvision.models.inception_v3(pretrained=False)
        if freeze:
            for param in self.inceptionnet.parameters():
                param.requires_grad = False
        self.inceptionnet.aux_logits = False
        self.inceptionnet.fc = torch.nn.Linear(2048, 1024)
        self.addition = nn.Sequential(
            nn.ReLU(),
            nn.BatchNorm1d(1024),
            nn.Dropout(p=0.5),
            nn.Linear(1024, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(p=0.5),
            nn.Linear(512, 6))

    def forward(self, image):
        z = self.inceptionnet(image)
        z = self.addition(z)
        return z


class Model_mobilenet(nn.Module):
    def __init__(self, freeze=False, pretrained=True):
        super(Model_mobilenet, self).__init__()
        if pretrained:
            self.mobilenet = torchvision.models.mobilenet_v2(pretrained=True)
        else:
            self.mobilenet = torchvision.models.mobilenet_v2(pretrained=False)
        if freeze:
            for param in self.mobilenet.parameters():
                param.requires_grad = False
        self.mobilenet.classifier = nn.Sequential(
            nn.Dropout(p=0.2, inplace=False),
            nn.Linear(1280, 6, bias=True))

    def forward(self, image):
        z = self.mobilenet(image)
        return z
