#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import os, torch, torchvision
import numpy as np
from torch import nn
from PIL import Image


class Model_v1(nn.Module):
    def __init__(self):
        super(Model_v1, self).__init__()
        self.resnet = torchvision.models.resnet34(pretrained=True)
        self.resnet.fc = torch.nn.Linear(512, 512)
        self.addition = nn.Sequential(
            nn.BatchNorm1d(512),
            nn.Dropout(p=0.5),
            nn.Linear(512, 6))

    def forward(self, image):
        z = self.resnet(image)
        z = self.addition(z)
        return z


def call_back(filename):
    # prepare input file
    input_image = Image.open(filename)
    input_image = input_image.resize((512, 384))
    # print(input_image.size)
    input_image = np.moveaxis(np.asarray(input_image), 2, 0).astype(np.float32)
    input_tensor = torch.from_numpy(input_image)
    input_tensor = torch.unsqueeze(input_tensor, dim=0)
    # print(input_tensor.shape)

    # query model
    predicted_logits = model(input_tensor).squeeze()
    # print(predicted_logits.shape)
    predicted_label = torch.argmax(predicted_logits).numpy().tolist()

    type_dict = {0: 'cardboard',
                 1: 'glass'    ,
                 2: 'metal'    ,
                 3: 'paper'    ,
                 4: 'plastic'  ,
                 5: 'trash'    }

    print('type: ', type_dict[predicted_label])

    # publish a message, name of this node is 'cv_model'
    ​pub = rospy.Publisher('cv_model', String, queue_size=10)
​    ​rospy.init_node('talker', anonymous=True)
​    ​rate = rospy.Rate(10) # 10hz
    
    ​garbage_type = type_dict[predicted_label]
    ​rospy.loginfo(filename + ':' + garbage_type)
    ​pub.publish(filename + ':' + garbage_type)
    ​rate.sleep()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ariac/logical_camera_1', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    # prepare pretrained model
    model = Model_v1()
    model.load_state_dict(torch.load('v1_0.001.pt', map_location='cpu'))
    model = model.to('cpu')
    model.eval()

    # listen to another node
    listener()
