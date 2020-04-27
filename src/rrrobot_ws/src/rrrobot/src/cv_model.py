#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import os, torch, torchvision
import numpy as np
from torch import nn
from PIL import Image


class Model_mobilenet(nn.Module):
    def __init__(self):
        super(Model_mobilenet, self).__init__()
        self.mobilenet = torchvision.models.mobilenet_v2(pretrained=True)

        self.mobilenet.classifier = nn.Sequential(
            nn.Dropout(p=0.2, inplace=False),
            nn.Linear(1280, 6, bias=True))

    def forward(self, image):
        z = self.mobilenet(image)
        return z


def call_back(filename):
    # prepare input file
    filename = filename.data
    input_image = Image.open(filename)
    if input_image.size[0] < input_image.size[1]:
            input_image = input_image.transpose(Image.ROTATE_90)
    input_image = input_image.resize((512, 384))
    # print(input_image.size)
    input_image = np.moveaxis(np.asarray(input_image), 2, 0).astype(np.float32)
    input_tensor = torch.from_numpy(input_image)
    input_tensor = torch.unsqueeze(input_tensor, dim=0)
    # print(input_tensor.shape)

    # query model
    predicted_logits = model(input_tensor).squeeze()
    # print(predicted_logits.shape)
    predicted_label = torch.argmax(predicted_logits).detach().numpy().tolist()

    type_dict = {0: 'cardboard',
                 1: 'glass'    ,
                 2: 'metal'    ,
                 3: 'paper'    ,
                 4: 'plastic'  ,
                 5: 'trash'    }

    print('type: ', type_dict[predicted_label])

    # publish a message, name of this node is 'cv_model'
    pub = rospy.Publisher('/cv_model', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    garbage_type = type_dict[predicted_label]
    rospy.loginfo(filename + ':' + garbage_type)
    pub.publish(filename + ':' + garbage_type)
    # rate.sleep()


def listener():
    # rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/current_image', String, call_back)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cv_node')

    # prepare pretrained model
    model = Model_mobilenet()
    model.load_state_dict(torch.load('pytorch_pretrain_model.pt', map_location='cpu'))
    model = model.to('cpu')
    model.eval()

    # listen to another node
    listener()
