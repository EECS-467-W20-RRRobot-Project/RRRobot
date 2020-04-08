#!/bin/bash

catkin_make clean &&
catkin_make &&
catkin_make install &&

source devel/setup.bash
