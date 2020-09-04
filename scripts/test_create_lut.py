#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy

def main():
    lut = dict()
    lut['x{}y{}'.format(1,1)] = dict()
    print lut

if __name__ == "__main__":
    main()