#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

def main():
    lut = dict()
    lut['x{}y{}'.format(1,1)] = dict()
    lut['x1y1'] = {}
    print lut

if __name__ == "__main__":
    main()