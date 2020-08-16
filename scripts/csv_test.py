#!/usr/bin/env python
#-*- coding:utf-8 -*-

import csv


def main():
    with open('something.csv', "w", newline="") as somefile:
        writer = csv.writer(somefile, dialect='excel')
        header = ['Pose', 'Soll', 'Ist']
        writer.writerow(header)

if __name__=='__main__':
    main()