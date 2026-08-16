#!/bin/bash

cython3 -3 --embed -o xycar_motor.c xycar_motor.py
gcc -Os -I /usr/include/python3.8 xycar_motor.c -lpython3.8 -o xycar_motor
