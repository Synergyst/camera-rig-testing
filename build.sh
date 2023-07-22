#!/bin/bash

rm v4l2-testing.o
aarch64-linux-gnu-g++-12 -std=gnu++20 `pkg-config --cflags libv4l2` -g -Wall -Wformat -O2 -march=native -ftree-vectorize -flax-vector-conversions -fopenmp -c -o v4l2-testing.o v4l2-testing.cpp && aarch64-linux-gnu-g++-12 -o /usr/local/bin/v4l2-testing v4l2-testing.o -std=gnu++20 `pkg-config --cflags libv4l2` -g -Wall -Wformat -O2 -march=native -ftree-vectorize -flax-vector-conversions -fopenmp `pkg-config --libs libv4l2` -lm -ldl
#v4l2-testing /dev/video0 1280 512 yuyv422 /dev/video2 640 512 gray8
