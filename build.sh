#!/bin/bash
sudo rm -r build && mkdir build && cd build && cmake .. && make -j$(nproc)&&cp robotSim* ..
