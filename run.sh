#!/bin/bash
#sleep 6
sudo killall cgminer
sudo killall cgminer
sudo killall cgminer
sudo killall cgminer

sudo ./cgminer -o stratum+tcp://stratum.f2pool.com:8888 -u testapi.20 -p 4321 -o / -u / -p / -o / -u / -p / --A1Pll1 1200 --A1Pll2 1200 --A1Pll3 1200 --A1Pll4 1200 --A1Pll5 1200 --A1Pll6 1200 --A1Vol 820  --diff 8 --api-listen --api-network --cs 8 --stmcu 0
