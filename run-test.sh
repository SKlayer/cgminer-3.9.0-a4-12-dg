#!/bin/bash
sudo ./cgminer -o stratum+tcp://192.168.0.201:3333 -u inno -p x --A1Pll1 1200 --A1Pll2 1200 --A1Pll3 1200 --A1Pll4 1200 --A1Pll5 1200 --A1Pll6 1200 --diff 8 --api-listen --api-network --cs 8 --stmcu 0 --hwreset --no-submit-stale --lowmem
