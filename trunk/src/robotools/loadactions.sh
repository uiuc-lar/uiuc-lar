#!/bin/bash

./dataPumper --target /actions:i --file gromble00.csv
sleep 5
./dataPumper --target /actions:i --file gromble01.csv 
sleep 5
./dataPumper --target /actions:i --file gromble02.csv 
sleep 5
./dataPumper --target /actions:i --file gromble03.csv 
echo loaded actions successfully????
