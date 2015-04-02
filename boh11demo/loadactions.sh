#!/bin/bash
dataPumper --target /actions:i --file data/initactions/aleft00.csv --type sequence
sleep 5
dataPumper --target /actions:i --file data/initactions/aleft01.csv --type sequence
sleep 5
dataPumper --target /actions:i --file data/initactions/aleft02.csv --type sequence
sleep 5
dataPumper --target /actions:i --file data/initactions/aleft03.csv --type sequence
sleep 5
dataPumper --target /actions:i --file data/initactions/aleft04.csv --type sequence
sleep 5
dataPumper --target /actions:i --file data/initactions/aleft05.csv --type sequence
sleep 5
dataPumper --target /actions:i --file data/initactions/aleft08.csv --type sequence
sleep 5
echo loaded actions successfully????
