
import time
import machine
import pycom

from lopy_max31856 import *

pycom.heartbeat(False) # turn blue LED off
time.sleep(2)

# K_type thermocouple, 8 samples averaged
tc = MAX31856(avgsel=0x03,tc_type=MAX31856_K_TYPE)

while True:
    try:
        temp=tc.read_temp_c()
        tempi=tc.read_internal_temp_c()
        print('hot junction: ',temp, '   cold junction: ', tempi)
    except MAX31856Error as e:
        print('ERROR: ', e)

    time.sleep(5)
