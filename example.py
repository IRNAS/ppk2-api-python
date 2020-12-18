
"""
Basic usage of PPK2 Python API.
The basic ampere mode sequence is:
1. read modifiers
2. set ampere mode
3. read stream of data
"""
import time
from src.ppk2_api import PPK2_API

ppk2s_connected = PPK2_API.list_devices()
if(len(ppk2s_connected) == 1):
    ppk2_port = ppk2s_connected[0]
    print(f'Found PPK2 at {ppk2_port}')
else:
    print(f'Too many connected PPK2\'s: {ppk2s_connected}')
    exit()

ppk2_test = PPK2_API(ppk2_port)
ppk2_test.get_modifiers()
ppk2_test.use_ampere_meter()  # set ampere meter mode
ppk2_test.toggle_DUT_power("OFF")  # disable DUT power

ppk2_test.start_measuring()  # start measuring
# measurements are a constant stream of bytes
# the number of measurements in one sampling period depends on the wait between serial reads
# it appears the maximum number of bytes received is 1024
# the sampling rate of the PPK2 is 100 samples per millisecond
for i in range(0, 1000):
    read_data = ppk2_test.get_data()
    if read_data != b'':
        samples = ppk2_test.get_samples(read_data)
        print(f"Average of {len(samples)} samples is: {sum(samples)/len(samples)}uA")
    time.sleep(0.01)

ppk2_test.toggle_DUT_power("ON")

ppk2_test.start_measuring()
for i in range(0, 1000):
    read_data = ppk2_test.get_data()
    if read_data != b'':
        samples = ppk2_test.get_samples(read_data)
        print(f"Average of {len(samples)} samples is: {sum(samples)/len(samples)}uA")
    time.sleep(0.001)  # lower time between sampling -> less samples read in one sampling period

ppk2_test.stop_measuring()