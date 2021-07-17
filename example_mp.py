
"""
Basic usage of PPK2 Python API - multiprocessing version
The basic ampere mode sequence is:
1. read modifiers
2. set ampere mode
3. read stream of data
"""
import time
from ppk2_api.ppk2_api.ppk2_api import PPK2_MP

ppk2s_connected = PPK2_MP.list_devices()
if(len(ppk2s_connected) == 1):
    ppk2_port = ppk2s_connected[0]
    print(f'Found PPK2 at {ppk2_port}')
else:
    print(f'Too many connected PPK2\'s: {ppk2s_connected}')
    exit()

ppk2_test = PPK2_MP(ppk2_port)
ppk2_test.get_modifiers()
ppk2_test.use_ampere_meter()  # set ampere meter mode
ppk2_test.toggle_DUT_power("OFF")  # disable DUT power

ppk2_test.start_measuring()  # start measuring

# measurements are a constant stream of bytes
# multiprocessing variant starts a process in the background which constantly
# polls the device in order to prevent losing samples. It will buffer the
# last 10s (by default) of data so get_data() can be called less frequently.
for i in range(0, 10):
    read_data = ppk2_test.get_data()
    if read_data != b'':
        samples = ppk2_test.get_samples(read_data)
        print(f"Average of {len(samples)} samples is: {sum(samples)/len(samples)}uA")
    time.sleep(0.5)

ppk2_test.toggle_DUT_power("ON")

ppk2_test.start_measuring()
for i in range(0, 10):
    read_data = ppk2_test.get_data()
    if read_data != b'':
        samples = ppk2_test.get_samples(read_data)
        print(f"Average of {len(samples)} samples is: {sum(samples)/len(samples)}uA")
    time.sleep(0.5)  # lower time between sampling -> less samples read in one sampling period

ppk2_test.stop_measuring()

