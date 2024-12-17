
"""
Basic usage of PPK2 Python API - multiprocessing version.
The basic ampere mode sequence is:
1. read modifiers
2. set ampere mode
3. read stream of data
"""
import time
from ppk2_api.ppk2_api import PPK2_MP as PPK2_API

ppk2s_connected = PPK2_API.list_devices()
if len(ppk2s_connected) == 1:
    ppk2_port = ppk2s_connected[0][0]
    ppk2_serial = ppk2s_connected[0][1]
    print(f"Found PPK2 at {ppk2_port} with serial number {ppk2_serial}")
else:
    print(f"Too many connected PPK2's: {ppk2s_connected}")
    exit()

ppk2_test = PPK2_API(ppk2_port, buffer_max_size_seconds=1, buffer_chunk_seconds=0.01, timeout=1, write_timeout=1, exclusive=True)
ppk2_test.get_modifiers()
ppk2_test.set_source_voltage(3300)

"""
Source mode example
"""
ppk2_test.use_source_meter()  # set source meter mode
ppk2_test.toggle_DUT_power("ON")  # enable DUT power

ppk2_test.start_measuring()  # start measuring
# measurements are a constant stream of bytes
# the number of measurements in one sampling period depends on the wait between serial reads
# it appears the maximum number of bytes received is 1024
# the sampling rate of the PPK2 is 100 samples per millisecond
while True:
    read_data = ppk2_test.get_data()
    if read_data != b'':
        samples, raw_digital = ppk2_test.get_samples(read_data)
        print(f"Average of {len(samples)} samples is: {sum(samples)/len(samples)}uA")

        # Raw digital contains the raw digital data from the PPK2
        # The number of raw samples is equal to the number of samples in the samples list
        # We have to process the raw digital data to get the actual digital data
        digital_channels = ppk2_test.digital_channels(raw_digital)
        for ch in digital_channels:
            # Print last 10 values of each channel
            print(ch[-10:])
        print()

    time.sleep(0.001)

ppk2_test.toggle_DUT_power("OFF")  # disable DUT power
ppk2_test.stop_measuring()

"""
Ampere mode example
"""
ppk2_test.use_ampere_meter()  # set ampere meter mode

ppk2_test.start_measuring()
while True:
    read_data = ppk2_test.get_data()
    if read_data != b'':
        samples, raw_digital = ppk2_test.get_samples(read_data)
        print(f"Average of {len(samples)} samples is: {sum(samples)/len(samples)}uA")

        # Raw digital contains the raw digital data from the PPK2
        # The number of raw samples is equal to the number of samples in the samples list
        # We have to process the raw digital data to get the actual digital data
        digital_channels = ppk2_test.digital_channels(raw_digital)
        for ch in digital_channels:
            # Print last 10 values of each channel
            print(ch[-10:])
        print()
    time.sleep(0.001)  # lower time between sampling -> less samples read in one sampling period

ppk2_test.stop_measuring()
