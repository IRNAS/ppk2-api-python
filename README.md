## Description
The new Nordic Semiconductor's [Power Profiler Kit II (PPK 2)](https://www.nordicsemi.com/Software-and-tools/Development-Tools/Power-Profiler-Kit-2) is very useful for real time measurement of device power consumption. The official [nRF Connect Power Profiler tool](https://github.com/NordicSemiconductor/pc-nrfconnect-ppk) provides a friendly GUI with real-time data display. However there is no support for automated power monitoring. The puropose of this Python API is to enable automated power monitoring and data logging in Python applications.

![Power Profiler Kit II](https://github.com/IRNAS/ppk2-api-python/blob/main/images/power-profiler-kit-II.jpg)

## Features
The main features of the PPK2 Python API (will) include:
* All nRF Connect Power Profiler GUI functionality - In progress
* Data logging to user selectable format - In progress
* Cross-platform support

## Requirements
Unlike the original Power Profiler Kit, the PPK2 uses Serial to communicate with the computer. No additional modules are required.

## Usage
At this point in time the library provides the basic API with a basic example showing how to read data and toggle DUT power.

To enable power monitoring in Ampere mode implement the following sequence:
```
ppk2_test = PPK2_API("/dev/ttyACM3")  # serial port will be different for you
ppk2_test.get_modifiers()
ppk2_test.use_ampere_meter()  # set ampere meter mode
ppk2_test.start_measuring()  # start measuring

# read measured values in a for loop like this:
for i in range(0, 1000):
    read_data = ppk2_test.get_data()
    if read_data != b'':
        ppk2_test.average_of_sampling_period(read_data)
    time.sleep(0.01)
```
