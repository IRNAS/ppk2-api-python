## Description
The new Nordic Semiconductor's [Power Profiler Kit II (PPK 2)](https://www.nordicsemi.com/Software-and-tools/Development-Tools/Power-Profiler-Kit-2) is very useful for real time measurement of device power consumption. The official [nRF Connect Power Profiler tool](https://github.com/NordicSemiconductor/pc-nrfconnect-ppk) provides a friendly GUI with real-time data display. However there is no support for automated power monitoring. The puropose of this Python API is to enable automated power monitoring and data logging in Python applications.

![Power Profiler Kit II](https://github.com/IRNAS/ppk2-api-python/blob/master/images/power-profiler-kit-II.jpg)

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


## Licensing
pp2-api-python is licensed under [GPL V2 license](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html).

What this means is that you can use this hardware and documentation without paying a royalty and knowing that you'll be able to use your version forever. You are also free to make changes but if you share these changes then you have to do so on the same conditions that you enjoy.

IRNAS is name and mark of Institute IRNAS. You may use these name and terms only to attribute the appropriate entity as required by the Open Licence referred to above. You may not use them in any other way and in particular you may not use them to imply endorsement or authorization of any hardware that you design, make or sell.
