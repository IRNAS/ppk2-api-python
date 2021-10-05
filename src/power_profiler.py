import time
import csv
import datetime
from threading import Thread
# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib
from ppk2_api.ppk2_api import PPK2_MP as PPK2_API

class PowerProfiler():
    def __init__(self, serial_port=None, source_voltage_mV=3300, filename=None):
        """Initialize PPK2 power profiler with serial"""
        self.measuring = None
        self.measurement_thread = None
        self.ppk2 = None

        print(f"Initing power profiler")

        # try:
        if serial_port:
            self.ppk2 = PPK2_API(serial_port)
        else:
            serial_port = self.discover_port()
            print(f"Opening serial port: {serial_port}")
            if serial_port:
                self.ppk2 = PPK2_API(serial_port)

        try:
            ret = self.ppk2.get_modifiers()  # try to read modifiers, if it fails serial port is probably not correct
            print(f"Initialized ppk2 api: {ret}")
        except Exception as e:
            print(f"Error initializing power profiler: {e}")
            ret = None
            raise e

        if not ret:
            self.ppk2 = None
            raise Exception(f"Error when initing PowerProfiler with serial port {serial_port}")
        else:
            self.ppk2.use_source_meter()

            self.source_voltage_mV = source_voltage_mV

            self.ppk2.set_source_voltage(self.source_voltage_mV)  # set to 3.3V

            print(f"Set power profiler source voltage: {self.source_voltage_mV}")

            self.measuring = False
            self.current_measurements = []

            # local variables used to calculate power consumption
            self.measurement_start_time = None
            self.measurement_stop_time = None

            time.sleep(1)

            self.stop = False

            self.measurement_thread = Thread(target=self.measurement_loop, daemon=True)
            self.measurement_thread.start()

            # write to csv
            self.filename = filename
            if self.filename is not None:
                with open(self.filename, 'w', newline='') as file:
                    writer = csv.writer(file)
                    row = []
                    for key in ["ts", "avg1000"]:
                        row.append(key)
                    writer.writerow(row)

    def write_csv_rows(self, samples):
        """Write csv row"""
        with open(self.filename, 'a', newline='') as file:
            writer = csv.writer(file)
            for sample in samples:
                row = [datetime.datetime.now().strftime('%d-%m-%Y %H:%M:%S.%f'), sample]
                writer.writerow(row)

    def delete_power_profiler(self):
        """Join thread"""
        self.measuring = False
        self.stop = True

        print("Deleting power profiler")

        if self.measurement_thread:
            print(f"Joining measurement thread")
            self.measurement_thread.join()
            self.measurement_thread = None

        if self.ppk2:
            print(f"Disabling ppk2 power")
            self.disable_power()
            del self.ppk2

        print(f"Deleted power profiler")

    def discover_port(self):
        """Discovers ppk2 serial port"""
        ppk2s_connected = PPK2_API.list_devices()
        if(len(ppk2s_connected) == 1):
            ppk2_port = ppk2s_connected[0]
            print(f'Found PPK2 at {ppk2_port}')
            return ppk2_port
        else:
            print(f'Too many connected PPK2\'s: {ppk2s_connected}')
            return None

    def enable_power(self):
        """Enable ppk2 power"""
        if self.ppk2:
            self.ppk2.toggle_DUT_power("ON")
            return True
        return False

    def disable_power(self):
        """Disable ppk2 power"""
        if self.ppk2:
            self.ppk2.toggle_DUT_power("OFF")
            return True
        return False

    def measurement_loop(self):
        """Endless measurement loop will run in a thread"""
        while True and not self.stop:
            if self.measuring:  # read data if currently measuring
                read_data = self.ppk2.get_data()
                if read_data != b'':
                    samples = self.ppk2.get_samples(read_data)
                    self.current_measurements += samples  # can easily sum lists, will append individual data
            time.sleep(0.001)  # TODO figure out correct sleep duration

    def _average_samples(self, list, window_size):
        """Average samples based on window size"""
        chunks = [list[val:val + window_size] for val in range(0, len(list), window_size)]
        avgs = []
        for chunk in chunks:
            avgs.append(sum(chunk) / len(chunk))

        return avgs

    def start_measuring(self):
        """Start measuring"""
        if not self.measuring:  # toggle measuring flag only if currently not measuring
            self.current_measurements = []  # reset current measurements
            self.measuring = True  # set internal flag
            self.ppk2.start_measuring()  # send command to ppk2
            self.measurement_start_time = time.time()

    def stop_measuring(self):
        """Stop measuring and return average of period"""
        self.measurement_stop_time = time.time()
        self.measuring = False
        self.ppk2.stop_measuring()  # send command to ppk2

        #samples_average = self._average_samples(self.current_measurements, 1000)
        if self.filename is not None:
            self.write_csv_rows(self.current_measurements)

    def get_min_current_mA(self):
        return min(self.current_measurements) / 1000

    def get_max_current_mA(self):
        return max(self.current_measurements) / 1000

    def get_num_measurements(self):
        return len(self.current_measurements)

    def get_average_current_mA(self):
        """Returns average current of last measurement in mA"""
        if len(self.current_measurements) == 0:
            return 0

        average_current_mA = (sum(self.current_measurements) / len(self.current_measurements)) / 1000 # measurements are in microamperes, divide by 1000
        return average_current_mA

    def get_average_power_consumption_mWh(self):
        """Return average power consumption of last measurement in mWh"""
        average_current_mA = self.get_average_current_mA()
        average_power_mW = (self.source_voltage_mV / 1000) * average_current_mA  # divide by 1000 as source voltage is in millivolts - this gives us milliwatts
        measurement_duration_h = self.get_measurement_duration_s() / 3600  # duration in seconds, divide by 3600 to get hours
        average_consumption_mWh = average_power_mW * measurement_duration_h
        return average_consumption_mWh

    def get_average_charge_mC(self):
        """Returns average charge in milli coulomb"""
        average_current_mA = self.get_average_current_mA()
        measurement_duration_s = self.get_measurement_duration_s()  # in seconds
        return average_current_mA * measurement_duration_s

    def get_measurement_duration_s(self):
        """Returns duration of measurement"""
        measurement_duration_s = (self.measurement_stop_time - self.measurement_start_time)  # measurement duration in seconds
        return measurement_duration_s