import time
import threading

from src.ppk2_api.ppk2_api import PPK2_API


class PowerProfiler:
    conversion_factor = 1_000_000  # uA to mW

    def __init__(self, voltage_mv=3700):
        self.voltage_mv = voltage_mv
        self.total_power_mW = 0
        self.total_samples = 0

        self.lock = threading.Lock()
        self.ppk2 = None
        self.sampling_thread = None
        self.sampling_enabled = False


    def _setup_ppk2(self):
        ppk2s_connected = PPK2_API.list_devices()

        # Check if we have at least one PPK2 device
        if not ppk2s_connected:
            raise ConnectionError("No PPK2 devices found!")

        print(ppk2s_connected)
        # Just select the first available PPK2 device
        for ppk2_port_tuple in ppk2s_connected:
            ppk2_port = ppk2_port_tuple[0] #Just get the port part of the tuple
            print(f"Connecting to {ppk2_port}")
            
            self.ppk2 = PPK2_API(ppk2_port, timeout=1, write_timeout=1, exclusive=True)

            ret = self.ppk2.get_modifiers()
            if ret is not None:
                break

            print(f"Failed to connect to {ppk2_port}")

        self.ppk2.set_source_voltage(self.voltage_mv)
        self.ppk2.use_source_meter()
        self.ppk2.toggle_DUT_power("ON")

        self.ppk2.start_measuring()
        print("Initialized Power Profiler")
        

    def _run_sampling(self):
        try:
            self._setup_ppk2()
            while self.sampling_enabled:
                time.sleep(0.01)
                read_data = self.ppk2.get_data()

                if read_data == b"":
                    continue

                samples, raw_digital = self.ppk2.get_samples(read_data)
                if not samples:
                    continue

                average_current_uA = sum(samples) / len(samples)
                average_power_mW = (average_current_uA * self.voltage_mv) / self.conversion_factor
                formatted_power = round(average_power_mW, 2)

                with self.lock:
                    self.total_power_mW += formatted_power
                    self.total_samples += 1
                    average_of_averages_mW = self.total_power_mW / self.total_samples

                print(f"{formatted_power} mW, Avg: {average_of_averages_mW:.2f} mW")

        except Exception as e:
            self.sampling_enabled = False
            print(f"An error occurred: {e}")

    def start_sampling(self):
        self.sampling_enabled = True
        self.sampling_thread = threading.Thread(target=self._run_sampling, daemon=True)
        self.sampling_thread.start()

    def stop_sampling(self):
        self.sampling_enabled = False
        self.sampling_thread.join()


def main():
    sampler = PowerProfiler(voltage_mv=3800)
    sampler.start_sampling()
    input("Press Enter to exit...\n")
    sampler.stop_sampling()


if __name__ == "__main__":
    main()
