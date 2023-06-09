"""
This python API is written for use with the Nordic Semiconductor's Power Profiler Kit II (PPK 2).
The PPK2 uses Serial communication.
The official nRF Connect Power Profiler was used as a reference: https://github.com/NordicSemiconductor/pc-nrfconnect-ppk
"""

import time
import serial
import struct
import logging
import os
import queue
import threading

class PPK2_Command():
    """Serial command opcodes"""
    NO_OP = 0x00
    TRIGGER_SET = 0x01
    AVG_NUM_SET = 0x02  # no-firmware
    TRIGGER_WINDOW_SET = 0x03
    TRIGGER_INTERVAL_SET = 0x04
    TRIGGER_SINGLE_SET = 0x05
    AVERAGE_START = 0x06
    AVERAGE_STOP = 0x07
    RANGE_SET = 0x08
    LCD_SET = 0x09
    TRIGGER_STOP = 0x0a
    DEVICE_RUNNING_SET = 0x0c
    REGULATOR_SET = 0x0d
    SWITCH_POINT_DOWN = 0x0e
    SWITCH_POINT_UP = 0x0f
    TRIGGER_EXT_TOGGLE = 0x11
    SET_POWER_MODE = 0x11
    RES_USER_SET = 0x12
    SPIKE_FILTERING_ON = 0x15
    SPIKE_FILTERING_OFF = 0x16
    GET_META_DATA = 0x19
    RESET = 0x20
    SET_USER_GAINS = 0x25


class PPK2_Modes():
    """PPK2 measurement modes"""
    AMPERE_MODE = "AMPERE_MODE"
    SOURCE_MODE = "SOURCE_MODE"


class PPK2_API():
    def __init__(self, port: str, **kwargs):
        '''
        port - port where PPK2 is connected
        **kwargs - keyword arguments to pass to the pySerial constructor
        '''

        self.ser = None
        self.ser = serial.Serial(port, **kwargs)
        self.ser.baudrate = 9600

        self.modifiers = {
            "Calibrated": None,
            "R": {"0": 1031.64, "1": 101.65, "2": 10.15, "3": 0.94, "4": 0.043},
            "GS": {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1},
            "GI": {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1},
            "O": {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0},
            "S": {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0},
            "I": {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0},
            "UG": {"0": 1, "1": 1, "2": 1, "3": 1, "4": 1},
            "HW": None,
            "IA": None
        }

        self.vdd_low = 800
        self.vdd_high = 5000

        self.current_vdd = None

        self.adc_mult = 1.8 / 163840

        self.MEAS_ADC = self._generate_mask(14, 0)
        self.MEAS_RANGE = self._generate_mask(3, 14)
        self.MEAS_LOGIC = self._generate_mask(8, 24)

        self.mode = None

        self.rolling_avg = None
        self.rolling_avg4 = None
        self.prev_range = None
        self.consecutive_range_samples = 0

        self.spike_filter_alpha = 0.18
        self.spike_filter_alpha5 = 0.06
        self.spike_filter_samples = 3
        self.after_spike = 0

        # adc measurement buffer remainder and len of remainder
        self.remainder = {"sequence": b'', "len": 0}

    def __del__(self):
        """Destructor"""
        try:
            if self.ser:
                self.ser.close()
        except Exception as e:
            logging.error(f"An error occured while closing ppk2_api: {e}")

    def _pack_struct(self, cmd_tuple):
        """Returns packed struct"""
        return struct.pack("B" * len(cmd_tuple), *cmd_tuple)

    def _write_serial(self, cmd_tuple):
        """Writes cmd bytes to serial"""
        try:
            cmd_packed = self._pack_struct(cmd_tuple)
            self.ser.write(cmd_packed)
        except Exception as e:
            logging.error(f"An error occured when writing to serial port: {e}")

    def _twos_comp(self, val):
        """Compute the 2's complement of int32 value"""
        if (val & (1 << (32 - 1))) != 0:
            val = val - (1 << 32)  # compute negative value
        return val

    def _convert_source_voltage(self, mV):
        """Convert input voltage to device command"""
        # minimal possible mV is 800
        if mV < self.vdd_low:
            mV = self.vdd_low

        # maximal possible mV is 5000
        if mV > self.vdd_high:
            mV = self.vdd_high

        offset = 32
        # get difference to baseline (the baseline is 800mV but the initial offset is 32)
        diff_to_baseline = mV - self.vdd_low + offset
        base_b_1 = 3
        base_b_2 = 0  # is actually 32 - compensated with above offset

        # get the number of times we have to increase the first byte of the command
        ratio = int(diff_to_baseline / 256)
        remainder = diff_to_baseline % 256  # get the remainder for byte 2

        set_b_1 = base_b_1 + ratio
        set_b_2 = base_b_2 + remainder

        return set_b_1, set_b_2

    def _read_metadata(self):
        """Read metadata"""
        # try to get metadata from device
        for _ in range(0, 5):
            # it appears the second reading is the metadata
            read = self.ser.read(self.ser.in_waiting)
            time.sleep(0.1)

            # TODO add a read_until serial read function with a timeout
            if read != b'' and "END" in read.decode("utf-8"):
                return read.decode("utf-8")

    def _parse_metadata(self, metadata):
        """Parse metadata and store it to modifiers"""
        # TODO handle more robustly
        try:
            data_split = [row.split(": ") for row in metadata.split("\n")]

            for key in self.modifiers.keys():
                for data_pair in data_split:
                    if key == data_pair[0]:
                        self.modifiers[key] = data_pair[1]
                    for ind in range(0, 5):
                        if key+str(ind) == data_pair[0]:
                            if "R" in data_pair[0]:
                                # problem on some PPK2s with wrong calibration values - this doesn't fix it
                                if float(data_pair[1]) != 0:
                                    self.modifiers[key][str(ind)] = float(
                                        data_pair[1])
                            else:
                                self.modifiers[key][str(ind)] = float(
                                    data_pair[1])
            return True
        except Exception as e:
            # if exception triggers serial port is probably not correct
            return None

    def _generate_mask(self, bits, pos):
        pos = pos
        mask = ((2**bits-1) << pos)
        mask = self._twos_comp(mask)
        return {"mask": mask, "pos": pos}

    def _get_masked_value(self, value, meas, is_bits=False):
        masked_value = (value & meas["mask"]) >> meas["pos"]
        return masked_value

    def _handle_raw_data(self, adc_value):
        """Convert raw value to analog value"""
        try:
            current_measurement_range = min(self._get_masked_value(
                adc_value, self.MEAS_RANGE), 4)  # 5 is the number of parameters
            adc_result = self._get_masked_value(adc_value, self.MEAS_ADC) * 4
            bits = self._get_masked_value(adc_value, self.MEAS_LOGIC)
            analog_value = self.get_adc_result(
                current_measurement_range, adc_result) * 10**6
            return analog_value, bits
        except Exception as e:
            print("Measurement outside of range!")
            return None, None

    @staticmethod
    def list_devices():
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if os.name == 'nt':
            devices = [port.device for port in ports if port.description.startswith("nRF Connect USB CDC ACM")]
        else:
            devices = [port.device for port in ports if port.product == 'PPK2']
        return devices

    def get_data(self):
        """Return readings of one sampling period"""
        sampling_data = self.ser.read(self.ser.in_waiting)
        return sampling_data

    def get_modifiers(self):
        """Gets and sets modifiers from device memory"""
        self._write_serial((PPK2_Command.GET_META_DATA, ))
        metadata = self._read_metadata()
        ret = self._parse_metadata(metadata)
        return ret

    def start_measuring(self):
        """Start continuous measurement"""
        if not self.current_vdd:
            if self.mode == PPK2_Modes.SOURCE_MODE:
                raise Exception("Output voltage not set!")
            if self.mode == PPK2_Modes.AMPERE_MODE:
                raise Exception("Input voltage not set!")

        self._write_serial((PPK2_Command.AVERAGE_START, ))

    def stop_measuring(self):
        """Stop continuous measurement"""
        self._write_serial((PPK2_Command.AVERAGE_STOP, ))

    def set_source_voltage(self, mV):
        """Inits device - based on observation only REGULATOR_SET is the command. 
        The other two values correspond to the voltage level.

        800mV is the lowest setting - [3,32] - the values then increase linearly
        """
        b_1, b_2 = self._convert_source_voltage(mV)
        self._write_serial((PPK2_Command.REGULATOR_SET, b_1, b_2))
        self.current_vdd = mV

    def toggle_DUT_power(self, state):
        """Toggle DUT power based on parameter"""
        if state == "ON":
            self._write_serial(
                (PPK2_Command.DEVICE_RUNNING_SET, PPK2_Command.TRIGGER_SET))  # 12,1

        if state == "OFF":
            self._write_serial(
                (PPK2_Command.DEVICE_RUNNING_SET, PPK2_Command.NO_OP))  # 12,0

    def use_ampere_meter(self):
        """Configure device to use ampere meter"""
        self.mode = PPK2_Modes.AMPERE_MODE
        self._write_serial((PPK2_Command.SET_POWER_MODE,
                            PPK2_Command.TRIGGER_SET))  # 17,1

    def use_source_meter(self):
        """Configure device to use source meter"""
        self.mode = PPK2_Modes.SOURCE_MODE
        self._write_serial((PPK2_Command.SET_POWER_MODE,
                            PPK2_Command.AVG_NUM_SET))  # 17,2

    def get_adc_result(self, current_range, adc_value):
        """Get result of adc conversion"""
        current_range = str(current_range)
        result_without_gain = (adc_value - self.modifiers["O"][current_range]) * (
            self.adc_mult / self.modifiers["R"][current_range])
        adc = self.modifiers["UG"][current_range] * (result_without_gain * (self.modifiers["GS"][current_range] * result_without_gain + self.modifiers["GI"][current_range]) + (
            self.modifiers["S"][current_range] * (self.current_vdd / 1000) + self.modifiers["I"][current_range]))

        prev_rolling_avg = self.rolling_avg
        prev_rolling_avg4 = self.rolling_avg4

        # spike filtering / rolling average
        if self.rolling_avg is None:
            self.rolling_avg = adc
        else:
            self.rolling_avg = self.spike_filter_alpha * adc + (1 - self.spike_filter_alpha) * self.rolling_avg
        
        if self.rolling_avg4 is None:
            self.rolling_avg4 = adc
        else:
            self.rolling_avg4 = self.spike_filter_alpha5 * adc + (1 - self.spike_filter_alpha5) * self.rolling_avg4

        if self.prev_range is None:
            self.prev_range = current_range

        if self.prev_range != current_range or self.after_spike > 0:
            if self.prev_range != current_range:
                self.consecutive_range_samples = 0
                self.after_spike = self.spike_filter_samples
            else:
                self.consecutive_range_samples += 1

            if current_range == "4":
                if self.consecutive_range_samples < 2:
                    self.rolling_avg = prev_rolling_avg
                    self.rolling_avg4 = prev_rolling_avg4
                adc = self.rolling_avg4
            else:
                adc = self.rolling_avg
            
            self.after_spike -= 1

        self.prev_range = current_range
        return adc

    def _digital_to_analog(self, adc_value):
        """Convert discrete value to analog value"""
        return int.from_bytes(adc_value, byteorder="little", signed=False)  # convert reading to analog value

    def digital_channels(self, bits):
        """
        Convert raw digital data to digital channels.

        Returns a 2d matrix with 8 rows (one for each channel). Each row contains HIGH and LOW values for the selected channel.
        """

        # Prepare 2d matrix with 8 rows (one for each channel)
        digital_channels = [[], [], [], [], [], [], [], []]
        for sample in bits:
            digital_channels[0].append((sample & 1) >> 0)
            digital_channels[1].append((sample & 2) >> 1)
            digital_channels[2].append((sample & 4) >> 2)
            digital_channels[3].append((sample & 8) >> 3)
            digital_channels[4].append((sample & 16) >> 4)
            digital_channels[5].append((sample & 32) >> 5)
            digital_channels[6].append((sample & 64) >> 6)
            digital_channels[7].append((sample & 128) >> 7)
        return digital_channels

    def get_samples(self, buf):
        """
        Returns list of samples read in one sampling period.
        The number of sampled values depends on the delay between serial reads.
        Manipulation of samples is left to the user.
        See example for more info.
        """

        sample_size = 4  # one analog value is 4 bytes in size
        offset = self.remainder["len"]
        samples = []
        raw_digital_output = []

        first_reading = (
            self.remainder["sequence"] + buf[0:sample_size-offset])[:4]
        adc_val = self._digital_to_analog(first_reading)
        measurement, bits = self._handle_raw_data(adc_val)
        if measurement is not None:
            samples.append(measurement)
        if bits is not None:
            raw_digital_output.append(bits)

        offset = sample_size - offset

        while offset <= len(buf) - sample_size:
            next_val = buf[offset:offset + sample_size]
            offset += sample_size
            adc_val = self._digital_to_analog(next_val)
            measurement, bits = self._handle_raw_data(adc_val)
            if measurement is not None:
                samples.append(measurement)
            if bits is not None:
                raw_digital_output.append(bits)

        self.remainder["sequence"] = buf[offset:len(buf)]
        self.remainder["len"] = len(buf)-offset

        # return list of samples and raw digital outputs
        # handle those lists in PPK2 API wrapper
        return samples, raw_digital_output  


class PPK_Fetch(threading.Thread):
    '''
    Background process for polling the data in multi-threaded variant
    '''
    def __init__(self, ppk2, quit_evt, buffer_len_s=10, buffer_chunk_s=0.5):
        super().__init__()
        self._ppk2 = ppk2
        self._quit = quit_evt

        self.print_stats = False
        self._stats = (None, None)
        self._last_timestamp = 0

        self._buffer_max_len = int(buffer_len_s * 100000 * 4)    # 100k 4-byte samples per second
        self._buffer_chunk = int(buffer_chunk_s * 100000 * 4)    # put in the queue in chunks of 0.5s

        # round buffers to a whole sample
        if self._buffer_max_len % 4 != 0:
            self._buffer_max_len = (self._buffer_max_len // 4) * 4
        if self._buffer_chunk % 4 != 0:
            self._buffer_chunk = (self._buffer_chunk // 4) * 4

        self._buffer_q = queue.Queue()

    def run(self):
        s = 0
        t = time.time()
        local_buffer = b''
        while not self._quit.is_set():
            d = PPK2_API.get_data(self._ppk2)
            tm_now = time.time()
            local_buffer += d
            while len(local_buffer) >= self._buffer_chunk:
                # FIXME: check if lock might be needed when discarding old data
                self._buffer_q.put(local_buffer[:self._buffer_chunk])
                while self._buffer_q.qsize()>self._buffer_max_len/self._buffer_chunk:
                    self._buffer_q.get()
                local_buffer = local_buffer[self._buffer_chunk:]
                self._last_timestamp = tm_now

            # calculate stats
            s += len(d)
            dt = tm_now - t
            if dt >= 0.1:
                if self.print_stats:
                    print(f"Samples: {s}, delta time: {dt}")
                self._stats = (s, dt)
                s = 0
                t = tm_now

            time.sleep(0.0001)

        # process would hang on join() if there's data in the buffer after the measurement is done
        while True:
            try:
                self._buffer_q.get(block=False)
            except queue.Empty:
                break

    def get_data(self):
        ret = b''
        count = 0
        while True:
            try:
                ret += self._buffer_q.get(timeout=0.001) # get_nowait sometimes skips a chunk for some reason
                count += 1
            except queue.Empty:
                break
        return ret


class PPK2_MP(PPK2_API):
    '''
    Multiprocessing variant of the object. The interface is the same as for the regular one except it spawns
    a background process on start_measuring()
    '''
    def __init__(self, port, buffer_max_size_seconds=10, buffer_chunk_seconds=0.1, **kwargs):
        '''
        port - port where PPK2 is connected
        buffer_max_size_seconds - how many seconds of data to keep in the buffer
        buffer_chunk_seconds - how many seconds of data to put in the queue at once
        **kwargs - keyword arguments to pass to the pySerial constructor
        '''
        super().__init__(port, **kwargs)

        self._fetcher = None
        self._quit_evt = threading.Event()
        self._buffer_max_size_seconds = buffer_max_size_seconds
        self._buffer_chunk_seconds = buffer_chunk_seconds

    def __del__(self):
        """Destructor"""
        PPK2_API.stop_measuring(self)
        self._quit_evt.clear()
        self._quit_evt = None
        del self._quit_evt
        if self._fetcher is not None:
            self._fetcher.join()
        self._fetcher = None
        del self._fetcher

    def start_measuring(self):
        # discard the data in the buffer
        self.stop_measuring()
        while self.get_data()!=b'':
            pass

        PPK2_API.start_measuring(self)
        self._quit_evt.clear()
        if self._fetcher is not None:
            return
        
        self._fetcher = PPK_Fetch(self, self._quit_evt, self._buffer_max_size_seconds, self._buffer_chunk_seconds)
        self._fetcher.start()

    def stop_measuring(self):
        PPK2_API.stop_measuring(self)
        self.get_data() # flush the serial buffer (to prevent unicode error on next command)
        self._quit_evt.set()
        if self._fetcher is not None:
            self._fetcher.join() # join() will block if the queue isn't empty
            self._fetcher = None

    def get_data(self):
        try:
            return self._fetcher.get_data()
        except (TypeError, AttributeError):
            return b''
