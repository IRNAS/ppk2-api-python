# ----------------------------------------------------------------------
# PPK2 API INTEGRATION (Adapter Classes)
# ----------------------------------------------------------------------

# --- API LOADING LOGIC ---
try:
	# Attempt to import the real API components
	from ppk2_api.ppk2_api import PPK2_API, PPK2_Command, PPK2_Modes

	API_AVAILABLE = True
	API_MODE = "REAL"
	print("INFO: PPK2 API found. Attempting REAL mode.")
except ImportError:
	API_AVAILABLE = False
	API_MODE = "MOCK"
	print("WARNING: PPK2 API not found. Using MOCK mode for simulation.")

# MOCK DEFINITION for PPK2_API (to allow code compilation in MOCK mode)


class PPK2_API:
	def __init__(self, port=None): pass

	def use_source_meter(self): pass

	# PPK2 API works in mV, so set_source_voltage accepts mV
	def set_source_voltage(self, voltage_mv): pass

	def start_acquisition(self): pass

	def stop_acquisition(self): pass

	def get_samples(self):
		# API returns current in mA (PPK2 standard)
		return {'current': [1.0], 'voltage': [3300]}

	def close(self): pass

	@staticmethod
	def list_devices(): return []  # Mock for auto-detection


import time
import numpy as np
# Assuming utils.py is available and contains these functions
from utils import get_temperature, check_limits


class PPK2SMU:
	def __init__(self, port1=None, port2=None):

		# Connect PPK1
		if API_AVAILABLE or API_MODE == "MOCK":
			self.ppk1 = PPK2_API(port=port1)
			self.ppk1.use_source_meter()
		else:
			raise EnvironmentError("PPK2 API is not available.")

		# Connect PPK2 (if port is provided)
		self.ppk2 = None
		if port2:
			if API_AVAILABLE or API_MODE == "MOCK":
				self.ppk2 = PPK2_API(port=port2)
				self.ppk2.use_source_meter()
			else:
				raise EnvironmentError("PPK2 API is not available for PPK2.")

	def set_voltage_ppk1(self, voltage_v):
		# Convert Volts (V) to millivolts (mV), expected by the API
		self.ppk1.set_source_voltage(voltage_v * 1000)

	def set_voltage_ppk2(self, voltage_v):
		if self.ppk2:
			# Convert Volts (V) to millivolts (mV)
			self.ppk2.set_source_voltage(voltage_v * 1000)

	def start_measurement(self):
		# Measurement is typically handled inside the sweep loop
		pass

	def stop_measurement(self):
		pass

	def get_current_ppk1(self):
		# API returns data in mA. Convert to A (Amperes)
		data = self.ppk1.get_samples()
		return (data['current'][0] / 1000) if data and data['current'] else 0.0

	def get_current_ppk2(self):
		if not self.ppk2:
			return 0.0
		# API returns data in mA. Convert to A (Amperes)
		data = self.ppk2.get_samples()
		return (data['current'][0] / 1000) if data and data['current'] else 0.0

	def close(self):
		self.ppk1.close()
		if self.ppk2:
			self.ppk2.close()

	# UPDATED FUNCTION: SWEEP (Added current measurement logging)
	def sweep(self, v1_start, v1_stop, v1_steps, v2_start, v2_stop, v2_steps, delay):
		"""
		Performs an I-V sweep test by gradually changing V1 and optionally V2 voltage.

		Returns: (v1_data, i1_data, v2_data, i2_data, temps)
		"""
		v1_data, i1_data, v2_data_out, i2_data_out, temps = [], [], [], [], []

		if v1_steps <= 0:
			print("WARNING: V1 steps must be greater than 0.")
			return v1_data, i1_data, v2_data_out, i2_data_out, temps

		# Creating voltage vectors for PPK1 sweep
		v1_range = np.linspace(v1_start, v1_stop, v1_steps)
		# Creating voltage vectors for PPK2 sweep
		v2_range = np.linspace(v2_start, v2_stop, v2_steps)
		# Use only the first V2 point if sweep has only 1 step (constant voltage)
		v2_iter = v2_range if len(v2_range) > 1 else [v2_range[0]]

		total_steps = len(v1_range) * len(v2_iter)
		current_step = 0

		# Main sweep loop (for PPK1)
		for v1 in v1_range:
			self.set_voltage_ppk1(v1)

			# Inner sweep loop (for PPK2)
			for v2 in v2_iter:
				current_step += 1

				if self.ppk2:
					self.set_voltage_ppk2(v2)

				# Wait for stabilization
				time.sleep(delay)

				# Current measurement
				i1 = self.get_current_ppk1()  # In Amperes
				i2 = self.get_current_ppk2()  # In Amperes

				# LOGGING MEASUREMENT INFORMATION (Displaying current in mA)
				log_msg = f"STEP {current_step}/{total_steps} | PPK1: V={v1:.4f}V, I={i1 * 1000:.3f}mA"
				if self.ppk2:
					log_msg += f" | PPK2: V={v2:.4f}V, I={i2 * 1000:.3f}mA"
				print(log_msg)

				# Save data
				v1_data.append(v1)
				i1_data.append(i1)
				v2_data_out.append(v2)
				i2_data_out.append(i2)

				# Temperature measurement
				temps.append(get_temperature() or 25.0)  # Default to 25.0 if no sensor

		# Set voltage to 0 after test completion
		self.set_voltage_ppk1(0)
		if self.ppk2:
			self.set_voltage_ppk2(0)

		print("------------------------------")
		print("I-V Test Finished.")

		return v1_data, i1_data, v2_data_out, i2_data_out, temps