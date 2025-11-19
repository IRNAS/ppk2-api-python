import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from config_loader import load_components
from smu_interface import PPK2SMU, API_AVAILABLE
from utils import calculate_power, check_limits, get_temperature
from schemas_handler import load_schema_image
from spice_exporter import export_to_spice
import csv
from datetime import datetime
import numpy as np
import time
import os  # Import os for path manipulation

# Conditional import of PPK2_API if available
PPK2_API = None
if API_AVAILABLE:
	try:
		from ppk2_api.ppk2_api import PPK2_API
	except ImportError:
		pass


class SMUApp:
	def __init__(self, root):
		self.root = root
		self.root.title("SMU Emulator – Nordic PPK2 Pro")
		self.version = "1.1.0"
		self.components = load_components()

		self.use_second_ppk = tk.BooleanVar(value=False)
		self.port1_var = tk.StringVar()
		self.port2_var = tk.StringVar()

		# New: Serial number variables for display
		self.serial1_var = tk.StringVar(value="N/A")
		self.serial2_var = tk.StringVar(value="N/A")

		# New: Storage for detected device info (Port -> Serial)
		self.device_info = {}

		self.smu = None
		self.current_comp = None
		self.last_data = None

		self.build_menu()
		self.build_wizard()

	def build_menu(self):
		menubar = tk.Menu(self.root)
		self.root.config(menu=menubar)

		file_menu = tk.Menu(menubar, tearoff=0)
		menubar.add_cascade(label="File", menu=file_menu)
		# Changed menu item to reflect combined action
		file_menu.add_command(label="Save Results (CSV & SPICE)", command=self.save_all_results)
		file_menu.add_separator()
		file_menu.add_command(label="Exit", command=self.root.quit)

		info_menu = tk.Menu(menubar, tearoff=0)
		menubar.add_cascade(label="Help", menu=info_menu)
		info_menu.add_command(label="About", command=lambda: messagebox.showinfo("About",
		                                                                         f"v{self.version}\nNordic PPK2 SMU Emulator"))
		info_menu.add_command(label="Check for updates", command=self.check_update)
		info_menu.add_command(label="Update", command=self.do_update)
		info_menu.add_command(label="Reinstall", command=self.do_reinstall)

	def check_update(self):
		pass

	def do_update(self):
		pass

	def do_reinstall(self):
		pass

	def build_wizard(self):
		nb = ttk.Notebook(self.root)
		nb.pack(fill='both', expand=True)

		# Step 1 – Configuration
		f1 = ttk.Frame(nb, padding=20)

		# Component Selection
		ttk.Label(f1, text="Component:").grid(row=0, column=0, sticky='w', pady=5)
		self.comp_var = tk.StringVar()
		cb = ttk.Combobox(f1, textvariable=self.comp_var, values=list(self.components.keys()), width=40)
		cb.grid(row=0, column=1, columnspan=3, pady=5, sticky='ew')
		cb.bind("<<ComboboxSelected>>", self.on_component_selected)

		# PPK2 Configuration
		ttk.Checkbutton(f1, text="Use second PPK2 (e.g. for transistor tests)",
		                variable=self.use_second_ppk, command=self.on_component_selected).grid(row=1, column=0,
		                                                                                       columnspan=4, pady=10,
		                                                                                       sticky='w')

		# Port 1
		ttk.Label(f1, text="Port PPK2 #1:").grid(row=2, column=0, sticky='e')
		ttk.Entry(f1, textvariable=self.port1_var).grid(row=2, column=1, pady=2, sticky='ew')
		# Display Serial 1
		ttk.Label(f1, text="S/N:").grid(row=2, column=2, sticky='w', padx=5)
		self.sn1_label = ttk.Label(f1, textvariable=self.serial1_var)
		self.sn1_label.grid(row=2, column=3, pady=2, sticky='w')

		# Port 2
		ttk.Label(f1, text="Port PPK2 #2:").grid(row=3, column=0, sticky='e')
		ttk.Entry(f1, textvariable=self.port2_var).grid(row=3, column=1, pady=2, sticky='ew')
		# Display Serial 2
		ttk.Label(f1, text="S/N:").grid(row=3, column=2, sticky='w', padx=5)
		self.sn2_label = ttk.Label(f1, textvariable=self.serial2_var)
		self.sn2_label.grid(row=3, column=3, pady=2, sticky='w')

		# Auto-detection Button
		detect_button = ttk.Button(
			f1,
			text="Auto-Detect PPK2",
			command=self.detect_ppk2_ports
		)
		detect_button.grid(row=4, column=0, columnspan=4, pady=10)

		# Separator
		ttk.Separator(f1, orient='horizontal').grid(row=5, column=0, columnspan=4, sticky='ew', pady=10)

		# Parameter Display Section
		ttk.Label(f1, text="Loaded Parameters:").grid(row=6, column=0, sticky='w', columnspan=4)
		self.params_text = tk.Text(f1, height=8, width=50, wrap='word', state='disabled')
		self.params_text.grid(row=7, column=0, columnspan=4, sticky='ew')

		# Adjust grid for the frame
		f1.grid_columnconfigure(1, weight=1)

		nb.add(f1, text="1. Configuration")

		# Step 2 – Connection Diagram
		self.schema_frame = ttk.Frame(nb, padding=20)
		self.schema_label = ttk.Label(self.schema_frame, text="Select component → diagram will appear")
		self.schema_label.pack()
		nb.add(self.schema_frame, text="2. Connection Diagram")

		# Step 3 – Settings & Start
		f3 = ttk.Frame(nb, padding=20)
		self.temp_label = ttk.Label(f3, text="Temp: -- °C")
		self.temp_label.grid(row=0, columnspan=2, pady=10)
		self.root.after(500, self.update_temp)

		ttk.Button(f3, text="Start full test", command=self.full_test).grid(row=1, columnspan=2, pady=20)
		nb.add(f3, text="3. Test and Run")

		# Step 4 – Plot
		fig_frame = ttk.Frame(nb)
		self.fig, self.ax = plt.subplots(figsize=(8, 6))
		self.canvas = FigureCanvasTkAgg(self.fig, fig_frame)
		self.canvas.get_tk_widget().pack(fill='both', expand=True)
		nb.add(fig_frame, text="4. Results Plot")

	def detect_ppk2_ports(self):
		"""
		Automatically detects connected PPK2 devices, sets port variables,
		stores device info, and displays serial numbers.
		"""
		if not API_AVAILABLE or PPK2_API is None:
			messagebox.showwarning("Warning",
			                       "PPK2 API is not available (MOCK mode or missing API). Cannot detect ports.")
			return

		try:
			# list_devices() returns a list of tuples: [(port, serial_number), ...]
			devices = PPK2_API.list_devices()
			# Store device info (Port -> Serial) for later lookups
			self.device_info = {port: sn for port, sn in devices}

			# Reset port/serial fields
			self.port1_var.set("")
			self.port2_var.set("")
			self.serial1_var.set("N/A")
			self.serial2_var.set("N/A")

			if not devices:
				messagebox.showinfo("Port Detection", "No connected PPK2 devices found.")
				return

			info_msg = f"Found {len(devices)} PPK2 devices:\n\n"

			# Filling ports and updating serials
			for i, (port, serial_num) in enumerate(devices):
				info_msg += f"Device #{i + 1}: Port={port}, S/N={serial_num}\n"
				if i == 0:
					self.port1_var.set(port)
					self.serial1_var.set(serial_num)
				elif i == 1:
					self.port2_var.set(port)
					self.serial2_var.set(serial_num)

			# Clear remaining port field if only one device is found
			if len(devices) < 2:
				self.port2_var.set("")

			messagebox.showinfo("Port Detection", info_msg + "\n\nPorts set automatically in detection order.")

		except Exception as e:
			messagebox.showerror("API Error", f"Error during PPK2 device detection: {e}")

	def update_temp(self):
		t = get_temperature()
		self.temp_label.config(text=f"Temp: {t:.1f} °C" if t else "Temp: no sensor")
		self.root.after(500, self.update_temp)

	def on_component_selected(self, event=None):
		name = self.comp_var.get()
		if name:
			self.current_comp = self.components[name]

			# 1. Update Connection Diagram
			component_data = self.current_comp
			# Default to name if 'type' is missing in YAML, then convert to lowercase for file naming
			component_type = component_data.get('type', name)

			img = load_schema_image(component_type, self.use_second_ppk.get())
			if img:
				self.schema_label.configure(image=img)
				self.schema_label.image = img

			# 2. Display Loaded Parameters
			params_display = ""
			for key, value in self.current_comp.items():
				# Format list/tuple values nicely
				if isinstance(value, list) or isinstance(value, tuple):
					value = str(value).strip('[]()')
				params_display += f"{key}: {value}\n"

			self.params_text.config(state='normal')
			self.params_text.delete('1.0', tk.END)
			self.params_text.insert('1.0', params_display)
			self.params_text.config(state='disabled')

			# 3. Update serial number display if a port is set and device info is available
			port1 = self.port1_var.get()
			port2 = self.port2_var.get()

			# Check if port1 is in device_info and update display
			if port1 and port1 in self.device_info:
				self.serial1_var.set(self.device_info[port1])
			elif not port1:
				self.serial1_var.set("N/A")

			# Check if port2 is in device_info
			if port2 and port2 in self.device_info:
				self.serial2_var.set(self.device_info[port2])
			elif not port2:
				self.serial2_var.set("N/A")

	def full_test(self):
		if not self.current_comp:
			messagebox.showerror("Error", "Select a component first")
			return

		# 1. Get and display loaded configuration parameters
		params = self.current_comp
		sweep1 = params.get('sweep_v1', [0, 3.3, 50])
		sweep2 = params.get('sweep_v2', [0, 0, 1]) if self.use_second_ppk.get() else [0, 0, 1]
		delay = params.get('delay', 0.1)
		log_scale = params.get('log_scale', False)

		test_info = f"""--- Starting I-V Test ---
Component: {self.comp_var.get()}
PPK1 Sweep V: {sweep1[0]}V to {sweep1[1]}V ({int(sweep1[2])} steps)
PPK2 Sweep V: {sweep2[0]}V to {sweep2[1]}V ({int(sweep2[2])} steps)
Delay between points: {delay} s
Logarithmic Y Scale: {'Yes' if log_scale else 'No'}
------------------------------"""
		print(test_info)

		# 2. SMU Initialization and test execution
		if self.smu:
			self.smu.close()

		port1 = self.port1_var.get() or None
		port2 = self.port2_var.get() or None if self.use_second_ppk.get() else None

		self.smu = PPK2SMU(port1, port2)

		v1_data, i1_data, v2_data, i2_data, temps = self.smu.sweep(
			sweep1[0], sweep1[1], int(sweep1[2]),
			sweep2[0], sweep2[1], int(sweep2[2]),
			delay
		)

		# 3. Process and display results
		self.last_data = {
			'v1': v1_data, 'i1': i1_data,
			'v2': v2_data, 'i2': i2_data,
			'temp': temps, 'comp': self.comp_var.get()
		}

		self.ax.clear()
		style = 'b-o' if not log_scale else 'b-'
		self.ax.plot(v1_data, i1_data, style, label="PPK2 #1")
		if self.use_second_ppk.get() and any(i2_data):
			self.ax.plot(v2_data, i2_data, 'r-o', label="PPK2 #2")
		self.ax.legend()
		self.ax.set_xlabel('Voltage [V]')
		self.ax.set_ylabel('Current [A]')
		self.ax.set_title(f"Test: {self.comp_var.get()}")
		self.canvas.draw()

		messagebox.showinfo("Finished", "Measurement complete – use File menu to save/export")

	def save_all_results(self):
		"""
		Prompts user for a save location and exports both CSV and SPICE model
		using a consistent filename base.
		"""
		if not self.last_data:
			messagebox.showwarning("Warning", "No data available to save.")
			return

		component_name = self.comp_var.get()
		timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
		# Use component name and timestamp for default filename
		default_name = f"{component_name}_{timestamp}"

		# Prompt user for save path using the CSV extension as the primary output format
		save_path = filedialog.asksaveasfilename(
			defaultextension=".csv",
			initialfile=default_name,
			filetypes=[("CSV and SPICE Files", "*.csv"), ("All Files", "*.*")]
		)

		if not save_path:
			return

		# Determine the base filename and directory
		save_dir = os.path.dirname(save_path)
		base_name = os.path.splitext(os.path.basename(save_path))[0]

		# Save CSV
		csv_path = os.path.join(save_dir, f"{base_name}.csv")
		try:
			with open(csv_path, 'w', newline='') as f:
				writer = csv.writer(f)
				writer.writerow(['V1 [V]', 'I1 [A]', 'V2 [V]', 'I2 [A]', 'Temp [°C]', 'Power [mW]'])
				for i in range(len(self.last_data['v1'])):
					v1 = self.last_data['v1'][i]
					i1 = self.last_data['i1'][i]
					v2 = self.last_data['v2'][i] if i < len(self.last_data['v2']) else 0
					i2 = self.last_data['i2'][i] if i < len(self.last_data['i2']) else 0
					t = self.last_data['temp'][i]
					p = calculate_power(v1, i1) * 1000
					writer.writerow([f"{v1:.4f}", f"{i1:.6f}", f"{v2:.4f}", f"{i2:.6f}", f"{t:.1f}", f"{p:.1f}"])

			# Export SPICE Model
			lib_path = os.path.join(save_dir, f"{base_name}.lib")
			export_to_spice(component_name, self.last_data['v1'], self.last_data['i1'], lib_path)

			messagebox.showinfo("Saved", f"Results saved successfully:\n- CSV: {csv_path}\n- SPICE: {lib_path}")

		except Exception as e:
			messagebox.showerror("Save Error", f"An error occurred while saving: {e}")


def main():
	root = tk.Tk()
	app = SMUApp(root)
	root.geometry("1200x800")
	root.mainloop()


if __name__ == "__main__":
	main()