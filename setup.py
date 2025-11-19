import setuptools

# Use the README.md as the long description
with open("README.md", "r", encoding="utf-8") as fh:
	long_description = fh.read()

# Define the core dependencies (pyserial is essential for PPK2 via UART)
REQUIRED_PACKAGES = [
	'pyserial>=3.4',
	# Add any other core dependencies here if needed (e.g., numpy for data processing)
]

setuptools.setup(
	# --- Metadata ---
	name="ppk2-api",
	version="0.1.0",  # Start with a reasonable version number
	author="RolandWa",  # Replace with your name/alias
	description="Unofficial Python API for Nordic Semiconductor Power Profiling Kit 2 (PPK2).",
	long_description=long_description,
	long_description_content_type="text/markdown",
	url="https://github.com/RolandWa/ppk2-api-python",  # Replace with your correct GitHub URL
	license="GPL-2.0-only",  # Explicitly state the license

	# --- Package & Source Files ---
	# Finds all packages in the 'src' directory
	packages=setuptools.find_packages(where="src"),
	package_dir={"": "src"},

	# --- Dependencies and Compatibility ---
	install_requires=REQUIRED_PACKAGES,
	python_requires='>=3.6',  # Define minimum required Python version

	# --- Classifiers (For PyPI) ---
	classifiers=[
		"Programming Language :: Python :: 3",
		"Programming Language :: Python :: 3.6",
		"Programming Language :: Python :: 3.7",
		"Programming Language :: Python :: 3.8",
		"Programming Language :: Python :: 3.9",
		"License :: OSI Approved :: GNU General Public License v2 only (GPLv2 only)",
		"Operating System :: OS Independent",
		"Topic :: Scientific/Engineering :: Electronic"
	],
)