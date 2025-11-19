import os
import yaml


def load_components(config_dir="components"):
	# 1. Uzyskaj katalog, w którym znajduje się bieżący skrypt (config_loader.py), czyli 'src'.
	base_dir = os.path.dirname(os.path.abspath(__file__))

	# 2. Przejdź do katalogu nadrzędnego (..) i dołącz 'components'
	# To przechodzi z '\...src\' do '\...\' i dodaje 'components', dając ścieżkę do:
	# C:\Users\RWache\...\smu-ppk2-gui\components
	full_config_dir = os.path.join(base_dir, '..', config_dir)

	# Upewnij się, że pełna ścieżka jest znormalizowana
	full_config_dir = os.path.normpath(full_config_dir)

	components = {}

	# 3. Weryfikacja i ładowanie
	if not os.path.isdir(full_config_dir):
		print(f"WARNING: Configuration directory not found at {full_config_dir}. Returning empty components.")
		return components

	for file in os.listdir(full_config_dir):
		if file.endswith(".yaml") or file.endswith(".yml"):
			path = os.path.join(full_config_dir, file)
			with open(path, "r", encoding="utf-8") as f:
				name = file.rsplit(".", 1)[0]
				try:
					components[name] = yaml.safe_load(f)
				except yaml.YAMLError as e:
					print(f"Error loading YAML file {file}: {e}")

	return components