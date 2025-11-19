from PIL import Image, ImageTk
import os
import io

# --- OBSŁUGA SVG: Wymaga instalacji cairosvg ---
try:
	import cairosvg

	SVG_AVAILABLE = True
except ImportError:
	SVG_AVAILABLE = False


def load_schema_image(comp_name, use_second_ppk=False, max_width=None, max_height=None):
	"""
	Loads the connection diagram image (SVG, PNG, or JPG) for the selected component,
	automatically scaling it to fit the provided dimensions.

	Args:
		comp_name (str): The name of the component (e.g., 'resistor').
		use_second_ppk (bool): Flag indicating if the two-PPK2 schema should be used.
		max_width (int, optional): Maximum width for scaling.
		max_height (int, optional): Maximum height for scaling.

	Returns:
		ImageTk.PhotoImage: The loaded and scaled image object, or None if loading fails.
	"""
	folder = "schemas"
	suffix = "_ppk2" if use_second_ppk else "_ppk1"
	base_filename = f"{comp_name}{suffix}"

	# Definicja kolejności poszukiwania plików: SVG -> PNG -> JPG
	# Ścieżka do folderu 'schemas' powinna być obliczana względem położenia schemas_handler.py
	schemas_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', folder)

	path = None
	img = None

	print(base_filename)
	print(schemas_dir)

	# 1. Próba wczytania SVG
	if SVG_AVAILABLE:
		svg_path = os.path.join(schemas_dir, f"{base_filename}.svg")
		if os.path.exists(svg_path):
			try:
				# Rasteryzacja SVG do bytow PNG w pamięci
				# Użycie max_width/max_height w cairosvg dla lepszej jakości początkowej rasteryzacji
				png_bytes = cairosvg.svg2png(url=svg_path, output_width=max_width, output_height=max_height)
				img = Image.open(io.BytesIO(png_bytes))
				path = svg_path
			except Exception as e:
				print(f"Błąd rasteryzacji SVG {svg_path}: {e}")

	# 2. Próba wczytania PNG/JPG (tylko jeśli SVG nie działało lub nie było dostępne)
	if img is None:
		for ext in ['.png', '.jpg']:
			temp_path = os.path.join(schemas_dir, f"{base_filename}{ext}")
			if os.path.exists(temp_path):
				path = temp_path
				break

		# Wczytywanie z pliku PNG/JPG
		if path:
			try:
				img = Image.open(path)
			except Exception as e:
				print(f"Błąd podczas wczytywania obrazu {path}: {e}")
				return None

	if img is None:
		return None

	# 3. Skalowanie obrazu (jeśli jest potrzebne i podano wymiary)
	if max_width and max_height:
		original_width, original_height = img.size
		width_ratio = max_width / original_width
		height_ratio = max_height / original_height

		# Użycie mniejszego współczynnika, aby obraz zmieścił się w całości
		scale_factor = min(width_ratio, height_ratio)

		# Skalowanie w dół, lub lekkie skalowanie w górę (do 110%)
		if scale_factor < 1.0 or scale_factor < 1.1:
			new_width = int(original_width * scale_factor)
			new_height = int(original_height * scale_factor)
			# Użycie Image.Resampling.LANCZOS dla wysokiej jakości
			img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)

	# 4. Konwersja do formatu Tkinter i zwrócenie
	try:
		return ImageTk.PhotoImage(img)
	except Exception as e:
		print(f"Błąd konwersji obrazu do formatu Tkinter: {e}")
		return None