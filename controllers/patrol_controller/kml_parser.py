from pathlib import Path
import xml.etree.ElementTree as ET

def load_kml(filename, project_path):
    kml_path = Path(project_path) / "kml" / filename
    print("Loading:", kml_path)

    tree = ET.parse(kml_path)
    root = tree.getroot()

    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    latlon_points = []

    for coord_tag in root.findall('.//kml:coordinates', ns):
        text = coord_tag.text.strip()
        lines = text.split()

        for line in lines:
            parts = line.split(',')
            if len(parts) >= 2:
                lon = float(parts[0])
                lat = float(parts[1])
                latlon_points.append((lat, lon))

    return latlon_points

if __name__ == "__main__":
    pts = load_kml("patrol_area.kml", ".")
    print("\nCoordinates found:")
    for p in pts:
        print(p)