import xml.etree.ElementTree as ET
import re

# Charger le fichier SVG
tree = ET.parse('line.svg')
root = tree.getroot()

# Namespace pour SVG
namespace = {'svg': 'http://www.w3.org/2000/svg'}

# Fonction pour extraire les coordonnées du chemin
def extraire_points_path(d):
    points = []
    # Extraction des coordonnées en utilisant des expressions régulières
    commands = re.findall(r'[MLZmlz]\s*[\d\s\.-]*', d)
    current_x, current_y = 0, 0

    for command in commands:
        cmd_type = command[0]
        coords = re.findall(r'-?\d+\.?\d*', command)
        coords = list(map(float, coords))

        if cmd_type.lower() == 'm':  # Move to (relative or absolute)
            current_x, current_y = coords[:2]
            points.append((current_x, current_y))
        
        elif cmd_type.lower() == 'l':  # Line to (relative or absolute)
            for i in range(0, len(coords), 2):
                dx, dy = coords[i:i+2]
                if cmd_type.islower():  # Si la commande est relative
                    current_x += dx
                    current_y += dy
                else:  # Absolue
                    current_x, current_y = dx, dy
                points.append((current_x, current_y))
        
        elif cmd_type.lower() == 'z':  # Close path
            points.append(points[0])  # Ferme le chemin en revenant au point initial

    return points

# Trouver le chemin dans le fichier SVG
path_element = root.find('.//svg:path', namespace)
if path_element is not None:
    d_attribute = path_element.get('d')
    points = extraire_points_path(d_attribute)
    print("Coordonnées extraites :", points)
else:
    print("Aucun chemin trouvé.")

