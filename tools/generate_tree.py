#!/usr/bin/env python3
"""
Script de generation de l'arborescence du projet pour ARCHITECTURE.md
Usage: python generate_tree.py
"""

import os
from pathlib import Path

def get_file_description(filepath):
    """
    Retourne une description du fichier basee sur son extension et son nom
    """
    descriptions = {
        # Extensions
        '.ino': 'Sketch Arduino principal',
        '.h': 'Header',
        '.cpp': 'Implementation C++',
        '.c': 'Implementation C',
        '.py': 'Script Python',
        '.md': 'Documentation Markdown',
        '.json': 'Configuration JSON',
        '.gitignore': 'Configuration Git',

        # Fichiers specifiques
        'README.md': 'Documentation projet',
        'LICENSE': 'Licence MIT',
        'ARCHITECTURE.md': 'Documentation architecture',
        'LOG_REFERENCE.md': 'Reference des logs',
        'HARDWARE.md': 'Documentation hardware',
        'USER_GUIDE.md': 'Guide utilisateur',
        'CONTRIBUTING.md': 'Guide contribution',
        'config.h': 'Configuration globale',
        'pins.h': 'Definition des pins GPIO',
        'lang.h': 'Gestion multilingue',
    }

    # Descriptions par nom de fichier
    filename = os.path.basename(filepath)
    if filename in descriptions:
        return descriptions[filename]

    # Descriptions par extension
    ext = os.path.splitext(filename)[1]
    if ext in descriptions:
        return descriptions[ext]

    return ''

def get_directory_description(dirname):
    """
    Retourne une description du repertoire
    """
    descriptions = {
        'docs': 'Documentation',
        'config': 'Configuration',
        'src': 'Code source',
        'core': 'Logique metier',
        'data': 'Structures de donnees',
        'parsers': 'Parseurs (NMEA, OSM, JSON)',
        'system': 'Systeme (logger, config)',
        'hal': 'Abstraction hardware',
        'drivers': 'Drivers capteurs',
        'tasks': 'Taches FreeRTOS',
        'ui': 'Interface utilisateur LVGL',
        'tools': 'Outils developpement',
        'tests': 'Tests unitaires',
        'BMP5XX_ESP32': 'Driver BMP5 (barometre)',
        'BNO08x_ESP32': 'Driver BNO085 (IMU)',
        'GPS_I2C_ESP32': 'Driver GPS I2C',
    }

    return descriptions.get(dirname, '')

def should_ignore(path, ignore_dirs, ignore_files):
    """
    Verifie si le chemin doit etre ignore
    """
    name = os.path.basename(path)

    # Ignorer les repertoires
    if os.path.isdir(path) and name in ignore_dirs:
        return True

    # Ignorer les fichiers
    if os.path.isfile(path):
        if name in ignore_files:
            return True
        # Ignorer les fichiers caches
        if name.startswith('.') and name != '.gitignore':
            return True

    return False

def generate_tree(root_path, prefix='', is_last=True, ignore_dirs=None, ignore_files=None, output_lines=None, max_depth=None, current_depth=0):
    """
    Genere l'arborescence du projet de maniere recursive
    """
    if ignore_dirs is None:
        ignore_dirs = {'.git', '.vscode', '.pio', 'build', '.build', '__pycache__'}

    if ignore_files is None:
        ignore_files = {'.DS_Store', 'Thumbs.db'}

    if output_lines is None:
        output_lines = []

    if max_depth is not None and current_depth > max_depth:
        return output_lines

    # Ignorer ce chemin si necessaire
    if should_ignore(root_path, ignore_dirs, ignore_files):
        return output_lines

    # Symboles pour l'arborescence
    branch = '└── ' if is_last else '├── '
    extension = '    ' if is_last else '│   '

    # Nom du fichier/repertoire
    name = os.path.basename(root_path)

    # Ajouter la ligne avec description
    if os.path.isdir(root_path):
        desc = get_directory_description(name)
        desc_str = f'  # {desc}' if desc else ''
        output_lines.append(f'{prefix}{branch}{name}/{desc_str}')
    else:
        desc = get_file_description(root_path)
        desc_str = f'  # {desc}' if desc else ''
        output_lines.append(f'{prefix}{branch}{name}{desc_str}')

    # Si c'est un repertoire, parcourir recursivement
    if os.path.isdir(root_path):
        try:
            entries = sorted(os.listdir(root_path))
            # Trier: repertoires d'abord, puis fichiers
            entries = sorted(entries, key=lambda x: (not os.path.isdir(os.path.join(root_path, x)), x))

            # Filtrer les entrees a ignorer
            entries = [e for e in entries if not should_ignore(os.path.join(root_path, e), ignore_dirs, ignore_files)]

            for i, entry in enumerate(entries):
                entry_path = os.path.join(root_path, entry)
                is_last_entry = (i == len(entries) - 1)
                generate_tree(
                    entry_path,
                    prefix + extension,
                    is_last_entry,
                    ignore_dirs,
                    ignore_files,
                    output_lines,
                    max_depth,
                    current_depth + 1
                )
        except PermissionError:
            pass

    return output_lines

def main():
    """
    Fonction principale
    """
    # Chemin du projet (remonter a la racine depuis tools/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)  # Remonter d'un niveau
    project_name = os.path.basename(project_root)

    print(f"Generation de l'arborescence pour: {project_name}")
    print(f"Chemin: {project_root}")

    # Generer l'arborescence
    tree_lines = [f'{project_name}/  # Racine du projet']

    # Parcourir le repertoire racine
    entries = sorted(os.listdir(project_root))
    entries = sorted(entries, key=lambda x: (not os.path.isdir(os.path.join(project_root, x)), x))

    ignore_dirs = {'.git', '.vscode', '.pio', 'build', '.build', '__pycache__', '.idea'}
    ignore_files = {'.DS_Store', 'Thumbs.db', 'generate_tree.py', '*.pyc'}

    # Filtrer les entrees
    entries = [e for e in entries if not should_ignore(os.path.join(project_root, e), ignore_dirs, ignore_files)]

    for i, entry in enumerate(entries):
        entry_path = os.path.join(project_root, entry)
        is_last = (i == len(entries) - 1)
        generate_tree(entry_path, '', is_last, ignore_dirs, ignore_files, tree_lines)

    # Creer le contenu du fichier ARCHITECTURE.md
    architecture_content = """# Architecture du Projet Variomètre

## Structure des Fichiers

```
"""

    # Ajouter l'arborescence
    architecture_content += '\n'.join(tree_lines)
    architecture_content += '\n```\n\n'

    # Ajouter des sections supplementaires
    architecture_content += """
## Description des Modules

### Configuration (`config/`)
Contient tous les fichiers de configuration du projet :
- `config.h` : Parametres globaux, versions, priorites taches
- `pins.h` : Definition des broches GPIO
- `lang.h` : Systeme de traduction multilingue

### Code Source (`src/`)

#### Core (`src/core/`)
Modules de calcul pur :
- Filtre de Kalman (fusion capteurs)
- Calculs quaternions (orientation independante)
- Calculs de vol (finesse, distances, vent)

#### Data (`src/data/`)
Structures de donnees partagees entre modules

#### Parsers (`src/parsers/`)
Decodage des formats :
- NMEA (GPS)
- Tiles OSM
- Configuration JSON

#### System (`src/system/`)
Services systeme :
- Logger configurable
- Chargement configuration
- Theme par defaut

#### HAL (`src/hal/`)
Couche d'abstraction materielle

#### Drivers (`src/drivers/`)
Pilotes capteurs :
- BMP5 : Barometre
- BNO085 : IMU 9 axes
- GPS I2C : Positionnement

#### Tasks (`src/tasks/`)
Taches FreeRTOS du systeme

#### UI (`src/ui/`)
Interface utilisateur LVGL avec widgets dynamiques

### Documentation (`docs/`)
Documentation technique complete du projet

### Outils (`tools/`)
Scripts utilitaires pour le developpement

## Compilation

### Arduino IDE
1. Ouvrir `variometer.ino`
2. Selectionner la carte ESP32-P4
3. Configurer les parametres dans `config/config.h`
4. Compiler et televerser

### Dependances
- ESP32 Arduino Core 3.3.2
- LVGL 9.3.0
- FreeRTOS (inclus dans ESP32 Core)

## Conventions de Code

- Fichiers `.h` pour le code principal
- Fichiers `.h` + `.cpp` pour les bibliotheques
- Documentation Doxygen pour toutes les fonctions
- Logs via systeme `logger.h` (pas de `#ifdef DEBUG_MODE`)
- Textes UI sans accents, geres par `lang.h`

---
*Documentation generee automatiquement par generate_tree.py*
"""

    # Ecrire dans le fichier ARCHITECTURE.md
    architecture_path = os.path.join(project_root, 'docs', 'ARCHITECTURE.md')

    # Creer le repertoire docs s'il n'existe pas
    os.makedirs(os.path.dirname(architecture_path), exist_ok=True)

    with open(architecture_path, 'w', encoding='utf-8') as f:
        f.write(architecture_content)

    print(f"\nArborescence generee avec succes dans: {architecture_path}")
    print(f"Nombre de lignes: {len(tree_lines)}")

if __name__ == '__main__':
    main()
