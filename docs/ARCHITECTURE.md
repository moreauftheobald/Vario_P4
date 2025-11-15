# Architecture du Projet Variomètre

## Structure des Fichiers

```
Vario_P4/  # Racine du projet
├── config/  # Configuration
│   ├── config.h  # Configuration globale
│   ├── lang.h  # Gestion multilingue
│   └── pins.h  # Definition des pins GPIO
├── data/  # Structures de donnees
│   ├── fonts/
│   ├── config.json  # Configuration JSON
│   └── theme_default.json  # Configuration JSON
├── docs/  # Documentation
│   ├── ARCHITECTURE.md  # Documentation architecture
│   ├── HARDWARE.md  # Documentation hardware
│   ├── LOG_REFERENCE.md  # Reference des logs
│   ├── Specifications Projet Variometre ESP32-P4.md  # Documentation Markdown
│   └── USER_GUIDE.md  # Guide utilisateur
├── src/  # Code source
│   ├── core/  # Logique metier
│   │   ├── flight_calculations/
│   │   │   ├── flight_calculations.cpp  # Implementation C++
│   │   │   └── flight_calculations.h  # Header
│   │   ├── kalman_filter/
│   │   │   ├── kalman_filter.cpp  # Implementation C++
│   │   │   └── kalman_filter.h  # Header
│   │   ├── quaternion/
│   │   │   ├── quaternion.cpp  # Implementation C++
│   │   │   └── quaternion.h  # Header
│   │   └── wind_estimation/
│   │       ├── wind_estimation.cpp  # Implementation C++
│   │       └── wind_estimation.h  # Header
│   ├── data/  # Structures de donnees
│   │   ├── config_data.h  # Header
│   │   ├── flight_data.h  # Header
│   │   ├── gps_data.h  # Header
│   │   └── sensor_data.h  # Header
│   ├── drivers/  # Drivers capteurs
│   │   ├── BMP3XX_ESP32/  # Driver BMP390 (barometre)
│   │   │   ├── BMP3XX_ESP32.cpp  # Implementation C++
│   │   │   ├── BMP3XX_ESP32.h  # Header
│   │   │   ├── bmp3.c  # Implementation C
│   │   │   ├── bmp3.h  # Header
│   │   │   └── bmp3_defs.h  # Header
│   │   ├── BNO08x_ESP32/  # Driver BNO085 (IMU)
│   │   │   ├── BNO08x_ESP32.cpp  # Implementation C++
│   │   │   ├── BNO08x_ESP32.h  # Header
│   │   │   ├── sh2.c  # Implementation C
│   │   │   ├── sh2.h  # Header
│   │   │   ├── sh2_SensorValue.c  # Implementation C
│   │   │   ├── sh2_SensorValue.h  # Header
│   │   │   ├── sh2_err.h  # Header
│   │   │   ├── sh2_hal.h  # Header
│   │   │   ├── sh2_util.c  # Implementation C
│   │   │   ├── sh2_util.h  # Header
│   │   │   ├── shtp.c  # Implementation C
│   │   │   └── shtp.h  # Header
│   │   └── GPS_I2C_ESP32/  # Driver GPS I2C
│   │       ├── GPS_I2C_ESP32.cpp  # Implementation C++
│   │       └── GPS_I2C_ESP32.h  # Header
│   ├── hal/  # Abstraction hardware
│   │   ├── i2c_wrapper/
│   │   │   ├── i2c_wrapper.cpp  # Implementation C++
│   │   │   └── i2c_wrapper.h  # Header
│   │   ├── uart_wrapper/
│   │   │   ├── uart_wrapper.cpp  # Implementation C++
│   │   │   └── uart_wrapper.h  # Header
│   │   └── display_init.h  # Header
│   ├── parsers/  # Parseurs (NMEA, OSM, JSON)
│   │   ├── nmea_parser/
│   │   │   ├── nmea_parser.cpp  # Implementation C++
│   │   │   └── nmea_parser.h  # Header
│   │   ├── osm_decoder/
│   │   │   ├── osm_decoder.cpp  # Implementation C++
│   │   │   └── osm_decoder.h  # Header
│   │   └── theme_loader/
│   │       ├── theme_loader.cpp  # Implementation C++
│   │       └── theme_loader.h  # Header
│   ├── system/  # Systeme (logger, config)
│   │   ├── config_loader/
│   │   │   ├── config_loader.cpp  # Implementation C++
│   │   │   └── config_loader.h  # Header
│   │   ├── logger/
│   │   │   ├── logger.cpp  # Implementation C++
│   │   │   └── logger.h  # Header
│   │   ├── default_config.h  # Header
│   │   └── default_theme.h  # Header
│   ├── tasks/  # Taches FreeRTOS
│   │   ├── task_display.h  # Header
│   │   ├── task_flight.h  # Header
│   │   ├── task_map.h  # Header
│   │   ├── task_storage.h  # Header
│   │   └── task_wifi.h  # Header
│   └── ui/  # Interface utilisateur LVGL
│       ├── widget/
│       │   ├── widget_registry./
│       │   │   ├── widget_registry.cpp  # Implementation C++
│       │   │   └── widget_registry.h  # Header
│       │   └── widget_base.h  # Header
│       ├── ui_screen_igc.h  # Header
│       ├── ui_screen_map.h  # Header
│       ├── ui_screen_settings.h  # Header
│       ├── ui_screen_stats.h  # Header
│       └── ui_widgets.h  # Header
├── tools/  # Outils developpement
│   ├── generate_log_doc.py  # Script Python
│   └── theme_validator.py  # Script Python
├── .gitignore  # Configuration Git
├── LICENSE  # Licence MIT
├── README.md  # Documentation projet
├── Vario_P4.ino  # Sketch Arduino principal
└── partitions.csv
```


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
- BMP390 : Barometre
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
