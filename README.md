# Variomètre ESP32-S3/P4

Variomètre intelligent pour parapente basé sur ESP32-S3/P4 avec écran tactile, GPS, capteurs inertiels et carte OpenStreetMap.

## Caractéristiques

- 📊 Vario haute précision (fusion baromètre + IMU via filtre de Kalman)
- 🗺️ Carte OpenStreetMap en temps réel avec cache hors ligne
- 📈 Calcul automatique : finesse, distances, vent, gain thermique
- 🎨 Interface personnalisable (thèmes JSON)
- 💾 Enregistrement traces IGC
- 📶 WiFi pour téléchargement cartes

## Hardware Supporté

- **ESP32-S3** : Waveshare ESP32-S3 4.3" RGB LCD
- **ESP32-P4** : Waveshare ESP32-P4 7" MIPI DSI (à venir)

## Capteurs Requis

- BMP390 : Pression atmosphérique
- IMU 6/9 axes : Accéléromètre + gyroscope
- GPS GNSS : Position et vitesse

## Installation

### Prérequis
- Arduino IDE 2.x
- ESP32 Arduino Core 3.3.2
- LVGL 9.3.0

### Bibliothèques Nécessaires
```
- LVGL 9.3.0
- Adafruit_BMP3XX (ou équivalent BMP390)
- (Liste à compléter)
```

### Configuration
1. Cloner le repository
2. Ouvrir `variometer.ino` dans Arduino IDE
3. Configurer les pins dans `config/pins.h`
4. Compiler et téléverser

## Structure du Projet

Voir [ARCHITECTURE.md](docs/ARCHITECTURE.md) pour la documentation complète.

## Personnalisation

### Thème
Placer un fichier `theme.json` sur la carte SD pour personnaliser l'interface.
Voir `data/theme_default.json` pour un exemple complet.

### Logging
Configurer les niveaux de log via `config.json` sur la carte SD.

## Développement

### Tests
Les modules core (Kalman, quaternions, calculs) peuvent être testés sans hardware.

### Contribution
Les pull requests sont bienvenues ! Voir [CONTRIBUTING.md](CONTRIBUTING.md).

## Licence

MIT License - Voir [LICENSE](LICENSE)

## Auteur

[Votre nom]

## Remerciements

- LVGL pour l'interface graphique
- OpenStreetMap pour les cartes
