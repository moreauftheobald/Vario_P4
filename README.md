# Variomètre ESP32-P4

Variomètre intelligent pour parapente basé sur ESP32-P4 avec écran tactile 7" MIPI DSI, GPS, capteurs inertiels et carte OpenStreetMap.

## Caractéristiques

- 📊 Vario haute précision (fusion baromètre + IMU via filtre de Kalman)
- 🗺️ Carte OpenStreetMap en temps réel avec cache hors ligne
- 📈 Calcul automatique : finesse, distances, vent, gain thermique
- 🎨 Interface personnalisable (thèmes JSON)
- 💾 Enregistrement traces IGC
- 📶 WiFi pour téléchargement cartes
- 🖥️ Grand écran 7" haute résolution

## Hardware

### Carte principale
- **ESP32-P4** : Waveshare ESP32-P4 7" MIPI DSI
  - Dual-core 400MHz
  - Écran tactile capacitif 7 pouces
  - Interface MIPI DSI
  - WiFi intégré
  - Carte SD

### Capteurs Requis
- **BMP390** : Baromètre haute précision
- **BNO085** : IMU 9 axes (accéléromètre + gyroscope + magnétomètre)
- **GPS GNSS** : Module GPS I2C pour position et vitesse

## Installation

### Prérequis
- Arduino IDE 2.x
- ESP32 Arduino Core 3.3.2
- LVGL 9.3.0

### Bibliothèques Nécessaires
```
- LVGL 9.3.0
- Driver BMP390 (inclus dans src/drivers/)
- Driver BNO085 (inclus dans src/drivers/)
- Driver GPS I2C (inclus dans src/drivers/)
```

### Configuration
1. Cloner le repository
   ```bash
   git clone https://github.com/votre-username/variometer-esp32-p4.git
   ```
2. Ouvrir `variometer.ino` dans Arduino IDE
3. Sélectionner la carte ESP32-P4
4. Configurer les paramètres dans `config/config.h`
5. Ajuster les pins si nécessaire dans `config/pins.h`
6. Compiler et téléverser

## Structure du Projet

```
variometer-esp32-p4/
├── variometer.ino          # Sketch principal
├── config/                 # Configuration
├── src/                    # Code source
│   ├── core/              # Calculs (Kalman, quaternions)
│   ├── drivers/           # Drivers capteurs
│   ├── tasks/             # Tâches FreeRTOS
│   ├── ui/                # Interface LVGL
│   └── system/            # Logger, thèmes
├── docs/                   # Documentation
└── tools/                  # Scripts utilitaires
```

Voir [ARCHITECTURE.md](docs/ARCHITECTURE.md) pour la documentation complète.

## Personnalisation

### Thème Graphique
L'interface est entièrement personnalisable via un fichier JSON :
1. Placer un fichier `theme.json` sur la carte SD
2. Configurer couleurs, positions, tailles des widgets
3. Redémarrer le variomètre

Voir `data/theme_default.json` pour un exemple complet.

**Éditeur web** (à venir) : Créez votre thème visuellement et exportez-le directement.

### Configuration des Logs
Ajuster les niveaux de debug par module via `config.json` sur la carte SD :
```json
{
  "logger": {
    "Output": "UART",
    "Flight": "Verbose",
    "GPS": "Info",
    "Display": "Warning"
  }
}
```

## Développement

### Tests Sans Hardware
Les modules suivants peuvent être développés et testés sans matériel :
- Filtre de Kalman
- Calculs quaternions
- Parseur NMEA
- Décodeur tiles OSM
- Calculs de vol

### Architecture Logicielle
- **FreeRTOS** : Tâches temps réel non attachées aux cores
- **LVGL 9.3.0** : Interface graphique moderne
- **Widgets dynamiques** : Positionnement et style depuis JSON
- **Logging configurable** : Niveau de debug par module

### Contribution
Les pull requests sont bienvenues ! 

**Avant de contribuer :**
- Suivre les conventions de documentation Doxygen
- Utiliser le système de logging (pas de `#ifdef DEBUG_MODE`)
- Tester le code si possible
- Documenter les nouveaux messages de log

Voir [CONTRIBUTING.md](CONTRIBUTING.md) pour plus de détails.

## Documentation

- [ARCHITECTURE.md](docs/ARCHITECTURE.md) : Architecture détaillée du projet
- [LOG_REFERENCE.md](docs/LOG_REFERENCE.md) : Référence complète des logs
- [HARDWARE.md](docs/HARDWARE.md) : Brochage et spécifications hardware
- [USER_GUIDE.md](docs/USER_GUIDE.md) : Guide utilisateur

## Fonctionnalités Principales

### Données de Vol
- Altitude GPS, QNE, QNH, QFE (AGL)
- Vario instantané et intégré
- Vitesse GPS et air
- Finesse instantanée et moyenne
- Distance parcourue et vers point de décollage
- Gain dans thermique
- Estimation vent (vitesse et direction)

### Navigation
- Carte OpenStreetMap centrée sur position
- Zoom réglable
- Orientation selon trajectoire
- Trail de vol
- Cache hors ligne automatique

### Enregistrement
- Traces IGC standard
- Export via carte SD
- Statistiques de vol

## Roadmap

- [x] Architecture logicielle
- [x] Système de logging
- [x] Système de thèmes JSON
- [ ] Implémentation filtre Kalman
- [ ] Interface LVGL complète
- [ ] Gestion cartes OSM
- [ ] Calculateur de vol complet
- [ ] Enregistrement IGC
- [ ] Tests en vol

## Licence

MIT License - Voir [LICENSE](LICENSE)

Ce projet est open-source et libre d'utilisation. Les contributions sont encouragées !

## Auteur

Theobald Moreau

## Remerciements

- **LVGL** pour l'excellente bibliothèque graphique
- **OpenStreetMap** pour les données cartographiques
- **Waveshare** pour la carte de développement ESP32-P4
- La communauté du parapente pour les retours et suggestions

## Contact

Pour toute question ou suggestion :
- Ouvrir une issue sur GitHub
- Contribuer via pull request

---

*Bon vol ! 🪂*
