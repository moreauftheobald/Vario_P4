# Spécifications Projet Variomètre ESP32-P4

## Informations Générales

### Environnement de Développement
- **IDE**: Arduino IDE V2
- **Plateforme**: ESP32 Arduino Core 3.3.2
- **Carte**: ESP32-P4 Waveshare 7" MIPI DSI
- **Bibliothèques**: LVGL 9.3.0
- **Langage**: C
- **RTOS**: FreeRTOS (tâches non attachées à un core spécifique)

### Structure des Fichiers
- Fichiers `.h` pour le code principal (convention Arduino IDE V2)
- Fichiers `.h` et `.cpp` uniquement pour les bibliothèques personnalisées
- Un fichier `.h` par tâche FreeRTOS
- Un fichier `.h` par écran UI

---

## Architecture Logicielle

### Fichier Principal
- `variometer.ino` : Point d'entrée, initialisation des tâches FreeRTOS

### Configuration
- `config.h` : Paramètres globaux, pins, constantes
- `lang.h` : Gestion multilingue (textes sans accents ni caractères spéciaux)

### Tâches FreeRTOS
- `task_flight.h` : **Cœur du variomètre**
  - Lecture capteurs (IMU, baro, GPS via DMA UART)
  - Filtrage Kalman (altitude, vario)
  - Calculs de TOUS les paramètres de vol
  - Mise à jour structure globale `flight_data` (avec mutex)
  
- `task_display.h` : Gestion LVGL et affichage
- `task_map.h` : Gestion carte OSM (téléchargement, cache, décodage)
- `task_storage.h` : Enregistrement SD (logs de vol IGC)
- `task_wifi.h` : Gestion WiFi (téléchargement tiles OSM)

### Structures de Données
- `flight_data.h` : Structure globale partagée (mutex) avec tous les paramètres de vol
- `gps_data.h` : Structures GPS
- `sensor_data.h` : Données brutes capteurs

### Bibliothèques Personnalisées
- `kalman_filter.h/.cpp` : Filtre de Kalman
- `quaternion.h/.cpp` : Calculs quaternions pour vario IMU
- `osm_tiles.h/.cpp` : Décodage/gestion tiles OpenStreetMap
- `wind_estimation.h/.cpp` : Estimation vent
- `logger.h/.cpp` : Système de logging configurable

### Interface Utilisateur
- `theme_loader.h/.cpp` : Chargement thème depuis JSON
- `default_theme.h` : Thème par défaut embarqué en flash (PROGMEM)
- `ui_screen_map.h` : Écran carte + vario
- `ui_screen_stats.h` : Statistiques de vol
- `ui_screen_settings.h` : Paramètres
- `ui_screen_igc.h` : Sélection/visualisation traces
- `ui_widgets.h` : Widgets réutilisables
- `widget_base.h` : Structure de widget générique
- `widget_registry.h` : Registry des widgets pour positionnement dynamique

---

## Paramètres de Vol Calculés

### Altitudes
- Altitude GPS
- Altitude QNE (1013.25 hPa)
- Altitude QNH (pression locale)
- Hauteur sol - QFE (AGL)
- Historique d'altitude

### Vario
- Vario instantané (combinaison baro + IMU avec quaternions)
- Vario intégré
- Vario graphique
- Historique du vario
- Gain dans le thermique
- Gain depuis décollage

### Vitesses et Distances
- Vitesse GPS
- Vitesse air
- Distance depuis décollage
- Distance linéaire
- Distance cumulée
- Cap vers décollage

### Navigation et Performance
- Finesse instantanée
- Finesse moyenne
- Direction du vent
- Vitesse du vent
- Infos vent graphique

### Données Diverses
- Heure
- Temps de vol
- G-mètre
- Nombre de satellites GNSS
- Pression atmosphérique (hPa)
- Coordonnées GPS (Latitude, Longitude)
- Batterie

---

## Système de Filtrage

### Filtre de Kalman
- Fusion des données baromètre + IMU
- Calcul vario par quaternions (indépendant de l'orientation)
- Filtrage altitude et vario
- Pondération adaptative selon qualité des mesures

---

## Gestion de la Carte

### OpenStreetMap
- Fond de carte centré sur position GPS
- Zoom réglable par bouton
- Pointeur de position du parapente
- Orientation de la carte selon trajectoire

### Cache Tiles
- Téléchargement via WiFi
- Décodage et stockage sur carte SD
- Réutilisation en mode hors ligne
- Mise à jour automatique des tiles > 6 mois

---

## Système de Thème (JSON)

### Principe
- **Pas de `#define`**, tout en JSON
- Thème par défaut embarqué en flash (`default_theme.h` avec `PROGMEM`)
- Thème personnalisé sur SD (`theme.json`)
- Chargement avec priorité SD > Flash

### Personnalisation Complète
Chaque élément UI est individualisé :
- Position (x, y)
- Taille (width, height)
- Couleurs (bg, borders, text, etc.)
- Polices (tailles)
- Opacités
- Bordures et rayons
- Couches (z-index)
- Visibilité (on/off)

### Structure JSON
```json
{
  "screens": {
    "main": {
      "background": "#1A1A1A",
      "layout": [
        {
          "id": "vario_main",
          "type": "vario",
          "position": {"x": 120, "y": 50},
          "size": {"width": 200, "height": 150},
          "layer": 2,
          "visible": true,
          "style": {
            "bg_color": "#2D2D2D",
            "border_color": "#404040"
          },
          "content": {
            "value_font_size": 72,
            "colors": {
              "up_strong": "#00FF00"
            }
          }
        }
      ]
    }
  },
  "global": {
    "vario_thresholds": {
      "strong": 3.0,
      "medium": 1.5
    }
  }
}
```

### Widgets Dynamiques
- Création par factory depuis JSON
- Registry de widgets
- Repositionnement dynamique
- Application layout depuis JSON au démarrage

### Éditeur Web (futur)
- Drag & drop pour repositionnement
- Color picker pour chaque couleur
- Live preview
- Export JSON direct vers SD

---

## Système de Logging

### Principe
- Configuration via JSON (`config.json`)
- Pas de blocs `#ifdef DEBUG_MODE` dans le code
- Fonction `debug_print()` centralisée
- Niveaux de log par module

### Niveaux de Log
- `LOG_LEVEL_NONE` : Aucun log
- `LOG_LEVEL_ERROR` : Erreurs critiques
- `LOG_LEVEL_WARNING` : Avertissements
- `LOG_LEVEL_INFO` : Informations importantes
- `LOG_LEVEL_VERBOSE` : Détails de fonctionnement

### Sorties de Log
- `LOG_OUTPUT_NONE` : Désactivé
- `LOG_OUTPUT_UART` : Serial
- `LOG_OUTPUT_FILE` : Fichier sur SD
- `LOG_OUTPUT_BOTH` : Serial + Fichier

### Configuration JSON
```json
{
  "logger": {
    "Output": "UART",
    "Kalman": "Verbose",
    "I2C": "None",
    "BMP390": "Error",
    "IMU": "Info",
    "GPS": "Verbose",
    "Theme": "Warning",
    "Display": "Info",
    "Map": "Verbose",
    "WiFi": "Info",
    "Storage": "Error",
    "Flight": "Verbose",
    "System": "Info"
  }
}
```

### Modules de Log
- `LOG_MODULE_KALMAN`
- `LOG_MODULE_I2C`
- `LOG_MODULE_BMP390`
- `LOG_MODULE_IMU`
- `LOG_MODULE_GPS`
- `LOG_MODULE_THEME`
- `LOG_MODULE_DISPLAY`
- `LOG_MODULE_MAP`
- `LOG_MODULE_WIFI`
- `LOG_MODULE_STORAGE`
- `LOG_MODULE_FLIGHT`
- `LOG_MODULE_SYSTEM`

### Macros de Convenance
```c
LOG_E(module, ...) // Error
LOG_W(module, ...) // Warning
LOG_I(module, ...) // Info
LOG_V(module, ...) // Verbose
```

### Format de Log
```
[timestamp][MODULE][LEVEL] message
```

---

## Convention de Documentation du Code

### En-tête de Fichier
```c
/**
 * @file nom_fichier.h
 * @brief Description brève du fichier
 * 
 * Description détaillée du contenu et des responsabilités.
 * 
 * @author [Nom]
 * @date 2025-11-14
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: [NOM_MODULE]
 * [ERROR]
 *   - "Message erreur" : Description
 * [WARNING]
 *   - "Message warning" : Description
 * [INFO]
 *   - "Message info" : Description
 * [VERBOSE]
 *   - "Message verbose" : Description
 */
```

### Documentation des Fonctions
```c
/**
 * @brief Description courte sur une ligne
 * 
 * Description détaillée sur plusieurs lignes si nécessaire.
 * Explique le comportement, les cas particuliers, les prérequis.
 * 
 * @param[in] param_name Description du paramètre d'entrée
 * @param[out] param_name Description du paramètre de sortie
 * @param[in,out] param_name Description du paramètre entrée/sortie
 * 
 * @return Description de la valeur de retour
 * @retval valeur_specifique Description (si codes d'erreur multiples)
 */
void fonction_exemple(int param);
```

### Commentaires Inline
```c
// Etape 1: Description de l'étape
code();

// TODO: Fonctionnalité à implémenter
// FIXME: Bug connu à corriger
// XXX: Code temporaire à améliorer
```

### Documentation des Logs
Chaque fichier doit documenter ses messages de log dans son en-tête :
```c
/**
 * LOG DOCUMENTATION - MODULE: THEME
 * 
 * [ERROR]
 *   - "Failed to load any theme" : Aucun theme (SD ni flash) charge
 *   - "JSON parse error: %s" : Erreur parsing theme.json
 * 
 * [WARNING]
 *   - "Theme file corrupted" : Fichier theme.json corrompu
 * 
 * [INFO]
 *   - "Theme loaded from %s" : Source du theme (SD/flash)
 * 
 * [VERBOSE]
 *   - "Parsing theme JSON..." : Debut du parsing
 */
```

---

## Règles de Codage

### Général
- Textes à l'écran : Sans accents ni caractères spéciaux
- Traductions : Via `lang.h`
- Pas de `#ifdef DEBUG_MODE` : Utiliser `debug_print()` à la place

### Multitâche FreeRTOS
- Tâches non attachées à un core spécifique
- Protection des données partagées par mutex
- Communication par queues si nécessaire

### Capteurs
- Lecture IMU : Calculs par quaternions (orientation-indépendant)
- GPS : Interception DMA UART
- Fusion données : Filtre de Kalman

### Interface
- Bibliothèque : LVGL 9.3.0
- Création widgets dynamique depuis JSON
- Registry de widgets pour gestion globale

---

## Points Clés à Retenir

1. **Structure unifiée** : `task_flight.h` gère capteurs + Kalman + calculs + données de vol
2. **Tout en JSON** : Thème et configuration logger (pas de `#define`)
3. **Logging configurable** : Niveau par module, sortie configurable
4. **UI dynamique** : Widgets positionnables, thème personnalisable
5. **Documentation complète** : Chaque fonction et chaque log documentés
6. **Tiles OSM** : Cache SD avec mise à jour auto > 6 mois
7. **Filtre Kalman** : Fusion baro + IMU avec quaternions

---

## Fichiers de Référence

### Documentation Centralisée
- `docs/LOG_REFERENCE.h` (ou `.md`) : Référence complète de tous les logs
- `docs/ARCHITECTURE.md` : Architecture détaillée du projet

### Fichiers JSON Utilisateur
- `/theme.json` : Thème personnalisé (sur SD)
- `/config.json` : Configuration logger (sur SD)

---

*Document généré le 2025-11-14*
*Version 1.0*
*Carte cible : ESP32-P4*
