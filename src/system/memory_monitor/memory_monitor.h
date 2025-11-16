/**
 * @file memory_monitor.h
 * @brief Monitoring de la mémoire système
 * 
 * Surveille l'utilisation de la SRAM, PSRAM, Flash et détecte
 * la fragmentation mémoire. Fournit des statistiques détaillées
 * pour le débogage et l'optimisation.
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: MEMORY
 * [ERROR]
 *   - "Critical memory shortage: %d bytes free" : Mémoire critique
 *   - "Memory allocation failed: %d bytes" : Échec allocation
 * 
 * [WARNING]
 *   - "Low memory: %d bytes free" : Mémoire faible
 *   - "High fragmentation: largest block %d bytes" : Fragmentation élevée
 *   - "PSRAM not available" : PSRAM non détectée
 * 
 * [INFO]
 *   - "Memory stats: SRAM=%d PSRAM=%d Flash=%d" : Stats globales
 *   - "Heap free: %d bytes (%d%%)" : Heap libre
 *   - "Largest free block: %d bytes" : Plus gros bloc dispo
 * 
 * [VERBOSE]
 *   - "SRAM: free=%d largest=%d" : Détails SRAM
 *   - "PSRAM: free=%d largest=%d" : Détails PSRAM
 *   - "Flash partition: size=%d used=%d" : Détails Flash
 */

#ifndef MEMORY_MONITOR_H
#define MEMORY_MONITOR_H

#include <Arduino.h>

/**
 * @brief Structure contenant les statistiques mémoire
 */
typedef struct {
    // SRAM (heap interne)
    uint32_t sram_total;           // Taille totale SRAM
    uint32_t sram_free;            // SRAM libre
    uint32_t sram_used;            // SRAM utilisée
    uint32_t sram_largest_block;   // Plus gros bloc libre SRAM
    float    sram_fragmentation;   // Fragmentation SRAM (0.0-1.0)
    
    // PSRAM (si disponible)
    bool     psram_available;      // PSRAM présente
    uint32_t psram_total;          // Taille totale PSRAM
    uint32_t psram_free;           // PSRAM libre
    uint32_t psram_used;           // PSRAM utilisée
    uint32_t psram_largest_block;  // Plus gros bloc libre PSRAM
    float    psram_fragmentation;  // Fragmentation PSRAM (0.0-1.0)
    
    // Flash
    uint32_t flash_total;          // Taille partition flash
    uint32_t flash_used;           // Flash utilisée
    uint32_t flash_free;           // Flash libre
    
    // Général
    uint32_t total_free;           // Total mémoire libre (SRAM + PSRAM)
    uint32_t timestamp;            // Timestamp de la mesure (millis)
} memory_stats_t;

/**
 * @brief Initialise le monitoring mémoire
 * 
 * Détecte la présence de PSRAM et initialise les statistiques.
 * 
 * @return true si succès, false si erreur
 */
bool memory_monitor_init();

/**
 * @brief Obtient les statistiques mémoire actuelles
 * 
 * Remplit la structure avec les valeurs à jour.
 * 
 * @param[out] stats Structure à remplir
 */
void memory_monitor_get_stats(memory_stats_t* stats);

/**
 * @brief Affiche un rapport détaillé dans les logs
 * 
 * Log les statistiques mémoire avec le niveau approprié
 * selon l'état de la mémoire (ERROR/WARNING/INFO/VERBOSE).
 */
void memory_monitor_print_report();

/**
 * @brief Vérifie l'état de la mémoire
 * 
 * Retourne un code d'état selon le niveau de mémoire disponible.
 * 
 * @return 0=OK, 1=LOW, 2=CRITICAL
 */
int memory_monitor_check_status();

/**
 * @brief Calcule le taux de fragmentation
 * 
 * Compare la taille du plus gros bloc libre à la mémoire totale libre.
 * Plus le taux est élevé, plus la mémoire est fragmentée.
 * 
 * @param[in] free_memory Mémoire libre totale
 * @param[in] largest_block Plus gros bloc libre
 * @return Taux de fragmentation (0.0 = pas fragmenté, 1.0 = très fragmenté)
 */
float memory_monitor_calc_fragmentation(uint32_t free_memory, uint32_t largest_block);

/**
 * @brief Vérifie si PSRAM est disponible
 * 
 * @return true si PSRAM détectée et utilisable, false sinon
 */
bool memory_monitor_has_psram();

/**
 * @brief Obtient la mémoire libre totale
 * 
 * Somme de SRAM libre + PSRAM libre (si disponible).
 * 
 * @return Mémoire libre en octets
 */
uint32_t memory_monitor_get_free_total();

/**
 * @brief Obtient le plus gros bloc allouable
 * 
 * Retourne la taille du plus gros bloc mémoire qui peut
 * être alloué d'un seul tenant (utile pour détecter fragmentation).
 * 
 * @return Taille du plus gros bloc en octets
 */
uint32_t memory_monitor_get_largest_block();

#endif // MEMORY_MONITOR_H