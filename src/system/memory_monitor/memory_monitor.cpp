/**
 * @file memory_monitor.cpp
 * @brief Implémentation du monitoring mémoire
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "memory_monitor.h"
#include "src/system/logger/logger.h"
#include <esp_heap_caps.h>
#include <esp_partition.h>

// Variables privées
static bool psram_detected = false;

/**
 * @brief Initialise le monitoring mémoire
 */
bool memory_monitor_init() {
    // Détecter PSRAM
    psram_detected = psramFound();
    
    if (psram_detected) {
        LOG_I(LOG_MODULE_MEMORY, "PSRAM detected: %d bytes", ESP.getPsramSize());
    } else {
        LOG_W(LOG_MODULE_MEMORY, "PSRAM not available");
    }
    
    // Afficher un rapport initial
    memory_monitor_print_report();
    
    return true;
}

/**
 * @brief Obtient les statistiques mémoire
 */
void memory_monitor_get_stats(memory_stats_t* stats) {
    if (stats == NULL) return;
    
    // SRAM (heap interne)
    stats->sram_total = ESP.getHeapSize();
    stats->sram_free = ESP.getFreeHeap();
    stats->sram_used = stats->sram_total - stats->sram_free;
    stats->sram_largest_block = ESP.getMaxAllocHeap();
    stats->sram_fragmentation = memory_monitor_calc_fragmentation(
        stats->sram_free, stats->sram_largest_block);
    
    // PSRAM
    stats->psram_available = psram_detected;
    if (psram_detected) {
        stats->psram_total = ESP.getPsramSize();
        stats->psram_free = ESP.getFreePsram();
        stats->psram_used = stats->psram_total - stats->psram_free;
        stats->psram_largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        stats->psram_fragmentation = memory_monitor_calc_fragmentation(
            stats->psram_free, stats->psram_largest_block);
    } else {
        stats->psram_total = 0;
        stats->psram_free = 0;
        stats->psram_used = 0;
        stats->psram_largest_block = 0;
        stats->psram_fragmentation = 0.0f;
    }
    
    // Flash
    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
    if (partition != NULL) {
        stats->flash_total = partition->size;
        stats->flash_used = 0; // TODO: calculer usage réel
        stats->flash_free = partition->size;
    } else {
        stats->flash_total = 0;
        stats->flash_used = 0;
        stats->flash_free = 0;
    }
    
    // Total
    stats->total_free = stats->sram_free + stats->psram_free;
    stats->timestamp = millis();
}

/**
 * @brief Affiche un rapport détaillé
 */
void memory_monitor_print_report() {
    memory_stats_t stats;
    memory_monitor_get_stats(&stats);
    
    // Déterminer le niveau de log selon l'état mémoire
    int status = memory_monitor_check_status();
    
    if (status == 2) {
        // Critique
        LOG_E(LOG_MODULE_MEMORY, "Critical memory shortage: %d bytes free", stats.sram_free);
    } else if (status == 1) {
        // Faible
        LOG_W(LOG_MODULE_MEMORY, "Low memory: %d bytes free", stats.sram_free);
    }
    
    // SRAM
    LOG_I(LOG_MODULE_MEMORY, "SRAM: %d/%d KB free (%.1f%%), largest: %d KB",
          stats.sram_free / 1024, stats.sram_total / 1024,
          (stats.sram_free * 100.0f) / stats.sram_total,
          stats.sram_largest_block / 1024);
    
    if (stats.sram_fragmentation > MEMORY_FRAG_THRESHOLD) {
        LOG_W(LOG_MODULE_MEMORY, "High SRAM fragmentation: %.1f%%", 
              stats.sram_fragmentation * 100.0f);
    }
    
    LOG_V(LOG_MODULE_MEMORY, "SRAM: free=%d largest=%d frag=%.1f%%",
          stats.sram_free, stats.sram_largest_block, 
          stats.sram_fragmentation * 100.0f);
    
    // PSRAM
    if (stats.psram_available) {
        LOG_I(LOG_MODULE_MEMORY, "PSRAM: %d/%d KB free (%.1f%%), largest: %d KB",
              stats.psram_free / 1024, stats.psram_total / 1024,
              (stats.psram_free * 100.0f) / stats.psram_total,
              stats.psram_largest_block / 1024);
        
        if (stats.psram_fragmentation > MEMORY_FRAG_THRESHOLD) {
            LOG_W(LOG_MODULE_MEMORY, "High PSRAM fragmentation: %.1f%%",
                  stats.psram_fragmentation * 100.0f);
        }
        
        LOG_V(LOG_MODULE_MEMORY, "PSRAM: free=%d largest=%d frag=%.1f%%",
              stats.psram_free, stats.psram_largest_block,
              stats.psram_fragmentation * 100.0f);
    }
    
    // Flash
    LOG_V(LOG_MODULE_MEMORY, "Flash partition: total=%d KB",
          stats.flash_total / 1024);
    
    // Total
    LOG_I(LOG_MODULE_MEMORY, "Total free memory: %d KB", stats.total_free / 1024);
}

/**
 * @brief Vérifie l'état de la mémoire
 */
int memory_monitor_check_status() {
    uint32_t free_heap = ESP.getFreeHeap();
    
    if (free_heap < MEMORY_CRITICAL_THRESHOLD) {
        return 2; // CRITICAL
    } else if (free_heap < MEMORY_LOW_THRESHOLD) {
        return 1; // LOW
    }
    
    return 0; // OK
}

/**
 * @brief Calcule le taux de fragmentation
 */
float memory_monitor_calc_fragmentation(uint32_t free_memory, uint32_t largest_block) {
    if (free_memory == 0) return 0.0f;
    
    // Fragmentation = 1 - (plus_gros_bloc / total_libre)
    // 0.0 = pas fragmenté (un seul bloc)
    // 1.0 = très fragmenté (beaucoup de petits blocs)
    return 1.0f - ((float)largest_block / (float)free_memory);
}

/**
 * @brief Vérifie si PSRAM est disponible
 */
bool memory_monitor_has_psram() {
    return psram_detected;
}

/**
 * @brief Obtient la mémoire libre totale
 */
uint32_t memory_monitor_get_free_total() {
    uint32_t total = ESP.getFreeHeap();
    
    if (psram_detected) {
        total += ESP.getFreePsram();
    }
    
    return total;
}

/**
 * @brief Obtient le plus gros bloc allouable
 */
uint32_t memory_monitor_get_largest_block() {
    uint32_t largest = ESP.getMaxAllocHeap();
    
    if (psram_detected) {
        uint32_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        if (psram_largest > largest) {
            largest = psram_largest;
        }
    }
    
    return largest;
}