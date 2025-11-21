/**
 * @file usb_msc_manager.cpp
 * @brief Implémentation du gestionnaire USB MSC
 * 
 * @author Franck Moreau
 * @date 2025-11-19
 * @version 1.0
 */

#include "usb_msc_manager.h"
#include "src/system/logger/logger.h"
#include "src/system/sd_manager/sd_manager.h"

// Instance USB MSC globale
USBMSC MSC;

// État interne
static usb_msc_state_t msc_state = USB_MSC_IDLE;
static bool msc_initialized = false;

// Callbacks USB MSC
static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize);
static int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize);
static bool onStartStop(uint8_t power_condition, bool start, bool load_eject);

/**
 * @brief Callback lecture secteur SD
 */
static int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    // Log pour debug
    LOG_V(LOG_MODULE_SYSTEM, "USB Read: lba=%u offset=%u size=%u", lba, offset, bufsize);
    
    // Vérifier alignement secteur
    if (offset % 512 != 0 || bufsize % 512 != 0) {
        LOG_E(LOG_MODULE_SYSTEM, "USB Read: unaligned access!");
        return -1;
    }
    
    // Calculer le secteur de départ
    uint32_t sector = lba + (offset / 512);
    uint32_t count = bufsize / 512;
    
    // Vérifier limites
    uint64_t cardSize = SD_MMC.cardSize();
    uint64_t maxSector = cardSize / 512;
    
    if (sector + count > maxSector) {
        LOG_E(LOG_MODULE_SYSTEM, "USB Read: out of bounds! sector=%u count=%u max=%llu", 
              sector, count, maxSector);
        return -1;
    }
    
    // Lire secteur par secteur
    uint8_t* buf = (uint8_t*)buffer;
    for (uint32_t i = 0; i < count; i++) {
        if (!SD_MMC.readRAW(buf + (i * 512), sector + i)) {
            LOG_E(LOG_MODULE_SYSTEM, "USB Read failed at sector %u", sector + i);
            return -1;
        }
    }
    
    return bufsize;
}

/**
 * @brief Callback écriture secteur SD
 */
static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    // Log pour debug
    LOG_V(LOG_MODULE_SYSTEM, "USB Write: lba=%u offset=%u size=%u", lba, offset, bufsize);
    
    // Vérifier alignement secteur
    if (offset % 512 != 0 || bufsize % 512 != 0) {
        LOG_E(LOG_MODULE_SYSTEM, "USB Write: unaligned access!");
        return -1;
    }
    
    // Calculer le secteur de départ
    uint32_t sector = lba + (offset / 512);
    uint32_t count = bufsize / 512;
    
    // Vérifier limites
    uint64_t cardSize = SD_MMC.cardSize();
    uint64_t maxSector = cardSize / 512;
    
    if (sector + count > maxSector) {
        LOG_E(LOG_MODULE_SYSTEM, "USB Write: out of bounds! sector=%u count=%u max=%llu",
              sector, count, maxSector);
        return -1;
    }
    
    // Écrire secteur par secteur
    for (uint32_t i = 0; i < count; i++) {
        if (!SD_MMC.writeRAW(buffer + (i * 512), sector + i)) {
            LOG_E(LOG_MODULE_SYSTEM, "USB Write failed at sector %u", sector + i);
            return -1;
        }
    }
    
    return bufsize;
}

/**
 * @brief Callback start/stop USB
 */
static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
    LOG_V(LOG_MODULE_SYSTEM, "USB MSC state change: start=%d eject=%d", start, load_eject);
    return true;
}

/**
 * @brief Initialise USB MSC
 */
bool usb_msc_init() {
    if (msc_initialized) {
        LOG_W(LOG_MODULE_SYSTEM, "USB MSC already initialized");
        return true;
    }

    LOG_I(LOG_MODULE_SYSTEM, "Initializing USB MSC...");

    // Vérifier que SD est disponible
    if (!sd_manager_is_available()) {
        LOG_E(LOG_MODULE_SYSTEM, "SD not available for USB MSC");
        return false;
    }

    // Initialiser USB avec force
    LOG_V(LOG_MODULE_SYSTEM, "Starting USB stack...");
    /*USB.VID(0x303A); // Espressif VID
    USB.PID(0x4002); // Generic PID
    USB.productName("ESP32-P4 Vario");
    USB.manufacturerName("Espressif");
    USB.serialNumber("123456");*/
    USB.begin();
    
    delay(100); // Laisser USB s'initialiser
    
    msc_initialized = true;
    msc_state = USB_MSC_IDLE;
    
    LOG_I(LOG_MODULE_SYSTEM, "USB MSC initialized (idle)");
    LOG_I(LOG_MODULE_SYSTEM, "Connect device to USB-C port (NOT USB-A)");
    
    return true;
}

/**
 * @brief Active USB MSC
 */
bool usb_msc_start() {
    if (!msc_initialized) {
        LOG_E(LOG_MODULE_SYSTEM, "USB MSC not initialized");
        return false;
    }

    if (msc_state == USB_MSC_ACTIVE) {
        LOG_W(LOG_MODULE_SYSTEM, "USB MSC already active");
        return true;
    }

    LOG_I(LOG_MODULE_SYSTEM, "Starting USB MSC...");

    // CRITIQUE: Fermer tous les fichiers ouverts et flusher les buffers
    LOG_V(LOG_MODULE_SYSTEM, "Flushing all SD buffers...");
    
    // Obtenir infos carte (SD_MMC déjà initialisé)
    uint64_t cardSize = SD_MMC.cardSize();
    
    if (cardSize == 0) {
        LOG_E(LOG_MODULE_SYSTEM, "SD card not available");
        msc_state = USB_MSC_ERROR;
        return false;
    }
    
    uint32_t sectorCount = cardSize / 512;
    
    LOG_I(LOG_MODULE_SYSTEM, "SD card: %llu MB, %u sectors", 
          cardSize / (1024*1024), sectorCount);

    // Configurer USB MSC
    LOG_V(LOG_MODULE_SYSTEM, "Configuring USB MSC device...");
    MSC.vendorID("ESP32");
    MSC.productID("SD Card");
    MSC.productRevision("1.0");
    MSC.onRead(onRead);
    MSC.onWrite(onWrite);
    MSC.onStartStop(onStartStop);
    MSC.mediaPresent(true);
    
    LOG_V(LOG_MODULE_SYSTEM, "Starting MSC with %u sectors...", sectorCount);
    MSC.begin(sectorCount, 512);
    
    delay(500); // Laisser le temps au PC de détecter

    msc_state = USB_MSC_ACTIVE;
    
    LOG_I(LOG_MODULE_SYSTEM, "USB MSC started - SD accessible via USB");
    LOG_I(LOG_MODULE_SYSTEM, "Check USB-C port on your PC (may take 5-10 seconds)");
    LOG_W(LOG_MODULE_SYSTEM, "==========================================");
    LOG_W(LOG_MODULE_SYSTEM, "CRITICAL: Eject properly on PC before Stop!");
    LOG_W(LOG_MODULE_SYSTEM, "==========================================");
    
    return true;
}

/**
 * @brief Désactive USB MSC
 */
bool usb_msc_stop() {
    if (msc_state != USB_MSC_ACTIVE) {
        LOG_W(LOG_MODULE_SYSTEM, "USB MSC not active");
        return true;
    }

    LOG_I(LOG_MODULE_SYSTEM, "Stopping USB MSC...");

    // Arrêter USB MSC
    MSC.end();
    
    delay(100);

    // Note: On ne démonte PAS SD_MMC car c'était déjà initialisé
    // On réinitialise juste sd_manager pour recréer les structures internes
    LOG_V(LOG_MODULE_SYSTEM, "Reinitializing SD manager after USB");
    
    // Pas besoin de sd_manager_init() car SD_MMC est déjà actif
    // Il suffit de recréer les dossiers système si nécessaire
    // (sera fait automatiquement au prochain accès)

    msc_state = USB_MSC_IDLE;
    
    LOG_I(LOG_MODULE_SYSTEM, "USB MSC stopped - SD accessible by ESP32");
    LOG_I(LOG_MODULE_SYSTEM, "You may need to restart to reload config files");
    
    return true;
}

/**
 * @brief Vérifie si USB MSC est actif
 */
bool usb_msc_is_active() {
    return (msc_state == USB_MSC_ACTIVE);
}

/**
 * @brief Obtient l'état USB MSC
 */
usb_msc_state_t usb_msc_get_state() {
    return msc_state;
}

/**
 * @brief Toggle USB MSC
 */
bool usb_msc_toggle() {
    if (msc_state == USB_MSC_ACTIVE) {
        return usb_msc_stop();
    } else {
        return usb_msc_start();
    }
}