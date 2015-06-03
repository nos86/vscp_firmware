#ifndef VSCP_APPLAYER_H
#define	VSCP_APPLAYER_H

#include "inttypes.h"
#include "vscp_firmware.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define VSCP_TX_MAX_TRIAL 3 //Max Trial before abort (based on 1mS)
#define VSCP_BTN_DEBOUNCE   20 //Time-base: 100mS

#define VSCP_EEPROM_BOOTLOADER_FLAG	0x00    // Reserved for bootloader
#define VSCP_EEPROM_NICKNAME            0x01	// Persistant nickname id storage
#define VSCP_EEPROM_SEGMENT_CRC         0x02    // Persistant segment crc storage
#define VSCP_EEPROM_ZONE                0x03    // Persistant vscp zone



#define VSCP_DM_EEPROM_START_LOC APP_EEPROM_SIZE - 8*VSCP_DM_COUNT 
#if VSCP_DM_EEPROM_START_LOC < 0 | VSCP_DM_COUNT > 256
#error("Decision matrix is too big")
#endif
    
//EEPROM-mirrored variables
uint8_t vscp_zone;
extern const uint8_t GuID[16];

/* Decision Matrix */
extern struct _dmrow decisionMatrix[VSCP_DM_COUNT];
void doApplicationDM(struct _dmrow row, struct _imsg *message);

void init_app_eeprom();
void vscp_freeRunning();
void vscp_10mS_Running();
void vscp_100mS_Running();
void vscp_ledActivity();

void doDM(BOOL oMsg);

void init_augusto_ram( void );
void init_augusto_eeprom();
void vscp_loadAllFromEEPROM();

#ifdef	__cplusplus
}
#endif

#endif	/* VSCP_APPLAYER_H */

