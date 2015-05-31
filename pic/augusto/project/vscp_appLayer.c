#include <xc.h>
#include "inttypes.h"
#include "vscp_firmware.h"
#include "vscp_projdefs.h"
#include "vscp_appLayer.h"
#include <eeprom.h>
#include "vscp_class.h"
#include "vscp_type.h"

#include <driver.h>
#include <ECANPoll.h>

extern const char mdfLink[];
uint16_t vscp_registerPage = 0;

const uint8_t *GuID_LSB = ( uint8_t * ) 0x200000;

uint8_t vscp_zone;

struct _dmrow decisionMatrix[VSCP_DM_SIZE];

void vscp_loadDecisionMatrixFromEEPROM();


/* Main flow chart */
void vscp_freeRunning(){
    //Check for valid messages
    vscp_imsg.flags = 0; vscp_getEvent();
    if ( vscp_node_state == VSCP_STATE_INIT ){ 
        vscp_handleProbeState();
        return;
    }else if (vscp_node_state == VSCP_STATE_ACTIVE){
        if (vscp_imsg.flags & VSCP_VALID_MSG) doDM();
    }
}
void vscp_10mS_Running(){
}
void vscp_100mS_Running(){
    if (vscp_btnPin == 0) 
        vscp_initbtncnt++;
    else
        vscp_initbtncnt = 0;
    if((vscp_initbtncnt > VSCP_BTN_DEBOUNCE) & (VSCP_STATE_INIT != vscp_node_state)){
        vscp_nickname = VSCP_ADDRESS_FREE;
        eeprom_write(VSCP_EEPROM_NICKNAME, VSCP_ADDRESS_FREE);
        vscp_init();
    }
    switch (vscp_node_state){
        case VSCP_STATE_STARTUP: // Cold/Warm startup
            if (VSCP_ADDRESS_FREE == vscp_nickname) vscp_node_state = VSCP_STATE_INIT;
            else{
                vscp_node_state = VSCP_STATE_ACTIVE;
                vscp_goActiveState();
            }
            break;
        case VSCP_STATE_INIT:
             // No job.. It's defined in freeRunning
            break;
        case VSCP_STATE_PREACTIVE:
            vscp_goActiveState();
            break;
        case VSCP_STATE_ACTIVE:
            // No job.. It's defined in freeRunning
            break;
        case VSCP_STATE_ERROR:
            vscp_error();
        default:
            vscp_node_state = VSCP_STATE_STARTUP;
            break;
    }
}
void vscp_ledActivity(){
    if (VSCP_LED_BLINK1 == vscp_initledfunc)
        if (vscp_ledPin==1)  vscp_ledPin=0; else vscp_ledPin=1;
    else if (VSCP_LED_ON == vscp_initledfunc)
        vscp_ledPin = 0;
    else if (VSCP_LED_OFF == vscp_initledfunc)
        vscp_ledPin = 1;
}

void doDM( ){
    unsigned char i;
    unsigned short class_filter;
    unsigned short class_mask;

    // Don't deal with the control functionality
    if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class ){ vscp_handleProtocolEvent(); return;}
    for ( i=0; i<VSCP_DM_SIZE; i++ ) {
        if (!(decisionMatrix[i].flags & VSCP_DM_FLAG_ENABLED) ) continue; // Is the DM row enabled?
        if ( (decisionMatrix[i].flags & VSCP_DM_FLAG_CHECK_OADDR) && //Is the address checked?
            (vscp_imsg.oaddr != decisionMatrix[i].oaddr)) continue;
        if ( decisionMatrix[i].flags & VSCP_DM_FLAG_CHECK_ZONE  ) { // Check if zone should match and if so if it match
            if ((255 != vscp_imsg.data[ 1 ]) & (vscp_imsg.data[ 1 ] != vscp_zone )) continue;
        }
        class_filter = ( decisionMatrix[i].flags & VSCP_DM_FLAG_CLASS_FILTER )*256 + decisionMatrix[i].class_filter;
        class_mask = ( (decisionMatrix[i].flags & VSCP_DM_FLAG_CLASS_MASK) >0 )*256 + decisionMatrix[i].class_mask;
        if ( !((class_filter ^ vscp_imsg.vscp_class) & class_mask) &&
             !( ( decisionMatrix[i].type_filter ^ vscp_imsg.vscp_type ) & decisionMatrix[i].type_mask )) {
            switch ( decisionMatrix[i].action ) { // OK Trigger this action
                
                default:
                    doApplicationDM(i);
                    break;
            } // case
        } // Mask and Filter matched
    } // for each row
}

void vscp_loadAllFromEEPROM(){
    vscp_zone = eeprom_read(VSCP_EEPROM_ZONE);
    vscp_loadDecisionMatrixFromEEPROM();
}
void vscp_loadDecisionMatrixFromEEPROM(){
    for (char i=0; i<VSCP_DM_SIZE; i++){
        decisionMatrix[i].oaddr = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 0);
        decisionMatrix[i].flags = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 1);
        decisionMatrix[i].class_mask = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 2);
        decisionMatrix[i].class_filter = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 3);
        decisionMatrix[i].type_mask = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 4);
        decisionMatrix[i].type_filter = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 5);
        decisionMatrix[i].action = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 6);
        decisionMatrix[i].action_param = eeprom_read(VSCP_DM_EEPROM_START_LOC + 8*i + 7);
    }
}
void vscp_saveDecisionMatrixToEEPROM(){
    for (char i=0; i<VSCP_DM_SIZE; i++){
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 0, decisionMatrix[i].oaddr);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 1, decisionMatrix[i].flags);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 2, decisionMatrix[i].class_mask);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 3, decisionMatrix[i].class_filter);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 4, decisionMatrix[i].type_mask);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 5, decisionMatrix[i].type_filter);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 6, decisionMatrix[i].action);
         eeprom_write(VSCP_DM_EEPROM_START_LOC + 8*i + 7, decisionMatrix[i].action_param);
    }
}

int8_t sendVSCPFrame( uint16_t vscpclass, uint8_t vscptype, uint8_t nodeid,
		      uint8_t priority, uint8_t size, uint8_t *pData ){
	uint32_t id = ( (uint32_t)priority << 26 ) |
			( (uint32_t)vscpclass << 16 ) |
			( (uint32_t)vscptype << 8) |
			nodeid;		// nodeaddress (our address)
        uint8_t maxTxTrial = 0;
	while(!ECANSendMessage(id, pData, size, ECAN_TX_XTD_FRAME) & 
                !ECANIsBusOff() &
                maxTxTrial<VSCP_TX_MAX_TRIAL) {
		// Failed to send message
		__delay_ms(1);
		maxTxTrial++;
	}
        if (!ECANIsBusOff() & maxTxTrial<VSCP_TX_MAX_TRIAL){
            return TRUE;
        }else{
            vscp_errorcnt++;
            return FALSE;
        }
}


int8_t getVSCPFrame( uint16_t *pvscpclass, uint8_t *pvscptype, uint8_t *pNodeId,
                     uint8_t *pPriority, uint8_t *pSize, uint8_t *pData ){
	uint32_t id;
        ECAN_RX_MSG_FLAGS flags;
	// Dont read in new message if there already is a message
	// in the input buffer
        
	if ( vscp_imsg.flags & VSCP_VALID_MSG ) return TRUE;

	if ( ECANReceiveMessage(&id, pData, pSize,  &flags) ) {
            if ( flags == ECAN_RX_RTR_FRAME ) return FALSE; // RTR not interesting
            if ( flags != ECAN_RX_XTD_FRAME ) return FALSE; // Must be extended frame
            *pNodeId = id & 0x0ff;
            *pvscptype = ( id >> 8 ) & 0xff;
            *pvscpclass = ( id >> 16 ) & 0x1ff;
            *pPriority = (uint16_t)( 0x07 & ( id >> 26 ) );
            return TRUE;
	}
	return FALSE;
}

void sendDMatrixInfo( void ){
/*	vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
	vscp_omsg.flags = VSCP_VALID_MSG + 2;
	vscp_omsg.vscp_class = VSCP_CLASS1_PROTOCOL;
	vscp_omsg.vscp_type = VSCP_TYPE_PROTOCOL_GET_MATRIX_INFO_RESPONSE;

	vscp_omsg.data[ 0 ] = DESCION_MATRIX_ELEMENTS;
	vscp_omsg.data[ 1 ] = REG_DESCION_MATRIX;

	vscp_sendEvent();	// Send data
 */
}


///////////////////////////////////////////////////////////////////////////////
//                        VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////


void vscp_getMatrixInfo(char *pData){
/*!
        Get DM matrix info
        The output message data structure should be filled with
        the following data by this routine.
        byte 0 - Number of DM rows. 0 if none.
        byte 1 - offset in register space.
        byte 2 - start page MSB
        byte 3 - start page LSB
        byte 4 - End page MSB
        byte 5 - End page LSB
        byte 6 - Level II size of DM row (Just for Level II nodes).
 */
    *(pData+0) = VSCP_DM_COUNT;
    *(pData+1) = VSCP_REG_DM_OFFSET;
    *(pData+2) = (char)((VSCP_REG_DM_PAGE >> 8 ) & 0xFF);
    *(pData+3) = (char)(VSCP_REG_DM_PAGE & 0xFF);
    *(pData+4) = (char)((VSCP_REG_DM_PAGE >> 8 ) & 0xFF);
    *(pData+5) = (char)(VSCP_REG_DM_PAGE & 0xFF);
}


void vscp_goBootloaderMode( uint8_t algorithm){
/*!
        Go bootloader mode
        This routine force the system into bootloader mode according
        to the selected protocol.
 */
    if (algorithm!=APP_BOOTLOADER) return;
    eeprom_write( VSCP_EEPROM_BOOTLOADER_FLAG, VSCP_BOOT_FLAG );
    asm("reset");
}

uint8_t vscp_getZone(void){ return vscp_zone;}
uint8_t vscp_getSubzone(void){ return 0;}
/*!
        Write control byte permanent storage
 */
#ident("ControlByte is not used in this ")
void vscp_setControlByte(uint8_t ctrl){return;}
/*!
        Fetch nickname from permanent storage
        @return read nickname.
 */
uint8_t vscp_readNicknamePermanent(void){ return eeprom_read(VSCP_EEPROM_NICKNAME); }
void vscp_writeNicknamePermanent(uint8_t nickname){ eeprom_write(VSCP_EEPROM_NICKNAME, nickname); }
/*!
        Fetch segment CRC from permanent storage
 */
uint8_t vscp_getSegmentCRC(void){ return eeprom_read(VSCP_EEPROM_SEGMENT_CRC); }
void vscp_setSegmentCRC(uint8_t crc){ eeprom_write(VSCP_EEPROM_SEGMENT_CRC, crc); }
/*!
        Get URL from device from permanent storage
        index 0-15
 */
uint8_t vscp_getMDF_URL(uint8_t idx){ return mdfLink[idx];}
uint8_t vscp_getGUID(uint8_t idx){
    if (idx<7) // FF FF FF FF FF FF FF FC xx xx xx xx xx xx xx xx
        return 0xFF;
    else if (idx==7)
        return 0xFC;
    else if (idx<16)
        return GuID_LSB[idx-8];
    else
        return 0;
}
#ident("UserID is not handled in this project")
uint8_t vscp_getUserID(uint8_t idx){ return 0;}
void vscp_setUserID(uint8_t idx, uint8_t data){}
#ident("Manufacturer ID is not used in this project")
uint8_t vscp_getManufacturerId(uint8_t idx){ return 0; }
uint32_t vscp_getFamilyCode(void){return DEVICE_CODE;}
uint32_t vscp_getFamilyType(void){return DEVICE_TYPE;}
uint8_t vscp_getBootLoaderAlgorithm(void){ return APP_BOOTLOADER;}
uint8_t vscp_getMajorVersion(void){ return APP_VARIANT;}
uint8_t vscp_getMinorVersion(void){ return APP_VERSION;}
uint8_t vscp_getSubMinorVersion(void){ return APP_SUB_VERSION;}
/*!
        Get number of register pages used by app.
 */
uint8_t vscp_getRegisterPagesUsed(void){ return APP_REG_PAGES;}
uint8_t vscp_getPageSelect(uint8_t idx){
/*!
        Get page select bytes
                idx=0 - byte 0 MSB
                idx=1 - byte 1 LSB
 */
    if (idx==0)
        return (uint8_t)((vscp_registerPage>>8)&0xFF);
    else
        return (uint8_t)(vscp_registerPage&0xFF);
}
void vscp_setPageSelect(uint8_t idx, uint8_t data){
    uint16_t temp = vscp_registerPage;
    if (idx==0)
        temp = temp & 0x00FF + (data<<8);
    else
        temp = temp & 0xFF00 + data;
    if(temp<=APP_REG_PAGES) vscp_registerPage = temp;
}
uint8_t vscp_getBufferSize(void){ return 8; } //Level1 size