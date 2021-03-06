/* 
 * File:   driver.h
 * Author: Musumeci Salvatore
 *
 * Aim of this module is to provided an abstrated layer for your VSCP application.
 * This part of code is demanded to inteface the HW with your code, ignoring the HW variant
 *
 * For example, if you have to hardware variant where same pin is connected in two different pin, your code will not change
 * Will be this interface that will be recompiled in order to support the change.
 *
 * Also, this interface is able to handling the CAN message for input
 * Obviously all parameters could be customized using normal VSCP register access
 * Created on 10 agosto 2014, 2.47
 */

#ifndef DRIVER_H
#define	DRIVER_H

#include <xc.h>
#include <inttypes.h>
#include <vscp_projdefs.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define VSCP_BOARD_EEPROM_START 0x08
#define VSCP_BOARD_EEPROM_LENGTH 3*PIN_IN_SIZE + 2*PIN_OUT_SIZE

#define OFFSET_PORT_TO_TRIS 0x12
#define OFFSET_LAT_TO_TRIS  0x09
#define TMR0H_INIT 0x63
#define TMR0L_INIT 0xC0

#define HARDWARE_SHORT_DEBOUNCE_THRESOLD  4
#define HARDWARE_LONG_DEBOUNCE_THRESOLD 100
#if (HARDWARE_LONG_DEBOUNCE_THRESOLD > 255)
    #error("Thresold is too big")
#elif(HARDWARE_LONG_DEBOUNCE_THRESOLD <= HARDWARE_SHORT_DEBOUNCE_THRESOLD)
    #error("Short Threshold must be smaller than Long Threshold")
#endif

#define HARDWARE_BOARD_TEMPERATURE 0
#define HARDWARE_EXTERNAL_TEMPERATURE 1

#define HARDWARE_ADC_MIN_VALUE  0x10
#define HARDWARE_ADC_MAX_VALUE 0x3F0

//VSCP button and led definition
#define vscp_ledPin   PORTAbits.RA2
#define vscp_ledTris  TRISAbits.TRISA2
#define vscp_btnPin   PORTBbits.RB5
#define vscp_btnTris  TRISBbits.TRISB5
#define greenLed_pin  PORTCbits.RC7
#define greenLed_tris TRISCbits.TRISC7
#define redLed_pin    PORTCbits.RC6
#define redLed_tris   TRISCbits.TRISC6

#define PICKIT_CH1 PORTBbits.RB7
#define PICKIT_CH2 PORTBbits.RB6

typedef struct {
    unsigned  _10mS :1;
    unsigned _100mS :1;
    unsigned    _1s :1;
}timeBasedEventStruct;

extern struct vscpBoard_inputVar hardware_input[PIN_IN_SIZE];
extern struct vscpBoard_outputVar hardware_output[PIN_OUT_SIZE];
extern uint8_t hardware_zoneForInput[PIN_IN_SIZE];
extern uint8_t hardware_subzoneForInput[PIN_IN_SIZE];
extern uint8_t hardware_subzoneForOutput[PIN_OUT_SIZE];
extern uint8_t vscp_zone;

struct vscpBoard_inputVar{
/* DEFINITION OF STATUS / CONFIGURATION BYTE OF INPUT PIN
 * bit 0: currentStatus
 * bit 1: Reversed logic
 * bit 2: Reserved for future use ****
 * bit 3: Reserved for future use ****
 * bit 4: 0=Button/switch, 1=door/window switch
 * bit 5: OFF/ Closed event is required
 * bit 6: ON / Opened event is required
 * bit 7: Button event is required
 */
    unsigned currentStatus: 1;
    unsigned reversedLogic: 1;
    unsigned doorLogic: 1;
    unsigned offEvent: 1;
    unsigned onEvent: 1;
    unsigned buttonEvent: 1;
    unsigned debounce: 8; //For debounce time
};
struct vscpBoard_outputVar{
/* DEFINITION OF STATUS / CONFIGURATION BYTE OF OUTPUT PIN
 * bit 0: currentStatus
 * bit 1: Reversed logic
 * bit 2: Reserved for future use ****
 * bit 3: Reserved for future use ****
 * bit 4: Reserved for future use ****
 * bit 5: OFF is required
 * bit 6: ON is required
 * bit 7: Reserved for future use ****
 */
    unsigned currentStatus: 1;
    unsigned reversedLogic: 1;
    unsigned offEvent: 1;
    unsigned onEvent: 1;
};
struct vscpBoard_temperature{
   /* DEFINITION OF TEMPERATURE STRUCTURE
    * 
    * current_value = (raw - offset) * conversionFactor
    * 
    * filtered_value = filterCoefficient*filter_value +
    *                  (1 - filterCoefficent) * current_value
    */ 
    uint8_t scheduleTime; //How often send message (1S based)
    uint8_t currentScheduleTime; //Counter
    uint8_t offset; //Electrical offset
    float conversionFactor;
    float filterCoefficient;
    float value;
    BOOL fault;
};

void hardware_setup();
void hardware_10mS();
void hardware_100mS(BOOL init);
void hardware_loadEEPROM();
void hardware_saveEEPROM();
uint8_t hardware_saveStructForInput(struct vscpBoard_inputVar in);
uint8_t hardware_saveStructForOutput(struct vscpBoard_outputVar out);
void hardware_loadStructForOutput(struct vscpBoard_outputVar *out, uint8_t value);
void hardware_loadStructForInput(struct vscpBoard_inputVar *in, uint8_t value);

void hardware_setOutput (unsigned char pin, unsigned char state);
uint8_t getInput (unsigned char pin);
void TMR0_interrupt();

uint8_t hardware_writeRegister(uint8_t address, uint8_t value);
uint8_t hardware_readRegister(uint8_t address);

extern void doDM(BOOL oMsg);
#ifdef	__cplusplus
}
#endif

#endif	/* DRIVER_H */

