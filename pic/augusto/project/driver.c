#include "driver.h"
#include <vscp_firmware.h>
#include <vscp_class.h>
#include <vscp_type.h>
#include <xc.h>
#include <ECANPoll.h>

//Definition of I/O
volatile unsigned char *IN_PIN_PORT[PIN_IN_SIZE] = {&PORTD, &PORTD, &PORTD, &PORTD, &PORTB, &PORTB, &PORTB, &PORTA};
char IN_PIN_NUM[PIN_IN_SIZE] = {0x10, 0x20, 0x40, 0x80, 0x01, 0x02, 0x10, 0x10};
volatile unsigned char *OUT_PIN_PORT[PIN_OUT_SIZE] = {&LATC, &LATD, &LATD, &LATD, &LATD, &LATC, &LATC, &LATC};
char OUT_PIN_NUM[PIN_OUT_SIZE] = {0x20, 0x08, 0x04, 0x02, 0x01, 0x04, 0x02, 0x01};

/* DEFINITION OF VARIABLES */
struct vscpBoard_inputVar hardware_input[PIN_IN_SIZE];
struct vscpBoard_outputVar hardware_output[PIN_OUT_SIZE];
uint8_t hardware_zoneForInput[PIN_IN_SIZE];
uint8_t hardware_subzoneForInput[PIN_IN_SIZE];
uint8_t hardware_subzoneForOutput[PIN_OUT_SIZE];
timeBasedEventStruct timeEvent, timeOverride;

void TMR0_setup();//Internal usage


void hardware_sendInputInformation(int8_t idx, uint8_t type){
    vscp_omsg.vscp_class = VSCP_CLASS1_INFORMATION;
    vscp_omsg.priority = 3;
    vscp_omsg.flags = VSCP_VALID_MSG + 3;
    vscp_omsg.data[0] = 0;
    vscp_omsg.data[1] = hardware_zoneForInput[idx];
    vscp_omsg.data[2] = hardware_subzoneForInput[idx];
    switch(type){
        case VSCP_TYPE_INFORMATION_ON:
            if (hardware_input[idx].onEvent){
                if (hardware_input[idx].doorLogic)
                    vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_OPENED;
                else
                    vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_ON;
                doDM(TRUE);
                vscp_sendEvent();
            }
            break;
        case VSCP_TYPE_INFORMATION_OFF:
            if (hardware_input[idx].offEvent){
                if (hardware_input[idx].doorLogic)
                    vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_CLOSED;
                else
                    vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_OFF;
                doDM(TRUE);
                vscp_sendEvent();
            }
            break;
        case VSCP_TYPE_INFORMATION_BUTTON:
            if(hardware_input[idx].buttonEvent){
                vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_BUTTON;
                if(hardware_input[idx].debounce>=HARDWARE_LONG_DEBOUNCE_THRESOLD)
                   vscp_omsg.data[0] = 2;
                doDM(TRUE);
                vscp_sendEvent();
            }
            break;
    }
}

void hardware_10mS(){
    uint8_t newState;
    for (uint8_t i = 0; i < PIN_IN_SIZE; i++){
        //Electrical signal is working in negate logic
        newState = (((*(IN_PIN_PORT[i]) & IN_PIN_NUM[i]) == 0)^ hardware_input[i].reversedLogic );
        
        if (hardware_input[i].currentStatus == 0){//OFF or ON-Debouncing State
            if(newState == 1){
                if(hardware_input[i].debounce++ == HARDWARE_SHORT_DEBOUNCE_THRESOLD){
                    hardware_sendInputInformation(i, VSCP_TYPE_INFORMATION_ON);//ON EVENT
                    hardware_input[i].currentStatus=1;
                }
            }else
                hardware_input[i].debounce = 0; //Reset debounce
            
        }else{ //ON or Off-Deboncing or LongON
            if(hardware_input[i].debounce>HARDWARE_SHORT_DEBOUNCE_THRESOLD){ //ON
                if(newState == 0){
                    hardware_sendInputInformation(i, VSCP_TYPE_INFORMATION_BUTTON);//SHORT EVENT
                    hardware_input[i].debounce=0;
                }else if(hardware_input[i].debounce++ == HARDWARE_LONG_DEBOUNCE_THRESOLD){
                    hardware_sendInputInformation(i, VSCP_TYPE_INFORMATION_BUTTON);//LONG EVENT
                    hardware_input[i].debounce=0;
                }
            }else if(newState==1){
                hardware_input[i].debounce = 0; //Reset debounce
            }else{
                if(hardware_input[i].debounce++ == HARDWARE_SHORT_DEBOUNCE_THRESOLD){
                    hardware_sendInputInformation(i, VSCP_TYPE_INFORMATION_OFF);//OFF EVENT
                    hardware_input[i].currentStatus=0;
                }
            }
        }
    }
}
void hardware_setOutput (unsigned char pin, unsigned char state){
    if (pin>PIN_OUT_SIZE) return;
    state = state & 0x01;
    //Electrical info is working in negate logic
    if ((state == 0) ^ hardware_output[pin].reversedLogic)
        *(OUT_PIN_PORT[pin]) |= OUT_PIN_NUM[pin];
    else{
        *(OUT_PIN_PORT[pin]) &= (0xFF ^ OUT_PIN_NUM[pin]);
    }
    if (state != hardware_output[pin].currentStatus){
        vscp_omsg.vscp_class = VSCP_CLASS1_INFORMATION;
        vscp_omsg.priority = 3;
        vscp_omsg.flags = VSCP_VALID_MSG + 3;
        vscp_omsg.data[0]= 0;
        vscp_omsg.data[1] = vscp_zone;
        vscp_omsg.data[2] = hardware_subzoneForOutput[pin];
        if (hardware_output[pin].onEvent & state){
            vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_ON;
            vscp_sendEvent();
        }else if (hardware_output[pin].offEvent & (state==0)){
            vscp_omsg.vscp_type = VSCP_TYPE_INFORMATION_OFF;
            vscp_sendEvent();
        }
        hardware_output[pin].currentStatus = state;
    }

}
void TMR0_interrupt(){
    static char time[2] = {0, 0};
    if (INTCONbits.TMR0IF){ //10mS Event
        INTCONbits.TMR0IF = 0;
        char temp = TMR0L;
        TMR0H = TMR0H + TMR0H_INIT;
        TMR0L += temp + TMR0L_INIT+1;
        vscp_timer+=10; //Timer of vscp firmware
        if (timeEvent._10mS == 1) timeOverride._10mS = 1; else timeEvent._10mS = 1;
        if((++time[0])>=10){ //100mS Event
            time[0] = 0;
            if (timeEvent._100mS == 1) timeOverride._100mS = 1; else timeEvent._100mS = 1;
            if((++time[1])>=10){ //1s Event
                time[1] = 0;
                if (timeEvent._1s == 1) timeOverride._1s = 1; else timeEvent._1s = 1;
            }
        }

    }
}

/* HARDWARE INITIALIZATION */
void hardware_setup(){
    /*  PORTA
     * RA0 --> TEMP_INT
     * RA1 --> TEMP_EXT
     * RA2 --> VSCP LED
     * RA3 --> Vref
     * RA4 --> INPUT6
     * RA5 --> ANALOG3
     *
     *   PORTB
     * RB0 --> INPUT5
     * RB1 --> INPUT4
     * RB2 --> CAN_TXD
     * RB3 --> CAN_RXD
     * RB4 --> INPUT7
     * RB5 --> VSSCP BUTTON
     * RB6 --> ISCP PGC
     * RB7 --> ISCP PGD
     *
     *   PORTC
     * RC0 --> OUTPUT4
     * RC1 --> OUTPUT5
     * RC2 --> OUTPUT6
     * RC3 --> I2C SCL
     * RC4 --> I2C SDA
     * RC5 --> OUTPUT0
     * RC6 --> LED RED
     * RC7 --> LED GREEN
     *
     *   PORTD
     * RD0 --> OUTPUT7
     * RD1 --> OUTPUT3
     * RD2 --> OUTPUT2
     * RD3 --> OUTPUT1
     * RD4 --> INPUT0
     * RD5 --> INPUT1
     * RD6 --> INPUT2
     * RD7 --> INPUT3
     *
     *   PORTE
     * RE0 --> ANALOG2
     * RE1 --> ANALOG1
     * RE2 --> ANALOG0
     */
    ADCON1 = 0x17; //ADC Configuration
    ADCON0 = 0x01;
    CMCON = 0x07;//Disable comparator
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    TRISA = 0xFF;
    TRISB = 0x3F;
    TRISC = 0xFF;
    TRISD = 0xFF;
    TRISE = 0x0F;

    //Leds and button configuration
    vscp_ledTris = 0;
    vscp_btnTris = 1;
    redLed_tris = 0;
    greenLed_tris = 0;

    vscp_ledPin = 0;
    greenLed_pin = 1;
    redLed_pin = 1;

    // I/O configuration
    for (char i=0; i<PIN_IN_SIZE; i++)
        *(IN_PIN_PORT[i] + OFFSET_PORT_TO_TRIS) |= IN_PIN_NUM[i]; //offset is used to access to TRIS register

    for (char i=0; i<PIN_OUT_SIZE; i++)
        *(OUT_PIN_PORT[i] + OFFSET_LAT_TO_TRIS) &= (0xFF ^ OUT_PIN_NUM[i]); //offset is used to access to TRIS register
    
    hardware_loadEEPROM();
    for (uint8_t i=0; i<PIN_OUT_SIZE; i++){
        hardware_output[i].currentStatus = 0;
        if(hardware_output[i].reversedLogic)
            *(OUT_PIN_PORT[i]) &= (0xFF ^ OUT_PIN_NUM[i]);
        else
            *(OUT_PIN_PORT[i]) |= OUT_PIN_NUM[i];
    }


    ECANInitialize();
    TMR0_setup();
    ei(); 
}
void TMR0_setup(){
/* Timer0 Configuration
 *
 * Target = interrupt each 10mS
 * Fosc = 32MHz --> Counter 80'000 ticks (input Fosc/4)
 * Prescaler 1:2, TMR0 = 0xFFFF - 80000d = 0x63C0
 *
 * If the TMR0L register is written, the increment is inhibited for the
 * following two instruction cycles. The user can work
 * around this by writing an adjusted value to the TMR0L register.
 */
    T0CONbits.T08BIT = 0;
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T0PS2 = 0;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    TMR0H = TMR0H_INIT;
    TMR0L = TMR0L_INIT;
    T0CONbits.TMR0ON = 1;
    INTCONbits.TMR0IF = 0;
    INTCON2bits.TMR0IP = 1;
    INTCONbits.TMR0IE = 1;
}




/* REGISTER MAP
 * This part of code uses all first page of VSCP register dedicated to application 0x00 to 0x7F
 * Allocation is dynamic and it depends on PIN_OUT_SIZE and PIN_IN_SIZE
 *
 * Rules used to maps the registers are:
 * R- FROM                            0 TO                  PIN_IN_SIZE -1 --> Status of input (according to reversedLogic)
 * RW FROM                  PIN_IN_SIZE TO                2*PIN_IN_SIZE -1 --> Input Status byte
 * RW FROM                2*PIN_IN_SIZE TO                3*PIN_IN_SIZE -1 --> Zone for input
 * RW FROM                3*PIN_IN_SIZE TO                4*PIN_IN_SIZE -1 --> Subzone for input
 * RW FROM                4*PIN_IN_SIZE TO   4*PIN_IN_SIZE+PIN_OUT_SIZE -1 --> Status of output (according to reversedLogic)
 * RW FROM 4*PIN_IN_SIZE+  PIN_OUT_SIZE TO 4*PIN_IN_SIZE+2*PIN_OUT_SIZE -1 --> Output Status byte
 * RW FROM 4*PIN_IN_SIZE+2*PIN_OUT_SIZE TO 4*PIN_IN_SIZE+3*PIN_OUT_SIZE -1 --> Subzone for Output
 *
 * 0x7F save RAM in EEPROM (when 1 is written) --> should used for all modules
 */

#if 4*PIN_IN_SIZE+3*PIN_OUT_SIZE > 126
#error("No enough space for VSCP register mapping")
#endif

uint8_t hardware_readRegister(uint8_t address){
    if (address<PIN_IN_SIZE) return hardware_input[address].currentStatus;
    else if (address<2*PIN_IN_SIZE) return hardware_saveStructForInput(hardware_input[address-PIN_IN_SIZE]);
    else if (address<3*PIN_IN_SIZE) return hardware_zoneForInput[address-2*PIN_IN_SIZE];
    else if (address<4*PIN_IN_SIZE) return hardware_subzoneForInput[address-3*PIN_IN_SIZE];
    else if (address<(4*PIN_IN_SIZE+PIN_OUT_SIZE))   return hardware_output[address-4*PIN_IN_SIZE].currentStatus;
    else if (address<(4*PIN_IN_SIZE+2*PIN_OUT_SIZE)) return hardware_saveStructForOutput(hardware_output[address-4*PIN_IN_SIZE-PIN_OUT_SIZE]);
    else if (address<(4*PIN_IN_SIZE+3*PIN_OUT_SIZE)) return hardware_subzoneForOutput[address-4*PIN_IN_SIZE-2*PIN_OUT_SIZE];
    else return 0;
}
/*!
        Write application register (lower part)
        @param reg Register to read (<0x80)
        @param value Value to write to register.
        @return Register content or 0xff for non valid register
 */
uint8_t hardware_writeRegister(uint8_t address, uint8_t value){
    if (address<PIN_IN_SIZE); //Ignored because it's not possible to override an input
    else if (address<2*PIN_IN_SIZE) hardware_loadStructForInput(&(hardware_input[address-PIN_IN_SIZE]), value);
    else if (address<3*PIN_IN_SIZE) hardware_zoneForInput[address-2*PIN_IN_SIZE] = value;
    else if (address<4*PIN_IN_SIZE) hardware_subzoneForInput[address-3*PIN_IN_SIZE] = value;
    else if (address<(4*PIN_IN_SIZE+PIN_OUT_SIZE)) hardware_setOutput(address-4*PIN_IN_SIZE, value & 0x01);
    else if (address<(4*PIN_IN_SIZE+2*PIN_OUT_SIZE)){
        hardware_loadStructForOutput(&(hardware_output[address-4*PIN_IN_SIZE-PIN_OUT_SIZE]), value & 0xFE);
        hardware_setOutput(address-4*PIN_IN_SIZE, value & 0x01);
    }
    else if (address<(4*PIN_IN_SIZE+3*PIN_OUT_SIZE)) hardware_subzoneForOutput[address-4*PIN_IN_SIZE-2*PIN_OUT_SIZE] = value;
    else return 0xFF;
    return value;
}

/*  EEPROM LAYOUT
 *
 *  Config input: 1byte x PIN_IN_SIZE
 *  Zone for input: 1 byte x PIN_IN_SIZE
 *  Subzone for input: 1byte x PIN_IN_SIZE
 *  Config output: 1byte x PIN_OUT_SIZE
 *  Subzone for output: 1byte x PIN_OUT_SIZE
 *
 */
void hardware_saveEEPROM(){
    for (uint8_t i=0; i<PIN_IN_SIZE; i++){
        eeprom_write(VSCP_BOARD_EEPROM_START+i, hardware_saveStructForInput(hardware_input[i]));
        eeprom_write(VSCP_BOARD_EEPROM_START+i+PIN_IN_SIZE, hardware_zoneForInput[i]);
        eeprom_write(VSCP_BOARD_EEPROM_START+i+2*PIN_IN_SIZE, hardware_subzoneForInput[i]);
    }
    for (uint8_t i=0; i<PIN_OUT_SIZE; i++){
        eeprom_write(VSCP_BOARD_EEPROM_START+i+3*PIN_IN_SIZE, hardware_saveStructForOutput(hardware_output[i]));
        eeprom_write(VSCP_BOARD_EEPROM_START+i+3*PIN_IN_SIZE+PIN_OUT_SIZE, hardware_subzoneForOutput[i]);
    }
}
void hardware_loadEEPROM(){
    for (uint8_t i=0; i<PIN_IN_SIZE; i++){
        hardware_loadStructForInput(&(hardware_input[i]),eeprom_read(VSCP_BOARD_EEPROM_START+i));
        hardware_zoneForInput[i] = eeprom_read(VSCP_BOARD_EEPROM_START+i+PIN_IN_SIZE);
        hardware_subzoneForInput[i] = eeprom_read(VSCP_BOARD_EEPROM_START+i+2*PIN_IN_SIZE);
    }
    for (uint8_t i=0; i<PIN_OUT_SIZE; i++){
        hardware_loadStructForOutput(&(hardware_output[i]),eeprom_read(VSCP_BOARD_EEPROM_START+i+3*PIN_IN_SIZE));
        hardware_subzoneForOutput[i] = eeprom_read(VSCP_BOARD_EEPROM_START+i+3*PIN_IN_SIZE+PIN_OUT_SIZE);
    }
}

/* Methods to save and load I/O Configuration */
uint8_t hardware_saveStructForOutput(struct vscpBoard_outputVar out){
    uint8_t temp; temp=0;
    if (out.currentStatus) temp |=0x01;
    if (out.reversedLogic) temp |= 0x02;
    if (out.offEvent) temp |= 0x40;
    if (out.onEvent) temp |= 0x80;
    return temp;
}
void hardware_loadStructForOutput(struct vscpBoard_outputVar *out, uint8_t value){
    (*out).reversedLogic = ((value & 0x02)>0);
    (*out).offEvent = ((value & 0x20)>0);
    (*out).onEvent = ((value & 0x40)>0);
}
uint8_t hardware_saveStructForInput(struct vscpBoard_inputVar in){
    uint8_t temp; temp=0;
    if (in.reversedLogic) temp |= 0x02;
    if (in.doorLogic) temp |= 0x10;
    if (in.offEvent) temp |= 0x20;
    if (in.onEvent) temp |= 0x40;
    if (in.buttonEvent) temp |= 0x80;
    return temp;
}
void hardware_loadStructForInput(struct vscpBoard_inputVar *in, uint8_t value){
    (*in).reversedLogic = ((value & 0x02)>0);
    (*in).doorLogic = ((value & 0x10)>0);
    (*in).offEvent = ((value & 0x20)>0);
    (*in).onEvent = ((value & 0x40)>0);
    (*in).buttonEvent = ((value & 0x80)>0);
    (*in).debounce = 0;
}


