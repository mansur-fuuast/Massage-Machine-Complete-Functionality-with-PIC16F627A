# Massage Machine PIC MCU Programmming
<h2>Hardware Configuration:</h2> The circuit comprises 1 PIC16F627A, 04 SPST Buttons, 03 LEDs, and 04 Motors. Buttons and LEDs are part of a remote control used to control various modes and ON/OFF of device. Four Buttons are connected in 2 x 2 Matrix to Pins RA2, RA3, RA4 & RB0. RA4 & RB0 are connected to VDD via external pull up resistors of 10K each. AUTO button is connected to RA2 & RB0. FULL button is connected to RA2 & RA4. STEP button is connected to RA3 & RB0. STOP button is connected to RA3 & RA4. Three LEDs Indicators' positive terminals are connected to VDD while negative terminals are connected to Pins RB1, RB2 & RB3 via 460Ohm resistors. Four Motors' positive terminals are connected to 12VDC power. While negative terminals are connected to emitters of 2N2222 transistors. CollectorS of transistors are grounded while base are driven by Pins RB4, RB5, RB6, and RB7 via 4.5K resistors.
 <img width="562" height="211" alt="image" src="https://github.com/user-attachments/assets/15f6e7f9-80d4-4e1a-bf8e-681a941e15c6" />

<h2>Functional Description:</h2> We need to develop a code for following functionality:

<b>Mode Auto:</b> On Auto Button Click, 

•	AUTO LED (RB1) must get ON and remain ON until next mode change. 

•	Motors 1 & 4 Switch ON for 670ms with 41 pulses (PWM), then Motors 2 & 3 switch ON for same no of time and pulses and so on until mode is changed. 

<b>Mode Step:</b> On STEP Button Click, 

•	STEP LED (RB2) must get ON and remain ON until next mode change. 

•	Motors 1 & 2 Switch ON for 670ms with 41 pulses (PWM), then Motors 3 & 4 switch ON for same no of time and pulses and so on until mode is changed. 

<b>Mode Full:</b> FULL mode has two further modes. 

•	On Full Button 1st Click 

  o	FULL LED (RB3) must get ON and remain ON until next mode change. 

  o	All Motors Switch ON simultaneously with independent array of ON time, pulses and OFF time as given below:
    
    	Motor 1: ON 190ms with 11 pulses, OFF 30ms, ON 70ms with 5 pulses, OFF 218ms, ON 146ms with 9 pulses, OFF 236ms, ON  74ms with 5 pulses, OFF 300ms, ON 78ms with 5 pulses, OFF 120ms, ON 70ms with 4 pulses, OFF 120ms, ON 70ms with 4 pulses, OFF 370ms (TOTAL 2092ms ), and so on. 
    
    	Motor 2: ON 70ms with 5 pulses, OFF 130ms, ON 140ms with 9 pulses, OFF 230ms, ON 140ms with 9 pulses, OFF 800ms, ON 70ms with 5 pulses, OFF 120ms, ON 70ms with 4 pulses, OFF 120ms, ON 70ms with 5 pulses, OFF 115ms, (TOTAL 2075) and so on. 
    
    	Motor 3: ON 190ms with 11 pulses, OFF 30ms, ON 75ms with 5 pulses, OFF 215ms, ON 140ms with 9 pulses, OFF 236ms, ON 70ms with 4 pulses, OFF 115ms, ON 70ms with 5 pulses, OFF 115ms, ON 70ms with 4 pulses, OFF 115ms, ON 70ms with 4 pulses, OFF 560ms (TOTAL 2071ms ), and so on. 
    
    	Motor 4: ON 70ms with 5 pulses, OFF 30ms, ON 80ms with 5 pulses, OFF 218ms, ON 140ms with 9 pulses, OFF 234ms, ON 80ms with 5 pulses, OFF 110ms, ON 75ms with 5 pulses, OFF 670ms, ON 75ms with 5 pulses, OFF 300ms, (TOTAL 2082ms) and so on. 

•	On Subsequent 2nd Click on Full Button When Machine is already in Full Mode 
  
  o	FULL LED (RB3) must remain ON and remain ON until next mode change. Motors operate in following pattern, 
    
    	Motor 1: ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 1300ms (TOTAL 2340ms), and so on. 
    
    	Motor 2: OFF 930ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 370ms, (TOTAL 2340ms) and so on. 
    
    	Motor 3: OFF 130ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 1300ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 60ms (TOTAL 2340ms) and so on. 
    
    	Motor 4: 440ms OFF, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 190ms, ON 120ms with 8 pulses, OFF 180ms, ON 120ms with 8 pulses, OFF 8600ms, (TOTAL 2340ms) and so on. 

<h1>Implementation</h1>
Written following Code in MPLab X IDE and generated the HEX code for uploading to PIC16F627A: Also, include scan_buttons.asm and auto_step_modes.asm 
main.c

```
/*
 * File:        main.c
 * Author:      M Mansoor Ahmed
 * E-mail:      mansoor.ahmed@fuuast.edu.pk, 
 *              m.mansoor.ahmed@outlook.com
 * Whatsapp:    +92-313-123-3939
 * Created on:  August 17, 2025, 1:25 AM
 *
 * NOTE: Kindly include scan_buttons.asm and auto_step_modes.asm as source   * files in the MPLab Project to comile.
 *
 * This file is written for PIC16F627A micro-controller of a massage machine. It 
 * controls the operation of four 12VDC Motors powered through an external 
 * circuit while driven to logic level LOW at the other terminal by 2N2222 
 * transistors. The Transistor's base is connected to PIC micro-controller's 
 * Pins RB4, RB5, RB6 and RB7 via 4.5K Ohms resistors. Motors are controlled by 
 * PIC micro-controller via a remote control that have four SPST switches and 03
 * LEDs. Full, Auto, Step and Stop switches are connected to PIC 
 * micro-controller in 2x2 grid like calculator keypads. Pins are RA2, RA3, RA4
 * and RB0. While three LEDs indicating current operational mode are Full, Auto
 * and Step. Positive Terminal of LEDs' are connected to VDD while negative 
 * terminal is driven by PIC micro-controller via 330Ohms Resistors.
 * The program constantly listen for switches press every 20ms and switches to 
 * the corresponding mode. 
 * 
 * AUTO MODE:   When auto mode switch is pressed the PIC micro-controller 
 * switches ON AUTO MODE LED indicator and motors start operating in AUTO mode 
 * (Motor 1 & 4 ON for 700ms then Motor 2 & 3 ON for 700ms and so on) until stop
 * switch is pressed or another mode is switched.
 * 
 * FULL MODE:   When full mode switch is pressed the PIC micro-controller 
 * switches ON FULL MODE LED indicator and motors start operating in FULL mode 
 * (All motors switches ON/OFF at variable time periods) until stop switch is 
 * pressed or another mode is switched.
 * 
 * <h3>STEP MODE<h3>:   When step mode switch is pressed the PIC micro-controller 
 * switches ON STEP MODE LED indicator and motors start operating in step mode 
 * (Motor 1 & 2 ON for 700ms then Motor 3 & 4 ON for 700ms and so on) until stop 
 * switch is presses or another mode is switched.
 */

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000UL                    // Internal Frequency 4MHz

extern uint8_t scanButtons(void);               // External assy function for Button Scanning
extern void processAutoStepModes(void);         // External assy function for AUTO/STEP Modes

/*==============================================================================
 
                                 Configuration Bits  
 
 =============================================================================*/
#pragma config FOSC  = INTOSCIO  // Internal oscillator
#pragma config WDTE  = OFF       // Watchdog Timer disabled
#pragma config PWRTE = OFF       // Power-up Timer disabled
#pragma config MCLRE = OFF       // RA5/MCLR is MCLR
#pragma config BOREN = OFF       // Brown-out Reset disabled
#pragma config LVP   = OFF       // Low-voltage programming disabled
#pragma config CPD   = OFF       // Data EEPROM memory code protection off
#pragma config CP    = OFF       // Flash program memory code protection off

/*==============================================================================
 
                                Global Variables  
 
 =============================================================================*/
// -------- Motor Patterns (Full A, scaled to 10ms) --------
const uint8_t motor1A[] = {19, 3, 7, 22, 15, 24, 7, 30, 8, 12, 7, 12, 7, 37};
const uint8_t motor2A[] = {7, 13, 14, 23, 14, 80, 7, 12, 7, 12, 7, 12};
const uint8_t motor3A[] = {19, 3, 8, 22, 14, 24, 7, 12, 7, 12, 7, 12, 7, 56};
const uint8_t motor4A[] = {7, 3, 8, 22, 14, 23, 8, 11, 8, 67, 8, 30};

// -------- Motor Patterns (Full B, scaled to 10ms) --------
const uint8_t motor1B[] = {0, 0, 12, 19, 12, 19, 12, 19, 12, 130};
const uint8_t motor2B[] = {0, 93, 12, 19, 12, 19, 12, 19, 12, 37};
const uint8_t motor3B[] = {0, 13, 12, 19, 12, 130, 12, 19, 12, 6};
const uint8_t motor4B[] = {0, 44, 12, 19, 12, 19, 12, 19, 12, 86};

typedef struct {
    const uint8_t *pattern;             // Motor ON/OFF Patterns
    uint8_t length;                     // Patterns Length
    uint8_t idx;                        // id
    uint16_t remaining;                 // Remaining Time in ms
    uint8_t active;                     // 0 = OFF Segment, 1 = ON segment
} Motor;

#define PWM_FREQ    62                  // PWM Frequency
#define DUTY_CYCLE  60                  // Duty Cycle in %

#define LEN1A       (sizeof(motor1A))
#define LEN2A       (sizeof(motor2A))
#define LEN3A       (sizeof(motor3A))
#define LEN4A       (sizeof(motor4A))

#define LEN1B       (sizeof(motor1B))
#define LEN2B       (sizeof(motor2B))
#define LEN3B       (sizeof(motor3B))
#define LEN4B       (sizeof(motor4B))

volatile uint8_t        current_mode    = 0;    // 0 = STOP, 1 = AUTO, 2 = STEP, 3 = FULL
volatile unsigned char  full_sub_mode   = 0;    // 0 = FULL A, 1 = FULL B
volatile unsigned char  auto_step_state = 0;    // 0 = Motor Pair 1, 1 = Motor Pair 2
volatile uint8_t        pwm_counter     = 0;    // PWM Pulses Counter
volatile uint16_t       ms_counter      = 0;    // Milli seconds Counter
volatile uint8_t        pwm_period      = (1000 / PWM_FREQ); 
volatile uint8_t        on_time         = (1000 / PWM_FREQ) * DUTY_CYCLE / 100;

volatile Motor motors[4];

/*==============================================================================
 
                      Initialize Full Mode A Function 
 
 =============================================================================*/
void initFullA(void) {
    motors[0].pattern = motor1A;
    motors[0].length = LEN1A;
    motors[0].idx = 0;
    motors[0].remaining = (uint16_t) motor1A[0] * 10;
    motors[0].active = 1;

    motors[1].pattern = motor2A;
    motors[1].length = LEN2A;
    motors[1].idx = 0;
    motors[1].remaining = (uint16_t) motor2A[0] * 10;
    motors[1].active = 1;

    motors[2].pattern = motor3A;
    motors[2].length = LEN3A;
    motors[2].idx = 0;
    motors[2].remaining = (uint16_t) motor3A[0] * 10;
    motors[2].active = 1;

    motors[3].pattern = motor4A;
    motors[3].length = LEN4A;
    motors[3].idx = 0;
    motors[3].remaining = (uint16_t) motor4A[0] * 10;
    motors[3].active = 1;

    PORTB &= 0x0F; // motors OFF initially
}

/*==============================================================================
  
                        Initialize Full Mode B Function 
  
 =============================================================================*/
void initFullB(void) {
    
    motors[0].pattern = motor1B;
    motors[0].length = LEN1B;
    motors[0].idx = 0;
    motors[0].remaining = (uint16_t) motor1B[0] * 10;
    motors[0].active = 1;

    motors[1].pattern = motor2B;
    motors[1].length = LEN2B;
    motors[1].idx = 0;
    motors[1].remaining = (uint16_t) motor2B[0] * 10;
    motors[1].active = 1;

    motors[2].pattern = motor3B;
    motors[2].length = LEN3B;
    motors[2].idx = 0;
    motors[2].remaining = (uint16_t) motor3B[0] * 10;
    motors[2].active = 1;

    motors[3].pattern = motor4B;
    motors[3].length = LEN4B;
    motors[3].idx = 0;
    motors[3].remaining = (uint16_t) motor4B[0] * 10;
    motors[3].active = 1;

    PORTB &= 0x0F; // motors OFF initially
}

/*==============================================================================
 
                        Process Full Mode Function  
 
 =============================================================================*/
void processFullMode(void) {
    for (uint8_t i = 0; i < 4; i++) {
        Motor *m = (Motor*) & motors[i];
        if (m->remaining) {
            m->remaining--;
        } else {
            // advance pattern
            m->idx = (m->idx + 1) % m->length;
            m->remaining = m->pattern[m->idx]*10;
            m->active = (m->idx % 2 == 0); // even = ON
        }

        if (m->active && pwm_counter < on_time) {
            PORTB |= (1 << (7-i));
        } else {
            PORTB &= ~(1 << (7-i));
        }
        }

    // PWM counter
    pwm_counter++;
    if (pwm_counter >= pwm_period) pwm_counter = 0;
}

/*==============================================================================
 
                         Interrupt Service Routine  
 
 =============================================================================*/
void __interrupt() isr(void) {
    if (T0IF) {
        T0IF = 0;
        TMR0 = 131; // reload for 1ms tick

        if (current_mode == 0) {    // STOP Mode
            //processFullA();
        }
        if (current_mode == 1 || current_mode == 2) {    // AUTO & STEP Mode
            processAutoStepModes();
        }
        if (current_mode == 3) {    // FULL Mode
            processFullMode();
        }
    }
}

/*==============================================================================
  
                               Initialization 
  
 =============================================================================*/
void init(void) {
    CMCON = 0x07;           // Switch OFF Comparators on PIC16F627A
    TRISA = 0b00110000;     // RA4 input (for button detection)
    TRISB = 0b00000001;     // RB0 input, (for button detection) while RB1-RB7 output for LEDs and Motor Controls
    PORTA = 0;
    PORTB = 0;

    OPTION_REG = 0b00000010; // Prescaler 1:8
    TMR0 = 131;
    T0IE = 1;
    GIE = 1;
}

/*==============================================================================
  
                                   Update LEDs 
  
 =============================================================================*/
void update_leds(void) {
    const unsigned char mask[4] = {
        0x0E,           // STOP -> all OFF  (binary equivalent 0b00001110)
        0x0B,           // AUTO -> LED1 ON  (binary equivalent 0b00001011)
        0x07,           // STEP -> LED2 ON  (binary equivalent 0b00000111)
        0x0D            // FULL -> LED3 ON  (binary equivalent 0b00001101)
    };
    PORTB = (PORTB & 0xF0) | mask[current_mode];
}

/*==============================================================================
  
                               Main Function 
  
 =============================================================================*/
void main(void) {
    init();
    uint8_t last_button = 255;
    
    while (1) {
        // 1. Check for button press
        uint8_t button_pressed = scanButtons();

        // 2. Handle the button press event
        if (button_pressed != 255 && button_pressed != last_button) {
            switch (button_pressed) {
            default:
            case 0: // STOP Button
                current_mode = 0;
                full_sub_mode = 0;
                // Turn off all motors
                PORTB &= 0x0F;
                break;
            case 1: // AUTO Button
                current_mode = 1;
                full_sub_mode = 0;
                break;
            case 2: // STEP Button
                current_mode = 2;
                full_sub_mode = 0;
                break;
            case 3: // FULL Button
                if (current_mode != 3) {
                    current_mode = 3;
                    full_sub_mode = 0;
                    initFullA();
                    //initFullMode(patternsA, lengthsA);
                } else {
                    if(!full_sub_mode) {
                        full_sub_mode = !full_sub_mode;
                        initFullB();  
                    }else{
                        full_sub_mode = !full_sub_mode;
                        initFullA();
                    }
                }
                break;
            }
        }
        last_button = button_pressed;
        // 3. Update LEDs based on current mode
        update_leds();

        // 4. Small delay to reduce CPU load
        __delay_ms(5);
    }
}

```

scan_buttons.asm

```
; ================================
; File: scan_buttons.asm
; Function: scan_buttons
; Returns: WREG = button code
;   0 = STOP
;   1 = AUTO
;   2 = STEP
;   3 = FULL
;   255 = no button
; ================================
        LIST    P=16F627A       ; tell assembler which device
        #include <xc.inc>       ; include XC8 definitions for SFRs

    GLOBAL  _scanButtons   ; export to C

    PSECT   text,class=CODE,delta=2

_scanButtons:
    ; Default return = 255 (no button)
    movlw   0xFF
    movwf   0x70        ; temp variable (choose a free GPR)

    ; --- ROW1: RA2=0, RA3=1 ---
    bcf     TRISA,2     ; RA2 output
    bsf     TRISA,3     ; RA3 input
    bcf     PORTA,2     ; RA2 low
    bsf     PORTA,3     ; RA3 high

    btfss   PORTA,4     ; Check RA4 -> FULL
    movlw   3
    btfss   PORTB,0     ; Check RB0 -> AUTO
    movlw   1
    movwf   0x70

    ; --- ROW2: RA3=0, RA2=1 ---
    bcf     TRISA,3     ; RA3 output
    bsf     TRISA,2     ; RA2 input
    bcf     PORTA,3     ; RA3 low
    bsf     PORTA,2     ; RA2 high

    btfss   PORTA,4     ; Check RA4 -> STOP
    movlw   0
    btfss   PORTB,0     ; Check RB0 -> STEP
    movlw   2
    movwf   0x70

    ; Return value
    movf    0x70,W
    return

```

auto_step_modes.asm

```
; runAutoStepModes.asm
; Assembly implementation of runAutoStepModes function for PIC16F627A

    list p=16f627a
    
    ; Register definitions
STATUS  equ 0x03
PORTB   equ 0x06
PCL     equ 0x02
    
    ; Flag bit positions in STATUS register
Z_FLAG  equ 2    ; Zero flag bit position
C_FLAG  equ 0    ; Carry flag bit position
    
    ; Declare global symbols
    global _processAutoStepModes
    
    ; External variables
    global _pwm_counter
    global _on_time
    global _current_mode
    global _auto_step_state
    global _pwm_period
    global _ms_counter
    
    ; Define temporary storage
temp     equ 0x70
temp2    equ 0x71
    
    ; Code section with PSECT
    PSECT text0, class=CODE, delta=2

_processAutoStepModes:
    ; Check if pwm_counter < on_time
    movf    _pwm_counter, W
    subwf   _on_time, W
    btfss   STATUS, C_FLAG
    goto    motors_off
    
    ; Check current_mode
    movf    _current_mode, W
    xorlw   1           ; Check if current_mode == 1 (AUTO)
    btfss   STATUS, Z_FLAG
    goto    step_mode
    
    ; AUTO mode
    movf    _auto_step_state, W
    btfss   STATUS, Z_FLAG
    goto    auto_state_1
    
    ; auto_step_state == 0: M1+M4 on
    movf    PORTB, W
    andlw   0x0F
    iorlw   0x90        ; 0b10010000 - M1 and M4
    movwf   PORTB
    goto    update_counters
    
auto_state_1:
    ; auto_step_state == 1: M2+M3 on
    movf    PORTB, W
    andlw   0x0F
    iorlw   0x60        ; 0b01100000 - M2 and M3
    movwf   PORTB
    goto    update_counters
    
step_mode:
    ; STEP mode
    movf    _auto_step_state, W
    btfss   STATUS, Z_FLAG
    goto    step_state_1
    
    ; auto_step_state == 0: M1+M2 on
    movf    PORTB, W
    andlw   0x0F
    iorlw   0xC0        ; 0b11000000 - M1 and M2
    movwf   PORTB
    goto    update_counters
    
step_state_1:
    ; auto_step_state == 1: M3+M4 on
    movf    PORTB, W
    andlw   0x0F
    iorlw   0x30        ; 0b00110000 - M3 and M4
    movwf   PORTB
    goto    update_counters
    
motors_off:
    ; Turn off all motors
    movf    PORTB, W
    andlw   0x0F
    movwf   PORTB
    
update_counters:
    ; Update pwm_counter
    incf    _pwm_counter, F
    movf    _pwm_period, W
    subwf   _pwm_counter, W
    btfss   STATUS, C_FLAG
    goto    update_ms_counter
    
    ; Reset pwm_counter if >= pwm_period
    clrf    _pwm_counter
    
update_ms_counter:
    ; Update ms_counter
    incf    _ms_counter, F
    btfss   STATUS, Z_FLAG
    goto    check_ms_counter
    
    incf    _ms_counter+1, F
    
check_ms_counter:
    ; Check if ms_counter >= 670 (0x029E)
    movlw   0x9E
    subwf   _ms_counter, W
    movlw   0x02
    btfss   STATUS, C_FLAG
    movlw   0x03
    subwf   _ms_counter+1, W
    btfss   STATUS, C_FLAG
    return
    
    ; Reset ms_counter and toggle auto_step_state
    clrf    _ms_counter
    clrf    _ms_counter+1
    movlw   0x01
    xorwf   _auto_step_state, F
    
    return
    
    end
```


# Contact (for any query/questions)
mansoor.ahmed@fuuast.edu.pk
m.mansoor.ahmed@outlook.com

