# Massage Machine Microcontroller (PIC16F627A) Programmming
A friend of mine got stuck into troubleshooting of a massage machine. He did repaired some components but when he recovered hex code from the PIC microcontroller and tried to use it another PIC, it never worked. The code was basically protected for copying and reading by the programmer. He asked me to rescue him. This project is actually built for him and it worked like charm. 

<h1>Working Princple of Massage Controller</h1>
The machine had four modes namely, AUTO, STEP, FULL and STOP. There were a remote which is attached to the machine via a serial cable. The Remote had 04 SPST Buttons and 03 LED indicators. 01 Button for each mode selection and the 03 LEDs were for AUTO, STEP and FULL mode each and in STOP mode all remained OFF. There were 04 12VDC motors whose positive terminal were connected to the 12VDC Power while negative terminals were driven by 04 2N2222 Transistors in emitter-follower configuartion. Base of each Transisor was connected to PIC microcontroller (RB4, RB5, RB6 and RB7) via a 4.5K Ohm resistor. All the pins are kept HIGH and driven to LOW when Motors are needed to operate.
The Buttons are connected in 2x2 Button Matrix like a calculator/mobile/computer keyboards. 02 Rows connections are always HIGH via external 10K OHMs pull-up resistors i.e RA4 and RB0 while 02 Coulmn connections are switched to logic LOW during Button Scanning. Positive termnal of LED Indicators in connected to VDD and negative terminal is driven by PIC microcontroller (RB1, RB2, RB3) via 460 OHM resistors. Follwoing operational modes of Massage Machine are managed by PIC Microcontroller. 
<h3>AUTO Mode: </h3> When Auto Mode Button is pressed, Auto Mode Indicator LED lit up and Motors start operating in Auto Mode (Motor 1 and 4 operate simultaneously for 700ms and then Motor 2 and 3 for the same period).
<h3>STEP Mode: </h3> When Step Mode Button is pressed, Step Mode Indicator LED lit up and Motors start operating in Step Mode (Motor 1 and 2 operate simultaneously for 700ms and then Motor 3 and 4 for the same period).
<h3>FULL Mode: </h3> When Full Mode Button is pressed, Full Mode Indicator LED lit up and Motors start operating in Full Mode (All motors operate in independent time periods and sequence).
<h3>STOP Mode: </h3> When Stop Mode Button is pressed, All LED Indicators and all Motors swicthes OFF.

<h1>Implementation</h1>
Written following Code in MPLab X IDE and generated the HEX code for uploading to PIC16F627A: Also, 
```C Code
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

# Contact
mansoor.ahmed@fuuast.edu.pk

m.mansoor.ahmed@outlook.com

