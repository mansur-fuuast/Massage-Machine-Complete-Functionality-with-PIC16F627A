/*==============================================================================
 File:  This file is part of a project utilizing PIC16F627 Microchip 
        micro-controller IC. It controls a Massage Kit Installed in Executive
        class buses. Massage Kits are installed with four 12VDC High Speed 
        Motors. Massage Kits operate through a remote control connected via a 
        serial cable. It has four operational modes with corresponding 
        indications incorporated by using LEDs. 
        (i)     Stop Mode:  keeps massage kits OFF.
        (ii)    Auto Mode:  Motors operate in pairs 02 ON 02 OFF (Horizontally)
        (iii)   Step Mode:  Motors operate in pairs 02 ON 02 OFF (Vertically)
        (iv)    Full Mode:  Further 2 Modes. Operates All Motors at variable
                            time periods and speed.
 Author: M Mansoor Ahmed (mansoor.ahmed@fuuast.edu.pk)
 Created on:    03 September, 2025     
 =============================================================================*/
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
volatile uint8_t        on_time_A       = ((1000 / PWM_FREQ) * DUTY_CYCLE / 100)+1;
volatile uint8_t        on_time_B       = ((1000 / PWM_FREQ) * DUTY_CYCLE / 100)+2;
//volatile uint8_t        off_time        = (1000 / PWM_FREQ) - ((1000 / PWM_FREQ) * DUTY_CYCLE / 100);

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

        uint8_t p_time = 0;
        // Apply PWM ON/OFF
        if(full_sub_mode) p_time = on_time_B;
        if(!full_sub_mode) p_time = on_time_A;
        
        if (m->active && pwm_counter < p_time) {
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
