# Massage Machine Microcontroller (PIC16F627A) Programmming (MPLab) and Simulation in Proteus 
A friend of mine got stuck into troubleshooting of a massage machine. He did repaired some components but when he recovered hex code from the PIC microcontroller and tried to use it another PIC, it never worked. The code was basically protected for copying and reading by the programmer. He asked me to rescue him. This project is actually built for him and it worked like charm. 

# Working Princple of Massage Controller
The machine had four modes namely, AUTO, STEP, FULL and STOP. There were a remote which is attached to the machine via a serial cable. The Remote had 04 SPST Buttons and 03 LED indicators. 01 Button for each mode selection and the 03 LEDs were for AUTO, STEP and FULL mode each and in STOP mode all remained OFF. There were 04 12VDC motors whose positive terminal were connected to the 12VDC Power while negative terminals were driven by 04 2N2222 Transistors in emitter-follower configuartion. Base of each Transisor was connected to PIC microcontroller (RB4, RB5, RB6 and RB7) via a 4.5K Ohm resistor. All the pins are kept HIGH and driven to LOW when Motors are needed to operate.
The Buttons are connected in 2x2 Button Matrix like a calculator/mobile/computer keyboards. 02 Rows connections are always HIGH via external 10K OHMs pull-up resistors i.e RA4 and RB0 while 02 Coulmn connections are switched to logic LOW during Button Scanning. Positive termnal of LED Indicators in connected to VDD and negative terminal is driven by PIC microcontroller (RB1, RB2, RB3) via 460 OHM resistors. Follwoing operational modes of Massage Machine are managed by PIC Microcontroller. 
<h3>AUTO Mode: </h3> When Auto Mode Button is pressed, Auto Mode Indicator LED lit up and Motors start operating in Auto Mode (Motor 1 and 4 operate simultaneously for 700ms and then Motor 2 and 3 for the same period).
<h3>STEP Mode: </h3> When Step Mode Button is pressed, Step Mode Indicator LED lit up and Motors start operating in Step Mode (Motor 1 and 2 operate simultaneously for 700ms and then Motor 3 and 4 for the same period).
<h3>FULL Mode: </h3> When Full Mode Button is pressed, Full Mode Indicator LED lit up and Motors start operating in Full Mode (All motors operate in independent time periods and sequence).
<h3>STOP Mode: </h3> When Stop Mode Button is pressed, All LED Indicators and all Motors swicthes OFF.

# Implementation
Written following Code in MPLab X IDE and generated the HEX code for uploading to PIC16F627A:
```C Code
/*
 * File:        main.c
 * Author:      M Mansoor Ahmed
 * E-mail:      mansoor.ahmed@fuuast.edu.pk, 
 *              m.mansoor.ahmed@outlook.com
 * Whatsapp:    +92-313-123-3939
 * Created on:  August 17, 2025, 1:25 AM
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
 * STEP MODE:   When step mode switch is pressed the PIC micro-controller 
 * switches ON STEP MODE LED indicator and motors start operating in step mode 
 * (Motor 1 & 2 ON for 700ms then Motor 3 & 4 ON for 700ms and so on) until stop 
 * switch is presses or another mode is switched.
 */

#define _XTAL_FREQ 4000000
#include <xc.h>

//Configuration Bits ---DO NOT CHANGE THESE---
#pragma config FOSC = INTOSCIO
#pragma WDTE = OFF 
#pragma PWRTE = OFF
#pragma MCLRE = OFF
#pragma config BOREN = OFF 
#pragma LVP = OFF 
#pragma CPD = OFF 
#pragma CP = ON

// Function prototype declaration
unsigned char scan_matrix();

// LED macros (active LOW)
#define LED_AUTO  PORTBbits.RB1
#define LED_FULL  PORTBbits.RB2
#define LED_STEP  PORTBbits.RB3

// Modes Definitions
#define MODE_STOP 0
#define MODE_AUTO 1
#define MODE_FULL 2
#define MODE_STEP 3

// Motor timing patterns for FULL MODE
const unsigned char motor_timings[4][12] = {
    {30, 20, 16, 22,  9, 30,  9, 13,  6, 13,  9, 35}, // M1
    { 9,  9,  9,  9,  9,  9,  9, 13, 17, 20, 17, 76}, // M2
    {32, 20, 16, 22,  9, 10,  9, 10,  9, 10,  9, 54}, // M3
    { 9, 30, 18, 21, 14, 22,  9, 10,  9, 67,  0,  0}  // M4
};

// Global state variables
unsigned char current_mode = MODE_STOP;
unsigned char last_sw_state = 0;

void main() {
    CMCON = 0x07;        // Disable comparators
    TRISB = 0b00000001;  // RB0 input, others outputs
    TRISA = 0b00010000;  // RA4 input, others outputs
    
    // Initialize outputs - ALL LEDs OFF, ALL motors OFF
    PORTB = 0b00001111;  // LEDs OFF (active low), motors OFF (active HIGH)
    PORTA = 0x00;
    
    // FULL mode state variables
    unsigned char motor_step[4] = {0};
    unsigned char motor_timer[4] = {0};
    unsigned char motor_state[4] = {0};
    unsigned char sequence_step = 0;
    unsigned char time_counter = 0;

    while(1) {
        __delay_ms(10);  // 10ms time base
        time_counter++;

        // Scan buttons every 20ms
        if((time_counter % 2) == 0) {
            unsigned char sw = scan_matrix();
            
            if(sw != 0 && sw != last_sw_state) {
                // Turn off all LEDs first
                PORTB |= 0b00001110;
                
                switch(sw) {
                    case 1:  // AUTO
                        current_mode = MODE_AUTO;
                        LED_AUTO = 0;
                        sequence_step = 0;
                        PORTB = (PORTB & 0x0F) | 0b10010000;  // M1&M4 ON
                        break;
                        
                    case 2:  // FULL
                        current_mode = MODE_FULL;
                        LED_FULL = 0;
                        for(int i=0; i<4; i++) {
                            motor_step[i] = 0;
                            motor_timer[i] = 0;
                            motor_state[i] = 1;  // Start in ON state
                        }
                        PORTB |= 0xF0;  // All motors ON
                        break;
                        
                    case 3:  // STEP
                        current_mode = MODE_STEP;
                        LED_STEP = 0;
                        sequence_step = 0;
                        PORTB = (PORTB & 0x0F) | 0b00110000;  // M1&M2 ON
                        break;
                        
                    case 4:  // STOP
                        current_mode = MODE_STOP;
                        PORTB &= 0x0F;  // All motors OFF
                        break;
                }
                last_sw_state = sw;
                time_counter = 0;  // Reset timing
            }
        }

        //Execute current mode (Motor sequence and timings as per selected mode)
        switch(current_mode) {
            //AUTO MODE
            case MODE_AUTO:
                if(time_counter >= 70) {
                    time_counter = 0;
                    sequence_step = !sequence_step;
                    if(sequence_step) {
                        PORTB = (PORTB & 0x0F) | 0b01100000;
                    }
                    else {
                        PORTB = (PORTB & 0x0F) | 0b10010000;
                    }
                }
                break;
                
            case MODE_FULL: 
                for(int i=0; i<4; i++) {
                    unsigned char dur = motor_timings[i][motor_step[i]];
                    if(dur == 0) { 
                        motor_step[i] = (motor_step[i] + 1) % 12;
                        motor_timer[i] = 0;
                        continue;
                    }
                    if(++motor_timer[i] >= dur) {
                        motor_timer[i] = 0;
                        motor_state[i] = !motor_state[i];
                        motor_step[i] = (motor_step[i] + 1) % 12;
                        
                        if(motor_state[i]) {
                            PORTB |= (1 << (4+i));  // Motor ON
                        } else {
                            PORTB &= ~(1 << (4+i)); // Motor OFF
                        }
                    }
                }
                break;
            //STEP MODE
            case MODE_STEP: 
                if(time_counter >= 70) { 
                    time_counter = 0;
                    sequence_step = !sequence_step;
                    if(sequence_step) {
                        PORTB = (PORTB & 0x0F) | 0b11000000;
                    } 
                    else {
                        PORTB = (PORTB & 0x0F) | 0b00110000;
                    }
                }
                break;
        }
    }
}

// Function definition
unsigned char scan_matrix() {
    unsigned char sw = 0;

    /*Scan Column RA2*/
    
    TRISAbits.TRISA2 = 0; 
    
    PORTAbits.RA2 = 0;
    
    TRISAbits.TRISA3 = 1;
    
    __delay_us(10);

    if (PORTBbits.RB0 == 0) sw = 1;  // AUTO

    if (PORTAbits.RA4 == 0) sw = 2;  // FULL

    // Scan Column RA3

    TRISAbits.TRISA2 = 1;
    
    TRISAbits.TRISA3 = 0; 
    PORTAbits.RA3 = 0;
    __delay_us(10);
    if (PORTBbits.RB0 == 0) sw = 3;  // STEP
    if (PORTAbits.RA4 == 0) sw = 4;  // STOP

    // Release columns
    TRISAbits.TRISA2 = TRISAbits.TRISA3 = 1;
    
    return sw;
}
```
# Testing in Proteus before final uploading and testing in real device:

<h3>AUTO Mode</h3>
![AUTO MODE](https://github.com/user-attachments/assets/3abfc9a9-8cd9-46d9-9ea7-e89d638e159d)

<h3>STEP Mode</h3>
![STEP MODE](https://github.com/user-attachments/assets/ce746857-0b43-492e-9133-fd702c6f1420)

<h3>FULL Mode</h3>
![FULL MODE](https://github.com/user-attachments/assets/2bbe45dc-f7d9-4a69-b88b-ac85585b9441)

# Contact
mansoor.ahmed@fuuast.edu.pk

m.mansoor.ahmed@outlook.com

