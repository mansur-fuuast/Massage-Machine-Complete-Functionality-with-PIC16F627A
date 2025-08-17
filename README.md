![Untitled video - Made with Clipchamp](https://github.com/user-attachments/assets/863e9994-d47a-46fa-aff8-a7f2d4b32045)# Massage Machine Microcontroller (PIC16F627A) Programmming (MPLab) and Simulation in Proteus 
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

//Internal Oscillator Frequency set at 4MHz
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
/* This is the function responsible for scanning the switches and setting the 
 * corresponding mode bit ON
 */
unsigned char scan_matrix();

// LED macros (active LOW)
/*Defining the LEDs coonections to PIC16F627A*/
#define LED_AUTO  PORTBbits.RB1
#define LED_FULL  PORTBbits.RB2
#define LED_STEP  PORTBbits.RB3

// Modes Definitions
#define MODE_STOP 0
#define MODE_AUTO 1
#define MODE_FULL 2
#define MODE_STEP 3

// Motor timing patterns for FULL MODE (in 10ms units)
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
                        //Set the current selected mode bit to MODE AUTO
                        current_mode = MODE_AUTO;
                        //switch ON the AUTO LED
                        LED_AUTO = 0;
                        //set sequence step to 0 so at start M1 and M4 are 
                        //selected
                        sequence_step = 0;
                        //Switch On M1 and M4 at start
                        PORTB = (PORTB & 0x0F) | 0b10010000;  // M1&M4 ON
                        break;
                        
                    case 2:  // FULL
                        //Set the current selected mode bit to MODE FULL
                        current_mode = MODE_FULL;
                        //switch ON the FULL LED
                        LED_FULL = 0;
                        // Reset motor states
                        for(int i=0; i<4; i++) {
                            motor_step[i] = 0;
                            motor_timer[i] = 0;
                            motor_state[i] = 1;  // Start in ON state
                        }
                        //Switch On All motors at start
                        PORTB |= 0xF0;  // All motors ON
                        break;
                        
                    case 3:  // STEP
                        //Set the current selected mode bit to MODE STEP
                        current_mode = MODE_STEP;
                        //switch ON the FULL LED
                        LED_STEP = 0;
                        //set sequence step to 0 so at start M1 and M2 are 
                        //selected
                        sequence_step = 0;
                        //Switch ON M1 & M2
                        PORTB = (PORTB & 0x0F) | 0b00110000;  // M1&M2 ON
                        break;
                        
                    case 4:  // STOP
                        //Set the current selected mode bit to MODE STOP
                        current_mode = MODE_STOP;
                        //Switch OFF all Motors
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
                //Until 700ms keep motor pair ON
                if(time_counter >= 70) {  // 700ms reached
                    //reset time counter
                    time_counter = 0;
                    //change sequence to switch to M2 & M3 from M1 & M4 after 700ms
                    sequence_step = !sequence_step;
                    //When sequence changed (after 700ms), Switch OFF M 1 & 4
                    //and Switch ON M 2 & 3.
                    if(sequence_step) {
                        // M1 & M4 OFF, M2 & M3 ON
                        PORTB = (PORTB & 0x0F) | 0b01100000;
                    }
                    //Else if still sequence not changed keep running M 1 & 4
                    //and keep OFF M 2 & 3.
                    else {
                        // M2 & M3 OFF, M1 & M4 ON
                        PORTB = (PORTB & 0x0F) | 0b10010000;
                    }
                }
                break;
                
            case MODE_FULL: 
                //loop to iterate in array of timings and steps in FULL MODE
                for(int i=0; i<4; i++) {
                    //Pick the duration from the Array 
                    unsigned char dur = motor_timings[i][motor_step[i]];
                    //if duration is zero ignore it
                    if(dur == 0) {  // Skip zero-duration steps
                        motor_step[i] = (motor_step[i] + 1) % 12;
                        motor_timer[i] = 0;
                        continue;
                    }
                    //Switch Motors ON/OFF based on timers for each motor picked
                    //previously from two dimensional array
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
                //Until 700ms keep motor pair ON
                if(time_counter >= 70) {  // 700ms reached
                    //Reset time counter
                    time_counter = 0;
                    //change motor sequence to switch ON now M 3 & 4.
                    sequence_step = !sequence_step;
                    //If sequence is changed switch OFF M 1 & 2 and ON M 3 & 4.
                    if(sequence_step) {
                        // M1 & M2 OFF, M3 & M4 ON
                        PORTB = (PORTB & 0x0F) | 0b11000000;
                    } 
                    //If sequence is not changed then continue with M 1 & 2 ON
                    //and M 3 & 4 OFF.
                    else {
                        // M3 & M4 OFF, M1 & M2 ON
                        PORTB = (PORTB & 0x0F) | 0b00110000;
                    }
                }
                break;
        }
    }
}

// Function definition
/*This is the function responsible for scanning of the switch matrix. It will 
keep listening to switch presses and will set corresponding modes. As we have 
 four Pins connection (RB0 & RA4 HIGH, While RA2 & 3 LOW), we have to use this
 matrix manipulation. No dedicated connection for each switch so we will form a 
 matrix of 2x2 i.e. Two Rows (RA4 and RB0 both HIGH) while Two Rows (RA2 and RA3
 * both LOW)*/
unsigned char scan_matrix() {
    unsigned char sw = 0;

    /*Scan Column RA2*/
    
    //Set RA2 as output Pin
    TRISAbits.TRISA2 = 0; 
    
    //Set RA2 to LOW
    PORTAbits.RA2 = 0;
    
    //Set RA3 as input, the Pin will float with High Impedance making possible 
    //to detect Whether its RB0 (AUTO) or RA4 (FULL) as both will be pulled to
    //LOW when connected to RA2 otherwise will remain HIGH (by default)
    TRISAbits.TRISA3 = 1;
    
    //de-bounce delay
    __delay_us(10);

    //if RB0 is pulled LOW its AUTO MODE switch pressed
    if (PORTBbits.RB0 == 0) sw = 1;  // AUTO

    //if RA4 is pulled LOW its FULL MODE switch pressed
    if (PORTAbits.RA4 == 0) sw = 2;  // FULL

    // Scan Column RA3

    //Set RA2 as input Pin, the Pin will float with High Impedance making 
    //possible to detect Whether its RB0 (STEP) or RA4 (STOP) as both will be 
    //pulled to LOW when connected to RA3 otherwise will remain HIGH (by default)
    TRISAbits.TRISA2 = 1;
    
    //RA3 as output pin so we can control it
    TRISAbits.TRISA3 = 0; 
    //Set RA3 to LOW
    PORTAbits.RA3 = 0;
    //de-bounce time
    __delay_us(10);
    //if RB0 is pulled LOW by RA3 its STEP MODE switch pressed
    if (PORTBbits.RB0 == 0) sw = 3;  // STEP
    //if RA4 is pulled LOW by RA3 its STOP MODE switch pressed
    if (PORTAbits.RA4 == 0) sw = 4;  // STOP

    // Release columns
    TRISAbits.TRISA2 = TRISAbits.TRISA3 = 1;
    
    //return the selected mode
    return sw;
}
```
# Testing in Proteus before final uploading and testing in real device:

<h3>AUTO Mode</h3>
![Untitled video - Made with Clipchamp](https://github.com/user-attachments/assets/0ff0f27b-deac-4887-9f26-2b40587e83d3)

<h3>STEP Mode</h3>
![STEP MODE - Made with Clipchamp](https://github.com/user-attachments/assets/e37905fb-e90a-4dfc-ad76-65f53ae8e828)

<h3>FULL Mode</h3>
![FULL MODE - Made with Clipchamp](https://github.com/user-attachments/assets/0cdfea84-08ef-4f53-8297-03dd4810b55a)

# Contact
mansoor.ahmed@fuuast.edu.pk
m.mansoor.ahmed@outlook.com

