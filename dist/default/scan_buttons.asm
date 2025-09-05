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
