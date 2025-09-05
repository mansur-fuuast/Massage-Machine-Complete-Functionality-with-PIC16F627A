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
