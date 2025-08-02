
; FINAL PROJECT: EEBOT MAZE ROAMING
;                
               XDEF Entry, _Startup     ; export 'Entry' symbol
               ABSENTRY Entry           ; for absolute assembly: mark this as application entry point
               INCLUDE 'derivative.inc' ; Include derivative-specific definitions 

; Constant Definitions                                                            
LCD_DAT        EQU PORTB  ; LCD data port, bits - PB7,...,PB0
LCD_CNTR       EQU PTJ    ; LCD control port, bits - PJ6(RS),PJ7(E)
LCD_E          EQU $80    ; LCD E-signal pin
LCD_RS         EQU $40    ; LCD RS-signal pin

  ; Path detection threshold        
PTH_A_INT      EQU $CF    ; Minimum sensor detection value for A
PTH_B_INT      EQU $CF    ; Minimum sensor detection value for B        
PTH_C_INT      EQU $CF    ; Minimum sensor detection value for C
PTH_D_INT      EQU $CF    ; Minimum sensor detection value for D

PTH_E_INT      EQU $68   ;Differential E-F value where SENSOR_LINE < PTH_E_INT (robot must move right)
PTH_F_INT      EQU $68  ; Differential E-F value where SENSOR_LINE > PTH_F_INT (robot must move left)

  ; Displacement values 
INC_DIS        EQU 500   ; INCREMENT distance
FWD_DIS        EQU 2000  ; FORWARD distance
REV_DIS        EQU 1000  ; REVERSE distance
STR_DIS        EQU 1000  ; STRAIGHT distance
TRN_DIS        EQU 14700 ; TURN distance
UTRN_DIS       EQU 12500 ; U-TURN distance

  ; Intersection Values 
PRI_PTH_INT    EQU 0 ; Path 1 value (primary route)
SEC_PTH_INT    EQU 1 ; Path 2 Value (secondary route)

  ; eebot State values
START          EQU 0 ; START state value
FWD            EQU 1 ; FORWARD state value
REV            EQU 2 ; REVERSE state value
RT_TRN         EQU 3 ; RIGHT TURN state value
LT_TRN         EQU 4 ; LEFT TURN state value
BK_TRK         EQU 5 ; BACKTRACK state value
STND_BY        EQU 6 ; STANDBY state value

; Liquid Crystal Display Equates ----
CLEAR_HOME     EQU $01 ; Clear the display and home the cursor
INTERFACE      EQU $38 ; 8 bit interface, two line display
CURSOR_OFF     EQU $0C ; Display on, cursor off
SHIFT_OFF      EQU $06 ; Address increments, no character shift
LCD_SEC_LINE   EQU 64  ; Starting addr. of 2nd line of LCD (note decimal value!)

; Other codes
NULL           EQU 00    ; The string ?null terminator?
CR             EQU $0D   ; ?Carriage Return? character
SPACE          EQU ' '   ; The ?space? character 

  ; Variables
               ORG   $3850 
                
CRNT_STATE     DC.B  6  ; Current state register

COUNT1         DC.W  0  ; Initializing first counter value
COUNT2         DC.W  0  ; Initializing second counter value

A_DETN         DC.B  0  ; Path detection "boolean" for Sensor A (PATH = 1, NO PATH = 0)
B_DETN         DC.B  0  ; Path detection "boolean" for Sensor B (PATH = 1, NO PATH = 0)
C_DETN         DC.B  0  ; Path detection "boolean" for Sensor C (PATH = 1, NO PATH = 0)
D_DETN         DC.B  0  ; Path detection "boolean" for Sensor D (PATH = 1, NO PATH = 0)
E_DETN         DC.B  0  ; Path detection "boolean" for Sensor E (PATH = 1, NO PATH = 0)
F_DETN         DC.B  0  ; Path detection "boolean" for Sensor F (PATH = 1, NO PATH = 0)

RETURN         DC.B  0  ; RETURN (TRUE = 1, FALSE = 0)
NEXT_DIR       DC.B  1  ; Next direction instruction

TEN_THOUS      DS.B  1  ; 10,000 digit
THOUSANDS      DS.B  1  ; 1,000 digit
HUNDREDS       DS.B  1  ; 100 digit
TENS           DS.B  1  ; 10 digit
UNITS          DS.B  1  ; 1 digit
BCD_SPARE      DS.B  10
NO_BLANK       DS.B  1  

  ; Storage for guider sensor readings 
SENSOR_LINE    DC.B  $0 ; Sensor E-F (LINE )  
SENSOR_BOW     DC.B  $0 ; Sensor A (FRONT)    
SENSOR_PORT    DC.B  $0 ; Sensor B (LEFT )    
SENSOR_MID     DC.B  $0 ; Sensor C (MIDDLE)   
SENSOR_STBD    DC.B  $0 ; Sensor D (RIGHT)   
SENSOR_NUM     DS.B  1  ; Current selected sensor

  ; Variables for LCD Displays
TOP_LINE       RMB 20   ; Top line of display
               FCB NULL ; terminated by null
BOT_LINE       RMB 20   ; Bottom line of display
               FCB NULL ; terminated by null
CLEAR_LINE     FCC '                ' ; (around 12 spaces)
               FCB NULL ; terminated by null
TEMP           DS.B  1  ; Temporary memory location

               
               ORG   $4000                     
Entry:                                          
_Startup:                                       
               LDS   #$4000                    
               
               ; initialize ports, A/D, LCD; enable system timer and interrupts 
               JSR   initPORTS                  
               JSR   initAD                    
               JSR   initLCD                   
               JSR   clrLCD                    
               JSR   initTCNT     ; enable the 16-bit TCNT hardware counter                                             
               CLI                ; Enable global interrupts 
               
               ; initial LCD display headings                               
               JSR CLR_LCD_BUF    

  ;MAIN loop                                 
MAIN:          JSR   UPDT_READING  ; Update sensor readings,          
               JSR   UPDT_DISPL    ; Update display            
               LDAA  CRNT_STATE    ; Update current state           
               JSR   DISPATCHER    ; Jump to DISPATCHER subroutine          
               BRA   MAIN 

msg1           DC.B  "S:",0       ; Current state label
msg2           DC.B  "R:",0       ; Sensor readings label
msg3           DC.B  "V:",0       ; Battery voltage label
msg4           DC.B  "B:",0       ; Bumper status label
          
tab:           DC.B  "START  ",0
               DC.B  "FWD    ",0 
               DC.B  "REV    ",0 
               DC.B  "RT_TRN ",0 
               DC.B  "LT_TRN ",0 
               DC.B  "RETURN ",0 
               DC.B  "ALL_STP",0     

; EEBOT Starboard/Port Mechanisms Subroutines ************************************************************************************************************************** 

;------------------------------------------------------------------------------------------
; Starboard (Right) Motor ON                                                               
STARON:        
               BSET  PTT,%00100000
               RTS

;------------------------------------------------------------------------------------------
; Starboard (Right) Motor OFF                                                              
STAROFF:       
               BCLR  PTT,%00100000
               RTS

;------------------------------------------------------------------------------------------
; Starboard (Right) Motor FWD                                                              
STARFWD:       
               BCLR  PORTA,%00000010
               RTS
;------------------------------------------------------------------------------------------
; Starboard (Right) Motor REV                                                              
STARREV:       
               BSET  PORTA,%00000010
               RTS

;------------------------------------------------------------------------------------------
; Port (Left) Motor ON                                                                    
PORTON:        
               BSET  PTT,%00010000
               RTS

;------------------------------------------------------------------------------------------
; Port (Left) Motor OFF                                                                    
PORTOFF:       
               BCLR  PTT,%00010000
               RTS

;------------------------------------------------------------------------------------------
; Port (Left) Motor FWD                                                                   
PORTFWD:       
               BCLR  PORTA,%00000001
               RTS
;------------------------------------------------------------------------------------------
; Port (Left) Motor REV                                                                   
PORTREV:       
               BSET  PORTA,%00000001
               RTS


; EEBOT Roaming States Subroutines ************************************************************************************************************************** @@@@@@@@@@@@@@@@@@

;------------------------------------------------------------------------------------------
DISPATCHER:    CMPA  #START                    
               BNE   NOT_START                 
               JSR   START_ST                  
               RTS                             
;------------------------------------------------------------------------------------------
NOT_START:     CMPA  #FWD                      
               BNE   NOT_FORWARD               
               JMP   FWD_ST                    
;------------------------------------------------------------------------------------------
NOT_FORWARD:   CMPA  #RT_TRN                   
               BNE   NOT_RT_TRN 
               LDY   #1000                      ; 20 ms delay to allow the
               JSR   del_50us               
               JSR   RT_TRN_ST                 
               RTS                             
;------------------------------------------------------------------------------------------
NOT_RT_TRN:    CMPA  #LT_TRN                   
               BNE   NOT_LT_TRN 
               LDY   #1000                      ; 20 ms delay to allow the
               JSR   del_50us               
               JSR   LT_TRN_ST                 
               RTS                             
;------------------------------------------------------------------------------------------
NOT_LT_TRN:    CMPA  #REV                      
               BNE   NOT_REVERSE               
               JSR   REV_ST                    
               RTS                              
;------------------------------------------------------------------------------------------
NOT_REVERSE:   CMPA  #BK_TRK                   
               BNE   NOT_BK_TRK                
               JMP   BK_TRK_ST                 
;------------------------------------------------------------------------------------------
NOT_BK_TRK:    CMPA  #STND_BY                      
               BNE   NOT_STND_BY                   
               JSR   STND_BY_ST                    
               RTS                             
;------------------------------------------------------------------------------------------
NOT_STND_BY:   NOP                             

DISP_EXIT:     RTS                             


; EEBOT Directional Movement Instructions/Subroutines ************************************************************************************************************************** @@@@@@@@@@@@@@@@@@

START_ST:      BRCLR PORTAD0,$04,NO_FWD        ; If "NOT" FWD_BUMP
               JSR   INIT_FWD                  ; Initialize the FORWARD state and
               MOVB  #FWD,CRNT_STATE           ; Set CRNT_STATE to FWD
               BRA   START_EXIT                ; Then exit                                                ;
NO_FWD:        NOP                             ; Else

START_EXIT:    RTS                           	 ; return to the MAIN routine
;------------------------------------------------------------------------------------------
FWD_ST:        PULD                            ; @@@@@@@@

               BRSET PORTAD0,$04,NO_FWD_BUMP   ; If FWD_BUMP detected, then eebot has hit a wall  
               LDAA  SEC_PTH_INT               ; Switch to second path
               STAA  NEXT_DIR                    ; Correct next direction value 
               JSR   INIT_REV                  ; Initialize the REV routine
               MOVB  #REV,CRNT_STATE           ; Set CRNT_STATE to REV
               JMP   FWD_EXIT                  ; and return
              
NO_FWD_BUMP:   BRSET PORTAD0,$08,NO_REAR_BUMP   ; Else if REV_BUMP, then 
               JMP   INIT_STND_BY               ; Initialize the BACKTRACK state
               MOVB  #STND_BY,CRNT_STATE        ; Set CRNT_STATE to STND_BY        @@@@@@@@@@@@@@@@@@@@@@@@@
               JMP   FWD_EXIT                   ; and return

               ; If no bumpers detected, choose a specific subroutine below depending on sensor values   

NO_REAR_BUMP:  LDAA  D_DETN                    ; Detected D sensor; DEFAULT BEHAVIOUR: RIGHT TURN      
               BEQ   NO_D_DETECT               ; if D not detected, branch to other detections NO_RT_INTXN
               
               LDAA  NEXT_DIR                  ; So save direction for the previous intersection to the stack
               PSHA                            ; 
               LDAA  PRI_PTH_INT               ; 
               STAA  NEXT_DIR                  ; Next D = 0 
               
               JSR   INIT_FWD                  ; turns off motors, resets timers, set wheel direction to forward
               MOVB  #FWD, CRNT_STATE          ; When intersection detected, move slightly forward   
               
               LDY   #570                      ; 20 ms delay to allow the
               JSR   del_50us                  ; Delayed-forward movement to land ontop of intersection
               
               JSR   INIT_RT_TRN               ; Initialize the RT_TRN state
               MOVB  #RT_TRN,CRNT_STATE        ; Set CRNT_STATE to RT_TRN
               JMP   FWD_EXIT                  ; Exit out of FWD subroutine   

                                               ; if no sensor D detected, then no Right Turn procedure.
NO_D_DETECT:   LDAA  B_DETN                    ; Else if B_DETN equals 1
               BEQ   NO_B_DETECT               ; Check if A_DETN equals 1
               
               LDAA  A_DETN                    ; If A_DETN is zero (not on dark line) then take left turn, if A is 1 the bot is centered on the path  
               BEQ   LT_TURN                   ; Should continue forward                             
               
               LDAA  NEXT_DIR                  ; Push direction for the previous
               PSHA                            ; Intersection to the stack
               LDAA  PRI_PTH_INT               ; Then track direction taken to NEXT_DIR  
               STAA  NEXT_DIR                    ; ""
               BRA   NO_SHFT_LT                ; Else if A_DETN equals 0

LT_TURN:       LDAA  NEXT_DIR                    ; Push direction for the previous
               PSHA                            ; Intersection to the stack
               LDAA  SEC_PTH_INT               ; Then track direction taken to NEXT_DIR  
               STAA  NEXT_DIR
               
               JSR   INIT_FWD                   ;When intersection detected, move slightly forward   
               MOVB  #FWD, CRNT_STATE           ; ""
               LDY   #570                       ; Delay-controled forward movment to lane ontop of intersection  
               JSR   del_50us                   ; ""
               
               JSR   INIT_LT_TRN               ; Initialize the LT_TRN state
               MOVB  #LT_TRN,CRNT_STATE        ; Set CRNT_STATE to LT_TRN 
               JMP   FWD_EXIT                  ; Exit out of FWD subroutine    

NO_B_DETECT:   LDAA  F_DETN                    ; Else if F sensor is on path,      
               BEQ   NO_SHFT_RT                ; The robot needs to make a slight shift RIGHT to be on path  
               JSR   PORTON                    ; and turn on the LEFT motor

RT_FWD_DIS:    LDD   COUNT2                    ;
               CPD   #INC_DIS                  ;
               BLO   RT_FWD_DIS                ; If Dc>Dfwd then
               JSR   INIT_FWD                  ; Turn motors off
               JMP   FWD_EXIT                  ; Exit out of FWD subroutine    

NO_SHFT_RT:    LDAA  E_DETN                    ; Else if E sensor is on path,                  
               BEQ   NO_SHFT_LT                ; The robot needs to make a slight shift LEFT to be on path   
               JSR   STARON                    ; and turn on the RIGHT motor
LT_FWD_DIS:    LDD   COUNT1                    ;
               CPD   #INC_DIS                  ;
               BLO   LT_FWD_DIS                ; If Dc>Dfwd then
               JSR   INIT_FWD                  ; Turn motors off
               JMP   FWD_EXIT                  ; Exit out of FWD subroutine    

NO_SHFT_LT:    JSR   STARON                    ; Turn motors on
               JSR   PORTON                    ; ""

FWD_STR_DIS:   LDD   COUNT1                    ;
               CPD   #FWD_DIS                  ;
               BLO   FWD_STR_DIS               ; If Dc>Dfwd then
               JSR   INIT_FWD                  ; Turn motors off
                
FWD_EXIT:      JMP   MAIN                      ; return to the MAIN routine

;------------------------------------------------------------------------------------------
REV_ST:        LDD   COUNT1                    ; If Dc>Drev then
               CPD   #REV_DIS                  ; The robot should make a U TURN
               BLO   REV_ST                    ; so
               JSR   STARFWD                   ; Set STBD Motor to FWD direction
               LDD   #0                        ; Reset timer
               STD   COUNT1                    ; ""
                
REV_U_TRN:     LDD   COUNT1                    ; If Dc>Dutrn then
               CPD   #UTRN_DIS                 ; The robot should stop
               BLO   REV_U_TRN                 ; so
               JSR   INIT_FWD                  ; Initialize the FWD state
               LDAA  RETURN                    ; If RETURN equals 1 
               BNE   BK_TRK_REV                ;
               MOVB  #FWD,CRNT_STATE           ; Then set state to FWD
               BRA   REV_EXIT                  ; and exit

BK_TRK_REV:    JSR   INIT_FWD                  ;
               MOVB  #BK_TRK,CRNT_STATE        ; Else set CRNT_STATE to BK_TRK
               
REV_EXIT:      RTS                             ; return to the MAIN routine

;------------------------------------------------------------------------------------------
RT_TRN_ST:     LDD   COUNT2                    ; If Dc>Dfwd then
               CPD   #STR_DIS                  ; The robot should make a TURN
               BLO   RT_TRN_ST                 ; so
               JSR   STAROFF                   ; Set STBD Motor to OFF
               LDD   #0                        ; Reset timer
               STD   COUNT2                    ; ""
                
RT_TURN_DEL:   LDD   COUNT2                    ; If Dc>Dfwdturn then
               CPD   #TRN_DIS                  ; The robot should stop
               BLO   RT_TURN_DEL               ; so
               JSR   INIT_FWD                  ; Initialize the FWD state
               LDAA  RETURN                    ; If RETURN equals 1 
               BNE   BK_TRK_RT_TRN             ;
               MOVB  #FWD,CRNT_STATE           ; Then set state to FWD
               BRA   RT_TRN_EXIT               ; and exit

BK_TRK_RT_TRN: MOVB  #BK_TRK,CRNT_STATE        ; Else set state to BK_TRK
            
RT_TRN_EXIT:   RTS                             ; return to the MAIN routine

;------------------------------------------------------------------------------------------
LT_TRN_ST:     LDD   COUNT1                    ; If Dc>Dfwd then
               CPD   #STR_DIS                  ; The robot should make a TURN
               BLO   LT_TRN_ST                 ; so
               JSR   PORTOFF                   ; Set PORT Motor to OFF
               LDD   #0                        ; Reset timer
               STD   COUNT1                    ; ""
                
LT_TURN_DEL:   LDD   COUNT1                    ; If Dc>Dfwdturn then
               CPD   #TRN_DIS                  ; The robot should stop
               BLO   LT_TURN_DEL               ; so
               JSR   INIT_FWD                  ; Initialize the FWD state
               LDAA  RETURN                    ; If RETURN equals 1 
               BNE   BK_TRK_LT_TRN             ;
               MOVB  #FWD,CRNT_STATE           ; Then set state to FWD
               BRA   LT_TRN_EXIT               ; and exit

BK_TRK_LT_TRN: MOVB  #BK_TRK,CRNT_STATE        ; Else set state to BK_TRK

LT_TRN_EXIT:   RTS                             ; return to the MAIN routine

;------------------------------------------------------------------------------------------ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
BK_TRK_ST:     PULD                            ;
               BRSET PORTAD0,$08,NO_BK_BUMP    ; If REV_BUMP, then we should stop
               JSR   INIT_STND_BY              ; Initialize the STANDBY state
               MOVB  #STND_BY,CRNT_STATE       ; set the state to STND_BY
               JMP   BK_TRK_EXIT               ; and exit

NO_BK_BUMP:    LDAA  NEXT_DIR                    ; If NEXT_DIR equals 0
               BEQ   GOOD_PATH                 ; branch to "good" pathing 
               BNE   BAD_PATH                  ; branch to "bad" (alternate) pathing 

;--------------------------------------------------------------------------------------------  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
GOOD_PATH:     LDAA  D_DETN                    ; If D_DETN equals 1
               BEQ   NO_RT_TRN                 ; The robot should make a RIGHT turn
               
               PULA                            ; Pull the next direction value from the stack
               PULA                            ; and store it in NEXT_DIR
               STAA  NEXT_DIR                  
               JSR   INIT_RT_TRN               ; Initialize the RT_TRN state
               MOVB  #RT_TRN,CRNT_STATE        ; Set CRNT_STATE to RT_TRN
               JMP   BK_TRK_EXIT               ; Then exit

NO_RT_TRN:     LDAA  B_DETN                    ; If B_DETN equals 1
               BEQ   RT_LINE_S                 ; Check if A_DETN equals 1
               LDAA  A_DETN                    ; If A_DETN equals 1 a FORWARD path exists
               BEQ   LEFT_TURN                 ; The robot should continue forward
               PULA                            ; Pull the next direction value from the stack
               PULA                            ; and store it in NEXT_DIR
               STAA  NEXT_DIR                    ; ""
               BRA   NO_LINE_S                 ; Else if A_DETN equals 0

LEFT_TURN:     PULA                            ; The robot should make a LEFT turn
               PULA                            ; Pull the next direction value from the stack
               STAA  NEXT_DIR                  ; and store it in NEXT_DIR
               JSR   INIT_LT_TRN               ; Initialize the LT_TRN state
               MOVB  #LT_TRN,CRNT_STATE        ; Set CRNT_STATE to LT_TRN
               JMP   BK_TRK_EXIT               ; and exit

;--------------------------------------------------------------------------------------------
BAD_PATH:      LDAA  B_DETN                    ; If B_DETN equals 1
               BEQ   NO_LT_TRN                 ; The robot should make a LEFT turn
               PULA                            ; Pull the next direction value from the stack
               STAA  NEXT_DIR                  ; and store it in NEXT_DIR
               JSR   INIT_LT_TRN               ; Initialize the LT_TRN state
               MOVB  #LT_TRN,CRNT_STATE        ; Set CRNT_STATE to LT_TRN
               JMP   BK_TRK_EXIT               ; and exit

NO_LT_TRN:     LDAA  D_DETN                    ; If D_DETN equals 1
               BEQ   RT_LINE_S                 ; Check if A_DETN equals 1
               LDAA  A_DETN                    ; If A_DETN equals 1 a FORWARD path exists
               BEQ   RIGHT_TURN                ; The robot should continue forward
               PULA                            ; Pull the next direction value from the stack
               STAA  NEXT_DIR                    ; and store it in NEXT_DIR
               BRA   NO_LINE_S                 ; Else if A_DETN equals 0

RIGHT_TURN:    PULA                            ; The robot should make a RIGHT turn
               STAA  NEXT_DIR                    ; Pull the next direction value from the stack
               JSR   INIT_RT_TRN               ; Initialize the RT_TRN state
               MOVB  #RT_TRN,CRNT_STATE        ; Set CRNT_STATE to RT_TRN
               JMP   BK_TRK_EXIT               ; Then exit

;--------------------------------------------------------------------------------------------
RT_LINE_S:     LDAA  F_DETN                    ; Else if F_DETN equals 1
               BEQ   LT_LINE_S                 ; The robot should shift RIGHT
               JSR   PORTON                    ; and turn on the LEFT motor

RT_FWD_D:      LDD   COUNT2                    ;
               CPD   #INC_DIS                  ; 
               BLO   RT_FWD_D                  ; If Dc>Dfwd then
               JSR   INIT_FWD                  ; Turn motors off
               JMP   BK_TRK_EXIT               ; and exit

LT_LINE_S:     LDAA  E_DETN                    ; Else if F_DETN equals 1
               BEQ   NO_LINE_S                 ; The robot should shift RIGHT
               JSR   STARON                    ; and turn on the LEFT motor

LT_FWD_D:      LDD   COUNT1                    ;
               CPD   #INC_DIS                  ;
               BLO   LT_FWD_D                  ; If Dc>Dfwd then
               JSR   INIT_FWD                  ; Turn motors off
               JMP   BK_TRK_EXIT               ; and exit

NO_LINE_S:     JSR   STARON                    ; Turn motors on
               JSR   PORTON                    ; ""

FWD_STR_D:     LDD   COUNT1                    ;
               CPD   #FWD_DIS                  ;
               BLO   FWD_STR_D                 ; If Dc>Dfwd then
               JSR   INIT_FWD                  ; Turn motors off
                
BK_TRK_EXIT:   JMP   MAIN                      ; return to the MAIN routine

;--------------------------------------------------------------------------------------------
STND_BY_ST:        BRSET PORTAD0,$04,NO_START      ; If FWD_BUMP
               BCLR  PTT,%00110000             ; Initialize the START state
               MOVB  #START,CRNT_STATE         ; Set CRNT_STATE to START
               BRA   STND_BY_EXIT                  ; Then exit
                                                ;
NO_START:      NOP                             ; Else

STND_BY_EXIT:      RTS                             ; return to the MAIN routine

  ; Initialize the EEBOT Roaming State 
;--------------------------------------------------------------------------------------------
INIT_FWD:      BCLR  PTT,%00110000             ; Turn OFF the drive motors
               LDD   #0                        ; Reset timer to start from zero   
               STD   COUNT1                    ; ""
               STD   COUNT2                    ; ""
               BCLR  PORTA,%00000011           ; Set FWD direction for both motors
               RTS

;--------------------------------------------------------------------------------------------
INIT_REV:      BSET  PORTA,%00000011           ; Set REV direction for both motors
               LDD   #0                        ; Reset timer to start from zero   
               STD   COUNT1                    ; ""
               BSET  PTT,%00110000             ; Turn ON the drive motors
               RTS

;--------------------------------------------------------------------------------------------
INIT_RT_TRN:   BCLR  PORTA,%00000011           ; Set FWD direction for both motors
               LDD   #0                        ; Reset timer to start from zero   
               STD   COUNT2                    ; ""
               BSET  PTT,%00110000             ; Turn ON the drive motors
               RTS

;--------------------------------------------------------------------------------------------
INIT_LT_TRN:   BCLR  PORTA,%00000011           ; Set FWD direction for both motors
               LDD   #0                        ;; Reset timer to start from zero   
               STD   COUNT1                    ; ""
               BSET  PTT,%00110000             ; Turn ON the drive motors
               RTS

;--------------------------------------------------------------------------------------------
INIT_BK_TRK:   INC   RETURN                    ; Change RETURN value to 1
               PULA                            ; Pull the next direction value from the stack
               STAA  NEXT_DIR                    ; and store it in NEXT_DIR
               JSR   INIT_REV                  ; Initialize the REVERSE routine
               JSR   REV_ST                    ; Jump to REV_ST
               JMP   MAIN

;--------------------------------------------------------------------------------------------
INIT_STND_BY:  BCLR  PTT,%00110000             ; Turn off the drive motors
               RTS
                

; Sensor Readings Subroutines **************************************************************************************************************************
;--------------------------------------------------------------------------------------------
UPDT_READING:  JSR   G_LEDS_ON                 ; Turn ON LEDS
               JSR   READ_SENSORS              ; Take readings from sensors
               JSR   G_LEDS_OFF                ; Turn OFF LEDS
                
               LDAA  #0                        ; Set sensor A detection value to 0
               STAA  A_DETN                    ; Sensor A
               STAA  B_DETN                    ; Sensor B
               STAA  C_DETN                    ; Sensor C
               STAA  D_DETN                    ; Sensor D
               STAA  E_DETN                    ; Sensor E
               STAA  F_DETN                    ; Sensor F
               
CHECK_A:       LDAA  SENSOR_BOW                ; If SENSOR_BOW is GREATER than
               CMPA  #PTH_A_INT                ; Specific A Sensor value while on path   
               BLO   CHECK_B                   ; Else, leave A_DETN = 0 and move onto B   
               INC   A_DETN                    ; Set A_DETN to 1

CHECK_B:       LDAA  SENSOR_PORT               ; If SENSOR_PORT is GREATER than
               CMPA  #PTH_B_INT                ; Specific B Sensor value while on path   
               BLO   CHECK_C                   ; Else, leave B_DETN = 0 and move onto C   
               INC   B_DETN                    ; Set B_DETN to 1

CHECK_C:       LDAA  SENSOR_MID                ; If SENSOR_MID is GREATER than
               CMPA  #PTH_C_INT                ; Specific C Sensor value while on path  
               BLO   CHECK_D                   ; Else, leave C_DETN = 0 and move onto D   
               INC   C_DETN                    ; Set C_DETN to 1
                
CHECK_D:       LDAA  SENSOR_STBD               ; If SENSOR_STBD is GREATER than
               CMPA  #PTH_D_INT                ; Specific D Sensor value while on path   
               BLO   CHECK_E                   ; Else, leave D_DETN = 0 and move onto E   
               INC   D_DETN                    ; Set D_DETN to 1

CHECK_E:       LDAA  SENSOR_LINE               ; If SENSOR_LINE is LESS than
               CMPA  #PTH_E_INT                ; Specific E Sensor value while on path   
               BHI   CHECK_F                   ; Else, leave E_DETN = 0 and move onto F   
               INC   E_DETN                    ; Set E_DETN to 1
                
CHECK_F:       LDAA  SENSOR_LINE               ; If SENSOR_LINE is GREATER than
               CMPA  #PTH_F_INT                ; Specific F Sensor value while on path   
               BLO   READ_COMPL                ; Else, leave F_DETN = 0 and exit subroutine   
               INC   F_DETN                    ; Set F_DETN to 1
                
READ_COMPL:    RTS

;--------------------------------------------------------------------------------------------
G_LEDS_ON:     BSET  PORTA,%00100000           ; Set bit 5
               RTS

G_LEDS_OFF:    BCLR  PORTA,%00100000           ; Clear bit 5
               RTS

;--------------------------------------------------------------------------------------------
READ_SENSORS:  CLR   SENSOR_NUM                ; Select sensor number 0
               LDX   #SENSOR_LINE              ; Point at the start of the sensor array

RS_MAIN_LOOP:  LDAA  SENSOR_NUM                ; Select the correct sensor input
               JSR   SELECT_SENSOR             ; on the hardware
               LDY   #250                      ; 20 ms delay to allow the
               JSR   del_50us                  ; sensor to stabilize
               LDAA  #%10000001                ; Start A/D conversion on AN1
               STAA  ATDCTL5
               BRCLR ATDSTAT0,$80,*            ; Repeat until A/D signals done
               LDAA  ATDDR0L                   ; A/D conversion is complete in ATDDR0L
               STAA  0,X                       ; so copy it to the sensor register
               CPX   #SENSOR_STBD              ; If this is the last reading
               BEQ   RS_EXIT                   ; Then exit
               INC   SENSOR_NUM                ; Else, increment the sensor number
               INX                             ; and the pointer into the sensor array
               BRA   RS_MAIN_LOOP              ; and do it again
RS_EXIT:       RTS

;--------------------------------------------------------------------------------------------
SELECT_SENSOR: PSHA                            ; Save the sensor number for the moment
               LDAA  PORTA                     ; Clear the sensor selection bits to zeros
               ANDA  #%11100011                
               STAA  TEMP                      ; and save it into TEMP
               PULA                            ; Get the sensor number
               ASLA                            ; Shift the selection number left, twice
               ASLA
               ANDA  #%00011100                ; Clear irrelevant bit positions
               ORAA  TEMP                      ; OR it into the sensor bit positions
               STAA  PORTA                     ; Update the hardware
               RTS


;LCD Subroutines ************************************************************************************************************************** 
del_50us:      PSHX                            ;2 E-clk Protect the X register

eloop:         LDX   #300                      ;2 E-clk Initialize the inner loop counter

iloop:         NOP                             ;1 E-clk No operation
               DBNE  X,iloop                   ;3 E-clk If the inner cntr not 0, loop again
               DBNE  Y,eloop                   ;3 E-clk If the outer cntr not 0, loop again
               PULX                            ;3 E-clk Restore the X register
               RTS                             ;5 E-clk Else return
;--------------------------------------------------------------------------------------------
cmd2LCD:       BCLR  LCD_CNTR,LCD_RS           ; select the LCD Instruction Register (IR)
               JSR   dataMov                   ; send data to IR
      	       RTS

;--------------------------------------------------------------------------------------------
putsLCD        LDAA  1,X+                      ; get one character from the string
               BEQ   donePS                    ; reach NULL character?
               JSR   putcLCD
               BRA   putsLCD

donePS 	       RTS

;--------------------------------------------------------------------------------------------
putcLCD        BSET  LCD_CNTR,LCD_RS           ; select the LCD Data register (DR)
               JSR   dataMov                   ; send data to DR
               RTS

;--------------------------------------------------------------------------------------------
dataMov        BSET  LCD_CNTR,LCD_E            ; pull the LCD E-sigal high
               STAA  LCD_DAT                   ; send the upper 4 bits of data to LCD
               BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.
               LSLA                            ; match the lower 4 bits with the LCD data pins
               LSLA                            ; -"-
               LSLA                            ; -"-
               LSLA                            ; -"-
               BSET  LCD_CNTR,LCD_E            ; pull the LCD E signal high
               STAA  LCD_DAT                   ; send the lower 4 bits of data to LCD
               BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.
               LDY   #1                        ; adding this delay will complete the internal
               JSR   del_50us                  ; operation for most instructions
               RTS
;--------------------------------------------------------------------------------------------
int2BCD        XGDX                            ; Save the binary number into .X
               LDAA  #0                        ; Clear the BCD_BUFFER
               STAA  TEN_THOUS
               STAA  THOUSANDS
               STAA  HUNDREDS
               STAA  TENS
               STAA  UNITS
               STAA  BCD_SPARE
               STAA  BCD_SPARE+1

               CPX   #0                        ; Check for a zero input
               BEQ   CON_EXIT                  ; and if so, exitentry

               XGDX                            ; Not zero, get the binary number back to .D as dividend
               LDX   #10                       ; Setup 10 (Decimal!) as the divisor
               IDIV                            ; Divide: Quotient is now in .X, remainder in .D
               STAB  UNITS                     ; Store remainder
               CPX   #0                        ; If quotient is zero,
               BEQ   CON_EXIT                  ; then exit

               XGDX                            ; else swap first quotient back into .D
               LDX   #10                       ; and setup for another divide by 10
               IDIV
               STAB  TENS
               CPX   #0
               BEQ   CON_EXIT

               XGDX                            ; Swap quotient back into .D
               LDX   #10                       ; and setup for another divide by 10
               IDIV
               STAB  HUNDREDS
               CPX   #0
               BEQ   CON_EXIT

               XGDX                            ; Swap quotient back into .D
               LDX   #10                       ; and setup for another divide by 10
               IDIV
               STAB  THOUSANDS
               CPX   #0
               BEQ   CON_EXIT

               XGDX                            ; Swap quotient back into .D
               LDX   #10                       ; and setup for another divide by 10
               IDIV
               STAB  TEN_THOUS

CON_EXIT:      RTS                             ; 

;--------------------------------------------------------------------------------------------
BCD2ASC:       LDAA  #$0                       ; Initialize the blanking flag
               STAA  NO_BLANK

C_TTHOU:       LDAA  TEN_THOUS                 ; 
               ORAA  NO_BLANK
               BNE   NOT_BLANK1

ISBLANK1:      LDAA  #$20                      ; 
               STAA  TEN_THOUS                 ; 
               BRA   C_THOU                    ; 

NOT_BLANK1:    LDAA  TEN_THOUS                 ; 
               ORAA  #$30                      ; Convert to ascii
               STAA  TEN_THOUS
               LDAA  #$1                       ; 
               STAA  NO_BLANK

C_THOU:        LDAA  THOUSANDS                 ; Check the thousands digit for blankness
               ORAA  NO_BLANK                  ; 
               BNE   NOT_BLANK2
                     
ISBLANK2:      LDAA  #$30                      ; Thousands digit is blank
               STAA  THOUSANDS                 ; so store a space
               BRA   C_HUNS                    ; and check the hundreds digit

NOT_BLANK2:    LDAA  THOUSANDS                 ; 
               ORAA  #$30
               STAA  THOUSANDS
               LDAA  #$1
               STAA  NO_BLANK

C_HUNS:        LDAA  HUNDREDS                  ; Check the hundreds digit for blankness
               ORAA  NO_BLANK                  ; 
               BNE   NOT_BLANK3

ISBLANK3:      LDAA  #$20                      ; Hundreds digit is blank
               STAA  HUNDREDS                  ; so store a space
               BRA   C_TENS                    ; and check the tens digit
                     
NOT_BLANK3:    LDAA  HUNDREDS                  ; 
               ORAA  #$30
               STAA  HUNDREDS
               LDAA  #$1
               STAA  NO_BLANK

C_TENS:        LDAA  TENS                      ; Check the tens digit for blankness
               ORAA  NO_BLANK                  ; 
               BNE   NOT_BLANK4
                     
ISBLANK4:      LDAA  #$20                      ; Tens digit is blank
               STAA  TENS                      ; so store a space
               BRA   C_UNITS                   ; and check the units digit

NOT_BLANK4:    LDAA  TENS                      ; 
               ORAA  #$30
               STAA  TENS

C_UNITS:       LDAA  UNITS                     ; No blank check necessary, convert to ascii.
               ORAA  #$30
               STAA  UNITS

               RTS                             ; 

;--------------------------------------------------------------------------------------------
HEX_TABLE:     FCC '0123456789ABCDEF'          ; Table for converting values

BIN2ASC:       PSHA                            ; Save a copy of the input number on the stack
               TAB                             ; and copy it into ACCB
               ANDB #%00001111                 ; Strip off the upper nibble of ACCB
               CLRA                            ; D now contains 000n where n is the LSnibble
               ADDD #HEX_TABLE                 ; Set up for indexed load
               XGDX                
               LDAA 0,X                        ; Get the LSnibble character
               PULB                            ; Retrieve the input number into ACCB
               PSHA                            ; and push the LSnibble character in its place
               RORB                            ; Move the upper nibble of the input number
               RORB                            ;  into the lower nibble position.
               RORB
               RORB 
               ANDB #%00001111                 ; Strip off the upper nibble
               CLRA                            ; D now contains 000n where n is the MSnibble 
               ADDD #HEX_TABLE                 ; Set up for indexed load
               XGDX                                                               
               LDAA 0,X                        ; Get the MSnibble character into ACCA
               PULB                            ; Retrieve the LSnibble character into ACCB
               RTS

;--------------------------------------------------------------------------------------------

DP_FRONT_SENSOR EQU TOP_LINE+9
DP_PORT_SENSOR  EQU BOT_LINE+6
DP_MID_SENSOR   EQU BOT_LINE+9
DP_STBD_SENSOR  EQU BOT_LINE+12
DP_LINE_SENSOR  EQU BOT_LINE+15

UPDT_DISPL     LDAA SENSOR_BOW     ; Get the FRONT sensor value
                
               JSR BIN2ASC          ; Convert to ascii string in D
               LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
               STD 0,X              ; and write the 2 ascii digits there
               LDAA SENSOR_PORT     ; Repeat for the PORT value
               JSR BIN2ASC
               LDX #DP_PORT_SENSOR
               STD 0,X
               LDAA SENSOR_MID      ; Repeat for the MID value
               JSR BIN2ASC
               LDX #DP_MID_SENSOR
               STD 0,X
                
               LDAA SENSOR_STBD     ; Repeat for the STARBOARD value
               JSR BIN2ASC
               LDX #DP_STBD_SENSOR
               STD 0,X
               LDAA SENSOR_LINE     ; Repeat for the LINE value
               JSR BIN2ASC
               LDX #DP_LINE_SENSOR
               STD 0,X
               LDAA #CLEAR_HOME     ; Clear the display and home the cursor
               JSR cmd2LCD          ; _"_
               LDY #40              ; Wait 2 ms until "clear display" command is complete
               JSR del_50us
               LDX #TOP_LINE        ; Now copy the buffer top line to the LCD
               JSR putsLCD
               LDAA #LCD_SEC_LINE   ; Position the LCD cursor on the second line
               JSR LCD_POS_CRSR
               LDX #BOT_LINE        ; Copy the buffer bottom line to the LCD
               JSR putsLCD
               MOVB  #$90,ATDCTL5        ; R-just., uns., sing. conv., mult., ch=0, start
               BRCLR ATDSTAT0,$80,*      ; Wait until the conver. seq. is complete
               LDAA  ATDDR0L             ; Load the ch0 result - battery volt - into A
			
                                        ; Display the battery voltage
			         MOVB  #$90,ATDCTL5        ; R-just., uns., sing. conv., mult., ch=0, start
               BRCLR ATDSTAT0,$80,*      ; Wait until the conver. seq. is complete
               LDAA  ATDDR0L             ; Load the ch0 result - battery volt - into A
               LDAB  #39                 ; AccB = 39
               MUL                       ; AccD = 1st result x 39
               ADDD  #600                ; AccD = 1st result x 39 + 600
               JSR   int2BCD
               JSR   BCD2ASC
               LDAA  #$8F                ; move LCD cursor to the 1st row, end of msg1
               JSR   cmd2LCD             ; "
               LDAA  TEN_THOUS           ; output the TEN_THOUS ASCII character
               JSR   putcLCD             ; "
               LDAA  THOUSANDS
               JSR   putcLCD
               LDAA  #$2E
               JSR   putcLCD
               LDAA  HUNDREDS
               JSR   putcLCD             ; Display the battery voltage
;-------------------------
               LDAA  #$80                ; Move LCD cursor to the 2nd row, end of msg2
               JSR   cmd2LCD             ;
               LDAB  CRNT_STATE          ; Display current state
               LSLB                      ; "
               LSLB                      ; "
               LSLB                      ; "
               LDX   #tab                ; "
               ABX                       ; "
               JSR   putsLCD             ; 
               RTS
               
               
               
              


UPDT_DIS_EXIT: RTS                             ; and exit

;---------------------------------------------------------------------------
; Clear LCD Buffer
; This routine writes ?space? characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
; display routine should maintain the buffer properly.

CLR_LCD_BUF:   LDX #CLEAR_LINE
               LDY #TOP_LINE
               JSR STRCPY
            
CLB_SECOND:    LDX #CLEAR_LINE
               LDY #BOT_LINE
               JSR STRCPY

CLB_EXIT:      RTS

;---------------------------------------------------------------------------
; String Copy
; Copies a null-terminated string (including the null) from one location to
; another
; Passed: X contains starting address of null-terminated string
; Y contains first address of destination

STRCPY:        PSHX ; Protect the registers used
               PSHY
               PSHA

STRCPY_LOOP:   LDAA 0,X ; Get a source character
               STAA 0,Y ; Copy it to the destination
               BEQ STRCPY_EXIT ; If it was the null, then exit
            
               INX ; Else increment the pointers
               INY
               BRA STRCPY_LOOP ; and do it again
               
STRCPY_EXIT:   PULA ; Restore the registers
               PULY
               PULX
               RTS 



                
; Initialization of various ports and control registers ************************************************************************************************************************** 

;--------------------------------------------------------------------------------------------
initPORTS:     BCLR  DDRAD,$FF                 ; Set PORTAD as input
               BSET  DDRA, $FF                 ; Set PORTA as output
               BSET  DDRT, $30                 ; Set channels 4 & 5 of PORTT as output
               RTS
;--------------------------------------------------------------------------------------------        
initAD:        MOVB  #$C0,ATDCTL2              ; power up AD, select fast flag clear
               JSR   del_50us                  ; wait for 50 us
               MOVB  #$00,ATDCTL3              ; 8 conversions in a sequence
               MOVB  #$85,ATDCTL4              ; res=8, conv-clks=2, prescal=12
               BSET  ATDDIEN,$0C               ; configure pins AN03,AN02 as digital inputs
               RTS   
;--------------------------------------------------------------------------------------------
initLCD:       BSET  DDRB,%11111111            ; configure pins PB7,...,PB0 for output
               BSET  DDRJ,%11000000            ; configure pins PJ7(E), PJ6(RS) for output
               LDY   #2000                     ; wait for LCD to be ready
               JSR   del_50us                  ; -"-
               LDAA  #$28                      ; set 4-bit data, 2-line display
               JSR   cmd2LCD                   ; -"-
               LDAA  #$0C                      ; display on, cursor off, blinking off
               JSR   cmd2LCD                   ; -"-
               LDAA  #$06                      ; move cursor right after entering a character
               JSR   cmd2LCD                   ; -"-
               RTS

;--------------------------------------------------------------------------------------------
clrLCD:        LDAA  #$01                      ; clear cursor and return to home position
               JSR   cmd2LCD                   ; -"-
               LDY   #40                       ; wait until "clear cursor" command is complete
               JSR   del_50us                  ; -"-
               RTS
;--------------------------------------------------------------------------------------------               
; Position the Cursor 
LCD_POS_CRSR:  ORAA #%10000000     ; Set the high bit of the control word
               JSR cmd2LCD         ; and set the cursor address
               RTS               
               

; Timer Systems
;--------------------------------------------------------------------------------------------
initTCNT:      MOVB  #$80,TSCR1                ; enable TCNT
               MOVB  #$00,TSCR2                ; disable TCNT OVF interrupt, set prescaler to 1
               MOVB  #$FC,TIOS                 ; channels PT1/IC1,PT0/IC0 are input captures
               MOVB  #$05,TCTL4                ; capture on rising edges of IC1,IC0 signals
               MOVB  #$03,TFLG1                ; clear the C1F,C0F input capture flags
               MOVB  #$03,TIE                  ; enable interrupts for channels IC1,IC0
               RTS

; First Interrupt @@@@@@@@@
ISR1           MOVB  #$01,TFLG1                ; clear the C0F input capture flag
               INC   COUNT1                    ; increment COUNT1
               RTI

;Second Interrupt @@@@@@@@@
ISR2           MOVB  #$02,TFLG1                ; clear the C1F input capture flag
               INC   COUNT2                    ; increment COUNT2 
               RTI

               
               
               
               ;********************************************************************************************
               ;* Interrupt Vectors                                                                        *
               ;********************************************************************************************
               ORG   $FFFE
               DC.W  Entry                     ; Reset Vector

               ORG   $FFEE
               DC.W  ISR1                      ; COUNT1 INT

               ORG   $FFEC
               DC.W  ISR2                      ; COUNT2 INT


