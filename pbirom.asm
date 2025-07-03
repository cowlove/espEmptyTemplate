PDVMSK  =   $0247   ;Parallel device mask (indicates which are
NDEVREQ =   $0248   ; activated PBI device
PDIMSK  =   $0249   ;Parallel interrupt mask (not used in this
GPDVV   =   $E48F   ;Generic Parallel Device Vector
GENDEV  =   $E48F
HATABS  =   $031A   ;Device handler table
CRITIC  =   $42     ;Critical code section flag
DEVNAM  =   'J

NEWDEV  =   $E486

IOCBCHIDZ = $20

* = $d800
;* = $0600

.word    $fff,                       // D800 ROM cksum lo
.byt    $01,                        // D802 ROM version
.byt    $80,                        // D803 ID num
.byt    $01,                        // D804 Device Type
jmp PBI_IO
jmp PBI_ISR                         // 8,9,a
.byt    $91,                        // D80B ID num 2 (0x91)
.byt    DEVNAM,                        // D80C Device Name (ASCII)
.word PBI_OPEN - 1
.word PBI_CLOSE - 1
.word PBI_GETB - 1
.word PBI_PUTB - 1
.word PBI_STATUS - 1
.word PBI_SPECIAL - 1
jmp PBI_INIT                        // D819,A,B
.byt $ff
.byt $0
.byt $0
.byt $0

ESP32_IOCB_REQ
    .byt $0     ;  request - 6502 sets to 1 after filling out ESP32_IOCB struct, esp32 clears after handling
ESP32_IOCB_A
    .byt $ee     ;  A - iocb index * $20 
ESP32_IOCB_X
    .byt $ee     ;  X -  
ESP32_IOCB_Y
    .byt $ee    ;  Y -  
ESP32_IOCB_CMD
    .byt $ee     ;  CMD 
ESP32_IOCB_CARRY
    .byt $ee

TEST_ENTRY
    PLA
    JMP SAFE_WAIT

PBI_INIT
    nop
    nop
    lda PDVMSK  //;Get enabled device flags
    ora #1      //;Set bit 0.
    sta PDVMSK  //;& replace.

 ;Put device name in Handler table HATABS
     LDX #0
 ;        Top of loop
 SEARCH
     LDA HATABS,X ;Get a byte from table
     BEQ FNDIT   ;0? Then we found space.
     INX 
     INX 
     INX 
     CPX #36     ;Length of HATABS
     BCC SEARCH  ;Still looking
     RTS         ;No room in HATABS; device not initialized
 ;
 ;         We found a spot.
 FNDIT
     LDA #DEVNAM ;Get device name.
     STA HATABS,X ;Put it in blank spot.
     LDA #GPDVV&$FF ;Get lo byte of vector.
     STA HATABS+1,X
     LDA #GPDVV/$0100 ;Get hi byte of vector.
     STA HATABS+2,X


    ;ldx #DEVNAM
    ;lda GENDEV / $100
    ;ldy GENDEV & $ff
    ;jsr NEWDEV		//; returns: N = 1 - failed, C = 0 - success, C =1 - entry already exists

    ldx #COPY_END-COPY_BEGIN-1
L1
    lda COPY_BEGIN,x
    sta $0600,x
    dex
    bpl L1
    rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; SIO ROUTINES 

PBI_IO
    sta ESP32_IOCB_A
    lda #7
    jmp PBI_ALL

PBI_ISR     
    sta ESP32_IOCB_A
    lda #8
    jmp PBI_ALL

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
// CIO ROUTINES 

PBI_OPEN
// check IOCBCHIDZ see if this is for us
    sta ESP32_IOCB_A
    lda #1 
    JMP PBI_ALL
PBI_CLOSE
    sta ESP32_IOCB_A
    lda #2 // cmd close
    JMP PBI_ALL
PBI_GETB
    sta ESP32_IOCB_A
    lda #3 // cmd close
    JMP PBI_ALL
PBI_PUTB
    sta ESP32_IOCB_A
    lda #4 // cmd close
    JMP PBI_ALL
PBI_STATUS
    sta ESP32_IOCB_A
    lda #5 // cmd close
    JMP PBI_ALL
PBI_SPECIAL
    sta ESP32_IOCB_A
    lda #6 // cmd close
PBI_ALL
    sta ESP32_IOCB_CMD
    stx ESP32_IOCB_X
    sty ESP32_IOCB_Y

    ///////////////////////////////////////////////////
    // TODO: replace this PBI_WAITREQ loop with:
    // jsr SAFE_WAIT
    // OPTION 1 
#if 0
    lda #1
    sta ESP32_IOCB_REQ
PBI_WAITREQ
    lda ESP32_IOCB_REQ
    bne PBI_WAITREQ

#else
    //////////////////////////////////////////////////
    // OPTION 2
    jsr SAFE_WAIT
#endif

    lda ESP32_IOCB_CARRY
    ror  
    lda ESP32_IOCB_A
    ldx ESP32_IOCB_X
    ldy ESP32_IOCB_Y
    rts 

//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
// Simple test code copied into page 6 by PBI_INIT 

#if 1
COPY_BEGIN
TEST_MPD
    pla
    ldx #$ff
L2
    ldy #$ff
L3
   lda #1
    sta $d1ff
    lda #0
    sta $d1ff
    dey
    bpl L3
    dex
    bpl L2
    rts
COPY_END
#endif 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Busy wait in RAM while the PBI ROM is mapped out
;; To avoid having to find free ram to do this, put the small 6-byte
;; program on the stack and call it

SAFE_WAIT
    // push mini-program on stack in reverse order
    ldx #(stack_res_wait_end - stack_res_wait - 1)
push_prog_loop
    lda stack_res_wait,x
    pha
    dex
    bpl push_prog_loop

    tsx       ; stack pointer now points to newly-placed program - 1 

    lda #(return_from_stackprog - 1) / $100     // push JUMP_BACK -1 onto stack for RTS 
    pha                                         // from mini-program
    lda #(return_from_stackprog - 1) & $ff      // 
    pha    

    lda #$01      // push 16-bit address of stack-resident mini-prog onto stack 
    pha
    txa 
    pha

    lda #1                      //  
    rts                         // jump to mini-prog

return_from_stackprog
    tsx
    txa
    clc
    adc #stack_res_wait_end - stack_res_wait
    tax
    txs
    rts        

stack_res_wait
    sta ESP32_IOCB_REQ      // called with req value in A
stack_res_loop
    lda ESP32_IOCB_REQ
    bne stack_res_loop
    rts
stack_res_wait_end


