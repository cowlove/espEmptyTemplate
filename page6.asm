* = $0600

TESTAREA = $8000
SCREENMEM = $9c40
CMD
.byt $00
MAGIC1
.byt $00
.byt $00
.byt $00
.byt $00
ERRCOUNT
.byt $00
.byt $00
.byt $00
.byt $00


pla

    lda #$de
    sta MAGIC1
    lda #$ad
    sta MAGIC1 + 1
    lda #$be
    sta MAGIC1 + 2
    lda #$ef
    sta MAGIC1 + 3

    // get ESP32 to modify memory 
    lda #1
    sta CMD
WAIT4
    lda CMD
    bne WAIT4   

    ldy #0
LOOP1
    ldx #0
    lda #0
LOOP2
    sta TESTAREA,x
    sta TESTAREA+$100,x
    sta TESTAREA+$200,x
    sta TESTAREA+$300,x
    sta TESTAREA,x
    sta TESTAREA+$100,x
    sta TESTAREA+$200,x
    sta TESTAREA+$300,x
    clc
    adc #1
    inx
    bne LOOP2

    ldx #0
    lda #0
LOOP3
    cmp TESTAREA,x
    bne ERR1
    cmp TESTAREA+$100,x
    bne ERR1
    cmp TESTAREA+$200,x
    bne ERR1
    cmp TESTAREA+$300,x
    bne ERR1
    cmp TESTAREA+$400,x
    bne ERR1
    cmp TESTAREA+$500,x
    bne ERR1
    cmp TESTAREA+$600,x
    bne ERR1
    cmp TESTAREA+$700,x
    beq OK1
ERR1
    jsr LOG_ERROR1
OK1    
    clc
    adc #1
    inx
    bne LOOP3

    iny
    bne LOOP1

    inc SCREENMEM
    lda #1
    sta CMD

    jmp LOOP1

LOG_ERROR1
    pha
    lda SCREENMEM+2
    clc
    adc #1
    sta SCREENMEM+2
    jmp LOG_ERROR

LOG_ERROR2
    pha
    lda SCREENMEM+4
    clc
    adc #1
    sta SCREENMEM+4
    txa 
    sta SCREENMEM+5

    //jmp LOG_ERROR

LOG_ERROR
    sec
    lda #0
    adc ERRCOUNT
    sta ERRCOUNT
    lda #0
    adc ERRCOUNT+1
    sta ERRCOUNT+1
    lda #0
    adc ERRCOUNT+2
    sta ERRCOUNT+2
    lda #0
    adc ERRCOUNT+3
    sta ERRCOUNT+3
    pla
    rts

