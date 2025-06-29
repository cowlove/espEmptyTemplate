.byt     $0f,                     // D800 ROM cksum lo
.byt    $0f,                     // D801 ROM cksum hi
.byt    $01,                     // D802 ROM version
.byt    $80,                     // D803 ID num
.byt    $01,                     // D804 Device Type
.byt    $4c,                     // 5
.byt    $00,                     // 6
.byt    $00,                     // 7   
jmp PBI_ISR
.byt    $91,                     // D80B ID num 2 (0x91)
.byt    $65,                     // D80C Device Name (ASCII)
.byt    $00,                        // D
.byt    $00,                        // E
.byt    $00,                        // F
.byt    $00,                         // 0
.byt    $00,                         // 1
.byt    $00,                         // 2
.byt    $00,                          // 3
.byt    $00,                          // 4
.byt    $00,                          // 5
.byt    $00,                          // 6
.byt    $00,                          // 7
.byt    $00,                          // 8
jmp PBI_INIT
    
PBI_INIT 
    lda #$68
    sta $0600
    lda #$60
    sta $0601
    rts


PBI_ISR     rts 