; rom_emu.s — tight ROM serve for Monitor R (IAR STM8)
; Serves rom_ram[addr] on PORTB; exits on address seq 44h->40h->00.
;
; Address: A0=PD0 A1=PD2 A2=PD3 A3=PD4 A4=PC4 A5=PC5 A6=PC6 A7=PC7
; STM8 ALU has no OR/ADD A,XL — merge via rom_addr_hi in RAM.

        NAME    rom_emu

        PUBLIC  romEmulation
        EXTERN  rom_ram
        EXTERN  rom_addr_hi

PB_ODR  EQU     05005h
PB_DDR  EQU     05007h
PB_CR1  EQU     05008h
PC_IDR  EQU     0500Bh
PD_IDR  EQU     05010h

        SECTION .near_func.text:CODE:REORDER:NOROOT(0)

romEmulation:
        ; Ensure Port B out: PP mix, OD on PB4/PB5 (CR1=0xCF), DDR=FF
        LD      A, #0CFh
        LD      PB_CR1, A
        LD      A, #0FFh
        LD      PB_DDR, A

loop0:
        LD      A, PC_IDR
        AND     A, #0F0h
        LD      rom_addr_hi, A

        LD      A, PD_IDR
        AND     A, #01Fh
        CLRW    X
        LD      XL, A
        LD      A, (pd_lut,X)
        OR      A, rom_addr_hi

        AND     A, #07Fh
        CLRW    X
        LD      XL, A
        LD      A, (rom_ram,X)
        LD      PB_ODR, A

        LD      A, XL
        CP      A, #044h
        JRNE    loop0

loop1:
        LD      A, PC_IDR
        AND     A, #0F0h
        LD      rom_addr_hi, A

        LD      A, PD_IDR
        AND     A, #01Fh
        CLRW    X
        LD      XL, A
        LD      A, (pd_lut,X)
        OR      A, rom_addr_hi

        AND     A, #07Fh
        CLRW    X
        LD      XL, A
        LD      A, (rom_ram,X)
        LD      PB_ODR, A

        LD      A, XL
        CP      A, #044h
        JREQ    loop1
        CP      A, #040h
        JRNE    loop0

loop2:
        LD      A, PC_IDR
        AND     A, #0F0h
        LD      rom_addr_hi, A

        LD      A, PD_IDR
        AND     A, #01Fh
        CLRW    X
        LD      XL, A
        LD      A, (pd_lut,X)
        OR      A, rom_addr_hi

        AND     A, #07Fh
        CLRW    X
        LD      XL, A
        LD      A, (rom_ram,X)
        LD      PB_ODR, A

        LD      A, XL
        CP      A, #040h
        JREQ    loop2
        TNZ     A
        JRNE    loop0
        RET

        SECTION .near.rodata:CONST:REORDER:NOROOT(0)
pd_lut:
        DC8     0,1,0,1,2,3,2,3,4,5,4,5,6,7,6,7
        DC8     8,9,8,9,10,11,10,11,12,13,12,13,14,15,14,15

        END
