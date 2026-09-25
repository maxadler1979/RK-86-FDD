; FDD BIOS for Computer "Radio 86RK"
; (c) 09-10-2014 vinxru (aleksey.f.morozov@gmail.com)

     .org 07600h-683 ;      075FFh
                       
;----------------------------------------------------------------------------

INIT_VIDEO      = 0F82DH
USER_PORT       = 0A000H    ;  58055
INIT_STACK      = 076CFh
SEND_MODE       = 10000000b ;   (1 0 0 A H 0 B CL)
RECV_MODE       = 10010000b ;   (1 0 0 A H 0 B CL)

ERR_START   	= 040h
ERR_WAIT    	= 041h
ERR_OK_NEXT 	= 042h
ERR_OK          = 043h
ERR_OK_READ     = 044h
ERR_OK_ENTRY    = 045h
ERR_OK_WRITE	= 046h
ERR_OK_ADDR  	= 047h
ERR_OK_BLOCK    = 04Fh 

VER_BUF = 0

;----------------------------------------------------------------------------
;  RK 

     .db ($+2)>>8, ($+2)&0FFh
     
;----------------------------------------------------------------------------
	      
Entry:
     ;    
     LXI	H, SELF_NAME
     CALL	0F833h

     ;     
     LXI	H, aHello
     CALL	0F818h

     ;   
     CALL	PrintVer

     ;  
     lxi	h, aCrLf
     CALL	0F818h

     ;   SHELL.RK   
     LXI	H, aShellRk
     LXI	D, aEmpty
     CALL	CmdExec
     PUSH	PSW

     ;  -   
     CPI	04h
     JNZ 	Error2

     ;   "   BOOT/SHELL.RK"
     LXI	H, aErrorShellRk
     CALL	0F818h
     JMP	$

;----------------------------------------------------------------------------

PrintVer:
     ;   
     MVI	A, 1
     CALL	StartCommand	;      
     CALL	SwitchRecv
     
     ;      
     LXI	B, VER_BUF
     LXI	D, 18          ; 1   ,   -  
     CALL	RecvBlock
          
     ;   
     XRA	A
     STA	VER_BUF+17
     LXI	H, VER_BUF+1
     JMP 	0F818h

;----------------------------------------------------------------------------

aHello:         .db 13,10,"FDD BIOS V1",13,10
aSdController:  .db "FDD CONTROLLER ",0
aCrLf:          .db 13,10,0
aErrorShellRk:  .db "fajl ne najden "
aShellRk:       .db "BOOT/SHELL.RK",0
                .db "(c) 04-05-2014 vinxru"

;         

SELF_NAME    = $-512 ;  ( 256 )
CMD_LINE     = $-256 ;   256 

;----------------------------------------------------------------------------
;   SD BIOS
;----------------------------------------------------------------------------

aError:    .db "o{ibka FDD"
aEmpty:    .db 0

;----------------------------------------------------------------------------
;   ,      

Error:     
     ;  
     LXI	SP, INIT_STACK

     ;   
     PUSH	PSW

     ;  
     ;         ,    
     MVI	C, 1Fh
     CALL	0F809h     
     ;    
     CALL       INIT_VIDEO

Error2:
     ;   " SD "
     LXI	H, aError
     CALL	0F818h

     ;   
     POP	PSW
     CALL	0F815h

     ; 
     JMP	$

;----------------------------------------------------------------------------

BiosEntry:
     PUSH       H
     LXI	H, JmpTbl
     ADD	L
     MOV	L, A
     MOV	L, M
     XTHL
     RET

;----------------------------------------------------------------------------
;  8D00.   JmpTbl    

JmpTbl:
     .db CmdExec           ; 0 HL- , DE-   / A- 
     .db CmdFind           ; 1 HL- , DE-   , BC- / HL- , A- 
     .db CmdOpenDelete     ; 2 D-, HL-  / A- 
     .db CmdSeekGetSize    ; 3 B-, DE:HL- / A- , DE:HL-
     .db CmdRead           ; 4 HL-, DE- / HL- , A- 
     .db CmdWrite          ; 5 HL-, DE- / A- 
     .db CmdMove           ; 6 HL-, DE- / A- 

;----------------------------------------------------------------------------
; HL-, DE-   , BC- / HL- , A- 

CmdFind:
     ;  
     MVI	A, 3
     CALL	StartCommand

     ; 
     CALL	SendString

     ;  
     XCHG
     CALL	SendWord

     ;    
     CALL	SwitchRecv

     ; 
     LXI	H, 0

CmdFindLoop:
     ;    
     CALL	WaitForReady
     CPI	ERR_OK
     JZ		Ret0
     CPI	ERR_OK_ENTRY
     JNZ	EndCommand

     ;   
     LXI	D, 20	;  
     CALL	RecvBlock

     ;   
     INX	H

     ; 
     JMP	CmdFindLoop

;----------------------------------------------------------------------------
; D-, HL-  / A- 

CmdOpenDelete: 
     ;  
     MVI	A, 4
     CALL	StartCommand

     ; 
     MOV	A, D
     CALL	Send

     ;  
     CALL	SendString

     ;    
     CALL	SwitchRecvAndWait
     CPI	ERR_OK
     JZ		Ret0
     JMP	EndCommand
     
;----------------------------------------------------------------------------
; B-, DE:HL- / A- , DE:HL-

CmdSeekGetSize:
     ;  
     MVI 	A, 5
     CALL	StartCommand

     ;      
     MOV	A, B
     CALL	Send

     ;      
     CALL	SendWord
     XCHG
     CALL	SendWord

     ;    .     ERR_OK
     CALL	SwitchRecvAndWait
     CPI	ERR_OK
     JNZ	EndCommand

     ;  
     CALL	RecvWord
     XCHG
     CALL	RecvWord

     ; 
     JMP	Ret0
     
;----------------------------------------------------------------------------
; HL-, DE- / HL- , A- 

CmdRead:
     ;  
     MVI	A, 6
     CALL	StartCommand

     ;   BC
     MOV	B, D
     MOV	C, E

     ;  
     CALL	SendWord        ; HL-

     ;    
     CALL	SwitchRecv

     ;  .    BC,    HL
     JMP	RecvBuf

;----------------------------------------------------------------------------
; HL-, DE- / A- 

CmdWrite:
     ;  
     MVI	A, 7
     CALL	StartCommand
     
     ;  
     CALL	SendWord        ; HL-

     ;    HL
     XCHG

CmdWriteFile2:
     ;   
     CALL	SwitchRecvAndWait
     CPI  	ERR_OK
     JZ  	Ret0
     CPI  	ERR_OK_WRITE
     JNZ	EndCommand

     ;  ,      DE
     CALL	RecvWord

     ;        
     CALL	SwitchSend

     ;  .  BC  DE. (  )
CmdWriteFile1:
     MOV	A, M
     INX	H
     CALL	Send
     DCX	D
     MOV	A, D
     ORA	E
     JNZ 	CmdWriteFile1

     JMP	CmdWriteFile2

;----------------------------------------------------------------------------
; HL-, DE- / A- 

CmdMove:     
     ;  
     MVI	A, 8
     CALL	StartCommand

     ;  
     CALL	SendString

     ;    
     CALL	SwitchRecvAndWait
     CPI	ERR_OK_WRITE
     JNZ	EndCommand

     ;    
     CALL	SwitchSend

     ;  
     XCHG
     CALL	SendString

WaitEnd:
     ;    
     CALL	SwitchRecvAndWait
     CPI	ERR_OK
     JZ		Ret0
     JMP	EndCommand

;----------------------------------------------------------------------------
; HL- , DE-  / A- 

CmdExec:
     ;  
     MVI	A, 2
     CALL	StartCommand

     ;  
     PUSH	H
     CALL	SendString
     POP	H

     ;     
     ;     ERR_OK_ADDR
     CALL	SwitchRecvAndWait
     CPI	ERR_OK_ADDR
     JNZ	EndCommand

     ;    (HL-)
     PUSH	D
     XCHG
     LXI	H, SELF_NAME
     CALL	strcpy255
     POP	D

     ;    (DE-)
     LXI	H, CMD_LINE
     CALL	strcpy255

     ; ***   .     . ***

     ;   (  )
     LXI	SP, INIT_STACK

     ;     BC     
     CALL	RecvWord
     PUSH	D
     MOV 	B, D
     MOV 	C, E

     ;  
     CALL	RecvBuf
     JNZ 	Error

     ;  
     ;         ,    
     MVI	C, 1Fh
     CALL	0F809h     
     ;    
     CALL       INIT_VIDEO

     ;   
     MVI  A, 1		;  
     LXI  B, BiosEntry  ;   SD BIOS
     LXI  D, SELF_NAME  ;  
     LXI  H, CMD_LINE   ;  

     ;   
     RET

;----------------------------------------------------------------------------
;    .   8E00.
;----------------------------------------------------------------------------

;----------------------------------------------------------------------------
;   . 
; A -  

StartCommand:
     ;      
     ;  256 ,      256+ 
     ;     - ,    
     PUSH	B
     PUSH	H
     PUSH	PSW
     MVI	C, 0

StartCommand1:
     ;   ( )   HL
     CALL       SwitchRecv

     ;    (  )
     LXI	H, USER_PORT+1
     MVI        M, 0
     MVI        M, 44h
     MVI        M, 40h
     MVI        M, 0h

     ;   ,    ERR_START
     CALL	Recv
     CPI	ERR_START
     JZ		StartCommand2

     ; .     256  (   
     ;  64  ,   )
     PUSH	B
     MVI	C, 0
StartCommand3:
     CALL	Recv
     DCR	C
     JNZ	StartCommand3
     POP	B
        
     ; 
     DCR	C
     JNZ	StartCommand1    

     ;  
     MVI	A, ERR_START
StartCommandErr2:
     POP	B ;   PSW
     POP	H ;   H
     POP	B ;   B     
     POP	B ;   .
     RET

;----------------------------------------------------------------------------
;    .    ERR_OK_NEXT

StartCommand2:
     ;          	
     CALL	WaitForReady
     CPI	ERR_OK_NEXT
     JNZ	StartCommandErr2

     ;    
     CALL       SwitchSend

     POP        PSW
     POP        H
     POP        B

     ;   
     JMP        Send

;----------------------------------------------------------------------------
;    

SwitchSend:
     CALL	Recv
SwitchSend0:
     MVI	A, SEND_MODE
     STA	USER_PORT+3
     RET

;----------------------------------------------------------------------------
;    
;   ,     

Ret0:
     XRA	A

;----------------------------------------------------------------------------
;      A 
;   ,     

EndCommand:
     PUSH	PSW
     CALL	Recv
     POP	PSW
     RET

;----------------------------------------------------------------------------
;    DE 
;  A.

RecvWord:
    CALL Recv
    MOV  E, A
    CALL Recv
    MOV  D, A
    RET
    
;----------------------------------------------------------------------------
;    HL 
;  A.

SendWord:
    MOV		A, L
    CALL	Send
    MOV		A, H
    JMP		Send
    
;----------------------------------------------------------------------------
;  
; HL - 
;  A.

SendString:
     XRA	A
     ORA	M
     JZ		Send
     CALL	Send
     INX	H
     JMP	SendString
     
;----------------------------------------------------------------------------
;    

SwitchRecv:
     MVI	A, RECV_MODE
     STA	USER_PORT+3
     RET

;----------------------------------------------------------------------------
;        .

SwitchRecvAndWait:
     CALL SwitchRecv

;----------------------------------------------------------------------------
;   .

WaitForReady:
     CALL	Recv
     CPI	ERR_WAIT
     JZ		WaitForReady
     RET

;----------------------------------------------------------------------------
;  DE    BC
;  A

RecvBlock:
     PUSH	H
     LXI 	H, USER_PORT+1
     INR 	D
     XRA 	A
     ORA 	E
     JZ 	RecvBlock2
RecvBlock1:
     MVI        M, 20h			; 7
     MVI        M, 0			; 7
     LDA	USER_PORT		; 13
     STAX	B		        ; 7
     INX	B		        ; 5
     DCR	E		        ; 5
     JNZ	RecvBlock1		; 10 = 54
RecvBlock2:
     DCR	D
     JNZ	RecvBlock1
     POP	H
     RET

;----------------------------------------------------------------------------
;     BC. 
;   HL  
;  A
;    ,   Z=1

RecvBuf:
     LXI	H, 0
RecvBuf0:   
     ; 
     CALL	WaitForReady
     CPI	ERR_OK_READ
     JZ		Ret0		;   Z ( )
     SUI        ERR_OK_BLOCK
     JNZ	EndCommand	;   NZ ()

     ;     DE
     CALL	RecvWord

     ;  HL  
     DAD D

     ;  DE    BC
     CALL	RecvBlock

     JMP	RecvBuf0

;----------------------------------------------------------------------------
;     256  ( )

strcpy255:
     MVI  B, 255
strcpy255_1:
     LDAX D
     INX  D
     MOV  M, A
     INX  H
     ORA  A
     RZ
     DCR  B
     JNZ  strcpy255_1
     MVI  M, 0 ; 
     RET

;----------------------------------------------------------------------------
;    A.

Send:
     STA	USER_PORT

;----------------------------------------------------------------------------
;    

Recv:
     MVI	A, 20h
     STA	USER_PORT+1
     XRA	A
     STA	USER_PORT+1
     LDA	USER_PORT
     RET

;----------------------------------------------------------------------------

.End