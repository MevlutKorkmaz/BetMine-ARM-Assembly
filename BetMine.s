	
	.section .vectors, "ax"
	B _start // reset vector
	B SERVICE_UND // undefined instruction vector
	B SERVICE_SVC // software interrrupt vector
	B SERVICE_ABT_INST // aborted prefetch vector
	B SERVICE_ABT_DATA // aborted data vector
	.word 0 // unused vector
	B SERVICE_IRQ // IRQ interrupt vector
	B SERVICE_FIQ // FIQ interrupt vector
.text
.global _start
_start:
	/* Set up stack pointers for IRQ and SVC processor modes */
	MOV R1, #0b11010010 // interrupts masked, MODE = IRQ
	MSR CPSR_c, R1 // change to IRQ mode
	LDR SP, =0xFFFFFFFF - 3 // set IRQ stack to A9 onchip memory

	/* Change to SVC (supervisor) mode with interrupts disabled */
	MOV R1, #0b11010011 // interrupts masked, MODE = SVC
	MSR CPSR, R1 // change to supervisor mode
	LDR SP, =0x3FFFFFFF - 3 // set SVC stack to top of DDR3 memory
	
BL CONFIG_GIC // configure the ARM GIC
BL CONFIG_KEYS // configure the pushbutton KEYs
BL CONFIG_INTERVAL_TIMER // configure the FPGA interval timer
BL CONFIG_PRIVATE_TIMER // configure the PRIVATE timer

	// enable IRQ interrupts in the processor
	MOV R0, #0b01010011 // IRQ unmasked, MODE = SVC
	MSR CPSR_c, R0

IDLE:
ldr r0,=GAME_STARTED
ldr r0,[r0]
cmp r0,#2

blEQ main

B IDLE // main program simply idles

main:
PUSH {r0-r12,LR}
LDR R2,=BALANCE
LDR R0,=BET_AMOUNT
//UPDATE BALANCE
LDR R3,[R0] //BET AMOUNT
LDR R1,[R2] //BALANCE
SUB R1,R1,R3
STR R1,[R2]
IS_MINE:
ldr r0,=GAME_STARTED
	LDR R1, [R0]
	CMP R1,#3
	Beq CHECK_OUT
LDR R0,=0xFF200000
LDR R1,[R0] //LED
LDR R2,=0xFF200040
LDR R3,[R2] //SWICH
LDR R4,=MINE_LOCATIONS
LDR R5,[R4] //MINE_LOCATION
LDR R6,=CURRENT_LED_STATUS
LDR R7,[R6] //CURRENT_LED_STATUS
ORR R8,R7,R3
CMP R7,R8 //CHECK IF ANY NEW SWICH TURNED
STRNE R8,[R6] //STORE NEW CURRENT LED STATUS
STRNE R8,[R0] //SHOW CLEARED LOCATIONS
ANDNE R8,R8,R5
BEQ IS_MINE
CMP R8,#0 //CHECK IF NEW INPUT IS MINE
BNE LOST_SEQUENCE
BLEQ INCREASE_BET_AMOUNT



B IS_MINE
//RESET GAME VARIABLES AND UPDATE NEW BALANCE
CHECK_OUT:
MOV R1,#0
//RESETTING VARIABLES
LDR R0,=0xFF200000
STR R1,[R0]
LDR R0,=CURRENT_LED_STATUS
STR R1,[R0]
LDR R2,=NUMBER_OF_MINES
STR R1,[R2] 
LDR R2,=MINE_LOCATIONS
STR R1,[R2] 
LDR R2,=GAME_STARTED
STR R1,[R2] 
LDR R2,=PREVIOUS_BET_AMOUNT
STR R1,[R2] 
LDR R2,=BALANCE
LDR R0,=BET_AMOUNT
//UPDATE BALANCE
LDR R3,[R0] //BET AMOUNT
LDR R1,[R2] //BALANCE
ADD R1,R1,R3
STR R1,[R2]
MOV R1,#100
STR R1,[R0]
POP {r0-r12,PC}//GOES BACK TO IDLE TO GET NEW DATA FOR THE NEXT ROUND


INCREASE_BET_AMOUNT:
PUSH {r0-r12,LR}
LDR R0,=BET_AMOUNT
LDR R1,=NUMBER_OF_MINES
LDR R2,[R0]//BET_AMOUNT
LDR R3,[R1]//NUMBER_OF_MINES
LDR R4,=MULTIPLIER
LDR R5,[R4,R3,LSL #2] //GET MULTIPLIER ACCORDING TO MINE NUMBER
MUL R2,R2,R5
MOV R6,#0
DIVIDER_LOOP:
CMP R2,#10
SUBGE R2,R2,#10
ADDGE R6,R6,#1
BGE DIVIDER_LOOP
STR R6,[R0]
BL SEVEN_SEGMENT_SLOW_INCREASE
LDR R0,=PREVIOUS_BET_AMOUNT
STR R6,[R0] //TO START COUNTING FROM LAST BET AMOUNT
POP {r0-r12,PC}
LOST_SEQUENCE:
LDR R0,=BET_AMOUNT
MOV R1, #0
STR R1,[R0]
PUSH {r0-r12}
	ldr r3, =BET_AMOUNT
	ldr r0, [r3]		//given number value in r0
	BL SEVEN_SEGMENT_DISPLAYER
POP {r0-r12}
LOST_SEQUENCE2:

LDR R0,=0xFF200000
LDR R1,[R0] //LED
LDR R2,=MINE_LOCATIONS
LDR R3,[R2] //MINE_LOCATION
LDR R4,=CURRENT_LED_STATUS
LDR R5,[R4] //CURRENT_LED_STATUS
ORR R6,R3,R5
EOR R7,R6,R3
STR R7,[R0]
DELAY: 
LDR R7, =2000000 // delay counter
DELAY_LOOP:
SUBS R7, R7, #1
BNE DELAY_LOOP
STR R6,[R0]
DELAY1: 
LDR R7, =2000000 // delay counter
DELAY_LOOP1:
SUBS R7, R7, #1
BNE DELAY_LOOP1

ldr r0,=GAME_STARTED
	LDR R1, [R0]
	CMP R1,#3
	Beq CHECK_OUT
	

B LOST_SEQUENCE2
/*************************************************************************
* Pushbutton - Interrupt Service Routine
*
* This routine checks which KEY has been pressed. It writes to HEX0
************************************************************************/
.equ KEY_BASE, 0xFF200050
.equ LED_BASE, 0xFF200000
.equ SWITCH_BASE, 0xFF200040
CURRENT_LED_STATUS: .WORD 0

NUMBER_OF_MINES: .word 0
GAME_STARTED: .word 0
MINE_LOCATIONS: .WORD 0
LOCATIONS: .WORD 0,1,2,3,4,5,6,7,8,9
MULTIPLIER: .WORD 1,13,15,19,22,30,10,150
BET_AMOUNT: .WORD 100
PREVIOUS_BET_AMOUNT:.WORD 0
BALANCE: .WORD 1000
KEY_ISR:

	LDR R0, =KEY_BASE // base address of pushbutton KEY port
	LDR R1, [R0, #0xC] // read edge capture register
	MOV R2, #0xF
	STR R2, [R0, #0xC] // clear the interrupt
	

CHECK_KEY0://GET THE NUMBER OF MINES FOR THE PLAY
	MOV R3, #0x1
	ANDS R3, R3, R1 // check for KEY0
	BEQ CHECK_KEY1
	ldr r0,=GAME_STARTED
	LDR R1, [R0]
	CMP R1,#0
	BNE SHOW_MINE_NUM
	//GET THE NUMBER OF MINES
	LDR R0, =NUMBER_OF_MINES
	LDR R1, [R0]
	CMP R1,#7
	moveq r1,#0
	ADD R1,R1,#1
	STR R1, [R0]
	SHOW_MINE_NUM:
	PUSH {r0-r12,LR}
	ldr r3, =NUMBER_OF_MINES
	ldr r0, [r3]		//given number value in r0
	BL SEVEN_SEGMENT_DISPLAYER
	POP {r0-r12,PC}
	B END_KEY_ISR
CHECK_KEY1:
	MOV R3, #0x2
	ANDS R3, R3, R1 // check for KEY1
	BEQ CHECK_KEY2
	ldr r0,=GAME_STARTED
	LDR R1, [R0]
	CMP R1,#1
	BNE SHOW_BET
	//DECREASE THE BET AMOUNT
	LDR R0, =BET_AMOUNT
	LDR R1, [R0]
	CMP R1,#20
	moveq r1,#30
	SUB R1,R1,#10
	STR R1, [R0]
	SHOW_BET:
	PUSH {r0-r12,LR}
	ldr r3, =BET_AMOUNT
	ldr r0, [r3]		//given number value in r0
	BL SEVEN_SEGMENT_DISPLAYER
	POP {r0-r12,PC}
	B END_KEY_ISR
CHECK_KEY2:
	MOV R3, #0x4
	ANDS R3, R3, R1 // check for KEY2
	BEQ IS_KEY3
	ldr r0,=GAME_STARTED
	LDR R1, [R0]
	CMP R1,#1
	BNE SHOW_BALANCE
	//INCREASE THE BET AMOUNT
	LDR R0, =BET_AMOUNT
	LDR R1, [R0]
	CMP R1,#500
	ldrEQ r1,=490
	ADD R1,R1,#10
	STR R1, [R0]
	SHOW_BALANCE:
	PUSH {r0-r12,LR}
	ldr r0,=GAME_STARTED
	LDR R1, [R0]
	CMP R1,#1
	LDRNE R3,=BALANCE
	ldrEQ r3, =BET_AMOUNT
	ldr r0, [r3]		//given number value in r0
	BL SEVEN_SEGMENT_DISPLAYER
	POP {r0-r12,PC}
	B END_KEY_ISR
IS_KEY3:// START THE GAME //game satutus changer
	MOV R3, #0x8
	ANDS R3, R3, R1 // check for KEY2
	BEQ END_KEY_ISR
	ldr r0,=GAME_STARTED
	LDR R1, [R0] 
	add R1,R1,#1
	STR R1,[R0]
	CMP R1, #1
	Beq GET_MINE_LOCATIONS
END_KEY_ISR:
	BX LR


GET_MINE_LOCATIONS:
PUSH {r0-r12,LR}
LDR R0,=NUMBER_OF_MINES
LDR R1,[R0] //NUMBER OF MINES
LDR R0,=LOCATIONS
mov r7,#9
ldr R6,=100
LOOP://RANDOMIZE THE ARRAY
ldr r2, =0xFFFEC600
ldr r3, [r2,#4] //prıvate tımer current value
and r3,r3, #15
cmp r3,#9
bgt LOOP
LDR R2,[R0,R3, LSL #2] //
SUB R4,R7,R3
LDR R5,[R0,R4, LSL #2]
STR R2,[R0,R4, LSL #2]
STR R5,[R0,R3, LSL #2]
SUBS R6,R6,#1
BGT LOOP
LDR R0,=NUMBER_OF_MINES
LDR R1,[R0] //NUMBER OF MINES
LDR R0,=LOCATIONS
MOV R2,#1
MOV R4,#0// ACTUAL MINE LOCATIONS
LOOP2: //GET NEEDED NUMBER OF ELEMENTS
LDR R3,[R0],#4
LSL R3,R2,R3
ORR R4,R4,R3
SUBS R1,R1,#1
BGt LOOP2
LDR R0,=MINE_LOCATIONS
STR R4,[R0]
LDR R0,=0xff200000
STR R4,[R0]
POP {r0-r12,PC}
B END_KEY_ISR
SEVEN_SEGMENT_DISPLAYER:
ldr r5, =DIVISORS
ldr r6,[r5]		//divisor in r6
mov r2,#0
mov r10,#0
counterfordec:
mov r1,#0 //COUNTER FOR EVERY DECİMAL BIT
add r2,#1	//COUNTER TO CHECK WE RUN 6 TIMES TOTAL, FOR EVERY DECİMAL BIT ONE TIME
cmp r2,#4
bgt FIN
divisor:
cmp r0,r6
blt smallerdivisor	//AFTER CHECKING OUR NUMBER IS SMALLER THAN OUR DIVISOR WE CHANGE TO SMALLER ONE
sub r0,r0,r6
add r1,r1,#1
b divisor
smallerdivisor:
ldr r6,[r5,#4]! //GET THE NEXT DIVISOR FROM LIST OF DIVISORS
ldr r3, =HEXTABLE
ldr R12, [R3, r1, LSL #2]
orr r10,r10,r12
cmp r2,#4
lsllt r10,#8	//TO STORE THEM IN ONE REGISTER
b counterfordec	//TO UPDATE VARIABLES
FIN:
ldr r2, =ADDR_7SEG1
str r10,[r2]
bx lr

HEXTABLE:.word 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
DIVISORS:.WORD 0x3E8,0x64,0xA,0x1
.equ ADDR_7SEG1, 0xFF200020


SEVEN_SEGMENT_SLOW_INCREASE:
PUSH {r0-r12,LR}
/*TASK4*/	 

ldr r0, =PREVIOUS_BET_AMOUNT
ldr r3,[r0]	
SLOWstart:
ldr r1, =SLOWHEXTABLE
ldr r2, =ADDR_7SEG1
mov r0,r3
ldr r5, =SLOWDIVISORS
ldr r6,[r5]		//divisors in r6
push {r1-r4}

mov r2,#0
mov r10,#0

bl SLOWcounterfordec

pop {r1-r4}

str r10,[r2]

SLOWDO_DELAY: 
LDR R7, =2000 // delay counter
SLOWSUB_LOOP:
SUBS R7, R7, #1
BNE SLOWSUB_LOOP

LDR R9,=BET_AMOUNT
LDR R9,[R9]
cmp r3,R9
add r3,#1
beq SLOWEND
b SLOWstart

SLOWcounterfordec:
mov r1,#0 //COUNTER FOR EVERY DECİMAL BIT
add r2,#1		//COUNTER TO CHECK WE RUN 4 TIMES TOTAL, FOR EVERY DECİMAL BIT ONE TIME
cmp r2,#4
bgt SLOWFIN

SLOWdivisor:
cmp r0,r6
blt SLOWsmallerdivisor	//AFTER CHECKING OUR NUMBER IS SMALLER THAN OUR DIVISOR WE CHANGE TO SMALLER ONE

sub r0,r0,r6
add r1,r1,#1

b SLOWdivisor

SLOWsmallerdivisor:

ldr r6,[r5,#4]! //GET THE NEXT DIVISOR
ldr r3, =SLOWHEXTABLE
ldr R12, [R3, r1, LSL #2]
orr r10,r10,r12
cmp r2,#4
lsllt r10,#8	//only shift r2<4

b SLOWcounterfordec	//TO UPDATE VARIABLES


SLOWFIN:bx lr

SLOWEND:
POP {r0-r12,PC}
 

SLOWHEXTABLE:.word 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
SLOWDIVISORS:.WORD 0x3E8,0x64,0xA,0x1






/* Define the exception service routines */
/*--- Undefined instructions --------------------------------------------------*/
SERVICE_UND:
	B SERVICE_UND
/*--- Software interrupts -----------------------------------------------------*/
SERVICE_SVC:
	B SERVICE_SVC
/*--- Aborted data reads ------------------------------------------------------*/
SERVICE_ABT_DATA:
	B SERVICE_ABT_DATA
/*--- Aborted instruction fetch -----------------------------------------------*/
SERVICE_ABT_INST:
	B SERVICE_ABT_INST

/*--- IRQ ---------------------------------------------------------------------*/
SERVICE_IRQ:
	PUSH {R0-R7, LR}
	/* Read the ICCIAR from the CPU Interface */
	LDR R4, =0xFFFEC100
	LDR R5, [R4, #0x0C] // read from ICCIAR
	
INTERVAL_TIMER_CHECK:
CMP R5, #72 // check for FPGA timer interrupt
BNE PRIVATE_TIMER_CHECK
BL INTERVAL_TIMER_ISR
B EXIT_IRQ
PRIVATE_TIMER_CHECK:
CMP R5, #29 // check for FPGA timer interrupt
BNE FPGA_IRQ1_HANDLER
BL PRIVATE_TIMER_ISR
B EXIT_IRQ
FPGA_IRQ1_HANDLER:
	CMP R5, #73
UNEXPECTED:
	BNE UNEXPECTED // if not recognized, stop here
	BL KEY_ISR
EXIT_IRQ:
	/* Write to the End of Interrupt Register (ICCEOIR) */
	STR R5, [R4, #0x10] // write to ICCEOIR
	POP {R0-R7, LR}
	SUBS PC, LR, #4
/*--- FIQ ---------------------------------------------------------------------*/
	SERVICE_FIQ:
	B SERVICE_FIQ

/* ^^^^ END of Define the exception service routines ^^^^ */


PRIVATE_TIMER_ISR:

LDR R1, =0xFFFEC600 // interval timer base address
MOV R0, #1
STR R0, [R1,#12] // clear the interrupt

BX LR

INTERVAL_TIMER_ISR:

LDR R1, =0xFF202000 // interval timer base address
MOVS R0, #0
STR R0, [R1] // clear the interrupt

BX LR


//CONFIGURATIONS

/* Configure the Generic Interrupt Controller (GIC)
	*/
CONFIG_GIC:
	PUSH {LR}
/* To configure the FPGA KEYS interrupt (ID 73):
* 1. set the target to cpu0 in the ICDIPTRn register
* 2. enable the interrupt in the ICDISERn register */

/* CONFIG_INTERRUPT (int_ID (R0), CPU_target (R1)); */
	MOV R0, #73 // KEY port (Interrupt ID = 73)
	MOV R1, #1 // this field is a bit-mask; bit 0 targets cpu0
	BL CONFIG_INTERRUPT
	MOV R0, #29 // Private timer port (Interrupt ID = 29)
	MOV R1, #1
	BL CONFIG_INTERRUPT
	MOV R0, #72 // Interval timer port (Interrupt ID = 72)
	MOV R1, #1
	BL CONFIG_INTERRUPT
	/* configure the GIC CPU Interface */
	LDR R0, =0xFFFEC100 // base address of CPU Interface
	
	/* Set Interrupt Priority Mask Register (ICCPMR) */
	LDR R1, =0xFFFF // enable interrupts of all priorities levels
	STR R1, [R0, #0x04]
	
	/* Set the enable bit in the CPU Interface Control Register (ICCICR).
	* This allows interrupts to be forwarded to the CPU(s) */
	MOV R1, #1
	STR R1, [R0]
	
	/* Set the enable bit in the Distributor Control Register (ICDDCR).
	* This enables forwarding of interrupts to the CPU Interface(s) */
	LDR R0, =0xFFFED000
	STR R1, [R0]
	POP {PC}

/* Configure the pushbutton KEYS to generate interrupts */
CONFIG_KEYS:
// write to the pushbutton port interrupt mask register
LDR R0, =0xFF200050 // pushbutton key base address
MOV R1, #0xF // set interrupt mask bits
STR R1, [R0, #0x8] // interrupt mask register is (base + 8)
BX LR
	
/* Configure the FPGA interval timer to create interrupts at 50-msec intervals */
CONFIG_INTERVAL_TIMER:
LDR R0, =0xFF202000 // Interval timer base address
LDR R1, =5000000 // 1/(100 MHz) ×(5000000) = 50 msec
STR R1, [R0, #0x8] // store the low half word of counter start value
LSR R1, R1, #16
STR R1, [R0, #0xC] // high half word of counter start value
// start the interval timer, enable its interrupts
MOV R1, #0x7 // START = 1, CONT = 1, ITO = 1
STR R1, [R0, #0x4]
BX LR

/* Configure the Private timer to generate interrupts */
CONFIG_PRIVATE_TIMER:
LDR R0, =0xFFFEC600 // Interval timer base address
LDR R1, =5000000 // 1/(200 MHz) ×(5000000) = 100 msec
STR R1, [R0] // write to timer load register
MOV R2, #0b111 // set bits: mode = 1 (auto), enable = 1
STR R2, [R0, #0x8] // write to timer control register
BX LR

/*
* Configure registers in the GIC for an individual Interrupt ID
* We configure only the Interrupt Set Enable Registers (ICDISERn) and
* Interrupt Processor Target Registers (ICDIPTRn). The default (reset)
* values are used for other registers in the GIC
* Arguments: R0 = Interrupt ID, N
* R1 = CPU target
*/
CONFIG_INTERRUPT:
	PUSH {R4-R5, LR}
/* Configure Interrupt Set-Enable Registers (ICDISERn).
* reg_offset = (integer_div(N / 32) * 4
* value = 1 << (N mod 32) */
	LSR R4, R0, #3 // calculate reg_offset
	BIC R4, R4, #3 // R4 = reg_offset
	LDR R2, =0xFFFED100
	ADD R4, R2, R4 // R4 = address of ICDISER
	AND R2, R0, #0x1F // N mod 32
	MOV R5, #1 // enable
	LSL R2, R5, R2 // R2 = value
	
/* Using the register address in R4 and the value in R2 set the
* correct bit in the GIC register */
	LDR R3, [R4] // read current register value
	ORR R3, R3, R2 // set the enable bit
	STR R3, [R4] // store the new register value
	
/* Configure Interrupt Processor Targets Register (ICDIPTRn)
* reg_offset = integer_div(N / 4) * 4
* index = N mod 4 */
	BIC R4, R0, #3 // R4 = reg_offset
	LDR R2, =0xFFFED800
	ADD R4, R2, R4 // R4 = word address of ICDIPTR
	AND R2, R0, #0x3 // N mod 4
	ADD R4, R2, R4 // R4 = byte address in ICDIPTR
	
/* Using register address in R4 and the value in R2 write to
* (only) the appropriate byte */
	STRB R1, [R4]
	POP {R4-R5, PC}
	