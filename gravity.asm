;Simple n-body simulation
%macro wait 0
  mov rax, 35
  mov rdi, UPDATE_DELAY
  mov rsi, 0
  syscall
%endmacro

%macro clearTerminal 0
  mov rax, SYS_write
  mov rdi, STDOUT
  mov rsi, clear_term
  mov rdx, clear_len
  syscall
%endmacro

section .data

;Simulation properties
SIZE_X  equ 250
SIZE_Y  equ 100

BODY_COUNT  equ 50

;Body properties are:
;0: Position X
;1: Position Y
;2: Velocity X
;3: Velocity Y 
;4: Mass
BODY_PROPERTY_COUNT equ 5

;Define max indices
CELL_COUNT  equ SIZE_X * SIZE_Y 
BODY_INDEX_COUNT    equ BODY_COUNT * BODY_PROPERTY_COUNT

SYS_write   equ 1
STDOUT  equ 1

clear_term: db 27, "[H", 27, "[2J" ; ESC[H ESC[2J (clear screen)
clear_len equ $-clear_term     ; Length of terminal control string

SYS_exit	equ	60
EXIT_SUCCESS	equ	0

;ASCII
LF  equ 10
EMPTY_CELL  equ "   "
FILL_CELL   equ "*"

TRUE    equ 1
FALSE   equ 0

;Define common values
MEM_SIZE_X   dd  SIZE_X
MEM_SIZE_Y   dd  SIZE_Y
MEM_BODY_PROPERTY_COUNT dd  BODY_PROPERTY_COUNT

FL_ZERO dd 0.0
FL_ONE  dd 1.0
FL_TWO  dd 2.0

;Temp variables
positionX1  dd  0.0
positionX2  dd  0.0
positionY1  dd  0.0
positionY2  dd  0.0

velocityX1  dd  0.0
velocityX2  dd  0.0
velocityY1  dd  0.0
velocityY2  dd  0.0

mass1   dd  0.0
mass2   dd  0.0

distance    dd  0.0
force   dd  0.0

newX    dd  0.0
newY    dd  0.0

;Physics modifiers
G_CONST dd  0.000005
UPDATE_DELAY dq 0, 40000000

POS_X_OFFSET    dd  25.0
POS_Y_OFFSET    dd  25.0

;Initialization modifiers
BODY_SPACE_MULT dd  5.0

section .bss

gridData    resb    SIZE_X * SIZE_Y
bodyData    resd    BODY_COUNT * BODY_PROPERTY_COUNT

section .text

; Prints the grid in the console
; Input:    N/A
; Returns:  N/A
global printGrid
printGrid:
    push rbx ;Push callee saved registers
    push r12
    push r13
    push r14
    push r15

    mov r12, 0

    printGridLoop:
        lea rbx, byte[gridData + r12]
        
        mov byte[gridData + r12 + SIZE_X - 1], LF

        mov rax, SYS_write
        mov rdi, STDOUT
        mov rsi, rbx
        mov rdx, SIZE_X
        syscall

        add r12, SIZE_X

        cmp r12, CELL_COUNT
        jl printGridLoop
    
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx

    ret

; Fills grid data with empty cells
; Input:    N/A
; Returns:  N/A
global clearGrid
clearGrid:
    push rbx ;Push callee saved registers
    push r12
    push r13
    push r14
    push r15

    mov r12, 0 ;row index
    mov r13, 0 ;column index

    initOuterLoop:
        mov r13, 0
        initInnerLoop:
            ;Calculate index
            mov r14, r12
            imul r14, SIZE_X ;Multiply row index by SIZE_X
            add r14, r13 ;r14 contains the index

            mov byte[gridData + r14], EMPTY_CELL
            
            inc r13
            cmp r13, SIZE_X
            jl initInnerLoop

        inc r12
        cmp r12, SIZE_Y
        jl initOuterLoop

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx

    ret

; Initializes bodies in a zigzag formation
; Input:    N/A
; Returns:  N/A
;
;Body properties are:
;0: Position X
;1: Position Y
;2: Velocity X
;3: Velocity Y 
;4: Mass
global initBodies
initBodies:
    push rbx ;Push callee saved registers
    push r12
    push r13
    push r14
    push r15

    cvtsi2ss xmm2, dword[MEM_SIZE_Y] ;size Y midpoint
    divss xmm2, dword[FL_TWO]

    cvtsi2ss xmm4, dword[MEM_BODY_PROPERTY_COUNT]

    movss xmm2, dword[FL_ONE] ;mass

    mov r12, 0 ;Index
    mov r13, FALSE ;Zigzag flag
    bodyInitLoop:
        cvtsi2ss xmm3, r12d
        divss xmm3, xmm4    
        mulss xmm3, dword[BODY_SPACE_MULT]

        movss xmm0, dword[FL_ZERO]
        movss xmm1, dword[FL_ONE]

        addss xmm3, dword[POS_X_OFFSET]
        movss dword[bodyData + 0 + (4 * r12)], xmm3 ;Based on X index
        
        ;Do zigzag for position Y
        cmp r13, FALSE
        je upperZig

        ;lower zig
        mov r13, FALSE
        movss xmm3, xmm2
        addss xmm3, dword[BODY_SPACE_MULT]
        addss xmm3, dword[POS_Y_OFFSET]
        movss dword[bodyData + (4 * 1) + (4 * r12)],  xmm3

        jmp finishZig
    upperZig:
        mov r13, TRUE
        movss xmm3, xmm2
        subss xmm3, dword[BODY_SPACE_MULT]
        addss xmm3, dword[POS_Y_OFFSET]
        movss dword[bodyData + (4 * 1) + (4 * r12)],  xmm3

    finishZig:

        movss dword[bodyData + (4 * 2) + (4 * r12)], xmm0
        movss dword[bodyData + (4 * 3) + (4 * r12)], xmm0

        addss xmm2, dword[FL_ONE]

        movss dword[bodyData + (4 * 4) + (4 * r12)], xmm2

        add r12, BODY_PROPERTY_COUNT
        cmp r12, BODY_INDEX_COUNT
        jle bodyInitLoop

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx

    ret

; Updates cells on the grid to display bodies
; Input:    N/A
; Returns:  N/A
global updateGrid
updateGrid:
    push rbx ;Push callee saved registers
    push r12
    push r13
    push r14
    push r15
    
    mov r12, 0 ;index
    updateGridLoop:
        cvtss2si r14, dword[bodyData + 0 + (4 * r12)] ;Position x
        cvtss2si r15, dword[bodyData + (4 * 1) + (4 * r12)] ;Position y

        cmp r14, 0
        jle skipSave

        cmp r15, 0
        jle skipSave

        cmp r14, SIZE_X
        jge skipSave

        cmp r15, SIZE_Y
        jge skipSave

        ;Calculate index
        mov rdi, r15
        imul rdi, SIZE_X ;Multiply row index by SIZE_X
        add rdi, r14 ;r14 contains the index

        mov bl, byte[bodyData + (4 * 4) + (4 * r12)] ;mass
        add bl, "0"

        mov byte[gridData + rdi], bl

    skipSave:

        add r12, BODY_PROPERTY_COUNT
        cmp r12, BODY_INDEX_COUNT
        jle updateGridLoop

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx

    ret

global physicsStep
physicsStep:
    push rbx ;Push callee saved registers
    push r12
    push r13
    push r14
    push r15

    mov r12, 0 ;outer index
    mov r13, 0 ;inner index
    physicsStepOuterLoop:
        mov r13, 0 ;reset inner index
        movss xmm0, dword[bodyData + (4 * 4) + (4 * r12)] ;mass1
        movss dword[mass1], xmm0
        physicsStepInnerLoop:
            cmp r12, r13
            je continuePhysicsStepInner

        movss xmm0, dword[bodyData + (4 * 4) + (4 * r13)] ;mass2
        movss dword[mass2], xmm0

        movss xmm0, dword[bodyData + (4 * 0) + (4 * r12)] ;positionX1
        movss dword[positionX1], xmm0

        movss xmm0, dword[bodyData + (4 * 0) + (4 * r13)] ;positionX2
        movss dword[positionX2], xmm0

        movss xmm0, dword[bodyData + (4 * 1) + (4 * r12)] ;positionY1
        movss dword[positionY1], xmm0

        movss xmm0, dword[bodyData + (4 * 1) + (4 * r13)] ;positionY2
        movss dword[positionY2], xmm0

        ;Get distance
        movss xmm0, dword[positionX1]
        movss xmm1, dword[positionX2]
        movss xmm2, dword[positionY1]
        movss xmm3, dword[positionY2]

        subss xmm1, xmm0 ;(x2 - x1)
        subss xmm3, xmm2 ;(y2 - y1)

        mulss xmm1, xmm1 ;(x2 - x1)^2
        mulss xmm3, xmm3 ;(y2 - y1)^2

        addss xmm1, xmm3 ;(x2 - x1)^2 + (y2 - y1)^2
        sqrtss xmm0, xmm1 ;sqrt((x2 - x1)^2 + (y2 - y1)^2) (distance)
        addss xmm0, dword[FL_ONE] ;So you don't divide by 0 ever
        movss dword[distance], xmm0

        movss xmm0, dword[mass1]
        movss xmm1, dword[mass2]

        mulss xmm0, xmm1 ;(mass1 * mass1)
        divss xmm0, dword[distance] ;(mass1 * mass1)/distance
        mulss xmm0, dword[G_CONST] ;G_CONST * (mass1 * mass1)/distance (force)
        movss dword[force], xmm0

        movss xmm0, dword[positionX1]
        movss xmm1, dword[positionX2]
        subss xmm1, xmm0 ;(x2 - x1)
        mulss xmm1, dword[force] ;(x2 - x1) * force (newX)
        movss dword[newX], xmm1

        movss xmm0, dword[positionY1]
        movss xmm1, dword[positionY2]
        subss xmm1, xmm0 ;(y2 - y1)
        mulss xmm1, dword[force] ;(y2 - y1) * force (newY)
        movss dword[newY], xmm1

        movss xmm0, dword[newX]
        movss xmm1, dword[newY]
        divss xmm0, dword[mass2] ;newX / mass2
        divss xmm1, dword[mass2] ;newX / mass2

        movss xmm2, dword[bodyData + (4 * 2) + (4 * r13)] ;velocityX
        movss xmm3, dword[bodyData + (4 * 3) + (4 * r13)] ;velocityY

        subss xmm2, xmm0 ;velocityX - newX / mass2
        subss xmm3, xmm1 ;velocityY - newY / mass2

        movss dword[bodyData + (4 * 2) + (4 * r13)], xmm2 ;update velocity
        movss dword[velocityX2], xmm2
        movss dword[bodyData + (4 * 3) + (4 * r13)], xmm3 
        movss dword[velocityY2], xmm3

        movss xmm0, dword[positionX2]
        addss xmm0, dword[velocityX2]
        movss dword[bodyData + (4 * 0) + (4 * r13)], xmm0 ;update positionX2

        movss xmm1, dword[positionY2]
        addss xmm1, dword[velocityY2]
        movss dword[bodyData + (4 * 1) + (4 * r13)], xmm1 ;update positionY2

        continuePhysicsStepInner:
            add r13, BODY_PROPERTY_COUNT
            cmp r13, BODY_INDEX_COUNT
            jle physicsStepInnerLoop

    add r12, BODY_PROPERTY_COUNT
    cmp r12, BODY_INDEX_COUNT
    jle physicsStepOuterLoop

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx

    ret

global _start
_start:
    call initBodies

    simulate:
        wait

        call printGrid
        call clearGrid
        call physicsStep
        call updateGrid
    
        jmp simulate
last:
	mov	rax, SYS_exit
	mov	rdi, EXIT_SUCCESS
	syscall