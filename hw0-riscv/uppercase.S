        .section .data
input_string:   .asciz "Hello world!"

        # tohost/fromhost are data locations used by Spike
        .align 8
tohost: .byte 0
        .align 8
fromhost: .byte 0

        
        .section .text
        .globl _start
_start:
        # Load the address of the input string into a0
        la a0, input_string

        # Your code here!
        li a1, 97
        li a2, 122
loop:
        lb t0, 0(a0)
        addi a0, a0, 1
        beqz t0, end_program
        blt t0, a1, loop
        bgt t0, a2, loop
        addi t0, t0, -32
        sb t0, -1(a0)
        j loop

end_program:
        # jump here when you're done
        j end_program

