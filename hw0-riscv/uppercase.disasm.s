
uppercase.bin:     file format elf32-littleriscv


Disassembly of section .text:

00010074 <_start>:
   10074:	ffff2517          	auipc	a0,0xffff2
   10078:	f8c50513          	addi	a0,a0,-116 # 2000 <__DATA_BEGIN__>
   1007c:	06100593          	li	a1,97
   10080:	07a00613          	li	a2,122

00010084 <loop>:
   10084:	00050283          	lb	t0,0(a0)
   10088:	00150513          	addi	a0,a0,1
   1008c:	00028c63          	beqz	t0,100a4 <end_program>
   10090:	feb2cae3          	blt	t0,a1,10084 <loop>
   10094:	fe5648e3          	blt	a2,t0,10084 <loop>
   10098:	fe028293          	addi	t0,t0,-32
   1009c:	fe550fa3          	sb	t0,-1(a0)
   100a0:	fe5ff06f          	j	10084 <loop>

000100a4 <end_program>:
   100a4:	0000006f          	j	100a4 <end_program>
