#include <linux/linkage.h>
#include <asm/assembler.h>
/*
#define GPIO4_IRQSTATUS             0x49054018
#define GP57_SETREG                 0x49050094
#define GP57_CLRREG                 0x49050090
#define GP57_STATEREG               0x4905003C
#define GP57_BIT                    0x02000000

#define INTCPS_CONTROL              0x48200048
#define NEWFIQAGR                   0x2
*/
#define GPIO4_IRQSTATUS             0xfb054018
#define GP57_SETREG                 0xfb050094
#define GP57_CLRREG                 0xfb050090
#define GP57_STATEREG               0xfb05003C
#define GP57_BIT                    0x02000000

#define INTCPS_CONTROL              0xfa200048
#define NEWFIQAGR                   0x2

		.text

		.global	p347_fiq_end
		
ENTRY(p347_fiq_start)
@        stmfd sp!, {r0-r12, lr}
@        mrs r11, spsr

@clear bit in irq status, r8 - pointer, r9 - temporary value
	ldr r8,     =0xfb054018
	ldr r9,     [r8]
	orr r9,     #0x200
	str r9,     [r8]

@load gpio data out, r8 - pointer, r9 - GPIO_DATAOUT, r10 - bit value
	ldr r8,     =0xfb05003c
	ldr r10,    =0x02000000
	ldr r9,     [r8]
	tst r9,     r10
	bne bit_clear

bit_set:
        ldr r8,     =0xfb050094
        str r10,    [r8]
        b bit_end
bit_clear:
        ldr r8,     =0xfb050090
        str r10,    [r8]
bit_end:
        ldr r8,     =0xfa200048
        ldr r9,     =0x00000002
        str r9,     [r8]

@        msr spsr, r11
@        ldmfd sp!, {r0-r12, lr}
@restore
	subs	pc, lr, #4
p347_fiq_end:
