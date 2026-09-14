
080000b4 <__aeabi_uidiv>:
 80000b4:	2200      	movs    r2, #0
 80000b6:	0843      	lsrs    r3, r0, #1
 80000b8:	428b      	cmp     r3, r1
 80000ba:	d374      	blo     #0x80001a6
 80000bc:	0903      	lsrs    r3, r0, #4
 80000be:	428b      	cmp     r3, r1
 80000c0:	d35f      	blo     #0x8000182
 80000c2:	0a03      	lsrs    r3, r0, #8
 80000c4:	428b      	cmp     r3, r1
 80000c6:	d344      	blo     #0x8000152
 80000c8:	0b03      	lsrs    r3, r0, #0xc
 80000ca:	428b      	cmp     r3, r1
 80000cc:	d328      	blo     #0x8000120
 80000ce:	0c03      	lsrs    r3, r0, #0x10
 80000d0:	428b      	cmp     r3, r1
 80000d2:	d30d      	blo     #0x80000f0
 80000d4:	22ff      	movs    r2, #0xff
 80000d6:	0209      	lsls    r1, r1, #8
 80000d8:	ba12      	rev     r2, r2
 80000da:	0c03      	lsrs    r3, r0, #0x10
 80000dc:	428b      	cmp     r3, r1
 80000de:	d302      	blo     #0x80000e6
 80000e0:	1212      	asrs    r2, r2, #8
 80000e2:	0209      	lsls    r1, r1, #8
 80000e4:	d065      	beq     #0x80001b2
 80000e6:	0b03      	lsrs    r3, r0, #0xc
 80000e8:	428b      	cmp     r3, r1
 80000ea:	d319      	blo     #0x8000120
 80000ec:	e000      	b       #0x80000f0
 80000ee:	0a09      	lsrs    r1, r1, #8
 80000f0:	0bc3      	lsrs    r3, r0, #0xf
 80000f2:	428b      	cmp     r3, r1
 80000f4:	d301      	blo     #0x80000fa
 80000f6:	03cb      	lsls    r3, r1, #0xf
 80000f8:	1ac0      	subs    r0, r0, r3
 80000fa:	4152      	adcs    r2, r2
 80000fc:	0b83      	lsrs    r3, r0, #0xe
 80000fe:	428b      	cmp     r3, r1
 8000100:	d301      	blo     #0x8000106
 8000102:	038b      	lsls    r3, r1, #0xe
 8000104:	1ac0      	subs    r0, r0, r3
 8000106:	4152      	adcs    r2, r2
 8000108:	0b43      	lsrs    r3, r0, #0xd
 800010a:	428b      	cmp     r3, r1
 800010c:	d301      	blo     #0x8000112
 800010e:	034b      	lsls    r3, r1, #0xd
 8000110:	1ac0      	subs    r0, r0, r3
 8000112:	4152      	adcs    r2, r2
 8000114:	0b03      	lsrs    r3, r0, #0xc
 8000116:	428b      	cmp     r3, r1
 8000118:	d301      	blo     #0x800011e
 800011a:	030b      	lsls    r3, r1, #0xc
 800011c:	1ac0      	subs    r0, r0, r3
 800011e:	4152      	adcs    r2, r2
 8000120:	0ac3      	lsrs    r3, r0, #0xb
 8000122:	428b      	cmp     r3, r1
 8000124:	d301      	blo     #0x800012a
 8000126:	02cb      	lsls    r3, r1, #0xb
 8000128:	1ac0      	subs    r0, r0, r3
 800012a:	4152      	adcs    r2, r2
 800012c:	0a83      	lsrs    r3, r0, #0xa
 800012e:	428b      	cmp     r3, r1
 8000130:	d301      	blo     #0x8000136
 8000132:	028b      	lsls    r3, r1, #0xa
 8000134:	1ac0      	subs    r0, r0, r3
 8000136:	4152      	adcs    r2, r2
 8000138:	0a43      	lsrs    r3, r0, #9
 800013a:	428b      	cmp     r3, r1
 800013c:	d301      	blo     #0x8000142
 800013e:	024b      	lsls    r3, r1, #9
 8000140:	1ac0      	subs    r0, r0, r3
 8000142:	4152      	adcs    r2, r2
 8000144:	0a03      	lsrs    r3, r0, #8
 8000146:	428b      	cmp     r3, r1
 8000148:	d301      	blo     #0x800014e
 800014a:	020b      	lsls    r3, r1, #8
 800014c:	1ac0      	subs    r0, r0, r3
 800014e:	4152      	adcs    r2, r2
 8000150:	d2cd      	bhs     #0x80000ee
 8000152:	09c3      	lsrs    r3, r0, #7
 8000154:	428b      	cmp     r3, r1
 8000156:	d301      	blo     #0x800015c
 8000158:	01cb      	lsls    r3, r1, #7
 800015a:	1ac0      	subs    r0, r0, r3
 800015c:	4152      	adcs    r2, r2
 800015e:	0983      	lsrs    r3, r0, #6
 8000160:	428b      	cmp     r3, r1
 8000162:	d301      	blo     #0x8000168
 8000164:	018b      	lsls    r3, r1, #6
 8000166:	1ac0      	subs    r0, r0, r3
 8000168:	4152      	adcs    r2, r2
 800016a:	0943      	lsrs    r3, r0, #5
 800016c:	428b      	cmp     r3, r1
 800016e:	d301      	blo     #0x8000174
 8000170:	014b      	lsls    r3, r1, #5
 8000172:	1ac0      	subs    r0, r0, r3
 8000174:	4152      	adcs    r2, r2
 8000176:	0903      	lsrs    r3, r0, #4
 8000178:	428b      	cmp     r3, r1
 800017a:	d301      	blo     #0x8000180
 800017c:	010b      	lsls    r3, r1, #4
 800017e:	1ac0      	subs    r0, r0, r3
 8000180:	4152      	adcs    r2, r2
 8000182:	08c3      	lsrs    r3, r0, #3
 8000184:	428b      	cmp     r3, r1
 8000186:	d301      	blo     #0x800018c
 8000188:	00cb      	lsls    r3, r1, #3
 800018a:	1ac0      	subs    r0, r0, r3
 800018c:	4152      	adcs    r2, r2
 800018e:	0883      	lsrs    r3, r0, #2
 8000190:	428b      	cmp     r3, r1
 8000192:	d301      	blo     #0x8000198
 8000194:	008b      	lsls    r3, r1, #2
 8000196:	1ac0      	subs    r0, r0, r3
 8000198:	4152      	adcs    r2, r2
 800019a:	0843      	lsrs    r3, r0, #1
 800019c:	428b      	cmp     r3, r1
 800019e:	d301      	blo     #0x80001a4
 80001a0:	004b      	lsls    r3, r1, #1
 80001a2:	1ac0      	subs    r0, r0, r3
 80001a4:	4152      	adcs    r2, r2
 80001a6:	1a41      	subs    r1, r0, r1
 80001a8:	d200      	bhs     #0x80001ac
 80001aa:	4601      	mov     r1, r0
 80001ac:	4152      	adcs    r2, r2
 80001ae:	4610      	mov     r0, r2
 80001b0:	4770      	bx      lr
 80001b2:	e7ff      	b       #0x80001b4
 80001b4:	b501      	push    {r0, lr}
 80001b6:	2000      	movs    r0, #0
 80001b8:	f000 f806 	bl      #0x80001c8  <__aeabi_idiv0>
 80001bc:	bd02      	pop     {r1, pc}
 80001be:	46c0      	mov     r8, r8

080001c0 <__aeabi_uidivmod>:
 80001c0:	2900      	cmp     r1, #0
 80001c2:	d0f7      	beq     #0x80001b4
 80001c4:	e776      	b       #0x80000b4  <__aeabi_uidiv>
 80001c6:	4770      	bx      lr

080001c8 <__aeabi_idiv0>:
 80001c8:	4770      	bx      lr
 80001ca:	46c0      	mov     r8, r8

080001cc <__gnu_thumb1_case_shi>:
 80001cc:	b403      	push    {r0, r1}
 80001ce:	4671      	mov     r1, lr
 80001d0:	0849      	lsrs    r1, r1, #1
 80001d2:	0040      	lsls    r0, r0, #1
 80001d4:	0049      	lsls    r1, r1, #1
 80001d6:	5e09      	ldrsh   r1, [r1, r0]
 80001d8:	0049      	lsls    r1, r1, #1
 80001da:	448e      	add     lr, r1
 80001dc:	bc03      	pop     {r0, r1}
 80001de:	4770      	bx      lr

080001e0 <__do_global_dtors_aux>:
 80001e0:	b510      	push    {r4, lr}
 80001e2:	4c06      	ldr     r4, [pc, #0x18]
 80001e4:	7823      	ldrb    r3, [r4]
 80001e6:	2b00      	cmp     r3, #0
 80001e8:	d107      	bne     #0x80001fa
 80001ea:	4b05      	ldr     r3, [pc, #0x14]
 80001ec:	2b00      	cmp     r3, #0
 80001ee:	d002      	beq     #0x80001f6
 80001f0:	4804      	ldr     r0, [pc, #0x10]
 80001f2:	e000      	b       #0x80001f6
 80001f4:	bf00      	nop
 80001f6:	2301      	movs    r3, #1
 80001f8:	7023      	strb    r3, [r4]
 80001fa:	bd10      	pop     {r4, pc}
 80001fc:	20000060 	.word	0x20000060
 8000200:	00000000 	.word	0x00000000
 8000204:	08001354 	.word	0x08001354

08000208 <frame_dummy>:
 8000208:	4b04      	ldr     r3, [pc, #0x10]
 800020a:	b510      	push    {r4, lr}
 800020c:	2b00      	cmp     r3, #0
 800020e:	d003      	beq     #0x8000218
 8000210:	4903      	ldr     r1, [pc, #0xc]
 8000212:	4804      	ldr     r0, [pc, #0x10]
 8000214:	e000      	b       #0x8000218
 8000216:	bf00      	nop
 8000218:	bd10      	pop     {r4, pc}
 800021a:	46c0      	mov     r8, r8
 800021c:	00000000 	.word	0x00000000
 8000220:	20000064 	.word	0x20000064
 8000224:	08001354 	.word	0x08001354

08000228 <TIM1_BRK_UP_TRG_COM_IRQHandler>:
 8000228:	e7fe      	b       #0x8000228  <TIM1_BRK_UP_TRG_COM_IRQHandler>
 800022a:	0000      	movs    r0, r0

0800022c <SystemInit>:
 800022c:	2007      	movs    r0, #7
 800022e:	2301      	movs    r3, #1
 8000230:	4a0b      	ldr     r2, [pc, #0x2c]
 8000232:	6811      	ldr     r1, [r2]
 8000234:	4381      	bics    r1, r0
 8000236:	430b      	orrs    r3, r1
 8000238:	6013      	str     r3, [r2]
 800023a:	6813      	ldr     r3, [r2]
 800023c:	4003      	ands    r3, r0
 800023e:	2b01      	cmp     r3, #1
 8000240:	d1fb      	bne     #0x800023a
 8000242:	4b08      	ldr     r3, [pc, #0x20]
 8000244:	4908      	ldr     r1, [pc, #0x20]
 8000246:	681a      	ldr     r2, [r3]
 8000248:	400a      	ands    r2, r1
 800024a:	601a      	str     r2, [r3]
 800024c:	2280      	movs    r2, #0x80
 800024e:	00d2      	lsls    r2, r2, #3
 8000250:	6819      	ldr     r1, [r3]
 8000252:	4211      	tst     r1, r2
 8000254:	d0fc      	beq     #0x8000250
 8000256:	4b05      	ldr     r3, [pc, #0x14]
 8000258:	4a05      	ldr     r2, [pc, #0x14]
 800025a:	601a      	str     r2, [r3]
 800025c:	4770      	bx      lr
 800025e:	46c0      	mov     r8, r8
 8000260:	40022000 	.word	0x40022000
 8000264:	40021000 	.word	0x40021000
 8000268:	ffffc7ff 	.word	0xffffc7ff
 800026c:	20000000 	.word	0x20000000
 8000270:	02dc6c00 	.word	0x02dc6c00

08000274 <Reset_Handler>:
 8000274:	b510      	push    {r4, lr}
 8000276:	4a0b      	ldr     r2, [pc, #0x2c]
 8000278:	4b0b      	ldr     r3, [pc, #0x2c]
 800027a:	490c      	ldr     r1, [pc, #0x30]
 800027c:	428b      	cmp     r3, r1
 800027e:	d30b      	blo     #0x8000298
 8000280:	2100      	movs    r1, #0
 8000282:	4b0b      	ldr     r3, [pc, #0x2c]
 8000284:	4a0b      	ldr     r2, [pc, #0x2c]
 8000286:	4293      	cmp     r3, r2
 8000288:	d309      	blo     #0x800029e
 800028a:	f7ff ffcf 	bl      #0x800022c  <SystemInit>
 800028e:	f000 fb31 	bl      #0x80008f4  <__libc_init_array>
 8000292:	f000 f845 	bl      #0x8000320  <main>
 8000296:	e7fe      	b       #0x8000296
 8000298:	ca01      	ldm     r2!, {r0}
 800029a:	c301      	stm     r3!, {r0}
 800029c:	e7ee      	b       #0x800027c
 800029e:	c302      	stm     r3!, {r1}
 80002a0:	e7f1      	b       #0x8000286
 80002a2:	46c0      	mov     r8, r8
 80002a4:	0800142c 	.word	0x0800142c
 80002a8:	20000000 	.word	0x20000000
 80002ac:	20000060 	.word	0x20000060
 80002b0:	20000060 	.word	0x20000060
 80002b4:	200001cc 	.word	0x200001cc

080002b8 <_write>:
 80002b8:	b5f0      	push    {r4, r5, r6, r7, lr}
 80002ba:	2300      	movs    r3, #0
 80002bc:	2080      	movs    r0, #0x80
 80002be:	250d      	movs    r5, #0xd
 80002c0:	4c09      	ldr     r4, [pc, #0x24]
 80002c2:	4293      	cmp     r3, r2
 80002c4:	db01      	blt     #0x80002ca
 80002c6:	0010      	movs    r0, r2
 80002c8:	bdf0      	pop     {r4, r5, r6, r7, pc}
 80002ca:	5cce      	ldrb    r6, [r1, r3]
 80002cc:	2e0a      	cmp     r6, #0xa
 80002ce:	d103      	bne     #0x80002d8
 80002d0:	69e6      	ldr     r6, [r4, #0x1c]
 80002d2:	4206      	tst     r6, r0
 80002d4:	d0fc      	beq     #0x80002d0
 80002d6:	62a5      	str     r5, [r4, #0x28]
 80002d8:	5cce      	ldrb    r6, [r1, r3]
 80002da:	69e7      	ldr     r7, [r4, #0x1c]
 80002dc:	4207      	tst     r7, r0
 80002de:	d0fc      	beq     #0x80002da
 80002e0:	62a6      	str     r6, [r4, #0x28]
 80002e2:	3301      	adds    r3, #1
 80002e4:	e7ed      	b       #0x80002c2
 80002e6:	46c0      	mov     r8, r8
 80002e8:	40004400 	.word	0x40004400

080002ec <_read>:
 80002ec:	2000      	movs    r0, #0
 80002ee:	4770      	bx      lr

080002f0 <_close>:
 80002f0:	2001      	movs    r0, #1
 80002f2:	4240      	rsbs    r0, r0, #0
 80002f4:	4770      	bx      lr

080002f6 <_isatty>:
 80002f6:	2001      	movs    r0, #1
 80002f8:	4770      	bx      lr

080002fa <_lseek>:
 80002fa:	2000      	movs    r0, #0
 80002fc:	4770      	bx      lr

080002fe <_fstat>:
 80002fe:	2000      	movs    r0, #0
 8000300:	4770      	bx      lr
 8000302:	0000      	movs    r0, r0

08000304 <_sbrk>:
 8000304:	4a04      	ldr     r2, [pc, #0x10]
 8000306:	6813      	ldr     r3, [r2]
 8000308:	2b00      	cmp     r3, #0
 800030a:	d100      	bne     #0x800030e
 800030c:	4b03      	ldr     r3, [pc, #0xc]
 800030e:	1818      	adds    r0, r3, r0
 8000310:	6010      	str     r0, [r2]
 8000312:	0018      	movs    r0, r3
 8000314:	4770      	bx      lr
 8000316:	46c0      	mov     r8, r8
 8000318:	2000007c 	.word	0x2000007c
 800031c:	200001d0 	.word	0x200001d0

08000320 <main>:
 8000320:	2201      	movs    r2, #1
 8000322:	b570      	push    {r4, r5, r6, lr}
 8000324:	24a0      	movs    r4, #0xa0
 8000326:	4b2a      	ldr     r3, [pc, #0xa8]
 8000328:	05e4      	lsls    r4, r4, #0x17
 800032a:	6b59      	ldr     r1, [r3, #0x34]
 800032c:	4829      	ldr     r0, [pc, #0xa4]
 800032e:	4311      	orrs    r1, r2
 8000330:	6359      	str     r1, [r3, #0x34]
 8000332:	6821      	ldr     r1, [r4]
 8000334:	4d28      	ldr     r5, [pc, #0xa0]
 8000336:	4001      	ands    r1, r0
 8000338:	6021      	str     r1, [r4]
 800033a:	2180      	movs    r1, #0x80
 800033c:	6820      	ldr     r0, [r4]
 800033e:	00c9      	lsls    r1, r1, #3
 8000340:	4301      	orrs    r1, r0
 8000342:	6021      	str     r1, [r4]
 8000344:	6b59      	ldr     r1, [r3, #0x34]
 8000346:	6828      	ldr     r0, [r5]
 8000348:	430a      	orrs    r2, r1
 800034a:	635a      	str     r2, [r3, #0x34]
 800034c:	2280      	movs    r2, #0x80
 800034e:	6bd9      	ldr     r1, [r3, #0x3c]
 8000350:	0292      	lsls    r2, r2, #0xa
 8000352:	430a      	orrs    r2, r1
 8000354:	63da      	str     r2, [r3, #0x3c]
 8000356:	6a23      	ldr     r3, [r4, #0x20]
 8000358:	4a20      	ldr     r2, [pc, #0x80]
 800035a:	21e1      	movs    r1, #0xe1
 800035c:	4013      	ands    r3, r2
 800035e:	6223      	str     r3, [r4, #0x20]
 8000360:	2388      	movs    r3, #0x88
 8000362:	6a22      	ldr     r2, [r4, #0x20]
 8000364:	015b      	lsls    r3, r3, #5
 8000366:	4313      	orrs    r3, r2
 8000368:	22f0      	movs    r2, #0xf0
 800036a:	6223      	str     r3, [r4, #0x20]
 800036c:	6823      	ldr     r3, [r4]
 800036e:	0249      	lsls    r1, r1, #9
 8000370:	4393      	bics    r3, r2
 8000372:	6023      	str     r3, [r4]
 8000374:	23a0      	movs    r3, #0xa0
 8000376:	6822      	ldr     r2, [r4]
 8000378:	4313      	orrs    r3, r2
 800037a:	6023      	str     r3, [r4]
 800037c:	23e1      	movs    r3, #0xe1
 800037e:	021b      	lsls    r3, r3, #8
 8000380:	18c0      	adds    r0, r0, r3
 8000382:	f7ff fe97 	bl      #0x80000b4  <__aeabi_uidiv>
 8000386:	220d      	movs    r2, #0xd
 8000388:	4b15      	ldr     r3, [pc, #0x54]
 800038a:	60d8      	str     r0, [r3, #0xc]
 800038c:	601a      	str     r2, [r3]
 800038e:	4815      	ldr     r0, [pc, #0x54]
 8000390:	f000 f966 	bl      #0x8000660  <puts>
 8000394:	6829      	ldr     r1, [r5]
 8000396:	4814      	ldr     r0, [pc, #0x50]
 8000398:	f000 f8fc 	bl      #0x8000594  <iprintf>
 800039c:	4b13      	ldr     r3, [pc, #0x4c]
 800039e:	4914      	ldr     r1, [pc, #0x50]
 80003a0:	4814      	ldr     r0, [pc, #0x50]
 80003a2:	1ac9      	subs    r1, r1, r3
 80003a4:	f000 f8f6 	bl      #0x8000594  <iprintf>
 80003a8:	4b13      	ldr     r3, [pc, #0x4c]
 80003aa:	4914      	ldr     r1, [pc, #0x50]
 80003ac:	4814      	ldr     r0, [pc, #0x50]
 80003ae:	1ac9      	subs    r1, r1, r3
 80003b0:	f000 f8f0 	bl      #0x8000594  <iprintf>
 80003b4:	4813      	ldr     r0, [pc, #0x4c]
 80003b6:	f000 f953 	bl      #0x8000660  <puts>
 80003ba:	2220      	movs    r2, #0x20
 80003bc:	6963      	ldr     r3, [r4, #0x14]
 80003be:	4053      	eors    r3, r2
 80003c0:	6163      	str     r3, [r4, #0x14]
 80003c2:	4b11      	ldr     r3, [pc, #0x44]
 80003c4:	0019      	movs    r1, r3
 80003c6:	3b01      	subs    r3, #1
 80003c8:	2900      	cmp     r1, #0
 80003ca:	d0f7      	beq     #0x80003bc
 80003cc:	46c0      	mov     r8, r8
 80003ce:	e7f9      	b       #0x80003c4
 80003d0:	40021000 	.word	0x40021000
 80003d4:	fffff3ff 	.word	0xfffff3ff
 80003d8:	20000000 	.word	0x20000000
 80003dc:	ffff00ff 	.word	0xffff00ff
 80003e0:	40004400 	.word	0x40004400
 80003e4:	0800136c 	.word	0x0800136c
 80003e8:	08001394 	.word	0x08001394
 80003ec:	20000000 	.word	0x20000000
 80003f0:	20000060 	.word	0x20000060
 80003f4:	080013ae 	.word	0x080013ae
 80003f8:	20000060 	.word	0x20000060
 80003fc:	200001cc 	.word	0x200001cc
 8000400:	080013ca 	.word	0x080013ca
 8000404:	080013e6 	.word	0x080013e6
 8000408:	00061a80 	.word	0x00061a80

0800040c <std>:
 800040c:	2300      	movs    r3, #0
 800040e:	b510      	push    {r4, lr}
 8000410:	0004      	movs    r4, r0
 8000412:	6003      	str     r3, [r0]
 8000414:	6043      	str     r3, [r0, #4]
 8000416:	6083      	str     r3, [r0, #8]
 8000418:	8181      	strh    r1, [r0, #0xc]
 800041a:	6643      	str     r3, [r0, #0x64]
 800041c:	81c2      	strh    r2, [r0, #0xe]
 800041e:	6103      	str     r3, [r0, #0x10]
 8000420:	6143      	str     r3, [r0, #0x14]
 8000422:	6183      	str     r3, [r0, #0x18]
 8000424:	0019      	movs    r1, r3
 8000426:	2208      	movs    r2, #8
 8000428:	305c      	adds    r0, #0x5c
 800042a:	f000 fa0d 	bl      #0x8000848  <memset>
 800042e:	4b0b      	ldr     r3, [pc, #0x2c]
 8000430:	6224      	str     r4, [r4, #0x20]
 8000432:	6263      	str     r3, [r4, #0x24]
 8000434:	4b0a      	ldr     r3, [pc, #0x28]
 8000436:	62a3      	str     r3, [r4, #0x28]
 8000438:	4b0a      	ldr     r3, [pc, #0x28]
 800043a:	62e3      	str     r3, [r4, #0x2c]
 800043c:	4b0a      	ldr     r3, [pc, #0x28]
 800043e:	6323      	str     r3, [r4, #0x30]
 8000440:	4b0a      	ldr     r3, [pc, #0x28]
 8000442:	429c      	cmp     r4, r3
 8000444:	d005      	beq     #0x8000452
 8000446:	4b0a      	ldr     r3, [pc, #0x28]
 8000448:	429c      	cmp     r4, r3
 800044a:	d002      	beq     #0x8000452
 800044c:	4b09      	ldr     r3, [pc, #0x24]
 800044e:	429c      	cmp     r4, r3
 8000450:	d103      	bne     #0x800045a
 8000452:	0020      	movs    r0, r4
 8000454:	3058      	adds    r0, #0x58
 8000456:	f000 fa71 	bl      #0x800093c  <__retarget_lock_init_recursive>
 800045a:	bd10      	pop     {r4, pc}
 800045c:	08000675 	.word	0x08000675
 8000460:	0800069d 	.word	0x0800069d
 8000464:	080006d5 	.word	0x080006d5
 8000468:	08000701 	.word	0x08000701
 800046c:	20000080 	.word	0x20000080
 8000470:	200000e8 	.word	0x200000e8
 8000474:	20000150 	.word	0x20000150

08000478 <stdio_exit_handler>:
 8000478:	b510      	push    {r4, lr}
 800047a:	4a03      	ldr     r2, [pc, #0xc]
 800047c:	4903      	ldr     r1, [pc, #0xc]
 800047e:	4804      	ldr     r0, [pc, #0x10]
 8000480:	f000 f86c 	bl      #0x800055c  <_fwalk_sglue>
 8000484:	bd10      	pop     {r4, pc}
 8000486:	46c0      	mov     r8, r8
 8000488:	20000004 	.word	0x20000004
 800048c:	080011b1 	.word	0x080011b1
 8000490:	20000014 	.word	0x20000014

08000494 <cleanup_stdio>:
 8000494:	6841      	ldr     r1, [r0, #4]
 8000496:	4b0b      	ldr     r3, [pc, #0x2c]
 8000498:	b510      	push    {r4, lr}
 800049a:	0004      	movs    r4, r0
 800049c:	4299      	cmp     r1, r3
 800049e:	d001      	beq     #0x80004a4
 80004a0:	f000 fe86 	bl      #0x80011b0  <_fflush_r>
 80004a4:	68a1      	ldr     r1, [r4, #8]
 80004a6:	4b08      	ldr     r3, [pc, #0x20]
 80004a8:	4299      	cmp     r1, r3
 80004aa:	d002      	beq     #0x80004b2
 80004ac:	0020      	movs    r0, r4
 80004ae:	f000 fe7f 	bl      #0x80011b0  <_fflush_r>
 80004b2:	68e1      	ldr     r1, [r4, #0xc]
 80004b4:	4b05      	ldr     r3, [pc, #0x14]
 80004b6:	4299      	cmp     r1, r3
 80004b8:	d002      	beq     #0x80004c0
 80004ba:	0020      	movs    r0, r4
 80004bc:	f000 fe78 	bl      #0x80011b0  <_fflush_r>
 80004c0:	bd10      	pop     {r4, pc}
 80004c2:	46c0      	mov     r8, r8
 80004c4:	20000080 	.word	0x20000080
 80004c8:	200000e8 	.word	0x200000e8
 80004cc:	20000150 	.word	0x20000150

080004d0 <global_stdio_init.part.0>:
 80004d0:	b510      	push    {r4, lr}
 80004d2:	4b09      	ldr     r3, [pc, #0x24]
 80004d4:	4a09      	ldr     r2, [pc, #0x24]
 80004d6:	2104      	movs    r1, #4
 80004d8:	601a      	str     r2, [r3]
 80004da:	4809      	ldr     r0, [pc, #0x24]
 80004dc:	2200      	movs    r2, #0
 80004de:	f7ff ff95 	bl      #0x800040c  <std>
 80004e2:	2201      	movs    r2, #1
 80004e4:	2109      	movs    r1, #9
 80004e6:	4807      	ldr     r0, [pc, #0x1c]
 80004e8:	f7ff ff90 	bl      #0x800040c  <std>
 80004ec:	2202      	movs    r2, #2
 80004ee:	2112      	movs    r1, #0x12
 80004f0:	4805      	ldr     r0, [pc, #0x14]
 80004f2:	f7ff ff8b 	bl      #0x800040c  <std>
 80004f6:	bd10      	pop     {r4, pc}
 80004f8:	200001b8 	.word	0x200001b8
 80004fc:	08000479 	.word	0x08000479
 8000500:	20000080 	.word	0x20000080
 8000504:	200000e8 	.word	0x200000e8
 8000508:	20000150 	.word	0x20000150

0800050c <__sfp_lock_acquire>:
 800050c:	b510      	push    {r4, lr}
 800050e:	4802      	ldr     r0, [pc, #8]
 8000510:	f000 fa15 	bl      #0x800093e  <__retarget_lock_acquire_recursive>
 8000514:	bd10      	pop     {r4, pc}
 8000516:	46c0      	mov     r8, r8
 8000518:	200001c1 	.word	0x200001c1

0800051c <__sfp_lock_release>:
 800051c:	b510      	push    {r4, lr}
 800051e:	4802      	ldr     r0, [pc, #8]
 8000520:	f000 fa0e 	bl      #0x8000940  <__retarget_lock_release_recursive>
 8000524:	bd10      	pop     {r4, pc}
 8000526:	46c0      	mov     r8, r8
 8000528:	200001c1 	.word	0x200001c1

0800052c <__sinit>:
 800052c:	b510      	push    {r4, lr}
 800052e:	0004      	movs    r4, r0
 8000530:	f7ff ffec 	bl      #0x800050c  <__sfp_lock_acquire>
 8000534:	6a23      	ldr     r3, [r4, #0x20]
 8000536:	2b00      	cmp     r3, #0
 8000538:	d002      	beq     #0x8000540
 800053a:	f7ff ffef 	bl      #0x800051c  <__sfp_lock_release>
 800053e:	bd10      	pop     {r4, pc}
 8000540:	4b04      	ldr     r3, [pc, #0x10]
 8000542:	6223      	str     r3, [r4, #0x20]
 8000544:	4b04      	ldr     r3, [pc, #0x10]
 8000546:	681b      	ldr     r3, [r3]
 8000548:	2b00      	cmp     r3, #0
 800054a:	d1f6      	bne     #0x800053a
 800054c:	f7ff ffc0 	bl      #0x80004d0  <global_stdio_init.part.0>
 8000550:	e7f3      	b       #0x800053a
 8000552:	46c0      	mov     r8, r8
 8000554:	08000495 	.word	0x08000495
 8000558:	200001b8 	.word	0x200001b8

0800055c <_fwalk_sglue>:
 800055c:	b5f7      	push    {r0, r1, r2, r4, r5, r6, r7, lr}
 800055e:	0014      	movs    r4, r2
 8000560:	2600      	movs    r6, #0
 8000562:	9000      	str     r0, [sp]
 8000564:	9101      	str     r1, [sp, #4]
 8000566:	68a5      	ldr     r5, [r4, #8]
 8000568:	6867      	ldr     r7, [r4, #4]
 800056a:	3f01      	subs    r7, #1
 800056c:	d504      	bpl     #0x8000578
 800056e:	6824      	ldr     r4, [r4]
 8000570:	2c00      	cmp     r4, #0
 8000572:	d1f8      	bne     #0x8000566
 8000574:	0030      	movs    r0, r6
 8000576:	bdfe      	pop     {r1, r2, r3, r4, r5, r6, r7, pc}
 8000578:	89ab      	ldrh    r3, [r5, #0xc]
 800057a:	2b01      	cmp     r3, #1
 800057c:	d908      	bls     #0x8000590
 800057e:	220e      	movs    r2, #0xe
 8000580:	5eab      	ldrsh   r3, [r5, r2]
 8000582:	3301      	adds    r3, #1
 8000584:	d004      	beq     #0x8000590
 8000586:	0029      	movs    r1, r5
 8000588:	9800      	ldr     r0, [sp]
 800058a:	9b01      	ldr     r3, [sp, #4]
 800058c:	4798      	blx     r3
 800058e:	4306      	orrs    r6, r0
 8000590:	3568      	adds    r5, #0x68
 8000592:	e7ea      	b       #0x800056a

08000594 <iprintf>:
 8000594:	b40f      	push    {r0, r1, r2, r3}
 8000596:	b507      	push    {r0, r1, r2, lr}
 8000598:	4905      	ldr     r1, [pc, #0x14]
 800059a:	ab04      	add     r3, sp, #0x10
 800059c:	6808      	ldr     r0, [r1]
 800059e:	cb04      	ldm     r3!, {r2}
 80005a0:	6881      	ldr     r1, [r0, #8]
 80005a2:	9301      	str     r3, [sp, #4]
 80005a4:	f000 faf2 	bl      #0x8000b8c  <_vfprintf_r>
 80005a8:	b003      	add     sp, #0xc
 80005aa:	bc08      	pop     {r3}
 80005ac:	b004      	add     sp, #0x10
 80005ae:	4718      	bx      r3
 80005b0:	20000010 	.word	0x20000010

080005b4 <_puts_r>:
 80005b4:	6a03      	ldr     r3, [r0, #0x20]
 80005b6:	b570      	push    {r4, r5, r6, lr}
 80005b8:	0005      	movs    r5, r0
 80005ba:	000e      	movs    r6, r1
 80005bc:	6884      	ldr     r4, [r0, #8]
 80005be:	2b00      	cmp     r3, #0
 80005c0:	d101      	bne     #0x80005c6
 80005c2:	f7ff ffb3 	bl      #0x800052c  <__sinit>
 80005c6:	6e63      	ldr     r3, [r4, #0x64]
 80005c8:	07db      	lsls    r3, r3, #0x1f
 80005ca:	d405      	bmi     #0x80005d8
 80005cc:	89a3      	ldrh    r3, [r4, #0xc]
 80005ce:	059b      	lsls    r3, r3, #0x16
 80005d0:	d402      	bmi     #0x80005d8
 80005d2:	6da0      	ldr     r0, [r4, #0x58]
 80005d4:	f000 f9b3 	bl      #0x800093e  <__retarget_lock_acquire_recursive>
 80005d8:	89a3      	ldrh    r3, [r4, #0xc]
 80005da:	071b      	lsls    r3, r3, #0x1c
 80005dc:	d502      	bpl     #0x80005e4
 80005de:	6923      	ldr     r3, [r4, #0x10]
 80005e0:	2b00      	cmp     r3, #0
 80005e2:	d11e      	bne     #0x8000622
 80005e4:	0021      	movs    r1, r4
 80005e6:	0028      	movs    r0, r5
 80005e8:	f000 f8d2 	bl      #0x8000790  <__swsetup_r>
 80005ec:	2800      	cmp     r0, #0
 80005ee:	d018      	beq     #0x8000622
 80005f0:	2501      	movs    r5, #1
 80005f2:	426d      	rsbs    r5, r5, #0
 80005f4:	6e63      	ldr     r3, [r4, #0x64]
 80005f6:	07db      	lsls    r3, r3, #0x1f
 80005f8:	d405      	bmi     #0x8000606
 80005fa:	89a3      	ldrh    r3, [r4, #0xc]
 80005fc:	059b      	lsls    r3, r3, #0x16
 80005fe:	d402      	bmi     #0x8000606
 8000600:	6da0      	ldr     r0, [r4, #0x58]
 8000602:	f000 f99d 	bl      #0x8000940  <__retarget_lock_release_recursive>
 8000606:	0028      	movs    r0, r5
 8000608:	bd70      	pop     {r4, r5, r6, pc}
 800060a:	2b00      	cmp     r3, #0
 800060c:	da04      	bge     #0x8000618
 800060e:	69a2      	ldr     r2, [r4, #0x18]
 8000610:	4293      	cmp     r3, r2
 8000612:	db17      	blt     #0x8000644
 8000614:	290a      	cmp     r1, #0xa
 8000616:	d015      	beq     #0x8000644
 8000618:	6823      	ldr     r3, [r4]
 800061a:	1c5a      	adds    r2, r3, #1
 800061c:	6022      	str     r2, [r4]
 800061e:	7019      	strb    r1, [r3]
 8000620:	3601      	adds    r6, #1
 8000622:	68a3      	ldr     r3, [r4, #8]
 8000624:	7831      	ldrb    r1, [r6]
 8000626:	3b01      	subs    r3, #1
 8000628:	60a3      	str     r3, [r4, #8]
 800062a:	2900      	cmp     r1, #0
 800062c:	d1ed      	bne     #0x800060a
 800062e:	428b      	cmp     r3, r1
 8000630:	da0f      	bge     #0x8000652
 8000632:	0022      	movs    r2, r4
 8000634:	0028      	movs    r0, r5
 8000636:	310a      	adds    r1, #0xa
 8000638:	f000 f868 	bl      #0x800070c  <__swbuf_r>
 800063c:	3001      	adds    r0, #1
 800063e:	d0d7      	beq     #0x80005f0
 8000640:	250a      	movs    r5, #0xa
 8000642:	e7d7      	b       #0x80005f4
 8000644:	0022      	movs    r2, r4
 8000646:	0028      	movs    r0, r5
 8000648:	f000 f860 	bl      #0x800070c  <__swbuf_r>
 800064c:	3001      	adds    r0, #1
 800064e:	d1e7      	bne     #0x8000620
 8000650:	e7ce      	b       #0x80005f0
 8000652:	6823      	ldr     r3, [r4]
 8000654:	1c5a      	adds    r2, r3, #1
 8000656:	6022      	str     r2, [r4]
 8000658:	220a      	movs    r2, #0xa
 800065a:	701a      	strb    r2, [r3]
 800065c:	e7f0      	b       #0x8000640
 800065e:	0000      	movs    r0, r0

08000660 <puts>:
 8000660:	b510      	push    {r4, lr}
 8000662:	4b03      	ldr     r3, [pc, #0xc]
 8000664:	0001      	movs    r1, r0
 8000666:	6818      	ldr     r0, [r3]
 8000668:	f7ff ffa4 	bl      #0x80005b4  <_puts_r>
 800066c:	bd10      	pop     {r4, pc}
 800066e:	46c0      	mov     r8, r8
 8000670:	20000010 	.word	0x20000010

08000674 <__sread>:
 8000674:	b570      	push    {r4, r5, r6, lr}
 8000676:	000c      	movs    r4, r1
 8000678:	250e      	movs    r5, #0xe
 800067a:	5f49      	ldrsh   r1, [r1, r5]
 800067c:	f000 f912 	bl      #0x80008a4  <_read_r>
 8000680:	2800      	cmp     r0, #0
 8000682:	db03      	blt     #0x800068c
 8000684:	6d63      	ldr     r3, [r4, #0x54]
 8000686:	181b      	adds    r3, r3, r0
 8000688:	6563      	str     r3, [r4, #0x54]
 800068a:	bd70      	pop     {r4, r5, r6, pc}
 800068c:	89a3      	ldrh    r3, [r4, #0xc]
 800068e:	4a02      	ldr     r2, [pc, #8]
 8000690:	4013      	ands    r3, r2
 8000692:	81a3      	strh    r3, [r4, #0xc]
 8000694:	e7f9      	b       #0x800068a
 8000696:	46c0      	mov     r8, r8
 8000698:	ffffefff 	.word	0xffffefff

0800069c <__swrite>:
 800069c:	b5f8      	push    {r3, r4, r5, r6, r7, lr}
 800069e:	001f      	movs    r7, r3
 80006a0:	898b      	ldrh    r3, [r1, #0xc]
 80006a2:	0005      	movs    r5, r0
 80006a4:	000c      	movs    r4, r1
 80006a6:	0016      	movs    r6, r2
 80006a8:	05db      	lsls    r3, r3, #0x17
 80006aa:	d505      	bpl     #0x80006b8
 80006ac:	230e      	movs    r3, #0xe
 80006ae:	5ec9      	ldrsh   r1, [r1, r3]
 80006b0:	2200      	movs    r2, #0
 80006b2:	2302      	movs    r3, #2
 80006b4:	f000 f8e2 	bl      #0x800087c  <_lseek_r>
 80006b8:	89a3      	ldrh    r3, [r4, #0xc]
 80006ba:	4a05      	ldr     r2, [pc, #0x14]
 80006bc:	0028      	movs    r0, r5
 80006be:	4013      	ands    r3, r2
 80006c0:	81a3      	strh    r3, [r4, #0xc]
 80006c2:	0032      	movs    r2, r6
 80006c4:	230e      	movs    r3, #0xe
 80006c6:	5ee1      	ldrsh   r1, [r4, r3]
 80006c8:	003b      	movs    r3, r7
 80006ca:	f000 f8ff 	bl      #0x80008cc  <_write_r>
 80006ce:	bdf8      	pop     {r3, r4, r5, r6, r7, pc}
 80006d0:	ffffefff 	.word	0xffffefff

080006d4 <__sseek>:
 80006d4:	b570      	push    {r4, r5, r6, lr}
 80006d6:	000c      	movs    r4, r1
 80006d8:	250e      	movs    r5, #0xe
 80006da:	5f49      	ldrsh   r1, [r1, r5]
 80006dc:	f000 f8ce 	bl      #0x800087c  <_lseek_r>
 80006e0:	220c      	movs    r2, #0xc
 80006e2:	5ea3      	ldrsh   r3, [r4, r2]
 80006e4:	1c42      	adds    r2, r0, #1
 80006e6:	d103      	bne     #0x80006f0
 80006e8:	4a04      	ldr     r2, [pc, #0x10]
 80006ea:	4013      	ands    r3, r2
 80006ec:	81a3      	strh    r3, [r4, #0xc]
 80006ee:	bd70      	pop     {r4, r5, r6, pc}
 80006f0:	2280      	movs    r2, #0x80
 80006f2:	0152      	lsls    r2, r2, #5
 80006f4:	4313      	orrs    r3, r2
 80006f6:	81a3      	strh    r3, [r4, #0xc]
 80006f8:	6560      	str     r0, [r4, #0x54]
 80006fa:	e7f8      	b       #0x80006ee
 80006fc:	ffffefff 	.word	0xffffefff

08000700 <__sclose>:
 8000700:	b510      	push    {r4, lr}
 8000702:	230e      	movs    r3, #0xe
 8000704:	5ec9      	ldrsh   r1, [r1, r3]
 8000706:	f000 f8a7 	bl      #0x8000858  <_close_r>
 800070a:	bd10      	pop     {r4, pc}

0800070c <__swbuf_r>:
 800070c:	b5f8      	push    {r3, r4, r5, r6, r7, lr}
 800070e:	0005      	movs    r5, r0
 8000710:	000f      	movs    r7, r1
 8000712:	0014      	movs    r4, r2
 8000714:	2800      	cmp     r0, #0
 8000716:	d004      	beq     #0x8000722
 8000718:	6a03      	ldr     r3, [r0, #0x20]
 800071a:	2b00      	cmp     r3, #0
 800071c:	d101      	bne     #0x8000722
 800071e:	f7ff ff05 	bl      #0x800052c  <__sinit>
 8000722:	69a3      	ldr     r3, [r4, #0x18]
 8000724:	60a3      	str     r3, [r4, #8]
 8000726:	89a3      	ldrh    r3, [r4, #0xc]
 8000728:	071b      	lsls    r3, r3, #0x1c
 800072a:	d502      	bpl     #0x8000732
 800072c:	6923      	ldr     r3, [r4, #0x10]
 800072e:	2b00      	cmp     r3, #0
 8000730:	d109      	bne     #0x8000746
 8000732:	0021      	movs    r1, r4
 8000734:	0028      	movs    r0, r5
 8000736:	f000 f82b 	bl      #0x8000790  <__swsetup_r>
 800073a:	2800      	cmp     r0, #0
 800073c:	d003      	beq     #0x8000746
 800073e:	2601      	movs    r6, #1
 8000740:	4276      	rsbs    r6, r6, #0
 8000742:	0030      	movs    r0, r6
 8000744:	bdf8      	pop     {r3, r4, r5, r6, r7, pc}
 8000746:	6923      	ldr     r3, [r4, #0x10]
 8000748:	6820      	ldr     r0, [r4]
 800074a:	1ac0      	subs    r0, r0, r3
 800074c:	6963      	ldr     r3, [r4, #0x14]
 800074e:	4283      	cmp     r3, r0
 8000750:	dc05      	bgt     #0x800075e
 8000752:	0021      	movs    r1, r4
 8000754:	0028      	movs    r0, r5
 8000756:	f000 fd2b 	bl      #0x80011b0  <_fflush_r>
 800075a:	2800      	cmp     r0, #0
 800075c:	d1ef      	bne     #0x800073e
 800075e:	68a3      	ldr     r3, [r4, #8]
 8000760:	3001      	adds    r0, #1
 8000762:	3b01      	subs    r3, #1
 8000764:	60a3      	str     r3, [r4, #8]
 8000766:	6823      	ldr     r3, [r4]
 8000768:	b2fe      	uxtb    r6, r7
 800076a:	1c5a      	adds    r2, r3, #1
 800076c:	6022      	str     r2, [r4]
 800076e:	701f      	strb    r7, [r3]
 8000770:	6963      	ldr     r3, [r4, #0x14]
 8000772:	4283      	cmp     r3, r0
 8000774:	d004      	beq     #0x8000780
 8000776:	89a3      	ldrh    r3, [r4, #0xc]
 8000778:	07db      	lsls    r3, r3, #0x1f
 800077a:	d5e2      	bpl     #0x8000742
 800077c:	2e0a      	cmp     r6, #0xa
 800077e:	d1e0      	bne     #0x8000742
 8000780:	0021      	movs    r1, r4
 8000782:	0028      	movs    r0, r5
 8000784:	f000 fd14 	bl      #0x80011b0  <_fflush_r>
 8000788:	2800      	cmp     r0, #0
 800078a:	d0da      	beq     #0x8000742
 800078c:	e7d7      	b       #0x800073e
 800078e:	0000      	movs    r0, r0

08000790 <__swsetup_r>:
 8000790:	4b2c      	ldr     r3, [pc, #0xb0]
 8000792:	b570      	push    {r4, r5, r6, lr}
 8000794:	0005      	movs    r5, r0
 8000796:	6818      	ldr     r0, [r3]
 8000798:	000c      	movs    r4, r1
 800079a:	2800      	cmp     r0, #0
 800079c:	d004      	beq     #0x80007a8
 800079e:	6a03      	ldr     r3, [r0, #0x20]
 80007a0:	2b00      	cmp     r3, #0
 80007a2:	d101      	bne     #0x80007a8
 80007a4:	f7ff fec2 	bl      #0x800052c  <__sinit>
 80007a8:	220c      	movs    r2, #0xc
 80007aa:	5ea3      	ldrsh   r3, [r4, r2]
 80007ac:	071a      	lsls    r2, r3, #0x1c
 80007ae:	d421      	bmi     #0x80007f4
 80007b0:	06da      	lsls    r2, r3, #0x1b
 80007b2:	d407      	bmi     #0x80007c4
 80007b4:	2209      	movs    r2, #9
 80007b6:	602a      	str     r2, [r5]
 80007b8:	2240      	movs    r2, #0x40
 80007ba:	2001      	movs    r0, #1
 80007bc:	4313      	orrs    r3, r2
 80007be:	81a3      	strh    r3, [r4, #0xc]
 80007c0:	4240      	rsbs    r0, r0, #0
 80007c2:	e038      	b       #0x8000836
 80007c4:	075a      	lsls    r2, r3, #0x1d
 80007c6:	d512      	bpl     #0x80007ee
 80007c8:	6b61      	ldr     r1, [r4, #0x34]
 80007ca:	2900      	cmp     r1, #0
 80007cc:	d008      	beq     #0x80007e0
 80007ce:	0023      	movs    r3, r4
 80007d0:	3344      	adds    r3, #0x44
 80007d2:	4299      	cmp     r1, r3
 80007d4:	d002      	beq     #0x80007dc
 80007d6:	0028      	movs    r0, r5
 80007d8:	f000 f8b4 	bl      #0x8000944  <_free_r>
 80007dc:	2300      	movs    r3, #0
 80007de:	6363      	str     r3, [r4, #0x34]
 80007e0:	2224      	movs    r2, #0x24
 80007e2:	89a3      	ldrh    r3, [r4, #0xc]
 80007e4:	4393      	bics    r3, r2
 80007e6:	2200      	movs    r2, #0
 80007e8:	6062      	str     r2, [r4, #4]
 80007ea:	6922      	ldr     r2, [r4, #0x10]
 80007ec:	6022      	str     r2, [r4]
 80007ee:	2208      	movs    r2, #8
 80007f0:	4313      	orrs    r3, r2
 80007f2:	81a3      	strh    r3, [r4, #0xc]
 80007f4:	6923      	ldr     r3, [r4, #0x10]
 80007f6:	2b00      	cmp     r3, #0
 80007f8:	d10b      	bne     #0x8000812
 80007fa:	23a0      	movs    r3, #0xa0
 80007fc:	2280      	movs    r2, #0x80
 80007fe:	89a1      	ldrh    r1, [r4, #0xc]
 8000800:	009b      	lsls    r3, r3, #2
 8000802:	0092      	lsls    r2, r2, #2
 8000804:	400b      	ands    r3, r1
 8000806:	4293      	cmp     r3, r2
 8000808:	d003      	beq     #0x8000812
 800080a:	0021      	movs    r1, r4
 800080c:	0028      	movs    r0, r5
 800080e:	f000 fd25 	bl      #0x800125c  <__smakebuf_r>
 8000812:	220c      	movs    r2, #0xc
 8000814:	5ea3      	ldrsh   r3, [r4, r2]
 8000816:	2101      	movs    r1, #1
 8000818:	001a      	movs    r2, r3
 800081a:	400a      	ands    r2, r1
 800081c:	420b      	tst     r3, r1
 800081e:	d00b      	beq     #0x8000838
 8000820:	2200      	movs    r2, #0
 8000822:	60a2      	str     r2, [r4, #8]
 8000824:	6962      	ldr     r2, [r4, #0x14]
 8000826:	4252      	rsbs    r2, r2, #0
 8000828:	61a2      	str     r2, [r4, #0x18]
 800082a:	2000      	movs    r0, #0
 800082c:	6922      	ldr     r2, [r4, #0x10]
 800082e:	4282      	cmp     r2, r0
 8000830:	d101      	bne     #0x8000836
 8000832:	061a      	lsls    r2, r3, #0x18
 8000834:	d4c0      	bmi     #0x80007b8
 8000836:	bd70      	pop     {r4, r5, r6, pc}
 8000838:	0799      	lsls    r1, r3, #0x1e
 800083a:	d400      	bmi     #0x800083e
 800083c:	6962      	ldr     r2, [r4, #0x14]
 800083e:	60a2      	str     r2, [r4, #8]
 8000840:	e7f3      	b       #0x800082a
 8000842:	46c0      	mov     r8, r8
 8000844:	20000010 	.word	0x20000010

08000848 <memset>:
 8000848:	0003      	movs    r3, r0
 800084a:	1882      	adds    r2, r0, r2
 800084c:	4293      	cmp     r3, r2
 800084e:	d100      	bne     #0x8000852
 8000850:	4770      	bx      lr
 8000852:	7019      	strb    r1, [r3]
 8000854:	3301      	adds    r3, #1
 8000856:	e7f9      	b       #0x800084c

08000858 <_close_r>:
 8000858:	2300      	movs    r3, #0
 800085a:	b570      	push    {r4, r5, r6, lr}
 800085c:	4c06      	ldr     r4, [pc, #0x18]
 800085e:	0005      	movs    r5, r0
 8000860:	0008      	movs    r0, r1
 8000862:	6023      	str     r3, [r4]
 8000864:	f7ff fd44 	bl      #0x80002f0  <_close>
 8000868:	1c43      	adds    r3, r0, #1
 800086a:	d103      	bne     #0x8000874
 800086c:	6823      	ldr     r3, [r4]
 800086e:	2b00      	cmp     r3, #0
 8000870:	d000      	beq     #0x8000874
 8000872:	602b      	str     r3, [r5]
 8000874:	bd70      	pop     {r4, r5, r6, pc}
 8000876:	46c0      	mov     r8, r8
 8000878:	200001bc 	.word	0x200001bc

0800087c <_lseek_r>:
 800087c:	b570      	push    {r4, r5, r6, lr}
 800087e:	0005      	movs    r5, r0
 8000880:	0008      	movs    r0, r1
 8000882:	0011      	movs    r1, r2
 8000884:	2200      	movs    r2, #0
 8000886:	4c06      	ldr     r4, [pc, #0x18]
 8000888:	6022      	str     r2, [r4]
 800088a:	001a      	movs    r2, r3
 800088c:	f7ff fd35 	bl      #0x80002fa  <_lseek>
 8000890:	1c43      	adds    r3, r0, #1
 8000892:	d103      	bne     #0x800089c
 8000894:	6823      	ldr     r3, [r4]
 8000896:	2b00      	cmp     r3, #0
 8000898:	d000      	beq     #0x800089c
 800089a:	602b      	str     r3, [r5]
 800089c:	bd70      	pop     {r4, r5, r6, pc}
 800089e:	46c0      	mov     r8, r8
 80008a0:	200001bc 	.word	0x200001bc

080008a4 <_read_r>:
 80008a4:	b570      	push    {r4, r5, r6, lr}
 80008a6:	0005      	movs    r5, r0
 80008a8:	0008      	movs    r0, r1
 80008aa:	0011      	movs    r1, r2
 80008ac:	2200      	movs    r2, #0
 80008ae:	4c06      	ldr     r4, [pc, #0x18]
 80008b0:	6022      	str     r2, [r4]
 80008b2:	001a      	movs    r2, r3
 80008b4:	f7ff fd1a 	bl      #0x80002ec  <_read>
 80008b8:	1c43      	adds    r3, r0, #1
 80008ba:	d103      	bne     #0x80008c4
 80008bc:	6823      	ldr     r3, [r4]
 80008be:	2b00      	cmp     r3, #0
 80008c0:	d000      	beq     #0x80008c4
 80008c2:	602b      	str     r3, [r5]
 80008c4:	bd70      	pop     {r4, r5, r6, pc}
 80008c6:	46c0      	mov     r8, r8
 80008c8:	200001bc 	.word	0x200001bc

080008cc <_write_r>:
 80008cc:	b570      	push    {r4, r5, r6, lr}
 80008ce:	0005      	movs    r5, r0
 80008d0:	0008      	movs    r0, r1
 80008d2:	0011      	movs    r1, r2
 80008d4:	2200      	movs    r2, #0
 80008d6:	4c06      	ldr     r4, [pc, #0x18]
 80008d8:	6022      	str     r2, [r4]
 80008da:	001a      	movs    r2, r3
 80008dc:	f7ff fcec 	bl      #0x80002b8  <_write>
 80008e0:	1c43      	adds    r3, r0, #1
 80008e2:	d103      	bne     #0x80008ec
 80008e4:	6823      	ldr     r3, [r4]
 80008e6:	2b00      	cmp     r3, #0
 80008e8:	d000      	beq     #0x80008ec
 80008ea:	602b      	str     r3, [r5]
 80008ec:	bd70      	pop     {r4, r5, r6, pc}
 80008ee:	46c0      	mov     r8, r8
 80008f0:	200001bc 	.word	0x200001bc

080008f4 <__libc_init_array>:
 80008f4:	b570      	push    {r4, r5, r6, lr}
 80008f6:	2600      	movs    r6, #0
 80008f8:	4c0c      	ldr     r4, [pc, #0x30]
 80008fa:	4d0d      	ldr     r5, [pc, #0x34]
 80008fc:	1b64      	subs    r4, r4, r5
 80008fe:	10a4      	asrs    r4, r4, #2
 8000900:	42a6      	cmp     r6, r4
 8000902:	d109      	bne     #0x8000918
 8000904:	2600      	movs    r6, #0
 8000906:	f000 fd25 	bl      #0x8001354  <_init>
 800090a:	4c0a      	ldr     r4, [pc, #0x28]
 800090c:	4d0a      	ldr     r5, [pc, #0x28]
 800090e:	1b64      	subs    r4, r4, r5
 8000910:	10a4      	asrs    r4, r4, #2
 8000912:	42a6      	cmp     r6, r4
 8000914:	d105      	bne     #0x8000922
 8000916:	bd70      	pop     {r4, r5, r6, pc}
 8000918:	00b3      	lsls    r3, r6, #2
 800091a:	58eb      	ldr     r3, [r5, r3]
 800091c:	4798      	blx     r3
 800091e:	3601      	adds    r6, #1
 8000920:	e7ee      	b       #0x8000900
 8000922:	00b3      	lsls    r3, r6, #2
 8000924:	58eb      	ldr     r3, [r5, r3]
 8000926:	4798      	blx     r3
 8000928:	3601      	adds    r6, #1
 800092a:	e7f2      	b       #0x8000912
 800092c:	08001424 	.word	0x08001424
 8000930:	08001424 	.word	0x08001424
 8000934:	08001428 	.word	0x08001428
 8000938:	08001424 	.word	0x08001424

0800093c <__retarget_lock_init_recursive>:
 800093c:	4770      	bx      lr

0800093e <__retarget_lock_acquire_recursive>:
 800093e:	4770      	bx      lr

08000940 <__retarget_lock_release_recursive>:
 8000940:	4770      	bx      lr
 8000942:	0000      	movs    r0, r0

08000944 <_free_r>:
 8000944:	b570      	push    {r4, r5, r6, lr}
 8000946:	0005      	movs    r5, r0
 8000948:	2900      	cmp     r1, #0
 800094a:	d010      	beq     #0x800096e
 800094c:	1f0c      	subs    r4, r1, #4
 800094e:	6823      	ldr     r3, [r4]
 8000950:	2b00      	cmp     r3, #0
 8000952:	da00      	bge     #0x8000956
 8000954:	18e4      	adds    r4, r4, r3
 8000956:	0028      	movs    r0, r5
 8000958:	f000 f8e0 	bl      #0x8000b1c  <__malloc_lock>
 800095c:	4a1d      	ldr     r2, [pc, #0x74]
 800095e:	6813      	ldr     r3, [r2]
 8000960:	2b00      	cmp     r3, #0
 8000962:	d105      	bne     #0x8000970
 8000964:	6063      	str     r3, [r4, #4]
 8000966:	6014      	str     r4, [r2]
 8000968:	0028      	movs    r0, r5
 800096a:	f000 f8df 	bl      #0x8000b2c  <__malloc_unlock>
 800096e:	bd70      	pop     {r4, r5, r6, pc}
 8000970:	42a3      	cmp     r3, r4
 8000972:	d908      	bls     #0x8000986
 8000974:	6820      	ldr     r0, [r4]
 8000976:	1821      	adds    r1, r4, r0
 8000978:	428b      	cmp     r3, r1
 800097a:	d1f3      	bne     #0x8000964
 800097c:	6819      	ldr     r1, [r3]
 800097e:	685b      	ldr     r3, [r3, #4]
 8000980:	1809      	adds    r1, r1, r0
 8000982:	6021      	str     r1, [r4]
 8000984:	e7ee      	b       #0x8000964
 8000986:	001a      	movs    r2, r3
 8000988:	685b      	ldr     r3, [r3, #4]
 800098a:	2b00      	cmp     r3, #0
 800098c:	d001      	beq     #0x8000992
 800098e:	42a3      	cmp     r3, r4
 8000990:	d9f9      	bls     #0x8000986
 8000992:	6811      	ldr     r1, [r2]
 8000994:	1850      	adds    r0, r2, r1
 8000996:	42a0      	cmp     r0, r4
 8000998:	d10b      	bne     #0x80009b2
 800099a:	6820      	ldr     r0, [r4]
 800099c:	1809      	adds    r1, r1, r0
 800099e:	1850      	adds    r0, r2, r1
 80009a0:	6011      	str     r1, [r2]
 80009a2:	4283      	cmp     r3, r0
 80009a4:	d1e0      	bne     #0x8000968
 80009a6:	6818      	ldr     r0, [r3]
 80009a8:	685b      	ldr     r3, [r3, #4]
 80009aa:	1841      	adds    r1, r0, r1
 80009ac:	6011      	str     r1, [r2]
 80009ae:	6053      	str     r3, [r2, #4]
 80009b0:	e7da      	b       #0x8000968
 80009b2:	42a0      	cmp     r0, r4
 80009b4:	d902      	bls     #0x80009bc
 80009b6:	230c      	movs    r3, #0xc
 80009b8:	602b      	str     r3, [r5]
 80009ba:	e7d5      	b       #0x8000968
 80009bc:	6820      	ldr     r0, [r4]
 80009be:	1821      	adds    r1, r4, r0
 80009c0:	428b      	cmp     r3, r1
 80009c2:	d103      	bne     #0x80009cc
 80009c4:	6819      	ldr     r1, [r3]
 80009c6:	685b      	ldr     r3, [r3, #4]
 80009c8:	1809      	adds    r1, r1, r0
 80009ca:	6021      	str     r1, [r4]
 80009cc:	6063      	str     r3, [r4, #4]
 80009ce:	6054      	str     r4, [r2, #4]
 80009d0:	e7ca      	b       #0x8000968
 80009d2:	46c0      	mov     r8, r8
 80009d4:	200001c8 	.word	0x200001c8

080009d8 <sbrk_aligned>:
 80009d8:	b570      	push    {r4, r5, r6, lr}
 80009da:	4c0f      	ldr     r4, [pc, #0x3c]
 80009dc:	000e      	movs    r6, r1
 80009de:	6821      	ldr     r1, [r4]
 80009e0:	0005      	movs    r5, r0
 80009e2:	2900      	cmp     r1, #0
 80009e4:	d102      	bne     #0x80009ec
 80009e6:	f000 fc97 	bl      #0x8001318  <_sbrk_r>
 80009ea:	6020      	str     r0, [r4]
 80009ec:	0031      	movs    r1, r6
 80009ee:	0028      	movs    r0, r5
 80009f0:	f000 fc92 	bl      #0x8001318  <_sbrk_r>
 80009f4:	1c43      	adds    r3, r0, #1
 80009f6:	d103      	bne     #0x8000a00
 80009f8:	2401      	movs    r4, #1
 80009fa:	4264      	rsbs    r4, r4, #0
 80009fc:	0020      	movs    r0, r4
 80009fe:	bd70      	pop     {r4, r5, r6, pc}
 8000a00:	2303      	movs    r3, #3
 8000a02:	1cc4      	adds    r4, r0, #3
 8000a04:	439c      	bics    r4, r3
 8000a06:	42a0      	cmp     r0, r4
 8000a08:	d0f8      	beq     #0x80009fc
 8000a0a:	1a21      	subs    r1, r4, r0
 8000a0c:	0028      	movs    r0, r5
 8000a0e:	f000 fc83 	bl      #0x8001318  <_sbrk_r>
 8000a12:	3001      	adds    r0, #1
 8000a14:	d1f2      	bne     #0x80009fc
 8000a16:	e7ef      	b       #0x80009f8
 8000a18:	200001c4 	.word	0x200001c4

08000a1c <_malloc_r>:
 8000a1c:	b5f7      	push    {r0, r1, r2, r4, r5, r6, r7, lr}
 8000a1e:	2203      	movs    r2, #3
 8000a20:	1ccb      	adds    r3, r1, #3
 8000a22:	4393      	bics    r3, r2
 8000a24:	3308      	adds    r3, #8
 8000a26:	0005      	movs    r5, r0
 8000a28:	001f      	movs    r7, r3
 8000a2a:	2b0c      	cmp     r3, #0xc
 8000a2c:	d234      	bhs     #0x8000a98
 8000a2e:	270c      	movs    r7, #0xc
 8000a30:	42b9      	cmp     r1, r7
 8000a32:	d833      	bhi     #0x8000a9c
 8000a34:	0028      	movs    r0, r5
 8000a36:	f000 f871 	bl      #0x8000b1c  <__malloc_lock>
 8000a3a:	4e37      	ldr     r6, [pc, #0xdc]
 8000a3c:	6833      	ldr     r3, [r6]
 8000a3e:	001c      	movs    r4, r3
 8000a40:	2c00      	cmp     r4, #0
 8000a42:	d12f      	bne     #0x8000aa4
 8000a44:	0039      	movs    r1, r7
 8000a46:	0028      	movs    r0, r5
 8000a48:	f7ff ffc6 	bl      #0x80009d8  <sbrk_aligned>
 8000a4c:	0004      	movs    r4, r0
 8000a4e:	1c43      	adds    r3, r0, #1
 8000a50:	d15f      	bne     #0x8000b12
 8000a52:	6834      	ldr     r4, [r6]
 8000a54:	9400      	str     r4, [sp]
 8000a56:	9b00      	ldr     r3, [sp]
 8000a58:	2b00      	cmp     r3, #0
 8000a5a:	d14a      	bne     #0x8000af2
 8000a5c:	429c      	cmp     r4, r3
 8000a5e:	d052      	beq     #0x8000b06
 8000a60:	6823      	ldr     r3, [r4]
 8000a62:	0028      	movs    r0, r5
 8000a64:	18e3      	adds    r3, r4, r3
 8000a66:	9900      	ldr     r1, [sp]
 8000a68:	9301      	str     r3, [sp, #4]
 8000a6a:	f000 fc55 	bl      #0x8001318  <_sbrk_r>
 8000a6e:	9b01      	ldr     r3, [sp, #4]
 8000a70:	4283      	cmp     r3, r0
 8000a72:	d148      	bne     #0x8000b06
 8000a74:	6823      	ldr     r3, [r4]
 8000a76:	0028      	movs    r0, r5
 8000a78:	1aff      	subs    r7, r7, r3
 8000a7a:	0039      	movs    r1, r7
 8000a7c:	f7ff ffac 	bl      #0x80009d8  <sbrk_aligned>
 8000a80:	3001      	adds    r0, #1
 8000a82:	d040      	beq     #0x8000b06
 8000a84:	6823      	ldr     r3, [r4]
 8000a86:	19db      	adds    r3, r3, r7
 8000a88:	6023      	str     r3, [r4]
 8000a8a:	6833      	ldr     r3, [r6]
 8000a8c:	685a      	ldr     r2, [r3, #4]
 8000a8e:	2a00      	cmp     r2, #0
 8000a90:	d133      	bne     #0x8000afa
 8000a92:	9b00      	ldr     r3, [sp]
 8000a94:	6033      	str     r3, [r6]
 8000a96:	e019      	b       #0x8000acc
 8000a98:	2b00      	cmp     r3, #0
 8000a9a:	dac9      	bge     #0x8000a30
 8000a9c:	230c      	movs    r3, #0xc
 8000a9e:	602b      	str     r3, [r5]
 8000aa0:	2000      	movs    r0, #0
 8000aa2:	bdfe      	pop     {r1, r2, r3, r4, r5, r6, r7, pc}
 8000aa4:	6821      	ldr     r1, [r4]
 8000aa6:	1bc9      	subs    r1, r1, r7
 8000aa8:	d420      	bmi     #0x8000aec
 8000aaa:	290b      	cmp     r1, #0xb
 8000aac:	d90a      	bls     #0x8000ac4
 8000aae:	19e2      	adds    r2, r4, r7
 8000ab0:	6027      	str     r7, [r4]
 8000ab2:	42a3      	cmp     r3, r4
 8000ab4:	d104      	bne     #0x8000ac0
 8000ab6:	6032      	str     r2, [r6]
 8000ab8:	6863      	ldr     r3, [r4, #4]
 8000aba:	6011      	str     r1, [r2]
 8000abc:	6053      	str     r3, [r2, #4]
 8000abe:	e005      	b       #0x8000acc
 8000ac0:	605a      	str     r2, [r3, #4]
 8000ac2:	e7f9      	b       #0x8000ab8
 8000ac4:	6862      	ldr     r2, [r4, #4]
 8000ac6:	42a3      	cmp     r3, r4
 8000ac8:	d10e      	bne     #0x8000ae8
 8000aca:	6032      	str     r2, [r6]
 8000acc:	0028      	movs    r0, r5
 8000ace:	f000 f82d 	bl      #0x8000b2c  <__malloc_unlock>
 8000ad2:	0020      	movs    r0, r4
 8000ad4:	2207      	movs    r2, #7
 8000ad6:	300b      	adds    r0, #0xb
 8000ad8:	1d23      	adds    r3, r4, #4
 8000ada:	4390      	bics    r0, r2
 8000adc:	1ac2      	subs    r2, r0, r3
 8000ade:	4298      	cmp     r0, r3
 8000ae0:	d0df      	beq     #0x8000aa2
 8000ae2:	1a1b      	subs    r3, r3, r0
 8000ae4:	50a3      	str     r3, [r4, r2]
 8000ae6:	e7dc      	b       #0x8000aa2
 8000ae8:	605a      	str     r2, [r3, #4]
 8000aea:	e7ef      	b       #0x8000acc
 8000aec:	0023      	movs    r3, r4
 8000aee:	6864      	ldr     r4, [r4, #4]
 8000af0:	e7a6      	b       #0x8000a40
 8000af2:	9c00      	ldr     r4, [sp]
 8000af4:	6863      	ldr     r3, [r4, #4]
 8000af6:	9300      	str     r3, [sp]
 8000af8:	e7ad      	b       #0x8000a56
 8000afa:	001a      	movs    r2, r3
 8000afc:	685b      	ldr     r3, [r3, #4]
 8000afe:	42a3      	cmp     r3, r4
 8000b00:	d1fb      	bne     #0x8000afa
 8000b02:	2300      	movs    r3, #0
 8000b04:	e7da      	b       #0x8000abc
 8000b06:	230c      	movs    r3, #0xc
 8000b08:	0028      	movs    r0, r5
 8000b0a:	602b      	str     r3, [r5]
 8000b0c:	f000 f80e 	bl      #0x8000b2c  <__malloc_unlock>
 8000b10:	e7c6      	b       #0x8000aa0
 8000b12:	6007      	str     r7, [r0]
 8000b14:	e7da      	b       #0x8000acc
 8000b16:	46c0      	mov     r8, r8
 8000b18:	200001c8 	.word	0x200001c8

08000b1c <__malloc_lock>:
 8000b1c:	b510      	push    {r4, lr}
 8000b1e:	4802      	ldr     r0, [pc, #8]
 8000b20:	f7ff ff0d 	bl      #0x800093e  <__retarget_lock_acquire_recursive>
 8000b24:	bd10      	pop     {r4, pc}
 8000b26:	46c0      	mov     r8, r8
 8000b28:	200001c0 	.word	0x200001c0

08000b2c <__malloc_unlock>:
 8000b2c:	b510      	push    {r4, lr}
 8000b2e:	4802      	ldr     r0, [pc, #8]
 8000b30:	f7ff ff06 	bl      #0x8000940  <__retarget_lock_release_recursive>
 8000b34:	bd10      	pop     {r4, pc}
 8000b36:	46c0      	mov     r8, r8
 8000b38:	200001c0 	.word	0x200001c0

08000b3c <__sfputc_r>:
 8000b3c:	6893      	ldr     r3, [r2, #8]
 8000b3e:	b510      	push    {r4, lr}
 8000b40:	3b01      	subs    r3, #1
 8000b42:	6093      	str     r3, [r2, #8]
 8000b44:	2b00      	cmp     r3, #0
 8000b46:	da04      	bge     #0x8000b52
 8000b48:	6994      	ldr     r4, [r2, #0x18]
 8000b4a:	42a3      	cmp     r3, r4
 8000b4c:	db07      	blt     #0x8000b5e
 8000b4e:	290a      	cmp     r1, #0xa
 8000b50:	d005      	beq     #0x8000b5e
 8000b52:	6813      	ldr     r3, [r2]
 8000b54:	1c58      	adds    r0, r3, #1
 8000b56:	6010      	str     r0, [r2]
 8000b58:	7019      	strb    r1, [r3]
 8000b5a:	0008      	movs    r0, r1
 8000b5c:	bd10      	pop     {r4, pc}
 8000b5e:	f7ff fdd5 	bl      #0x800070c  <__swbuf_r>
 8000b62:	0001      	movs    r1, r0
 8000b64:	e7f9      	b       #0x8000b5a

08000b66 <__sfputs_r>:
 8000b66:	b5f8      	push    {r3, r4, r5, r6, r7, lr}
 8000b68:	0006      	movs    r6, r0
 8000b6a:	000f      	movs    r7, r1
 8000b6c:	0014      	movs    r4, r2
 8000b6e:	18d5      	adds    r5, r2, r3
 8000b70:	42ac      	cmp     r4, r5
 8000b72:	d101      	bne     #0x8000b78
 8000b74:	2000      	movs    r0, #0
 8000b76:	e007      	b       #0x8000b88
 8000b78:	7821      	ldrb    r1, [r4]
 8000b7a:	003a      	movs    r2, r7
 8000b7c:	0030      	movs    r0, r6
 8000b7e:	f7ff ffdd 	bl      #0x8000b3c  <__sfputc_r>
 8000b82:	3401      	adds    r4, #1
 8000b84:	1c43      	adds    r3, r0, #1
 8000b86:	d1f3      	bne     #0x8000b70
 8000b88:	bdf8      	pop     {r3, r4, r5, r6, r7, pc}
 8000b8a:	0000      	movs    r0, r0

08000b8c <_vfprintf_r>:
 8000b8c:	b5f0      	push    {r4, r5, r6, r7, lr}
 8000b8e:	b09f      	sub     sp, #0x7c
 8000b90:	000e      	movs    r6, r1
 8000b92:	0017      	movs    r7, r2
 8000b94:	001c      	movs    r4, r3
 8000b96:	9002      	str     r0, [sp, #8]
 8000b98:	2800      	cmp     r0, #0
 8000b9a:	d004      	beq     #0x8000ba6
 8000b9c:	6a03      	ldr     r3, [r0, #0x20]
 8000b9e:	2b00      	cmp     r3, #0
 8000ba0:	d101      	bne     #0x8000ba6
 8000ba2:	f7ff fcc3 	bl      #0x800052c  <__sinit>
 8000ba6:	6e73      	ldr     r3, [r6, #0x64]
 8000ba8:	07db      	lsls    r3, r3, #0x1f
 8000baa:	d405      	bmi     #0x8000bb8
 8000bac:	89b3      	ldrh    r3, [r6, #0xc]
 8000bae:	059b      	lsls    r3, r3, #0x16
 8000bb0:	d402      	bmi     #0x8000bb8
 8000bb2:	6db0      	ldr     r0, [r6, #0x58]
 8000bb4:	f7ff fec3 	bl      #0x800093e  <__retarget_lock_acquire_recursive>
 8000bb8:	89b3      	ldrh    r3, [r6, #0xc]
 8000bba:	071b      	lsls    r3, r3, #0x1c
 8000bbc:	d502      	bpl     #0x8000bc4
 8000bbe:	6933      	ldr     r3, [r6, #0x10]
 8000bc0:	2b00      	cmp     r3, #0
 8000bc2:	d113      	bne     #0x8000bec
 8000bc4:	0031      	movs    r1, r6
 8000bc6:	9802      	ldr     r0, [sp, #8]
 8000bc8:	f7ff fde2 	bl      #0x8000790  <__swsetup_r>
 8000bcc:	2800      	cmp     r0, #0
 8000bce:	d00d      	beq     #0x8000bec
 8000bd0:	6e73      	ldr     r3, [r6, #0x64]
 8000bd2:	07db      	lsls    r3, r3, #0x1f
 8000bd4:	d503      	bpl     #0x8000bde
 8000bd6:	2001      	movs    r0, #1
 8000bd8:	4240      	rsbs    r0, r0, #0
 8000bda:	b01f      	add     sp, #0x7c
 8000bdc:	bdf0      	pop     {r4, r5, r6, r7, pc}
 8000bde:	89b3      	ldrh    r3, [r6, #0xc]
 8000be0:	059b      	lsls    r3, r3, #0x16
 8000be2:	d4f8      	bmi     #0x8000bd6
 8000be4:	6db0      	ldr     r0, [r6, #0x58]
 8000be6:	f7ff feab 	bl      #0x8000940  <__retarget_lock_release_recursive>
 8000bea:	e7f4      	b       #0x8000bd6
 8000bec:	2300      	movs    r3, #0
 8000bee:	ad06      	add     r5, sp, #0x18
 8000bf0:	616b      	str     r3, [r5, #0x14]
 8000bf2:	3320      	adds    r3, #0x20
 8000bf4:	766b      	strb    r3, [r5, #0x19]
 8000bf6:	3310      	adds    r3, #0x10
 8000bf8:	76ab      	strb    r3, [r5, #0x1a]
 8000bfa:	9405      	str     r4, [sp, #0x14]
 8000bfc:	003c      	movs    r4, r7
 8000bfe:	7823      	ldrb    r3, [r4]
 8000c00:	2b00      	cmp     r3, #0
 8000c02:	d001      	beq     #0x8000c08
 8000c04:	2b25      	cmp     r3, #0x25
 8000c06:	d143      	bne     #0x8000c90
 8000c08:	1be3      	subs    r3, r4, r7
 8000c0a:	9303      	str     r3, [sp, #0xc]
 8000c0c:	42bc      	cmp     r4, r7
 8000c0e:	d00b      	beq     #0x8000c28
 8000c10:	003a      	movs    r2, r7
 8000c12:	0031      	movs    r1, r6
 8000c14:	9802      	ldr     r0, [sp, #8]
 8000c16:	f7ff ffa6 	bl      #0x8000b66  <__sfputs_r>
 8000c1a:	3001      	adds    r0, #1
 8000c1c:	d100      	bne     #0x8000c20
 8000c1e:	e0a4      	b       #0x8000d6a
 8000c20:	696b      	ldr     r3, [r5, #0x14]
 8000c22:	9a03      	ldr     r2, [sp, #0xc]
 8000c24:	189b      	adds    r3, r3, r2
 8000c26:	616b      	str     r3, [r5, #0x14]
 8000c28:	7823      	ldrb    r3, [r4]
 8000c2a:	2b00      	cmp     r3, #0
 8000c2c:	d100      	bne     #0x8000c30
 8000c2e:	e09c      	b       #0x8000d6a
 8000c30:	2201      	movs    r2, #1
 8000c32:	2300      	movs    r3, #0
 8000c34:	4252      	rsbs    r2, r2, #0
 8000c36:	606a      	str     r2, [r5, #4]
 8000c38:	aa02      	add     r2, sp, #8
 8000c3a:	3253      	adds    r2, #0x53
 8000c3c:	3401      	adds    r4, #1
 8000c3e:	602b      	str     r3, [r5]
 8000c40:	60eb      	str     r3, [r5, #0xc]
 8000c42:	60ab      	str     r3, [r5, #8]
 8000c44:	7013      	strb    r3, [r2]
 8000c46:	65ab      	str     r3, [r5, #0x58]
 8000c48:	4f54      	ldr     r7, [pc, #0x150]
 8000c4a:	2205      	movs    r2, #5
 8000c4c:	7821      	ldrb    r1, [r4]
 8000c4e:	0038      	movs    r0, r7
 8000c50:	f000 fb74 	bl      #0x800133c  <memchr>
 8000c54:	682a      	ldr     r2, [r5]
 8000c56:	1c61      	adds    r1, r4, #1
 8000c58:	2800      	cmp     r0, #0
 8000c5a:	d11b      	bne     #0x8000c94
 8000c5c:	06d3      	lsls    r3, r2, #0x1b
 8000c5e:	d503      	bpl     #0x8000c68
 8000c60:	2320      	movs    r3, #0x20
 8000c62:	2753      	movs    r7, #0x53
 8000c64:	a802      	add     r0, sp, #8
 8000c66:	55c3      	strb    r3, [r0, r7]
 8000c68:	0713      	lsls    r3, r2, #0x1c
 8000c6a:	d503      	bpl     #0x8000c74
 8000c6c:	232b      	movs    r3, #0x2b
 8000c6e:	2753      	movs    r7, #0x53
 8000c70:	a802      	add     r0, sp, #8
 8000c72:	55c3      	strb    r3, [r0, r7]
 8000c74:	7823      	ldrb    r3, [r4]
 8000c76:	2b2a      	cmp     r3, #0x2a
 8000c78:	d013      	beq     #0x8000ca2
 8000c7a:	2000      	movs    r0, #0
 8000c7c:	210a      	movs    r1, #0xa
 8000c7e:	68eb      	ldr     r3, [r5, #0xc]
 8000c80:	7822      	ldrb    r2, [r4]
 8000c82:	3a30      	subs    r2, #0x30
 8000c84:	2a09      	cmp     r2, #9
 8000c86:	d94f      	bls     #0x8000d28
 8000c88:	2800      	cmp     r0, #0
 8000c8a:	d012      	beq     #0x8000cb2
 8000c8c:	60eb      	str     r3, [r5, #0xc]
 8000c8e:	e010      	b       #0x8000cb2
 8000c90:	3401      	adds    r4, #1
 8000c92:	e7b4      	b       #0x8000bfe
 8000c94:	2301      	movs    r3, #1
 8000c96:	1bc0      	subs    r0, r0, r7
 8000c98:	4083      	lsls    r3, r0
 8000c9a:	4313      	orrs    r3, r2
 8000c9c:	000c      	movs    r4, r1
 8000c9e:	602b      	str     r3, [r5]
 8000ca0:	e7d2      	b       #0x8000c48
 8000ca2:	9b05      	ldr     r3, [sp, #0x14]
 8000ca4:	1d18      	adds    r0, r3, #4
 8000ca6:	681b      	ldr     r3, [r3]
 8000ca8:	9005      	str     r0, [sp, #0x14]
 8000caa:	2b00      	cmp     r3, #0
 8000cac:	db36      	blt     #0x8000d1c
 8000cae:	60eb      	str     r3, [r5, #0xc]
 8000cb0:	000c      	movs    r4, r1
 8000cb2:	7823      	ldrb    r3, [r4]
 8000cb4:	2b2e      	cmp     r3, #0x2e
 8000cb6:	d10c      	bne     #0x8000cd2
 8000cb8:	7863      	ldrb    r3, [r4, #1]
 8000cba:	2b2a      	cmp     r3, #0x2a
 8000cbc:	d139      	bne     #0x8000d32
 8000cbe:	9b05      	ldr     r3, [sp, #0x14]
 8000cc0:	3402      	adds    r4, #2
 8000cc2:	1d1a      	adds    r2, r3, #4
 8000cc4:	681b      	ldr     r3, [r3]
 8000cc6:	9205      	str     r2, [sp, #0x14]
 8000cc8:	2b00      	cmp     r3, #0
 8000cca:	da01      	bge     #0x8000cd0
 8000ccc:	2301      	movs    r3, #1
 8000cce:	425b      	rsbs    r3, r3, #0
 8000cd0:	606b      	str     r3, [r5, #4]
 8000cd2:	4f33      	ldr     r7, [pc, #0xcc]
 8000cd4:	2203      	movs    r2, #3
 8000cd6:	0038      	movs    r0, r7
 8000cd8:	7821      	ldrb    r1, [r4]
 8000cda:	f000 fb2f 	bl      #0x800133c  <memchr>
 8000cde:	2800      	cmp     r0, #0
 8000ce0:	d006      	beq     #0x8000cf0
 8000ce2:	2340      	movs    r3, #0x40
 8000ce4:	1bc0      	subs    r0, r0, r7
 8000ce6:	4083      	lsls    r3, r0
 8000ce8:	682a      	ldr     r2, [r5]
 8000cea:	3401      	adds    r4, #1
 8000cec:	4313      	orrs    r3, r2
 8000cee:	602b      	str     r3, [r5]
 8000cf0:	7821      	ldrb    r1, [r4]
 8000cf2:	2206      	movs    r2, #6
 8000cf4:	482b      	ldr     r0, [pc, #0xac]
 8000cf6:	7629      	strb    r1, [r5, #0x18]
 8000cf8:	f000 fb20 	bl      #0x800133c  <memchr>
 8000cfc:	2800      	cmp     r0, #0
 8000cfe:	d043      	beq     #0x8000d88
 8000d00:	4829      	ldr     r0, [pc, #0xa4]
 8000d02:	2800      	cmp     r0, #0
 8000d04:	d127      	bne     #0x8000d56
 8000d06:	2207      	movs    r2, #7
 8000d08:	9b05      	ldr     r3, [sp, #0x14]
 8000d0a:	3307      	adds    r3, #7
 8000d0c:	4393      	bics    r3, r2
 8000d0e:	3308      	adds    r3, #8
 8000d10:	9305      	str     r3, [sp, #0x14]
 8000d12:	696b      	ldr     r3, [r5, #0x14]
 8000d14:	1c67      	adds    r7, r4, #1
 8000d16:	181b      	adds    r3, r3, r0
 8000d18:	616b      	str     r3, [r5, #0x14]
 8000d1a:	e76f      	b       #0x8000bfc
 8000d1c:	425b      	rsbs    r3, r3, #0
 8000d1e:	60eb      	str     r3, [r5, #0xc]
 8000d20:	2302      	movs    r3, #2
 8000d22:	4313      	orrs    r3, r2
 8000d24:	602b      	str     r3, [r5]
 8000d26:	e7c3      	b       #0x8000cb0
 8000d28:	434b      	muls    r3, r1, r3
 8000d2a:	2001      	movs    r0, #1
 8000d2c:	3401      	adds    r4, #1
 8000d2e:	189b      	adds    r3, r3, r2
 8000d30:	e7a6      	b       #0x8000c80
 8000d32:	2300      	movs    r3, #0
 8000d34:	200a      	movs    r0, #0xa
 8000d36:	001a      	movs    r2, r3
 8000d38:	3401      	adds    r4, #1
 8000d3a:	606b      	str     r3, [r5, #4]
 8000d3c:	7821      	ldrb    r1, [r4]
 8000d3e:	3930      	subs    r1, #0x30
 8000d40:	2909      	cmp     r1, #9
 8000d42:	d903      	bls     #0x8000d4c
 8000d44:	2b00      	cmp     r3, #0
 8000d46:	d0c4      	beq     #0x8000cd2
 8000d48:	606a      	str     r2, [r5, #4]
 8000d4a:	e7c2      	b       #0x8000cd2
 8000d4c:	4342      	muls    r2, r0, r2
 8000d4e:	2301      	movs    r3, #1
 8000d50:	3401      	adds    r4, #1
 8000d52:	1852      	adds    r2, r2, r1
 8000d54:	e7f2      	b       #0x8000d3c
 8000d56:	aa05      	add     r2, sp, #0x14
 8000d58:	9200      	str     r2, [sp]
 8000d5a:	0029      	movs    r1, r5
 8000d5c:	0032      	movs    r2, r6
 8000d5e:	4b13      	ldr     r3, [pc, #0x4c]
 8000d60:	9802      	ldr     r0, [sp, #8]
 8000d62:	e000      	b       #0x8000d66
 8000d64:	bf00      	nop
 8000d66:	1c43      	adds    r3, r0, #1
 8000d68:	d1d3      	bne     #0x8000d12
 8000d6a:	6e73      	ldr     r3, [r6, #0x64]
 8000d6c:	07db      	lsls    r3, r3, #0x1f
 8000d6e:	d405      	bmi     #0x8000d7c
 8000d70:	89b3      	ldrh    r3, [r6, #0xc]
 8000d72:	059b      	lsls    r3, r3, #0x16
 8000d74:	d402      	bmi     #0x8000d7c
 8000d76:	6db0      	ldr     r0, [r6, #0x58]
 8000d78:	f7ff fde2 	bl      #0x8000940  <__retarget_lock_release_recursive>
 8000d7c:	89b3      	ldrh    r3, [r6, #0xc]
 8000d7e:	065b      	lsls    r3, r3, #0x19
 8000d80:	d500      	bpl     #0x8000d84
 8000d82:	e728      	b       #0x8000bd6
 8000d84:	6968      	ldr     r0, [r5, #0x14]
 8000d86:	e728      	b       #0x8000bda
 8000d88:	aa05      	add     r2, sp, #0x14
 8000d8a:	9200      	str     r2, [sp]
 8000d8c:	0029      	movs    r1, r5
 8000d8e:	0032      	movs    r2, r6
 8000d90:	4b06      	ldr     r3, [pc, #0x18]
 8000d92:	9802      	ldr     r0, [sp, #8]
 8000d94:	f000 f87c 	bl      #0x8000e90  <_printf_i>
 8000d98:	e7e5      	b       #0x8000d66
 8000d9a:	46c0      	mov     r8, r8
 8000d9c:	080013ef 	.word	0x080013ef
 8000da0:	080013f5 	.word	0x080013f5
 8000da4:	080013f9 	.word	0x080013f9
 8000da8:	00000000 	.word	0x00000000
 8000dac:	08000b67 	.word	0x08000b67

08000db0 <_printf_common>:
 8000db0:	b5f7      	push    {r0, r1, r2, r4, r5, r6, r7, lr}
 8000db2:	0016      	movs    r6, r2
 8000db4:	9301      	str     r3, [sp, #4]
 8000db6:	688a      	ldr     r2, [r1, #8]
 8000db8:	690b      	ldr     r3, [r1, #0x10]
 8000dba:	000c      	movs    r4, r1
 8000dbc:	9000      	str     r0, [sp]
 8000dbe:	4293      	cmp     r3, r2
 8000dc0:	da00      	bge     #0x8000dc4
 8000dc2:	0013      	movs    r3, r2
 8000dc4:	0022      	movs    r2, r4
 8000dc6:	6033      	str     r3, [r6]
 8000dc8:	3243      	adds    r2, #0x43
 8000dca:	7812      	ldrb    r2, [r2]
 8000dcc:	2a00      	cmp     r2, #0
 8000dce:	d001      	beq     #0x8000dd4
 8000dd0:	3301      	adds    r3, #1
 8000dd2:	6033      	str     r3, [r6]
 8000dd4:	6823      	ldr     r3, [r4]
 8000dd6:	069b      	lsls    r3, r3, #0x1a
 8000dd8:	d502      	bpl     #0x8000de0
 8000dda:	6833      	ldr     r3, [r6]
 8000ddc:	3302      	adds    r3, #2
 8000dde:	6033      	str     r3, [r6]
 8000de0:	6822      	ldr     r2, [r4]
 8000de2:	2306      	movs    r3, #6
 8000de4:	0015      	movs    r5, r2
 8000de6:	401d      	ands    r5, r3
 8000de8:	421a      	tst     r2, r3
 8000dea:	d027      	beq     #0x8000e3c
 8000dec:	0023      	movs    r3, r4
 8000dee:	3343      	adds    r3, #0x43
 8000df0:	781b      	ldrb    r3, [r3]
 8000df2:	1e5a      	subs    r2, r3, #1
 8000df4:	4193      	sbcs    r3, r2
 8000df6:	6822      	ldr     r2, [r4]
 8000df8:	0692      	lsls    r2, r2, #0x1a
 8000dfa:	d430      	bmi     #0x8000e5e
 8000dfc:	0022      	movs    r2, r4
 8000dfe:	9901      	ldr     r1, [sp, #4]
 8000e00:	9800      	ldr     r0, [sp]
 8000e02:	9d08      	ldr     r5, [sp, #0x20]
 8000e04:	3243      	adds    r2, #0x43
 8000e06:	47a8      	blx     r5
 8000e08:	3001      	adds    r0, #1
 8000e0a:	d025      	beq     #0x8000e58
 8000e0c:	2206      	movs    r2, #6
 8000e0e:	6823      	ldr     r3, [r4]
 8000e10:	2500      	movs    r5, #0
 8000e12:	4013      	ands    r3, r2
 8000e14:	2b04      	cmp     r3, #4
 8000e16:	d105      	bne     #0x8000e24
 8000e18:	6833      	ldr     r3, [r6]
 8000e1a:	68e5      	ldr     r5, [r4, #0xc]
 8000e1c:	1aed      	subs    r5, r5, r3
 8000e1e:	43eb      	mvns    r3, r5
 8000e20:	17db      	asrs    r3, r3, #0x1f
 8000e22:	401d      	ands    r5, r3
 8000e24:	68a3      	ldr     r3, [r4, #8]
 8000e26:	6922      	ldr     r2, [r4, #0x10]
 8000e28:	4293      	cmp     r3, r2
 8000e2a:	dd01      	ble     #0x8000e30
 8000e2c:	1a9b      	subs    r3, r3, r2
 8000e2e:	18ed      	adds    r5, r5, r3
 8000e30:	2600      	movs    r6, #0
 8000e32:	42b5      	cmp     r5, r6
 8000e34:	d120      	bne     #0x8000e78
 8000e36:	2000      	movs    r0, #0
 8000e38:	e010      	b       #0x8000e5c
 8000e3a:	3501      	adds    r5, #1
 8000e3c:	68e3      	ldr     r3, [r4, #0xc]
 8000e3e:	6832      	ldr     r2, [r6]
 8000e40:	1a9b      	subs    r3, r3, r2
 8000e42:	42ab      	cmp     r3, r5
 8000e44:	ddd2      	ble     #0x8000dec
 8000e46:	0022      	movs    r2, r4
 8000e48:	2301      	movs    r3, #1
 8000e4a:	9901      	ldr     r1, [sp, #4]
 8000e4c:	9800      	ldr     r0, [sp]
 8000e4e:	9f08      	ldr     r7, [sp, #0x20]
 8000e50:	3219      	adds    r2, #0x19
 8000e52:	47b8      	blx     r7
 8000e54:	3001      	adds    r0, #1
 8000e56:	d1f0      	bne     #0x8000e3a
 8000e58:	2001      	movs    r0, #1
 8000e5a:	4240      	rsbs    r0, r0, #0
 8000e5c:	bdfe      	pop     {r1, r2, r3, r4, r5, r6, r7, pc}
 8000e5e:	2030      	movs    r0, #0x30
 8000e60:	18e1      	adds    r1, r4, r3
 8000e62:	3143      	adds    r1, #0x43
 8000e64:	7008      	strb    r0, [r1]
 8000e66:	0021      	movs    r1, r4
 8000e68:	1c5a      	adds    r2, r3, #1
 8000e6a:	3145      	adds    r1, #0x45
 8000e6c:	7809      	ldrb    r1, [r1]
 8000e6e:	18a2      	adds    r2, r4, r2
 8000e70:	3243      	adds    r2, #0x43
 8000e72:	3302      	adds    r3, #2
 8000e74:	7011      	strb    r1, [r2]
 8000e76:	e7c1      	b       #0x8000dfc
 8000e78:	0022      	movs    r2, r4
 8000e7a:	2301      	movs    r3, #1
 8000e7c:	9901      	ldr     r1, [sp, #4]
 8000e7e:	9800      	ldr     r0, [sp]
 8000e80:	9f08      	ldr     r7, [sp, #0x20]
 8000e82:	321a      	adds    r2, #0x1a
 8000e84:	47b8      	blx     r7
 8000e86:	3001      	adds    r0, #1
 8000e88:	d0e6      	beq     #0x8000e58
 8000e8a:	3601      	adds    r6, #1
 8000e8c:	e7d1      	b       #0x8000e32
 8000e8e:	0000      	movs    r0, r0

08000e90 <_printf_i>:
 8000e90:	b5f0      	push    {r4, r5, r6, r7, lr}
 8000e92:	b08b      	sub     sp, #0x2c
 8000e94:	9307      	str     r3, [sp, #0x1c]
 8000e96:	9005      	str     r0, [sp, #0x14]
 8000e98:	9206      	str     r2, [sp, #0x18]
 8000e9a:	7e0a      	ldrb    r2, [r1, #0x18]
 8000e9c:	000c      	movs    r4, r1
 8000e9e:	3143      	adds    r1, #0x43
 8000ea0:	9b10      	ldr     r3, [sp, #0x40]
 8000ea2:	9103      	str     r1, [sp, #0xc]
 8000ea4:	2a78      	cmp     r2, #0x78
 8000ea6:	d809      	bhi     #0x8000ebc
 8000ea8:	2a62      	cmp     r2, #0x62
 8000eaa:	d80b      	bhi     #0x8000ec4
 8000eac:	2a00      	cmp     r2, #0
 8000eae:	d100      	bne     #0x8000eb2
 8000eb0:	e0ba      	b       #0x8001028
 8000eb2:	497a      	ldr     r1, [pc, #0x1e8]
 8000eb4:	9104      	str     r1, [sp, #0x10]
 8000eb6:	2a58      	cmp     r2, #0x58
 8000eb8:	d100      	bne     #0x8000ebc
 8000eba:	e08e      	b       #0x8000fda
 8000ebc:	0025      	movs    r5, r4
 8000ebe:	3542      	adds    r5, #0x42
 8000ec0:	702a      	strb    r2, [r5]
 8000ec2:	e022      	b       #0x8000f0a
 8000ec4:	0010      	movs    r0, r2
 8000ec6:	3863      	subs    r0, #0x63
 8000ec8:	2815      	cmp     r0, #0x15
 8000eca:	d8f7      	bhi     #0x8000ebc
 8000ecc:	f7ff f97e 	bl      #0x80001cc  <__gnu_thumb1_case_shi>
 8000ed0:	001f0016 	.word	0x001f0016
 8000ed4:	fff6fff6 	.word	0xfff6fff6
 8000ed8:	fff6fff6 	.word	0xfff6fff6
 8000edc:	fff6001f 	.word	0xfff6001f
 8000ee0:	fff6fff6 	.word	0xfff6fff6
 8000ee4:	009ffff6 	.word	0x009ffff6
 8000ee8:	007e0035 	.word	0x007e0035
 8000eec:	fff6fff6 	.word	0xfff6fff6
 8000ef0:	fff600b0 	.word	0xfff600b0
 8000ef4:	fff60035 	.word	0xfff60035
 8000ef8:	0082fff6 	.word	0x0082fff6
 8000efc:	0025      	movs    r5, r4
 8000efe:	681a      	ldr     r2, [r3]
 8000f00:	3542      	adds    r5, #0x42
 8000f02:	1d11      	adds    r1, r2, #4
 8000f04:	6019      	str     r1, [r3]
 8000f06:	6813      	ldr     r3, [r2]
 8000f08:	702b      	strb    r3, [r5]
 8000f0a:	2301      	movs    r3, #1
 8000f0c:	e09e      	b       #0x800104c
 8000f0e:	681a      	ldr     r2, [r3]
 8000f10:	6820      	ldr     r0, [r4]
 8000f12:	1d11      	adds    r1, r2, #4
 8000f14:	6019      	str     r1, [r3]
 8000f16:	0605      	lsls    r5, r0, #0x18
 8000f18:	d501      	bpl     #0x8000f1e
 8000f1a:	6816      	ldr     r6, [r2]
 8000f1c:	e003      	b       #0x8000f26
 8000f1e:	0640      	lsls    r0, r0, #0x19
 8000f20:	d5fb      	bpl     #0x8000f1a
 8000f22:	2300      	movs    r3, #0
 8000f24:	5ed6      	ldrsh   r6, [r2, r3]
 8000f26:	2e00      	cmp     r6, #0
 8000f28:	da03      	bge     #0x8000f32
 8000f2a:	232d      	movs    r3, #0x2d
 8000f2c:	9a03      	ldr     r2, [sp, #0xc]
 8000f2e:	4276      	rsbs    r6, r6, #0
 8000f30:	7013      	strb    r3, [r2]
 8000f32:	4b5a      	ldr     r3, [pc, #0x168]
 8000f34:	270a      	movs    r7, #0xa
 8000f36:	9304      	str     r3, [sp, #0x10]
 8000f38:	e013      	b       #0x8000f62
 8000f3a:	681a      	ldr     r2, [r3]
 8000f3c:	6821      	ldr     r1, [r4]
 8000f3e:	ca40      	ldm     r2!, {r6}
 8000f40:	0608      	lsls    r0, r1, #0x18
 8000f42:	d402      	bmi     #0x8000f4a
 8000f44:	0649      	lsls    r1, r1, #0x19
 8000f46:	d500      	bpl     #0x8000f4a
 8000f48:	b2b6      	uxth    r6, r6
 8000f4a:	601a      	str     r2, [r3]
 8000f4c:	7e23      	ldrb    r3, [r4, #0x18]
 8000f4e:	4a53      	ldr     r2, [pc, #0x14c]
 8000f50:	270a      	movs    r7, #0xa
 8000f52:	9204      	str     r2, [sp, #0x10]
 8000f54:	2b6f      	cmp     r3, #0x6f
 8000f56:	d100      	bne     #0x8000f5a
 8000f58:	3f02      	subs    r7, #2
 8000f5a:	0023      	movs    r3, r4
 8000f5c:	2200      	movs    r2, #0
 8000f5e:	3343      	adds    r3, #0x43
 8000f60:	701a      	strb    r2, [r3]
 8000f62:	6863      	ldr     r3, [r4, #4]
 8000f64:	60a3      	str     r3, [r4, #8]
 8000f66:	2b00      	cmp     r3, #0
 8000f68:	db06      	blt     #0x8000f78
 8000f6a:	2104      	movs    r1, #4
 8000f6c:	6822      	ldr     r2, [r4]
 8000f6e:	9d03      	ldr     r5, [sp, #0xc]
 8000f70:	438a      	bics    r2, r1
 8000f72:	6022      	str     r2, [r4]
 8000f74:	4333      	orrs    r3, r6
 8000f76:	d00c      	beq     #0x8000f92
 8000f78:	9d03      	ldr     r5, [sp, #0xc]
 8000f7a:	0030      	movs    r0, r6
 8000f7c:	0039      	movs    r1, r7
 8000f7e:	f7ff f91f 	bl      #0x80001c0  <__aeabi_uidivmod>
 8000f82:	9b04      	ldr     r3, [sp, #0x10]
 8000f84:	3d01      	subs    r5, #1
 8000f86:	5c5b      	ldrb    r3, [r3, r1]
 8000f88:	702b      	strb    r3, [r5]
 8000f8a:	0033      	movs    r3, r6
 8000f8c:	0006      	movs    r6, r0
 8000f8e:	429f      	cmp     r7, r3
 8000f90:	d9f3      	bls     #0x8000f7a
 8000f92:	2f08      	cmp     r7, #8
 8000f94:	d109      	bne     #0x8000faa
 8000f96:	6823      	ldr     r3, [r4]
 8000f98:	07db      	lsls    r3, r3, #0x1f
 8000f9a:	d506      	bpl     #0x8000faa
 8000f9c:	6862      	ldr     r2, [r4, #4]
 8000f9e:	6923      	ldr     r3, [r4, #0x10]
 8000fa0:	429a      	cmp     r2, r3
 8000fa2:	dc02      	bgt     #0x8000faa
 8000fa4:	2330      	movs    r3, #0x30
 8000fa6:	3d01      	subs    r5, #1
 8000fa8:	702b      	strb    r3, [r5]
 8000faa:	9b03      	ldr     r3, [sp, #0xc]
 8000fac:	1b5b      	subs    r3, r3, r5
 8000fae:	6123      	str     r3, [r4, #0x10]
 8000fb0:	9b07      	ldr     r3, [sp, #0x1c]
 8000fb2:	0021      	movs    r1, r4
 8000fb4:	9300      	str     r3, [sp]
 8000fb6:	9805      	ldr     r0, [sp, #0x14]
 8000fb8:	9b06      	ldr     r3, [sp, #0x18]
 8000fba:	aa09      	add     r2, sp, #0x24
 8000fbc:	f7ff fef8 	bl      #0x8000db0  <_printf_common>
 8000fc0:	3001      	adds    r0, #1
 8000fc2:	d148      	bne     #0x8001056
 8000fc4:	2001      	movs    r0, #1
 8000fc6:	4240      	rsbs    r0, r0, #0
 8000fc8:	b00b      	add     sp, #0x2c
 8000fca:	bdf0      	pop     {r4, r5, r6, r7, pc}
 8000fcc:	2220      	movs    r2, #0x20
 8000fce:	6821      	ldr     r1, [r4]
 8000fd0:	430a      	orrs    r2, r1
 8000fd2:	6022      	str     r2, [r4]
 8000fd4:	2278      	movs    r2, #0x78
 8000fd6:	4932      	ldr     r1, [pc, #0xc8]
 8000fd8:	9104      	str     r1, [sp, #0x10]
 8000fda:	0021      	movs    r1, r4
 8000fdc:	3145      	adds    r1, #0x45
 8000fde:	700a      	strb    r2, [r1]
 8000fe0:	6819      	ldr     r1, [r3]
 8000fe2:	6822      	ldr     r2, [r4]
 8000fe4:	c940      	ldm     r1!, {r6}
 8000fe6:	0610      	lsls    r0, r2, #0x18
 8000fe8:	d402      	bmi     #0x8000ff0
 8000fea:	0650      	lsls    r0, r2, #0x19
 8000fec:	d500      	bpl     #0x8000ff0
 8000fee:	b2b6      	uxth    r6, r6
 8000ff0:	6019      	str     r1, [r3]
 8000ff2:	07d3      	lsls    r3, r2, #0x1f
 8000ff4:	d502      	bpl     #0x8000ffc
 8000ff6:	2320      	movs    r3, #0x20
 8000ff8:	4313      	orrs    r3, r2
 8000ffa:	6023      	str     r3, [r4]
 8000ffc:	2e00      	cmp     r6, #0
 8000ffe:	d001      	beq     #0x8001004
 8001000:	2710      	movs    r7, #0x10
 8001002:	e7aa      	b       #0x8000f5a
 8001004:	2220      	movs    r2, #0x20
 8001006:	6823      	ldr     r3, [r4]
 8001008:	4393      	bics    r3, r2
 800100a:	6023      	str     r3, [r4]
 800100c:	e7f8      	b       #0x8001000
 800100e:	681a      	ldr     r2, [r3]
 8001010:	6825      	ldr     r5, [r4]
 8001012:	1d10      	adds    r0, r2, #4
 8001014:	6961      	ldr     r1, [r4, #0x14]
 8001016:	6018      	str     r0, [r3]
 8001018:	6813      	ldr     r3, [r2]
 800101a:	062e      	lsls    r6, r5, #0x18
 800101c:	d501      	bpl     #0x8001022
 800101e:	6019      	str     r1, [r3]
 8001020:	e002      	b       #0x8001028
 8001022:	066d      	lsls    r5, r5, #0x19
 8001024:	d5fb      	bpl     #0x800101e
 8001026:	8019      	strh    r1, [r3]
 8001028:	2300      	movs    r3, #0
 800102a:	9d03      	ldr     r5, [sp, #0xc]
 800102c:	6123      	str     r3, [r4, #0x10]
 800102e:	e7bf      	b       #0x8000fb0
 8001030:	681a      	ldr     r2, [r3]
 8001032:	1d11      	adds    r1, r2, #4
 8001034:	6019      	str     r1, [r3]
 8001036:	6815      	ldr     r5, [r2]
 8001038:	2100      	movs    r1, #0
 800103a:	0028      	movs    r0, r5
 800103c:	6862      	ldr     r2, [r4, #4]
 800103e:	f000 f97d 	bl      #0x800133c  <memchr>
 8001042:	2800      	cmp     r0, #0
 8001044:	d001      	beq     #0x800104a
 8001046:	1b40      	subs    r0, r0, r5
 8001048:	6060      	str     r0, [r4, #4]
 800104a:	6863      	ldr     r3, [r4, #4]
 800104c:	6123      	str     r3, [r4, #0x10]
 800104e:	2300      	movs    r3, #0
 8001050:	9a03      	ldr     r2, [sp, #0xc]
 8001052:	7013      	strb    r3, [r2]
 8001054:	e7ac      	b       #0x8000fb0
 8001056:	002a      	movs    r2, r5
 8001058:	6923      	ldr     r3, [r4, #0x10]
 800105a:	9906      	ldr     r1, [sp, #0x18]
 800105c:	9805      	ldr     r0, [sp, #0x14]
 800105e:	9d07      	ldr     r5, [sp, #0x1c]
 8001060:	47a8      	blx     r5
 8001062:	3001      	adds    r0, #1
 8001064:	d0ae      	beq     #0x8000fc4
 8001066:	6823      	ldr     r3, [r4]
 8001068:	079b      	lsls    r3, r3, #0x1e
 800106a:	d415      	bmi     #0x8001098
 800106c:	9b09      	ldr     r3, [sp, #0x24]
 800106e:	68e0      	ldr     r0, [r4, #0xc]
 8001070:	4298      	cmp     r0, r3
 8001072:	daa9      	bge     #0x8000fc8
 8001074:	0018      	movs    r0, r3
 8001076:	e7a7      	b       #0x8000fc8
 8001078:	0022      	movs    r2, r4
 800107a:	2301      	movs    r3, #1
 800107c:	9906      	ldr     r1, [sp, #0x18]
 800107e:	9805      	ldr     r0, [sp, #0x14]
 8001080:	9e07      	ldr     r6, [sp, #0x1c]
 8001082:	3219      	adds    r2, #0x19
 8001084:	47b0      	blx     r6
 8001086:	3001      	adds    r0, #1
 8001088:	d09c      	beq     #0x8000fc4
 800108a:	3501      	adds    r5, #1
 800108c:	68e3      	ldr     r3, [r4, #0xc]
 800108e:	9a09      	ldr     r2, [sp, #0x24]
 8001090:	1a9b      	subs    r3, r3, r2
 8001092:	42ab      	cmp     r3, r5
 8001094:	dcf0      	bgt     #0x8001078
 8001096:	e7e9      	b       #0x800106c
 8001098:	2500      	movs    r5, #0
 800109a:	e7f7      	b       #0x800108c
 800109c:	08001400 	.word	0x08001400
 80010a0:	08001411 	.word	0x08001411

080010a4 <__sflush_r>:
 80010a4:	b5f7      	push    {r0, r1, r2, r4, r5, r6, r7, lr}
 80010a6:	220c      	movs    r2, #0xc
 80010a8:	5e8b      	ldrsh   r3, [r1, r2]
 80010aa:	0005      	movs    r5, r0
 80010ac:	000c      	movs    r4, r1
 80010ae:	071a      	lsls    r2, r3, #0x1c
 80010b0:	d457      	bmi     #0x8001162
 80010b2:	684a      	ldr     r2, [r1, #4]
 80010b4:	2a00      	cmp     r2, #0
 80010b6:	dc02      	bgt     #0x80010be
 80010b8:	6c0a      	ldr     r2, [r1, #0x40]
 80010ba:	2a00      	cmp     r2, #0
 80010bc:	dd4f      	ble     #0x800115e
 80010be:	6ae7      	ldr     r7, [r4, #0x2c]
 80010c0:	2f00      	cmp     r7, #0
 80010c2:	d04c      	beq     #0x800115e
 80010c4:	2200      	movs    r2, #0
 80010c6:	2180      	movs    r1, #0x80
 80010c8:	682e      	ldr     r6, [r5]
 80010ca:	602a      	str     r2, [r5]
 80010cc:	001a      	movs    r2, r3
 80010ce:	0149      	lsls    r1, r1, #5
 80010d0:	400a      	ands    r2, r1
 80010d2:	420b      	tst     r3, r1
 80010d4:	d034      	beq     #0x8001140
 80010d6:	6d62      	ldr     r2, [r4, #0x54]
 80010d8:	89a3      	ldrh    r3, [r4, #0xc]
 80010da:	075b      	lsls    r3, r3, #0x1d
 80010dc:	d506      	bpl     #0x80010ec
 80010de:	6863      	ldr     r3, [r4, #4]
 80010e0:	1ad2      	subs    r2, r2, r3
 80010e2:	6b63      	ldr     r3, [r4, #0x34]
 80010e4:	2b00      	cmp     r3, #0
 80010e6:	d001      	beq     #0x80010ec
 80010e8:	6c23      	ldr     r3, [r4, #0x40]
 80010ea:	1ad2      	subs    r2, r2, r3
 80010ec:	2300      	movs    r3, #0
 80010ee:	0028      	movs    r0, r5
 80010f0:	6ae7      	ldr     r7, [r4, #0x2c]
 80010f2:	6a21      	ldr     r1, [r4, #0x20]
 80010f4:	47b8      	blx     r7
 80010f6:	230c      	movs    r3, #0xc
 80010f8:	5ee2      	ldrsh   r2, [r4, r3]
 80010fa:	1c43      	adds    r3, r0, #1
 80010fc:	d106      	bne     #0x800110c
 80010fe:	6829      	ldr     r1, [r5]
 8001100:	291d      	cmp     r1, #0x1d
 8001102:	d847      	bhi     #0x8001194
 8001104:	4b29      	ldr     r3, [pc, #0xa4]
 8001106:	40cb      	lsrs    r3, r1
 8001108:	07db      	lsls    r3, r3, #0x1f
 800110a:	d543      	bpl     #0x8001194
 800110c:	2300      	movs    r3, #0
 800110e:	6063      	str     r3, [r4, #4]
 8001110:	6923      	ldr     r3, [r4, #0x10]
 8001112:	6023      	str     r3, [r4]
 8001114:	04d2      	lsls    r2, r2, #0x13
 8001116:	d505      	bpl     #0x8001124
 8001118:	1c43      	adds    r3, r0, #1
 800111a:	d102      	bne     #0x8001122
 800111c:	682b      	ldr     r3, [r5]
 800111e:	2b00      	cmp     r3, #0
 8001120:	d100      	bne     #0x8001124
 8001122:	6560      	str     r0, [r4, #0x54]
 8001124:	6b61      	ldr     r1, [r4, #0x34]
 8001126:	602e      	str     r6, [r5]
 8001128:	2900      	cmp     r1, #0
 800112a:	d018      	beq     #0x800115e
 800112c:	0023      	movs    r3, r4
 800112e:	3344      	adds    r3, #0x44
 8001130:	4299      	cmp     r1, r3
 8001132:	d002      	beq     #0x800113a
 8001134:	0028      	movs    r0, r5
 8001136:	f7ff fc05 	bl      #0x8000944  <_free_r>
 800113a:	2300      	movs    r3, #0
 800113c:	6363      	str     r3, [r4, #0x34]
 800113e:	e00e      	b       #0x800115e
 8001140:	2301      	movs    r3, #1
 8001142:	0028      	movs    r0, r5
 8001144:	6a21      	ldr     r1, [r4, #0x20]
 8001146:	47b8      	blx     r7
 8001148:	0002      	movs    r2, r0
 800114a:	1c43      	adds    r3, r0, #1
 800114c:	d1c4      	bne     #0x80010d8
 800114e:	682b      	ldr     r3, [r5]
 8001150:	2b00      	cmp     r3, #0
 8001152:	d0c1      	beq     #0x80010d8
 8001154:	2b1d      	cmp     r3, #0x1d
 8001156:	d001      	beq     #0x800115c
 8001158:	2b16      	cmp     r3, #0x16
 800115a:	d11a      	bne     #0x8001192
 800115c:	602e      	str     r6, [r5]
 800115e:	2000      	movs    r0, #0
 8001160:	e01d      	b       #0x800119e
 8001162:	690e      	ldr     r6, [r1, #0x10]
 8001164:	2e00      	cmp     r6, #0
 8001166:	d0fa      	beq     #0x800115e
 8001168:	680f      	ldr     r7, [r1]
 800116a:	600e      	str     r6, [r1]
 800116c:	1bba      	subs    r2, r7, r6
 800116e:	9201      	str     r2, [sp, #4]
 8001170:	2200      	movs    r2, #0
 8001172:	079b      	lsls    r3, r3, #0x1e
 8001174:	d100      	bne     #0x8001178
 8001176:	694a      	ldr     r2, [r1, #0x14]
 8001178:	60a2      	str     r2, [r4, #8]
 800117a:	9b01      	ldr     r3, [sp, #4]
 800117c:	2b00      	cmp     r3, #0
 800117e:	ddee      	ble     #0x800115e
 8001180:	6aa3      	ldr     r3, [r4, #0x28]
 8001182:	0032      	movs    r2, r6
 8001184:	001f      	movs    r7, r3
 8001186:	0028      	movs    r0, r5
 8001188:	9b01      	ldr     r3, [sp, #4]
 800118a:	6a21      	ldr     r1, [r4, #0x20]
 800118c:	47b8      	blx     r7
 800118e:	2800      	cmp     r0, #0
 8001190:	dc06      	bgt     #0x80011a0
 8001192:	89a2      	ldrh    r2, [r4, #0xc]
 8001194:	2340      	movs    r3, #0x40
 8001196:	2001      	movs    r0, #1
 8001198:	4313      	orrs    r3, r2
 800119a:	81a3      	strh    r3, [r4, #0xc]
 800119c:	4240      	rsbs    r0, r0, #0
 800119e:	bdfe      	pop     {r1, r2, r3, r4, r5, r6, r7, pc}
 80011a0:	9b01      	ldr     r3, [sp, #4]
 80011a2:	1836      	adds    r6, r6, r0
 80011a4:	1a1b      	subs    r3, r3, r0
 80011a6:	9301      	str     r3, [sp, #4]
 80011a8:	e7e7      	b       #0x800117a
 80011aa:	46c0      	mov     r8, r8
 80011ac:	20400001 	.word	0x20400001

080011b0 <_fflush_r>:
 80011b0:	690b      	ldr     r3, [r1, #0x10]
 80011b2:	b570      	push    {r4, r5, r6, lr}
 80011b4:	0005      	movs    r5, r0
 80011b6:	000c      	movs    r4, r1
 80011b8:	2b00      	cmp     r3, #0
 80011ba:	d102      	bne     #0x80011c2
 80011bc:	001d      	movs    r5, r3
 80011be:	0028      	movs    r0, r5
 80011c0:	bd70      	pop     {r4, r5, r6, pc}
 80011c2:	2800      	cmp     r0, #0
 80011c4:	d004      	beq     #0x80011d0
 80011c6:	6a03      	ldr     r3, [r0, #0x20]
 80011c8:	2b00      	cmp     r3, #0
 80011ca:	d101      	bne     #0x80011d0
 80011cc:	f7ff f9ae 	bl      #0x800052c  <__sinit>
 80011d0:	220c      	movs    r2, #0xc
 80011d2:	5ea3      	ldrsh   r3, [r4, r2]
 80011d4:	2b00      	cmp     r3, #0
 80011d6:	d0f1      	beq     #0x80011bc
 80011d8:	6e62      	ldr     r2, [r4, #0x64]
 80011da:	07d2      	lsls    r2, r2, #0x1f
 80011dc:	d404      	bmi     #0x80011e8
 80011de:	059b      	lsls    r3, r3, #0x16
 80011e0:	d402      	bmi     #0x80011e8
 80011e2:	6da0      	ldr     r0, [r4, #0x58]
 80011e4:	f7ff fbab 	bl      #0x800093e  <__retarget_lock_acquire_recursive>
 80011e8:	0028      	movs    r0, r5
 80011ea:	0021      	movs    r1, r4
 80011ec:	f7ff ff5a 	bl      #0x80010a4  <__sflush_r>
 80011f0:	6e63      	ldr     r3, [r4, #0x64]
 80011f2:	0005      	movs    r5, r0
 80011f4:	07db      	lsls    r3, r3, #0x1f
 80011f6:	d4e2      	bmi     #0x80011be
 80011f8:	89a3      	ldrh    r3, [r4, #0xc]
 80011fa:	059b      	lsls    r3, r3, #0x16
 80011fc:	d4df      	bmi     #0x80011be
 80011fe:	6da0      	ldr     r0, [r4, #0x58]
 8001200:	f7ff fb9e 	bl      #0x8000940  <__retarget_lock_release_recursive>
 8001204:	e7db      	b       #0x80011be
 8001206:	0000      	movs    r0, r0

08001208 <__swhatbuf_r>:
 8001208:	b570      	push    {r4, r5, r6, lr}
 800120a:	000e      	movs    r6, r1
 800120c:	001d      	movs    r5, r3
 800120e:	230e      	movs    r3, #0xe
 8001210:	5ec9      	ldrsh   r1, [r1, r3]
 8001212:	0014      	movs    r4, r2
 8001214:	b096      	sub     sp, #0x58
 8001216:	2900      	cmp     r1, #0
 8001218:	da0c      	bge     #0x8001234
 800121a:	2180      	movs    r1, #0x80
 800121c:	000b      	movs    r3, r1
 800121e:	89b2      	ldrh    r2, [r6, #0xc]
 8001220:	4013      	ands    r3, r2
 8001222:	4211      	tst     r1, r2
 8001224:	d114      	bne     #0x8001250
 8001226:	2280      	movs    r2, #0x80
 8001228:	00d2      	lsls    r2, r2, #3
 800122a:	2000      	movs    r0, #0
 800122c:	602b      	str     r3, [r5]
 800122e:	6022      	str     r2, [r4]
 8001230:	b016      	add     sp, #0x58
 8001232:	bd70      	pop     {r4, r5, r6, pc}
 8001234:	466a      	mov     r2, sp
 8001236:	f000 f84b 	bl      #0x80012d0  <_fstat_r>
 800123a:	2800      	cmp     r0, #0
 800123c:	dbed      	blt     #0x800121a
 800123e:	22f0      	movs    r2, #0xf0
 8001240:	9b01      	ldr     r3, [sp, #4]
 8001242:	0212      	lsls    r2, r2, #8
 8001244:	4013      	ands    r3, r2
 8001246:	4a04      	ldr     r2, [pc, #0x10]
 8001248:	189b      	adds    r3, r3, r2
 800124a:	425a      	rsbs    r2, r3, #0
 800124c:	4153      	adcs    r3, r2
 800124e:	e7ea      	b       #0x8001226
 8001250:	2300      	movs    r3, #0
 8001252:	2240      	movs    r2, #0x40
 8001254:	e7e9      	b       #0x800122a
 8001256:	46c0      	mov     r8, r8
 8001258:	ffffe000 	.word	0xffffe000

0800125c <__smakebuf_r>:
 800125c:	b5f7      	push    {r0, r1, r2, r4, r5, r6, r7, lr}
 800125e:	2502      	movs    r5, #2
 8001260:	898b      	ldrh    r3, [r1, #0xc]
 8001262:	0006      	movs    r6, r0
 8001264:	000c      	movs    r4, r1
 8001266:	421d      	tst     r5, r3
 8001268:	d006      	beq     #0x8001278
 800126a:	0023      	movs    r3, r4
 800126c:	3347      	adds    r3, #0x47
 800126e:	6023      	str     r3, [r4]
 8001270:	6123      	str     r3, [r4, #0x10]
 8001272:	2301      	movs    r3, #1
 8001274:	6163      	str     r3, [r4, #0x14]
 8001276:	bdf7      	pop     {r0, r1, r2, r4, r5, r6, r7, pc}
 8001278:	466a      	mov     r2, sp
 800127a:	ab01      	add     r3, sp, #4
 800127c:	f7ff ffc4 	bl      #0x8001208  <__swhatbuf_r>
 8001280:	9f00      	ldr     r7, [sp]
 8001282:	0030      	movs    r0, r6
 8001284:	0039      	movs    r1, r7
 8001286:	f7ff fbc9 	bl      #0x8000a1c  <_malloc_r>
 800128a:	220c      	movs    r2, #0xc
 800128c:	5ea3      	ldrsh   r3, [r4, r2]
 800128e:	2800      	cmp     r0, #0
 8001290:	d106      	bne     #0x80012a0
 8001292:	059a      	lsls    r2, r3, #0x16
 8001294:	d4ef      	bmi     #0x8001276
 8001296:	2203      	movs    r2, #3
 8001298:	4393      	bics    r3, r2
 800129a:	431d      	orrs    r5, r3
 800129c:	81a5      	strh    r5, [r4, #0xc]
 800129e:	e7e4      	b       #0x800126a
 80012a0:	2280      	movs    r2, #0x80
 80012a2:	4313      	orrs    r3, r2
 80012a4:	81a3      	strh    r3, [r4, #0xc]
 80012a6:	9b01      	ldr     r3, [sp, #4]
 80012a8:	6020      	str     r0, [r4]
 80012aa:	6120      	str     r0, [r4, #0x10]
 80012ac:	6167      	str     r7, [r4, #0x14]
 80012ae:	2b00      	cmp     r3, #0
 80012b0:	d0e1      	beq     #0x8001276
 80012b2:	0030      	movs    r0, r6
 80012b4:	230e      	movs    r3, #0xe
 80012b6:	5ee1      	ldrsh   r1, [r4, r3]
 80012b8:	f000 f81c 	bl      #0x80012f4  <_isatty_r>
 80012bc:	2800      	cmp     r0, #0
 80012be:	d0da      	beq     #0x8001276
 80012c0:	2303      	movs    r3, #3
 80012c2:	89a2      	ldrh    r2, [r4, #0xc]
 80012c4:	439a      	bics    r2, r3
 80012c6:	3b02      	subs    r3, #2
 80012c8:	4313      	orrs    r3, r2
 80012ca:	81a3      	strh    r3, [r4, #0xc]
 80012cc:	e7d3      	b       #0x8001276
 80012ce:	0000      	movs    r0, r0

080012d0 <_fstat_r>:
 80012d0:	2300      	movs    r3, #0
 80012d2:	b570      	push    {r4, r5, r6, lr}
 80012d4:	4c06      	ldr     r4, [pc, #0x18]
 80012d6:	0005      	movs    r5, r0
 80012d8:	0008      	movs    r0, r1
 80012da:	0011      	movs    r1, r2
 80012dc:	6023      	str     r3, [r4]
 80012de:	f7ff f80e 	bl      #0x80002fe  <_fstat>
 80012e2:	1c43      	adds    r3, r0, #1
 80012e4:	d103      	bne     #0x80012ee
 80012e6:	6823      	ldr     r3, [r4]
 80012e8:	2b00      	cmp     r3, #0
 80012ea:	d000      	beq     #0x80012ee
 80012ec:	602b      	str     r3, [r5]
 80012ee:	bd70      	pop     {r4, r5, r6, pc}
 80012f0:	200001bc 	.word	0x200001bc

080012f4 <_isatty_r>:
 80012f4:	2300      	movs    r3, #0
 80012f6:	b570      	push    {r4, r5, r6, lr}
 80012f8:	4c06      	ldr     r4, [pc, #0x18]
 80012fa:	0005      	movs    r5, r0
 80012fc:	0008      	movs    r0, r1
 80012fe:	6023      	str     r3, [r4]
 8001300:	f7fe fff9 	bl      #0x80002f6  <_isatty>
 8001304:	1c43      	adds    r3, r0, #1
 8001306:	d103      	bne     #0x8001310
 8001308:	6823      	ldr     r3, [r4]
 800130a:	2b00      	cmp     r3, #0
 800130c:	d000      	beq     #0x8001310
 800130e:	602b      	str     r3, [r5]
 8001310:	bd70      	pop     {r4, r5, r6, pc}
 8001312:	46c0      	mov     r8, r8
 8001314:	200001bc 	.word	0x200001bc

08001318 <_sbrk_r>:
 8001318:	2300      	movs    r3, #0
 800131a:	b570      	push    {r4, r5, r6, lr}
 800131c:	4c06      	ldr     r4, [pc, #0x18]
 800131e:	0005      	movs    r5, r0
 8001320:	0008      	movs    r0, r1
 8001322:	6023      	str     r3, [r4]
 8001324:	f7fe ffee 	bl      #0x8000304  <_sbrk>
 8001328:	1c43      	adds    r3, r0, #1
 800132a:	d103      	bne     #0x8001334
 800132c:	6823      	ldr     r3, [r4]
 800132e:	2b00      	cmp     r3, #0
 8001330:	d000      	beq     #0x8001334
 8001332:	602b      	str     r3, [r5]
 8001334:	bd70      	pop     {r4, r5, r6, pc}
 8001336:	46c0      	mov     r8, r8
 8001338:	200001bc 	.word	0x200001bc

0800133c <memchr>:
 800133c:	b2c9      	uxtb    r1, r1
 800133e:	1882      	adds    r2, r0, r2
 8001340:	4290      	cmp     r0, r2
 8001342:	d101      	bne     #0x8001348
 8001344:	2000      	movs    r0, #0
 8001346:	4770      	bx      lr
 8001348:	7803      	ldrb    r3, [r0]
 800134a:	428b      	cmp     r3, r1
 800134c:	d0fb      	beq     #0x8001346
 800134e:	3001      	adds    r0, #1
 8001350:	e7f6      	b       #0x8001340
 8001352:	0000      	movs    r0, r0

08001354 <_init>:
 8001354:	b5f8      	push    {r3, r4, r5, r6, r7, lr}
 8001356:	46c0      	mov     r8, r8
 8001358:	bcf8      	pop     {r3, r4, r5, r6, r7}
 800135a:	bc08      	pop     {r3}
 800135c:	469e      	mov     lr, r3
 800135e:	4770      	bx      lr

08001360 <_fini>:
 8001360:	b5f8      	push    {r3, r4, r5, r6, r7, lr}
 8001362:	46c0      	mov     r8, r8
 8001364:	bcf8      	pop     {r3, r4, r5, r6, r7}
 8001366:	bc08      	pop     {r3}
 8001368:	469e      	mov     lr, r3
 800136a:	4770      	bx      lr
