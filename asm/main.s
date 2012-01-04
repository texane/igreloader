	.file "/home/texane/tmp/hex/main.c"
	.align	2
	.global	_main	; export
_main:
	.set ___PA___,1
	lnk	#2
	mov	w0,[w14]
	mov	#3,w2
	mov	#42,w1
	mov	[w14],w0
	rcall	_fubar
	clr	w0
	ulnk	
	return	
	.set ___PA___,0
	.align	2
_fubar:
	.set ___PA___,0
	lnk	#6
	mov	w0,[w14]
	mov	w1,[w14+2]
	mov	w2,[w14+4]
	nop
	ulnk	
	return	
	.set ___PA___,0

	.section __c30_signature, info, data
	.word 0x0001
	.word 0x0000
	.word 0x0000

	.set ___PA___,0
	.end
