#ifdef INLINE
/* Inline assembly */
/*https://www.nongnu.org/avr-libc/user-manual/inline_asm.html */

#include "portpins.h"
void main_inline (void){
	
	asm("ldi r16,255");
	asm("out DDRB,r16");
	//asm("out PORTB, r16");

}
#endif