#define nop()  __asm__ __volatile__ ("nop" ::)
#define NOP()  __asm__ __volatile__ ("nop" ::)
#define _NOP()  __asm__ __volatile__ ("nop" ::)

#define CLI()	cli()
#define SEI()	sei()
#define BIT(x)	(1 << (x))

typedef unsigned char		byte;
typedef unsigned char		u8;
typedef signed int			s16;
typedef unsigned int		u16;
typedef signed long			s32;
typedef unsigned long		u32;
typedef signed long long	s64;
typedef unsigned long long 	u64;

#define sbi(port, bit) 	(port |= (1<<bit))
#define cbi(port, bit) 	(port &= (~(1<<bit)))
#define inp(port, bit) 	(port & (1<<bit))

// bit ��ũ��
#define SetBit(x,y)		(x|=(1<<y))
#define ClrBit(x,y)		(x&=~(1<<y))
#define ToggleBit(x,y)	(x^=(1<<y))
#define FlipBit(x,y)	(x^=(1<<y)) // Same as ToggleBit.
#define TestBit(x,y)	(x&(1<<y))

// bit ��ũ��
#define SETBIT(x,y)		(x|=(1<<y))
#define CLEARBIT(x,y)	(x&=~(1<<y))
#define TOGGLEBIT(x,y)	(x^=(1<<y))
#define FLIPBIT(x,y)	(x^=(1<<y)) // Same as ToggleBit.
#define TESTBIT(x,y)	(x&(1<<y))

extern unsigned int INT0_DataView;
extern unsigned int INT1_DataView;
extern unsigned int Uart1_DataView;

// Demo function declarations
void main_blink_port(void);
void main_blink_pin(void);
void main_serial_polling_single_char(void);
void main_serial_polling_string(void);
void main_serial_polling_echo(void);
void main_serial_polling_sentence(void);
void main_serial_interrupt_rx(void);
void main_serial_interrupt_tx(void);
void main_adc_interrupt(void);
void main_adc_interrupt_uart_interrupt(void);
void main_adc_interrupt_uart_polling(void);
void main_adc_polling(void);
void main_graphics_basics(void);
void main_graphics_movement(void);
void main_graphics_random(void);
void main_graphics_bouncing_ball(void);
void main_graphics_moving_square(void);
void main_graphics_sine_wave(void);
void main_motors_fullstep(void);
void main_motors_fullstep_interrupt(void);
void main_motors_halfstep(void);
void main_motors_stepper_demo(void);
void main_motors_pwm_fast(void);
void main_motors_pwm_phasecorrect(void);
void main_motors_servo(void);
void main_timer_counter(void);
void main_timer_ctc(void);
void main_timer_fastpwm(void);
void main_timer_normal(void);
void main_interrupt_external(void);
void main_interrupt_timer(void);
void main_interrupt_timer_ctc(void);
void main_interrupt_ext_timer(void);
void main_blink_asm(void);
void main_blink_asm_macro(void);
void main_blink_asm_random(void);
void main_blink_asm_random_delay(void);
void main_cds(void);
void main_iot(void);
void main_inline(void);
void main_memory_eeprom(void);
void main_memory_program(void);
void main_joystick(void);