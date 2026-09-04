#include "config.h" // Include config.h to access macros

int main(void)
{
	// UART Examples
	#ifdef SERIAL_POLLING_SINGLE_CHAR
	main_serial_polling_single_char();
	#endif

	#ifdef SERIAL_POLLING_STRING
	main_serial_polling_string();
	#endif

	#ifdef SERIAL_POLLING_ECHO
	main_serial_polling_echo();
	#endif

	#ifdef SERIAL_POLLING_SENTENCE
	main_serial_polling_sentence();
	#endif

	#ifdef SERIAL_INTERRUPT_RX
	main_serial_interrupt_rx();
	#endif

	#ifdef SERIAL_INTERRUPT_TX
	main_serial_interrupt_tx();
	#endif

	// ADC Examples
	#ifdef ADC_INTERRUPT
	main_adc_interrupt();
	#endif

	#ifdef ADC_INTERRUPT_UART_INTERRUPT
	main_adc_interrupt_uart_interrupt();
	#endif

	#ifdef ADC_INTERRUPT_UART_POLLING
	main_adc_interrupt_uart_polling();
	#endif

	#ifdef ADC_POLLING
	main_adc_polling();
	#endif

	// Graphics Examples
	#ifdef GRAPHICS_BASICS
	main_graphics_basics();
	#endif

	#ifdef GRAPHICS_MOVEMENT
	main_graphics_movement();
	#endif

	#ifdef GRAPHICS_RANDOM
	main_graphics_random();
	#endif

	#ifdef GRAPHICS_BOUNCING_BALL
	main_graphics_bouncing_ball();
	#endif
	
	#ifdef GRAPHICS_MOVING_SQUARE
	main_graphics_moving_square();
	#endif
	
	#ifdef GRAPHICS_SINE_WAVE
	main_graphics_sine_wave();
	#endif

	// Motor Examples
	#ifdef MOTORS_FULLSTEP
	main_motors_fullstep();
	#endif

	#ifdef MOTORS_FULLSTEP_INTERRUPT
	main_motors_fullstep_interrupt();
	#endif

	#ifdef MOTORS_HALFSTEP
	main_motors_halfstep();
	#endif

	#ifdef MOTORS_STEPPER_DEMO
	main_motors_stepper_demo();
	#endif

	#ifdef MOTORS_PWM_FAST
	main_motors_pwm_fast();
	#endif

	#ifdef MOTORS_PWM_PHASECORRECT
	main_motors_pwm_phasecorrect();
	#endif

	#ifdef MOTORS_SERVO
	main_motors_servo();
	#endif

	#ifdef MOTORS_SERVO_ADC
	main_motors_servo_adc();
	#endif

	// Sound Examples
	#ifdef SOUND
	main_sound();
	#endif

	#ifdef SOUND_ATARI
	main_sound_atari();
	#endif

	#ifdef SOUND_TWINKLE
	main_sound_twingkle();
	#endif

	// Game Examples
	#ifdef GAME_HANGMAN
	main_game_hangman();
	#endif

	#ifdef GAME_OBSTACLE
	main_game_obstacle();
	#endif

	#ifdef GAME_OBSTACLE_LEVEL
	main_game_obstacle_level();
	#endif

	#ifdef GAME_PUZZLE
	main_game_puzzle();
	#endif

	#ifdef GAME_PONG_UART_CONTROL
	main_game_pong_uart_control();
	#endif

	// Timer Examples
	#ifdef TIMER_COUNTER
	main_timer_counter();
	#endif

	#ifdef TIMER_CTC
	main_timer_ctc();
	#endif

	#ifdef TIMER_FASTPWM
	main_timer_fastpwm();
	#endif

	#ifdef TIMER_NORMAL
	main_timer_normal();
	#endif

	// External Interrupt Examples
	#ifdef INTERRUPT_EXTERNAL
	main_interrupt_external();
	#endif

	#ifdef INTERRUPT_TIMER
	main_interrupt_timer();
	#endif

	#ifdef INTERRUPT_TIMER_CTC
	main_interrupt_timer_ctc();
	#endif

	#ifdef INTERRUPT_EXT_TIMER
	main_interrupt_ext_timer();
	#endif

	// Port/Pin Control Examples
	#ifdef BLINK_PORT
	main_blink_port();
	#endif

	#ifdef BLINK_PIN
	main_blink_pin();
	#endif

	#ifdef BLINK_ASM
	main_blink_asm();
	#endif

	#ifdef BLINK_ASM_MACRO
	main_blink_asm_macro();
	#endif

	#ifdef BLINK_ASM_RANDOM
	main_blink_asm_random();
	#endif

	#ifdef BLINK_ASM_RANDOM_DELAY
	main_blink_asm_random_delay();
	#endif

	// Miscellaneous Examples
	#ifdef CDS
	main_cds();
	#endif

	#ifdef IOT
	main_iot();
	#endif

	#ifdef INLINE
	main_inline();
	#endif

	#ifdef MEMORY_EEPROM
	main_memory_eeprom();
	#endif

	#ifdef MEMORY_PROGRAM
	main_memory_program();
	#endif

	#ifdef JOYSTICK
	main_joystick();
	#endif

	return 0;
}
