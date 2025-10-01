#include "config.h" // Include config.h to access macros

int main(void)
{
// Assembly Fundamentals Examples
#ifdef ASSEMBLY_BLINK_BASIC
	main_blink_asm();
#endif

#ifdef ASSEMBLY_BLINK_PATTERN
	main_blink_asm_macro();
#endif

#ifdef ASSEMBLY_BLINK_INDIVIDUAL
	main_blink_pin();
#endif

#ifdef ASSEMBLY_BUTTON_SIMPLE
	main_button_simple();
#endif

#ifdef ASSEMBLY_BUTTON_LED_CONTROL
	main_button_led_control();
#endif

// C Hardware Abstraction Examples
#ifdef BUZZER_BASIC_BEEP
	main_buzzer_basic_beep();
#endif

#ifdef ASSEMBLY_BLINK_INDIVIDUAL
	main_blink_pin();
#endif

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

// Audio Examples
#ifdef BUZZER_BASIC_BEEP
	main_buzzer_basic_beep();
#endif

// Timer Examples
#ifdef C_TIMER_BASIC
	main_timer_basic();
#endif

#ifdef C_TIMER_INTERRUPT
	main_timer_interrupt();
#endif

#ifdef C_TIMER_PWM
	main_timer_pwm();
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
#endif #ifdef GRAPHICS_BOUNCING_BALL
	main_graphics_bouncing_ball();
#endif

#ifdef GRAPHICS_MOVING_SQUARE
	main_graphics_moving_square();
#endif

#ifdef GRAPHICS_SINE_WAVE
	main_graphics_sine_wave();
#endif

#ifdef MEMORY_BASIC
	main_memory_basic();
#endif

#ifdef MEMORY_STACK
	main_memory_stack();
#endif

#ifdef MEMORY_HEAP
	main_memory_heap();
#endif

// EEPROM Data Storage Examples
#ifdef EEPROM_BASIC
	main_eeprom_basic();
#endif

#ifdef EEPROM_LOGGER
	main_eeprom_logger();
#endif

#ifdef EEPROM_SETTINGS
	main_eeprom_settings();
#endif

	// IoT and Advanced Communication Examples
#ifdef IOT_SENSOR_MONITORING
	main_iot_sensor_monitoring();
#endif

#ifdef IOT_REMOTE_CONTROL
	main_iot_remote_control();
#endif

#ifdef IOT_DATA_VISUALIZATION
	main_iot_data_visualization();
#endif

	// Port Control Examples
#ifdef PORT_BLINKING
	main_port_blinking();
#endif

#ifdef PORT_ROTATION
	main_port_rotation();
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

	// Graphics Examples
#ifdef GRAPHICS_BASIC_SHAPES
	main_graphics_basic_shapes();
#endif

#ifdef GRAPHICS_ANIMATION
	main_graphics_animation();
#endif

#ifdef GRAPHICS_SENSOR_DISPLAY
	main_graphics_sensor_display();
#endif

#ifdef GRAPHICS_BASICS
	main_graphics_basics();
#endif

#ifdef GRAPHICS_MOVEMENT
	main_graphics_movement();
#endif

#ifdef GRAPHICS_RANDOM
	main_graphics_random();
#endif #ifdef GRAPHICS_ANIMATION
	main_graphics_animation();
#endif

#ifdef GRAPHICS_SENSOR_DISPLAY
	main_graphics_sensor_display();
#endif

// Game Examples
#ifdef GAME_SIMON_SAYS
	main_game_simon_says();
#endif

#ifdef GAME_REACTION_TIMER
	main_game_reaction_timer();
#endif

#ifdef GAME_SENSOR_TARGET
	main_game_sensor_target();
#endif

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

// Educational Examples
#ifdef EDUCATIONAL_FLOW_TEST
	main_educational_flow_test();
#endif

#ifdef EDUCATIONAL_SIMPLE_TEST
	main_educational_simple_test();
#endif

#ifdef ASSEMBLY_PROGRESSION_EXAMPLE
	main_assembly_progression();
#endif

#ifdef EDUCATIONAL_DEMO
	main_educational_demo();
#endif

	return 0;
}
