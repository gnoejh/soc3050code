#define nop() __asm__ __volatile__("nop" ::)
#define NOP() __asm__ __volatile__("nop" ::)
#define _NOP() __asm__ __volatile__("nop" ::)

#define CLI() cli()
#define SEI() sei()
#define BIT(x) (1 << (x))

typedef unsigned char byte;
typedef unsigned char u8;
typedef signed int s16;
typedef unsigned int u16;
typedef signed long s32;
typedef unsigned long u32;
typedef signed long long s64;
typedef unsigned long long u64;

#define sbi(port, bit) (port |= (1 << bit))
#define cbi(port, bit) (port &= (~(1 << bit)))
#define inp(port, bit) (port & (1 << bit))

// bit ��ũ��
#define SetBit(x, y) (x |= (1 << y))
#define ClrBit(x, y) (x &= ~(1 << y))
#define ToggleBit(x, y) (x ^= (1 << y))
#define FlipBit(x, y) (x ^= (1 << y)) // Same as ToggleBit.
#define TestBit(x, y) (x & (1 << y))

// bit ��ũ��
#define SETBIT(x, y) (x |= (1 << y))
#define CLEARBIT(x, y) (x &= ~(1 << y))
#define TOGGLEBIT(x, y) (x ^= (1 << y))
#define FLIPBIT(x, y) (x ^= (1 << y)) // Same as ToggleBit.
#define TESTBIT(x, y) (x & (1 << y))

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
void main_serial_interrupt_echo(void);
void main_serial_interrupt_sentence(void);
void main_serial_interrupt_circular_buffer(void);
void main_adc_interrupt(void);
void main_adc_interrupt_uart_interrupt(void);
void main_adc_interrupt_uart_polling(void);
void main_adc_polling(void);
void main_graphics_basic_shapes(void);
void main_graphics_animation(void);
void main_graphics_sensor_display(void);
void main_graphics_basics(void);
void main_graphics_movement(void);
void main_graphics_random(void);
void main_graphics_bouncing_ball(void);
void main_graphics_moving_square(void);
void main_graphics_sine_wave(void);

// Port Control Examples
void main_port_blinking(void);
void main_port_rotation(void);

void main_motors_dc_pwm(void);
void main_motors_servo_basic(void);
void main_motors_stepper_basic(void);
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
// Assembly Examples - LED Blinking Functions
int main_blink_asm(void);
int main_blink_asm_macro(void);
void main_blink_asm_random(void);
void main_blink_asm_random_delay(void);

// Assembly Examples - Button Functions
void main_button_simple(void);
void main_button_led_control(void);

// C Hardware Abstraction Examples
void main_c_led_basic(void);
void main_c_led_patterns(void);
void main_c_led_button_interactive(void);
void main_serial_polling_single_char(void);
void main_serial_polling_string(void);
void main_serial_interrupt_rx(void);
void main_adc_basic_reading(void);
void main_adc_voltage_conversion(void);
void main_adc_multiple_channels(void);
void main_buzzer_basic_beep(void);
void main_timer_basic(void);
void main_timer_interrupt(void);
void main_timer_pwm(void);
void main_cds(void);
void main_iot(void);
void main_inline(void);
void main_memory_eeprom(void);
void main_memory_program(void);
void main_joystick(void);

// Game Examples
int main_game_simon_says(void);
int main_game_reaction_timer(void);
int main_game_sensor_target(void);
int main_game_hangman(void);
int main_game_obstacle(void);

// Memory Management Examples
void main_memory_basic(void);
void main_memory_stack(void);
void main_memory_heap(void);

// EEPROM Data Storage Examples
void main_eeprom_basic(void);
void main_eeprom_logger(void);
void main_eeprom_settings(void);

// IoT and Advanced Communication Examples
void main_iot_sensor_monitoring(void);
void main_iot_remote_control(void);
void main_iot_data_visualization(void);

// Educational progression functions
void main_educational_flow_test(void);
void main_assembly_progression(void);
void main_python_interface(void);
void main_educational_simple_test(void);
void demo_register_vs_abstraction(void);
void main_educational_demo(void);