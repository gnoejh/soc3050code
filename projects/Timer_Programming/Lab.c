/*
 * ═══════════════════════════════════════════════════════════════════════════
 * INTEGRATED TIMER SYSTEM - HANDS-ON LAB
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * PROJECT: Real-Time Multi-Function Timer Dashboard
 * COURSE: SOC 3050 - Embedded Systems Integration
 * 
 * LEARNING OBJECTIVES:
 * Students build a complete real-world embedded system integrating:
 *   ✓ Timer/Counter (ATmega128 Timer0, Timer1, Timer2)
 *   ✓ Interrupt Service Routines (ISR)
 *   ✓ Graphics LCD Display (128x64 GLCD)
 *   ✓ Serial Communication (UART)
 *   ✓ User Interface (buttons + serial commands)
 * 
 * REAL-WORLD APPLICATION:
 * Multi-Function Digital Dashboard with:
 *   - Real-time clock (RTC) with Timer2
 *   - Stopwatch with lap times (Timer1)
 *   - PWM-based LED brightness control (Timer0)
 *   - Task scheduler (cooperative multitasking)
 *   - Serial command interface
 *   - Graphical UI on LCD
 * 
 * HARDWARE SETUP:
 *   - ATmega128 @ 16MHz
 *   - KS0108 Graphics LCD (128x64)
 *   - 8 LEDs on PORTB
 *   - Push buttons: PD4 (START/STOP), PD5 (LAP/RESET), PD6 (MODE)
 *   - UART1 @ 9600 baud for serial commands
 * 
 * STUDENT LEARNING PATH:
 * 1. Understand timer configuration for different purposes
 * 2. Master ISR-based event handling
 * 3. Integrate graphics rendering with real-time updates
 * 4. Build responsive user interfaces
 * 5. Implement cooperative multitasking
 * 6. Debug timing-critical systems
 * 
 * DURATION: 2-3 hours (comprehensive lab session)
 * DIFFICULTY: ★★★★☆ (Advanced Integration)
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include "config.h"
#include <stdio.h>
#include <string.h>

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * SYSTEM CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════
 */

// Hardware pin definitions
#define BTN_START_STOP  4  // PD4
#define BTN_LAP_RESET   5  // PD5
#define BTN_MODE        6  // PD6

// System modes
typedef enum {
    MODE_CLOCK = 0,      // Real-time clock display
    MODE_STOPWATCH,      // Stopwatch with lap times
    MODE_LED_CONTROL,    // PWM LED brightness
    MODE_TASK_MONITOR,   // Task scheduler monitor
    MODE_COUNT           // Total number of modes
} SystemMode;

// Global system state
volatile SystemMode current_mode = MODE_CLOCK;
volatile uint8_t mode_changed = 1;  // Flag for screen refresh

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * REAL-TIME CLOCK (Timer2 + ISR)
 * ═══════════════════════════════════════════════════════════════════════════
 */

volatile uint8_t rtc_hours = 12;
volatile uint8_t rtc_minutes = 0;
volatile uint8_t rtc_seconds = 0;
volatile uint16_t rtc_milliseconds = 0;

ISR(TIMER2_OVF_vect) {
    // Timer2 overflow: ~4ms at 16MHz with prescaler 1024
    // 250 overflows ≈ 1 second
    static uint8_t overflow_count = 0;
    
    overflow_count++;
    rtc_milliseconds += 4;
    
    if (overflow_count >= 250) {
        overflow_count = 0;
        rtc_seconds++;
        
        if (rtc_seconds >= 60) {
            rtc_seconds = 0;
            rtc_minutes++;
            
            if (rtc_minutes >= 60) {
                rtc_minutes = 0;
                rtc_hours++;
                
                if (rtc_hours >= 24) {
                    rtc_hours = 0;
                }
            }
        }
    }
}

void rtc_init(void) {
    // Timer2: Normal mode, prescaler 1024
    TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);  // Prescaler 1024
    TIMSK |= (1 << TOIE2);  // Enable overflow interrupt
    TCNT2 = 0;
}

void rtc_set_time(uint8_t h, uint8_t m, uint8_t s) {
    cli();
    rtc_hours = h;
    rtc_minutes = m;
    rtc_seconds = s;
    rtc_milliseconds = 0;
    sei();
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * STOPWATCH (Timer1 + ISR)
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef struct {
    uint16_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
} LapTime;

volatile uint8_t stopwatch_running = 0;
volatile uint16_t stopwatch_minutes = 0;
volatile uint8_t stopwatch_seconds = 0;
volatile uint16_t stopwatch_milliseconds = 0;

LapTime lap_times[5];  // Store up to 5 lap times
volatile uint8_t lap_count = 0;

ISR(TIMER1_COMPA_vect) {
    // Timer1 Compare Match: Every 1ms
    if (stopwatch_running) {
        stopwatch_milliseconds++;
        
        if (stopwatch_milliseconds >= 1000) {
            stopwatch_milliseconds = 0;
            stopwatch_seconds++;
            
            if (stopwatch_seconds >= 60) {
                stopwatch_seconds = 0;
                stopwatch_minutes++;
            }
        }
    }
}

void stopwatch_init(void) {
    // Timer1 CTC mode: 1ms interrupts
    // OCR1A = (16MHz / (prescaler * freq)) - 1
    // OCR1A = (16,000,000 / (64 * 1000)) - 1 = 249
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // CTC mode, prescaler 64
    OCR1A = 249;
    TIMSK |= (1 << OCIE1A);  // Enable compare match interrupt
}

void stopwatch_start(void) {
    stopwatch_running = 1;
}

void stopwatch_stop(void) {
    stopwatch_running = 0;
}

void stopwatch_reset(void) {
    cli();
    stopwatch_running = 0;
    stopwatch_minutes = 0;
    stopwatch_seconds = 0;
    stopwatch_milliseconds = 0;
    lap_count = 0;
    sei();
}

void stopwatch_lap(void) {
    if (lap_count < 5) {
        cli();
        lap_times[lap_count].minutes = stopwatch_minutes;
        lap_times[lap_count].seconds = stopwatch_seconds;
        lap_times[lap_count].milliseconds = stopwatch_milliseconds;
        lap_count++;
        sei();
    }
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * PWM LED CONTROL (Timer0 PWM)
 * ═══════════════════════════════════════════════════════════════════════════
 */

uint8_t led_brightness = 128;  // 0-255

void pwm_init(void) {
    // Timer0 Fast PWM mode on OC0 (PB4)
    TCCR0 = (1 << WGM01) | (1 << WGM00) |  // Fast PWM
            (1 << COM01) |                  // Clear OC0 on compare match
            (1 << CS01);                    // Prescaler 8
    
    DDRB |= (1 << 4);  // OC0 as output
    OCR0 = led_brightness;
}

void set_led_brightness(uint8_t brightness) {
    led_brightness = brightness;
    OCR0 = brightness;
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * TASK SCHEDULER (Cooperative Multitasking)
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef struct {
    void (*function)(void);
    uint16_t interval_ms;
    uint16_t last_run;
    uint8_t enabled;
    const char *name;
    uint32_t run_count;
} Task;

void task_blink_led0(void) { PORTB ^= (1 << 0); }
void task_blink_led1(void) { PORTB ^= (1 << 1); }
void task_serial_heartbeat(void) { 
    if (current_mode == MODE_TASK_MONITOR) {
        putch_USART1('.');
        putch_USART1('\r');
    }
}

Task tasks[] = {
    {task_blink_led0, 500, 0, 1, "LED0 Blink", 0},
    {task_blink_led1, 1000, 0, 1, "LED1 Blink", 0},
    {task_serial_heartbeat, 2000, 0, 1, "Heartbeat", 0}
};

#define TASK_COUNT (sizeof(tasks) / sizeof(Task))

void scheduler_update(void) {
    uint16_t current_time = rtc_seconds * 1000 + (rtc_milliseconds % 1000);
    
    for (uint8_t i = 0; i < TASK_COUNT; i++) {
        if (tasks[i].enabled) {
            uint16_t elapsed = current_time - tasks[i].last_run;
            
            if (elapsed >= tasks[i].interval_ms) {
                tasks[i].function();
                tasks[i].last_run = current_time;
                tasks[i].run_count++;
            }
        }
    }
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * GRAPHICS LCD USER INTERFACE
 * ═══════════════════════════════════════════════════════════════════════════
 */

void draw_header(byte row, char *title) {
    // Draw title on first row
    lcd_string(row, 0, title);
}

void draw_clock_screen(void) {
    lcd_clear();
    
    // Header
    lcd_string(0, 0, "====================");
    lcd_string(1, 0, "  REAL-TIME CLOCK  ");
    lcd_string(2, 0, "====================");
    
    // Display time (row 3)
    char time_str[21];
    sprintf(time_str, "   %02u:%02u:%02u", rtc_hours, rtc_minutes, rtc_seconds);
    lcd_string(3, 0, time_str);
    
    // Instructions (bottom rows)
    lcd_string(6, 0, "MODE: Change Screen");
    lcd_string(7, 0, "SERIAL: Set Time");
}

void draw_stopwatch_screen(void) {
    lcd_clear();
    
    // Header
    lcd_string(0, 0, "====================");
    lcd_string(1, 0, "    STOPWATCH      ");
    lcd_string(2, 0, "====================");
    
    // Display stopwatch time
    char time_str[21];
    sprintf(time_str, " %02u:%02u.%03u", 
            stopwatch_minutes, stopwatch_seconds, stopwatch_milliseconds);
    lcd_string(3, 0, time_str);
    
    // Status
    lcd_string(4, 0, stopwatch_running ? "Status: RUNNING" : "Status: STOPPED");
    
    // Lap times
    if (lap_count > 0) {
        lcd_string(5, 0, "Lap Times:");
        
        for (uint8_t i = 0; i < lap_count && i < 2; i++) {
            char lap_str[21];
            sprintf(lap_str, "%u:%02u:%02u.%03u", 
                    i + 1,
                    lap_times[i].minutes, 
                    lap_times[i].seconds,
                    lap_times[i].milliseconds);
            lcd_string(6 + i, 0, lap_str);
        }
    }
}

void draw_led_control_screen(void) {
    lcd_clear();
    
    // Header
    lcd_string(0, 0, "====================");
    lcd_string(1, 0, " LED BRIGHTNESS    ");
    lcd_string(2, 0, "====================");
    
    // Brightness value
    char bright_str[21];
    sprintf(bright_str, "Level: %3u/255", led_brightness);
    lcd_string(3, 0, bright_str);
    
    // Percentage
    uint8_t percent = (led_brightness * 100) / 255;
    sprintf(bright_str, "  (%3u%%)", percent);
    lcd_string(4, 0, bright_str);
    
    // Bar graph using characters
    lcd_string(5, 0, "Bar:");
    uint8_t bars = (led_brightness * 15) / 255;  // 0-15 characters
    char bar_str[21] = "                ";
    for (uint8_t i = 0; i < bars; i++) {
        bar_str[i] = '#';
    }
    lcd_string(6, 0, bar_str);
    
    // Instructions
    lcd_string(7, 0, "+/- or 0-9 levels");
}

void draw_task_monitor_screen(void) {
    lcd_clear();
    
    // Header
    lcd_string(0, 0, "====================");
    lcd_string(1, 0, " TASK SCHEDULER    ");
    lcd_string(2, 0, "====================");
    
    lcd_string(3, 0, "Active Tasks:");
    
    for (uint8_t i = 0; i < TASK_COUNT && i < 3; i++) {
        char task_str[21];
        sprintf(task_str, "[%c] %s:%lu", 
                tasks[i].enabled ? 'X' : ' ',
                tasks[i].name, 
                tasks[i].run_count);
        lcd_string(4 + i, 0, task_str);
    }
    
    lcd_string(7, 0, "All tasks running");
}

void update_screen(void) {
    switch (current_mode) {
        case MODE_CLOCK:
            draw_clock_screen();
            break;
        case MODE_STOPWATCH:
            draw_stopwatch_screen();
            break;
        case MODE_LED_CONTROL:
            draw_led_control_screen();
            break;
        case MODE_TASK_MONITOR:
            draw_task_monitor_screen();
            break;
        default:
            break;
    }
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * BUTTON HANDLING
 * ═══════════════════════════════════════════════════════════════════════════
 */

void buttons_init(void) {
    // Configure as inputs with pull-ups
    DDRD &= ~((1 << BTN_START_STOP) | (1 << BTN_LAP_RESET) | (1 << BTN_MODE));
    PORTD |= (1 << BTN_START_STOP) | (1 << BTN_LAP_RESET) | (1 << BTN_MODE);
}

void handle_buttons(void) {
    // MODE button: Change screen
    if (button_pressed(BTN_MODE)) {
        current_mode = (current_mode + 1) % MODE_COUNT;
        mode_changed = 1;
        
        // Send mode change to serial
        char mode_msg[30];
        sprintf(mode_msg, "\r\nMode: %d\r\n", current_mode);
        puts_USART1(mode_msg);
    }
    
    // START/STOP button (mode-specific)
    if (button_pressed(BTN_START_STOP)) {
        switch (current_mode) {
            case MODE_STOPWATCH:
                if (stopwatch_running) {
                    stopwatch_stop();
                    char msg1[] = "STOP\r\n";
                    puts_USART1(msg1);
                } else {
                    stopwatch_start();
                    char msg2[] = "START\r\n";
                    puts_USART1(msg2);
                }
                mode_changed = 1;
                break;
            
            case MODE_LED_CONTROL:
                led_brightness += 25;
                if (led_brightness > 255) led_brightness = 0;
                set_led_brightness(led_brightness);
                mode_changed = 1;
                break;
            
            default:
                break;
        }
    }
    
    // LAP/RESET button (mode-specific)
    if (button_pressed(BTN_LAP_RESET)) {
        switch (current_mode) {
            case MODE_STOPWATCH:
                if (stopwatch_running) {
                    stopwatch_lap();
                    char msg3[] = "LAP!\r\n";
                    puts_USART1(msg3);
                } else {
                    stopwatch_reset();
                    char msg4[] = "RESET\r\n";
                    puts_USART1(msg4);
                }
                mode_changed = 1;
                break;
            
            case MODE_LED_CONTROL:
                set_led_brightness(0);
                mode_changed = 1;
                break;
            
            default:
                break;
        }
    }
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * SERIAL COMMAND INTERFACE
 * ═══════════════════════════════════════════════════════════════════════════
 */

void print_help(void) {
    char msg1[] = "\r\n=== TIMER DASHBOARD ===\r\n";
    char msg2[] = "Clock: Thh:mm:ss\r\n";
    char msg3[] = "Stopwatch: S/L/R\r\n";
    char msg4[] = "LED: +/- or 0-9\r\n";
    char msg5[] = "System: M (mode) ? (help)\r\n";
    char msg6[] = "=======================\r\n";
    puts_USART1(msg1);
    puts_USART1(msg2);
    puts_USART1(msg3);
    puts_USART1(msg4);
    puts_USART1(msg5);
    puts_USART1(msg6);
}

void handle_serial_command(char cmd) {
    char msg[30];
    
    switch (cmd) {
        case '?':
        case 'h':
        case 'H':
            print_help();
            break;
        
        case 'M':
        case 'm':
            current_mode = (current_mode + 1) % MODE_COUNT;
            mode_changed = 1;
            break;
        
        case 'S':
        case 's':
            if (stopwatch_running) {
                stopwatch_stop();
            } else {
                stopwatch_start();
            }
            mode_changed = 1;
            break;
        
        case 'L':
        case 'l':
            stopwatch_lap();
            mode_changed = 1;
            break;
        
        case 'R':
        case 'r':
            stopwatch_reset();
            mode_changed = 1;
            break;
        
        case '+':
            if (led_brightness < 245) led_brightness += 10;
            else led_brightness = 255;
            set_led_brightness(led_brightness);
            mode_changed = 1;
            break;
        
        case '-':
            if (led_brightness > 10) led_brightness -= 10;
            else led_brightness = 0;
            set_led_brightness(led_brightness);
            mode_changed = 1;
            break;
        
        case '0' ... '9':
            led_brightness = ((cmd - '0') * 255) / 9;
            set_led_brightness(led_brightness);
            mode_changed = 1;
            break;
        
        case 'T':
        case 't':
            // Time setting command (format: Thh:mm:ss)
            sprintf(msg, "Enter time hh:mm:ss: ");
            puts_USART1(msg);
            break;
        
        default:
            break;
    }
}

/*
 * ═══════════════════════════════════════════════════════════════════════════
 * MAIN PROGRAM
 * ═══════════════════════════════════════════════════════════════════════════
 */

int main(void) {
    // Initialize all subsystems
    init_devices();       // Basic I/O
    Uart1_init();        // Serial communication
    lcd_init();          // Graphics LCD
    buttons_init();      // Push buttons
    
    // Initialize timers
    rtc_init();          // Real-time clock (Timer2)
    stopwatch_init();    // Stopwatch (Timer1)
    pwm_init();          // PWM LED control (Timer0)
    
    // Configure LEDs
    DDRB = 0xFF;
    PORTB = 0xFF;
    
    sei();  // Enable global interrupts
    
    _delay_ms(100);
    
    // Welcome screen
    lcd_clear();
    lcd_string(0, 0, "====================");
    lcd_string(2, 0, "      TIMER");
    lcd_string(3, 0, "    DASHBOARD");
    lcd_string(5, 0, "  ATmega128 Lab");
    lcd_string(6, 0, "    Loading...");
    lcd_string(7, 0, "====================");
    
    // Serial welcome
    char msg1[] = "\r\n\r\n";
    char msg2[] = "====================================\r\n";
    char msg3[] = " INTEGRATED TIMER SYSTEM DASHBOARD\r\n";
    char msg4[] = "    ATmega128 Comprehensive Lab\r\n";
    char msg5[] = "====================================\r\n";
    char msg6[] = "System initialized!\r\n";
    char msg7[] = "- RTC (Timer2 + ISR)\r\n";
    char msg8[] = "- Stopwatch (Timer1 + ISR)\r\n";
    char msg9[] = "- PWM LED (Timer0)\r\n";
    char msg10[] = "- Graphics LCD Interface\r\n";
    char msg11[] = "- Task Scheduler\r\n";
    char msg12[] = "Type ? for help\r\n\r\n";
    
    puts_USART1(msg1);
    puts_USART1(msg2);
    puts_USART1(msg3);
    puts_USART1(msg4);
    puts_USART1(msg5);
    puts_USART1(msg6);
    puts_USART1(msg7);
    puts_USART1(msg8);
    puts_USART1(msg9);
    puts_USART1(msg10);
    puts_USART1(msg11);
    puts_USART1(msg12);
    
    _delay_ms(2000);
    
    // Initial screen
    update_screen();
    
    // Main loop: Integrate all systems
    uint16_t refresh_counter = 0;
    
    while (1) {
        // 1. Update task scheduler (cooperative multitasking)
        scheduler_update();
        
        // 2. Handle button inputs
        handle_buttons();
        
        // 3. Handle serial commands
        if (UCSR1A & (1 << RXC1)) {
            char cmd = UDR1;
            putch_USART1(cmd);  // Echo
            handle_serial_command(cmd);
        }
        
        // 4. Refresh LCD screen periodically or on mode change
        refresh_counter++;
        if (mode_changed || (refresh_counter >= 1000)) {
            update_screen();
            refresh_counter = 0;
            mode_changed = 0;
        }
        
        // Small delay for stability
        _delay_ms(1);
    }
    
    return 0;
}
