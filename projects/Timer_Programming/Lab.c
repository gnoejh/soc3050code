/*
 * =============================================================================
 * TIMER/COUNTER PROGRAMMING - HANDS-ON LAB EXERCISES
 * =============================================================================
 *
 * PROJECT: Timer_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Interactive laboratory exercises for hands-on experience with ATmega128 timers.
 * Students practice timing control through guided exercises and real-time challenges.
 *
 * LAB OBJECTIVES:
 * 1. Measure frequencies with input capture
 * 2. Generate PWM signals for LED dimming
 * 3. Create precision event schedulers
 * 4. Build digital stopwatch applications
 * 5. Implement tone generation for audio feedback
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - LEDs on PORTB for visual feedback
 * - Buzzer on PC0 for audio generation
 * - Push button on PD4 for stopwatch control
 * - Optional: Oscilloscope for frequency verification
 * - Serial terminal for interaction (9600 baud)
 *
 * LAB STRUCTURE:
 * - Exercise 1: Timer Configuration Practice
 * - Exercise 2: Precision Timing Challenges
 * - Exercise 3: PWM LED Dimmer
 * - Exercise 4: Digital Stopwatch
 * - Exercise 5: Musical Tone Generator
 *
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate to Advanced
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define BUZZER_PIN 0 // PC0

// Global variables
uint16_t lab_score = 0;
volatile uint32_t milliseconds = 0;
volatile uint16_t timer_overflows = 0;

/*
 * =============================================================================
 * TIMER SETUP HELPERS
 * =============================================================================
 */

void timer0_init_normal(void)
{
    // Timer0: Normal mode, prescaler 64
    TCCR0 = (1 << CS01) | (1 << CS00); // Prescaler 64
    TCNT0 = 0;
}

void timer0_init_ctc(uint8_t compare_value)
{
    // Timer0: CTC mode, prescaler 64
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0 = compare_value;
    TCNT0 = 0;
}

void timer1_init_pwm(void)
{
    // Timer1: Fast PWM, 8-bit, prescaler 64
    // PWM on OC1A (PB5)
    TCCR1A = (1 << WGM10) | (1 << COM1A1);             // 8-bit Fast PWM, non-inverting
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler 64
    OCR1A = 0;

    // Set PB5 as output
    DDRB |= (1 << 5);
}

void timer2_init_overflow_interrupt(void)
{
    // Timer2: Normal mode with overflow interrupt
    TCCR2 = (1 << CS22) | (1 << CS21); // Prescaler 256
    TIMSK |= (1 << TOIE2);             // Enable overflow interrupt
    TCNT2 = 0;

    sei(); // Enable global interrupts
}

// Timer2 overflow ISR for millisecond counter
ISR(TIMER2_OVF_vect)
{
    timer_overflows++;
    // At 7.3728 MHz with prescaler 256:
    // Overflow every 256 counts = ~8.8ms
    // Approximate milliseconds (not precise)
    milliseconds += 9;
}

/*
 * =============================================================================
 * LAB EXERCISE 1: TIMING BASICS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Understand timer modes and timing calculation
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_led_blink_timer(void)
{
    /*
     * CHALLENGE: Blink LED using timer polling
     * TASK: Toggle LED every 500ms using Timer0
     * LEARNING: Timer overflow detection, timing calculation
     */

    puts_USART1("\r\n=== Lab 1.1: LED Blink with Timer ===\r\n");
    puts_USART1("Blinking LED on PB0 using Timer0\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    // Configure LED
    DDRB |= (1 << 0);
    PORTB |= (1 << 0); // Off (active low)

    timer0_init_normal();

    uint16_t overflow_count = 0;
    uint16_t blinks = 0;

    while (1)
    {
        // Check for timer overflow (TIFR bit TOV0)
        if (TIFR & (1 << TOV0))
        {
            TIFR |= (1 << TOV0); // Clear flag
            overflow_count++;

            // At 7.3728 MHz, prescaler 64, 8-bit timer:
            // Overflow every 256 counts = ~2.22ms
            // Need ~225 overflows for 500ms

            if (overflow_count >= 225)
            {
                overflow_count = 0;
                PORTB ^= (1 << 0); // Toggle LED
                blinks++;

                char msg[30];
                sprintf(msg, "Blink %u\r", blinks);
                puts_USART1(msg);
            }
        }

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    PORTB |= (1 << 0); // LED off

    char summary[50];
    sprintf(summary, "\r\nTotal blinks: %u\r\n", blinks);
    puts_USART1(summary);

    lab_score += 75;
}

void lab_ex1_stopwatch(void)
{
    /*
     * CHALLENGE: Digital stopwatch
     * TASK: Count seconds/minutes using timer
     * LEARNING: Time accumulation, formatting
     */

    puts_USART1("\r\n=== Lab 1.2: Digital Stopwatch ===\r\n");
    puts_USART1("Press button (PD4) to start/stop\r\n");
    puts_USART1("Press 'R' to reset, 'Q' to quit\r\n\r\n");

    // Configure button
    DDRD &= ~(1 << 4);
    PORTD |= (1 << 4); // Pull-up

    timer0_init_normal();

    uint8_t running = 0;
    uint16_t overflow_count = 0;
    uint32_t total_ms = 0;
    uint8_t last_button = 1;

    while (1)
    {
        // Button debouncing and detection
        uint8_t button = (PIND & (1 << 4)) ? 1 : 0;

        if (button == 0 && last_button == 1) // Button pressed
        {
            running = !running;
            _delay_ms(200); // Debounce
        }

        last_button = button;

        // Timer counting
        if (running && (TIFR & (1 << TOV0)))
        {
            TIFR |= (1 << TOV0);
            overflow_count++;

            if (overflow_count >= 450) // ~1 second
            {
                overflow_count = 0;
                total_ms += 1000;

                // Display time
                uint16_t seconds = (total_ms / 1000) % 60;
                uint16_t minutes = (total_ms / 60000) % 60;
                uint16_t hours = total_ms / 3600000;

                char time_str[50];
                sprintf(time_str, "\r%02u:%02u:%02u %s",
                        hours, minutes, seconds, running ? "RUN " : "STOP");
                puts_USART1(time_str);
            }
        }

        // Check for commands
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;

            if (c == 'R' || c == 'r')
            {
                total_ms = 0;
                overflow_count = 0;
                puts_USART1("\r\nReset!\r\n");
            }
            else if (c == 'Q' || c == 'q')
            {
                break;
            }
        }
    }

    puts_USART1("\r\n\r\nStopwatch complete!\r\n");

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: PWM GENERATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Generate and control PWM signals
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_led_dimmer(void)
{
    /*
     * CHALLENGE: LED brightness control with PWM
     * TASK: Fade LED in and out smoothly
     * LEARNING: PWM duty cycle, Timer1 configuration
     */

    puts_USART1("\r\n=== Lab 2.1: LED Dimmer (PWM) ===\r\n");
    puts_USART1("Fading LED on PB5 using Timer1 PWM\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    timer1_init_pwm();

    uint8_t cycles = 0;

    while (cycles < 10)
    {
        // Fade in (0 -> 255)
        for (uint16_t brightness = 0; brightness < 256; brightness++)
        {
            OCR1A = brightness;
            _delay_ms(5);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                goto dimmer_exit;
            }
        }

        // Fade out (255 -> 0)
        for (uint16_t brightness = 255; brightness > 0; brightness--)
        {
            OCR1A = brightness;
            _delay_ms(5);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                goto dimmer_exit;
            }
        }

        cycles++;
        char msg[30];
        sprintf(msg, "Cycle %u/10\r", cycles);
        puts_USART1(msg);
    }

dimmer_exit:
    OCR1A = 0; // LED off

    puts_USART1("\r\nLED dimming complete!\r\n");

    lab_score += 100;
}

void lab_ex2_manual_brightness(void)
{
    /*
     * CHALLENGE: Interactive brightness control
     * TASK: Adjust LED brightness with serial commands
     * LEARNING: PWM control, user interface
     */

    puts_USART1("\r\n=== Lab 2.2: Manual Brightness Control ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  +/- : Increase/Decrease brightness\r\n");
    puts_USART1("  0-9 : Set brightness level (0=off, 9=max)\r\n");
    puts_USART1("  Q   : Quit\r\n\r\n");

    timer1_init_pwm();

    uint8_t brightness = 128;
    OCR1A = brightness;

    while (1)
    {
        char msg[60];
        uint8_t percent = (brightness * 100) / 255;
        sprintf(msg, "\rBrightness: %3u/255 (%3u%%) [", brightness, percent);
        puts_USART1(msg);

        // Bar graph
        uint8_t bars = brightness / 13; // 0-19
        for (uint8_t i = 0; i < 20; i++)
        {
            putch_USART1(i < bars ? '█' : '░');
        }
        puts_USART1("]");

        // Wait for command
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;

            if (c == '+' && brightness < 255)
            {
                brightness += 10;
                if (brightness > 255)
                    brightness = 255;
            }
            else if (c == '-' && brightness > 0)
            {
                brightness -= 10;
            }
            else if (c >= '0' && c <= '9')
            {
                brightness = ((c - '0') * 255) / 9;
            }
            else if (c == 'Q' || c == 'q')
            {
                break;
            }

            OCR1A = brightness;
        }

        _delay_ms(50);
    }

    OCR1A = 0;

    puts_USART1("\r\n\r\nBrightness control complete!\r\n");

    lab_score += 75;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: TONE GENERATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Generate audio frequencies
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void play_tone(uint16_t frequency, uint16_t duration_ms)
{
    /*
     * Generate square wave tone using timer CTC mode
     * Frequency = F_CPU / (2 * prescaler * (1 + OCR0))
     */

    if (frequency == 0)
        return;

    // Calculate OCR value
    // For prescaler 64: OCR = (F_CPU / (2 * 64 * freq)) - 1
    uint8_t ocr_value = (F_CPU / (2 * 64 * frequency)) - 1;

    timer0_init_ctc(ocr_value);

    // Configure buzzer pin
    DDRC |= (1 << BUZZER_PIN);

    // Toggle buzzer
    uint32_t toggles = (uint32_t)frequency * 2 * duration_ms / 1000;

    for (uint32_t i = 0; i < toggles; i++)
    {
        // Wait for compare match
        while (!(TIFR & (1 << OCF0)))
            ;
        TIFR |= (1 << OCF0); // Clear flag

        PORTC ^= (1 << BUZZER_PIN); // Toggle buzzer
    }

    PORTC &= ~(1 << BUZZER_PIN); // Buzzer off
}

void lab_ex3_musical_notes(void)
{
    /*
     * CHALLENGE: Play musical scale
     * TASK: Generate notes C4 through C5
     * LEARNING: Frequency generation, musical notes
     */

    puts_USART1("\r\n=== Lab 3.1: Musical Notes ===\r\n");
    puts_USART1("Playing C major scale...\r\n\r\n");

    // Musical notes (middle C octave)
    uint16_t notes[] = {
        262, // C4
        294, // D4
        330, // E4
        349, // F4
        392, // G4
        440, // A4
        494, // B4
        523  // C5
    };

    const char *note_names[] = {"C", "D", "E", "F", "G", "A", "B", "C"};

    for (uint8_t i = 0; i < 8; i++)
    {
        char msg[40];
        sprintf(msg, "Playing %s4: %u Hz\r\n", note_names[i], notes[i]);
        puts_USART1(msg);

        play_tone(notes[i], 500); // 500ms per note
        _delay_ms(100);           // Gap between notes
    }

    puts_USART1("\r\nScale complete!\r\n");

    lab_score += 100;
}

void lab_ex3_melody_player(void)
{
    /*
     * CHALLENGE: Play a simple melody
     * TASK: Play "Twinkle Twinkle Little Star"
     * LEARNING: Rhythm, melody programming
     */

    puts_USART1("\r\n=== Lab 3.2: Melody Player ===\r\n");
    puts_USART1("Playing 'Twinkle Twinkle Little Star'\r\n\r\n");

// Note definitions
#define C4 262
#define D4 294
#define E4 330
#define F4 349
#define G4 392
#define A4 440
#define REST 0

    // Melody: Twinkle Twinkle
    uint16_t melody[] = {
        C4, C4, G4, G4, A4, A4, G4, REST,
        F4, F4, E4, E4, D4, D4, C4, REST};

    uint16_t durations[] = {
        400, 400, 400, 400, 400, 400, 800, 200,
        400, 400, 400, 400, 400, 400, 800, 200};

    for (uint8_t i = 0; i < 16; i++)
    {
        if (melody[i] != REST)
        {
            play_tone(melody[i], durations[i]);
        }
        else
        {
            _delay_ms(durations[i]);
        }

        _delay_ms(50); // Note separation
    }

    puts_USART1("Melody complete!\r\n");

    lab_score += 125;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: EVENT SCHEDULING (25 minutes)
 * =============================================================================
 * OBJECTIVE: Build multi-task timer system
 * DIFFICULTY: ★★★★★ (Expert)
 */

typedef struct
{
    uint32_t interval_ms;
    uint32_t last_run_ms;
    void (*function)(void);
    uint8_t enabled;
} ScheduledTask;

void task_blink_led(void)
{
    PORTB ^= (1 << 0);
}

void task_print_time(void)
{
    char msg[40];
    sprintf(msg, "[%lu ms] Heartbeat\r\n", milliseconds);
    puts_USART1(msg);
}

void task_toggle_led2(void)
{
    PORTB ^= (1 << 1);
}

void lab_ex4_task_scheduler(void)
{
    /*
     * CHALLENGE: Multi-task event scheduler
     * TASK: Run multiple tasks at different intervals
     * LEARNING: Task scheduling, cooperative multitasking
     */

    puts_USART1("\r\n=== Lab 4.1: Task Scheduler ===\r\n");
    puts_USART1("Running 3 tasks at different intervals\r\n");
    puts_USART1("Task 1: Blink LED0 every 500ms\r\n");
    puts_USART1("Task 2: Print message every 2s\r\n");
    puts_USART1("Task 3: Blink LED1 every 1s\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    // Configure LEDs
    DDRB |= (1 << 0) | (1 << 1);
    PORTB |= (1 << 0) | (1 << 1);

    // Initialize timer for millisecond tracking
    timer2_init_overflow_interrupt();
    milliseconds = 0;

    // Define tasks
    ScheduledTask tasks[3] = {
        {500, 0, task_blink_led, 1},
        {2000, 0, task_print_time, 1},
        {1000, 0, task_toggle_led2, 1}};

    uint32_t runtime = 0;

    while (runtime < 30000) // Run for 30 seconds
    {
        // Check each task
        for (uint8_t i = 0; i < 3; i++)
        {
            if (tasks[i].enabled)
            {
                if ((milliseconds - tasks[i].last_run_ms) >= tasks[i].interval_ms)
                {
                    tasks[i].function();
                    tasks[i].last_run_ms = milliseconds;
                }
            }
        }

        // Update runtime
        runtime = milliseconds;

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    // Disable timer interrupt
    TIMSK &= ~(1 << TOIE2);
    PORTB |= (1 << 0) | (1 << 1); // LEDs off

    puts_USART1("\r\nTask scheduler complete!\r\n");

    lab_score += 150;
}

/*
 * =============================================================================
 * LAB MENU SYSTEM
 * =============================================================================
 */

void print_lab_menu(void)
{
    puts_USART1("\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("  TIMER PROGRAMMING - LAB EXERCISES\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Timing Basics\r\n");
    puts_USART1("  1. LED Blink with Timer\r\n");
    puts_USART1("  2. Digital Stopwatch\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: PWM Generation\r\n");
    puts_USART1("  3. LED Dimmer (PWM)\r\n");
    puts_USART1("  4. Manual Brightness Control\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Tone Generation\r\n");
    puts_USART1("  5. Musical Notes\r\n");
    puts_USART1("  6. Melody Player\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 4: Event Scheduling\r\n");
    puts_USART1("  7. Task Scheduler\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    char score_str[40];
    sprintf(score_str, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(score_str);
    puts_USART1("Select exercise (1-7, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 TIMER PROGRAMMING LAB             *\r\n");
    puts_USART1("*  Hands-On Timer/Counter Exercises            *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Timer Programming Lab!\r\n");
    puts_USART1("Master timers through practical exercises.\r\n");

    while (1)
    {
        print_lab_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\r');
        putch_USART1('\n');

        switch (choice)
        {
        case '1':
            lab_ex1_led_blink_timer();
            break;
        case '2':
            lab_ex1_stopwatch();
            break;
        case '3':
            lab_ex2_led_dimmer();
            break;
        case '4':
            lab_ex2_manual_brightness();
            break;
        case '5':
            lab_ex3_musical_notes();
            break;
        case '6':
            lab_ex3_melody_player();
            break;
        case '7':
            lab_ex4_task_scheduler();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_led_blink_timer();
            lab_ex1_stopwatch();
            lab_ex2_led_dimmer();
            lab_ex2_manual_brightness();
            lab_ex3_musical_notes();
            lab_ex3_melody_player();
            lab_ex4_task_scheduler();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\r\n");
        }

        puts_USART1("\r\nPress any key to continue...\r\n");
        getch_USART1();
    }

    return 0;
}
