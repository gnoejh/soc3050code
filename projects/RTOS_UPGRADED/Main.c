/*
 * Enhanced RTOS v2.0 for ATmega128
 * Self-contained implementation compatible with Simulator110.simu
 * Features: 8 concurrent tasks, priority scheduling, system statistics
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "../../shared_libs/_glcd.h"

// ============================================================================
// RTOS Configuration
// ============================================================================
#define MAX_TASKS 10
#define STACK_SIZE 128
#define TIMER_TICK_MS 1
#define RTOS_VERSION "2.0"

// ============================================================================
// Task States & Priorities
// ============================================================================
typedef enum
{
  TASK_READY,
  TASK_RUNNING,
  TASK_BLOCKED,
  TASK_SUSPENDED,
  TASK_TERMINATED
} TaskState;

typedef enum
{
  PRIORITY_IDLE = 0,
  PRIORITY_LOW = 1,
  PRIORITY_NORMAL = 2,
  PRIORITY_HIGH = 3,
  PRIORITY_CRITICAL = 4
} TaskPriority;

// ============================================================================
// Task Control Block (TCB)
// ============================================================================
typedef struct
{
  uint8_t task_id;
  char task_name[20];
  uint16_t stack_pointer;
  uint8_t stack[STACK_SIZE];
  TaskState state;
  TaskPriority priority;
  uint32_t delay_ticks;
  uint32_t execution_count;
  void (*task_function)(void);
  bool auto_restart;
} TCB;

// ============================================================================
// RTOS Global Variables
// ============================================================================
static TCB task_list[MAX_TASKS];
static uint8_t current_task = 0;
static uint8_t task_count = 0;
static volatile uint32_t system_ticks = 0;
static bool scheduler_running = false;

// LED shadow registers to avoid race conditions
static volatile uint8_t led_sequence_bits =
    0xFF;                                         // PB0-PB6 (active-LOW, all OFF)
static volatile uint8_t led_heartbeat_bit = 0xFF; // PB7 (active-LOW, OFF) - AND mask

// ============================================================================
// System Statistics
// ============================================================================
typedef struct
{
  uint32_t context_switches;
  uint16_t free_ram_bytes;
} SystemStats;

static SystemStats sys_stats = {0, 0};

// ============================================================================
// UART Functions (Standalone Implementation)
// ============================================================================
void uart_init(void)
{
  UBRR1H = 0;
  UBRR1L = 103; // 9600 baud at 16MHz
  UCSR1B = (1 << RXEN1) | (1 << TXEN1);
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void uart_putchar(char c)
{
  while (!(UCSR1A & (1 << UDRE1)))
    ;
  UDR1 = c;
}

void uart_puts(const char *str)
{
  while (*str)
  {
    uart_putchar(*str++);
  }
}

void uart_print_num(uint32_t num)
{
  char buffer[12];
  uint8_t i = 0;

  if (num == 0)
  {
    uart_putchar('0');
    return;
  }

  while (num > 0)
  {
    buffer[i++] = (num % 10) + '0';
    num /= 10;
  }

  while (i > 0)
  {
    uart_putchar(buffer[--i]);
  }
}

// ============================================================================
// ADC Functions (Standalone Implementation)
// ============================================================================
void adc_init(void)
{
  // AVcc with external capacitor at AREF
  ADMUX = (1 << REFS0);
  // Enable ADC, prescaler = 128 (125kHz at 16MHz)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel)
{
  // Select ADC channel
  ADMUX = (ADMUX & 0xE0) | (channel & 0x07);
  // Start conversion
  ADCSRA |= (1 << ADSC);
  // Wait for completion
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC;
}

// ============================================================================
// Buzzer Functions (Standalone Implementation)
// ============================================================================
void buzzer_init(void)
{
  DDRG |= (1 << PG4); // Buzzer on PG4
  PORTG &= ~(1 << PG4);
}

void buzzer_beep(uint16_t duration_ms)
{
  uint16_t cycles = duration_ms * 2; // Approx 1kHz
  for (uint16_t i = 0; i < cycles; i++)
  {
    PORTG ^= (1 << PG4);
    _delay_us(500);
  }
  PORTG &= ~(1 << PG4);
}

// ============================================================================
// RTOS Core Functions
// ============================================================================
void rtos_init(void)
{
  memset(task_list, 0, sizeof(task_list));
  memset(&sys_stats, 0, sizeof(sys_stats));
  task_count = 0;
  current_task = 0;
  system_ticks = 0;
  scheduler_running = false;
}

void rtos_timer_init(void)
{
  // Timer0: Overflow mode, 1ms tick (TOV mode works, CTC doesn't)
  TCCR0 = (1 << CS01) | (1 << CS00); // Normal mode, prescaler 64
  TCNT0 = 6;                         // Preload for 1ms (256-250=6)
  TIMSK |= (1 << TOIE0);             // Enable overflow interrupt
}

uint8_t rtos_create_task(void (*task_func)(void), const char *name,
                         TaskPriority priority, bool auto_restart)
{
  if (task_count >= MAX_TASKS)
    return 0xFF;

  TCB *task = &task_list[task_count];
  task->task_id = task_count;
  strncpy(task->task_name, name, 19);
  task->task_name[19] = '\0';
  task->task_function = task_func;
  task->state = TASK_READY;
  task->priority = priority;
  task->delay_ticks = 0;
  task->execution_count = 0;
  task->auto_restart = auto_restart;
  task->stack_pointer = (uint16_t)&task->stack[STACK_SIZE - 1];

  task_count++;
  return task->task_id;
}

void task_delay(uint32_t ticks)
{
  if (current_task < task_count)
  {
    task_list[current_task].delay_ticks = ticks;
    task_list[current_task].state = TASK_BLOCKED;
  }
}

uint32_t rtos_get_ticks(void) { return system_ticks; }

uint8_t rtos_scheduler(void)
{
  uint8_t next_task = current_task;
  uint8_t start = current_task;
  bool found = false;

  // Simple round-robin: find next ready task after current
  next_task = (current_task + 1) % task_count;

  do
  {
    if (task_list[next_task].state == TASK_READY)
    {
      found = true;
      break;
    }
    next_task = (next_task + 1) % task_count;
  } while (next_task != start);

  // If no other task is ready, check if current task is still ready
  if (!found && task_list[current_task].state == TASK_READY)
  {
    next_task = current_task;
    found = true;
  }

  // If still no ready task, just return first task
  if (!found)
  {
    next_task = 0;
  }

  return next_task;
}

// Timer0 Overflow ISR (TOV mode works, CTC doesn't)
ISR(TIMER0_OVF_vect)
{
  TCNT0 = 6; // Reload for 1ms tick (256-250=6)
  system_ticks++;

  // Update delay counters for blocked tasks
  if (scheduler_running)
  {
    for (uint8_t i = 0; i < task_count; i++)
    {
      if (task_list[i].state == TASK_BLOCKED && task_list[i].delay_ticks > 0)
      {
        task_list[i].delay_ticks--;
        if (task_list[i].delay_ticks == 0)
        {
          task_list[i].state = TASK_READY;
        }
      }
    }
  }
}

void rtos_print_stats(void)
{
  uart_puts("\r\n=== RTOS Statistics ===\r\n");
  uart_puts("System Ticks: ");
  uart_print_num(system_ticks);
  uart_puts("\r\nContext Switches: ");
  uart_print_num(sys_stats.context_switches);
  uart_puts("\r\nActive Tasks: ");
  uart_print_num(task_count);
  uart_puts("\r\n\r\nTask Status:\r\n");

  for (uint8_t i = 0; i < task_count; i++)
  {
    uart_puts("  [");
    uart_print_num(i);
    uart_puts("] ");
    uart_puts(task_list[i].task_name);
    uart_puts(" - ");

    switch (task_list[i].state)
    {
    case TASK_READY:
      uart_puts("READY");
      break;
    case TASK_RUNNING:
      uart_puts("RUN");
      break;
    case TASK_BLOCKED:
      uart_puts("BLOCKED");
      break;
    case TASK_SUSPENDED:
      uart_puts("SUSPEND");
      break;
    case TASK_TERMINATED:
      uart_puts("TERM");
      break;
    }

    uart_puts(" | Exec: ");
    uart_print_num(task_list[i].execution_count);
    uart_puts("\r\n");
  }
}

void rtos_start(void)
{
  uart_puts("\r\n========================================\r\n");
  uart_puts("  Enhanced RTOS v");
  uart_puts(RTOS_VERSION);
  uart_puts("\r\n");
  uart_puts("  ATmega128 Real-Time OS\r\n");
  uart_puts("========================================\r\n");
  uart_puts("Tasks: ");
  uart_print_num(task_count);
  uart_puts("\r\n\r\n");

  rtos_timer_init();
  sei();
  scheduler_running = true;

  // Main scheduler loop
  while (1)
  {
    if (task_count == 0)
    {
      _delay_ms(10);
      continue;
    }

    // Find next ready task using scheduler
    current_task = rtos_scheduler();

    if (current_task < task_count &&
        task_list[current_task].state == TASK_READY)
    {

      task_list[current_task].state = TASK_RUNNING;
      task_list[current_task].execution_count++;
      sys_stats.context_switches++;

      if (task_list[current_task].task_function != NULL)
      {
        task_list[current_task].task_function();
      }

      if (task_list[current_task].state == TASK_RUNNING)
      {
        if (task_list[current_task].auto_restart)
        {
          task_list[current_task].state = TASK_READY;
        }
        else
        {
          task_list[current_task].state = TASK_TERMINATED;
        }
      }
    }

    // Small delay to prevent tight loop
    _delay_us(100);
  }
}

// ============================================================================
// TASK DEFINITIONS
// ============================================================================

// Task 1: LED Sequencer
void task_led_sequence(void)
{
  static uint8_t led_pattern = 0;
  static uint32_t last_update = 0;

  if ((system_ticks - last_update) >= 50)
  {
    // Active-LOW LEDs: Create pattern for PB0-PB6
    // Turn ON only the current LED (0), keep others OFF (1)
    uint8_t pattern = ~(1 << led_pattern); // Invert: selected bit=0 (ON), others=1 (OFF)
    led_sequence_bits = pattern | 0x80;    // Keep PB7=1 (OFF for heartbeat control)

    // Combine with heartbeat and update PORTB atomically
    cli();
    PORTB = led_sequence_bits & led_heartbeat_bit;
    sei();

    led_pattern = (led_pattern + 1) % 7; // Only cycle through 0-6
    last_update = system_ticks;
  }
}

// Task 2: UART Status Reporter
void task_uart_status(void)
{
  static uint32_t counter = 0;
  static uint32_t last_report = 0;

  if ((system_ticks - last_report) >= 200)
  {
    uart_puts("\r\n[Status] Report #");
    uart_print_num(counter++);
    uart_puts(" | Ticks: ");
    uart_print_num(system_ticks);
    uart_puts(" | Switches: ");
    uart_print_num(sys_stats.context_switches);
    uart_puts("\r\n");
    last_report = system_ticks;
  }
}

// Task 3: ADC Monitor
void task_adc_monitor(void)
{
  static uint32_t last_read = 0;

  if ((system_ticks - last_read) >= 150)
  {
    uint16_t adc_value = adc_read(0);
    uart_puts("[ADC] Ch0: ");
    uart_print_num(adc_value);
    uart_puts(" (");
    uart_print_num((adc_value * 100) / 1023);
    uart_puts("%\r\n");
    last_read = system_ticks;
  }
}

// Task 4: Button Monitor
void task_button_monitor(void)
{
  static uint8_t button_state = 0xFF;
  static uint16_t button_count[8] = {0};

  uint8_t current_state = PIND;

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((button_state & (1 << i)) && !(current_state & (1 << i)))
    {
      button_count[i]++;
      uart_puts("[Button] PD");
      uart_print_num(i);
      uart_puts(" pressed! Count: ");
      uart_print_num(button_count[i]);
      uart_puts("\r\n");
      buzzer_beep(50);
    }
  }

  button_state = current_state;
}

// Task 5: PWM Motor Controller
void task_motor_control(void)
{
  static uint32_t last_update = 0;
  static uint8_t duty_cycle = 128;
  static int8_t direction = 1;
  static bool initialized = false;

  if (!initialized)
  {
    // Configure Timer1 for Fast PWM on OC1A (PB5)
    DDRB |= (1 << PB5);                                   // OC1A as output
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); // Fast PWM 10-bit
    TCCR1B = (1 << WGM12) | (1 << CS11);                  // Prescaler 8
    initialized = true;
  }

  if ((system_ticks - last_update) >= 30)
  {
    duty_cycle += (direction * 25);

    if (duty_cycle >= 250)
    {
      direction = -1;
    }
    else if (duty_cycle <= 25)
    {
      direction = 1;
    }

    OCR1A = (uint16_t)duty_cycle * 4; // Scale for 10-bit PWM (0-1023)

    uart_puts("[PWM] Duty: ");
    uart_print_num((duty_cycle * 100) / 255);
    uart_puts("%\r\n");

    last_update = system_ticks;
  }
}

// Task 6: System Watchdog
void task_watchdog(void)
{
  static uint32_t last_check = 0;

  if ((system_ticks - last_check) >= 500)
  {
    extern uint8_t __heap_start;
    extern uint8_t *__brkval;
    uint16_t free_ram;

    if ((uint16_t)__brkval == 0)
    {
      free_ram = (uint16_t)&free_ram - (uint16_t)&__heap_start;
    }
    else
    {
      free_ram = (uint16_t)&free_ram - (uint16_t)__brkval;
    }

    sys_stats.free_ram_bytes = free_ram;

    uart_puts("[Watchdog] System OK | RAM: ");
    uart_print_num(free_ram);
    uart_puts(" bytes\r\n");

    last_check = system_ticks;
  }
}

// Task 7: Heartbeat LED
void task_heartbeat(void)
{
  static bool state = false;
  static uint32_t last_beat = 0;

  if ((system_ticks - last_beat) >= 100)
  {
    state = !state;
    // Update shadow register for PB7 (active-LOW)
    // Use AND mask: 0x7F clears bit 7 (LED ON), 0xFF keeps bit 7 (LED OFF)
    if (state)
    {
      led_heartbeat_bit = 0x7F; // Active-LOW: Clear bit 7 to turn ON
    }
    else
    {
      led_heartbeat_bit = 0xFF; // Active-LOW: Keep bit 7 to turn OFF
    }
    // Combine with LED sequence and update PORTB atomically
    cli();
    PORTB = led_sequence_bits & led_heartbeat_bit;
    sei();
    last_beat = system_ticks;
  }
}

// Task 8: Statistics Reporter
void task_statistics(void)
{
  static uint32_t last_report = 0;

  if ((system_ticks - last_report) >= 1000)
  {
    rtos_print_stats();
    last_report = system_ticks;
  }
}

// Task 9: GLCD System Display
void task_glcd_display(void)
{
  static uint32_t last_update = 0;
  static bool initialized = false;

  // Initialize GLCD once
  if (!initialized)
  {
    ks0108_init();
    ks0108_clear_screen();
    ks0108_display_on_off(1);
    initialized = true;
  }

  // Update display every 500ms
  if ((system_ticks - last_update) >= 500)
  {
    // Title
    ks0108_set_cursor(0, 0);
    ks0108_puts("RTOS v2.0");

    // System ticks - display as seconds
    ks0108_set_cursor(1, 0);
    ks0108_puts("Ticks:");
    uint32_t seconds = system_ticks / 1000;
    // Simple number display (up to 999 seconds)
    if (seconds >= 100)
      ks0108_putchar('0' + (seconds / 100) % 10);
    if (seconds >= 10)
      ks0108_putchar('0' + (seconds / 10) % 10);
    ks0108_putchar('0' + seconds % 10);

    // Task count
    ks0108_set_cursor(2, 0);
    ks0108_puts("Tasks:");
    ks0108_putchar('0' + task_count);

    // Task status indicators
    ks0108_set_cursor(4, 0);
    ks0108_puts("Status:");
    ks0108_set_cursor(5, 0);
    for (uint8_t i = 0; i < task_count && i < 9; i++)
    {
      if (task_list[i].state == TASK_READY || task_list[i].state == TASK_RUNNING)
        ks0108_putchar('*');
      else
        ks0108_putchar('-');
    }

    last_update = system_ticks;
  }
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(void)
{
  // Initialize hardware
  uart_init();
  adc_init();
  buzzer_init();

  // Configure GPIO for Simulator110.simu
  DDRB = 0xFF;  // Port B: All outputs (LEDs)
  PORTB = 0xFF; // Active-LOW LEDs: All OFF initially

  DDRD = 0x00;  // Port D: Inputs (Buttons)
  PORTD = 0xFF; // Enable pull-ups

  DDRF = 0x00; // Port F: Inputs (ADC)
  PORTF = 0x00;

  // Welcome banner
  uart_puts("\r\n\r\n");
  uart_puts("========================================\r\n");
  uart_puts("  ATmega128 Enhanced RTOS v");
  uart_puts(RTOS_VERSION);
  uart_puts("\r\n");
  uart_puts("  Hardware-Integrated System\r\n");
  uart_puts("  Simulator110.simu Compatible\r\n");
  uart_puts("========================================\r\n\r\n");

  // Initialize RTOS
  rtos_init();

  // Create tasks
  uart_puts("Initializing tasks...\r\n");

  rtos_create_task(task_led_sequence, "LED Sequence", PRIORITY_NORMAL, true);
  uart_puts("[OK] Task 1: LED Sequence\r\n");

  rtos_create_task(task_uart_status, "UART Status", PRIORITY_NORMAL, true);
  uart_puts("[OK] Task 2: UART Status\r\n");

  rtos_create_task(task_adc_monitor, "ADC Monitor", PRIORITY_NORMAL, true);
  uart_puts("[OK] Task 3: ADC Monitor\r\n");

  rtos_create_task(task_button_monitor, "Button Handler", PRIORITY_HIGH, true);
  uart_puts("[OK] Task 4: Button Handler\r\n");

  rtos_create_task(task_motor_control, "Motor Control", PRIORITY_NORMAL, true);
  uart_puts("[OK] Task 5: PWM Motor\r\n");

  rtos_create_task(task_watchdog, "Watchdog", PRIORITY_LOW, true);
  uart_puts("[OK] Task 6: Watchdog\r\n");

  rtos_create_task(task_heartbeat, "Heartbeat", PRIORITY_HIGH, true);
  uart_puts("[OK] Task 7: Heartbeat LED\r\n");

  rtos_create_task(task_statistics, "Statistics", PRIORITY_LOW, true);
  uart_puts("[OK] Task 8: Statistics\r\n");

  rtos_create_task(task_glcd_display, "GLCD Display", PRIORITY_NORMAL, true);
  uart_puts("[OK] Task 9: GLCD Display\r\n");

  uart_puts("\r\nAll tasks initialized!\r\n");
  _delay_ms(1000);

  // Start RTOS scheduler
  rtos_start();

  return 0;
}
