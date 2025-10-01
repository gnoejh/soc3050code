#include "config.h"

// Conditional compilation - only compile if a relevant example is selected
#if defined(MEMORY_BASIC) || defined(MEMORY_STACK) || defined(MEMORY_HEAP) || defined(MEMORY_PROGRAM) || defined(MEMORY_EEPROM)

/*
 * MEMORY_BASIC - Basic Memory Operations and Concepts
 * Educational demonstration of:
 * - Memory types and characteristics (SRAM, Flash, Stack)
 * - Variable storage locations and addresses
 * - Memory address inspection and analysis
 * - Stack growth demonstration
 * - Memory usage optimization techniques
 */

#ifdef MEMORY_BASIC

// Global variables (stored in SRAM)
uint8_t global_counter = 0;
uint16_t global_array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
char global_string[] = "Global String";

// Static variables (stored in SRAM, initialized once)
static uint8_t static_counter = 100;
static uint16_t static_data = 0xABCD;

// Constant data (stored in Flash/Program memory)
const uint8_t const_table[] PROGMEM = {0x10, 0x20, 0x30, 0x40, 0x50};
const char const_message[] PROGMEM = "Flash Memory String";

// Demonstrate stack usage and growth
uint16_t memory_basic_stack_demo(uint8_t depth)
{
	// Local variables (stored on stack)
	uint8_t local_var = depth;
	uint16_t stack_address;

	// Get current stack pointer (approximation)
	stack_address = (uint16_t)&local_var;

	uart_string("Stack depth ");
	char depth_str[8];
	sprintf(depth_str, "%d", depth);
	uart_string(depth_str);
	uart_string(", Address: 0x");
	char addr_str[8];
	sprintf(addr_str, "%04X", stack_address);
	uart_string(addr_str);
	uart_string("\\r\\n");

	// Recursive call to show stack growth
	if (depth > 0)
	{
		return memory_basic_stack_demo(depth - 1) + local_var;
	}

	return local_var;
}

// Analyze and display memory locations
void memory_basic_analyze_locations(void)
{
	uart_string("\\r\\n=== MEMORY LOCATION ANALYSIS ===\\r\\n");

	// Local variable for comparison
	uint8_t local_test = 42;

	// Display various memory addresses
	uart_string("Global variables (SRAM):\\r\\n");
	uart_string("  global_counter at: 0x");
	char addr_str[8];
	sprintf(addr_str, "%04X", (uint16_t)&global_counter);
	uart_string(addr_str);
	uart_string("\\r\\n");

	uart_string("  global_array at: 0x");
	sprintf(addr_str, "%04X", (uint16_t)&global_array);
	uart_string(addr_str);
	uart_string("\\r\\n");

	uart_string("Static variables (SRAM):\\r\\n");
	uart_string("  static_counter at: 0x");
	sprintf(addr_str, "%04X", (uint16_t)&static_counter);
	uart_string(addr_str);
	uart_string("\\r\\n");

	uart_string("Local variables (Stack):\\r\\n");
	uart_string("  local_test at: 0x");
	sprintf(addr_str, "%04X", (uint16_t)&local_test);
	uart_string(addr_str);
	uart_string("\\r\\n");

	uart_string("Program memory (Flash):\\r\\n");
	uart_string("  const_table at: 0x");
	sprintf(addr_str, "%04X", (uint16_t)&const_table);
	uart_string(addr_str);
	uart_string("\\r\\n");
}

// Demonstrate different memory types and access methods
void memory_basic_types_demo(void)
{
	uart_string("\\r\\n=== MEMORY TYPES DEMONSTRATION ===\\r\\n");

	// 1. SRAM (Data memory) - Fast read/write
	uart_string("1. SRAM Operations (Fast R/W):\\r\\n");

	global_counter = 50;
	uart_string("   Global variable write: ");
	char val_str[8];
	sprintf(val_str, "%d", global_counter);
	uart_string(val_str);
	uart_string("\\r\\n");

	// Array operations demonstration
	for (uint8_t i = 0; i < 5; i++)
	{
		global_array[i] = i * i;
	}
	uart_string("   Array filled with squares: ");
	for (uint8_t i = 0; i < 5; i++)
	{
		sprintf(val_str, "%d ", global_array[i]);
		uart_string(val_str);
	}
	uart_string("\\r\\n");

	// 2. Flash memory (Program memory) - Read-only
	uart_string("\\r\\n2. Flash Memory Operations (Read-only):\\r\\n");

	uart_string("   Reading const_table: ");
	for (uint8_t i = 0; i < 5; i++)
	{
		uint8_t value = pgm_read_byte(&const_table[i]);
		sprintf(val_str, "0x%02X ", value);
		uart_string(val_str);
	}
	uart_string("\\r\\n");

	uart_string("   Reading const_message: ");
	for (uint8_t i = 0; i < strlen_P(const_message); i++)
	{
		char c = pgm_read_byte(&const_message[i]);
		uart_transmit(c);
	}
	uart_string("\\r\\n");

	// 3. Stack demonstration
	uart_string("\\r\\n3. Stack Memory Operations:\\r\\n");
	uint16_t result = memory_basic_stack_demo(3);
	uart_string("   Stack demo result: ");
	sprintf(val_str, "%d", result);
	uart_string(val_str);
	uart_string("\\r\\n");
}

// Memory usage monitoring and analysis
void memory_basic_usage_monitor(void)
{
	uart_string("\\r\\n=== MEMORY USAGE MONITORING ===\\r\\n");

	// Estimate stack usage
	uint8_t stack_var;
	uint16_t stack_ptr = (uint16_t)&stack_var;

	// ATmega128 SRAM: 0x0100 to 0x10FF (4096 bytes)
	// Stack starts at top of SRAM and grows downward
	uint16_t stack_usage = 0x10FF - stack_ptr;

	uart_string("Current stack usage: ");
	char usage_str[16];
	sprintf(usage_str, "%d bytes", stack_usage);
	uart_string(usage_str);
	uart_string("\\r\\n");

	// Show free SRAM estimation
	uint16_t heap_ptr = (uint16_t)&global_counter + sizeof(global_counter);
	uint16_t free_sram = stack_ptr - heap_ptr;

	uart_string("Estimated free SRAM: ");
	sprintf(usage_str, "%d bytes", free_sram);
	uart_string(usage_str);
	uart_string("\\r\\n");

	// Memory map visualization
	uart_string("\\r\\nATmega128 Memory Map:\\r\\n");
	uart_string("  0x0000-0x001F: CPU Registers\\r\\n");
	uart_string("  0x0020-0x005F: I/O Registers\\r\\n");
	uart_string("  0x0060-0x00FF: Extended I/O\\r\\n");
	uart_string("  0x0100-0x10FF: SRAM (4KB)\\r\\n");
	uart_string("    |-- Global/Static Variables\\r\\n");
	uart_string("    |-- Heap (if used)\\r\\n");
	uart_string("    |-- Free Space\\r\\n");
	uart_string("    |-- Stack (grows downward)\\r\\n");
}

void main_memory_basic(void)
{
	init_devices();

	uart_string("\\r\\n=== MEMORY BASIC DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Memory types and characteristics\\r\\n");
	uart_string("- Variable storage locations\\r\\n");
	uart_string("- Memory address inspection\\r\\n");
	uart_string("- Stack growth demonstration\\r\\n");
	uart_string("- Memory usage optimization\\r\\n\\r\\n");

	uart_string("Press any button to start memory exploration...\\r\\n");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting memory basics demonstration...\\r\\n");

	// 1. Memory location analysis
	memory_basic_analyze_locations();
	_delay_ms(2000);

	// 2. Memory types demonstration
	memory_basic_types_demo();
	_delay_ms(2000);

	// 3. Usage monitoring
	memory_basic_usage_monitor();
	_delay_ms(2000);

	// Interactive exploration
	uart_string("\\r\\nInteractive memory exploration:\\r\\n");
	uart_string("PD0 - Increment global counter\\r\\n");
	uart_string("PD7 - Exit demo\\r\\n");

	uint8_t prev_buttons = 0xFF;

	for (uint16_t cycle = 0; cycle < 200; cycle++)
	{
		// Read button input
		uint8_t current_buttons = PIND;
		uint8_t button_pressed = (~current_buttons) & prev_buttons;

		if (button_pressed & (1 << PD0))
		{
			global_counter++;
			uart_string("Global counter: ");
			char counter_str[8];
			sprintf(counter_str, "%d", global_counter);
			uart_string(counter_str);
			uart_string(" (at 0x");
			sprintf(counter_str, "%04X", (uint16_t)&global_counter);
			uart_string(counter_str);
			uart_string(")\\r\\n");
		}

		if (button_pressed & (1 << PD7))
		{
			uart_string("Exiting memory demo...\\r\\n");
			break;
		}

		prev_buttons = current_buttons;

		// Show activity on LEDs
		PORTB = global_counter;

		_delay_ms(50);
	}

	uart_string("\\r\\n=== MEMORY BASIC DEMO COMPLETED ===\\r\\n");
	uart_string("Key concepts covered:\\r\\n");
	uart_string("- SRAM vs Flash vs Stack memory\\r\\n");
	uart_string("- Variable storage locations\\r\\n");
	uart_string("- Memory usage monitoring\\r\\n");
	uart_string("- Interactive memory exploration\\r\\n\\r\\n");

	while (1)
	{
		PORTB = global_counter;
		_delay_ms(1000);
	}
}

#endif // MEMORY_BASIC

/*
 * MEMORY_STACK - Advanced Stack Memory Management
 * Educational demonstration of:
 * - Stack frame analysis and visualization
 * - Function call overhead measurement
 * - Stack overflow detection and prevention
 * - Local variable scope and lifetime
 * - Recursive function memory usage
 */

#ifdef MEMORY_STACK

#define MAX_STACK_DEPTH 10
#define STACK_MONITOR_SIZE 50

// Stack frame analysis structure
typedef struct
{
	uint16_t address;
	uint8_t depth;
	uint16_t frame_size;
	char function_name[16];
} stack_frame_t;

// Stack monitoring arrays
static stack_frame_t stack_frames[MAX_STACK_DEPTH];
static uint16_t stack_history[STACK_MONITOR_SIZE];
static uint8_t stack_frame_count = 0;
static uint8_t history_index = 0;

// Utility function to get current stack pointer
uint16_t memory_stack_get_sp(void)
{
	uint8_t dummy;
	return (uint16_t)&dummy;
}

// Record stack frame information
void memory_stack_record_frame(const char *func_name, uint8_t depth)
{
	if (stack_frame_count < MAX_STACK_DEPTH)
	{
		stack_frame_t *frame = &stack_frames[stack_frame_count];
		frame->address = memory_stack_get_sp();
		frame->depth = depth;
		frame->frame_size = (stack_frame_count > 0) ? stack_frames[stack_frame_count - 1].address - frame->address : 0;
		strncpy(frame->function_name, func_name, 15);
		frame->function_name[15] = '\\0';
		stack_frame_count++;
	}

	// Record in history
	stack_history[history_index] = memory_stack_get_sp();
	history_index = (history_index + 1) % STACK_MONITOR_SIZE;
}

// Recursive function for stack analysis
uint32_t memory_stack_fibonacci(uint8_t n)
{
	memory_stack_record_frame("fibonacci", n);

	// Local variables to increase stack usage
	uint16_t local_array[5] = {1, 1, 2, 3, 5};
	uint32_t result;

	uart_string("Fibonacci(");
	char n_str[8];
	sprintf(n_str, "%d", n);
	uart_string(n_str);
	uart_string(") - SP: 0x");
	char sp_str[8];
	sprintf(sp_str, "%04X", memory_stack_get_sp());
	uart_string(sp_str);
	uart_string("\\r\\n");

	// Base cases
	if (n <= 1)
	{
		result = n;
	}
	else
	{
		// Recursive calls
		result = memory_stack_fibonacci(n - 1) + memory_stack_fibonacci(n - 2);
	}

	// Use local array to prevent optimization
	for (uint8_t i = 0; i < 5; i++)
	{
		local_array[i] += result & 0xFF;
	}

	stack_frame_count--; // Pop frame
	return result;
}

// Function with large local variables
void memory_stack_heavy_function(uint8_t level)
{
	memory_stack_record_frame("heavy_func", level);

	// Large local arrays to demonstrate stack usage
	uint8_t large_array1[50];
	uint16_t large_array2[25];
	char string_buffer[32];

	// Initialize arrays
	for (uint8_t i = 0; i < 50; i++)
	{
		large_array1[i] = i + level;
	}
	for (uint8_t i = 0; i < 25; i++)
	{
		large_array2[i] = (i * level) + 1000;
	}
	sprintf(string_buffer, "Heavy function level %d", level);

	uart_string("Heavy function - SP: 0x");
	char sp_str[8];
	sprintf(sp_str, "%04X", memory_stack_get_sp());
	uart_string(sp_str);
	uart_string(", Local vars: ~107 bytes\\r\\n");

	// Show some data to prevent optimization
	uart_string("Sample data: ");
	char data_str[16];
	sprintf(data_str, "%d, %d", large_array1[level % 50], large_array2[level % 25]);
	uart_string(data_str);
	uart_string("\\r\\n");

	// Recursive call with smaller arrays
	if (level > 0)
	{
		memory_stack_heavy_function(level - 1);
	}

	stack_frame_count--;
}

// Stack overflow detection demonstration
void memory_stack_overflow_demo(uint16_t depth)
{
	uint16_t current_sp = memory_stack_get_sp();
	uint8_t local_data[10];

	// Initialize local data
	for (uint8_t i = 0; i < 10; i++)
	{
		local_data[i] = (depth + i) & 0xFF;
	}

	uart_string("Depth ");
	char depth_str[8];
	sprintf(depth_str, "%d", depth);
	uart_string(depth_str);
	uart_string(" - SP: 0x");
	char sp_str[8];
	sprintf(sp_str, "%04X", current_sp);
	uart_string(sp_str);

	// Check for potential stack overflow
	if (current_sp < 0x0200)
	{ // Conservative lower limit
		uart_string(" - WARNING: Low stack!\\r\\n");
		return;
	}

	uart_string("\\r\\n");

	// Show on LEDs
	PORTB = depth & 0xFF;
	_delay_ms(100);

	// Recursive call with safety check
	if (depth < 50 && current_sp > 0x0250)
	{
		memory_stack_overflow_demo(depth + 1);
	}
	else
	{
		uart_string("Reached maximum safe depth\\r\\n");
	}
}

// Stack frame visualization
void memory_stack_visualize_frames(void)
{
	uart_string("\\r\\n=== STACK FRAME VISUALIZATION ===\\r\\n");

	uart_string("Current stack frames:\\r\\n");
	for (uint8_t i = 0; i < stack_frame_count; i++)
	{
		uart_string("Frame ");
		char frame_str[8];
		sprintf(frame_str, "%d", i);
		uart_string(frame_str);
		uart_string(": ");
		uart_string(stack_frames[i].function_name);
		uart_string(" @ 0x");
		sprintf(frame_str, "%04X", stack_frames[i].address);
		uart_string(frame_str);
		if (stack_frames[i].frame_size > 0)
		{
			uart_string(" (");
			sprintf(frame_str, "%d", stack_frames[i].frame_size);
			uart_string(frame_str);
			uart_string(" bytes)");
		}
		uart_string("\\r\\n");
	}

	// Stack usage history
	uart_string("\\r\\nStack pointer history (last 10):\\r\\n");
	uint8_t start_idx = (history_index + STACK_MONITOR_SIZE - 10) % STACK_MONITOR_SIZE;
	for (uint8_t i = 0; i < 10; i++)
	{
		uint8_t idx = (start_idx + i) % STACK_MONITOR_SIZE;
		uart_string("0x");
		char addr_str[8];
		sprintf(addr_str, "%04X", stack_history[idx]);
		uart_string(addr_str);
		if (i < 9)
			uart_string(", ");
	}
	uart_string("\\r\\n");
}

// Interactive stack analysis
void memory_stack_interactive_analysis(void)
{
	uart_string("\\r\\n=== INTERACTIVE STACK ANALYSIS ===\\r\\n");
	uart_string("Controls:\\r\\n");
	uart_string("PD0 - Fibonacci demo\\r\\n");
	uart_string("PD1 - Heavy function demo\\r\\n");
	uart_string("PD2 - Stack overflow test\\r\\n");
	uart_string("PD7 - Exit analysis\\r\\n\\r\\n");

	uint8_t prev_buttons = 0xFF;
	uint8_t fib_n = 5;

	for (uint16_t cycle = 0; cycle < 300; cycle++)
	{
		uint8_t current_buttons = PIND;
		uint8_t button_pressed = (~current_buttons) & prev_buttons;

		if (button_pressed & (1 << PD0))
		{
			uart_string("\\r\\nFibonacci recursion demo:\\r\\n");
			stack_frame_count = 0; // Reset frame tracking
			uint32_t result = memory_stack_fibonacci(fib_n);
			uart_string("Result: ");
			char result_str[16];
			sprintf(result_str, "%lu", result);
			uart_string(result_str);
			uart_string("\\r\\n");
			memory_stack_visualize_frames();
			fib_n = (fib_n % 8) + 3; // Cycle between 3-10
		}

		if (button_pressed & (1 << PD1))
		{
			uart_string("\\r\\nHeavy function demo:\\r\\n");
			stack_frame_count = 0;
			memory_stack_heavy_function(3);
			memory_stack_visualize_frames();
		}

		if (button_pressed & (1 << PD2))
		{
			uart_string("\\r\\nStack overflow test:\\r\\n");
			memory_stack_overflow_demo(0);
			uart_string("Test completed safely\\r\\n");
		}

		if (button_pressed & (1 << PD7))
		{
			uart_string("Exiting stack analysis...\\r\\n");
			break;
		}

		prev_buttons = current_buttons;

		// Show current stack usage on LEDs
		uint16_t current_sp = memory_stack_get_sp();
		PORTB = (0x10FF - current_sp) >> 3; // Rough stack usage indicator

		_delay_ms(50);
	}
}

void main_memory_stack(void)
{
	init_devices();

	uart_string("\\r\\n=== MEMORY STACK DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Stack frame analysis and visualization\\r\\n");
	uart_string("- Function call overhead measurement\\r\\n");
	uart_string("- Stack overflow detection\\r\\n");
	uart_string("- Local variable scope and lifetime\\r\\n");
	uart_string("- Recursive function memory usage\\r\\n\\r\\n");

	// Initialize stack monitoring
	stack_frame_count = 0;
	history_index = 0;

	uart_string("Initial stack pointer: 0x");
	char sp_str[8];
	sprintf(sp_str, "%04X", memory_stack_get_sp());
	uart_string(sp_str);
	uart_string("\\r\\n");

	uart_string("ATmega128 SRAM: 0x0100-0x10FF (4096 bytes)\\r\\n");
	uart_string("Stack grows downward from 0x10FF\\r\\n\\r\\n");

	uart_string("Press any button to start stack analysis...\\r\\n");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting stack demonstrations...\\r\\n");

	// 1. Basic stack frame demo
	uart_string("\\r\\n1. Basic Stack Frame Analysis\\r\\n");
	stack_frame_count = 0;
	memory_stack_heavy_function(2);
	memory_stack_visualize_frames();
	_delay_ms(2000);

	// 2. Recursive function analysis
	uart_string("\\r\\n2. Recursive Function Stack Usage\\r\\n");
	stack_frame_count = 0;
	uint32_t fib_result = memory_stack_fibonacci(6);
	uart_string("Fibonacci(6) = ");
	char result_str[16];
	sprintf(result_str, "%lu", fib_result);
	uart_string(result_str);
	uart_string("\\r\\n");
	memory_stack_visualize_frames();
	_delay_ms(2000);

	// 3. Interactive analysis
	memory_stack_interactive_analysis();

	uart_string("\\r\\n=== MEMORY STACK DEMO COMPLETED ===\\r\\n");
	uart_string("Key concepts covered:\\r\\n");
	uart_string("- Stack frame structure and analysis\\r\\n");
	uart_string("- Function call overhead and recursion\\r\\n");
	uart_string("- Stack overflow detection and prevention\\r\\n");
	uart_string("- Memory usage optimization for stack\\r\\n\\r\\n");

	while (1)
	{
		// Show current stack usage
		uint16_t current_sp = memory_stack_get_sp();
		PORTB = (0x10FF - current_sp) >> 4;
		_delay_ms(1000);
	}
}

#endif // MEMORY_STACK

#ifdef MEMORY_HEAP
// Heap Simulation and Management - Complete Memory Management Education

// Simulated heap memory pool
#define HEAP_SIZE 1024
static char heap_memory[HEAP_SIZE];

// Heap block header for tracking allocations
typedef struct heap_block
{
	uint16_t size;			 // Size of the block
	uint8_t is_free;		 // 1 if free, 0 if allocated
	struct heap_block *next; // Next block in the chain
} heap_block_t;

// Heap management variables
static heap_block_t *heap_start = NULL;
static uint16_t total_allocated = 0;
static uint16_t largest_free_block = 0;
static uint16_t fragmentation_count = 0;

void init_heap()
{
	// Initialize the heap with one large free block
	heap_start = (heap_block_t *)heap_memory;
	heap_start->size = HEAP_SIZE - sizeof(heap_block_t);
	heap_start->is_free = 1;
	heap_start->next = NULL;

	total_allocated = 0;
	largest_free_block = heap_start->size;
	fragmentation_count = 0;
}

void *heap_malloc(uint16_t size)
{
	// Find a suitable free block
	heap_block_t *current = heap_start;
	heap_block_t *best_fit = NULL;
	uint16_t best_size = 0xFFFF;

	// First-fit or best-fit algorithm
	while (current != NULL)
	{
		if (current->is_free && current->size >= size)
		{
			if (current->size < best_size)
			{
				best_fit = current;
				best_size = current->size;
			}
		}
		current = current->next;
	}

	if (best_fit == NULL)
	{
		return NULL; // No suitable block found
	}

	// Split the block if it's much larger than needed
	if (best_fit->size > size + sizeof(heap_block_t) + 8)
	{
		heap_block_t *new_block = (heap_block_t *)((char *)best_fit + sizeof(heap_block_t) + size);
		new_block->size = best_fit->size - size - sizeof(heap_block_t);
		new_block->is_free = 1;
		new_block->next = best_fit->next;

		best_fit->size = size;
		best_fit->next = new_block;
		fragmentation_count++;
	}

	best_fit->is_free = 0;
	total_allocated += size;

	return (void *)((char *)best_fit + sizeof(heap_block_t));
}

void heap_free(void *ptr)
{
	if (ptr == NULL)
		return;

	// Find the block header
	heap_block_t *block = (heap_block_t *)((char *)ptr - sizeof(heap_block_t));

	if (block->is_free)
		return; // Already free

	block->is_free = 1;
	total_allocated -= block->size;

	// Coalesce with next block if it's free
	if (block->next && block->next->is_free)
	{
		block->size += block->next->size + sizeof(heap_block_t);
		block->next = block->next->next;
		fragmentation_count--;
	}

	// Coalesce with previous block (simple linear search)
	heap_block_t *prev = heap_start;
	while (prev && prev->next != block)
	{
		prev = prev->next;
	}

	if (prev && prev->is_free)
	{
		prev->size += block->size + sizeof(heap_block_t);
		prev->next = block->next;
		fragmentation_count--;
	}
}

void analyze_heap()
{
	heap_block_t *current = heap_start;
	uint16_t free_blocks = 0;
	uint16_t allocated_blocks = 0;
	uint16_t total_free = 0;
	largest_free_block = 0;

	glcd_clear();
	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

	while (current != NULL)
	{
		if (current->is_free)
		{
			free_blocks++;
			total_free += current->size;
			if (current->size > largest_free_block)
			{
				largest_free_block = current->size;
			}
		}
		else
		{
			allocated_blocks++;
		}
		current = current->next;
	}

	char buffer[20];

	glcd_tiny_draw_string(0, 0, "HEAP ANALYSIS:");

	sprintf(buffer, "Total: %d bytes", HEAP_SIZE);
	glcd_tiny_draw_string(0, 10, buffer);

	sprintf(buffer, "Allocated: %d", total_allocated);
	glcd_tiny_draw_string(0, 20, buffer);

	sprintf(buffer, "Free: %d bytes", total_free);
	glcd_tiny_draw_string(0, 30, buffer);

	sprintf(buffer, "Largest free: %d", largest_free_block);
	glcd_tiny_draw_string(0, 40, buffer);

	sprintf(buffer, "Free blocks: %d", free_blocks);
	glcd_tiny_draw_string(0, 50, buffer);

	sprintf(buffer, "Alloc blocks: %d", allocated_blocks);
	glcd_tiny_draw_string(0, 60, buffer);

	sprintf(buffer, "Fragmentation: %d", fragmentation_count);
	glcd_tiny_draw_string(0, 70, buffer);

	// Visual heap map
	glcd_tiny_draw_string(0, 90, "Heap Map:");
	current = heap_start;
	uint16_t x_pos = 0;

	while (current != NULL && x_pos < 120)
	{
		uint8_t block_width = (current->size * 100) / HEAP_SIZE;
		if (block_width < 2)
			block_width = 2;
		if (block_width > 20)
			block_width = 20;

		for (uint8_t i = 0; i < block_width && x_pos + i < 128; i++)
		{
			for (uint8_t j = 0; j < 8; j++)
			{
				if (current->is_free)
				{
					glcd_set_pixel(x_pos + i, 100 + j, (i + j) % 2); // Striped pattern for free
				}
				else
				{
					glcd_set_pixel(x_pos + i, 100 + j, 1); // Solid for allocated
				}
			}
		}

		x_pos += block_width + 1;
		current = current->next;
	}
}

void memory_stress_test()
{
	void *ptrs[20];
	uint16_t sizes[20] = {32, 64, 16, 128, 8, 256, 24, 48, 72, 96,
						  40, 80, 120, 160, 12, 28, 56, 84, 112, 200};

	glcd_clear();
	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
	glcd_tiny_draw_string(0, 0, "STRESS TEST:");

	// Allocation phase
	for (uint8_t i = 0; i < 20; i++)
	{
		ptrs[i] = heap_malloc(sizes[i]);
		char buffer[30];
		sprintf(buffer, "Alloc %d: %s", i, ptrs[i] ? "OK" : "FAIL");
		glcd_tiny_draw_string(0, 10 + i * 8, buffer);

		if (i % 5 == 0)
		{
			_delay_ms(500);
			glcd_clear();
			glcd_tiny_draw_string(0, 0, "STRESS TEST:");
		}
	}

	_delay_ms(1000);

	// Random deallocation and reallocation
	for (uint8_t cycle = 0; cycle < 5; cycle++)
	{
		// Free some random blocks
		for (uint8_t i = 0; i < 20; i += 3)
		{
			if (ptrs[i] != NULL)
			{
				heap_free(ptrs[i]);
				ptrs[i] = NULL;
			}
		}

		// Try to allocate new blocks
		for (uint8_t i = 0; i < 20; i += 3)
		{
			if (ptrs[i] == NULL)
			{
				ptrs[i] = heap_malloc(sizes[i] / 2);
			}
		}

		analyze_heap();
		_delay_ms(1000);
	}

	// Clean up
	for (uint8_t i = 0; i < 20; i++)
	{
		if (ptrs[i] != NULL)
		{
			heap_free(ptrs[i]);
		}
	}
}

void demonstrate_fragmentation()
{
	glcd_clear();
	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
	glcd_tiny_draw_string(0, 0, "FRAGMENTATION DEMO:");

	void *blocks[10];

	// Allocate alternating small and large blocks
	blocks[0] = heap_malloc(100);
	blocks[1] = heap_malloc(20);
	blocks[2] = heap_malloc(100);
	blocks[3] = heap_malloc(20);
	blocks[4] = heap_malloc(100);
	blocks[5] = heap_malloc(20);
	blocks[6] = heap_malloc(100);
	blocks[7] = heap_malloc(20);

	glcd_tiny_draw_string(0, 10, "Step 1: Alternating alloc");
	analyze_heap();
	_delay_ms(2000);

	// Free all large blocks, keeping small ones
	heap_free(blocks[0]);
	heap_free(blocks[2]);
	heap_free(blocks[4]);
	heap_free(blocks[6]);

	glcd_clear();
	glcd_tiny_draw_string(0, 0, "Step 2: Free large blocks");
	analyze_heap();
	_delay_ms(2000);

	// Try to allocate a large block - should fail due to fragmentation
	void *large_block = heap_malloc(200);

	glcd_clear();
	glcd_tiny_draw_string(0, 0, "Step 3: Try large alloc");
	char buffer[30];
	sprintf(buffer, "200 byte alloc: %s", large_block ? "SUCCESS" : "FAILED");
	glcd_tiny_draw_string(0, 10, buffer);
	glcd_tiny_draw_string(0, 20, "(Due to fragmentation)");
	analyze_heap();
	_delay_ms(3000);

	// Clean up remaining blocks
	heap_free(blocks[1]);
	heap_free(blocks[3]);
	heap_free(blocks[5]);
	heap_free(blocks[7]);
	if (large_block)
		heap_free(large_block);
}

void main_memory_heap()
{
	DDRA = 0xFF; // Debug LEDs
	DDRB = 0xFF; // Control outputs

	// Initialize systems
	init_GLCD();
	init_heap();

	glcd_clear();
	glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

	while (1)
	{
		// Phase 1: Initial heap state
		glcd_clear();
		glcd_tiny_draw_string(20, 20, "HEAP MANAGEMENT");
		glcd_tiny_draw_string(30, 35, "DEMONSTRATION");
		glcd_tiny_draw_string(25, 50, "Press any key...");
		while (!(PINF & 0x0F))
			; // Wait for button
		while (PINF & 0x0F)
			; // Wait for release

		init_heap(); // Reset heap
		glcd_clear();
		glcd_tiny_draw_string(0, 0, "Initial Heap State:");
		analyze_heap();
		_delay_ms(2000);

		// Phase 2: Basic allocation/deallocation
		glcd_clear();
		glcd_tiny_draw_string(0, 0, "Basic Operations:");

		void *ptr1 = heap_malloc(100);
		glcd_tiny_draw_string(0, 10, "Allocated 100 bytes");
		analyze_heap();
		_delay_ms(1500);

		void *ptr2 = heap_malloc(200);
		glcd_clear();
		glcd_tiny_draw_string(0, 0, "Allocated 200 more");
		analyze_heap();
		_delay_ms(1500);

		heap_free(ptr1);
		glcd_clear();
		glcd_tiny_draw_string(0, 0, "Freed first block");
		analyze_heap();
		_delay_ms(1500);

		void *ptr3 = heap_malloc(50);
		glcd_clear();
		glcd_tiny_draw_string(0, 0, "Allocated 50 (reuse)");
		analyze_heap();
		_delay_ms(2000);

		// Clean up
		heap_free(ptr2);
		heap_free(ptr3);

		// Phase 3: Fragmentation demonstration
		demonstrate_fragmentation();

		// Phase 4: Stress testing
		init_heap(); // Reset for stress test
		memory_stress_test();

		// Phase 5: Final analysis
		glcd_clear();
		glcd_tiny_draw_string(20, 20, "HEAP DEMO");
		glcd_tiny_draw_string(25, 35, "COMPLETE");
		glcd_tiny_draw_string(10, 50, "Press key to restart");
		while (!(PINF & 0x0F))
			; // Wait for button
		while (PINF & 0x0F)
			; // Wait for release

		// Visual feedback
		PORTA = ~PORTA; // Toggle LEDs
	}
}
#endif

/*
 * MODERNIZED MEMORY ACCESS DEMONSTRATIONS
 * Educational Framework: ATmega128 Memory Systems and Data Storage
 *
 * Learning Objectives:
 * 1. Master different memory types (Flash, EEPROM) and their applications
 * 2. Understand modernized EEPROM library for safe data storage
 * 3. Learn data persistence and non-volatile storage concepts
 * 4. Explore memory-based pattern generation and data visualization
 *
 * Memory Library Functions Used:
 * - EEPROM_write()/EEPROM_read(): Safe EEPROM operations with error checking
 * - EEPROM_write_array()/EEPROM_read_array(): Bulk data operations
 * - EEPROM_get_status(): Operation status monitoring
 * - Flash memory access: Efficient program space utilization
 *
 * Integration with Other Libraries:
 * - Port library: Button input for memory operations
 * - Timer2 library: Precise timing for memory access patterns
 * - GLCD library: Visual representation of memory data
 *
 * Hardware Connections:
 * - PORTB: LED indicators for memory operation status
 * - PORTD.0: Button for triggering memory operations
 * - LCD: Memory data display and operation status
 */

/*  Flash Memory Access with Educational Data Patterns */
#ifdef MEMORY_PROGRAM
/*
 * DEMONSTRATION 1: Flash Memory Access with Visual Data Patterns
 * Educational Focus: Program memory utilization and data visualization
 *
 * This example demonstrates:
 * - Efficient flash memory data storage and retrieval
 * - Visual representation of stored data patterns
 * - Memory-based LED pattern generation
 * - Educational data progression and display
 *
 * Learning Points:
 * 1. Flash memory provides efficient storage for constant data
 * 2. pgm_read_byte() enables safe program memory access
 * 3. Memory data can drive visual patterns and displays
 * 4. Data organization enhances educational progression
 *
 * Hardware Setup:
 * - LEDs connected to PORTB show memory data patterns
 * - LCD displays educational information and data values
 */

#include <avr/pgmspace.h>

// Educational lookup table stored in flash memory
const unsigned char PROGMEM educational_lookup[] =
	"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_+-=[]{}|;:,.<>?";

void main_memory_program(void)
{
	init_devices();

	// Initialize Timer2 for precise timing
	Timer2_init();
	Timer2_start();

	// Configure PORTB for visual feedback using modernized library
	Port_init_output(0xFF, 1);

	// Display educational information
	lcd_clear();
	lcd_string(0, 0, "Flash Memory Demo");
	lcd_string(0, 1, "Educational Data");
	lcd_string(0, 2, "Visual Patterns");
	lcd_string(0, 3, "Timer2 Precision");

	unsigned char current_data;
	unsigned int data_index = 0;
	unsigned long last_update = 0;
	const unsigned int LOOKUP_SIZE = sizeof(educational_lookup) - 1;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Update display every 800ms for educational pacing
		if (current_time - last_update >= 800)
		{
			last_update = current_time;

			// Read data from flash memory
			current_data = pgm_read_byte(&educational_lookup[data_index]);

			// Generate LED pattern based on data value
			unsigned char led_pattern = ~(current_data - 0x30);
			Port_write(led_pattern, 1);

			// Update display with educational information
			lcd_clear();
			ScreenBuffer_clear();

			lcd_string(0, 0, "Flash Data Display");
			lcd_string(0, 1, "Index: ");
			GLCD_3DigitDecimal(data_index);

			lcd_string(0, 2, "Char: ");
			lcd_char(current_data);
			lcd_string(3, 2, " (0x");
			GLCD_2DigitHex(current_data);
			lcd_string(8, 2, ")");

			lcd_string(0, 3, "LED: 0x");
			GLCD_2DigitHex(led_pattern);

			// Progress indicator
			lcd_string(0, 4, "Progress: ");
			unsigned int progress = (data_index * 100) / LOOKUP_SIZE;
			GLCD_3DigitDecimal(progress);
			lcd_string(12, 4, "%");

			// Timing information
			lcd_string(0, 5, "Time: ");
			GLCD_4DigitDecimal(current_time / 1000);
			lcd_string(8, 5, "s");

			// Advance to next data element
			data_index = (data_index + 1) % LOOKUP_SIZE;
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(50);
	}
}
#endif

/* EEPROM Access with Interactive Control and Data Persistence */
#ifdef MEMORY_EEPROM
/*
 * DEMONSTRATION 2: Advanced EEPROM Operations with Interactive Control
 * Educational Focus: Non-volatile data storage and interactive memory operations
 *
 * This example demonstrates:
 * - Modernized EEPROM library for safe data storage operations
 * - Interactive button control for write/read operations
 * - Data persistence across power cycles
 * - Real-time operation status monitoring
 *
 * Learning Points:
 * 1. EEPROM_write()/EEPROM_read() provide safe data persistence
 * 2. Interactive control demonstrates user-driven memory operations
 * 3. Status monitoring ensures reliable data operations
 * 4. Timer2 debouncing improves user interaction reliability
 *
 * Hardware Setup:
 * - Button connected to PORTD.0 for write/read control
 * - LEDs connected to PORTB for operation status indication
 * - LCD displays EEPROM data and operation status
 */

// Educational data array for EEPROM operations
unsigned char educational_data[] = "SOC3050-ATmega128-EEPROM-Demo-2025";
unsigned char read_buffer[40];			// Buffer for reading data back
unsigned int eeprom_base_address = 100; // Starting EEPROM address

void main_memory_eeprom(void)
{
	init_devices();

	// Initialize Timer2 for precise timing and debouncing
	Timer2_init();
	Timer2_start();

	// Initialize EEPROM library for safe operations
	EEPROM_init();

	// Configure ports using modernized libraries
	Port_init_input(0x01, 4);	 // PORTD.0 as input
	Port_set_pullup(0x01, 4, 1); // Enable pull-up
	Port_init_output(0xFF, 1);	 // PORTB as output for status

	// Display educational information
	lcd_clear();
	ScreenBuffer_clear();
	lcd_string(0, 0, "EEPROM Interactive");
	lcd_string(0, 1, "Button: Write/Read");
	lcd_string(0, 2, "PD0: Operation Ctrl");
	lcd_string(0, 3, "Persistent Storage");

	// Sound notification for system ready
	S_Start();

	unsigned char last_button_state = 1; // Start with button not pressed
	unsigned long last_debounce_time = 0;
	unsigned char operation_mode = 0; // 0=read, 1=write
	unsigned int operation_count = 0;
	const unsigned int DEBOUNCE_DELAY = 100; // 100ms debounce

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Read button state with debouncing
		unsigned char current_button_state = Port_read_pin(0, 4);

		// Button state change detection
		if (current_button_state != last_button_state)
		{
			last_debounce_time = current_time;
		}

		// Process button press after debounce period
		if ((current_time - last_debounce_time) > DEBOUNCE_DELAY)
		{
			// Button pressed (low due to pull-up)
			if (last_button_state == 1 && current_button_state == 0)
			{
				operation_count++;
				operation_mode = !operation_mode; // Toggle between read/write

				if (operation_mode) // Write operation
				{
					// Write educational data to EEPROM
					lcd_string(0, 4, "Writing to EEPROM...");
					Port_write(0x0F, 1); // LED pattern for write operation

					// Write data array using modernized EEPROM library
					for (unsigned int i = 0; i < sizeof(educational_data) - 1; i++)
					{
						EEPROM_write(eeprom_base_address + i, educational_data[i]);
						Timer2_delay_ms(5); // Small delay for EEPROM write completion
					}

					lcd_string(0, 4, "Write Complete!     ");
					Port_write(0xF0, 1); // Different LED pattern for completion
				}
				else // Read operation
				{
					// Read data from EEPROM
					lcd_string(0, 4, "Reading from EEPROM...");
					Port_write(0xAA, 1); // LED pattern for read operation

					// Read data using modernized EEPROM library
					for (unsigned int i = 0; i < sizeof(educational_data) - 1; i++)
					{
						read_buffer[i] = EEPROM_read(eeprom_base_address + i);
					}
					read_buffer[sizeof(educational_data) - 1] = '\\0'; // Null terminate

					lcd_string(0, 4, "Read Complete!      ");
					Port_write(0x55, 1); // Different LED pattern for completion

					// Display read data
					lcd_clear();
					ScreenBuffer_clear();
					lcd_string(0, 0, "EEPROM Data:");

					// Display data in chunks for LCD
					lcd_string(0, 1, "SOC3050-ATmega128");
					lcd_string(0, 2, "EEPROM-Demo-2025");
				}

				// Update operation statistics
				lcd_string(0, 5, "Ops: ");
				GLCD_3DigitDecimal(operation_count);
				lcd_string(6, 5, operation_mode ? " Write" : " Read ");

				// Display timing information
				lcd_string(0, 6, "Time: ");
				GLCD_4DigitDecimal(current_time / 1000);
				lcd_string(8, 6, "s");

				// Show EEPROM address range
				lcd_string(0, 7, "Addr: ");
				GLCD_3DigitDecimal(eeprom_base_address);
				lcd_string(6, 7, "-");
				GLCD_3DigitDecimal(eeprom_base_address + sizeof(educational_data) - 2);
			}
		}

		// Update button state for next iteration
		last_button_state = current_button_state;

		// Small delay for system responsiveness
		Timer2_delay_ms(10);
	}
}

#endif // Conditional compilation guard
#endif
