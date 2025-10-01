
#include "config.h"

/*
 * GRAPHICS_BASIC_SHAPES - Fundamental Graphics Programming
 * Educational demonstration of:
 * - Basic geometric shape drawing
 * - Coordinate system understanding
 * - Graphics library integration
 * - Interactive shape manipulation
 * - Mathematical visualization concepts
 */

#ifdef GRAPHICS_BASIC_SHAPES

// Shape drawing demonstration functions
void graphics_draw_basic_shapes(void)
{
	GLCD_ClearScreen();

	uart_string("Drawing basic shapes demonstration...\\r\\n");

	// Draw rectangles of different sizes
	uart_string("Drawing rectangles...\\r\\n");
	for (uint8_t i = 0; i < 4; i++)
	{
		uint8_t x = 10 + (i * 25);
		uint8_t y = 10;
		uint8_t width = 15 + (i * 3);
		uint8_t height = 10 + (i * 2);

		// Draw rectangle outline
		for (uint8_t px = x; px < x + width; px++)
		{
			GLCD_SetDot(px, y);				 // Top edge
			GLCD_SetDot(px, y + height - 1); // Bottom edge
		}
		for (uint8_t py = y; py < y + height; py++)
		{
			GLCD_SetDot(x, py);				// Left edge
			GLCD_SetDot(x + width - 1, py); // Right edge
		}

		_delay_ms(500);
	}

	// Draw circles of different sizes
	uart_string("Drawing circles...\\r\\n");
	for (uint8_t i = 0; i < 3; i++)
	{
		uint8_t center_x = 20 + (i * 30);
		uint8_t center_y = 45;
		uint8_t radius = 5 + (i * 3);

		GLCD_Circle(center_x, center_y, radius);
		_delay_ms(800);
	}
}

void graphics_draw_lines_demo(void)
{
	GLCD_ClearScreen();
	uart_string("Drawing lines and patterns...\\r\\n");

	// Draw diagonal lines
	for (uint8_t i = 0; i < 8; i++)
	{
		// Draw line from top-left to bottom-right
		for (uint8_t j = 0; j < 64; j++)
		{
			uint8_t x = j;
			uint8_t y = (j * i) / 8;
			if (y < 64)
				GLCD_SetDot(x, y);
		}
		_delay_ms(300);
	}

	_delay_ms(1000);
	GLCD_ClearScreen();

	// Draw grid pattern
	uart_string("Drawing grid pattern...\\r\\n");
	for (uint8_t x = 0; x < 128; x += 16)
	{
		for (uint8_t y = 0; y < 64; y++)
		{
			GLCD_SetDot(x, y); // Vertical lines
		}
	}
	for (uint8_t y = 0; y < 64; y += 8)
	{
		for (uint8_t x = 0; x < 128; x++)
		{
			GLCD_SetDot(x, y); // Horizontal lines
		}
	}

	_delay_ms(2000);
}

void graphics_interactive_shapes(void)
{
	GLCD_ClearScreen();
	uart_string("Interactive shapes - use buttons to control...\\r\\n");

	uint8_t shape_x = 64;
	uint8_t shape_y = 32;
	uint8_t shape_size = 10;
	uint8_t shape_type = 0; // 0=circle, 1=rectangle, 2=triangle

	while (1)
	{
		GLCD_ClearScreen();

		// Read button inputs
		uint8_t buttons = PIND;

		// Movement controls
		if (buttons & (1 << PD0))
			shape_x = (shape_x > 5) ? shape_x - 2 : shape_x; // Left
		if (buttons & (1 << PD1))
			shape_x = (shape_x < 120) ? shape_x + 2 : shape_x; // Right
		if (buttons & (1 << PD2))
			shape_y = (shape_y > 5) ? shape_y - 2 : shape_y; // Up
		if (buttons & (1 << PD3))
			shape_y = (shape_y < 55) ? shape_y + 2 : shape_y; // Down

		// Size controls
		if (buttons & (1 << PD4))
			shape_size = (shape_size < 20) ? shape_size + 1 : shape_size; // Bigger
		if (buttons & (1 << PD5))
			shape_size = (shape_size > 3) ? shape_size - 1 : shape_size; // Smaller

		// Shape type control
		if (buttons & (1 << PD6))
		{
			shape_type = (shape_type + 1) % 3;
			_delay_ms(200); // Debounce
		}

		// Exit control
		if (buttons & (1 << PD7))
			break;

		// Draw current shape
		switch (shape_type)
		{
		case 0: // Circle
			GLCD_Circle(shape_x, shape_y, shape_size / 2);
			break;
		case 1: // Rectangle
			for (uint8_t i = 0; i < shape_size; i++)
			{
				GLCD_SetDot(shape_x - shape_size / 2 + i, shape_y - shape_size / 2); // Top
				GLCD_SetDot(shape_x - shape_size / 2 + i, shape_y + shape_size / 2); // Bottom
				GLCD_SetDot(shape_x - shape_size / 2, shape_y - shape_size / 2 + i); // Left
				GLCD_SetDot(shape_x + shape_size / 2, shape_y - shape_size / 2 + i); // Right
			}
			break;
		case 2: // Triangle
			for (uint8_t i = 0; i < shape_size; i++)
			{
				GLCD_SetDot(shape_x - i / 2, shape_y + shape_size / 2 - i);			 // Left edge
				GLCD_SetDot(shape_x + i / 2, shape_y + shape_size / 2 - i);			 // Right edge
				GLCD_SetDot(shape_x - shape_size / 2 + i, shape_y + shape_size / 2); // Base
			}
			break;
		}

		// Display info on LEDs
		PORTB = (shape_x >> 1) | (shape_type << 6);

		_delay_ms(50);
	}
}

void graphics_mathematical_patterns(void)
{
	GLCD_ClearScreen();
	uart_string("Mathematical pattern demonstrations...\\r\\n");

	// Spiral pattern
	uart_string("Drawing spiral pattern...\\r\\n");
	for (uint16_t angle = 0; angle < 720; angle += 5)
	{
		float radius = angle / 20.0;
		float rad = angle * M_PI / 180.0;

		uint8_t x = 64 + (uint8_t)(radius * cos(rad));
		uint8_t y = 32 + (uint8_t)(radius * sin(rad));

		if (x < 128 && y < 64)
		{
			GLCD_SetDot(x, y);
		}
		_delay_ms(30);
	}

	_delay_ms(2000);
	GLCD_ClearScreen();

	// Lissajous pattern
	uart_string("Drawing Lissajous curves...\\r\\n");
	for (uint16_t t = 0; t < 360; t += 2)
	{
		float rad = t * M_PI / 180.0;

		uint8_t x = 64 + (uint8_t)(30 * sin(3 * rad));
		uint8_t y = 32 + (uint8_t)(20 * sin(2 * rad));

		if (x < 128 && y < 64)
		{
			GLCD_SetDot(x, y);
		}
		_delay_ms(20);
	}

	_delay_ms(2000);
}

void main_graphics_basic_shapes(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS BASIC SHAPES DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Basic geometric shape drawing\\r\\n");
	uart_string("- Coordinate system understanding\\r\\n");
	uart_string("- Graphics library integration\\r\\n");
	uart_string("- Interactive shape manipulation\\r\\n");
	uart_string("- Mathematical visualization\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("GRAPHICS DEMO");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Basic Shapes");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("to continue");

	// Wait for button press to start
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		; // Wait for release

	uart_string("Starting graphics demonstrations...\\r\\n");

	// Demo sequence
	graphics_draw_basic_shapes();
	_delay_ms(2000);

	graphics_draw_lines_demo();
	_delay_ms(1000);

	graphics_mathematical_patterns();
	_delay_ms(1000);

	// Interactive mode
	uart_string("\\r\\nEntering interactive mode...\\r\\n");
	uart_string("Controls:\\r\\n");
	uart_string("PD0/PD1 - Move Left/Right\\r\\n");
	uart_string("PD2/PD3 - Move Up/Down\\r\\n");
	uart_string("PD4/PD5 - Size Bigger/Smaller\\r\\n");
	uart_string("PD6 - Change Shape Type\\r\\n");
	uart_string("PD7 - Exit Interactive Mode\\r\\n\\r\\n");

	graphics_interactive_shapes();

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Basic shapes,");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("lines, patterns,");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("and interactive");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("graphics demo");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("completed!");

	uart_string("Graphics basic shapes demonstration completed!\\r\\n");
	uart_string("Demonstrated: shapes, lines, patterns, interaction\\r\\n\\r\\n");

	while (1)
	{
		// Keep display active
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_BASIC_SHAPES

/*
 * GRAPHICS_ANIMATION - Dynamic Graphics and Animation
 * Educational demonstration of:
 * - Frame-based animation techniques
 * - Object movement and transformation
 * - Timing control and smooth motion
 * - Multi-object animation coordination
 * - Interactive animation parameters
 */

#ifdef GRAPHICS_ANIMATION

// Animation object structure
typedef struct
{
	float x, y;		// Position (using float for smooth movement)
	float dx, dy;	// Velocity
	uint8_t size;	// Object size
	uint8_t type;	// Object type (0=circle, 1=square, 2=triangle)
	uint8_t active; // Active flag
} anim_object_t;

#define MAX_OBJECTS 4
static anim_object_t objects[MAX_OBJECTS];
static uint16_t frame_count = 0;

// Initialize animation objects
void animation_init_objects(void)
{
	for (uint8_t i = 0; i < MAX_OBJECTS; i++)
	{
		objects[i].x = 10 + (i * 30);
		objects[i].y = 10 + (i * 15);
		objects[i].dx = 0.5 + (i * 0.3);
		objects[i].dy = 0.3 + (i * 0.2);
		objects[i].size = 4 + (i * 2);
		objects[i].type = i % 3;
		objects[i].active = 1;
	}
	frame_count = 0;
}

// Update object positions
void animation_update_objects(void)
{
	for (uint8_t i = 0; i < MAX_OBJECTS; i++)
	{
		if (objects[i].active)
		{
			// Update position
			objects[i].x += objects[i].dx;
			objects[i].y += objects[i].dy;

			// Bounce off screen edges
			if (objects[i].x <= objects[i].size || objects[i].x >= 128 - objects[i].size)
			{
				objects[i].dx = -objects[i].dx;
				objects[i].x = (objects[i].x <= objects[i].size) ? objects[i].size : 128 - objects[i].size;
			}
			if (objects[i].y <= objects[i].size || objects[i].y >= 64 - objects[i].size)
			{
				objects[i].dy = -objects[i].dy;
				objects[i].y = (objects[i].y <= objects[i].size) ? objects[i].size : 64 - objects[i].size;
			}
		}
	}
	frame_count++;
}

// Draw animated objects
void animation_draw_objects(void)
{
	GLCD_ClearScreen();

	for (uint8_t i = 0; i < MAX_OBJECTS; i++)
	{
		if (objects[i].active)
		{
			uint8_t x = (uint8_t)objects[i].x;
			uint8_t y = (uint8_t)objects[i].y;
			uint8_t size = objects[i].size;

			switch (objects[i].type)
			{
			case 0: // Circle
				GLCD_Circle(x, y, size);
				break;

			case 1: // Square
				for (uint8_t px = x - size; px <= x + size; px++)
				{
					for (uint8_t py = y - size; py <= y + size; py++)
					{
						if (px < 128 && py < 64)
						{
							if (px == x - size || px == x + size || py == y - size || py == y + size)
							{
								GLCD_SetDot(px, py);
							}
						}
					}
				}
				break;

			case 2: // Triangle
				for (uint8_t j = 0; j < size; j++)
				{
					if (x - j / 2 < 128 && y + size - j < 64)
						GLCD_SetDot(x - j / 2, y + size - j);
					if (x + j / 2 < 128 && y + size - j < 64)
						GLCD_SetDot(x + j / 2, y + size - j);
					if (x - size + j < 128 && y + size < 64)
						GLCD_SetDot(x - size + j, y + size);
				}
				break;
			}
		}
	}
}

// Bouncing ball animation
void animation_bouncing_ball_demo(void)
{
	uart_string("Bouncing ball animation...\\r\\n");

	float ball_x = 64, ball_y = 32;
	float ball_dx = 2.5, ball_dy = 1.8;
	uint8_t ball_radius = 6;
	uint16_t trail_x[20], trail_y[20];
	uint8_t trail_index = 0;

	// Initialize trail
	for (uint8_t i = 0; i < 20; i++)
	{
		trail_x[i] = (uint16_t)ball_x;
		trail_y[i] = (uint16_t)ball_y;
	}

	for (uint16_t frame = 0; frame < 500; frame++)
	{
		GLCD_ClearScreen();

		// Update ball position
		ball_x += ball_dx;
		ball_y += ball_dy;

		// Bounce off walls
		if (ball_x <= ball_radius || ball_x >= 128 - ball_radius)
		{
			ball_dx = -ball_dx;
			ball_x = (ball_x <= ball_radius) ? ball_radius : 128 - ball_radius;
			buzzer_play_frequency(800, 50); // Bounce sound
		}
		if (ball_y <= ball_radius || ball_y >= 64 - ball_radius)
		{
			ball_dy = -ball_dy;
			ball_y = (ball_y <= ball_radius) ? ball_radius : 64 - ball_radius;
			buzzer_play_frequency(600, 50); // Bounce sound
		}

		// Update trail
		trail_x[trail_index] = (uint16_t)ball_x;
		trail_y[trail_index] = (uint16_t)ball_y;
		trail_index = (trail_index + 1) % 20;

		// Draw trail (fading effect)
		for (uint8_t i = 0; i < 20; i++)
		{
			if (i % 3 == 0)
			{ // Every 3rd trail point for performance
				GLCD_SetDot(trail_x[i], trail_y[i]);
			}
		}

		// Draw ball
		GLCD_Circle((uint8_t)ball_x, (uint8_t)ball_y, ball_radius);

		// Show frame count on LEDs
		PORTB = frame & 0xFF;

		_delay_ms(30); // Control animation speed

		// Check for user input to exit
		if (PIND & (1 << PD7))
			break;
	}
}

// Rotating objects animation
void animation_rotating_objects_demo(void)
{
	uart_string("Rotating objects animation...\\r\\n");

	uint8_t center_x = 64, center_y = 32;
	uint8_t radius = 25;

	for (uint16_t angle = 0; angle < 720; angle += 3)
	{
		GLCD_ClearScreen();

		float rad = angle * M_PI / 180.0;

		// Draw multiple rotating objects
		for (uint8_t obj = 0; obj < 4; obj++)
		{
			float obj_angle = rad + (obj * M_PI / 2); // 90 degree spacing
			uint8_t obj_x = center_x + (uint8_t)(radius * cos(obj_angle));
			uint8_t obj_y = center_y + (uint8_t)(radius * sin(obj_angle));

			// Draw object based on its number
			switch (obj)
			{
			case 0:
				GLCD_Circle(obj_x, obj_y, 4);
				break;
			case 1:
				for (uint8_t i = 0; i < 6; i++)
				{
					GLCD_SetDot(obj_x - 3 + i, obj_y - 3);
					GLCD_SetDot(obj_x - 3 + i, obj_y + 3);
					GLCD_SetDot(obj_x - 3, obj_y - 3 + i);
					GLCD_SetDot(obj_x + 3, obj_y - 3 + i);
				}
				break;
			case 2:
				for (uint8_t i = 0; i < 4; i++)
				{
					GLCD_SetDot(obj_x - i / 2, obj_y - 3 + i);
					GLCD_SetDot(obj_x + i / 2, obj_y - 3 + i);
					GLCD_SetDot(obj_x - 3 + i, obj_y + 3);
				}
				break;
			case 3:
				GLCD_SetDot(obj_x, obj_y);
				GLCD_SetDot(obj_x - 1, obj_y);
				GLCD_SetDot(obj_x + 1, obj_y);
				GLCD_SetDot(obj_x, obj_y - 1);
				GLCD_SetDot(obj_x, obj_y + 1);
				break;
			}
		}

		// Draw center point
		GLCD_SetDot(center_x, center_y);

		// Draw connection lines
		if (angle % 30 == 0)
		{ // Draw lines every 30 degrees for clarity
			for (uint8_t obj = 0; obj < 4; obj++)
			{
				float obj_angle = rad + (obj * M_PI / 2);
				uint8_t obj_x = center_x + (uint8_t)(radius * cos(obj_angle));
				uint8_t obj_y = center_y + (uint8_t)(radius * sin(obj_angle));

				// Simple line drawing (Bresenham would be overkill here)
				for (uint8_t r = 0; r < radius; r += 3)
				{
					uint8_t line_x = center_x + (uint8_t)(r * cos(obj_angle));
					uint8_t line_y = center_y + (uint8_t)(r * sin(obj_angle));
					GLCD_SetDot(line_x, line_y);
				}
			}
		}

		_delay_ms(40);

		// Check for user input to exit
		if (PIND & (1 << PD7))
			break;
	}
}

// Wave animation
void animation_wave_demo(void)
{
	uart_string("Wave animation demonstration...\\r\\n");

	for (uint16_t phase = 0; phase < 360; phase += 5)
	{
		GLCD_ClearScreen();

		float phase_rad = phase * M_PI / 180.0;

		// Draw multiple sine waves with different frequencies
		for (uint8_t x = 0; x < 128; x++)
		{
			// Primary wave
			float y1 = 32 + 15 * sin((x * M_PI / 32) + phase_rad);

			// Secondary wave (different frequency)
			float y2 = 32 + 8 * sin((x * M_PI / 16) + (phase_rad * 2));

			// Combined wave
			float y3 = 32 + 5 * sin((x * M_PI / 8) + (phase_rad * 0.5));

			if (y1 >= 0 && y1 < 64)
				GLCD_SetDot(x, (uint8_t)y1);
			if (y2 >= 0 && y2 < 64 && x % 2 == 0)
				GLCD_SetDot(x, (uint8_t)y2);
			if (y3 >= 0 && y3 < 64 && x % 4 == 0)
				GLCD_SetDot(x, (uint8_t)y3);
		}

		// Draw phase indicator
		uint8_t indicator_x = 10 + (phase * 100) / 360;
		GLCD_SetDot(indicator_x, 5);
		GLCD_SetDot(indicator_x, 6);
		GLCD_SetDot(indicator_x, 7);

		_delay_ms(50);

		// Check for user input to exit
		if (PIND & (1 << PD7))
			break;
	}
}

void main_graphics_animation(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS ANIMATION DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Frame-based animation techniques\\r\\n");
	uart_string("- Object movement and transformation\\r\\n");
	uart_string("- Timing control and smooth motion\\r\\n");
	uart_string("- Multi-object coordination\\r\\n");
	uart_string("- Interactive animation parameters\\r\\n\\r\\n");

	uart_string("Controls: PD7 - Skip to next animation\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("ANIMATION DEMO");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Dynamic Graphics");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("to start");

	// Wait for button press to start
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		; // Wait for release

	uart_string("Starting animation demonstrations...\\r\\n");

	// Animation sequence
	uart_string("\\r\\n1. Multi-object bouncing animation...\\r\\n");
	animation_init_objects();
	for (uint16_t frame = 0; frame < 300; frame++)
	{
		animation_update_objects();
		animation_draw_objects();

		// Show animation info on LEDs
		PORTB = (frame >> 2) & 0xFF;

		_delay_ms(50);

		if (PIND & (1 << PD7))
			break;
	}

	_delay_ms(1000);

	uart_string("\\r\\n2. Bouncing ball with trail...\\r\\n");
	animation_bouncing_ball_demo();

	_delay_ms(1000);

	uart_string("\\r\\n3. Rotating objects...\\r\\n");
	animation_rotating_objects_demo();

	_delay_ms(1000);

	uart_string("\\r\\n4. Wave animations...\\r\\n");
	animation_wave_demo();

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("ANIMATION");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Frame-based,");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("smooth motion,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("multi-object!");

	uart_string("\\r\\nGraphics animation demonstration completed!\\r\\n");
	uart_string("Demonstrated: bouncing, rotation, waves, trails\\r\\n\\r\\n");

	while (1)
	{
		// Keep display active
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_ANIMATION

/*
 * GRAPHICS_SENSOR_DISPLAY - Real-time Sensor Data Visualization
 * Educational demonstration of:
 * - Real-time sensor data graphing
 * - Dynamic chart generation and updates
 * - Multi-channel data visualization
 * - Interactive display controls
 * - Data buffering and scaling techniques
 */

#ifdef GRAPHICS_SENSOR_DISPLAY

#define SENSOR_BUFFER_SIZE 128
#define MAX_SENSORS 3

// Sensor data structure
typedef struct
{
	uint16_t values[SENSOR_BUFFER_SIZE]; // Circular buffer for sensor readings
	uint8_t write_index;				 // Current write position
	uint16_t min_value, max_value;		 // Auto-scaling bounds
	uint8_t channel;					 // ADC channel
	char name[16];						 // Sensor name
	uint8_t color_pattern;				 // Display pattern for differentiation
} sensor_data_t;

static sensor_data_t sensors[MAX_SENSORS];
static uint8_t display_mode = 0; // 0=line chart, 1=bar chart, 2=scope mode
static uint8_t update_rate = 5;	 // Update delay in 10ms units
static uint16_t sample_count = 0;

// Initialize sensor data structures
void sensor_display_init(void)
{
	// Sensor 0: Light sensor (CDS)
	for (uint16_t i = 0; i < SENSOR_BUFFER_SIZE; i++)
	{
		sensors[0].values[i] = 512; // Initialize to mid-range
	}
	sensors[0].write_index = 0;
	sensors[0].min_value = 1023;
	sensors[0].max_value = 0;
	sensors[0].channel = 0; // ADC0
	strcpy(sensors[0].name, "Light");
	sensors[0].color_pattern = 1; // Solid line

	// Sensor 1: Temperature simulation (potentiometer)
	for (uint16_t i = 0; i < SENSOR_BUFFER_SIZE; i++)
	{
		sensors[1].values[i] = 512;
	}
	sensors[1].write_index = 0;
	sensors[1].min_value = 1023;
	sensors[1].max_value = 0;
	sensors[1].channel = 1; // ADC1
	strcpy(sensors[1].name, "Temp");
	sensors[1].color_pattern = 2; // Dotted line

	// Sensor 2: Accelerometer simulation (button states as digital)
	for (uint16_t i = 0; i < SENSOR_BUFFER_SIZE; i++)
	{
		sensors[2].values[i] = 0;
	}
	sensors[2].write_index = 0;
	sensors[2].min_value = 1023;
	sensors[2].max_value = 0;
	sensors[2].channel = 2; // Digital input simulation
	strcpy(sensors[2].name, "Accel");
	sensors[2].color_pattern = 3; // Dashed line

	sample_count = 0;
}

// Read sensor values
void sensor_display_read_values(void)
{
	for (uint8_t s = 0; s < MAX_SENSORS; s++)
	{
		uint16_t new_value;

		if (s < 2)
		{
			// Read ADC channels
			new_value = adc_read(sensors[s].channel);
		}
		else
		{
			// Simulate accelerometer with button states
			uint8_t buttons = PIND & 0xFF;
			new_value = (buttons != 0xFF) ? 800 : 200; // High when button pressed
		}

		// Update circular buffer
		sensors[s].values[sensors[s].write_index] = new_value;
		sensors[s].write_index = (sensors[s].write_index + 1) % SENSOR_BUFFER_SIZE;

		// Update auto-scaling bounds
		if (new_value < sensors[s].min_value)
			sensors[s].min_value = new_value;
		if (new_value > sensors[s].max_value)
			sensors[s].max_value = new_value;

		// Prevent division by zero in scaling
		if (sensors[s].max_value == sensors[s].min_value)
		{
			sensors[s].max_value = sensors[s].min_value + 1;
		}
	}
	sample_count++;
}

// Scale sensor value to display coordinates
uint8_t sensor_display_scale_value(uint16_t value, uint8_t sensor_index, uint8_t display_height)
{
	sensor_data_t *sensor = &sensors[sensor_index];

	// Scale from sensor range to display range
	uint32_t scaled = ((uint32_t)(value - sensor->min_value) * display_height) /
					  (sensor->max_value - sensor->min_value);

	// Flip Y coordinate (screen coordinates are top-down)
	return display_height - (uint8_t)scaled;
}

// Draw line chart
void sensor_display_draw_line_chart(void)
{
	GLCD_ClearScreen();

	// Draw title
	GLCD_WriteString("SENSOR MONITOR");

	// Draw chart area border
	for (uint8_t x = 10; x < 118; x++)
	{
		GLCD_SetDot(x, 15); // Top border
		GLCD_SetDot(x, 58); // Bottom border
	}
	for (uint8_t y = 15; y < 59; y++)
	{
		GLCD_SetDot(10, y);	 // Left border
		GLCD_SetDot(117, y); // Right border
	}

	// Draw grid lines
	for (uint8_t x = 20; x < 117; x += 20)
	{
		for (uint8_t y = 20; y < 58; y += 5)
		{
			if (y % 10 == 0)
				GLCD_SetDot(x, y); // Major grid
		}
	}

	// Plot sensor data
	for (uint8_t s = 0; s < MAX_SENSORS; s++)
	{
		uint8_t start_index = sensors[s].write_index;

		for (uint8_t i = 0; i < 107; i++)
		{ // 107 pixels wide chart area
			uint8_t buffer_index = (start_index + SENSOR_BUFFER_SIZE - 107 + i) % SENSOR_BUFFER_SIZE;
			uint16_t value = sensors[s].values[buffer_index];
			uint8_t y = sensor_display_scale_value(value, s, 42) + 16; // 42 pixel high chart
			uint8_t x = 11 + i;

			// Draw point based on sensor pattern
			switch (sensors[s].color_pattern)
			{
			case 1: // Solid line
				GLCD_SetDot(x, y);
				break;
			case 2: // Dotted line
				if (i % 2 == 0)
					GLCD_SetDot(x, y);
				break;
			case 3: // Dashed line
				if ((i % 4) < 2)
					GLCD_SetDot(x, y);
				break;
			}
		}
	}

	// Draw legend
	GLCD_SetDot(0, 60);
	GLCD_WriteString("L:Light T:Temp A:Accel");
}

// Draw bar chart
void sensor_display_draw_bar_chart(void)
{
	GLCD_ClearScreen();

	GLCD_WriteString("SENSOR BARS");

	// Get current values
	for (uint8_t s = 0; s < MAX_SENSORS; s++)
	{
		uint8_t current_index = (sensors[s].write_index + SENSOR_BUFFER_SIZE - 1) % SENSOR_BUFFER_SIZE;
		uint16_t current_value = sensors[s].values[current_index];

		// Calculate bar parameters
		uint8_t bar_x = 20 + (s * 30);
		uint8_t bar_width = 20;
		uint8_t bar_height = sensor_display_scale_value(current_value, s, 40);
		uint8_t bar_top = 50 - bar_height;

		// Draw bar
		for (uint8_t x = bar_x; x < bar_x + bar_width; x++)
		{
			for (uint8_t y = bar_top; y < 50; y++)
			{
				// Pattern based on sensor
				if (s == 0 || (s == 1 && (x + y) % 2 == 0) || (s == 2 && (x % 3 == 0)))
				{
					GLCD_SetDot(x, y);
				}
			}
		}

		// Draw bar outline
		for (uint8_t x = bar_x; x < bar_x + bar_width; x++)
		{
			GLCD_SetDot(x, bar_top);
			GLCD_SetDot(x, 49);
		}
		for (uint8_t y = bar_top; y < 50; y++)
		{
			GLCD_SetDot(bar_x, y);
			GLCD_SetDot(bar_x + bar_width - 1, y);
		}

		// Draw sensor label
		GLCD_SetDot(bar_x + 5, 55);
		switch (s)
		{
		case 0:
			GLCD_WriteString("L");
			break;
		case 1:
			GLCD_WriteString("T");
			break;
		case 2:
			GLCD_WriteString("A");
			break;
		}
	}

	// Draw value scale
	GLCD_SetDot(0, 60);
	GLCD_WriteString("Real-time values");
}

// Draw oscilloscope mode
void sensor_display_draw_scope_mode(void)
{
	GLCD_ClearScreen();

	GLCD_WriteString("SCOPE MODE");

	// Draw scope screen
	for (uint8_t x = 5; x < 123; x++)
	{
		GLCD_SetDot(x, 15);
		GLCD_SetDot(x, 55);
	}
	for (uint8_t y = 15; y < 56; y++)
	{
		GLCD_SetDot(5, y);
		GLCD_SetDot(122, y);
	}

	// Draw center line
	for (uint8_t x = 6; x < 122; x += 3)
	{
		GLCD_SetDot(x, 35);
	}

	// Draw trigger line (for sensor 0)
	uint8_t trigger_level = sensor_display_scale_value(600, 0, 38) + 16;
	for (uint8_t x = 6; x < 122; x += 5)
	{
		GLCD_SetDot(x, trigger_level);
	}

	// Plot waveform (sensor 0 as main trace)
	uint8_t start_index = sensors[0].write_index;
	for (uint8_t i = 0; i < 116; i++)
	{
		uint8_t buffer_index = (start_index + SENSOR_BUFFER_SIZE - 116 + i) % SENSOR_BUFFER_SIZE;
		uint16_t value = sensors[0].values[buffer_index];
		uint8_t y = sensor_display_scale_value(value, 0, 38) + 16;
		uint8_t x = 6 + i;

		GLCD_SetDot(x, y);

		// Add persistence effect
		if (i > 0)
		{
			uint8_t prev_buffer_index = (start_index + SENSOR_BUFFER_SIZE - 116 + i - 1) % SENSOR_BUFFER_SIZE;
			uint16_t prev_value = sensors[0].values[prev_buffer_index];
			uint8_t prev_y = sensor_display_scale_value(prev_value, 0, 38) + 16;

			// Connect points
			if (abs(y - prev_y) > 1)
			{
				uint8_t step_y = (y > prev_y) ? prev_y + 1 : prev_y - 1;
				GLCD_SetDot(x, step_y);
			}
		}
	}

	// Status display
	GLCD_SetDot(0, 60);
	GLCD_WriteString("Trig:600 Rate:");
	// Display sample rate
	GLCD_WriteString("10Hz");
}

void main_graphics_sensor_display(void)
{
	init_devices();

	// Initialize GLCD and ADC
	GLCD_Initialize();
	GLCD_ClearScreen();
	adc_init();

	uart_string("\\r\\n=== GRAPHICS SENSOR DISPLAY DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Real-time sensor data graphing\\r\\n");
	uart_string("- Dynamic chart generation\\r\\n");
	uart_string("- Multi-channel visualization\\r\\n");
	uart_string("- Interactive display controls\\r\\n");
	uart_string("- Data buffering and scaling\\r\\n\\r\\n");

	uart_string("Controls:\\r\\n");
	uart_string("PD0 - Switch display mode\\r\\n");
	uart_string("PD1 - Adjust update rate\\r\\n");
	uart_string("PD7 - Exit demo\\r\\n\\r\\n");

	// Initialize sensor data
	sensor_display_init();

	uart_string("Sensors configured:\\r\\n");
	uart_string("- ADC0: Light sensor (CDS)\\r\\n");
	uart_string("- ADC1: Temperature (potentiometer)\\r\\n");
	uart_string("- Digital: Accelerometer simulation\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("SENSOR DISPLAY");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Real-time Data");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Visualization");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("Press button");

	_delay_ms(2000);

	uint8_t prev_buttons = 0xFF;
	uint16_t update_counter = 0;

	uart_string("Starting real-time sensor monitoring...\\r\\n");

	while (1)
	{
		// Read current button state
		uint8_t current_buttons = PIND;
		uint8_t button_pressed = (~current_buttons) & prev_buttons;

		// Handle button presses
		if (button_pressed & (1 << PD0))
		{
			display_mode = (display_mode + 1) % 3;
			uart_string("Display mode: ");
			switch (display_mode)
			{
			case 0:
				uart_string("Line Chart\\r\\n");
				break;
			case 1:
				uart_string("Bar Chart\\r\\n");
				break;
			case 2:
				uart_string("Scope Mode\\r\\n");
				break;
			}
		}

		if (button_pressed & (1 << PD1))
		{
			update_rate = (update_rate == 1) ? 10 : (update_rate == 10) ? 20
																		: 1;
			uart_string("Update rate: ");
			if (update_rate == 1)
				uart_string("Fast\\r\\n");
			else if (update_rate == 10)
				uart_string("Medium\\r\\n");
			else
				uart_string("Slow\\r\\n");
		}

		if (button_pressed & (1 << PD7))
		{
			uart_string("Exiting sensor display demo...\\r\\n");
			break;
		}

		prev_buttons = current_buttons;

		// Update sensor readings
		if (update_counter % update_rate == 0)
		{
			sensor_display_read_values();

			// Update display based on mode
			switch (display_mode)
			{
			case 0:
				sensor_display_draw_line_chart();
				break;
			case 1:
				sensor_display_draw_bar_chart();
				break;
			case 2:
				sensor_display_draw_scope_mode();
				break;
			}

			// Show sample count on LEDs
			PORTB = sample_count & 0xFF;
		}

		update_counter++;
		_delay_ms(10); // Base update interval
	}

	// Final summary display
	GLCD_ClearScreen();
	GLCD_WriteString("SENSOR DISPLAY");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Real-time data");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("visualization,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("multi-channel!");

	uart_string("\\r\\nSensor display demonstration completed!\\r\\n");
	uart_string("Total samples collected: ");
	char sample_str[16];
	sprintf(sample_str, "%u\\r\\n", sample_count);
	uart_string(sample_str);
	uart_string("Demonstrated: line/bar/scope modes, real-time data\\r\\n\\r\\n");

	while (1)
	{
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_SENSOR_DISPLAY

/*
 * GRAPHICS_BASICS - Fundamental Graphics Operations
 * Educational demonstration of:
 * - Basic GLCD drawing primitives
 * - Pixel manipulation and patterns
 * - Coordinate system understanding
 * - Display memory organization
 * - Graphics programming fundamentals
 */

#ifdef GRAPHICS_BASICS

// Draw a simple pattern
void graphics_basics_draw_pattern(uint8_t pattern_type)
{
	GLCD_ClearScreen();

	switch (pattern_type)
	{
	case 0: // Checkerboard pattern
		uart_string("Drawing checkerboard pattern...\\r\\n");
		for (uint8_t x = 0; x < 128; x += 8)
		{
			for (uint8_t y = 0; y < 64; y += 8)
			{
				if (((x / 8) + (y / 8)) % 2 == 0)
				{
					for (uint8_t px = x; px < x + 8 && px < 128; px++)
					{
						for (uint8_t py = y; py < y + 8 && py < 64; py++)
						{
							GLCD_SetDot(px, py);
						}
					}
				}
			}
		}
		break;

	case 1: // Grid pattern
		uart_string("Drawing grid pattern...\\r\\n");
		for (uint8_t x = 0; x < 128; x += 16)
		{
			for (uint8_t y = 0; y < 64; y++)
			{
				GLCD_SetDot(x, y);
			}
		}
		for (uint8_t y = 0; y < 64; y += 8)
		{
			for (uint8_t x = 0; x < 128; x++)
			{
				GLCD_SetDot(x, y);
			}
		}
		break;

	case 2: // Diagonal lines
		uart_string("Drawing diagonal pattern...\\r\\n");
		for (uint8_t i = 0; i < 128; i++)
		{
			if (i < 64)
				GLCD_SetDot(i, i);
			if (i < 64)
				GLCD_SetDot(127 - i, i);
			if (i >= 64)
				GLCD_SetDot(i, 127 - i);
		}
		break;

	case 3: // Concentric rectangles
		uart_string("Drawing concentric rectangles...\\r\\n");
		for (uint8_t size = 4; size < 32; size += 4)
		{
			uint8_t x1 = 64 - size, y1 = 32 - size / 2;
			uint8_t x2 = 64 + size, y2 = 32 + size / 2;

			// Draw rectangle outline
			for (uint8_t x = x1; x <= x2 && x < 128; x++)
			{
				if (y1 < 64)
					GLCD_SetDot(x, y1);
				if (y2 < 64)
					GLCD_SetDot(x, y2);
			}
			for (uint8_t y = y1; y <= y2 && y < 64; y++)
			{
				if (x1 < 128)
					GLCD_SetDot(x1, y);
				if (x2 < 128)
					GLCD_SetDot(x2, y);
			}
		}
		break;
	}
}

// Demonstrate pixel manipulation
void graphics_basics_pixel_demo(void)
{
	uart_string("Pixel manipulation demonstration...\\r\\n");

	GLCD_ClearScreen();
	GLCD_WriteString("PIXEL DEMO");

	// Draw individual pixels in a spiral pattern
	uint8_t x = 64, y = 32;
	uint8_t dx = 1, dy = 0;
	uint8_t steps = 1;
	uint8_t step_count = 0;
	uint8_t direction_changes = 0;

	for (uint16_t i = 0; i < 500; i++)
	{
		if (x < 128 && y < 64)
		{
			GLCD_SetDot(x, y);
		}

		x += dx;
		y += dy;
		step_count++;

		// Change direction for spiral
		if (step_count >= steps)
		{
			step_count = 0;
			direction_changes++;

			// Rotate direction
			int8_t temp_dx = dx;
			dx = -dy;
			dy = temp_dx;

			// Increase steps every two direction changes
			if (direction_changes % 2 == 0)
			{
				steps++;
			}
		}

		_delay_ms(10);

		// Show progress on LEDs
		PORTB = i & 0xFF;
	}

	_delay_ms(1000);
}

// Demonstrate coordinate system
void graphics_basics_coordinate_demo(void)
{
	uart_string("Coordinate system demonstration...\\r\\n");

	GLCD_ClearScreen();
	GLCD_WriteString("COORDINATES");

	// Draw coordinate axes
	for (uint8_t x = 0; x < 128; x += 2)
	{
		GLCD_SetDot(x, 32); // X-axis
	}
	for (uint8_t y = 16; y < 64; y += 2)
	{
		GLCD_SetDot(64, y); // Y-axis
	}

	// Mark coordinate points
	uint8_t points[][2] = {
		{20, 20}, {40, 25}, {60, 35}, {80, 40}, {100, 45}};

	for (uint8_t i = 0; i < 5; i++)
	{
		uint8_t px = points[i][0];
		uint8_t py = points[i][1];

		// Draw a small cross at each point
		GLCD_SetDot(px, py);
		GLCD_SetDot(px - 1, py);
		GLCD_SetDot(px + 1, py);
		GLCD_SetDot(px, py - 1);
		GLCD_SetDot(px, py + 1);

		// Show coordinates on UART
		char coord_str[32];
		sprintf(coord_str, "Point %d: (%d, %d)\\r\\n", i + 1, px, py);
		uart_string(coord_str);

		_delay_ms(500);
	}

	_delay_ms(2000);
}

// Text and font demonstration
void graphics_basics_text_demo(void)
{
	uart_string("Text display demonstration...\\r\\n");

	GLCD_ClearScreen();

	// Display various text samples
	GLCD_WriteString("HELLO WORLD!");
	_delay_ms(1000);

	GLCD_SetDot(0, 16);
	GLCD_WriteString("Line 2: Numbers");
	_delay_ms(1000);

	GLCD_SetDot(0, 24);
	GLCD_WriteString("0123456789");
	_delay_ms(1000);

	GLCD_SetDot(0, 32);
	GLCD_WriteString("ABCDEFGHIJKLM");
	_delay_ms(1000);

	GLCD_SetDot(0, 40);
	GLCD_WriteString("nopqrstuvwxyz");
	_delay_ms(1000);

	GLCD_SetDot(0, 48);
	GLCD_WriteString("Symbols: !@#$%");
	_delay_ms(1000);

	GLCD_SetDot(0, 56);
	GLCD_WriteString("Graphics Ready!");

	_delay_ms(2000);
}

// Memory and display organization demo
void graphics_basics_memory_demo(void)
{
	uart_string("Display memory organization demo...\\r\\n");

	// Clear screen and show page structure
	GLCD_ClearScreen();
	GLCD_WriteString("MEMORY PAGES");

	// GLCD is organized in 8 pages of 8 pixels each
	for (uint8_t page = 0; page < 8; page++)
	{
		// Draw a line to show page boundary
		for (uint8_t x = 0; x < 128; x += 4)
		{
			GLCD_SetDot(x, page * 8 + 7);
		}

		// Show page number
		char page_str[8];
		sprintf(page_str, "P%d", page);
		uart_string("Drawing page ");
		uart_string(page_str);
		uart_string("\\r\\n");

		_delay_ms(300);
	}

	_delay_ms(1000);

	// Demonstrate byte-oriented operations
	GLCD_ClearScreen();
	GLCD_WriteString("BYTE PATTERNS");

	// Draw different bit patterns
	uint8_t patterns[] = {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

	for (uint8_t i = 0; i < 8; i++)
	{
		for (uint8_t x = i * 16; x < (i + 1) * 16; x++)
		{
			for (uint8_t bit = 0; bit < 8; bit++)
			{
				if (patterns[i] & (1 << bit))
				{
					GLCD_SetDot(x, 20 + bit);
				}
			}
		}
		_delay_ms(200);
	}

	_delay_ms(2000);
}

void main_graphics_basics(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS BASICS DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Basic GLCD drawing primitives\\r\\n");
	uart_string("- Pixel manipulation and patterns\\r\\n");
	uart_string("- Coordinate system understanding\\r\\n");
	uart_string("- Display memory organization\\r\\n");
	uart_string("- Graphics programming fundamentals\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("GRAPHICS BASICS");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Fundamentals");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("of Graphics");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("Programming");
	GLCD_SetDot(0, 56);
	GLCD_WriteString("Press button");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting graphics basics demonstrations...\\r\\n\\r\\n");

	// 1. Pattern demonstrations
	uart_string("1. Pattern Drawing Demonstrations\\r\\n");
	for (uint8_t pattern = 0; pattern < 4; pattern++)
	{
		graphics_basics_draw_pattern(pattern);
		_delay_ms(2000);
	}

	// 2. Pixel manipulation
	uart_string("\\r\\n2. Pixel Manipulation\\r\\n");
	graphics_basics_pixel_demo();

	// 3. Coordinate system
	uart_string("\\r\\n3. Coordinate System\\r\\n");
	graphics_basics_coordinate_demo();

	// 4. Text display
	uart_string("\\r\\n4. Text Display\\r\\n");
	graphics_basics_text_demo();

	// 5. Memory organization
	uart_string("\\r\\n5. Display Memory Organization\\r\\n");
	graphics_basics_memory_demo();

	// Final comprehensive demo
	uart_string("\\r\\n6. Comprehensive Graphics Demo\\r\\n");
	GLCD_ClearScreen();

	// Combine multiple techniques
	GLCD_WriteString("FINAL DEMO");

	// Background pattern
	for (uint8_t x = 0; x < 128; x += 16)
	{
		for (uint8_t y = 16; y < 64; y += 8)
		{
			GLCD_SetDot(x, y);
		}
	}

	// Animated elements
	for (uint8_t frame = 0; frame < 50; frame++)
	{
		// Moving dot
		uint8_t dot_x = 20 + frame;
		uint8_t dot_y = 30 + (uint8_t)(10 * sin(frame * 0.2));

		if (dot_x < 128 && dot_y < 64)
		{
			GLCD_SetDot(dot_x, dot_y);
		}

		// Rotating line
		float angle = frame * 0.1;
		uint8_t line_x = 80 + (uint8_t)(15 * cos(angle));
		uint8_t line_y = 40 + (uint8_t)(10 * sin(angle));

		if (line_x < 128 && line_y < 64)
		{
			GLCD_SetDot(line_x, line_y);
		}

		// Show frame count on LEDs
		PORTB = frame;

		_delay_ms(100);
	}

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("BASICS COMPLETE");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Fundamentals:");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("- Pixels");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("- Patterns");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("- Coordinates");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("- Memory");
	GLCD_SetDot(0, 56);
	GLCD_WriteString("- Text");

	uart_string("\\r\\nGraphics basics demonstration completed!\\r\\n");
	uart_string("Covered: pixels, patterns, coordinates, memory, text\\r\\n\\r\\n");

	while (1)
	{
		// Keep display active
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_BASICS

/*
 * GRAPHICS_MOVEMENT - Moving Graphics Elements
 * Educational demonstration of:
 * - Object movement and animation
 * - Physics-based motion (velocity, acceleration)
 * - Collision detection and response
 * - Smooth animation techniques
 * - Interactive motion control
 */

#ifdef GRAPHICS_MOVEMENT

#define MAX_MOVING_OBJECTS 6

// Moving object structure
typedef struct
{
	float x, y;		// Position (float for smooth movement)
	float vx, vy;	// Velocity
	float ax, ay;	// Acceleration
	uint8_t size;	// Object size
	uint8_t shape;	// 0=dot, 1=circle, 2=square, 3=triangle
	uint8_t active; // Object active flag
	uint8_t bounce; // Bounce off edges flag
	uint8_t trail;	// Leave trail flag
} moving_object_t;

static moving_object_t objects[MAX_MOVING_OBJECTS];
static uint16_t animation_frame = 0;

// Initialize moving objects
void graphics_movement_init_objects(void)
{
	// Object 1: Simple horizontal movement
	objects[0].x = 10;
	objects[0].y = 20;
	objects[0].vx = 1.5;
	objects[0].vy = 0;
	objects[0].ax = 0;
	objects[0].ay = 0;
	objects[0].size = 3;
	objects[0].shape = 0;
	objects[0].active = 1;
	objects[0].bounce = 1;
	objects[0].trail = 0;

	// Object 2: Diagonal movement with bounce
	objects[1].x = 30;
	objects[1].y = 10;
	objects[1].vx = 2.0;
	objects[1].vy = 1.5;
	objects[1].ax = 0;
	objects[1].ay = 0;
	objects[1].size = 4;
	objects[1].shape = 1;
	objects[1].active = 1;
	objects[1].bounce = 1;
	objects[1].trail = 0;

	// Object 3: Gravity simulation
	objects[2].x = 60;
	objects[2].y = 10;
	objects[2].vx = 1.0;
	objects[2].vy = 0;
	objects[2].ax = 0;
	objects[2].ay = 0.1; // Gravity
	objects[2].size = 3;
	objects[2].shape = 2;
	objects[2].active = 1;
	objects[2].bounce = 1;
	objects[2].trail = 1;

	// Object 4: Circular motion
	objects[3].x = 64;
	objects[3].y = 32;
	objects[3].vx = 0;
	objects[3].vy = 0;
	objects[3].ax = 0;
	objects[3].ay = 0;
	objects[3].size = 2;
	objects[3].shape = 3;
	objects[3].active = 1;
	objects[3].bounce = 0;
	objects[3].trail = 1;

	// Object 5: Oscillating movement
	objects[4].x = 100;
	objects[4].y = 50;
	objects[4].vx = 0;
	objects[4].vy = -2.0;
	objects[4].ax = 0;
	objects[4].ay = 0;
	objects[4].size = 2;
	objects[4].shape = 0;
	objects[4].active = 1;
	objects[4].bounce = 1;
	objects[4].trail = 0;

	// Object 6: Random walk
	objects[5].x = 80;
	objects[5].y = 40;
	objects[5].vx = 0.5;
	objects[5].vy = 0.3;
	objects[5].ax = 0;
	objects[5].ay = 0;
	objects[5].size = 1;
	objects[5].shape = 0;
	objects[5].active = 1;
	objects[5].bounce = 1;
	objects[5].trail = 1;

	animation_frame = 0;
}

// Update object physics
void graphics_movement_update_physics(void)
{
	for (uint8_t i = 0; i < MAX_MOVING_OBJECTS; i++)
	{
		if (!objects[i].active)
			continue;

		moving_object_t *obj = &objects[i];

		// Special movement patterns
		switch (i)
		{
		case 3: // Circular motion
		{
			float angle = animation_frame * 0.1;
			obj->x = 64 + 20 * cos(angle);
			obj->y = 32 + 15 * sin(angle);
		}
		break;

		case 5: // Random walk with occasional direction changes
			if (animation_frame % 30 == 0)
			{
				obj->vx += (float)(rand() % 200 - 100) / 100.0;
				obj->vy += (float)(rand() % 200 - 100) / 100.0;
				// Limit velocity
				if (obj->vx > 3.0)
					obj->vx = 3.0;
				if (obj->vx < -3.0)
					obj->vx = -3.0;
				if (obj->vy > 3.0)
					obj->vy = 3.0;
				if (obj->vy < -3.0)
					obj->vy = -3.0;
			}
			// Fall through to normal physics

		default:
			// Apply acceleration to velocity
			obj->vx += obj->ax;
			obj->vy += obj->ay;

			// Apply velocity to position
			obj->x += obj->vx;
			obj->y += obj->vy;
			break;
		}

		// Boundary collision detection
		if (obj->bounce)
		{
			if (obj->x <= obj->size || obj->x >= 128 - obj->size)
			{
				obj->vx = -obj->vx * 0.9; // Energy loss on bounce
				obj->x = (obj->x <= obj->size) ? obj->size : 128 - obj->size;
			}
			if (obj->y <= obj->size || obj->y >= 64 - obj->size)
			{
				obj->vy = -obj->vy * 0.9; // Energy loss on bounce
				obj->y = (obj->y <= obj->size) ? obj->size : 64 - obj->size;
			}
		}
		else
		{
			// Wrap around screen
			if (obj->x < 0)
				obj->x = 128;
			if (obj->x > 128)
				obj->x = 0;
			if (obj->y < 0)
				obj->y = 64;
			if (obj->y > 64)
				obj->y = 0;
		}
	}

	animation_frame++;
}

// Draw object based on shape
void graphics_movement_draw_object(moving_object_t *obj)
{
	uint8_t x = (uint8_t)obj->x;
	uint8_t y = (uint8_t)obj->y;
	uint8_t size = obj->size;

	switch (obj->shape)
	{
	case 0: // Dot
		GLCD_SetDot(x, y);
		break;

	case 1: // Circle
		GLCD_Circle(x, y, size);
		break;

	case 2: // Square
		for (uint8_t dx = 0; dx <= size * 2; dx++)
		{
			for (uint8_t dy = 0; dy <= size * 2; dy++)
			{
				uint8_t px = x - size + dx;
				uint8_t py = y - size + dy;
				if (px < 128 && py < 64)
				{
					if (dx == 0 || dx == size * 2 || dy == 0 || dy == size * 2)
					{
						GLCD_SetDot(px, py);
					}
				}
			}
		}
		break;

	case 3: // Triangle
		for (uint8_t i = 0; i < size; i++)
		{
			if (x - i / 2 < 128 && y + size - i < 64)
				GLCD_SetDot(x - i / 2, y + size - i);
			if (x + i / 2 < 128 && y + size - i < 64)
				GLCD_SetDot(x + i / 2, y + size - i);
		}
		for (uint8_t i = 0; i < size * 2; i++)
		{
			if (x - size + i < 128 && y + size < 64)
				GLCD_SetDot(x - size + i, y + size);
		}
		break;
	}
}

// Demonstration of linear movement
void graphics_movement_linear_demo(void)
{
	uart_string("Linear movement demonstration...\\r\\n");

	float obj_x = 10, obj_y = 32;
	float velocity = 2.0;

	for (uint16_t frame = 0; frame < 100; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("LINEAR MOTION");

		// Draw moving object
		GLCD_Circle((uint8_t)obj_x, (uint8_t)obj_y, 4);

		// Draw velocity vector
		uint8_t arrow_end_x = (uint8_t)(obj_x + velocity * 10);
		for (uint8_t x = (uint8_t)obj_x; x < arrow_end_x && x < 128; x++)
		{
			GLCD_SetDot(x, (uint8_t)obj_y - 1);
		}

		// Update position
		obj_x += velocity;
		if (obj_x > 118)
		{
			obj_x = 10;
		}

		// Show velocity on LEDs
		PORTB = (uint8_t)(velocity * 50);

		_delay_ms(100);
	}
}

// Demonstration of parabolic motion (projectile)
void graphics_movement_projectile_demo(void)
{
	uart_string("Projectile motion demonstration...\\r\\n");

	float obj_x = 20, obj_y = 50;
	float vx = 2.5, vy = -4.0; // Initial velocity
	float gravity = 0.2;

	for (uint16_t frame = 0; frame < 80; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("PROJECTILE");

		// Draw trajectory trail
		float trail_x = 20, trail_y = 50;
		float trail_vx = 2.5, trail_vy = -4.0;

		for (uint8_t t = 0; t < frame && t < 60; t++)
		{
			if (trail_x < 128 && trail_y < 64 && trail_y > 0 && t % 3 == 0)
			{
				GLCD_SetDot((uint8_t)trail_x, (uint8_t)trail_y);
			}
			trail_x += trail_vx;
			trail_vy += gravity;
			trail_y += trail_vy;
		}

		// Draw current object
		if (obj_x < 128 && obj_y < 64 && obj_y > 0)
		{
			GLCD_Circle((uint8_t)obj_x, (uint8_t)obj_y, 3);
		}

		// Update physics
		obj_x += vx;
		vy += gravity;
		obj_y += vy;

		// Reset if off screen
		if (obj_x > 128 || obj_y > 64)
		{
			obj_x = 20;
			obj_y = 50;
			vx = 2.5;
			vy = -4.0;
		}

		_delay_ms(80);
	}
}

// Demonstration of wave motion
void graphics_movement_wave_demo(void)
{
	uart_string("Wave motion demonstration...\\r\\n");

	for (uint16_t phase = 0; phase < 200; phase++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("WAVE MOTION");

		// Draw wave with moving particles
		for (uint8_t x = 0; x < 128; x += 8)
		{
			float wave_phase = (x * 0.1) + (phase * 0.1);
			uint8_t y = 32 + (uint8_t)(15 * sin(wave_phase));

			// Draw particle
			GLCD_Circle(x, y, 2);

			// Draw wave line
			if (x > 0)
			{
				float prev_wave_phase = ((x - 8) * 0.1) + (phase * 0.1);
				uint8_t prev_y = 32 + (uint8_t)(15 * sin(prev_wave_phase));

				// Simple line drawing between points
				for (uint8_t px = x - 8; px < x; px++)
				{
					uint8_t py = prev_y + ((y - prev_y) * (px - (x - 8))) / 8;
					if (py < 64)
						GLCD_SetDot(px, py);
				}
			}
		}

		// Show wave parameters
		PORTB = phase & 0xFF;

		_delay_ms(50);
	}
}

void main_graphics_movement(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS MOVEMENT DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Object movement and animation\\r\\n");
	uart_string("- Physics-based motion\\r\\n");
	uart_string("- Collision detection\\r\\n");
	uart_string("- Smooth animation techniques\\r\\n");
	uart_string("- Interactive motion control\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("MOVEMENT DEMO");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Physics-based");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("Object Motion");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("to start");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting movement demonstrations...\\r\\n\\r\\n");

	// 1. Linear movement demo
	uart_string("1. Linear Movement\\r\\n");
	graphics_movement_linear_demo();

	_delay_ms(1000);

	// 2. Projectile motion demo
	uart_string("\\r\\n2. Projectile Motion\\r\\n");
	graphics_movement_projectile_demo();

	_delay_ms(1000);

	// 3. Wave motion demo
	uart_string("\\r\\n3. Wave Motion\\r\\n");
	graphics_movement_wave_demo();

	_delay_ms(1000);

	// 4. Multi-object physics simulation
	uart_string("\\r\\n4. Multi-Object Physics Simulation\\r\\n");
	graphics_movement_init_objects();

	for (uint16_t frame = 0; frame < 300; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("MULTI-OBJECT");

		// Update all objects
		graphics_movement_update_physics();

		// Draw all objects
		for (uint8_t i = 0; i < MAX_MOVING_OBJECTS; i++)
		{
			if (objects[i].active)
			{
				graphics_movement_draw_object(&objects[i]);
			}
		}

		// Show simulation info
		PORTB = frame & 0xFF;

		_delay_ms(50);

		// Check for exit
		if (PIND & (1 << PD7))
			break;
	}

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("MOVEMENT");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Physics-based");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("object motion,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("collisions!");

	uart_string("\\r\\nGraphics movement demonstration completed!\\r\\n");
	uart_string("Demonstrated: linear, projectile, wave, multi-object\\r\\n\\r\\n");

	while (1)
	{
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_MOVEMENT

/*
 * GRAPHICS_RANDOM - Random Graphics Patterns
 * Educational demonstration of:
 * - Pseudo-random number generation for graphics
 * - Procedural pattern creation
 * - Noise and organic visual effects
 * - Generative art techniques
 * - Statistical distribution visualization
 */

#ifdef GRAPHICS_RANDOM

static uint32_t random_seed = 12345;

// Simple linear congruential generator for consistent results
uint16_t graphics_random_lcg(void)
{
	random_seed = (random_seed * 1103515245 + 12345) & 0x7fffffff;
	return (uint16_t)(random_seed >> 16);
}

// Set random seed
void graphics_random_seed(uint32_t seed)
{
	random_seed = seed;
}

// Generate random starfield
void graphics_random_starfield_demo(void)
{
	uart_string("Random starfield demonstration...\\r\\n");

	for (uint16_t frame = 0; frame < 100; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("STARFIELD");

		// Generate stars based on frame for animation
		graphics_random_seed(frame * 1337);

		// Draw random stars
		for (uint8_t star = 0; star < 50; star++)
		{
			uint8_t x = graphics_random_lcg() % 128;
			uint8_t y = 16 + (graphics_random_lcg() % 48); // Below title
			uint8_t brightness = graphics_random_lcg() % 4;

			// Draw star with brightness levels
			switch (brightness)
			{
			case 0: // Dim star
				GLCD_SetDot(x, y);
				break;
			case 1: // Medium star
				GLCD_SetDot(x, y);
				if (x > 0)
					GLCD_SetDot(x - 1, y);
				if (x < 127)
					GLCD_SetDot(x + 1, y);
				break;
			case 2: // Bright star
				GLCD_SetDot(x, y);
				if (x > 0)
					GLCD_SetDot(x - 1, y);
				if (x < 127)
					GLCD_SetDot(x + 1, y);
				if (y > 0)
					GLCD_SetDot(x, y - 1);
				if (y < 63)
					GLCD_SetDot(x, y + 1);
				break;
			case 3: // Twinkling star
				if (frame % 3 == star % 3)
				{
					GLCD_SetDot(x, y);
					if (x > 0)
						GLCD_SetDot(x - 1, y);
					if (x < 127)
						GLCD_SetDot(x + 1, y);
					if (y > 0)
						GLCD_SetDot(x, y - 1);
					if (y < 63)
						GLCD_SetDot(x, y + 1);
				}
				break;
			}
		}

		// Show frame count on LEDs
		PORTB = frame & 0xFF;

		_delay_ms(150);
	}
}

// Random walk demonstration
void graphics_random_walk_demo(void)
{
	uart_string("Random walk demonstration...\\r\\n");

	GLCD_ClearScreen();
	GLCD_WriteString("RANDOM WALK");

	uint8_t walker_x = 64, walker_y = 32;
	graphics_random_seed(42);

	for (uint16_t step = 0; step < 500; step++)
	{
		// Current position
		GLCD_SetDot(walker_x, walker_y);

		// Random step direction
		uint8_t direction = graphics_random_lcg() % 8;

		switch (direction)
		{
		case 0:
			if (walker_x < 127)
				walker_x++;
			break; // East
		case 1:
			if (walker_x > 0)
				walker_x--;
			break; // West
		case 2:
			if (walker_y < 63)
				walker_y++;
			break; // South
		case 3:
			if (walker_y > 16)
				walker_y--;
			break; // North
		case 4:
			if (walker_x < 127 && walker_y < 63)
			{
				walker_x++;
				walker_y++;
			}
			break; // SE
		case 5:
			if (walker_x > 0 && walker_y < 63)
			{
				walker_x--;
				walker_y++;
			}
			break; // SW
		case 6:
			if (walker_x < 127 && walker_y > 16)
			{
				walker_x++;
				walker_y--;
			}
			break; // NE
		case 7:
			if (walker_x > 0 && walker_y > 16)
			{
				walker_x--;
				walker_y--;
			}
			break; // NW
		}

		// Show progress
		if (step % 50 == 0)
		{
			PORTB = (step / 50) << 4;
		}

		_delay_ms(20);
	}

	_delay_ms(2000);
}

// Perlin-like noise demonstration
void graphics_random_noise_demo(void)
{
	uart_string("Noise pattern demonstration...\\r\\n");

	GLCD_ClearScreen();
	GLCD_WriteString("NOISE PATTERNS");

	// Generate noise-like pattern
	for (uint8_t x = 0; x < 128; x += 2)
	{
		for (uint8_t y = 16; y < 64; y += 2)
		{
			// Create pseudo-noise by combining multiple random sources
			graphics_random_seed(x * 137 + y * 239 + 1337);
			uint16_t noise1 = graphics_random_lcg();

			graphics_random_seed(x * 97 + y * 131 + 4321);
			uint16_t noise2 = graphics_random_lcg();

			// Combine noise sources
			uint16_t combined = (noise1 + noise2) / 2;

			// Threshold for pattern density
			if (combined % 100 < 30)
			{
				GLCD_SetDot(x, y);
			}
		}
	}

	_delay_ms(3000);
}

// Random geometric shapes
void graphics_random_shapes_demo(void)
{
	uart_string("Random geometric shapes...\\r\\n");

	for (uint16_t frame = 0; frame < 50; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("RANDOM SHAPES");

		graphics_random_seed(frame * 999);

		// Generate random shapes
		for (uint8_t shape = 0; shape < 8; shape++)
		{
			uint8_t x = graphics_random_lcg() % 108 + 10;
			uint8_t y = graphics_random_lcg() % 40 + 20;
			uint8_t size = graphics_random_lcg() % 8 + 2;
			uint8_t shape_type = graphics_random_lcg() % 4;

			switch (shape_type)
			{
			case 0: // Random dots cluster
				for (uint8_t i = 0; i < size; i++)
				{
					uint8_t dx = graphics_random_lcg() % (size * 2) - size;
					uint8_t dy = graphics_random_lcg() % (size * 2) - size;
					if (x + dx < 128 && y + dy < 64)
					{
						GLCD_SetDot(x + dx, y + dy);
					}
				}
				break;

			case 1: // Random circle
				GLCD_Circle(x, y, size);
				break;

			case 2: // Random square
				for (uint8_t i = 0; i < size * 2; i++)
				{
					for (uint8_t j = 0; j < size * 2; j++)
					{
						if (x + i - size < 128 && y + j - size < 64)
						{
							if (i == 0 || i == size * 2 - 1 || j == 0 || j == size * 2 - 1)
							{
								GLCD_SetDot(x + i - size, y + j - size);
							}
						}
					}
				}
				break;

			case 3: // Random line
			{
				uint8_t x2 = x + (graphics_random_lcg() % (size * 4)) - size * 2;
				uint8_t y2 = y + (graphics_random_lcg() % (size * 4)) - size * 2;

				// Simple line drawing
				if (x2 < 128 && y2 < 64)
				{
					uint8_t steps = (abs(x2 - x) > abs(y2 - y)) ? abs(x2 - x) : abs(y2 - y);
					if (steps > 0)
					{
						for (uint8_t step = 0; step <= steps; step++)
						{
							uint8_t lx = x + ((x2 - x) * step) / steps;
							uint8_t ly = y + ((y2 - y) * step) / steps;
							if (lx < 128 && ly < 64)
							{
								GLCD_SetDot(lx, ly);
							}
						}
					}
				}
			}
			break;
			}
		}

		PORTB = frame;
		_delay_ms(200);
	}
}

// Random fractal tree
void graphics_random_tree_demo(void)
{
	uart_string("Random fractal tree...\\r\\n");

	GLCD_ClearScreen();
	GLCD_WriteString("FRACTAL TREE");

	// Simple recursive tree simulation (limited depth for microcontroller)
	graphics_random_seed(1234);

	// Tree trunk
	uint8_t trunk_x = 64, trunk_y = 60;
	uint8_t trunk_top_x = 64, trunk_top_y = 45;

	// Draw trunk
	for (uint8_t y = trunk_y; y >= trunk_top_y; y--)
	{
		GLCD_SetDot(trunk_x, y);
		GLCD_SetDot(trunk_x + 1, y);
	}

	// Generate branches
	for (uint8_t branch = 0; branch < 12; branch++)
	{
		uint8_t start_x = trunk_top_x + (graphics_random_lcg() % 3) - 1;
		uint8_t start_y = trunk_top_y - (graphics_random_lcg() % 10);

		uint8_t length = 8 + (graphics_random_lcg() % 12);
		uint8_t angle_offset = (graphics_random_lcg() % 60) - 30; // ±30 degrees

		// Draw branch
		for (uint8_t i = 0; i < length; i++)
		{
			uint8_t branch_x = start_x + (i * (angle_offset + 90)) / 100; // Simplified trig
			uint8_t branch_y = start_y - i;

			if (branch_x < 128 && branch_y < 64)
			{
				GLCD_SetDot(branch_x, branch_y);

				// Add small sub-branches
				if (i > length / 2 && (graphics_random_lcg() % 4) == 0)
				{
					uint8_t sub_x = branch_x + (graphics_random_lcg() % 6) - 3;
					uint8_t sub_y = branch_y - (graphics_random_lcg() % 4);
					if (sub_x < 128 && sub_y < 64)
					{
						GLCD_SetDot(sub_x, sub_y);
					}
				}
			}
		}
	}

	_delay_ms(3000);
}

// Random cellular automata
void graphics_random_cellular_demo(void)
{
	uart_string("Cellular automata demonstration...\\r\\n");

	uint8_t cells[128];
	uint8_t next_cells[128];

	// Initialize with random pattern
	graphics_random_seed(777);
	for (uint8_t i = 0; i < 128; i++)
	{
		cells[i] = (graphics_random_lcg() % 100) < 30 ? 1 : 0;
	}

	for (uint8_t generation = 0; generation < 40; generation++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("CELLULAR AUTO");

		// Draw current generation
		for (uint8_t x = 0; x < 128; x++)
		{
			if (cells[x])
			{
				GLCD_SetDot(x, 20 + generation);
			}
		}

		// Calculate next generation (Rule 30-like)
		for (uint8_t i = 0; i < 128; i++)
		{
			uint8_t left = (i == 0) ? 0 : cells[i - 1];
			uint8_t center = cells[i];
			uint8_t right = (i == 127) ? 0 : cells[i + 1];

			uint8_t pattern = (left << 2) | (center << 1) | right;

			// Rule 30: 00011110 in binary
			next_cells[i] = (0x1E >> pattern) & 1;
		}

		// Copy next generation to current
		for (uint8_t i = 0; i < 128; i++)
		{
			cells[i] = next_cells[i];
		}

		PORTB = generation << 3;
		_delay_ms(300);

		if (generation >= 40)
			break;
	}

	_delay_ms(2000);
}

void main_graphics_random(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS RANDOM DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Pseudo-random number generation\\r\\n");
	uart_string("- Procedural pattern creation\\r\\n");
	uart_string("- Noise and organic effects\\r\\n");
	uart_string("- Generative art techniques\\r\\n");
	uart_string("- Statistical visualization\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("RANDOM GRAPHICS");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Procedural");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("Pattern Gen");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("to start");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting random graphics demonstrations...\\r\\n\\r\\n");

	// 1. Starfield
	uart_string("1. Random Starfield\\r\\n");
	graphics_random_starfield_demo();

	_delay_ms(1000);

	// 2. Random walk
	uart_string("\\r\\n2. Random Walk\\r\\n");
	graphics_random_walk_demo();

	// 3. Noise patterns
	uart_string("\\r\\n3. Noise Patterns\\r\\n");
	graphics_random_noise_demo();

	// 4. Random shapes
	uart_string("\\r\\n4. Random Geometric Shapes\\r\\n");
	graphics_random_shapes_demo();

	_delay_ms(1000);

	// 5. Fractal tree
	uart_string("\\r\\n5. Random Fractal Tree\\r\\n");
	graphics_random_tree_demo();

	// 6. Cellular automata
	uart_string("\\r\\n6. Cellular Automata\\r\\n");
	graphics_random_cellular_demo();

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("RANDOM GRAPHICS");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Procedural gen,");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("fractals, noise,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("cellular auto!");

	uart_string("\\r\\nRandom graphics demonstration completed!\\r\\n");
	uart_string("Demonstrated: starfield, walk, noise, shapes, tree, cellular\\r\\n\\r\\n");

	while (1)
	{
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_RANDOM

/*
 * GRAPHICS_BOUNCING_BALL - Bouncing Ball Animation
 * Educational demonstration of:
 * - Physics simulation (velocity, acceleration, gravity)
 * - Collision detection and response
 * - Smooth animation techniques
 * - Trail effects and visual feedback
 * - Interactive parameter control
 */

#ifdef GRAPHICS_BOUNCING_BALL

// Ball physics structure
typedef struct
{
	float x, y;			  // Position (float for smooth movement)
	float vx, vy;		  // Velocity
	float ax, ay;		  // Acceleration
	uint8_t radius;		  // Ball radius
	uint8_t trail_length; // Trail effect length
	uint8_t active;		  // Ball active flag
	uint8_t color;		  // Ball display pattern
} bouncing_ball_t;

#define MAX_BALLS 4
#define MAX_TRAIL_POINTS 20

static bouncing_ball_t balls[MAX_BALLS];
static uint16_t trail_x[MAX_BALLS][MAX_TRAIL_POINTS];
static uint16_t trail_y[MAX_BALLS][MAX_TRAIL_POINTS];
static uint8_t trail_index[MAX_BALLS];
static float gravity_strength = 0.2;
static float energy_loss = 0.95; // Energy retained after bounce
static uint16_t animation_frame = 0;

// Initialize bouncing balls
void bouncing_ball_init(void)
{
	// Ball 1: Classic bouncing ball
	balls[0].x = 30;
	balls[0].y = 20;
	balls[0].vx = 2.0;
	balls[0].vy = -1.0;
	balls[0].ax = 0;
	balls[0].ay = gravity_strength;
	balls[0].radius = 4;
	balls[0].trail_length = 15;
	balls[0].active = 1;
	balls[0].color = 1;

	// Ball 2: Faster horizontal movement
	balls[1].x = 60;
	balls[1].y = 15;
	balls[1].vx = 3.5;
	balls[1].vy = 0;
	balls[1].ax = 0;
	balls[1].ay = gravity_strength;
	balls[1].radius = 3;
	balls[1].trail_length = 10;
	balls[1].active = 1;
	balls[1].color = 2;

	// Ball 3: Different starting conditions
	balls[2].x = 90;
	balls[2].y = 30;
	balls[2].vx = -2.5;
	balls[2].vy = -2.0;
	balls[2].ax = 0;
	balls[2].ay = gravity_strength;
	balls[2].radius = 5;
	balls[2].trail_length = 8;
	balls[2].active = 1;
	balls[2].color = 3;

	// Ball 4: Low gravity ball
	balls[3].x = 110;
	balls[3].y = 25;
	balls[3].vx = -1.5;
	balls[3].vy = -1.5;
	balls[3].ax = 0;
	balls[3].ay = gravity_strength * 0.5; // Half gravity
	balls[3].radius = 2;
	balls[3].trail_length = 20;
	balls[3].active = 1;
	balls[3].color = 4;

	// Initialize trail arrays
	for (uint8_t i = 0; i < MAX_BALLS; i++)
	{
		trail_index[i] = 0;
		for (uint8_t j = 0; j < MAX_TRAIL_POINTS; j++)
		{
			trail_x[i][j] = (uint16_t)balls[i].x;
			trail_y[i][j] = (uint16_t)balls[i].y;
		}
	}

	animation_frame = 0;
}

// Update ball physics
void bouncing_ball_update_physics(void)
{
	for (uint8_t i = 0; i < MAX_BALLS; i++)
	{
		if (!balls[i].active)
			continue;

		bouncing_ball_t *ball = &balls[i];

		// Apply acceleration to velocity
		ball->vx += ball->ax;
		ball->vy += ball->ay;

		// Apply velocity to position
		ball->x += ball->vx;
		ball->y += ball->vy;

		// Collision detection with boundaries

		// Left and right walls
		if (ball->x <= ball->radius)
		{
			ball->x = ball->radius;
			ball->vx = -ball->vx * energy_loss;
			buzzer_play_frequency(800, 30); // Bounce sound
		}
		if (ball->x >= 128 - ball->radius)
		{
			ball->x = 128 - ball->radius;
			ball->vx = -ball->vx * energy_loss;
			buzzer_play_frequency(800, 30); // Bounce sound
		}

		// Top and bottom walls
		if (ball->y <= ball->radius + 16)
		{ // Account for title area
			ball->y = ball->radius + 16;
			ball->vy = -ball->vy * energy_loss;
			buzzer_play_frequency(600, 30); // Different bounce sound
		}
		if (ball->y >= 64 - ball->radius)
		{
			ball->y = 64 - ball->radius;
			ball->vy = -ball->vy * energy_loss;
			buzzer_play_frequency(600, 30); // Different bounce sound
		}

		// Update trail
		trail_x[i][trail_index[i]] = (uint16_t)ball->x;
		trail_y[i][trail_index[i]] = (uint16_t)ball->y;
		trail_index[i] = (trail_index[i] + 1) % MAX_TRAIL_POINTS;
	}

	animation_frame++;
}

// Draw ball with trail effect
void bouncing_ball_draw_ball(uint8_t ball_idx)
{
	if (!balls[ball_idx].active)
		return;

	bouncing_ball_t *ball = &balls[ball_idx];
	uint8_t x = (uint8_t)ball->x;
	uint8_t y = (uint8_t)ball->y;

	// Draw trail with fading effect
	for (uint8_t i = 0; i < ball->trail_length && i < MAX_TRAIL_POINTS; i++)
	{
		uint8_t trail_idx = (trail_index[ball_idx] + MAX_TRAIL_POINTS - i - 1) % MAX_TRAIL_POINTS;
		uint8_t tx = trail_x[ball_idx][trail_idx];
		uint8_t ty = trail_y[ball_idx][trail_idx];

		// Draw trail point with diminishing intensity
		if (tx < 128 && ty < 64 && i % (ball_idx + 2) == 0)
		{ // Different patterns per ball
			GLCD_SetDot(tx, ty);
		}
	}

	// Draw ball based on color/pattern
	switch (ball->color)
	{
	case 1: // Solid circle
		GLCD_Circle(x, y, ball->radius);
		break;

	case 2: // Filled circle (approximation)
		for (uint8_t r = 0; r <= ball->radius; r++)
		{
			for (uint8_t angle = 0; angle < 8; angle++)
			{
				uint8_t px = x + (r * cos(angle * M_PI / 4));
				uint8_t py = y + (r * sin(angle * M_PI / 4));
				if (px < 128 && py < 64)
				{
					GLCD_SetDot(px, py);
				}
			}
		}
		break;

	case 3: // Square ball
		for (uint8_t dx = 0; dx < ball->radius * 2; dx++)
		{
			for (uint8_t dy = 0; dy < ball->radius * 2; dy++)
			{
				uint8_t px = x - ball->radius + dx;
				uint8_t py = y - ball->radius + dy;
				if (px < 128 && py < 64)
				{
					if (dx == 0 || dx == ball->radius * 2 - 1 ||
						dy == 0 || dy == ball->radius * 2 - 1)
					{
						GLCD_SetDot(px, py);
					}
				}
			}
		}
		break;

	case 4: // Cross pattern
		for (uint8_t i = 0; i < ball->radius; i++)
		{
			if (x + i < 128)
				GLCD_SetDot(x + i, y);
			if (x - i < 128)
				GLCD_SetDot(x - i, y);
			if (y + i < 64)
				GLCD_SetDot(x, y + i);
			if (y - i < 64)
				GLCD_SetDot(x, y - i);
		}
		break;
	}
}

// Single ball demonstration with physics explanation
void bouncing_ball_physics_demo(void)
{
	uart_string("Physics demonstration with single ball...\\r\\n");

	float ball_x = 64, ball_y = 20;
	float ball_vx = 3.0, ball_vy = 0;
	float ball_radius = 6;

	for (uint16_t frame = 0; frame < 200; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("PHYSICS DEMO");

		// Draw current ball
		GLCD_Circle((uint8_t)ball_x, (uint8_t)ball_y, (uint8_t)ball_radius);

		// Draw velocity vector
		uint8_t arrow_end_x = (uint8_t)(ball_x + ball_vx * 8);
		uint8_t arrow_end_y = (uint8_t)(ball_y + ball_vy * 8);

		// Simple line for velocity vector
		for (uint8_t i = 0; i < 8; i++)
		{
			uint8_t vec_x = (uint8_t)(ball_x + (ball_vx * i));
			uint8_t vec_y = (uint8_t)(ball_y + (ball_vy * i));
			if (vec_x < 128 && vec_y < 64)
			{
				GLCD_SetDot(vec_x, vec_y);
			}
		}

		// Update physics
		ball_vy += gravity_strength; // Apply gravity
		ball_x += ball_vx;
		ball_y += ball_vy;

		// Bounce off walls
		if (ball_x <= ball_radius || ball_x >= 128 - ball_radius)
		{
			ball_vx = -ball_vx * 0.9;
			ball_x = (ball_x <= ball_radius) ? ball_radius : 128 - ball_radius;
		}
		if (ball_y <= ball_radius + 16 || ball_y >= 64 - ball_radius)
		{
			ball_vy = -ball_vy * 0.9;
			ball_y = (ball_y <= ball_radius + 16) ? ball_radius + 16 : 64 - ball_radius;
		}

		// Display physics info
		PORTB = (uint8_t)(abs(ball_vx) + abs(ball_vy)) * 20; // Speed indicator

		_delay_ms(50);
	}
}

// Interactive ball control demo
void bouncing_ball_interactive_demo(void)
{
	uart_string("Interactive ball control...\\r\\n");
	uart_string("Use buttons to control ball physics:\\r\\n");
	uart_string("PD0 - Add velocity, PD1 - Change gravity\\r\\n");

	float ball_x = 64, ball_y = 32;
	float ball_vx = 0, ball_vy = 0;
	float ball_radius = 5;
	float current_gravity = 0.1;

	uint8_t prev_buttons = 0xFF;

	for (uint16_t frame = 0; frame < 400; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("INTERACTIVE");

		// Read button input
		uint8_t current_buttons = PIND;
		uint8_t button_pressed = (~current_buttons) & prev_buttons;

		// Handle button presses
		if (button_pressed & (1 << PD0))
		{
			ball_vx += (frame % 2 == 0) ? 2.0 : -2.0; // Add random velocity
			ball_vy -= 3.0;							  // Upward impulse
			uart_string("Velocity added!\\r\\n");
		}

		if (button_pressed & (1 << PD1))
		{
			current_gravity = (current_gravity > 0.3) ? 0.05 : current_gravity + 0.1;
			uart_string("Gravity changed!\\r\\n");
		}

		prev_buttons = current_buttons;

		// Draw ball
		GLCD_Circle((uint8_t)ball_x, (uint8_t)ball_y, (uint8_t)ball_radius);

		// Draw gravity indicator
		for (uint8_t i = 0; i < (uint8_t)(current_gravity * 50); i++)
		{
			GLCD_SetDot(5, 20 + i);
		}

		// Update physics
		ball_vy += current_gravity;
		ball_x += ball_vx;
		ball_y += ball_vy;

		// Apply air resistance
		ball_vx *= 0.999;
		ball_vy *= 0.999;

		// Boundary collisions
		if (ball_x <= ball_radius || ball_x >= 128 - ball_radius)
		{
			ball_vx = -ball_vx * 0.8;
			ball_x = (ball_x <= ball_radius) ? ball_radius : 128 - ball_radius;
		}
		if (ball_y <= ball_radius + 16 || ball_y >= 64 - ball_radius)
		{
			ball_vy = -ball_vy * 0.8;
			ball_y = (ball_y <= ball_radius + 16) ? ball_radius + 16 : 64 - ball_radius;
		}

		PORTB = frame & 0xFF;
		_delay_ms(40);

		if (button_pressed & (1 << PD7))
			break;
	}
}

void main_graphics_bouncing_ball(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS BOUNCING BALL DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Physics simulation (gravity, velocity)\\r\\n");
	uart_string("- Collision detection and response\\r\\n");
	uart_string("- Smooth animation techniques\\r\\n");
	uart_string("- Trail effects and visual feedback\\r\\n");
	uart_string("- Interactive parameter control\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("BOUNCING BALL");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Physics Sim");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("Gravity, Bounce");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("to start");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting bouncing ball demonstrations...\\r\\n\\r\\n");

	// 1. Single ball physics demo
	uart_string("1. Physics Explanation Demo\\r\\n");
	bouncing_ball_physics_demo();

	_delay_ms(1000);

	// 2. Multi-ball simulation
	uart_string("\\r\\n2. Multi-Ball Simulation\\r\\n");
	bouncing_ball_init();

	for (uint16_t frame = 0; frame < 300; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("MULTI-BALL");

		// Update all balls
		bouncing_ball_update_physics();

		// Draw all balls with trails
		for (uint8_t i = 0; i < MAX_BALLS; i++)
		{
			bouncing_ball_draw_ball(i);
		}

		// Show simulation stats
		PORTB = frame & 0xFF;

		_delay_ms(60);

		// Check for exit
		if (PIND & (1 << PD7))
			break;
	}

	_delay_ms(1000);

	// 3. Interactive control demo
	uart_string("\\r\\n3. Interactive Control\\r\\n");
	bouncing_ball_interactive_demo();

	// 4. Parameter variation demo
	uart_string("\\r\\n4. Parameter Variation Demo\\r\\n");

	// Different gravity settings
	float gravity_values[] = {0.05, 0.15, 0.3, 0.5};

	for (uint8_t g = 0; g < 4; g++)
	{
		gravity_strength = gravity_values[g];
		bouncing_ball_init();

		uart_string("Gravity: ");
		char gravity_str[16];
		sprintf(gravity_str, "%.2f\\r\\n", gravity_strength);
		uart_string(gravity_str);

		for (uint16_t frame = 0; frame < 80; frame++)
		{
			GLCD_ClearScreen();
			GLCD_WriteString("GRAVITY TEST");

			bouncing_ball_update_physics();

			// Draw only first ball for clarity
			bouncing_ball_draw_ball(0);

			// Show gravity level
			for (uint8_t i = 0; i < (uint8_t)(gravity_strength * 20); i++)
			{
				GLCD_SetDot(120, 60 - i);
			}

			_delay_ms(50);
		}

		_delay_ms(500);
	}

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("BOUNCING BALL");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Physics sim,");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("collisions,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("trails, control!");

	uart_string("\\r\\nBouncing ball demonstration completed!\\r\\n");
	uart_string("Demonstrated: physics, collisions, trails, interaction\\r\\n\\r\\n");

	while (1)
	{
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_BOUNCING_BALL

/*
 * GRAPHICS_MOVING_SQUARE - Moving Square Animation
 * Educational demonstration of:
 * - Geometric shape animation
 * - Rotation and transformation effects
 * - Path following and waypoint navigation
 * - Multi-square coordination
 * - Interactive shape control
 */

#ifdef GRAPHICS_MOVING_SQUARE

#define MAX_SQUARES 5

// Moving square structure
typedef struct
{
	float x, y;			  // Center position
	float vx, vy;		  // Velocity
	uint8_t size;		  // Square size (half-width)
	float rotation;		  // Rotation angle (0-360 degrees)
	float rotation_speed; // Rotation speed
	uint8_t pattern;	  // Fill pattern (0=outline, 1=filled, 2=cross, 3=dots)
	uint8_t active;		  // Square active flag
	uint8_t path_mode;	  // 0=linear, 1=circular, 2=figure-8, 3=random
	float path_param;	  // Path parameter for complex movements
} moving_square_t;

static moving_square_t squares[MAX_SQUARES];
static uint16_t animation_frame = 0;

// Initialize moving squares
void moving_square_init(void)
{
	// Square 1: Simple linear movement
	squares[0].x = 20;
	squares[0].y = 30;
	squares[0].vx = 1.5;
	squares[0].vy = 0.5;
	squares[0].size = 6;
	squares[0].rotation = 0;
	squares[0].rotation_speed = 2.0;
	squares[0].pattern = 0;
	squares[0].active = 1;
	squares[0].path_mode = 0;
	squares[0].path_param = 0;

	// Square 2: Rotating filled square
	squares[1].x = 60;
	squares[1].y = 40;
	squares[1].vx = -1.0;
	squares[1].vy = 1.0;
	squares[1].size = 8;
	squares[1].rotation = 45;
	squares[1].rotation_speed = -3.0;
	squares[1].pattern = 1;
	squares[1].active = 1;
	squares[1].path_mode = 0;
	squares[1].path_param = 0;

	// Square 3: Circular path
	squares[2].x = 64;
	squares[2].y = 32;
	squares[2].vx = 0;
	squares[2].vy = 0;
	squares[2].size = 4;
	squares[2].rotation = 0;
	squares[2].rotation_speed = 5.0;
	squares[2].pattern = 2;
	squares[2].active = 1;
	squares[2].path_mode = 1;
	squares[2].path_param = 0;

	// Square 4: Figure-8 path
	squares[3].x = 80;
	squares[3].y = 35;
	squares[3].vx = 0;
	squares[3].vy = 0;
	squares[3].size = 5;
	squares[3].rotation = 0;
	squares[3].rotation_speed = 1.0;
	squares[3].pattern = 3;
	squares[3].active = 1;
	squares[3].path_mode = 2;
	squares[3].path_param = 0;

	// Square 5: Random movement
	squares[4].x = 100;
	squares[4].y = 25;
	squares[4].vx = 0.8;
	squares[4].vy = -0.6;
	squares[4].size = 3;
	squares[4].rotation = 30;
	squares[4].rotation_speed = 4.0;
	squares[4].pattern = 0;
	squares[4].active = 1;
	squares[4].path_mode = 3;
	squares[4].path_param = 0;

	animation_frame = 0;
}

// Update square movements
void moving_square_update_physics(void)
{
	for (uint8_t i = 0; i < MAX_SQUARES; i++)
	{
		if (!squares[i].active)
			continue;

		moving_square_t *sq = &squares[i];

		// Update rotation
		sq->rotation += sq->rotation_speed;
		if (sq->rotation >= 360)
			sq->rotation -= 360;
		if (sq->rotation < 0)
			sq->rotation += 360;

		// Update position based on path mode
		switch (sq->path_mode)
		{
		case 0: // Linear movement
			sq->x += sq->vx;
			sq->y += sq->vy;

			// Bounce off boundaries
			if (sq->x <= sq->size || sq->x >= 128 - sq->size)
			{
				sq->vx = -sq->vx;
				sq->x = (sq->x <= sq->size) ? sq->size : 128 - sq->size;
			}
			if (sq->y <= sq->size + 16 || sq->y >= 64 - sq->size)
			{
				sq->vy = -sq->vy;
				sq->y = (sq->y <= sq->size + 16) ? sq->size + 16 : 64 - sq->size;
			}
			break;

		case 1: // Circular path
			sq->path_param += 0.1;
			sq->x = 64 + 25 * cos(sq->path_param);
			sq->y = 32 + 12 * sin(sq->path_param);
			break;

		case 2: // Figure-8 path
			sq->path_param += 0.08;
			sq->x = 80 + 20 * sin(sq->path_param);
			sq->y = 35 + 10 * sin(sq->path_param * 2);
			break;

		case 3: // Random walk with momentum
			// Occasional direction changes
			if (animation_frame % 40 == i * 8)
			{
				sq->vx += (float)(rand() % 200 - 100) / 200.0;
				sq->vy += (float)(rand() % 200 - 100) / 200.0;

				// Limit velocity
				if (sq->vx > 2.0)
					sq->vx = 2.0;
				if (sq->vx < -2.0)
					sq->vx = -2.0;
				if (sq->vy > 2.0)
					sq->vy = 2.0;
				if (sq->vy < -2.0)
					sq->vy = -2.0;
			}

			sq->x += sq->vx;
			sq->y += sq->vy;

			// Wrap around screen
			if (sq->x < 0)
				sq->x = 128;
			if (sq->x > 128)
				sq->x = 0;
			if (sq->y < 16)
				sq->y = 64;
			if (sq->y > 64)
				sq->y = 16;
			break;
		}

		sq->path_param += 0.01;
	}

	animation_frame++;
}

// Draw rotated square (simplified rotation)
void moving_square_draw_square(moving_square_t *sq)
{
	uint8_t cx = (uint8_t)sq->x;
	uint8_t cy = (uint8_t)sq->y;
	uint8_t size = sq->size;

	// Simple rotation approximation (45-degree increments)
	uint8_t rot_index = ((uint8_t)(sq->rotation / 45)) % 8;

	switch (sq->pattern)
	{
	case 0: // Outline square
		if (rot_index % 2 == 0)
		{
			// Regular square orientation
			for (uint8_t i = 0; i < size * 2; i++)
			{
				// Top and bottom edges
				if (cx - size + i < 128 && cy - size < 64)
					GLCD_SetDot(cx - size + i, cy - size);
				if (cx - size + i < 128 && cy + size < 64)
					GLCD_SetDot(cx - size + i, cy + size);
				// Left and right edges
				if (cx - size < 128 && cy - size + i < 64)
					GLCD_SetDot(cx - size, cy - size + i);
				if (cx + size < 128 && cy - size + i < 64)
					GLCD_SetDot(cx + size, cy - size + i);
			}
		}
		else
		{
			// Diamond orientation (45-degree rotation)
			for (uint8_t i = 0; i < size; i++)
			{
				// Diamond outline
				if (cx - i < 128 && cy - size + i < 64)
					GLCD_SetDot(cx - i, cy - size + i);
				if (cx + i < 128 && cy - size + i < 64)
					GLCD_SetDot(cx + i, cy - size + i);
				if (cx - i < 128 && cy + size - i < 64)
					GLCD_SetDot(cx - i, cy + size - i);
				if (cx + i < 128 && cy + size - i < 64)
					GLCD_SetDot(cx + i, cy + size - i);
			}
		}
		break;

	case 1: // Filled square
		if (rot_index % 2 == 0)
		{
			// Filled regular square
			for (uint8_t i = 0; i < size * 2; i++)
			{
				for (uint8_t j = 0; j < size * 2; j++)
				{
					uint8_t px = cx - size + i;
					uint8_t py = cy - size + j;
					if (px < 128 && py < 64)
					{
						GLCD_SetDot(px, py);
					}
				}
			}
		}
		else
		{
			// Filled diamond
			for (uint8_t i = 0; i < size; i++)
			{
				for (uint8_t j = 0; j <= i; j++)
				{
					// Top half
					if (cx - j < 128 && cy - size + i < 64)
						GLCD_SetDot(cx - j, cy - size + i);
					if (cx + j < 128 && cy - size + i < 64)
						GLCD_SetDot(cx + j, cy - size + i);
					// Bottom half
					if (cx - j < 128 && cy + size - i < 64)
						GLCD_SetDot(cx - j, cy + size - i);
					if (cx + j < 128 && cy + size - i < 64)
						GLCD_SetDot(cx + j, cy + size - i);
				}
			}
		}
		break;

	case 2: // Cross pattern
		for (uint8_t i = 0; i < size * 2; i++)
		{
			// Horizontal line
			if (cx - size + i < 128 && cy < 64)
				GLCD_SetDot(cx - size + i, cy);
			// Vertical line
			if (cx < 128 && cy - size + i < 64)
				GLCD_SetDot(cx, cy - size + i);
		}
		// Diagonal lines for rotation effect
		if (rot_index % 2 == 1)
		{
			for (uint8_t i = 0; i < size; i++)
			{
				if (cx - i < 128 && cy - i < 64)
					GLCD_SetDot(cx - i, cy - i);
				if (cx + i < 128 && cy - i < 64)
					GLCD_SetDot(cx + i, cy - i);
				if (cx - i < 128 && cy + i < 64)
					GLCD_SetDot(cx - i, cy + i);
				if (cx + i < 128 && cy + i < 64)
					GLCD_SetDot(cx + i, cy + i);
			}
		}
		break;

	case 3: // Dot pattern
		for (uint8_t i = 0; i < size * 2; i += 2)
		{
			for (uint8_t j = 0; j < size * 2; j += 2)
			{
				uint8_t px = cx - size + i;
				uint8_t py = cy - size + j;
				if (px < 128 && py < 64)
				{
					GLCD_SetDot(px, py);
				}
			}
		}
		break;
	}
}

// Single square transformation demo
void moving_square_transformation_demo(void)
{
	uart_string("Square transformation demonstration...\\r\\n");

	moving_square_t demo_square;
	demo_square.x = 64;
	demo_square.y = 32;
	demo_square.size = 10;
	demo_square.rotation = 0;
	demo_square.pattern = 0;
	demo_square.active = 1;

	for (uint16_t frame = 0; frame < 180; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("TRANSFORM");

		// Animate size and rotation
		demo_square.size = 5 + (uint8_t)(3 * sin(frame * 0.1));
		demo_square.rotation = frame * 2;
		demo_square.pattern = (frame / 45) % 4; // Change pattern every 45 frames

		// Draw the transforming square
		moving_square_draw_square(&demo_square);

		// Draw transformation info
		PORTB = frame & 0xFF;

		_delay_ms(80);
	}
}

// Path following demonstration
void moving_square_path_demo(void)
{
	uart_string("Path following demonstration...\\r\\n");

	moving_square_t path_square;
	path_square.size = 4;
	path_square.rotation = 0;
	path_square.rotation_speed = 3.0;
	path_square.pattern = 1;
	path_square.active = 1;
	path_square.path_param = 0;

	// Demo different path types
	for (uint8_t path_type = 0; path_type < 4; path_type++)
	{
		path_square.path_mode = path_type;
		path_square.path_param = 0;
		path_square.x = 64;
		path_square.y = 32;
		path_square.vx = 1.5;
		path_square.vy = 1.0;

		char path_name[20];
		switch (path_type)
		{
		case 0:
			strcpy(path_name, "Linear");
			break;
		case 1:
			strcpy(path_name, "Circular");
			break;
		case 2:
			strcpy(path_name, "Figure-8");
			break;
		case 3:
			strcpy(path_name, "Random");
			break;
		}

		uart_string("Path type: ");
		uart_string(path_name);
		uart_string("\\r\\n");

		for (uint16_t frame = 0; frame < 100; frame++)
		{
			GLCD_ClearScreen();
			GLCD_WriteString("PATH DEMO");
			GLCD_SetDot(0, 16);
			GLCD_WriteString(path_name);

			// Update square position based on path
			switch (path_square.path_mode)
			{
			case 0: // Linear
				path_square.x += path_square.vx;
				path_square.y += path_square.vy;
				if (path_square.x <= 5 || path_square.x >= 123)
					path_square.vx = -path_square.vx;
				if (path_square.y <= 20 || path_square.y >= 59)
					path_square.vy = -path_square.vy;
				break;
			case 1: // Circular
				path_square.path_param += 0.15;
				path_square.x = 64 + 30 * cos(path_square.path_param);
				path_square.y = 32 + 15 * sin(path_square.path_param);
				break;
			case 2: // Figure-8
				path_square.path_param += 0.1;
				path_square.x = 64 + 25 * sin(path_square.path_param);
				path_square.y = 32 + 12 * sin(path_square.path_param * 2);
				break;
			case 3: // Random
				if (frame % 20 == 0)
				{
					path_square.vx = (float)(rand() % 300 - 150) / 100.0;
					path_square.vy = (float)(rand() % 300 - 150) / 100.0;
				}
				path_square.x += path_square.vx;
				path_square.y += path_square.vy;
				if (path_square.x < 10)
					path_square.x = 10;
				if (path_square.x > 118)
					path_square.x = 118;
				if (path_square.y < 25)
					path_square.y = 25;
				if (path_square.y > 59)
					path_square.y = 59;
				break;
			}

			path_square.rotation += path_square.rotation_speed;

			// Draw the square
			moving_square_draw_square(&path_square);

			_delay_ms(60);
		}

		_delay_ms(500);
	}
}

void main_graphics_moving_square(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS MOVING SQUARE DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Geometric shape animation\\r\\n");
	uart_string("- Rotation and transformation\\r\\n");
	uart_string("- Path following navigation\\r\\n");
	uart_string("- Multi-square coordination\\r\\n");
	uart_string("- Interactive shape control\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("MOVING SQUARE");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Geometric");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("Animation");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("to start");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting moving square demonstrations...\\r\\n\\r\\n");

	// 1. Single square transformation
	uart_string("1. Square Transformation Demo\\r\\n");
	moving_square_transformation_demo();

	_delay_ms(1000);

	// 2. Path following demonstration
	uart_string("\\r\\n2. Path Following Demo\\r\\n");
	moving_square_path_demo();

	_delay_ms(1000);

	// 3. Multi-square coordination
	uart_string("\\r\\n3. Multi-Square Coordination\\r\\n");
	moving_square_init();

	for (uint16_t frame = 0; frame < 400; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("MULTI-SQUARE");

		// Update all squares
		moving_square_update_physics();

		// Draw all squares
		for (uint8_t i = 0; i < MAX_SQUARES; i++)
		{
			if (squares[i].active)
			{
				moving_square_draw_square(&squares[i]);
			}
		}

		// Show animation info
		PORTB = frame & 0xFF;

		_delay_ms(50);

		// Check for exit
		if (PIND & (1 << PD7))
			break;
	}

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("MOVING SQUARE");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Geometric anim,");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("paths, rotation,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("coordination!");

	uart_string("\\r\\nMoving square demonstration completed!\\r\\n");
	uart_string("Demonstrated: transformation, paths, rotation, multi-square\\r\\n\\r\\n");

	while (1)
	{
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_MOVING_SQUARE

/*
 * GRAPHICS_SINE_WAVE - Sine Wave Visualization
 * Educational demonstration of:
 * - Mathematical function visualization
 * - Waveform generation and display
 * - Phase, frequency, and amplitude control
 * - Multiple wave interference patterns
 * - Real-time parameter adjustment
 */

#ifdef GRAPHICS_SINE_WAVE

#define MAX_WAVES 4

// Wave parameters structure
typedef struct
{
	float amplitude;   // Wave amplitude (0-20)
	float frequency;   // Wave frequency (cycles per screen)
	float phase;	   // Phase offset (0-2π)
	float phase_speed; // Phase shift speed
	uint8_t active;	   // Wave active flag
	uint8_t pattern;   // Display pattern (0=solid, 1=dotted, 2=dashed)
	uint8_t y_offset;  // Vertical offset
} wave_params_t;

static wave_params_t waves[MAX_WAVES];
static uint16_t time_frame = 0;
static uint8_t display_mode = 0; // 0=individual, 1=combined, 2=interference
static float global_time = 0;

// Initialize wave parameters
void sine_wave_init(void)
{
	// Wave 1: Basic sine wave
	waves[0].amplitude = 15;
	waves[0].frequency = 2.0;
	waves[0].phase = 0;
	waves[0].phase_speed = 0.1;
	waves[0].active = 1;
	waves[0].pattern = 0;
	waves[0].y_offset = 32;

	// Wave 2: Higher frequency
	waves[1].amplitude = 10;
	waves[1].frequency = 4.0;
	waves[1].phase = M_PI / 4;
	waves[1].phase_speed = 0.15;
	waves[1].active = 1;
	waves[1].pattern = 1;
	waves[1].y_offset = 32;

	// Wave 3: Lower frequency, larger amplitude
	waves[2].amplitude = 8;
	waves[2].frequency = 1.0;
	waves[2].phase = M_PI / 2;
	waves[2].phase_speed = 0.05;
	waves[2].active = 1;
	waves[2].pattern = 2;
	waves[2].y_offset = 32;

	// Wave 4: Modulation wave
	waves[3].amplitude = 12;
	waves[3].frequency = 3.0;
	waves[3].phase = M_PI;
	waves[3].phase_speed = -0.08;
	waves[3].active = 1;
	waves[3].pattern = 0;
	waves[3].y_offset = 32;

	time_frame = 0;
	global_time = 0;
}

// Calculate wave value at given x position
float sine_wave_calculate(uint8_t wave_index, uint8_t x_pos)
{
	if (!waves[wave_index].active)
		return 0;

	wave_params_t *wave = &waves[wave_index];

	// Calculate sine wave: y = A * sin(2π * f * x + phase)
	float x_normalized = (float)x_pos / 128.0; // Normalize x to 0-1
	float angle = 2 * M_PI * wave->frequency * x_normalized + wave->phase + (global_time * wave->phase_speed);

	return wave->amplitude * sin(angle);
}

// Draw individual wave
void sine_wave_draw_wave(uint8_t wave_index)
{
	if (!waves[wave_index].active)
		return;

	wave_params_t *wave = &waves[wave_index];

	for (uint8_t x = 0; x < 128; x++)
	{
		float y_value = sine_wave_calculate(wave_index, x);
		uint8_t y_pos = wave->y_offset + (int8_t)y_value;

		// Clip to screen bounds
		if (y_pos < 16)
			y_pos = 16;
		if (y_pos >= 64)
			y_pos = 63;

		// Draw based on pattern
		switch (wave->pattern)
		{
		case 0: // Solid line
			GLCD_SetDot(x, y_pos);
			break;

		case 1: // Dotted line
			if (x % 2 == 0)
				GLCD_SetDot(x, y_pos);
			break;

		case 2: // Dashed line
			if ((x % 6) < 3)
				GLCD_SetDot(x, y_pos);
			break;
		}
	}
}

// Draw combined waves (superposition)
void sine_wave_draw_combined(void)
{
	for (uint8_t x = 0; x < 128; x++)
	{
		float combined_y = 0;
		uint8_t active_waves = 0;

		// Sum all active waves
		for (uint8_t i = 0; i < MAX_WAVES; i++)
		{
			if (waves[i].active)
			{
				combined_y += sine_wave_calculate(i, x);
				active_waves++;
			}
		}

		// Average the result to prevent overflow
		if (active_waves > 0)
		{
			combined_y /= active_waves;
		}

		uint8_t y_pos = 32 + (int8_t)combined_y;

		// Clip to screen bounds
		if (y_pos < 16)
			y_pos = 16;
		if (y_pos >= 64)
			y_pos = 63;

		GLCD_SetDot(x, y_pos);
	}
}

// Draw interference pattern
void sine_wave_draw_interference(void)
{
	// Draw wave 1 and wave 2 interference
	for (uint8_t x = 0; x < 128; x++)
	{
		float wave1 = sine_wave_calculate(0, x);
		float wave2 = sine_wave_calculate(1, x);

		// Individual waves (faint)
		uint8_t y1 = 32 + (int8_t)(wave1 * 0.3);
		uint8_t y2 = 32 + (int8_t)(wave2 * 0.3);

		if (x % 4 == 0)
		{
			if (y1 >= 16 && y1 < 64)
				GLCD_SetDot(x, y1);
			if (y2 >= 16 && y2 < 64)
				GLCD_SetDot(x, y2);
		}

		// Combined wave (prominent)
		float combined = wave1 + wave2;
		uint8_t y_combined = 32 + (int8_t)(combined * 0.5);

		if (y_combined >= 16 && y_combined < 64)
		{
			GLCD_SetDot(x, y_combined);
		}
	}
}

// Demonstrate basic sine wave properties
void sine_wave_properties_demo(void)
{
	uart_string("Sine wave properties demonstration...\\r\\n");

	// Demo different amplitudes
	uart_string("Amplitude variation...\\r\\n");
	for (uint8_t amp = 5; amp <= 20; amp += 5)
	{
		waves[0].amplitude = amp;
		waves[0].frequency = 2.0;
		waves[0].phase = 0;

		for (uint16_t frame = 0; frame < 30; frame++)
		{
			GLCD_ClearScreen();
			GLCD_WriteString("AMPLITUDE");

			char amp_str[16];
			sprintf(amp_str, "A=%d", amp);
			GLCD_SetDot(80, 8);
			GLCD_WriteString(amp_str);

			// Draw reference line
			for (uint8_t x = 0; x < 128; x += 8)
			{
				GLCD_SetDot(x, 32);
			}

			sine_wave_draw_wave(0);

			_delay_ms(100);
		}
		_delay_ms(500);
	}

	// Demo different frequencies
	uart_string("Frequency variation...\\r\\n");
	float frequencies[] = {1.0, 2.0, 4.0, 6.0};
	for (uint8_t f = 0; f < 4; f++)
	{
		waves[0].amplitude = 15;
		waves[0].frequency = frequencies[f];
		waves[0].phase = 0;

		for (uint16_t frame = 0; frame < 40; frame++)
		{
			GLCD_ClearScreen();
			GLCD_WriteString("FREQUENCY");

			char freq_str[16];
			sprintf(freq_str, "f=%.1f", frequencies[f]);
			GLCD_SetDot(80, 8);
			GLCD_WriteString(freq_str);

			// Draw reference line
			for (uint8_t x = 0; x < 128; x += 8)
			{
				GLCD_SetDot(x, 32);
			}

			sine_wave_draw_wave(0);

			_delay_ms(80);
		}
		_delay_ms(500);
	}

	// Demo phase shift
	uart_string("Phase shift demonstration...\\r\\n");
	for (uint16_t frame = 0; frame < 100; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("PHASE SHIFT");

		waves[0].amplitude = 15;
		waves[0].frequency = 2.0;
		waves[0].phase = frame * 0.1;

		// Draw reference line
		for (uint8_t x = 0; x < 128; x += 8)
		{
			GLCD_SetDot(x, 32);
		}

		sine_wave_draw_wave(0);

		// Show phase value
		uint8_t phase_degrees = (uint8_t)((waves[0].phase * 180 / M_PI)) % 360;
		PORTB = phase_degrees;

		_delay_ms(60);
	}
}

// Interactive wave control
void sine_wave_interactive_demo(void)
{
	uart_string("Interactive wave control...\\r\\n");
	uart_string("PD0 - Change amplitude, PD1 - Change frequency\\r\\n");
	uart_string("PD2 - Change display mode, PD7 - Exit\\r\\n");

	sine_wave_init();
	uint8_t prev_buttons = 0xFF;
	uint8_t selected_wave = 0;

	for (uint16_t frame = 0; frame < 600; frame++)
	{
		GLCD_ClearScreen();

		// Read button input
		uint8_t current_buttons = PIND;
		uint8_t button_pressed = (~current_buttons) & prev_buttons;

		// Handle button presses
		if (button_pressed & (1 << PD0))
		{
			waves[selected_wave].amplitude += 3;
			if (waves[selected_wave].amplitude > 20)
				waves[selected_wave].amplitude = 5;
			uart_string("Amplitude changed\\r\\n");
		}

		if (button_pressed & (1 << PD1))
		{
			waves[selected_wave].frequency += 0.5;
			if (waves[selected_wave].frequency > 6.0)
				waves[selected_wave].frequency = 1.0;
			uart_string("Frequency changed\\r\\n");
		}

		if (button_pressed & (1 << PD2))
		{
			display_mode = (display_mode + 1) % 3;
			selected_wave = (selected_wave + 1) % MAX_WAVES;
			uart_string("Display mode changed\\r\\n");
		}

		if (button_pressed & (1 << PD7))
		{
			uart_string("Exiting interactive demo...\\r\\n");
			break;
		}

		prev_buttons = current_buttons;

		// Update global time for animation
		global_time += 0.1;

		// Display based on mode
		switch (display_mode)
		{
		case 0:
			GLCD_WriteString("INDIVIDUAL");
			for (uint8_t i = 0; i < MAX_WAVES; i++)
			{
				if (i == selected_wave)
				{
					sine_wave_draw_wave(i);
				}
			}
			break;

		case 1:
			GLCD_WriteString("COMBINED");
			sine_wave_draw_combined();
			break;

		case 2:
			GLCD_WriteString("INTERFERENCE");
			sine_wave_draw_interference();
			break;
		}

		// Draw parameter info
		char param_str[16];
		sprintf(param_str, "W%d A%d F%.1f", selected_wave + 1,
				(uint8_t)waves[selected_wave].amplitude, waves[selected_wave].frequency);
		GLCD_SetDot(0, 56);
		GLCD_WriteString(param_str);

		PORTB = frame & 0xFF;
		_delay_ms(50);
	}
}

void main_graphics_sine_wave(void)
{
	init_devices();

	// Initialize GLCD
	GLCD_Initialize();
	GLCD_ClearScreen();

	uart_string("\\r\\n=== GRAPHICS SINE WAVE DEMO ===\\r\\n");
	uart_string("Educational demonstration of:\\r\\n");
	uart_string("- Mathematical function visualization\\r\\n");
	uart_string("- Waveform generation and display\\r\\n");
	uart_string("- Phase, frequency, amplitude control\\r\\n");
	uart_string("- Wave interference patterns\\r\\n");
	uart_string("- Real-time parameter adjustment\\r\\n\\r\\n");

	// Display title screen
	GLCD_WriteString("SINE WAVE DEMO");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("Mathematical");
	GLCD_SetDot(0, 24);
	GLCD_WriteString("Visualization");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("Press button");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("to start");

	// Wait for button press
	while (!(PIND & 0xFF))
		;
	while (PIND & 0xFF)
		;

	uart_string("Starting sine wave demonstrations...\\r\\n\\r\\n");

	// 1. Basic properties demonstration
	uart_string("1. Wave Properties Demo\\r\\n");
	sine_wave_properties_demo();

	_delay_ms(1000);

	// 2. Multiple waves animation
	uart_string("\\r\\n2. Multiple Waves Animation\\r\\n");
	sine_wave_init();

	for (uint16_t frame = 0; frame < 200; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("MULTI-WAVES");

		global_time += 0.08;

		// Draw all waves individually with slight offset
		for (uint8_t i = 0; i < MAX_WAVES; i++)
		{
			waves[i].y_offset = 20 + (i * 10);
			sine_wave_draw_wave(i);
		}

		PORTB = frame & 0xFF;
		_delay_ms(60);

		if (PIND & (1 << PD7))
			break;
	}

	_delay_ms(1000);

	// 3. Wave interference demonstration
	uart_string("\\r\\n3. Wave Interference\\r\\n");
	sine_wave_init();

	for (uint16_t frame = 0; frame < 150; frame++)
	{
		GLCD_ClearScreen();
		GLCD_WriteString("INTERFERENCE");

		global_time += 0.05;

		sine_wave_draw_interference();

		_delay_ms(80);

		if (PIND & (1 << PD7))
			break;
	}

	_delay_ms(1000);

	// 4. Interactive control
	uart_string("\\r\\n4. Interactive Wave Control\\r\\n");
	sine_wave_interactive_demo();

	// Final display
	GLCD_ClearScreen();
	GLCD_WriteString("SINE WAVE");
	GLCD_SetDot(0, 16);
	GLCD_WriteString("DEMO COMPLETE");
	GLCD_SetDot(0, 32);
	GLCD_WriteString("Math functions,");
	GLCD_SetDot(0, 40);
	GLCD_WriteString("waveforms,");
	GLCD_SetDot(0, 48);
	GLCD_WriteString("interference!");

	uart_string("\\r\\nSine wave demonstration completed!\\r\\n");
	uart_string("Demonstrated: properties, multi-wave, interference, interaction\\r\\n\\r\\n");

	while (1)
	{
		_delay_ms(1000);
	}
}

#endif // GRAPHICS_SINE_WAVE

/*
 * MODERNIZED GRAPHICS PROGRAMMING DEMONSTRATIONS
 * Educational Framework: ATmega128 Visual Programming and Animation
 *
 * Learning Objectives:
 * 1. Master graphics programming with GLCD library integration
 * 2. Understand animation principles and timing control
 * 3. Learn interactive graphics with user input control
 * 4. Explore mathematical visualization and pattern generation
 *
 * Graphics Integration with Modern Libraries:
 * - Timer2 library: Smooth animation timing and frame rate control
 * - Port library: Interactive control via buttons and inputs
 * - Random algorithms: Mathematical pattern generation
 * - GLCD functions: Professional graphics rendering
 *
 * Educational Progression:
 * - Basic shapes and text → Animated graphics → Interactive visualization
 * - Mathematical concepts → Real-time animation → User-driven graphics
 *
 * Hardware Connections:
 * - GLCD display: Primary graphics output
 * - Buttons: Interactive control for graphics parameters
 * - LEDs: Visual feedback coordination with graphics
 */

// Only compile this file if any GRAPHICS demo is enabled
#ifdef GRAPHICS_BASICS
#define GRAPHICS_DEMO_ENABLED
#endif
#ifdef GRAPHICS_MOVEMENT
#define GRAPHICS_DEMO_ENABLED
#endif
#ifdef GRAPHICS_RANDOM
#define GRAPHICS_DEMO_ENABLED
#endif
#ifdef GRAPHICS_BOUNCING_BALL
#define GRAPHICS_DEMO_ENABLED
#endif
#ifdef GRAPHICS_MOVING_SQUARE
#define GRAPHICS_DEMO_ENABLED
#endif
#ifdef GRAPHICS_SINE_WAVE
#define GRAPHICS_DEMO_ENABLED
#endif

#ifdef GRAPHICS_DEMO_ENABLED

/* Graphics: Advanced Basics with Animation */
#ifdef GRAPHICS_BASICS
/*
 * DEMONSTRATION 1: Advanced Graphics Basics with Smooth Animation
 * Educational Focus: Fundamental graphics operations with modern timing
 *
 * This example demonstrates:
 * - Basic GLCD functions with coordinated timing
 * - Text and graphics combination techniques
 * - Smooth animation using Timer2 precision
 * - Educational information display integration
 *
 * Learning Points:
 * 1. GLCD functions provide versatile graphics capabilities
 * 2. Timer2 precision enables smooth animation timing
 * 3. Text and graphics coordination enhances user experience
 * 4. Systematic display updates create professional interfaces
 *
 * Hardware Setup:
 * - GLCD display for graphics and text output
 * - System timing coordinated with visual updates
 */

// Educational display strings
char educational_header[] = "SOC3050 Graphics Demo";
char educational_pattern[] = "Pattern: #$%&'()*+,-./0123456";

void main_graphics_basics(void)
{
	init_devices();

	// Initialize Timer2 for smooth animation timing
	Timer2_init();
	Timer2_start();

	// Sound notification for graphics demo start
	S_Start();

	// Display educational information
	lcd_clear();
	lcd_string(0, 0, "Advanced Graphics");
	lcd_string(0, 1, "Basic Shapes Demo");
	lcd_string(0, 2, "Timer2 Animation");
	lcd_string(0, 3, "Educational Focus");

	unsigned long last_update = 0;
	unsigned int animation_frame = 0;
	unsigned char shape_cycle = 0;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Update graphics every 200ms for smooth animation
		if (current_time - last_update >= 200)
		{
			last_update = current_time;
			animation_frame++;

			// Clear screen for new frame
			lcd_clear();
			ScreenBuffer_clear();

			// Educational header display
			lcd_string(0, 0, "====================");
			lcd_string(1, 0, "  ATmega128 GLCD    ");
			lcd_string(2, 0, educational_pattern);

			// Animated graphics elements
			switch (shape_cycle % 4)
			{
			case 0:
				// Lines demonstration
				GLCD_Line(30, 10, 40, 20);
				GLCD_Line(35, 10, 35, 20);
				lcd_string(0, 4, "Shape: Lines");
				break;

			case 1:
				// Rectangle demonstration
				GLCD_Rectangle(30, 31, 40, 41);
				GLCD_Rectangle(32, 33, 38, 39);
				lcd_string(0, 4, "Shape: Rectangles");
				break;

			case 2:
				// Circle demonstration
				GLCD_Circle(35, 55, 5);
				GLCD_Circle(35, 55, 3);
				lcd_string(0, 4, "Shape: Circles");
				break;

			case 3:
				// Combined shapes
				GLCD_Line(30, 10, 40, 20);
				GLCD_Rectangle(30, 31, 40, 41);
				GLCD_Circle(35, 55, 5);
				lcd_string(0, 4, "Shape: Combined");
				break;
			}

			// Educational information display
			lcd_string(0, 5, "Frame: ");
			GLCD_4DigitDecimal(animation_frame);

			lcd_string(0, 6, "Time: ");
			GLCD_4DigitDecimal(current_time / 1000);
			lcd_string(8, 6, "s");

			lcd_string(0, 7, educational_header);

			// Advance shape cycle every 10 frames
			if ((animation_frame % 10) == 0)
			{
				shape_cycle++;
			}
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(50);
	}
}

#endif

/* Graphics: Intelligent Random Pattern Generation */
#ifdef GRAPHICS_RANDOM
/*
 * DEMONSTRATION 2: Advanced Random Graphics with Pattern Control
 * Educational Focus: Mathematical visualization and controlled randomness
 *
 * This example demonstrates:
 * - Controlled random number generation for educational patterns
 * - Mathematical visualization using graphics primitives
 * - Timer2-based frame rate control for smooth animation
 * - Progressive pattern complexity and educational progression
 *
 * Learning Points:
 * 1. Random algorithms can create engaging educational content
 * 2. Mathematical concepts can be visualized through graphics
 * 3. Controlled randomness provides predictable educational outcomes
 * 4. Frame rate control ensures smooth visual experience
 *
 * Hardware Setup:
 * - GLCD display for random pattern visualization
 * - Coordinated timing for educational pacing
 */

void main_graphics_random(void)
{
	init_devices();

	// Initialize Timer2 for controlled animation timing
	Timer2_init();
	Timer2_start();

	// Initialize random number generator with Timer2-based seed
	srand(Timer2_get_milliseconds());

	// Display educational information
	lcd_clear();
	lcd_string(0, 0, "Random Graphics");
	lcd_string(0, 1, "Mathematical Viz");
	lcd_string(0, 2, "Controlled Chaos");
	lcd_string(0, 3, "Educational Patterns");

	unsigned long last_pattern_update = 0;
	unsigned int pattern_count = 0;
	unsigned char complexity_level = 1;
	const unsigned int PATTERN_INTERVAL = 150; // 150ms between patterns

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Generate new pattern every interval
		if (current_time - last_pattern_update >= PATTERN_INTERVAL)
		{
			last_pattern_update = current_time;
			pattern_count++;

			// Generate controlled random parameters
			unsigned int rand_x = rand() % 54 + 5; // Keep within safe screen bounds
			unsigned int rand_y = rand() % 118 + 5;
			unsigned int rand_radius = (rand() % 8) + 2; // Radius 2-10

			// Apply complexity-based pattern generation
			switch (complexity_level)
			{
			case 1:
				// Simple circles
				GLCD_Circle(rand_x, rand_y, rand_radius);
				break;

			case 2:
				// Circles with rectangles
				GLCD_Circle(rand_x, rand_y, rand_radius);
				GLCD_Rectangle(rand_x - 3, rand_y - 3, rand_x + 3, rand_y + 3);
				break;

			case 3:
				// Complex patterns with lines
				GLCD_Circle(rand_x, rand_y, rand_radius);
				GLCD_Line(rand_x - rand_radius, rand_y, rand_x + rand_radius, rand_y);
				GLCD_Line(rand_x, rand_y - rand_radius, rand_x, rand_y + rand_radius);
				break;
			}

			// Update educational display information
			lcd_string(0, 4, "Patterns: ");
			GLCD_4DigitDecimal(pattern_count);

			lcd_string(0, 5, "Level: ");
			GLCD_1DigitDecimal(complexity_level);
			lcd_string(8, 5, "/3");

			lcd_string(0, 6, "Pos: ");
			GLCD_2DigitDecimal(rand_x);
			lcd_string(5, 6, ",");
			GLCD_3DigitDecimal(rand_y);

			lcd_string(0, 7, "R: ");
			GLCD_2DigitDecimal(rand_radius);
			lcd_string(4, 7, " Time:");
			GLCD_4DigitDecimal(current_time / 1000);

			// Progressive complexity increase
			if ((pattern_count % 50) == 0)
			{
				complexity_level = (complexity_level % 3) + 1;

				// Clear screen for new complexity level
				lcd_clear();
				ScreenBuffer_clear();
				lcd_string(0, 0, "Complexity Level ");
				GLCD_1DigitDecimal(complexity_level);
				Timer2_delay_ms(1000); // Brief pause to show level change
			}
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(10);
	}
}
#endif

/* 	Graphics: Random movement 	*/
#ifdef GRAPHICS_MOVEMENT

void main_graphics_movement(void)
{
	// random numbers and seed.
	uint16_t u_rand_x = 0, old_x = 0;
	uint16_t u_rand_y = 0, old_y = 0;
	time_t t;

	init_devices();
	lcd_clear();
	srand((unsigned)time(&t));
	while (1)
	{
		// S_Star();
		u_rand_x = rand() % 64;
		u_rand_y = rand() % 128;
		GLCD_Line(old_x, old_y, u_rand_x, u_rand_y);
		old_x = u_rand_x;
		old_y = u_rand_y;
		_delay_ms(100);
	}
}
#endif

#ifdef GRAPHICS_BOUNCING_BALL

void main_graphics_bouncing_ball(void)
{
	uint16_t x, y, radius = 5;
	int8_t dx = 6, dy = 6;
	time_t t;

	init_devices();
	lcd_clear();

	// Seed the random number generator
	srand((unsigned)time(&t));

	// Randomize the initial position of the ball
	x = rand() % (64 - radius * 2) + radius; // Ensure it starts within the screen bounds
	y = rand() % (128 - radius * 2) + radius;

	while (1)
	{
		GLCD_Circle(x, y, radius); // Draw ball

		_delay_ms(100);
		GLCD_Circle(x, y, radius); // Clear ball by redrawing it (as XOR)

		// Update position
		x += dx;
		y += dy;

		// Bounce if it hits the edge
		if (x + radius >= 64 || x - radius <= 0)
			dx = -dx;
		if (y + radius >= 128 || y - radius <= 0)
			dy = -dy;
	}
}
#endif

#ifdef GRAPHICS_MOVING_SQUARE

void main_graphics_moving_square(void)
{
	uint16_t x = 0;

	init_devices();
	lcd_clear();

	while (1)
	{
		GLCD_Rectangle(x, 30, x + 10, 40); // Draw square
		_delay_ms(100);
		GLCD_Rectangle(x, 30, x + 10, 40); // Erase square

		x += 8; // Move square to the right

		if (x > 64)
		{ // If it moves out of bounds, reset to the left side
			x = 0;
		}
	}
}
#endif

#ifdef GRAPHICS_SINE_WAVE

void main_graphics_sine_wave(void)
{
	uint16_t x = 0, y = 0;
	uint16_t radius = 0.5;

	init_devices();
	lcd_clear();

	for (y = 0; y < 128; y++)
	{
		// Invert the sine wave by negating the sine function
		x = (uint16_t)(32 - 30 * sin(y * M_PI / 64)); // Calculate vertical position (inverted sine wave)

		GLCD_Circle(x, y, radius); // Draw circle at the (x, y) position based on inverted sine wave
		_delay_ms(50);			   // Small delay to visualize the wave pattern
	}
}
#endif

#endif // GRAPHICS_DEMO_ENABLED
