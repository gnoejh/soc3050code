
#include "config.h"

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
