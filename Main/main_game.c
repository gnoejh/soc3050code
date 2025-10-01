/*
 * GAME_SIMON_SAYS - Interactive Memory Game
 * Educational demonstration of:
 * - Game state machines
 * - LED pattern generation
 * - Button input handling
 * - Score tracking and progression
 * - Random sequence generation
 */

#include "config.h"
#endif // GAME_SIMON_SAYS

/*
 * GAME_REACTION_TIMER - Interactive Reaction Time Measurement
 * Educational demonstration of:
 * - Precise timing measurement
 * - Random stimulus generation
 * - Statistical analysis
 * - Performance tracking
 * - Human-computer interaction
 */

#ifdef GAME_REACTION_TIMER

// Reaction timer state definitions
typedef enum
{
	REACTION_IDLE,
	REACTION_WAITING_START,
	REACTION_DELAY_PERIOD,
	REACTION_STIMULUS_ACTIVE,
	REACTION_MEASURING,
	REACTION_RESULTS,
	REACTION_STATISTICS
} reaction_state_t;

// Configuration constants
#define MAX_TRIALS 10
#define MIN_DELAY_MS 1000
#define MAX_DELAY_MS 5000
#define STIMULUS_TIMEOUT_MS 3000
#define RESULTS_DISPLAY_TIME 3000
#define FALSE_START_PENALTY_MS 1000

// Global reaction timer variables
static reaction_state_t reaction_state = REACTION_IDLE;
static uint16_t reaction_times[MAX_TRIALS];
static uint8_t current_trial = 0;
static uint16_t stimulus_start_time = 0;
static uint16_t reaction_time = 0;
static uint16_t delay_time = 0;
static uint8_t false_starts = 0;
static uint16_t total_trials_completed = 0;

// Timer variables for precise measurement
static volatile uint16_t timer_ms = 0;
static volatile uint8_t button_pressed = 0;

// Timer interrupt for millisecond precision
ISR(TIMER0_OVF_vect)
{
	static uint8_t timer_count = 0;
	timer_count++;
	if (timer_count >= 61)
	{ // Approximately 1ms at 16MHz with /1024 prescaler
		timer_ms++;
		timer_count = 0;
	}
}

// External interrupt for button detection
ISR(INT0_vect)
{
	if (reaction_state == REACTION_STIMULUS_ACTIVE || reaction_state == REACTION_MEASURING)
	{
		button_pressed = 1;
		reaction_time = timer_ms - stimulus_start_time;
	}
	else if (reaction_state == REACTION_DELAY_PERIOD)
	{
		// False start detected
		false_starts++;
		button_pressed = 2; // Flag for false start
	}
}

// Initialize timer for precise timing
void reaction_timer_init(void)
{
	// Configure Timer0 for 1ms interrupts
	TCCR0 = (1 << CS02) | (1 << CS00); // /1024 prescaler
	TIMSK |= (1 << TOIE0);			   // Enable overflow interrupt

	// Configure external interrupt INT0 for button
	EICRA |= (1 << ISC01); // Falling edge trigger
	EIMSK |= (1 << INT0);  // Enable INT0\n    \n    sei(); // Enable global interrupts
}

// Generate random delay between stimuli
uint16_t reaction_get_random_delay(void)
{
	static uint32_t seed = 12345;
	seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
	return MIN_DELAY_MS + (seed % (MAX_DELAY_MS - MIN_DELAY_MS));
}

// Calculate statistics
void reaction_calculate_stats(uint16_t *avg, uint16_t *best, uint16_t *worst)
{
	*avg = 0;
	*best = 0xFFFF;
	*worst = 0;

	uint8_t valid_trials = 0;
	for (uint8_t i = 0; i < current_trial; i++)
	{
		if (reaction_times[i] > 0 && reaction_times[i] < STIMULUS_TIMEOUT_MS)
		{
			*avg += reaction_times[i];
			if (reaction_times[i] < *best)
				*best = reaction_times[i];
			if (reaction_times[i] > *worst)
				*worst = reaction_times[i];
			valid_trials++;
		}
	}

	if (valid_trials > 0)
	{
		*avg /= valid_trials;
	}
	else
	{
		*avg = 0;
		*best = 0;
		*worst = 0;
	}
}

// Display reaction time performance rating
const char *reaction_get_rating(uint16_t avg_time)
{
	if (avg_time == 0)
		return \"No Data\";\n    if (avg_time < 200) return \"Excellent\";\n    if (avg_time < 250) return \"Very Good\";\n    if (avg_time < 300) return \"Good\";\n    if (avg_time < 400) return \"Average\";\n    if (avg_time < 500) return \"Below Avg\";\n    return \"Needs Work\";\n}

			int
			main_game_reaction_timer(void)
		{
			init_devices();
			reaction_timer_init();

			// Initialize display
			GLCD_Initialize();
			GLCD_ClearScreen();

	uart_string(\"\\r\\n=== REACTION TIMER EDUCATIONAL DEMO ===\\r\\n\");
    uart_string(\"Reaction time measurement demonstrating:\\r\\n\");
    uart_string(\"- Precise timing measurement\\r\\n\");
    uart_string(\"- Random stimulus generation\\r\\n\");
    uart_string(\"- Statistical analysis\\r\\n\");
    uart_string(\"- Performance tracking\\r\\n\\r\\n\");
    
    uart_string(\"Instructions:\\r\\n\");
    uart_string(\"1. Press button when you see the LED light up\\r\\n\");
    uart_string(\"2. Wait for the stimulus - don't anticipate!\\r\\n\");
    uart_string(\"3. React as fast as possible\\r\\n\\r\\n\");
    
    // Display initial screen
    GLCD_WriteString(\"REACTION TIMER\");
    GLCD_SetDot(0, 16);
    GLCD_WriteString(\"Press button to start\");
    GLCD_SetDot(0, 32);
    GLCD_WriteString(\"Ready for trial 1\");
    
    reaction_state = REACTION_WAITING_START;
    timer_ms = 0;
    current_trial = 0;
    false_starts = 0;
    
    while (1) {
				switch (reaction_state)
				{
				case REACTION_WAITING_START:
				{
					if (PIND & (1 << PD2))
					{ // Button pressed
						// Start new trial
						delay_time = reaction_get_random_delay();
						timer_ms = 0;
						button_pressed = 0;
						reaction_state = REACTION_DELAY_PERIOD;

						char trial_msg[32];
					sprintf(trial_msg, \"Trial %d: Wait for LED...\", current_trial + 1);
                    
                    GLCD_ClearScreen();
                    GLCD_WriteString(\"REACTION TIMER\");
                    GLCD_SetDot(0, 16);
                    GLCD_WriteString(trial_msg);
                    GLCD_SetDot(0, 32);
                    GLCD_WriteString(\"DON'T PRESS YET!\");
                    
                    uart_string(trial_msg);
                    uart_string(\"\\r\\n\");
                    
                    // Wait for button release
                    while (PIND & (1 << PD2));
					}
					\n break;
					\n
				}
				\n            \n case REACTION_DELAY_PERIOD:
				{
					\n if (timer_ms >= delay_time)
					{
						\n // Show stimulus\n                    PORTB = 0xFF; // Light up all LEDs\n                    buzzer_play_frequency(1000, 100); // Audio stimulus\n                    \n                    stimulus_start_time = timer_ms;\n                    button_pressed = 0;\n                    reaction_state = REACTION_STIMULUS_ACTIVE;\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"*** REACT NOW! ***\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"Press button fast!\");\n                    \n                    uart_string(\"STIMULUS! Press button now!\\r\\n\");\n                } else if (button_pressed == 2) {\n                    // False start detected\n                    PORTB = 0x00;\n                    reaction_state = REACTION_RESULTS;\n                    reaction_time = 0xFFFF; // Mark as false start\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"FALSE START!\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"Wait for the LED!\");\n                    \n                    uart_string(\"FALSE START! Wait for the stimulus.\\r\\n\");\n                    \n                    // Play error sound\n                    for (uint8_t i = 0; i < 3; i++) {\n                        buzzer_play_frequency(200, 200);\n                        _delay_ms(100);\n                    }\n                }\n                break;\n            }\n            \n            case REACTION_STIMULUS_ACTIVE: {\n                if (button_pressed == 1) {\n                    // Valid reaction detected\n                    PORTB = 0x00; // Turn off LEDs\n                    reaction_state = REACTION_RESULTS;\n                    \n                    char result_msg[64];\n                    if (reaction_time < STIMULUS_TIMEOUT_MS) {\n                        sprintf(result_msg, \"Reaction: %d ms\", reaction_time);\n                        reaction_times[current_trial] = reaction_time;\n                    } else {\n                        sprintf(result_msg, \"Too slow! (>%d ms)\", STIMULUS_TIMEOUT_MS);\n                        reaction_times[current_trial] = STIMULUS_TIMEOUT_MS;\n                    }\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"TRIAL COMPLETE\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(result_msg);\n                    \n                    uart_string(result_msg);\n                    uart_string(\"\\r\\n\");\n                    \n                } else if (timer_ms - stimulus_start_time >= STIMULUS_TIMEOUT_MS) {\n                    // Timeout - no response\n                    PORTB = 0x00;\n                    reaction_state = REACTION_RESULTS;\n                    reaction_time = STIMULUS_TIMEOUT_MS;\n                    reaction_times[current_trial] = STIMULUS_TIMEOUT_MS;\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"TIMEOUT!\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"No response detected\");\n                    \n                    uart_string(\"TIMEOUT! No response detected.\\r\\n\");\n                }\n                break;\n            }\n            \n            case REACTION_RESULTS: {\n                static uint16_t result_timer = 0;\n                result_timer++;\n                \n                if (result_timer >= RESULTS_DISPLAY_TIME || (PIND & (1 << PD2))) {\n                    result_timer = 0;\n                    current_trial++;\n                    \n                    if (current_trial >= MAX_TRIALS) {\n                        // Show final statistics\n                        reaction_state = REACTION_STATISTICS;\n                    } else {\n                        // Next trial\n                        reaction_state = REACTION_WAITING_START;\n                        \n                        char next_msg[32];\n                        sprintf(next_msg, \"Ready for trial %d\", current_trial + 1);\n                        \n                        GLCD_ClearScreen();\n                        GLCD_WriteString(\"REACTION TIMER\");\n                        GLCD_SetDot(0, 16);\n                        GLCD_WriteString(\"Press button to start\");\n                        GLCD_SetDot(0, 32);\n                        GLCD_WriteString(next_msg);\n                        \n                        uart_string(\"Press button for next trial.\\r\\n\");\n                    }\n                    \n                    // Wait for button release\n                    while (PIND & (1 << PD2));\n                }\n                break;\n            }\n            \n            case REACTION_STATISTICS: {\n                uint16_t avg_time, best_time, worst_time;\n                reaction_calculate_stats(&avg_time, &best_time, &worst_time);\n                \n                GLCD_ClearScreen();\n                GLCD_WriteString(\"FINAL RESULTS\");\n                \n                char stats_line1[32], stats_line2[32], stats_line3[32];\n                sprintf(stats_line1, \"Avg: %d ms\", avg_time);\n                sprintf(stats_line2, \"Best: %d ms\", best_time);\n                sprintf(stats_line3, \"Rating: %s\", reaction_get_rating(avg_time));\n                \n                GLCD_SetDot(0, 16);\n                GLCD_WriteString(stats_line1);\n                GLCD_SetDot(0, 24);\n                GLCD_WriteString(stats_line2);\n                GLCD_SetDot(0, 32);\n                GLCD_WriteString(stats_line3);\n                \n                uart_string(\"\\r\\n=== FINAL STATISTICS ===\\r\\n\");\n                uart_string(stats_line1); uart_string(\"\\r\\n\");\n                uart_string(stats_line2); uart_string(\"\\r\\n\");\n                sprintf(stats_line1, \"Worst: %d ms\", worst_time);\n                uart_string(stats_line1); uart_string(\"\\r\\n\");\n                sprintf(stats_line1, \"False starts: %d\", false_starts);\n                uart_string(stats_line1); uart_string(\"\\r\\n\");\n                uart_string(stats_line3); uart_string(\"\\r\\n\");\n                \n                uart_string(\"\\r\\nPress button to restart.\\r\\n\");\n                \n                if (PIND & (1 << PD2)) {\n                    // Restart game\n                    current_trial = 0;\n                    false_starts = 0;\n                    total_trials_completed++;\n                    timer_ms = 0;\n                    reaction_state = REACTION_WAITING_START;\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"REACTION TIMER\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"Press button to start\");\n                    GLCD_SetDot(0, 32);\n                    GLCD_WriteString(\"Ready for trial 1\");\n                    \n                    // Wait for button release\n                    while (PIND & (1 << PD2));\n                }\n                break;\n            }\n            \n            default:\n                reaction_state = REACTION_IDLE;\n                break;\n        }\n        \n        _delay_ms(1); // Small delay for timing\n    }\n    \n    return 0;\n}\n\n#endif // GAME_REACTION_TIMER

			/*
			 * GAME_SENSOR_TARGET - Sensor-Based Target Practice Game
			 * Educational demonstration of:
			 * - Analog sensor processing
			 * - Distance/proximity measurement
			 * - Accuracy calculation and scoring
			 * - Progressive difficulty systems
			 * - Sensor calibration techniques
			 */

#ifdef GAME_SENSOR_TARGET

							// Target game state definitions
							typedef enum {
								TARGET_IDLE,
								TARGET_CALIBRATION,
								TARGET_READY,
								TARGET_ACTIVE,
								TARGET_SCORING,
								TARGET_RESULTS
							} target_state_t;

// Game configuration
#define MAX_TARGET_ROUNDS 8
#define CALIBRATION_SAMPLES 10
#define TARGET_TIMEOUT_MS 5000
#define SCORE_DISPLAY_TIME 2000
#define PERFECT_SCORE_THRESHOLD 20 // ADC units for \"perfect\" hit
#define GOOD_SCORE_THRESHOLD 50	   // ADC units for \"good\" hit
#define TARGET_CENTER_VALUE 512	   // Middle of ADC range (1024/2)

// Target zones and scoring
#define ZONE_PERFECT 100  // Bull's eye
#define ZONE_EXCELLENT 80 // Inner ring
#define ZONE_GOOD 60	  // Middle ring
#define ZONE_FAIR 40	  // Outer ring
#define ZONE_MISS 0		  // Outside target

						// Global target game variables
						static target_state_t target_state = TARGET_IDLE;
						static uint16_t sensor_baseline = 0;
						static uint16_t target_values[MAX_TARGET_ROUNDS];
						static uint8_t target_scores[MAX_TARGET_ROUNDS];
						static uint8_t current_round = 0;
						static uint16_t round_start_time = 0;
						static uint16_t total_score = 0;
						static uint8_t perfect_hits = 0;
						static uint8_t good_hits = 0;

						// Sensor reading with averaging for stability
						uint16_t target_read_sensor_averaged(uint8_t samples)
						{
							uint32_t sum = 0;
							for (uint8_t i = 0; i < samples; i++)
							{
								sum += adc_read(0); // Read from ADC channel 0
								_delay_ms(10);
							}
							return sum / samples;
						}

						// Generate random target value within sensor range
						uint16_t target_generate_target_value(void)
						{
							static uint32_t seed = 54321;
							seed = (seed * 1664525 + 1013904223) & 0xFFFFFFFF;

							// Generate target within ±200 ADC units of center
							uint16_t offset = (seed % 400) - 200;
							\n uint16_t target = TARGET_CENTER_VALUE + offset;
							\n    \n // Ensure target is within valid ADC range\n    if (target < 100) target = 100;\n    if (target > 924) target = 924;\n    \n    return target;\n}

								// Calculate score based on distance from target
								uint8_t
								target_calculate_score(uint16_t sensor_value, uint16_t target_value)
							{
								uint16_t distance = (sensor_value > target_value) ? \n(sensor_value - target_value) : (target_value - sensor_value);
								\n    \n if (distance <= PERFECT_SCORE_THRESHOLD)
								{
									\n perfect_hits++;
									\n return ZONE_PERFECT;
									\n
								}
								else if (distance <= GOOD_SCORE_THRESHOLD)
								{
									\n good_hits++;
									\n return ZONE_EXCELLENT;
									\n
								}
								else if (distance <= 100)
								{
									\n return ZONE_GOOD;
									\n
								}
								else if (distance <= 200)
								{
									\n return ZONE_FAIR;
									\n
								}
								else
								{
									\n return ZONE_MISS;
									\n
								}
								\n
							}

							// Display target zone on LEDs
							void target_display_zone(uint8_t score)
							{
								PORTB = 0x00; // Clear all LEDs\n    \n    if (score >= ZONE_PERFECT) {\n        PORTB = 0xFF; // All LEDs for perfect\n    } else if (score >= ZONE_EXCELLENT) {\n        PORTB = 0x7E; // 6 LEDs for excellent\n    } else if (score >= ZONE_GOOD) {\n        PORTB = 0x3C; // 4 LEDs for good\n    } else if (score >= ZONE_FAIR) {\n        PORTB = 0x18; // 2 LEDs for fair\n    } else {\n        PORTB = 0x00; // No LEDs for miss\n    }\n}

								// Play sound based on score
								void target_play_score_sound(uint8_t score)
								{
									if (score >= ZONE_PERFECT)
									{
										\n // Perfect hit - ascending tone\n        buzzer_play_frequency(1000, 100);\n        _delay_ms(50);\n        buzzer_play_frequency(1200, 100);\n        _delay_ms(50);\n        buzzer_play_frequency(1500, 200);\n    } else if (score >= ZONE_EXCELLENT) {\n        // Excellent - high tone\n        buzzer_play_frequency(1000, 300);\n    } else if (score >= ZONE_GOOD) {\n        // Good - medium tone\n        buzzer_play_frequency(800, 200);\n    } else if (score >= ZONE_FAIR) {\n        // Fair - low tone\n        buzzer_play_frequency(600, 150);\n    } else {\n        // Miss - error sound\n        buzzer_play_frequency(200, 500);\n    }\n}

											// Get performance rating
											const char *
											target_get_rating(uint16_t avg_score)
										{
											if (avg_score >= 90)
												return \"Expert\";\n    if (avg_score >= 80) return \"Advanced\";\n    if (avg_score >= 70) return \"Good\";\n    if (avg_score >= 60) return \"Fair\";\n    if (avg_score >= 40) return \"Beginner\";\n    return \"Keep Trying\";\n}

													int
													main_game_sensor_target(void)
												{
													\n init_devices();
													\n    \n // Initialize ADC\n    adc_init();\n    \n    // Initialize display\n    GLCD_Initialize();\n    GLCD_ClearScreen();\n    \n    uart_string(\"\\r\\n=== SENSOR TARGET PRACTICE ===\\r\\n\");\n    uart_string(\"Educational demonstration of:\\r\\n\");\n    uart_string(\"- Analog sensor processing\\r\\n\");\n    uart_string(\"- Distance/proximity measurement\\r\\n\");\n    uart_string(\"- Accuracy calculation\\r\\n\");\n    uart_string(\"- Progressive difficulty\\r\\n\\r\\n\");\n    \n    uart_string(\"Instructions:\\r\\n\");\n    uart_string(\"1. Calibrate sensor baseline\\r\\n\");\n    uart_string(\"2. Move object to match target distance\\r\\n\");\n    uart_string(\"3. Press button when positioned\\r\\n\\r\\n\");\n    \n    // Display initial screen\n    GLCD_WriteString(\"SENSOR TARGET\");\n    GLCD_SetDot(0, 16);\n    GLCD_WriteString(\"Press to calibrate\");\n    \n    target_state = TARGET_CALIBRATION;\n    current_round = 0;\n    total_score = 0;\n    perfect_hits = 0;\n    good_hits = 0;\n    \n    while (1) {\n        switch (target_state) {\n            case TARGET_CALIBRATION: {\n                if (PIND & (1 << PD2)) { // Button pressed\n                    uart_string(\"Calibrating sensor baseline...\\r\\n\");\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"CALIBRATING...\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"Keep sensor clear\");\n                    \n                    _delay_ms(1000); // Wait for stable reading\n                    sensor_baseline = target_read_sensor_averaged(CALIBRATION_SAMPLES);\n                    \n                    char baseline_msg[32];\n                    sprintf(baseline_msg, \"Baseline: %d\", sensor_baseline);\n                    uart_string(baseline_msg);\n                    uart_string(\"\\r\\n\");\n                    \n                    target_state = TARGET_READY;\n                    \n                    // Wait for button release\n                    while (PIND & (1 << PD2));\n                }\n                break;\n            }\n            \n            case TARGET_READY: {\n                if (current_round >= MAX_TARGET_ROUNDS) {\n                    target_state = TARGET_RESULTS;\n                    break;\n                }\n                \n                // Generate new target\n                target_values[current_round] = target_generate_target_value();\n                \n                char round_msg[64];\n                sprintf(round_msg, \"Round %d: Target = %d\", \n                    current_round + 1, target_values[current_round]);\n                \n                GLCD_ClearScreen();\n                GLCD_WriteString(\"TARGET PRACTICE\");\n                GLCD_SetDot(0, 16);\n                GLCD_WriteString(round_msg);\n                GLCD_SetDot(0, 32);\n                GLCD_WriteString(\"Position & press\");\n                \n                uart_string(round_msg);\n                uart_string(\"\\r\\n\");\n                uart_string(\"Position object and press button.\\r\\n\");\n                \n                target_state = TARGET_ACTIVE;\n                round_start_time = 0; // Reset timer\n                break;\n            }\n            \n            case TARGET_ACTIVE: {\n                // Display current sensor reading\n                uint16_t current_sensor = adc_read(0);\n                \n                // Update display every 100ms\n                static uint8_t display_counter = 0;\n                display_counter++;\n                if (display_counter >= 100) {\n                    display_counter = 0;\n                    \n                    char sensor_msg[32];\n                    sprintf(sensor_msg, \"Current: %d\", current_sensor);\n                    \n                    GLCD_SetDot(0, 40);\n                    GLCD_WriteString(\"                \"); // Clear line\n                    GLCD_SetDot(0, 40);\n                    GLCD_WriteString(sensor_msg);\n                }\n                \n                if (PIND & (1 << PD2)) { // Button pressed\n                    // Take final sensor reading\n                    uint16_t final_sensor = target_read_sensor_averaged(3);\n                    uint8_t score = target_calculate_score(final_sensor, target_values[current_round]);\n                    target_scores[current_round] = score;\n                    total_score += score;\n                    \n                    // Display results\n                    target_display_zone(score);\n                    target_play_score_sound(score);\n                    \n                    char result_msg[64];\n                    uint16_t distance = (final_sensor > target_values[current_round]) ?\n                        (final_sensor - target_values[current_round]) :\n                        (target_values[current_round] - final_sensor);\n                    \n                    sprintf(result_msg, \"Score: %d (off by %d)\", score, distance);\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"ROUND COMPLETE\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(result_msg);\n                    \n                    if (score >= ZONE_PERFECT) {\n                        GLCD_SetDot(0, 32);\n                        GLCD_WriteString(\"PERFECT HIT!\");\n                    } else if (score >= ZONE_EXCELLENT) {\n                        GLCD_SetDot(0, 32);\n                        GLCD_WriteString(\"Excellent!\");\n                    } else if (score >= ZONE_GOOD) {\n                        GLCD_SetDot(0, 32);\n                        GLCD_WriteString(\"Good shot!\");\n                    } else if (score >= ZONE_FAIR) {\n                        GLCD_SetDot(0, 32);\n                        GLCD_WriteString(\"Keep trying!\");\n                    } else {\n                        GLCD_SetDot(0, 32);\n                        GLCD_WriteString(\"Try again!\");\n                    }\n                    \n                    uart_string(result_msg);\n                    uart_string(\"\\r\\n\");\n                    \n                    target_state = TARGET_SCORING;\n                    \n                    // Wait for button release\n                    while (PIND & (1 << PD2));\n                }\n                \n                round_start_time++;\n                if (round_start_time >= TARGET_TIMEOUT_MS) {\n                    // Timeout\n                    target_scores[current_round] = ZONE_MISS;\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"TIMEOUT!\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"No response\");\n                    \n                    uart_string(\"Timeout! Moving to next round.\\r\\n\");\n                    \n                    target_state = TARGET_SCORING;\n                }\n                \n                _delay_ms(1);\n                break;\n            }\n            \n            case TARGET_SCORING: {\n                static uint16_t score_timer = 0;\n                score_timer++;\n                \n                if (score_timer >= SCORE_DISPLAY_TIME || (PIND & (1 << PD2))) {\n                    score_timer = 0;\n                    current_round++;\n                    PORTB = 0x00; // Turn off LEDs\n                    target_state = TARGET_READY;\n                    \n                    // Wait for button release if pressed\n                    while (PIND & (1 << PD2));\n                }\n                \n                _delay_ms(1);\n                break;\n            }\n            \n            case TARGET_RESULTS: {\n                uint16_t avg_score = total_score / MAX_TARGET_ROUNDS;\n                \n                GLCD_ClearScreen();\n                GLCD_WriteString(\"FINAL RESULTS\");\n                \n                char stats_line1[32], stats_line2[32], stats_line3[32];\n                sprintf(stats_line1, \"Total: %d/%d\", total_score, MAX_TARGET_ROUNDS * 100);\n                sprintf(stats_line2, \"Average: %d\", avg_score);\n                sprintf(stats_line3, \"Rating: %s\", target_get_rating(avg_score));\n                \n                GLCD_SetDot(0, 16);\n                GLCD_WriteString(stats_line1);\n                GLCD_SetDot(0, 24);\n                GLCD_WriteString(stats_line2);\n                GLCD_SetDot(0, 32);\n                GLCD_WriteString(stats_line3);\n                \n                uart_string(\"\\r\\n=== FINAL STATISTICS ===\\r\\n\");\n                uart_string(stats_line1); uart_string(\"\\r\\n\");\n                uart_string(stats_line2); uart_string(\"\\r\\n\");\n                sprintf(stats_line1, \"Perfect hits: %d\", perfect_hits);\n                uart_string(stats_line1); uart_string(\"\\r\\n\");\n                sprintf(stats_line1, \"Good hits: %d\", good_hits);\n                uart_string(stats_line1); uart_string(\"\\r\\n\");\n                uart_string(stats_line3); uart_string(\"\\r\\n\");\n                \n                uart_string(\"\\r\\nPress button to restart.\\r\\n\");\n                \n                if (PIND & (1 << PD2)) {\n                    // Restart game\n                    current_round = 0;\n                    total_score = 0;\n                    perfect_hits = 0;\n                    good_hits = 0;\n                    target_state = TARGET_CALIBRATION;\n                    \n                    GLCD_ClearScreen();\n                    GLCD_WriteString(\"SENSOR TARGET\");\n                    GLCD_SetDot(0, 16);\n                    GLCD_WriteString(\"Press to calibrate\");\n                    \n                    // Wait for button release\n                    while (PIND & (1 << PD2));\n                }\n                break;\n            }\n            \n            default:\n                target_state = TARGET_IDLE;\n                break;\n        }\n    }\n    \n    return 0;\n}\n\n#endif // GAME_SENSOR_TARGET

											/*
											 * GAME_HANGMAN - Word Guessing Game
											 * Educational demonstration of:
											 * - String manipulation and processing
											 * - Character input and validation
											 * - Game logic and state management
											 * - Array processing and searching
											 * - User interface design
											 */

#ifdef GAME_HANGMAN

														// Hangman game state definitions
														typedef enum {
															HANGMAN_IDLE,
															HANGMAN_SETUP,
															HANGMAN_PLAYING,
															HANGMAN_INPUT,
															HANGMAN_WIN,
															HANGMAN_LOSE,
															HANGMAN_RESULTS
														} hangman_state_t;

// Game configuration
#define MAX_WORD_LENGTH 16
#define MAX_WRONG_GUESSES 6
#define ALPHABET_SIZE 26
#define NUM_WORDS 20

													// Global hangman variables
													static hangman_state_t hangman_state = HANGMAN_IDLE;
													static char current_word[MAX_WORD_LENGTH];
													static char guessed_word[MAX_WORD_LENGTH];
													static uint8_t guessed_letters[ALPHABET_SIZE];
													static uint8_t wrong_guesses = 0;
													static uint8_t current_word_index = 0;
													static uint8_t games_won = 0;
													static uint8_t games_played = 0;

													// Word bank for hangman game
													static const char *word_bank[NUM_WORDS] = {
														"MICROCONTROLLER", "ASSEMBLY", "PROGRAMMING", "EMBEDDED", "ARDUINO",
														"INTERRUPT", "REGISTER", "PROCESSOR", "VOLTAGE", "CURRENT",
														"SENSOR", "ACTUATOR", "DISPLAY", "PROTOCOL", "DIGITAL",
														"ANALOG", "CIRCUIT", "SIGNAL", "MEMORY", "TIMER"};

													// Initialize new hangman game
													void hangman_init_game(void)
													{
														// Select random word
														static uint32_t seed = 98765;
														seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
														current_word_index = seed % NUM_WORDS;

														// Copy word to current_word
														strcpy(current_word, word_bank[current_word_index]);

														// Initialize guessed word with underscores
														uint8_t word_len = strlen(current_word);
														for (uint8_t i = 0; i < word_len; i++)
														{
															if (current_word[i] == ' ')
															{
																guessed_word[i] = ' '; // Keep spaces visible
															}
															else
															{
																guessed_word[i] = '_';
															}
														}
														guessed_word[word_len] = '\0';

														// Clear guessed letters
														for (uint8_t i = 0; i < ALPHABET_SIZE; i++)
														{
															guessed_letters[i] = 0;
														}

														wrong_guesses = 0;
														hangman_state = HANGMAN_PLAYING;

														char game_msg[64];
														sprintf(game_msg, "Word has %d letters", word_len);

														uart_string("\\r\\n=== NEW HANGMAN GAME ===\\r\\n");
														uart_string(game_msg);
														uart_string("\\r\\n");
													}

													// Check if letter has been guessed
													uint8_t hangman_is_letter_guessed(char letter)
													{
														if (letter >= 'A' && letter <= 'Z')
														{
															return guessed_letters[letter - 'A'];
														}
														return 1; // Invalid letters are considered "guessed"
													}

													// Process a guessed letter
													uint8_t hangman_process_guess(char letter)
													{
														if (letter >= 'a' && letter <= 'z')
														{
															letter = letter - 'a' + 'A'; // Convert to uppercase
														}

														if (letter < 'A' || letter > 'Z')
														{
															return 0; // Invalid character
														}

														if (hangman_is_letter_guessed(letter))
														{
															return 0; // Already guessed
														}

														// Mark letter as guessed
														guessed_letters[letter - 'A'] = 1;

														// Check if letter is in the word
														uint8_t found = 0;
														uint8_t word_len = strlen(current_word);

														for (uint8_t i = 0; i < word_len; i++)
														{
															if (current_word[i] == letter)
															{
																guessed_word[i] = letter;
																found = 1;
															}
														}

														if (!found)
														{
															wrong_guesses++;
														}

														return 1; // Valid guess processed
													}

													// Check if word is completely guessed
													uint8_t hangman_is_word_complete(void)
													{
														uint8_t word_len = strlen(current_word);
														for (uint8_t i = 0; i < word_len; i++)
														{
															if (current_word[i] != ' ' && guessed_word[i] == '_')
															{
																return 0; // Still has unguessed letters
															}
														}
														return 1; // Word is complete
													}

													// Draw hangman figure based on wrong guesses
													void hangman_draw_figure(uint8_t wrong_count)
													{
														GLCD_SetDot(0, 40);

														switch (wrong_count)
														{
														case 0:
															GLCD_WriteString("         ");
															break;
														case 1:
															GLCD_WriteString("  +---   ");
															break;
														case 2:
															GLCD_WriteString("  +---+  ");
															break;
														case 3:
															GLCD_WriteString("  +---+  ");
															GLCD_SetDot(0, 48);
															GLCD_WriteString("  |   O  ");
															break;
														case 4:
															GLCD_WriteString("  +---+  ");
															GLCD_SetDot(0, 48);
															GLCD_WriteString("  |   O  ");
															GLCD_SetDot(0, 56);
															GLCD_WriteString("  |   |  ");
															break;
														case 5:
															GLCD_WriteString("  +---+  ");
															GLCD_SetDot(0, 48);
															GLCD_WriteString("  |   O  ");
															GLCD_SetDot(0, 56);
															GLCD_WriteString("  |  /|  ");
															break;
														case 6:
															GLCD_WriteString("  +---+  ");
															GLCD_SetDot(0, 48);
															GLCD_WriteString("  |   O  ");
															GLCD_SetDot(0, 56);
															GLCD_WriteString("  |  /|\\ ");
															break;
														default:
															break;
														}
													}

													// Display guessed letters
													void hangman_display_guessed_letters(void)
													{
														uart_string("Guessed letters: ");
														uint8_t first = 1;

														for (uint8_t i = 0; i < ALPHABET_SIZE; i++)
														{
															if (guessed_letters[i])
															{
																if (!first)
																	uart_string(", ");
																char letter_str[2] = {(char)('A' + i), '\0'};
																uart_string(letter_str);
																first = 0;
															}
														}

														if (first)
														{
															uart_string("(none)");
														}
														uart_string("\\r\\n");
													}

													// Get letter input via UART
													char hangman_get_letter_input(void)
													{
														uart_string("Enter a letter (A-Z): ");

														// Wait for character input
														while (!(UCSR0A & (1 << RXC0)))
															; // Wait for receive complete
														char input = UDR0;

														// Echo the character
														while (!(UCSR0A & (1 << UDRE0)))
															; // Wait for transmit ready
														UDR0 = input;
														uart_string("\\r\\n");

														return input;
													}

													int main_game_hangman(void)
													{
														init_devices();

														// Initialize display
														GLCD_Initialize();
														GLCD_ClearScreen();

														uart_string("\\r\\n=== HANGMAN EDUCATIONAL DEMO ===\\r\\n");
														uart_string("Word guessing game demonstrating:\\r\\n");
														uart_string("- String manipulation and processing\\r\\n");
														uart_string("- Character input and validation\\r\\n");
														uart_string("- Game logic and state management\\r\\n");
														uart_string("- Array processing and searching\\r\\n\\r\\n");

														uart_string("Instructions:\\r\\n");
														uart_string("1. Guess letters to reveal the hidden word\\r\\n");
														uart_string("2. You have 6 wrong guesses before losing\\r\\n");
														uart_string("3. Enter letters via UART terminal\\r\\n\\r\\n");

														// Display initial screen
														GLCD_WriteString("HANGMAN GAME");
														GLCD_SetDot(0, 16);
														GLCD_WriteString("Press button to start");

														hangman_state = HANGMAN_SETUP;
														games_won = 0;
														games_played = 0;

														while (1)
														{
															switch (hangman_state)
															{
															case HANGMAN_SETUP:
															{
																if (PIND & (1 << PD2))
																{ // Button pressed
																	hangman_init_game();
																	games_played++;

																	// Wait for button release
																	while (PIND & (1 << PD2))
																		;
																}
																break;
															}

															case HANGMAN_PLAYING:
															{
																// Update display
																GLCD_ClearScreen();
																GLCD_WriteString("HANGMAN");
																GLCD_SetDot(0, 16);
																GLCD_WriteString(guessed_word);
																GLCD_SetDot(0, 24);

																char status_msg[32];
																sprintf(status_msg, "Wrong: %d/%d", wrong_guesses, MAX_WRONG_GUESSES);
																GLCD_WriteString(status_msg);

																// Draw hangman figure
																hangman_draw_figure(wrong_guesses);

																// Display current status via UART
																uart_string("\\r\\nCurrent word: ");
																uart_string(guessed_word);
																uart_string("\\r\\n");
																uart_string(status_msg);
																uart_string("\\r\\n");
																hangman_display_guessed_letters();

																hangman_state = HANGMAN_INPUT;
																break;
															}

															case HANGMAN_INPUT:
															{
																char guessed_letter = hangman_get_letter_input();

																if (hangman_process_guess(guessed_letter))
																{
																	// Valid guess
																	char guess_msg[32];
																	sprintf(guess_msg, "You guessed: %c", guessed_letter);
																	uart_string(guess_msg);

																	// Check if letter was in word
																	uint8_t found = 0;
																	uint8_t word_len = strlen(current_word);
																	for (uint8_t i = 0; i < word_len; i++)
																	{
																		if (current_word[i] == guessed_letter)
																		{
																			found = 1;
																			break;
																		}
																	}

																	if (found)
																	{
																		uart_string(" - CORRECT!\\r\\n");
																		buzzer_play_frequency(800, 200); // Success sound
																	}
																	else
																	{
																		uart_string(" - Wrong!\\r\\n");
																		buzzer_play_frequency(300, 300); // Wrong sound
																	}

																	// Check win/lose conditions
																	if (hangman_is_word_complete())
																	{
																		hangman_state = HANGMAN_WIN;
																	}
																	else if (wrong_guesses >= MAX_WRONG_GUESSES)
																	{
																		hangman_state = HANGMAN_LOSE;
																	}
																	else
																	{
																		hangman_state = HANGMAN_PLAYING;
																	}
																}
																else
																{
																	// Invalid or already guessed
																	uart_string("Invalid letter or already guessed. Try again.\\r\\n");
																}
																break;
															}

															case HANGMAN_WIN:
															{
																games_won++;

																GLCD_ClearScreen();
																GLCD_WriteString("YOU WIN!");
																GLCD_SetDot(0, 16);
																GLCD_WriteString(current_word);
																GLCD_SetDot(0, 24);
																GLCD_WriteString("Congratulations!");

																uart_string("\\r\\n*** CONGRATULATIONS! ***\\r\\n");
																uart_string("You guessed the word: ");
																uart_string(current_word);
																uart_string("\\r\\n");

																char stats_msg[64];
																sprintf(stats_msg, "Wrong guesses: %d/%d", wrong_guesses, MAX_WRONG_GUESSES);
																uart_string(stats_msg);
																uart_string("\\r\\n");

																// Play victory sound
																for (uint8_t i = 0; i < 3; i++)
																{
																	buzzer_play_frequency(1000 + (i * 200), 200);
																	_delay_ms(100);
																}

																hangman_state = HANGMAN_RESULTS;
																break;
															}

															case HANGMAN_LOSE:
															{
																GLCD_ClearScreen();
																GLCD_WriteString("GAME OVER");
																GLCD_SetDot(0, 16);
																GLCD_WriteString("Word was:");
																GLCD_SetDot(0, 24);
																GLCD_WriteString(current_word);

																// Draw complete hangman
																hangman_draw_figure(MAX_WRONG_GUESSES);

																uart_string("\\r\\n*** GAME OVER ***\\r\\n");
																uart_string("The word was: ");
																uart_string(current_word);
																uart_string("\\r\\n");
																uart_string("Better luck next time!\\r\\n");

																// Play game over sound
																buzzer_play_frequency(200, 1000);

																hangman_state = HANGMAN_RESULTS;
																break;
															}

															case HANGMAN_RESULTS:
															{
																static uint16_t result_timer = 0;
																result_timer++;

																if (result_timer >= 3000 || (PIND & (1 << PD2)))
																{
																	result_timer = 0;

																	// Display overall statistics
																	uart_string("\\r\\n=== GAME STATISTICS ===\\r\\n");
																	char stats[64];
																	sprintf(stats, "Games played: %d", games_played);
																	uart_string(stats);
																	uart_string("\\r\\n");
																	sprintf(stats, "Games won: %d", games_won);
																	uart_string(stats);
																	uart_string("\\r\\n");

																	if (games_played > 0)
																	{
																		uint16_t win_percentage = (games_won * 100) / games_played;
																		sprintf(stats, "Win rate: %d%%", win_percentage);
																		uart_string(stats);
																		uart_string("\\r\\n");
																	}

																	uart_string("\\r\\nPress button for new game.\\r\\n");

																	GLCD_ClearScreen();
																	GLCD_WriteString("HANGMAN GAME");
																	GLCD_SetDot(0, 16);
																	GLCD_WriteString("Press button to start");
																	GLCD_SetDot(0, 24);
																	sprintf(stats, "Won: %d/%d", games_won, games_played);
																	GLCD_WriteString(stats);

																	hangman_state = HANGMAN_SETUP;

																	// Wait for button release if pressed
																	while (PIND & (1 << PD2))
																		;
																}

																_delay_ms(1);
																break;
															}

															default:
																hangman_state = HANGMAN_IDLE;
																break;
															}

															_delay_ms(1); // Small delay for timing
														}

														return 0;
													}

#endif // GAME_HANGMAN

											/*
											 * GAME_OBSTACLE - Real-time Obstacle Avoidance Game
											 * Educational demonstration of:
											 * - Real-time game logic and collision detection
											 * - Graphics animation and movement
											 * - Input handling and response
											 * - Score tracking and difficulty progression
											 * - Frame-based game loop design
											 */

#ifdef GAME_OBSTACLE

													// Obstacle game state definitions
													typedef enum
													{
														OBSTACLE_IDLE,
														OBSTACLE_MENU,
														OBSTACLE_PLAYING,
														OBSTACLE_PAUSED,
														OBSTACLE_GAME_OVER,
														OBSTACLE_HIGH_SCORE
													} obstacle_state_t;

// Game configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define PLAYER_WIDTH 8
#define PLAYER_HEIGHT 8
#define OBSTACLE_WIDTH 8
#define OBSTACLE_HEIGHT 16
#define MAX_OBSTACLES 6
#define OBSTACLE_SPEED_BASE 2
#define SCORE_SPEED_INCREASE 10

													// Player position and movement
													typedef struct
													{
														uint8_t x;
														uint8_t y;
														int8_t dx;
														int8_t dy;
													} player_t;

													// Obstacle structure
													typedef struct
													{
														uint8_t x;
														uint8_t y;
														uint8_t active;
														uint8_t speed;
													} obstacle_t;

													// Global obstacle game variables
													static obstacle_state_t obstacle_state = OBSTACLE_IDLE;
													static player_t player;
													static obstacle_t obstacles[MAX_OBSTACLES];
													static uint16_t score = 0;
													static uint16_t high_score = 0;
													static uint8_t difficulty_level = 1;
													static uint16_t frame_counter = 0;
													static uint8_t game_speed = 1;

													// Initialize player
													void obstacle_init_player(void)
													{
														player.x = 20;
														player.y = SCREEN_HEIGHT / 2 - PLAYER_HEIGHT / 2;
														player.dx = 0;
														player.dy = 0;
													}

													// Initialize obstacles
													void obstacle_init_obstacles(void)
													{
														for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
														{
															obstacles[i].active = 0;
															obstacles[i].x = SCREEN_WIDTH + (i * 30);
															obstacles[i].y = 10 + (i % 3) * 15;
															obstacles[i].speed = OBSTACLE_SPEED_BASE + (difficulty_level / 2);
														}
													}

													// Spawn new obstacle
													void obstacle_spawn_obstacle(uint8_t index)
													{
														if (!obstacles[index].active)
														{
															obstacles[index].active = 1;
															obstacles[index].x = SCREEN_WIDTH;

															// Random height with some safe zones
															static uint32_t seed = 54321;
															seed = (seed * 1664525 + 1013904223) & 0xFFFFFFFF;

															uint8_t safe_zones[] = {8, 24, 40}; // Safe passage areas
															uint8_t zone = seed % 3;
															obstacles[index].y = safe_zones[zone] + ((seed >> 8) % 8) - 4;

															// Ensure obstacle stays on screen
															if (obstacles[index].y < 4)
																obstacles[index].y = 4;
															if (obstacles[index].y > SCREEN_HEIGHT - OBSTACLE_HEIGHT - 4)
															{
																obstacles[index].y = SCREEN_HEIGHT - OBSTACLE_HEIGHT - 4;
															}

															obstacles[index].speed = OBSTACLE_SPEED_BASE + (difficulty_level / 2);
														}
													}

													// Update player position
													void obstacle_update_player(void)
													{
														// Handle input (buttons for movement)
														player.dx = 0;
														player.dy = 0;

														// Read button inputs (assuming buttons on PIND)
														if (PIND & (1 << PD0))
															player.dx = -2; // Move left
														if (PIND & (1 << PD1))
															player.dx = 2; // Move right
														if (PIND & (1 << PD2))
															player.dy = -2; // Move up
														if (PIND & (1 << PD3))
															player.dy = 2; // Move down

														// Update position
														player.x += player.dx;
														player.y += player.dy;

														// Keep player on screen
														if (player.x < 2)
															player.x = 2;
														if (player.x > 40)
															player.x = 40; // Keep player on left side
														if (player.y < 2)
															player.y = 2;
														if (player.y > SCREEN_HEIGHT - PLAYER_HEIGHT - 2)
														{
															player.y = SCREEN_HEIGHT - PLAYER_HEIGHT - 2;
														}
													}

													// Update obstacles
													void obstacle_update_obstacles(void)
													{
														for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
														{
															if (obstacles[i].active)
															{
																obstacles[i].x -= obstacles[i].speed;

																// Remove obstacles that have moved off screen
																if (obstacles[i].x < 0)
																{
																	obstacles[i].active = 0;
																	score += 10; // Points for surviving an obstacle
																}
															}
														}

														// Spawn new obstacles periodically
														static uint8_t spawn_counter = 0;
														spawn_counter++;

														uint8_t spawn_rate = 60 - (difficulty_level * 5); // Faster spawning at higher levels
														if (spawn_rate < 20)
															spawn_rate = 20;

														if (spawn_counter >= spawn_rate)
														{
															spawn_counter = 0;

															// Find inactive obstacle to spawn
															for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
															{
																if (!obstacles[i].active)
																{
																	obstacle_spawn_obstacle(i);
																	break;
																}
															}
														}
													}

													// Check collisions
													uint8_t obstacle_check_collision(void)
													{
														for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
														{
															if (obstacles[i].active)
															{
																// Simple rectangle collision detection
																if (player.x < obstacles[i].x + OBSTACLE_WIDTH &&
																	player.x + PLAYER_WIDTH > obstacles[i].x &&
																	player.y < obstacles[i].y + OBSTACLE_HEIGHT &&
																	player.y + PLAYER_HEIGHT > obstacles[i].y)
																{
																	return 1; // Collision detected
																}
															}
														}
														return 0; // No collision
													}

													// Draw player
													void obstacle_draw_player(void)
													{
														// Draw simple player sprite (rectangle with dot)
														for (uint8_t y = 0; y < PLAYER_HEIGHT; y++)
														{
															for (uint8_t x = 0; x < PLAYER_WIDTH; x++)
															{
																if (x == 0 || x == PLAYER_WIDTH - 1 || y == 0 || y == PLAYER_HEIGHT - 1)
																{
																	GLCD_SetDot(player.x + x, player.y + y);
																}
															}
														}

														// Add center dot
														GLCD_SetDot(player.x + PLAYER_WIDTH / 2, player.y + PLAYER_HEIGHT / 2);
													}

													// Draw obstacles
													void obstacle_draw_obstacles(void)
													{
														for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
														{
															if (obstacles[i].active)
															{
																// Draw obstacle as filled rectangle
																for (uint8_t y = 0; y < OBSTACLE_HEIGHT; y++)
																{
																	for (uint8_t x = 0; x < OBSTACLE_WIDTH; x++)
																	{
																		if (obstacles[i].x + x < SCREEN_WIDTH &&
																			obstacles[i].y + y < SCREEN_HEIGHT)
																		{
																			GLCD_SetDot(obstacles[i].x + x, obstacles[i].y + y);
																		}
																	}
																}
															}
														}
													}

													// Draw game interface
													void obstacle_draw_interface(void)
													{
														GLCD_ClearScreen();

														// Draw player
														obstacle_draw_player();

														// Draw obstacles
														obstacle_draw_obstacles();

														// Draw score (using LED display as well)
														PORTB = (score / 10) & 0xFF; // Display score on LEDs

														// Draw borders
														for (uint8_t x = 0; x < SCREEN_WIDTH; x++)
														{
															GLCD_SetDot(x, 0);
															GLCD_SetDot(x, SCREEN_HEIGHT - 1);
														}

														for (uint8_t y = 0; y < SCREEN_HEIGHT; y++)
														{
															GLCD_SetDot(0, y);
															GLCD_SetDot(SCREEN_WIDTH - 1, y);
														}
													}

													// Update difficulty
													void obstacle_update_difficulty(void)
													{
														uint8_t new_level = (score / 100) + 1;
														if (new_level > difficulty_level && new_level <= 10)
														{
															difficulty_level = new_level;

															char level_msg[32];
															sprintf(level_msg, "Level %d! Speed increased!", difficulty_level);
															uart_string(level_msg);
															uart_string("\\r\\n");

															// Play level up sound
															buzzer_play_frequency(800, 100);
															_delay_ms(50);
															buzzer_play_frequency(1000, 100);
															_delay_ms(50);
															buzzer_play_frequency(1200, 200);
														}
													}

													int main_game_obstacle(void)
													{
														init_devices();

														// Initialize display
														GLCD_Initialize();
														GLCD_ClearScreen();

														uart_string("\\r\\n=== OBSTACLE AVOIDANCE GAME ===\\r\\n");
														uart_string("Real-time game demonstrating:\\r\\n");
														uart_string("- Collision detection algorithms\\r\\n");
														uart_string("- Real-time graphics animation\\r\\n");
														uart_string("- Input handling and response\\r\\n");
														uart_string("- Progressive difficulty scaling\\r\\n\\r\\n");

														uart_string("Controls:\\r\\n");
														uart_string("PD0 - Move Left\\r\\n");
														uart_string("PD1 - Move Right\\r\\n");
														uart_string("PD2 - Move Up\\r\\n");
														uart_string("PD3 - Move Down\\r\\n");
														uart_string("Any button - Start/Pause\\r\\n\\r\\n");

														// Display initial screen
														GLCD_WriteString("OBSTACLE AVOID");
														GLCD_SetDot(0, 16);
														GLCD_WriteString("Press any button");
														GLCD_SetDot(0, 24);
														GLCD_WriteString("to start game");

														obstacle_state = OBSTACLE_MENU;
														score = 0;
														difficulty_level = 1;
														frame_counter = 0;

														while (1)
														{
															switch (obstacle_state)
															{
															case OBSTACLE_MENU:
															{
																if (PIND & 0x0F)
																{ // Any of the first 4 buttons pressed
																	obstacle_init_player();
																	obstacle_init_obstacles();
																	score = 0;
																	difficulty_level = 1;
																	frame_counter = 0;

																	obstacle_state = OBSTACLE_PLAYING;

																	uart_string("Game started! Avoid the obstacles!\\r\\n");

																	// Wait for button release
																	while (PIND & 0x0F)
																		;
																}
																break;
															}

															case OBSTACLE_PLAYING:
															{
																// Update game logic
																obstacle_update_player();
																obstacle_update_obstacles();
																obstacle_update_difficulty();

																// Check for collisions
																if (obstacle_check_collision())
																{
																	obstacle_state = OBSTACLE_GAME_OVER;

																	// Check for new high score
																	if (score > high_score)
																	{
																		high_score = score;
																	}

																	// Play crash sound
																	buzzer_play_frequency(200, 500);

																	char crash_msg[64];
																	sprintf(crash_msg, "CRASH! Final score: %d", score);
																	uart_string(crash_msg);
																	uart_string("\\r\\n");

																	break;
																}

																// Draw everything
																obstacle_draw_interface();

																// Check for pause
																if (PIND & (1 << PD4))
																{ // Pause button
																	obstacle_state = OBSTACLE_PAUSED;
																	uart_string("Game paused.\\r\\n");
																	while (PIND & (1 << PD4))
																		; // Wait for release
																}

																frame_counter++;
																_delay_ms(50); // Control game speed
																break;
															}

															case OBSTACLE_PAUSED:
															{
																GLCD_ClearScreen();
																GLCD_WriteString("GAME PAUSED");
																GLCD_SetDot(0, 16);
																GLCD_WriteString("Press button");
																GLCD_SetDot(0, 24);
																GLCD_WriteString("to continue");

																if (PIND & 0x1F)
																{ // Any button pressed
																	obstacle_state = OBSTACLE_PLAYING;
																	uart_string("Game resumed.\\r\\n");
																	while (PIND & 0x1F)
																		; // Wait for release
																}
																break;
															}

															case OBSTACLE_GAME_OVER:
															{
																GLCD_ClearScreen();
																GLCD_WriteString("GAME OVER");
																GLCD_SetDot(0, 16);

																char score_str[32];
																sprintf(score_str, "Score: %d", score);
																GLCD_WriteString(score_str);

																GLCD_SetDot(0, 24);
																sprintf(score_str, "Level: %d", difficulty_level);
																GLCD_WriteString(score_str);

																GLCD_SetDot(0, 32);
																sprintf(score_str, "Best: %d", high_score);
																GLCD_WriteString(score_str);

																GLCD_SetDot(0, 48);
																GLCD_WriteString("Press to restart");

																// Display statistics
																char stats_msg[64];
																sprintf(stats_msg, "Level reached: %d", difficulty_level);
																uart_string(stats_msg);
																uart_string("\\r\\n");
																sprintf(stats_msg, "High score: %d", high_score);
																uart_string(stats_msg);
																uart_string("\\r\\n");
																sprintf(stats_msg, "Frames survived: %d", frame_counter);
																uart_string(stats_msg);
																uart_string("\\r\\n");

																if (PIND & 0x1F)
																{ // Any button pressed
																	obstacle_state = OBSTACLE_MENU;

																	GLCD_ClearScreen();
																	GLCD_WriteString("OBSTACLE AVOID");
																	GLCD_SetDot(0, 16);
																	GLCD_WriteString("Press any button");
																	GLCD_SetDot(0, 24);
																	GLCD_WriteString("to start game");

																	uart_string("\\r\\nReady for new game!\\r\\n");

																	while (PIND & 0x1F)
																		; // Wait for release
																}
																break;
															}

															default:
																obstacle_state = OBSTACLE_IDLE;
																break;
															}

															_delay_ms(10); // Main loop timing
														}

														return 0;
													}

#endif // GAME_OBSTACLE"
#ifdef GAME_SIMON_SAYS

													// Game state definitions
													typedef enum
													{
														GAME_IDLE,
														GAME_SHOW_SEQUENCE,
														GAME_WAIT_INPUT,
														GAME_SUCCESS,
														GAME_FAILURE
													} simon_state_t;

// Game configuration
#define MAX_SEQUENCE_LENGTH 16
#define SEQUENCE_DISPLAY_TIME 800
#define INPUT_TIMEOUT 3000
#define SUCCESS_DISPLAY_TIME 1000
#define FAILURE_DISPLAY_TIME 2000

													// Global game variables
													static uint8_t game_sequence[MAX_SEQUENCE_LENGTH];
													static uint8_t current_level = 1;
													static uint8_t input_position = 0;
													static simon_state_t game_state = GAME_IDLE;
													static uint16_t score = 0;

													// Function to generate random LED pattern (0-7 for 8 LEDs)
													uint8_t simon_get_random_led(void)
													{
														static uint16_t seed = 1;
														seed = (seed * 25173 + 13849) & 0xFFFF; // Simple PRNG
														return seed % 8;
													}

													// Display LED pattern for Simon Says
													void simon_display_led(uint8_t led_num)
													{
														PORTB = 0x00; // Clear all LEDs
														if (led_num < 8)
														{
															PORTB = (1 << led_num); // Light specific LED
														}
													}

													// Play sound for LED activation
													void simon_play_sound(uint8_t led_num)
													{
														uint16_t frequency = 220 + (led_num * 55); // Different frequency for each LED
														buzzer_play_frequency(frequency, 200);
													}

													// Initialize new game
													void simon_init_game(void)
													{
														game_state = GAME_SHOW_SEQUENCE;
														current_level = 1;
														input_position = 0;
														score = 0;

														// Generate first sequence element
														game_sequence[0] = simon_get_random_led();

														// Display game start
														uart_string("=== SIMON SAYS GAME ===\r\n");
														uart_string("Watch the LED sequence, then repeat it!\r\n");
														uart_string("Use buttons 0-7 to match the pattern.\r\n\r\n");

														GLCD_ClearScreen();
														GLCD_WriteString("SIMON SAYS");
														GLCD_SetDot(0, 16);
														GLCD_WriteString("Level: 1");
														GLCD_SetDot(0, 32);
														GLCD_WriteString("Score: 0");
													}

													// Show the current sequence
													void simon_show_sequence(void)
													{
														static uint8_t sequence_index = 0;
														static uint16_t display_timer = 0;

														if (display_timer == 0)
														{
															// Start showing current LED
															simon_display_led(game_sequence[sequence_index]);
															simon_play_sound(game_sequence[sequence_index]);
															display_timer = SEQUENCE_DISPLAY_TIME / 2; // LED on time
														}
														else if (display_timer == SEQUENCE_DISPLAY_TIME / 2)
														{
															// Turn off LED (pause between LEDs)
															PORTB = 0x00;
															display_timer--;
														}
														else if (display_timer == 1)
														{
															// Move to next LED in sequence
															sequence_index++;
															if (sequence_index >= current_level)
															{
																// Sequence complete, wait for user input
																sequence_index = 0;
																input_position = 0;
																game_state = GAME_WAIT_INPUT;
																uart_string("Your turn! Press buttons to repeat sequence.\r\n");
															}
															display_timer = 0;
														}
														else
														{
															display_timer--;
														}
													}

													// Check user input
													void simon_check_input(void)
													{
														uint8_t button_pressed = 0xFF;

														// Check for button press (assuming PIND for input)
														for (uint8_t i = 0; i < 8; i++)
														{
															if (PIND & (1 << i))
															{
																button_pressed = i;
																break;
															}
														}

														if (button_pressed != 0xFF)
														{
															// Button was pressed
															simon_display_led(button_pressed);
															simon_play_sound(button_pressed);

															// Check if correct
															if (button_pressed == game_sequence[input_position])
															{
																input_position++;

																if (input_position >= current_level)
																{
																	// Level complete!
																	score += current_level * 10;
																	current_level++;

																	if (current_level > MAX_SEQUENCE_LENGTH)
																	{
																		// Game won!
																		uart_string("CONGRATULATIONS! You've mastered Simon Says!\r\n");
																		game_state = GAME_SUCCESS;
																	}
																	else
																	{
																		// Generate next sequence element
																		game_sequence[current_level - 1] = simon_get_random_led();
																		game_state = GAME_SHOW_SEQUENCE;

																		// Update display
																		char level_str[32];
																		char score_str[32];
																		sprintf(level_str, "Level: %d", current_level);
																		sprintf(score_str, "Score: %d", score);

																		GLCD_ClearScreen();
																		GLCD_WriteString("SIMON SAYS");
																		GLCD_SetDot(0, 16);
																		GLCD_WriteString(level_str);
																		GLCD_SetDot(0, 32);
																		GLCD_WriteString(score_str);

																		uart_string("Level complete! Next level...\r\n");
																	}
																}
															}
															else
															{
																// Wrong button!
																game_state = GAME_FAILURE;
																uart_string("Wrong button! Game Over.\r\n");

																char final_score[64];
																sprintf(final_score, "Final Score: %d (Level %d)\r\n", score, current_level);
																uart_string(final_score);
															}

															// Wait for button release
															while (PIND & (1 << button_pressed))
																;
															PORTB = 0x00; // Turn off LED
														}
													}

													// Handle success state
													void simon_handle_success(void)
													{
														static uint16_t success_timer = SUCCESS_DISPLAY_TIME;

														// Flash all LEDs in celebration
														static uint8_t flash_state = 0;
														if ((success_timer % 100) == 0)
														{
															flash_state = !flash_state;
															PORTB = flash_state ? 0xFF : 0x00;
														}

														success_timer--;
														if (success_timer == 0)
														{
															PORTB = 0x00;
															simon_init_game(); // Start new game
															success_timer = SUCCESS_DISPLAY_TIME;
														}
													}

													// Handle failure state
													void simon_handle_failure(void)
													{
														static uint16_t failure_timer = FAILURE_DISPLAY_TIME;

														// Display game over
														GLCD_ClearScreen();
														GLCD_WriteString("GAME OVER");
														GLCD_SetDot(0, 16);

														char final_display[32];
														sprintf(final_display, "Score: %d", score);
														GLCD_WriteString(final_display);
														GLCD_SetDot(0, 32);
														GLCD_WriteString("Press any key");

														failure_timer--;
														if (failure_timer == 0 || (PIND & 0xFF))
														{
															PORTB = 0x00;
															simon_init_game(); // Start new game
															failure_timer = FAILURE_DISPLAY_TIME;
														}
													}

													int main_game_simon_says(void)
													{
														init_devices();

														// Initialize display
														GLCD_Initialize();
														GLCD_ClearScreen();

														uart_string("\r\n=== SIMON SAYS EDUCATIONAL DEMO ===\r\n");
														uart_string("Interactive memory game demonstrating:\r\n");
														uart_string("- Game state machines\r\n");
														uart_string("- Pattern generation and display\r\n");
														uart_string("- User input handling\r\n");
														uart_string("- Score tracking and progression\r\n\r\n");

														simon_init_game();

														while (1)
														{
															switch (game_state)
															{
															case GAME_SHOW_SEQUENCE:
																simon_show_sequence();
																break;

															case GAME_WAIT_INPUT:
																simon_check_input();
																break;

															case GAME_SUCCESS:
																simon_handle_success();
																break;

															case GAME_FAILURE:
																simon_handle_failure();
																break;

															default:
																game_state = GAME_IDLE;
																break;
															}

															_delay_ms(1); // Small delay for timing
														}

														return 0;
													}

#endif // GAME_SIMON_SAYS
