#include "config.h"

#ifdef GAME_OBSTACLE

#define JOYSTICK_DEAD_ZONE 10
#define MAX_OBSTACLES 5
#define GOAL_X 30
#define GOAL_Y 100
#define OBSTACLE_RADIUS 2
#define PLAYER_RADIUS 3
#define GOAL_RADIUS 4

unsigned int x_position = 31;
unsigned int y_position = 63;
unsigned int score = 0;
unsigned char lives = 3;

typedef struct {
	unsigned int x;
	unsigned int y;
} Point;

Point obstacles[MAX_OBSTACLES];
Point goal = {GOAL_X, GOAL_Y};
	
void show_score(void) {
	char score_text[20];
	sprintf(score_text, "Score: %d", score); // Convert the score into a string
	lcd_string(1, 0, score_text); // Display the score on the second line of the LCD
}	

void place_obstacles(void) {
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		obstacles[i].x = rand() % 64;
		obstacles[i].y = rand() % 128;
	}
}

void check_collision(void) {
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		if (abs(x_position - obstacles[i].x) < (OBSTACLE_RADIUS + PLAYER_RADIUS) &&
		abs(y_position - obstacles[i].y) < (OBSTACLE_RADIUS + PLAYER_RADIUS)) {
			lives--;
			if (lives == 0) {
				// Game over logic
				lcd_clear();
				lcd_string(0, 0, "Game Over!");
				lcd_string(0, 1, "Final Score: "); // Display final score
				char final_score_text[12];
				sprintf(final_score_text, "%d", score);
				lcd_string(12, 1, final_score_text);
				while(1); // Stop the game loop
			}
			x_position = 31; // Reset position to start
			y_position = 63;
			show_score(); // Update score display after losing a life
		}
	}

	if (abs(x_position - goal.x) < (GOAL_RADIUS + PLAYER_RADIUS) &&
	abs(y_position - goal.y) < (GOAL_RADIUS + PLAYER_RADIUS)) {
		score++;
		show_score(); // Update score display after reaching the goal
		goal.x = rand() % 64;
		goal.y = rand() % 128;
		place_obstacles(); // Place new obstacles
	}
}


void draw_game_elements(void) {
	// Draw the player
	GLCD_Circle(x_position, y_position, PLAYER_RADIUS);

	// Draw the goal
	GLCD_Circle(goal.x, goal.y, GOAL_RADIUS);

	// Draw the obstacles
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		GLCD_Circle(obstacles[i].x, obstacles[i].y, OBSTACLE_RADIUS);
	}

	// Show the score on the screen
	show_score();
}



void game_obstacle_init(void) {
	lcd_clear();
	ScreenBuffer_clear();
	lcd_string(0, 0, "Use joystick to move");
	show_score(); // Show the initial score
	_delay_ms(1000);

	place_obstacles(); // Initialize obstacles at random positions
}


void game_obstacle_update(void) {
	unsigned int joystick_x = Read_Adc_Data(3) / 16;
	unsigned int joystick_y = 127 - Read_Adc_Data(4) / 8;

	if(abs(joystick_x - 31) > JOYSTICK_DEAD_ZONE) {
		x_position += (joystick_x > 31) ? 1 : -1;
	}
	if(abs(joystick_y - 63) > JOYSTICK_DEAD_ZONE) {
		y_position += (joystick_y > 63) ? 1 : -1;
	}

	x_position = (x_position > 63) ? 63 : (x_position < 0) ? 0 : x_position;
	y_position = (y_position >127) ? 127 : (y_position < 0) ? 0 : y_position;

	check_collision();
	
	lcd_clear();
	ScreenBuffer_clear();
	draw_game_elements();
}

void main_game_obstacle(void) {
	init_devices();
	game_obstacle_init();
	while(1) {
		game_obstacle_update();
		_delay_ms(50);
	}
}
#endif




#ifdef GAME_OBSTACLE_LEVEL

#define JOYSTICK_DEAD_ZONE 10
#define MAX_OBSTACLES 5
#define GOAL_X 30
#define GOAL_Y 100
#define OBSTACLE_RADIUS 2
#define PLAYER_RADIUS 3
#define GOAL_RADIUS 4
#define POWERUP_RADIUS 2
#define MAX_POWERUPS 2

unsigned int x_position = 31;
unsigned int y_position = 63;
unsigned int score = 0;
unsigned char lives = 3;
unsigned int time_left = 100; // Time limit for each level

typedef struct {
	unsigned int x;
	unsigned int y;
} Point;

typedef struct {
	unsigned int x;
	unsigned int y;
	unsigned char active;
} PowerUp;

Point obstacles[MAX_OBSTACLES];
PowerUp powerups[MAX_POWERUPS];
Point goal = {GOAL_X, GOAL_Y};

void show_score(void) {
	char score_text[20];
	sprintf(score_text, "Score: %d", score); // Convert the score into a string
	lcd_string(1, 0, score_text); // Display the score on the second line of the LCD
}

void show_lives(void) {
	char lives_text[20];
	sprintf(lives_text, "Lives: %d", lives); // Display lives left
	lcd_string(0, 0, lives_text);
}

void show_time(void) {
	char time_text[20];
	sprintf(time_text, "Time: %d", time_left);
	lcd_string(0, 10, time_text); // Display time left
}

void place_obstacles(void) {
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		obstacles[i].x = rand() % 64;
		obstacles[i].y = rand() % 128;
	}
}

void place_powerups(void) {
	for (int i = 0; i < MAX_POWERUPS; i++) {
		powerups[i].x = rand() % 64;
		powerups[i].y = rand() % 128;
		powerups[i].active = 1;
	}
}

void move_obstacles(void) {
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		obstacles[i].x = (obstacles[i].x + 1) % 64; // Obstacles move horizontally
	}
}

void check_collision(void) {
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		if (abs(x_position - obstacles[i].x) < (OBSTACLE_RADIUS + PLAYER_RADIUS) &&
		abs(y_position - obstacles[i].y) < (OBSTACLE_RADIUS + PLAYER_RADIUS)) {
			lives--;
			if (lives == 0) {
				// Game over logic
				lcd_clear();
				lcd_string(0, 0, "Game Over!");
				lcd_string(0, 1, "Final Score: "); // Display final score
				char final_score_text[12];
				sprintf(final_score_text, "%d", score);
				lcd_string(12, 1, final_score_text);
				while(1); // Stop the game loop
			}
			x_position = 31; // Reset position to start
			y_position = 63;
			show_lives(); // Update lives display after losing a life
		}
	}

	if (abs(x_position - goal.x) < (GOAL_RADIUS + PLAYER_RADIUS) &&
	abs(y_position - goal.y) < (GOAL_RADIUS + PLAYER_RADIUS)) {
		score++;
		time_left += 10; // Add more time for reaching the goal
		show_score(); // Update score display after reaching the goal
		goal.x = rand() % 64;
		goal.y = rand() % 128;
		place_obstacles(); // Place new obstacles
		place_powerups(); // Place new power-ups
	}

	for (int i = 0; i < MAX_POWERUPS; i++) {
		if (powerups[i].active && abs(x_position - powerups[i].x) < (POWERUP_RADIUS + PLAYER_RADIUS) &&
		abs(y_position - powerups[i].y) < (POWERUP_RADIUS + PLAYER_RADIUS)) {
			lives++; // Gain extra life when collecting power-up
			powerups[i].active = 0; // Disable the power-up after collecting
			show_lives();
		}
	}
}

void draw_game_elements(void) {
	// Draw the player
	GLCD_Circle(x_position, y_position, PLAYER_RADIUS);

	// Draw the goal
	GLCD_Circle(goal.x, goal.y, GOAL_RADIUS);

	// Draw the obstacles
	for (int i = 0; i < MAX_OBSTACLES; i++) {
		GLCD_Circle(obstacles[i].x, obstacles[i].y, OBSTACLE_RADIUS);
	}

	// Draw the power-ups
	for (int i = 0; i < MAX_POWERUPS; i++) {
		if (powerups[i].active) {
			GLCD_Circle(powerups[i].x, powerups[i].y, POWERUP_RADIUS);
		}
	}

	// Show the score, lives, and time on the screen
	show_score();
	show_lives();
	show_time();
}

void game_obstacle_init(void) {
	lcd_clear();
	ScreenBuffer_clear();
	lcd_string(0, 0, "Use joystick to move");
	show_score(); // Show the initial score
	_delay_ms(1000);

	place_obstacles(); // Initialize obstacles at random positions
	place_powerups();  // Initialize power-ups at random positions
}

void game_obstacle_update(void) {
	unsigned int joystick_x = Read_Adc_Data(3) / 16;
	unsigned int joystick_y = 127 - Read_Adc_Data(4) / 8;

	if(abs(joystick_x - 31) > JOYSTICK_DEAD_ZONE) {
		x_position += (joystick_x > 31) ? 1 : -1;
	}
	if(abs(joystick_y - 63) > JOYSTICK_DEAD_ZONE) {
		y_position += (joystick_y > 63) ? 1 : -1;
	}

	x_position = (x_position > 63) ? 63 : (x_position < 0) ? 0 : x_position;
	y_position = (y_position >127) ? 127 : (y_position < 0) ? 0 : y_position;

	check_collision();
	move_obstacles(); // Move obstacles every frame
	time_left--; // Decrease the time left
	if (time_left == 0) {
		// Game over due to time running out
		lcd_clear();
		lcd_string(0, 0, "Time's up!");
		lcd_string(0, 1, "Final Score: ");
		char final_score_text[12];
		sprintf(final_score_text, "%d", score);
		lcd_string(12, 1, final_score_text);
		while(1); // Stop the game loop
	}
	
	lcd_clear();
	ScreenBuffer_clear();
	draw_game_elements();
}

void main_game_obstacle_level(void) {
	init_devices();
	game_obstacle_init();
	while(1) {
		game_obstacle_update();
		_delay_ms(50);
	}
}
#endif
