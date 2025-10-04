#ifndef _GLCD_H_
#define _GLCD_H_

typedef unsigned char byte;
typedef unsigned int word;

void cmndl(byte cmd); /* lcd 명령 출력 */
void cmndr(byte cmd);
void cmnda(byte cmd);

void datal(byte dat); /* 1 문자 출력  */
void datar(byte dat);
void dataa(byte dat);

void lcd_clear(void);                          /* GLCD Clear */
void lcd_init(void);                           /* GLCD Initialize */
void lcd_xy(byte x, byte y);                   /* 문자 위치 세트 */
void lcd_char(byte character);                 /* 한 문자 출력 */
void lcd_string(byte x, byte y, char *string); /* 문자열 출력 */

void GLCD_Axis_xy(unsigned char x, unsigned char y);
void GLCD_Dot(unsigned char xx, unsigned char y); // 점을 그립니다.
void ScreenBuffer_clear(void);
void GLCD_Line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);
void GLCD_Rectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2); // 직사각형을 그립니다
void GLCD_Circle(unsigned char x1, unsigned char y1, unsigned char r);                       // 원을 그립니다.

unsigned char GLCD_1DigitDecimal(unsigned char number, unsigned char flag); // 1자리의 10진수 값을 표시합니다.
void GLCD_2DigitDecimal(unsigned char number);                              // 2자리의 10진수 값을 표시합니다.
void GLCD_3DigitDecimal(unsigned int number);                               // 3자리의 10진수 값을 표시합니다.
void GLCD_4DigitDecimal(unsigned int number);                               // 4자리의 10진수 값을 표시합니다.

/*=========================================================================*/
/*                    ADVANCED GRAPHICS LIBRARY EXTENSIONS                */
/*=========================================================================*/

/* Drawing Modes */
typedef enum
{
    GLCD_MODE_SET = 0,   // Set pixels (normal drawing)
    GLCD_MODE_CLEAR = 1, // Clear pixels
    GLCD_MODE_XOR = 2,   // XOR pixels (toggle)
    GLCD_MODE_OR = 3,    // OR pixels
    GLCD_MODE_AND = 4    // AND pixels
} GLCD_DrawMode;

/* Enhanced Drawing Functions */
void GLCD_SetDrawMode(GLCD_DrawMode mode);                 // Set drawing mode
void GLCD_Dot_Advanced(unsigned char xx, unsigned char y); // Enhanced dot with modes

/* Filled Shapes */
void GLCD_Rectangle_Filled(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);
void GLCD_Circle_Filled(unsigned char cx, unsigned char cy, unsigned char radius);
void GLCD_Triangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char x3, unsigned char y3);
void GLCD_Triangle_Filled(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char x3, unsigned char y3);

/* Advanced Shapes */
void GLCD_Ellipse(unsigned char cx, unsigned char cy, unsigned char a, unsigned char b);
void GLCD_Polygon(unsigned char vertices[][2], unsigned char num_vertices);

/* Pattern and Bitmap Functions */
void GLCD_Pattern_Fill(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char pattern);
void GLCD_Bitmap_8x8(unsigned char x, unsigned char y, const unsigned char bitmap[8]);
void GLCD_Bitmap_16x16(unsigned char x, unsigned char y, const unsigned char bitmap[32]);

/* Enhanced Text Functions */
void GLCD_Large_Number(unsigned char x, unsigned char y, unsigned char number);
void GLCD_Scroll_Text(unsigned char y, char *text, unsigned char speed);

/* Animation Functions */
void GLCD_Animation_Frame(unsigned char x, unsigned char y, const unsigned char frames[][8], unsigned char frame_count, unsigned char current_frame);

/* UI Elements */
void GLCD_Progress_Bar(unsigned char x, unsigned char y, unsigned char width, unsigned char height, unsigned char percentage);

/* Graph/Chart Functions */
void GLCD_Bar_Chart(unsigned char x, unsigned char y, unsigned char values[], unsigned char count, unsigned char max_value);
void GLCD_Line_Graph(unsigned char x, unsigned char y, unsigned char values[], unsigned char count, unsigned char max_value);

/* Utility Functions */
void GLCD_Screen_Capture(unsigned char buffer[8][128]);
void GLCD_Screen_Restore(unsigned char buffer[8][128]);
void GLCD_Invert_Screen(void);

#endif /* _GLCD_H_ */
