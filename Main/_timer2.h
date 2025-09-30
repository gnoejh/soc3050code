
void Timer2_init(void);

void Timer2_init(void);
void Timer2_start(void);
void Timer2_stop(void);
ISR(TIMER2_OVF_vect);

extern unsigned int Count_Of_Timer2;	
extern unsigned int Task1_Of_Timer2;	
extern unsigned int Time_Of_Timer2;	

