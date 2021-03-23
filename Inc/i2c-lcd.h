#include "stm32f3xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_shift_cmd (void);

void lcd_clear (void);  

void lcd_print_state_cmd (void);

void lcd_print_temp_cmd (void);

void frequency_print(void);

void getAnzahlEbenen(void);

void Temperature_print(void);

void Show_Remain_time(void);

void Show_temp_Freq_Time_Layer(void);

void lcd_print_End_cmd(void);
