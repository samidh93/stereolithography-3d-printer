/*
 *  i2c-lcd.c
 *
 *  Created on: 11.12.2019
 *  Author: Sami
 */
/******************* (20*4) display i2c********************/

#include "i2c-lcd.h"
#include "stm32f3xx_hal.h"
#include "math.h"


extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly
#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup
extern float frequenz_gemessen;
extern float tempC;
extern uint32_t Anzahl_Ebenen;
extern float remainTime;
void lcd_send_cmd (char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

	// dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
// shit Left and Right function
void lcd_shift_cmd (void)
{
	for (int i=0; i<3; i++)
	{
		lcd_send_cmd(0x1C);
		HAL_Delay(1500);
	}
	for (int i=0; i<3; i++)
	{
		lcd_send_cmd(0x18);
		HAL_Delay(1500);
	}
}

void lcd_print_state_cmd (void)
{

	lcd_send_cmd(0x80|0x00); //first line
	lcd_send_string("Welcome to ");
	lcd_send_cmd(0x80|0x40); //second line
	lcd_send_string("Monster");
	lcd_send_cmd(0x80|0x14); //third line
	lcd_send_string("STL-Printer");
	lcd_send_cmd(0x80|0x54); //fourth line
	lcd_send_string("Running.. ");
	lcd_shift_cmd ();

}

void lcd_print_temp_cmd (void)
{

	lcd_send_cmd(0x80|0x00); //first line
	lcd_send_string("Polymer Ready");
	lcd_send_cmd(0x80|0x40); //second line
	lcd_send_string("Press Start");
	lcd_send_cmd(0x80|0x14); //third line
	lcd_send_string("Heating polymer");
	lcd_send_cmd(0x80|0x54); //fourth line
	lcd_send_string("initializing..");
	lcd_shift_cmd ();

}
void lcd_print_End_cmd (void)
{

	lcd_send_cmd(0x80|0x00); //first line
	lcd_send_string("Printing is");
	lcd_send_cmd(0x80|0x40); //second line
	lcd_send_string("finished");
	lcd_send_cmd(0x80|0x14); //third line
	lcd_send_string("homing");
	lcd_send_cmd(0x80|0x54); //fourth line
	lcd_send_string("goodbye");
	lcd_shift_cmd ();

}
void Temperature_print(void)
{
	lcd_send_cmd(0x80); //first line

	lcd_send_string("Temperature= ");

	lcd_send_data((fmod(tempC,100)/10)+48); // Display 2th Digit
	lcd_send_data(fmod(tempC,10)+48); // Display 1th Digit
	lcd_send_string(",");
	lcd_send_data((fmod(tempC,1)*10)+48); // Display 2th Digit
	lcd_send_data((fmod(tempC,0.1)*100)+48); // Display 1th Digit
}

void frequency_print(void)
{

	lcd_send_cmd(0x80); //first line

	lcd_send_string("Frequency= ");
	/*
	lcd_send_data((frequenz_gemessen/100000)+48); // Display 6th Digit
	lcd_send_data(((frequenz_gemessen%100000)/10000)+48); // Display 5th Digit
	lcd_send_data(((frequenz_gemessen%10000)/1000)+48); // Display 4th Digit
	lcd_send_data(((frequenz_gemessen%1000)/100)+48); // Display 3th Digit
	lcd_send_data(((frequenz_gemessen%100)/10)+48); // Display 2th Digit
	lcd_send_data(((frequenz_gemessen%10))+48); // Display 1th Digit
	 */
	lcd_send_data(fmod(frequenz_gemessen,10)+48); // Display 1th Digit
	lcd_send_string(",");
	lcd_send_data((fmod(frequenz_gemessen,1)*10)+48); // Display 2th Digit
	lcd_send_data((fmod(frequenz_gemessen,0.1)*100)+48); // Display 1th Digit

}
void getAnzahlEbenen(void)
{
	lcd_send_cmd(0x80); //first line display

	lcd_send_string("Anzahl Ebenen= ");

	lcd_send_data((Anzahl_Ebenen/100000)+48);
	lcd_send_data(((Anzahl_Ebenen%100000)/10000)+48);
	lcd_send_data(((Anzahl_Ebenen%10000)/1000)+48);
	lcd_send_data(((Anzahl_Ebenen%1000)/100)+48);
	lcd_send_data(((Anzahl_Ebenen%100)/10)+48);
	lcd_send_data(((Anzahl_Ebenen%10))+48);
}

void Show_Remain_time(void)
{

	lcd_send_cmd(0x80); //first line

	lcd_send_string("Remain Time in Hrs= ");
	/*
	lcd_send_data((frequenz_gemessen/100000)+48); // Display 6th Digit
	lcd_send_data(((frequenz_gemessen%100000)/10000)+48); // Display 5th Digit
	lcd_send_data(((frequenz_gemessen%10000)/1000)+48); // Display 4th Digit
	lcd_send_data(((frequenz_gemessen%1000)/100)+48); // Display 3th Digit
	lcd_send_data(((frequenz_gemessen%100)/10)+48); // Display 2th Digit
	lcd_send_data(((frequenz_gemessen%10))+48); // Display 1th Digit
	 */
	lcd_send_data(fmod(remainTime,10)+48); // Display 1th Digit
	lcd_send_string(",");
	lcd_send_data((fmod(remainTime,1)*10)+48); // Display 2th Digit
	lcd_send_data((fmod(remainTime,0.1)*100)+48); // Display 1th Digit

}
void Show_temp_Freq_Time_Layer(void)
{
	lcd_send_cmd(0x80|0x00); //first line

		lcd_send_string("Temperature= ");

		lcd_send_data((fmod(tempC,100)/10)+48); // Display 2th Digit
		lcd_send_data(fmod(tempC,10)+48); // Display 1th Digit
		lcd_send_string(",");
		lcd_send_data((fmod(tempC,1)*10)+48); // Display 2th Digit
		lcd_send_data((fmod(tempC,0.1)*100)+48); // Display 1th Digit

	lcd_send_cmd(0x80|0x40); //second line

		lcd_send_string("Frequency= ");

		lcd_send_data(fmod(frequenz_gemessen,10)+48); // Display 1th Digit
		lcd_send_string(",");
		lcd_send_data((fmod(frequenz_gemessen,1)*10)+48); // Display 2th Digit
		lcd_send_data((fmod(frequenz_gemessen,0.1)*100)+48); // Display 1th Digit

	lcd_send_cmd(0x80|0x14); //third line

		lcd_send_string("Anzahl Ebenen= ");

		lcd_send_data((Anzahl_Ebenen/100000)+48);
		lcd_send_data(((Anzahl_Ebenen%100000)/10000)+48);
		lcd_send_data(((Anzahl_Ebenen%10000)/1000)+48);
		lcd_send_data(((Anzahl_Ebenen%1000)/100)+48);
		lcd_send_data(((Anzahl_Ebenen%100)/10)+48);
		lcd_send_data(((Anzahl_Ebenen%10))+48);

	lcd_send_cmd(0x80|0x54); //fourth line

		lcd_send_string("Remain Time in Hrs= ");

		lcd_send_data(fmod(remainTime,10)+48); // Display 1th Digit
		lcd_send_string(",");
		lcd_send_data((fmod(remainTime,1)*10)+48); // Display 2th Digit
		lcd_send_data((fmod(remainTime,0.1)*100)+48); // Display 1th Digit

}
