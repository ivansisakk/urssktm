/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t global_gpio_pin = 0;
static uint16_t clear = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t StartStop = 1;
uint16_t brzina = 0;



void delay (uint32_t us)   //Mikrosekunde
{
    __HAL_TIM_SET_COUNTER(&htim6,0);
    while ((__HAL_TIM_GET_COUNTER(&htim6))<us);
}

/*************************** LCD ***************************/
#define SLAVE_ADDRESS_LCD 0x4E

void lcd_send_cmd(char cmd)
{
    char upper_bits, lower_bits;
    uint8_t data[4];

    upper_bits = cmd & 0xF0;         // Get the upper 4 bits
    lower_bits = (cmd << 4) & 0xF0; // Shift the lower 4 bits to the upper position

    data[0] = upper_bits | 0x0C;    // Set EN=1 and RS=0
    data[1] = upper_bits | 0x08;    // Set EN=0 and RS=0
    data[2] = lower_bits | 0x0C;    // Set EN=1 and RS=0
    data[3] = lower_bits | 0x08;    // Set EN=0 and RS=0

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, data, 4, 100); // Send data via I2C
}


void lcd_send_data(char data)
{
    char upper_bits, lower_bits;
    uint8_t values[4];

    upper_bits = data & 0xF0;         // Get the upper 4 bits
    lower_bits = (data << 4) & 0xF0; // Shift the lower 4 bits to the upper position

    values[0] = upper_bits | 0x0D;   // Set EN=1 and RS=1
    values[1] = upper_bits | 0x09;   // Set EN=0 and RS=1
    values[2] = lower_bits | 0x0D;   // Set EN=1 and RS=1
    values[3] = lower_bits | 0x09;   // Set EN=0 and RS=1

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, values, 4, 100); // Send data via I2C
}


void lcd_clear (void)
{
	lcd_send_cmd (0x80);		// set cursor to the first position in the first line
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;		// set cursor to the first position in the first line and move it for col spaces
            break;
        case 1:
            col |= 0xC0;		// set cursor to the first position in the second line and move it for col spaces
            break;
    }

    lcd_send_cmd (col);
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
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);

  lcd_init();
  lcd_send_string("INITIALISING");
  HAL_Delay(2000);
  lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t rawValue;                // Varijabla za pohranu sirove ADC vrijednosti (0–4095)
  uint8_t  Speed;                   // Varijabla za brzinu izraženu u postocima (0–100)

  while (1)                         // Beskonačna petlja – glavni program se izvršava ovdje
  {
      if(StartStop % 2 != 0)        // Ako je StartStop neparan broj (motor uključen)
      {
          clear = 1;                // Postavi zastavicu za kasnije čišćenje LCD-a

          char str[20] = {0};       // Inicijalizacija stringa za prikaz na LCD-u

          switch(brzina)            // Provjera odabrane brzine
          {
              case 0:
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          // Pokreni PWM na TIM3, kanal 1
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);   // Postavi PWM ispunu na 0%
                  lcd_put_cur(0,0);                                   // Postavi kursor u prvi red LCD-a
                  sprintf(str, "Speed set 0 ");                       // Formatiraj string
                  lcd_send_string(str);                               // Prikaz stringa na LCD
                  lcd_send_data('%');                                 // Dodaj znak za postotak
                  lcd_put_cur(1,0);                                   // Prijeđi u drugi red
                  break;

              case 25:
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          // Pokreni PWM
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 250); // Postavi PWM ispunu na 25%
                  lcd_put_cur(0,0);
                  sprintf(str, "Speed set 25 ");
                  lcd_send_string(str);
                  lcd_send_data('%');
                  lcd_put_cur(1,0);
                  break;

              case 50:
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          // Pokreni PWM
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500); // Postavi PWM ispunu na 50%
                  lcd_put_cur(0,0);
                  sprintf(str, "Speed set 50 ");
                  lcd_send_string(str);
                  lcd_send_data('%');
                  lcd_put_cur(1,0);
                  break;

              case 100:
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          // Pokreni PWM
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);// Postavi PWM ispunu na 100%
                  lcd_put_cur(0,0);
                  sprintf(str, "Speed set 100 ");
                  lcd_send_string(str);
                  lcd_send_data('%');
                  lcd_put_cur(1,0);
                  break;
          }
      }
      else
      {
          if(clear == 1)            // LCD čistiš samo jednom pri promjeni načina rada
          {
              char str[20] = {0};   // Privremeni string za brisanje LCD-a
              lcd_put_cur(0,0);     // Prvi red
              sprintf(str, "                ");  // 16 praznih znakova
              lcd_send_string(str);             // Briši prvi red
              lcd_put_cur(1,0);     // Drugi red
              sprintf(str, "                ");  // Isto za drugi red
              lcd_send_string(str);             // Briši drugi red
              clear = 0;            // Resetiraj zastavicu da se ne briše opet
          }

          brzina = 0;               // Postavi brzinu na 0 – ne koristi se više unaprijed definirana brzina

          char str[20] = {0};       // Buffer za prikaz dinamičke brzine
          lcd_put_cur(0,0);         // Prvi red

          HAL_ADC_Start(&hadc1);                                 // Pokreni ADC konverziju
          HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);      // Pričekaj dok se ne završi konverzija
          rawValue = HAL_ADC_GetValue(&hadc1);                   // Učitaj sirovu ADC vrijednost
          Speed = (rawValue * 100) / 4095;                       // Pretvori u postotke (0–100%)

          lcd_put_cur(0,0);          // Prvi red
          sprintf(str, "Speed: %d ", Speed); // Formatiraj prikaz brzine
          lcd_send_string(str);      // Pošalji string na LCD
          lcd_send_data('%');        // Dodaj znak za postotak

          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);              // Pokreni PWM izlaz
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Speed*10);// Postavi PWM proporcionalno brzini
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    global_gpio_pin = GPIO_Pin;  // Pohranjujemo broj pina koji je izazvao prekid

    if(global_gpio_pin == GPIO_PIN_11) {  // Ako je prekid generiran na pinu 11
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }
    else if(global_gpio_pin == GPIO_PIN_10) {  // Ako je prekid generiran na pinu 10
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }
    else if(global_gpio_pin == GPIO_PIN_7) {  // Ako je prekid generiran na pinu 7
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }

    else if(global_gpio_pin == GPIO_PIN_4) {  // Ako je prekid generiran na pinu 4
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }
}

// Callback funkcija koja se poziva kada tajmer generira prekid (kada istekne zadano vrijeme)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM10) {  // Provjeravamo je li prekid generirao tajmer 10
        if (HAL_GPIO_ReadPin(GPIOC, global_gpio_pin) == GPIO_PIN_RESET) {  // Provjera je li pritisnut gumb (logička nula)
            if (global_gpio_pin == GPIO_PIN_11) {  // Ako je gumb na pinu 4
                StartStop++;  // Povećavamo varijablu Menu
            }

            if (StartStop % 2 != 0) {  // Ako je StartStop neparan broj (uključivanje neke opcije)
                if (global_gpio_pin == GPIO_PIN_10) {
                    brzina = 25;  // Povećavamo vrijednost varijable brzina
                }
                else if (global_gpio_pin == GPIO_PIN_7) {
                	brzina = 50;  // Smanjujemo vrijednost varijable brzina
                }
                else if (global_gpio_pin == GPIO_PIN_4) {
                    brzina = 100;  // Smanjujemo vrijednost varijable brzina
                                }
            }
        }
        HAL_TIM_Base_Stop_IT(&htim10);  // Zaustavljamo tajmer nakon što je obrada prekida završena
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
