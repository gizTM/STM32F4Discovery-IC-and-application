/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2559 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t address,data,x_accel,y_accel,z_accel;
uint8_t spk_setup[2],play[2],buffer[100];
uint16_t spk_out[1];
uint8_t key[] = { 0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF };	//all music note
uint8_t recorded[1000];
uint32_t timeout=10;
char input;
int k,i,period=1000,count=0;

int n = 150;
uint16_t sound_in[3000];
int temp[3000];
int buf[3000];
int binary[3000];
int vol;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void getAccel() {
	//read x-accel
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		address=0x29 + 0x80;	//address x_out @29h
		HAL_SPI_Transmit(&hspi1,&address,1,timeout);
		HAL_SPI_Receive(&hspi1,&x_accel,1,timeout);
	//read y-accel
		address=0x2B + 0x80;	//address x_out @29h
		HAL_SPI_Transmit(&hspi1,&address,1,timeout);
		HAL_SPI_Receive(&hspi1,&y_accel,1,timeout);
	//read z-accel
		address=0x2D + 0x80;	//address z_out @29h
		HAL_SPI_Transmit(&hspi1,&address,1,timeout);
		HAL_SPI_Receive(&hspi1,&z_accel,1,timeout);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
//	  int n = sprintf(buffer,"%d |%3d|%3d|%3d|\r\n",a,x_accel,y_accel,z_accel);
//	  HAL_UART_Transmit(&huart2,buffer,n,1000);
}
void getMic() {
	HAL_I2S_Receive(&hi2s2, sound_in, n, 1000);
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < 16; j++) {
				 	binary[j] = sound_in[i] % 2;
				 	sound_in[i] = sound_in[i]/2;
				}
				for (int j = 0; j < 16; j++) {
					temp[i * 16 + j] = binary[15 - j];
				}
			}

			for (int i = 7; i + 8 < n*16; i++) {
				int sum = 0;
				for (int j = -7; j <= 8; j++)
					sum += temp[i + j];
				if(sum - 8 < 0)  buf[i] = -(sum-8);
				else buf[i] = sum-8;
			}

			vol=0;
			for (int i = 14; i + 16 < n*16; i++) {
				for (int j = -7; j <= 8; j++)
					vol = vol + buf[i + j];
			}
//		nac = sprintf(buffer,"~vol: %5d\r\n",vol);
//		if(a) HAL_UART_Transmit(&huart2,buffer,nac,1000);
//	}
}
void recordEchoSound() {
	if ( HAL_UART_Receive(&huart2,&input,1,1000) == HAL_OK ) {
//      HAL_UART_Transmit(&huart2,&input,1,1000);

	  spk_setup[0] = 0x1E; spk_setup[1] = 0x20;	//set frequency continuous
	  HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);

	  play[0] = 0x1C; /*Select the right note*/
	  if (input=='q') { play[1] = key[0]; HAL_UART_Transmit(&huart2,"x ",2,1000); }
	  else if (input=='w') { play[1] = key[1]; HAL_UART_Transmit(&huart2,"C ",2,1000); }
	  else if (input=='e') { play[1] = key[2]; HAL_UART_Transmit(&huart2,"D ",2,1000); }
	  else if (input=='r') { play[1] = key[3]; HAL_UART_Transmit(&huart2,"E ",2,1000); }
	  else if (input=='t') { play[1] = key[4]; HAL_UART_Transmit(&huart2,"F ",2,1000); }
	  else if (input=='a') { play[1] = key[5]; HAL_UART_Transmit(&huart2,"G ",2,1000); }
	  else if (input=='s') { play[1] = key[6]; HAL_UART_Transmit(&huart2,"A ",2,1000); }
	  else if (input=='d') { play[1] = key[7]; HAL_UART_Transmit(&huart2,"B ",2,1000); }
	  else if (input=='f') { play[1] = key[8]; HAL_UART_Transmit(&huart2,"C' ",3,1000); }
	  else if (input=='g') { play[1] = key[9]; HAL_UART_Transmit(&huart2,"D' ",3,1000); }
	  else if (input=='z') { play[1] = key[10]; HAL_UART_Transmit(&huart2,"E' ",3,1000); }
	  else if (input=='x') { play[1] = key[11]; HAL_UART_Transmit(&huart2,"F' ",3,1000); }
	  else if (input=='c') { play[1] = key[12]; HAL_UART_Transmit(&huart2,"G' ",3,1000); }
	  else if (input=='v') { play[1] = key[13]; HAL_UART_Transmit(&huart2,"A' ",3,1000); }
	  else if (input=='b') { play[1] = key[14]; HAL_UART_Transmit(&huart2,"B' ",3,1000); }
	  else if (input=='n') { play[1] = key[15]; HAL_UART_Transmit(&huart2,"x ",2,1000); }

	  if(input=='p') {
		  getAccel();
//		  int nac = sprintf(buffer,"\r\nx:%d y:%d\r\n",x_accel,y_accel);
//		  HAL_UART_Transmit(&huart2,buffer,nac,100);
		  if(x_accel>0&&x_accel<60) {
			  int nac = sprintf(buffer,"\r\nPLAY FORWARD");
			  HAL_UART_Transmit(&huart2,buffer,nac,100);
			  if(y_accel>0&&y_accel<60) {
				  nac = sprintf(buffer,"..FAST\r\n");
				  HAL_UART_Transmit(&huart2,buffer,nac,100);
				  playRecorded(0,500);
			  } else if(y_accel>150&&y_accel<255) {
				  nac = sprintf(buffer,"..NORMALLY\r\n");
				  HAL_UART_Transmit(&huart2,buffer,nac,100);
				  playRecorded(0,1000);
			  }
		  } else if(x_accel>150&&x_accel<255) {
			  int nac = sprintf(buffer,"\r\nPLAY REVERSE");
			  HAL_UART_Transmit(&huart2,buffer,nac,100);
			  if(y_accel>0&&y_accel<60) {
				  nac = sprintf(buffer,"..FAST\r\n");
				  HAL_UART_Transmit(&huart2,buffer,nac,100);
				  playRecorded(1,500);
			  } else if(y_accel>150&&y_accel<255) {
				  nac = sprintf(buffer,"..NORMALLY\r\n");
				  HAL_UART_Transmit(&huart2,buffer,nac,100);
				  playRecorded(1,1000);
			  }
		  }

	  } else {
		  recorded[count]=play[1];
		  count++;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, play, 2, 50);
		  spk_setup[0] = 0x1E; spk_setup[1] = 0xE0;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
		  for (i=0;i<period;i++) HAL_I2S_Transmit (&hi2s3, spk_out , 0x10, 10 );
	  }
  }
}
void playRecorded(int direction,int speed) {
	if(direction==0) {
		for(k=0; k<count; k++) {
		  period=speed;
		  play[1] = recorded[k];
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, play, 2, 50);
		  spk_setup[0] = 0x1E; spk_setup[1] = 0xE0;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
		  //loop for playing notes
		  for (i=0;i<period;i++) { HAL_I2S_Transmit (&hi2s3, spk_out , 0x10, 10 );}
	  }
	}
	else if(direction==1) {
		for(k=count-1; k>=0; k--) {
			  period=speed;
			  play[1] = recorded[k];
			  HAL_I2C_Master_Transmit(&hi2c1, 0x94, play, 2, 50);
			  spk_setup[0] = 0x1E; spk_setup[1] = 0xE0;
			  HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
			  //loop for playing notes
			  for (i=0;i<period;i++) { HAL_I2S_Transmit (&hi2s3, spk_out , 0x10, 10 );}
		  }
	}

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  //init accel
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	address=0x20;		//address CTRL_REG1 @20h
	HAL_SPI_Transmit(&hspi1,&address,1,timeout);
	//  data=0x67;		//set value for CTRL_REG1 #01100111
	data=0x47;		//						   01000111
	HAL_SPI_Transmit(&hspi1,&data,1,timeout);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
  //init spk
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	spk_setup[0] = 0x47;spk_setup[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
	spk_setup[0] = 0x32;spk_setup[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
	spk_setup[0] = 0x32;spk_setup[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
	spk_setup[0] = 0x1C;spk_setup[1] = 0xAF;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
	spk_setup[0] = 0x1E;spk_setup[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);
	spk_setup[0] = 0x02;spk_setup[1] = 0x9E;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, spk_setup, 2, 50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  int nac;		//var for debuging purpose
	  getMic();
	  getAccel();

//	  nac = sprintf(buffer,"x: %d\r\n",x_accel);
//	  HAL_UART_Transmit(&huart2,buffer,nac,100);

//	  nac = sprintf(buffer,"vol: %d\r\n",vol);
//	  HAL_UART_Transmit(&huart2,buffer,nac,100);

	  if(vol>17000) {
		  nac = sprintf(buffer,"\r\nSONG RESET\r\n");
		  HAL_UART_Transmit(&huart2,buffer,nac,100);
		  count=0;
	  }
	  else recordEchoSound();
  }
  /* USER CODE END 3 */

}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = 16;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
   RCC_OscInitStruct.PLL.PLLM = 8;
   RCC_OscInitStruct.PLL.PLLN = 168;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 7;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
     Error_Handler();
   }

    /**Initializes the CPU, AHB and APB busses clocks
    */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
   {
     Error_Handler();
   }

   PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
   PeriphClkInitStruct.PLLI2S.PLLI2SN = 88;
   PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
   {
     Error_Handler();
   }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1680;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
