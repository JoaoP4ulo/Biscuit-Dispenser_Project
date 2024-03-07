/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include <unistd.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANALOG_READS 4
#define VOLTAGE_REF 3.3
#define ADC_TO_VOLT VOLTAGE_REF/4096.0
#define QTD_MEDIA 100

#define BLOCOCALIBRACAO	1000.0
#define AJUSTEVIBRACAO 15 // mudei Valor 13950
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint32_t ADCReadings[ANALOG_READS];
uint32_t anBuff[ANALOG_READS];
bool ADC_Complete = false;


float valorMassa = 0;
float valorMassaAux1 = 0;
float valorMassaAux2 = 0;
float valorMassaAux3 = 0;

uint8_t byteRx;
bool txOK = false;
bool rxOK = false;

struct Struct_mensagemQ{
	float peso;
	float pesoTara;
	float pesoPadrao;
} mensagemQ;

uint8_t page = 0;

float peso;
float pesoTara = 1445;
float pesoPadrao = 2251;

uint16_t pesoLimite1 = 30;
uint16_t pesoLimite2 = 80;
uint16_t pesoLimite3 = 90;

uint16_t potencia1 = 100;
uint16_t potencia2 = 100;
uint16_t potencia3 = 100;

bool vibrar = false;
bool vibrando = false;

volatile uint8_t estadoVibracao = 0;

uint16_t ADC_ContRead = 0;
uint16_t leituraAN2[QTD_MEDIA] = {0};

uint16_t timer_bfr = 0; // Timer before
uint16_t timer_bfr2 = 0;
uint16_t timer_bfr3 = 0;
uint16_t timer_curr = 0;


uint8_t byte;
uint8_t delimiter = '@';
uint8_t rxBuf[11];
uint8_t buffer[11];
uint8_t rxIndex = 0;

uint8_t TX_Buffer[40];
uint8_t txSize;
uint8_t str_valorMedido[7];
uint8_t pesoSize = 0;

char bufAux[10];
uint8_t contAux = 0;
uint8_t contPV = 0;
bool motor = false;

float pesoMedido = 0.0;

bool si2 = false;

uint16_t timer_diff;
uint16_t timer_diff2;
uint16_t timer_diff3;

int16_t flag= -1;

uint8_t delimeterCont = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);
void StartComTask(void const * argument);
uint8_t conv_toText(uint8_t *str, uint16_t valor);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */




	HAL_TIM_Base_Start(&htim4);


	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	
	

	HAL_ADCEx_Calibration_Start(&hadc1);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCReadings, ANALOG_READS);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		timer_curr = HAL_GetTick();

		

	/* ------------------------------------Start - ADC Reading-----------------------------------------------*/

		timer_diff = timer_curr - timer_bfr;

		if (timer_diff >= 5){ // a cada 5ms é feito a leitura das portas ADC

			if(ADC_Complete) {

				ADC_Complete = false;
				HAL_ADC_Stop_DMA(&hadc1);

				for(int i = 0 ; i < ANALOG_READS; i++){
					anBuff[i] = ADCReadings[i];
				}
					

				leituraAN2[ADC_ContRead] = anBuff[1];

				if(ADC_ContRead == (QTD_MEDIA-1)) {
					ADC_ContRead = 0;
				} else {
					ADC_ContRead++;
				}

				//Média

				valorMassaAux2 = 0;

				for(int i = 0; i < QTD_MEDIA; i++) {
					valorMassaAux2 += (float)leituraAN2[i];
				}

				valorMassaAux2 = valorMassaAux2/(float)QTD_MEDIA;

				peso = valorMassaAux2;


				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCReadings, ANALOG_READS);
					
			}
			timer_bfr = timer_curr;
		}

	/* ------------------------------------Stop - ADC Reading-----------------------------------------------*/

	/* ------------------------------------SI_ENABLE SET OFF-----------------------------------------------*/

		HAL_GPIO_WritePin(SI_ENABLE_GPIO_Port, SI_ENABLE_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);

	/* ----------------------------------------------------------------------------------------------------*/

	/* ------------------------------------Start - IHM Communication-----------------------------------------*/

		timer_diff2 = timer_curr - timer_bfr2;

		

		// if (timer_diff2 >= 300){ // a cada 300ms recebe um novo byte
			
			
		// 	timer_bfr2 = timer_curr;
		// }

			

			if(rxOK) {

				// Recebendo valores Display Nextion
				if(rxBuf[0]==0xff){
					flag = 0;

					switch(rxBuf[1]) {
						case 0xfa:
							page = rxBuf[2];
							flag = 1;
							
							break;

						case 0xe0: // Calibração
							if(rxBuf[2] == 0) { // Tara
								pesoTara = peso;
							} else { // Padrão
								pesoPadrao = peso;

								// set level baixo PWM
								__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
								__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
								__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
							}
							flag = 2;
							break;

						case 0xf0:

							pesoLimite1 = rxBuf[3];
							pesoLimite1 = rxBuf[2] + (pesoLimite1<<8);
							flag = 3;
							break;

						case 0xf1:
							pesoLimite2 = rxBuf[3];
							pesoLimite2 = rxBuf[2] + (pesoLimite2<<8);
							flag = 4;
							break;
						case 0xf2:
							pesoLimite3 = rxBuf[3];
							pesoLimite3 = rxBuf[2] + (pesoLimite3<<8);
							flag = 5;
							break;
						case 0xf3:
							potencia2 = rxBuf[3];
							potencia2 = rxBuf[2] + (potencia2<<8);
							flag = 6;
							break;
						case 0xf4:
							potencia1 = rxBuf[3];
							potencia1 = rxBuf[2] + (potencia1<<8);
							flag = 7;
							break;
						case 0xf5:
							potencia3 = rxBuf[3];
							potencia3 = rxBuf[2] + (potencia3<<8);
							flag = 8;
							break;
						// case 0xb0: //Não utilizado
						// 	if(rxBuf[rxIndex-1] == 0) {
						// 		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
						// 	} else {
						// 		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, potencia1);
						// 	}
						// 	flag = 9;
						// 	break;

						// case 0xb1: //Não utilizado
						// 	if(rxBuf[rxIndex-1] == 0) {
						// 		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
						// 	} else {
						// 		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, potencia2);
						// 	}
						// 	flag = 10;
						// 	break;

						// case 0xb2: //Não utilizado
						// 	if(rxBuf[rxIndex-1] == 0) {
						// 		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
						// 	} else {
						// 		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, potencia3);
						// 	}
						// 	flag = 11;
						// 	break;
					}
					rxOK = false;
				}

				flag = -3;

			}
			
			
			HAL_UART_Receive_IT(&huart1, &byteRx, 1);

			if(peso < pesoTara) {
				pesoMedido = 0;
			} else {
				pesoMedido = BLOCOCALIBRACAO*((peso-pesoTara)/(pesoPadrao-pesoTara));
			}


			switch (page) { // mudei sem o dois
				case 0:
					vibrar = false;
					vibrando = false;
					estadoVibracao = 5;
					break;
				case 1:
					// Página de Configuração
					break;
				case 2:
					// Página de Operação
					// Enviar Dados de Peso

					pesoSize = conv_toText(str_valorMedido, pesoMedido);
					TX_Buffer[0]='t';
					TX_Buffer[1]='5';
					TX_Buffer[2]='.';
					TX_Buffer[3]='t';
					TX_Buffer[4]='x';
					TX_Buffer[5]='t';
					TX_Buffer[6]='=';
					TX_Buffer[7]='"';

					for(int i = 0; i<pesoSize; i++) {
						TX_Buffer[8+i] = str_valorMedido[i];
					}

					TX_Buffer[8+pesoSize]='"';
					TX_Buffer[9+pesoSize]=0xFF;
					TX_Buffer[10+pesoSize]=0xFF;
					TX_Buffer[11+pesoSize]=0xFF;
					txSize = 12+pesoSize;

					if(estadoVibracao == 5) {
						estadoVibracao = 0;
					}

	/* ------------------------------------Sart - IHM / Vibration Control-----------------------------------------*/
					timer_diff3 = timer_curr - timer_bfr3;
						

					if(timer_diff3 >= 100) {

						if (vibrar){ // a cada 100ms o programa verifica o estado de vibração

							if(!vibrando) {

								// se não estiver vibrando, set PWM em 0
								__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
								__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
								__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
								HAL_Delay(500);

								if(potencia2>0)	{
									__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, round(AJUSTEVIBRACAO * (potencia2/100)));
								}
								if(potencia1>0) {
									__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,round(AJUSTEVIBRACAO * (potencia1/100)));
								}
								if(potencia3>0)	{
									__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, round(AJUSTEVIBRACAO * (potencia3/100)));
								}
								
								estadoVibracao = 1;
								vibrando = true;
							} else {
									
								if(pesoMedido > pesoLimite1) { // 

									__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
									__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
									__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
									estadoVibracao = 4;
									vibrando = false;
									vibrar = false;

								} else {
									if(pesoMedido > (pesoLimite3*pesoLimite1)/100.0) {
										__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
										__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
										estadoVibracao = 3;
									} else {
										if(estadoVibracao == 3) {
											__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
											__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
											__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
											HAL_Delay(300);
											if(potencia1>0)	{
												__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, round(AJUSTEVIBRACAO * (potencia1/100)));
											}
											if(potencia3>0)	{
												__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, round(AJUSTEVIBRACAO * (potencia3/100)));
											}
											estadoVibracao = 2;
										} else {
											if(pesoMedido > (pesoLimite2*pesoLimite1)/100.0) {
												__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
												estadoVibracao = 2;
												HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
											} else {
												if(estadoVibracao==2) {
													__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
													__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
													__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
													HAL_Delay(1000); // mudei 300
													if(potencia2>0)	{
														__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, round(AJUSTEVIBRACAO * (potencia2/100)));
													}
													if(potencia1>0)	{
														__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, round(AJUSTEVIBRACAO * (potencia1/100)));
													}
													if(potencia3>0)	{
														__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, round(AJUSTEVIBRACAO * (potencia3/100)));
													}
													HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
													estadoVibracao = 1;
												} 
											}
										}
									}
								}
							}
							
						} else {
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
							estadoVibracao = 0;
						}


						
						timer_bfr3 = timer_curr;

						
					}

	/* ------------------------------------Stop - IHM / Vibration Control-----------------------------------------*/


					TX_Buffer[txSize]='v';
					TX_Buffer[txSize+1]='a';
					TX_Buffer[txSize+2]='0';
					TX_Buffer[txSize+3]='.';
					TX_Buffer[txSize+4]='v';
					TX_Buffer[txSize+5]='a';
					TX_Buffer[txSize+6]='l';
					TX_Buffer[txSize+7]='=';
					TX_Buffer[txSize+8]= estadoVibracao + '0';
					TX_Buffer[txSize+9]=0xFF;
					TX_Buffer[txSize+10]=0xFF;
					TX_Buffer[txSize+11]=0xFF;
					txSize = txSize + 12;

					HAL_UART_Transmit_IT(&huart1, TX_Buffer, txSize);





					break;

				case 3: // Página de Teclado
					break;
				case 4: // Página de Calibração
						// Enviar Dados de Peso
					pesoSize = conv_toText(str_valorMedido, pesoMedido);
					TX_Buffer[0]='t';
					TX_Buffer[1]='5';
					TX_Buffer[2]='.';
					TX_Buffer[3]='t';
					TX_Buffer[4]='x';
					TX_Buffer[5]='t';
					TX_Buffer[6]='=';
					TX_Buffer[7]='"';

					for(int i = 0; i<pesoSize; i++) {
						TX_Buffer[8+i] = str_valorMedido[i];
					}

					TX_Buffer[8+pesoSize]='"';
					TX_Buffer[9+pesoSize]=0xFF;
					TX_Buffer[10+pesoSize]=0xFF;
					TX_Buffer[11+pesoSize]=0xFF;
					txSize = 12+pesoSize;

					HAL_UART_Transmit_IT(&huart1, TX_Buffer, txSize);

					break;

				case 5: // Página Logo Capital
					break;
				default:
					break;
			}
		}
			
			
		
		

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc == &hadc1){
		
		ADC_Complete = true;

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	
	

	if (GPIO_Pin == ZEROCD_Pin)	{ // Detector de Zero
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  
		 
	}

	if (GPIO_Pin == BOTAO1_Pin) { // Botão I
		// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if(estadoVibracao == 0 || estadoVibracao == 4) {
			HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AUX_GPIO_Port, AUX_Pin, GPIO_PIN_SET);
			motor = true;
			
			


		}
	}

	if(GPIO_Pin == BOTAO2_Pin) { // Botão II
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}

	if(GPIO_Pin == SI1_Pin) { //Sendor Indutivo I
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		//HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_RESET);
	}

	if(GPIO_Pin == SI2_Pin)	{ // Sensor Indutivo II
		si2 = true;
		// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AUX_GPIO_Port, AUX_Pin, GPIO_PIN_RESET);
		vibrar = true;
		motor = false;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM4) {

	}
	/* USER CODE END Callback 1 */
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	txOK = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
  	{
		buffer[rxIndex] = byteRx;
		rxIndex++;

		if(byteRx == delimiter) {
			memcpy(rxBuf, buffer, sizeof(buffer));
			rxOK=true;
			rxIndex=0;
		}
		

		if (rxIndex > sizeof(buffer)) 

        {
            rxIndex = 0; 
        }

		HAL_UART_Receive_IT(&huart1, &byteRx, 1);


  	}
}

uint8_t conv_toText(uint8_t *str, uint16_t valor) {
	if(valor < 0) {
				str[6] = '!';
				str[5] = 'o';
				str[4] = 'r';
				str[3] = 'r';
				str[2] = 'E';
				str[1] = '-';
				str[0] = '-';
				return 7;
	} else {
		if(valor < 10){
			str[0] = (uint8_t)(valor%10) + '0';
			return 1;
		} else{
			if(valor < 100){
				str[1] = (uint8_t)(valor%10) + '0';
				str[0] = (uint8_t)(valor/10) + '0';
				return 2;
			} else {
				if(valor < 1000) {
					str[2] = (uint8_t)(valor%10) + '0';
					str[1] = (uint8_t)((valor%100)/10) + '0';
					str[0] = (uint8_t)(valor/100) + '0';
					return 3;
				} else {
					if(valor < 10000)	{
						str[3] = (uint8_t)(valor%10) + '0';
						str[2] = (uint8_t)((valor%100)/10) + '0';
						str[1] = (uint8_t)((valor%1000)/100) + '0';
						str[0] = (uint8_t)(valor/1000) + '0';
						return 4;
					} else {
						str[4] = (uint8_t)(valor%10) + '0';
						str[3] = (uint8_t)((valor%100)/10) + '0';
						str[2] = (uint8_t)((valor%1000)/100) + '0';
						str[1] = (uint8_t)((valor%10000)/1000) + '0';
						str[0] = (uint8_t)(valor/10000) + '0';
						return 5;
					}
				}
			}
		}
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
