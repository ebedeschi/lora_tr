/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "hw.h"
#include "lora.h"
#include "timeServer.h"
#include "SHT2x/SHT2x.h"
#include "vcom.h"
#include "version.h"
#include "libtime/libtime.h"
#include "libsinc/libsinc.h"
#include "temp/temp.h"


/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            20000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                    ENABLE
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT							1

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi;
extern RTC_HandleTypeDef RtcHandle;
extern RTC_HandleTypeDef hrtc;

char Buffer[100];
uint16_t sT;
float   temperatureC;           //variable for temperature[°C] as float
uint8_t  error = 0;              //variable for error code. For codes see system.h

struct TimeStampStruct sincTs;
struct TimeStampStruct sendTs;
struct TimeStampStruct newTs;
struct TimeStampStruct second_sinc_ts;
struct TimeStampStruct temp_ts;

uint8_t  sinc = 0;

char Rx_indx_3, Rx_data_3[2], Rx_Buffer_3[100], Transfer_cplt_3, Tx_Buffer_3[100];

struct TimeStampStruct uartTs;
uint8_t  uart_sinc = 0;
uint8_t  uart_sinc_main = 0;
uint8_t  pause = 1;

TimerEvent_t wakeup;
TimerEvent_t setTimeTE;

/* call back when LoRa will transmit a frame*/
static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData( lora_AppData_t *AppData);

/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LoraTxData,
                                               LoraRxData};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {TX_ON_TIMER,
                                    APP_TX_DUTYCYCLE,
                                    CLASS_A,
                                    LORAWAN_ADR_ON,
									DR_5,
                                    LORAWAN_PUBLIC_NETWORK };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void OnWakeup( void )
{
//	PRINTF("OnWakeup\n");
    TimerStop( &wakeup );
}

static void OnSetTime( void )
{
    TimerStop( &setTimeTE );

	temp_ts = getRTCTime();
	PRINTF("Temp time\r\n");
	printTime(temp_ts);

	HAL_GPIO_TogglePin(OUT_PULSE_GPIO_Port, OUT_PULSE_Pin);

	setRTCTime(newTs);
//	setAlarm();
	PRINTF("---- SINC ---\r\n");
	second_sinc_ts = getRTCTime();
	PRINTF("-- SECOND SINC --\r\n");
	printTime(second_sinc_ts);
	PRINTF("---------------\r\n");

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int c=1;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC3_Init();
//  SystemClock_Config();

  /* USER CODE BEGIN 2 */

//  	char word[20];
//  	error |= SHT2x_MeasureHM(TEMP, &sT);
//  	temperatureC = SHT2x_CalcTemperatureC(sT);
//  	int d1 = temperatureC;
//  	float f2 = temperatureC - d1;
//  	int d2 = trunc(f2 * 10000);
//  	sprintf(word,"%d.%04d", d1, d2);
//  	int i = 0;
//  	for(i = 0; i<strlen(word); i++){
//  	sprintf(Buffer+i*2, "%02X", word[i]);
//  	}
//  	PRINTF("%s\n", Buffer);

//  uint32_t g_ADCValue = 0;
//  double con = 0.00118359375;
////  double con = 0.0023671875;
//  double v = 0;
//
//  HAL_GPIO_WritePin(ADCEN_GPIO_Port, ADCEN_Pin, GPIO_PIN_SET);
//
//  HAL_ADC_Start(&hadc3);
//
//  if (HAL_ADC_PollForConversion(&hadc3, 1000000) == HAL_OK)
//  {
//      g_ADCValue = HAL_ADC_GetValue(&hadc3);
//  }
//
//  v = con * g_ADCValue;


  HAL_GPIO_WritePin(RFPOWER_GPIO_Port, RFPOWER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_SET);

//  hspi = hspi1;

  /* Configure the hardware*/
  HW_Init( );
//  HW_RTC_Init( );

  hrtc = RtcHandle;

  HAL_UART_Receive_IT(&huart3, Rx_data_3, 1);

//  setAlarm(0);
//  PRINTF("delay 10\n");
//  HAL_Delay(10000); //delay

//  while(uart_sinc_main == 0)
//  {
//	  PRINTF("Wait uart sinc\r\n");
//	  HAL_Delay(1000); //delay
//  }
//  uart_sinc_main = 0;
//  HAL_Delay(5000); //delay

//	initTimeStampStruct(&sincTs);
//	initTimeStampStruct(&sendTs);
//	sincTs = getTimeStampStructfromMillisec(1444444444444);
//	setRTCTime(sincTs);

  /* Configure the Lora Stack*/
  lora_Init( &LoRaMainCallbacks, &LoRaParamInit);

  PRINTF("VERSION: %X\n", VERSION);

  initElementToBeSent();

  struct TimeStampStruct cts = getTimeStampStructfromMillisec(1501671314440);
  setRTCTime(cts);
//  setAcquire(true);
  setAcquire(false);
  startSinc();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* run the LoRa class A state machine*/
	  lora_fsm( );

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//	TimerInit( &wakeup, OnWakeup );
//	TimerSetValue( &wakeup, 5000 );
//	TimerStart( &wakeup );

	DISABLE_IRQ( );
    if ( lora_getDeviceState( ) == DEVICE_STATE_SLEEP )
    {
		  LowPower_Handler( );
    }
	ENABLE_IRQ();

//	  PRINTF("Hello %d\r\n", c++);
//	  //if(c%10==0)
//	    printStat();
//	    struct TempStruct ele;
//	  if(c%15==0)
//	  {
//		  ele = extract();
//			PRINTF("Quanti %d\r\n", ele.ct);
//			struct TimeStampStruct ts =  ele.t;
//			printTime(ts);
//	  }
//	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	  HAL_Delay(1000); //delay
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed)
{
  //uint8_t batteryLevel;

  uint32_t i = 0;

  //batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  *IsTxConfirmed =  LORAWAN_CONFIRMED_MSG;

  struct SincStatus status = getStatus();

	if(status._packet_sinc == 1)
	{
		if( (status._send1==false) || ( (status._send1==true) && (status._receive1==false) ) || ( (status._send1==true) && (status._receive1==true) && (status._send2==true) && (status._send2==false) ) )
		{
			AppData->Port = 6;
			AppData->Buff[i++] = 's';
			AppData->BuffSize = i;
			setStatusSend1(true);
			setStatusReceive1(false);
			setStatusSend2(false);
			setStatusReceive2(false);
			PRINTF("Send 1\r\n");
		}
		else if( (status._send1==true) && (status._receive1==true) && (status._send2==false) )
		{
			AppData->Port = 7;
			AppData->Buff[i++] = 'f';
			AppData->BuffSize = i;
			setStatusSend2(true);
			PRINTF("Send 2\r\n");
		}

	}
	else
	{

		 struct TempStruct ele;
		 uint64_t t_ms = 0;
		 if(checkExtract()==1)
		 {
			if(getSendSatusOfElementToBeSent())
			{
				ele = extract();
				setElementToBeSent(ele);
			}
			else
				ele = getElementToBeSent();

			PRINTF("Quanti %d\r\n", ele.ct);
			struct TimeStampStruct ts =  ele.t;
			printTime(ts);

			memcpy(&(AppData->Buff[0]), &(ele.ct), 1);
			t_ms = getMillisec(ele.t);
			memcpy(&(AppData->Buff[1]), &t_ms, 8);
			i = 1 + 8;
			for(int k=0;k<ele.ct;k++)
			{
				memcpy(&(AppData->Buff[i]), &((ele.temp[k])), 4);
				i+=4;
			}
			AppData->BuffSize = i;
			AppData->Port = 1;

			PRINTF("Send ele\r\n");
		 }

	}

}

static void LoraRxData( lora_AppData_t *AppData )
{
  uint8_t s = 0;
  unsigned long long ts = 0;
  unsigned long long nl=0;
  PRINTF("LoraRxData\r\n");
  switch (AppData->Port)
  {
  case LORAWAN_APP_PORT:
	  for(int i=0; i<AppData->BuffSize; i++)
		  PRINTF("%c", (char)(AppData->Buff[i]));
    break;
  case 6:
	  sincRTC( getTimeStampFromBuffer(AppData->Buff, AppData->BuffSize) );
	  stopSinc();
	  setAcquire(true);
	  PRINTF("Stop sinc\r\n");
//	  for(int i=(AppData->BuffSize-1); i>=0; i--)
//	  {
//		  nl = (AppData->Buff[i]);
//		  ts |= nl << s;
//		  s+=8;
//	  }
//	  sincTs = getTimeStampStructfromMillisec(ts);

//	  //sinc = true;
//
//	  uint64_t sinc_ts = getMicrosec(sincTs);
//	  uint64_t send_ts = getMicrosec(getSendTs());
//
//	  struct TimeStampStruct nowTs = getRTCTime();
//	  uint64_t now_ts = getMicrosec(nowTs);
//	  uint64_t diff = 0;
//	  uint64_t new_ts = 0;
//	  diff = now_ts - send_ts;
//	  new_ts = sinc_ts + diff;
////	  if(now_ts >= send_ts)
////	  {
////		  diff = now_ts - send_ts;
////		  new_ts = sinc_ts + diff;
////	  }
////	  else
////	  {
////		  diff = send_ts - now_ts;
////		  new_ts = sinc_ts - diff;
////	  }
//	 newTs = getTimeStampStructfromMicrosec(new_ts);
//
////		PRINTF("New time prima\r\n");
////		printTime(newTs);
//
////	if(sinc == true)
////	{
//		uint32_t fra = (1000000 - newTs.tv.tv_usec)/1000;
//		newTs.tv.tv_sec = newTs.tv.tv_sec + 1;
//		newTs.tv.tv_usec = 0;
//		extendTimeStampStruct(&newTs);
//
//		sinc = 1;
//
//		fra -= 8;
//
////		TimerInit( &setTimeTE, OnSetTime );
////		TimerSetValue( &setTimeTE, fra );
////		TimerStart( &setTimeTE );
//		PRINTF("fra %d\r\n", fra);
//		HAL_Delay(fra);
//		setRTCTime(newTs);
//		temp_ts = getRTCTime();
//		PRINTF("Temp time\r\n");
//		printTime(temp_ts);
////		setAlarm();
////		PRINTF("---- SINC ---\r\n");
////		sinc = false;
////	}
//
//	PRINTF("Sinc time\r\n");
//	printTime(sincTs);
//	PRINTF("Now time\r\n");
//	printTime(nowTs);
//	int diff2 = 0;
////	if(sinc_ts >= send_ts)
//		diff2 = sinc_ts - send_ts;
////	else
////		diff2 = - (send_ts - sinc_ts);
//	PRINTF("diff2 = %d\r\n", (int)diff2);
//	PRINTF("diff = %d\r\n", (int)diff);
//	PRINTF("New time dopo\r\n");
//	printTime(newTs);
////	pause = 1;

    break;
  default:
    break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

//	if (huart->Instance == USART1)	//current UART
//	{
//		Rx_Buffer_1[Rx_indx_1++]=Rx_data_1[0];	//add data to Rx_Buffer
//
//		if (Rx_data_1[0]==10)			//if received data = 13
//		{
//			Rx_Buffer_1[Rx_indx_1]='\0';
//			Rx_indx_1=0;
//			Transfer_cplt_1=1;//transfer complete, data is ready to read
//
//			int n = sprintf(Tx_Buffer_1, "%s", Rx_Buffer_1);
//			//HAL_UART_Transmit(&huart1, (uint8_t*) &Tx_Buffer_1, n, 1000);
//			HAL_UART_Transmit(&huart3, (uint8_t*) &Tx_Buffer_1, n, 1000);
//		}
//
//		HAL_UART_Receive_IT(&huart1, Rx_data_1, 1);	//activate UART receive interrupt every time
//	}
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	if (huart->Instance == USART3)	//current UART
	{
		Rx_Buffer_3[Rx_indx_3++] = Rx_data_3[0];	//add data to Rx_Buffer

		if (Rx_data_3[0] == 10)			//if received data = 13
		{
			Rx_Buffer_3[Rx_indx_3] = '\0';
			Rx_indx_3 = 0;
			Transfer_cplt_3 = 1;	//transfer complete, data is ready to read

			int n = sprintf(Tx_Buffer_3, "%s", Rx_Buffer_3);
			HAL_UART_Transmit(&huart3, (uint8_t*) &Tx_Buffer_3, n, 1000);

			  unsigned long long ts = 0;
			  unsigned long long nl=0;
			  unsigned long long s = 1;
			  char *ptr;

			  for(int i=(n-2); i>=0; i--)
			  {
				  nl = (uint8_t)(Tx_Buffer_3[i] - '0');
				  ts += nl * s;
				  s*=10;
			  }
			  uartTs = getTimeStampStructfromMillisec((uint64_t)ts);
			  printTime(uartTs);
			  uart_sinc = 1;
//			  pause=0;
		}

		HAL_UART_Receive_IT(&huart3, Rx_data_3, 1);	//activate UART receive interrupt every time
	}
}

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
