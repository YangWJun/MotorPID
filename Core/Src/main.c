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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t  speed=0;
int count =0;
int16_t  speed_now=0;
int16_t control=0;
int16_t ADD =0;
int16_t  circle =0;
int16_t  Target_Position=0;
int16_t  Target_Speed =0;
uint8_t  Data[2];
uint16_t ADC_Value[2]={0};
uint8_t  Mode_State=2;

enum Mode {
    Constant_Speed_key=1,
    Constant_Circle_P,
    Constant_Circle_N,
    Constant_Circle_Start,
    Constant_Speed_rocker,
    follow
};//定义了枚举量，来表示不同的状�??
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    OLED_Init();
    NRF24L01_Init();
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2);


    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_4);
   //   HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_3|TIM_CHANNEL_4);


    HAL_ADCEx_Calibration_Start(&hadc1);//校准ADC
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value, 2);//�?启ADC—�?�DMA转运获取摇杆数据

      if (Mode_State == follow) {
          if (R_IRQ() == 0) {
              ReceiveData(Data);
          }  //当中断置0时接收，节约资源
          uint16_t temp = (Data[1] << 8) + Data[0];
          Target_Position = -temp;
      }    //follow模式下接收从机发送的位置信息

      if (Mode_State == Constant_Speed_rocker){

          if(ADC_Value[0]>3000)
       Target_Speed=(int16_t)(80.0/1095*(ADC_Value[0]-3000));

        else if(ADC_Value[0]<2590)
              Target_Speed=(int16_t)(80.0/2590*(ADC_Value[0]-2590));

      else
          {Target_Speed=0;
          }
      }    //摇杆数据处理
      OLED_ShowString(1, 2, "Circle:");
      OLED_ShowSignedNum(1, 8, circle, 2);
      OLED_ShowString(2, 2, "Speed:");
      OLED_ShowSignedNum(2, 8, Target_Speed, 2);
      OLED_ShowSignedNum(2, 11, speed_now, 2);
      OLED_ShowString(3, 2, "Mode:");
      if (Mode_State == Constant_Speed_key)
          OLED_ShowString(3, 8, "SpeedK");
     else  if (Mode_State == Constant_Speed_rocker)
          OLED_ShowString(3, 8, "SpeedR");
      else if (Mode_State == Constant_Circle_P)
          OLED_ShowString(3, 8, "+Circle");
      else if (Mode_State == Constant_Circle_N)
          OLED_ShowString(3, 8, "-Circle");
      else if (Mode_State == follow)
          OLED_ShowString(3, 8, "Follow ");
      else if (Mode_State == Constant_Circle_Start)
          OLED_ShowString(3, 8, "Circle ");
      OLED_ShowSignedNum(4, 1, Target_Position, 4);
      OLED_ShowSignedNum(4, 7, ADD, 4);
    //  printf("%d,%d\n", ADC_Value[0],ADC_Value[1]);
   //OLED屏幕显示
      printf("%d,%d\n",Target_Speed, speed_now);
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
    if(GPIO_Pin==GPIO_PIN_2 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_RESET);
        HAL_Delay(20);
        if (Mode_State==Constant_Speed_key) {
            Mode_State=Constant_Speed_rocker;
            Motor_SetSpeed(0);
            speed_now=0;
            circle=0;
            Target_Speed=0;
        }
else if(Mode_State==Constant_Speed_rocker) {
    Mode_State =Constant_Circle_P;
    Motor_SetSpeed(0);
    speed_now=0;
    circle=0;
    Target_Speed=0;
}
 else if(Mode_State==Constant_Circle_P&&circle==0)
{  Mode_State=Constant_Circle_N;
    Motor_SetSpeed(0);
    speed_now=0;
}
  else  if(Mode_State==Constant_Circle_N&&circle==0)
{   Mode_State=follow;
    circle=0;
    Motor_SetSpeed(0);
    Target_Speed=0;
    speed_now=0;
}
  else if (Mode_State==Constant_Circle_P||Mode_State==Constant_Circle_N) {
     Target_Position=1560*circle;
    __HAL_TIM_SET_COUNTER(&htim1,0);
    Mode_State =Constant_Circle_Start;
}
else if(Mode_State==Constant_Circle_Start)
{
    Mode_State=follow;
    Target_Speed=0;
    ADD=0;
    circle=0;
    Motor_SetSpeed(0);
    speed_now=0;
    __HAL_TIM_SET_COUNTER(&htim1,0);
}
else if(Mode_State==follow)
{
    Mode_State=Constant_Speed_key;
    Target_Speed=0;
    ADD=0;
}
    }    /*摇杆按键改变模式*/
    if(GPIO_Pin==GPIO_PIN_15 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_RESET)
        { HAL_Delay(20);
            while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_RESET);//等待按键松开
            HAL_Delay(20);
          if(Mode_State==Constant_Circle_P)
                circle++;
            if(Mode_State==Constant_Circle_N)
                circle--;
            if(Mode_State==Constant_Speed_key)
            {  if (Target_Speed >= -40)
                    Target_Speed -= 20;
                else
                { Target_Speed = 0;
               }
            }
        }
        if(GPIO_Pin==GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_RESET)
        {   HAL_Delay(20);
            //while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_RESET);
            //HAL_Delay(20);
//等待按键松开
         if( Mode_State!=Constant_Circle_Start) {
             if (Target_Speed <= 40)
                 Target_Speed += 20;
             else
             { Target_Speed = 0;
                 }





         }
        }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&htim2))
    {
        if(Mode_State==Constant_Circle_Start)
        {
        speed_now=Encoder_Get();
        ADD+=speed_now;
        control= PositionControl(ADD,Target_Position);
          control= PWM_Limit(control,Target_Speed);
       if(ADD-Target_Position<5&&ADD-Target_Position>-5)
           Motor_SetSpeed(0);
       else
       {control=  SpeedControl(speed_now,control);
            control= PWM_Limit(control,1000);
       Motor_SetSpeed(control);}
}  //串级PID控制
 else  if(Mode_State==Constant_Speed_key ||Mode_State==Constant_Speed_rocker){

         speed_now = Encoder_Get();
        control = Const_SpeedControl(speed_now, Target_Speed);
         PWM_Limit(control, 1000);
       /*  if(Target_Speed==0)
             Motor_SetSpeed(0);
         else*/
       Motor_SetSpeed(control);


    }   //定速，增量式PID
        else  if(Mode_State==follow)
        {
            speed_now=Encoder_Get();
            ADD+=speed_now;
            control= Follow_PositionControl(ADD,Target_Position);
           // control= PWM_Limit(control,Target_Speed);
            control= PWM_Limit(control,1000);
            if(ADD-Target_Position<4&&ADD-Target_Position>-4)
                control=0;
           /* else
                control=  SpeedControl(speed_now,control);*/
            Motor_SetSpeed(control);
        } //跟随模式，仅位置式PID

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
