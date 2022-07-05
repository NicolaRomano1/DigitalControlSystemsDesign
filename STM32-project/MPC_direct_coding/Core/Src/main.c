/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * Digital Control Systems Design project, Model Predictive Control with OSQP, ballast implementation
  * Nicola ROMANO         0622701549  	N.ROMANO24@STUDENTI.UNISA.IT
  * Antonio ROTONDO       0622701489  	A.ROTONDO9@STUDENTI.UNISA.IT
  * Catello SORRENTINO    06227001490 	C.SORRENTINO61@STUDENTI.UNISA.IT
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "workspace.h"
#include "osqp.h"
#include  <errno.h>
#include  <sys/unistd.h>
#include "math.h"
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
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
double Dumax=3000; // input rate limit
double umax=12;    // max input
double umin=-12;   // min input
double ymax=130;   // max output
double ymin=-130;  // min output

double X[2][5] = {{18.2668,  114.9275,  -11.3576,    0.0000, -103.5699},
                  {16.5646,  103.3782,  -10.2993,    0.0000,  -93.0789}};

double dd[28][1] = {{3000},
                    {3000},
                    {3000},
                    {3000},
                    {12},
                    {12},
                    {12},
                    {12},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130},
                    {130}};

double ddu[28][1] ={{0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {-0.1604},
                    {-0.1764},
                    {-0.1780},
                    {-0.1781},
                    {-0.1782},
                    {-0.1782},
                    {-0.1782},
                    {-0.1782},
                    {-0.1782},
                    {-0.1782},
                    {0.1604},
                    {0.1764},
                    {0.1780},
                    {0.1781},
                    {0.1782},
                    {0.1782},
                    {0.1782},
                    {0.1782},
                    {0.1782},
                    {0.1782}};

double du[28][1] = {{0},
                    {0},
                    {0},
                    {0},
                   {-1},
                   {-1},
                    {1},
                    {1},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0}};

double dy[28][3] = {{0,         0,         0},
                    {0,         0,         0},
                    {0,         0,         0},
                    {0,         0,         0},
                    {0,         0,         0},
                    {0,         0,         0},
                    {0,         0,         0},
                    {0,         0,         0},
              {-1.0997,    0.0997,   -0.0000},
              {-1.1097,    0.1097,   -0.0000},
              {-1.1107,    0.1107,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              {-1.1108,    0.1108,   -0.0000},
              { 1.0997,   -0.0997,    0.0000},
               {1.1097,   -0.1097,    0.0000},
               {1.1107,   -0.1107,    0.0000},
               {1.1108,   -0.1108,    0.0000},
               {1.1108,   -0.1108,    0.0000},
               {1.1108,   -0.1108,    0.0000},
               {1.1108,   -0.1108,    0.0000},
               {1.1108,   -0.1108,    0.0000},
               {1.1108,   -0.1108,    0.0000},
               {1.1108,   -0.1108,    0.0000}};

double dupast = 0;
double ypast[3][1] = {{0},
					  {0},
					  {0}};

double lastU = 0;
double lastTicks = 0;
double currentTicks = 0;
double Ts = 0.05;
double startTime = 0;
double deltaTime = 0;
double rfut_array[9] = {-130, 80, -60, 30, 0, -30, 60, -80, 200};
long int counterOfIterations = 0;
int reference_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//overwrite of printf to write on USART
int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		errno = EBADF;
		return -1;
	}

	// arbitrary timeout 1000
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) data, len,
			1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
}

// upddate ypast matrix
void update_ypast(double val){
    ypast[2][0] = ypast[1][0];
    ypast[1][0] = ypast[0][0];
    ypast[0][0] = val;
}

// set PWM
void setPulseFromDutyValue(double dutyVal) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // enable the motor driver

	uint16_t channelToModulate;
	uint16_t channelToStop;

	if (dutyVal > 0) {
		channelToModulate = TIM_CHANNEL_1;
		channelToStop = TIM_CHANNEL_2;
	} else {
		channelToModulate = TIM_CHANNEL_2;
		channelToStop = TIM_CHANNEL_1;
	}
	__HAL_TIM_SET_COMPARE(&htim3, channelToStop, 0);
	__HAL_TIM_SET_COMPARE(&htim3, channelToModulate, (abs(dutyVal) * ((double )htim3.Init.Period)) / 100); //cast integer value to double to correctly perform division between decimal numbers
}

// get speed by ticks
double getSpeedByDelta(double ticksDelta, double Ts) {
	return ticksDelta * 60 / (3591.84 * Ts);
}

// get ticks in the delta time
double getTicksDelta(double currentTicks, double lastTicks, double Ts) {
	double delta;

	if (abs(currentTicks - lastTicks) <= ceil(8400 * Ts))
		delta = currentTicks - lastTicks;
	else {
		if (lastTicks > currentTicks)
			delta = currentTicks + pow(2, 16) - 1 - lastTicks;
		else
			delta = currentTicks - pow(2, 16) + 1 - lastTicks;
	}
	return delta;
}

// matrix sum
void matsum(int row, int col, double mat1[row][col], double mat2[row][col], double result[row][col]){
   //pre-checks
   assert(row > 0);
   assert(col > 0);

   //function
   for(int i = 0; i<row; i++){
      for(int j = 0; j<col; j++){
         result[i][j] = mat1[i][j] + mat2[i][j];
      }
   }
}

// matrix * scalar product
void scalprod(int row, int col, double scalar, double mat[row][col], double result[row][col]){
   //pre-checks
   assert(row > 0);
   assert(col > 0);

   //function
   for(int i = 0; i<row; i++){
      for(int j = 0; j<col; j++){
         result[i][j] = mat[i][j] * scalar;
      }
   }
}

//matrix product
void matprod(int row1,int col1,int row2,int col2, double mat1[row1][col1], double mat2[row2][col2], double result[row1][col2]){
   //pre-checks
   assert(row1 > 0);
   assert(col1 > 0);
   assert(row2 > 0);
   assert(col2 > 0);
   assert(col1 == row2);

   //function
   for(int i = 0; i < row1; ++i){
      for(int j = 0; j < col2; ++j) {
         int x = 0;
         for(int k = 0; k < row2; ++k){
            x += mat1[i][k] * mat2[k][j];
         }
         result[i][j] = x;
      }
   }
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int rfut = rfut_array[0];

  while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
  	  startTime = (double) HAL_GetTick();

  	  //code to change the reference every 2.5 seconds
  	  if(counterOfIterations >= 50){
  		 reference_index++;
  		 counterOfIterations = 0;
		 rfut = rfut_array[(reference_index) % 9];
  	  }

  	  printf(" Required speed: %d RPM\n\r", rfut);

  	  //take current value of ticks, calculate speed, update ypast matrix
  	  currentTicks = (double) __HAL_TIM_GET_COUNTER(&htim1);
  	  double speed = getSpeedByDelta(getTicksDelta(currentTicks, lastTicks, deltaTime/1000), deltaTime/1000);
  	  printf(" Actual speed: %3.2f RPM\n\r", speed);
  	  update_ypast(speed);


  	  //solve the MPC problem

  	  //du*lastU
  	  double p1[28][1];
  	  scalprod(28, 1, lastU, du, p1);

  	  //ddu*Dupast
  	  double p2[28][1];
  	  scalprod(28,1,dupast,ddu,p2);

  	  //dy*ypast
  	  double p3[28][1];
  	  matprod(28,3,3,1,dy,ypast,p3);

  	  //dt = dd+du*lastU+ddu*Dupast+dy*ypast;
  	  double dt[28][1];
  	  matsum(28,1,p1,p2,dt);
  	  matsum(28,1,dt,p3,dt);
  	  matsum(28,1,dt,dd,dt);

  	  //f=X*[Dupast;ypast;rfut];
  	  double p4[5][1] = {{dupast},
  	  			  {ypast[0][0]},
  	  			  {ypast[1][0]},
  	  			  {ypast[2][0]},
  	  					{rfut}};

  	  double f[2][1] = {{0}, {0}};
  	  matprod(2,5,5,1,X,p4,f);

  	  //update the problem
  	  osqp_update_lin_cost(&workspace, (c_float *) f);
  	  osqp_update_upper_bound(&workspace, (c_float *) dt);

  	  //solve the problem
  	  osqp_solve(&workspace);

  	  //take the solution
  	  double u = (double)(&workspace)->solution->x[0] + lastU;
  	  printf(" u from OSQP: %3.2f V\n\r", u);

  	  //check bounds
  	  if(u > umax){
  	  	u = umax;
  	  }
  	  if(u < umin){
  	  	u = umin;
  	  }
  	  if(u - lastU > Dumax){
  	  	u = lastU + Dumax;
  	  }

  	  //apply u
  	  setPulseFromDutyValue(u * 100 / 12);
  	  printf(" u applied: %3.2f V\n\r", u);

  	  //calculate the osqp execution time
  	  deltaTime = (double) HAL_GetTick() - startTime; //end time - startTime
  	  printf(" OSQP exec time: %3.0f ms \n\r", deltaTime);

  	  //update values for next iteration
  	  dupast = u - lastU;
  	  lastU = u;

  	  // wait that the iteration takes 50 ms to respect the sample time
  	  while(deltaTime < Ts*1000){
  		  //idle
  		  deltaTime = (double) HAL_GetTick() - startTime;
  	  }

  	  //update for next iteration
  	  lastTicks = currentTicks;
  	  counterOfIterations++;

  	  //useful to check if the execution took too much time and exceeded Ts
  	  deltaTime = (double) HAL_GetTick() - startTime; //execution time
  	  printf(" Total exec time: %3.0f ms \n\n\r", deltaTime);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

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
