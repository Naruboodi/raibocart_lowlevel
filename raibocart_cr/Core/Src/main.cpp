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
#include <main.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ros.h>
#include <ros/time.h>
#include <raibo_msgs/speed_fb.h>
#include <raibo_msgs/speed_sp.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	double pps = 0.0;
	double rpm = 0.0;
	double kmh = 0.0;
	int16_t delta_encoder = 0.0;
	int16_t angular = 0.0;
	int encoder_pose = 0;
	int steering_speed = 0;
} SPEED;

typedef struct {
	double bldc_sp = 0.0;
	double bldc_pv = 0.0;
	double angle_sp = 0.0;
	double angle_pv = 0.0;
} CONTROL;

typedef struct {
	double kp = 0;
	double ki = 0;
	double kd = 0;
	double err = 0;
	double err_l = 0;
	double sum_err = 0;
} PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_SP	0
#define PUBLISH		1
#define BLDC		2
#define STEERING	3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
//Initial variable of the control section
CONTROL _cmd_vel;
SPEED _hallBLDC, _encoder;
PID _MotorBLDC, _MotorSteering;
double speed_output;
unsigned char val[100];
unsigned char val1[100];

unsigned char param[100]; //sprintf test
unsigned char param1[100]; //sprintf test

//Initial variable of the control condition
bool initial_state=true;
uint8_t initial_count=0;

bool initial_loop = true;
bool hold_bldc_sp = false;
double pre_bldc_sp = 0;
double lastest_bldc_sp = 0;\

bool dt_init = true;
int time, time_tmp;

// Steering confirm(fix) variable
bool steering_init = false;
int setpoint_right,setpoint_left,setpoint_middle;
//teseting variable
bool steering_PID_state = false;
int number_testing = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ros::NodeHandle nh_;

raibo_msgs::speed_fb motor_fb_;
ros::Time last_time_[4];

void commandSpeedSPCallback(const raibo_msgs::speed_sp &msg);

ros::Publisher speed_fb_pub_("/micro/speed_fb", &motor_fb_);
ros::Subscriber<raibo_msgs::speed_sp> speed_sp_sub_("/micro/speed_sp",
		&commandSpeedSPCallback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh_.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh_.getHardware()->reset_rbuf();
}

void commandSpeedSPCallback(const raibo_msgs::speed_sp &msg) {
	last_time_[SPEED_SP] = nh_.now();
	_cmd_vel.bldc_sp = (double) msg.bldc_speed_sp;
	_cmd_vel.angle_sp = (double) msg.steering_speed_sp;
	motor_fb_.bldc_rpm_sp = _cmd_vel.bldc_sp;
	motor_fb_.steering_angle_sp = _cmd_vel.angle_sp;
}

void PIDControl_BLDC(PID *_pid_data, double _linear_x, double _pv, double _dt) {
	// init function var
	double dac_speed = 0;
	double min_forward = 1600;
	double min_backward = 1350;
	//prevent sum_err overflow at the start
	if(initial_state){_pid_data->err=0; _pid_data->sum_err += _pid_data->err; initial_count++;}
	if(initial_count>45){initial_state = false;}
	//assign error var
	_pid_data->err = abs(_linear_x) - _pv;
	if (_pid_data->err > 1000.0){_pid_data->err = 1000.0;}
	if (_pid_data->err < -1000.0){_pid_data->err = -1000.0;}
	_pid_data->sum_err += _pid_data->err;

	if(abs(_linear_x) > 1.2){
		dac_speed = (_pid_data->kp * _pid_data->err)
						+ (_pid_data->ki * _pid_data->sum_err * _dt)
						+ (_pid_data->kd * ((_pid_data->err - _pid_data->err_l) / _dt));
		_pid_data->err_l = _pid_data->err;

		if (dac_speed > 4095.0) dac_speed = 4095.0;
		if (dac_speed < 0) dac_speed = 0;
		// reduce scaling time
		speed_output = dac_speed;
	}
	//certain use for lower limit of the car (minimum speed at 0.9 kmh)
	if(abs(_linear_x) <= 0.3 ){_linear_x = 0.0; _pid_data->sum_err = 0; _pid_data->err = 0; speed_output = 0;}
	if(_linear_x > 0.3 && _linear_x <= 1.2){speed_output = min_forward; _pid_data->sum_err = 260; _pid_data->err = 0;}
	if(_linear_x < -0.3 && _linear_x >= -1.2){speed_output = min_backward; _pid_data->sum_err = 260; _pid_data->err = 0;}

	if(_linear_x > 0.0){
		motor_fb_.forward_motor = true;
		if(_pid_data->sum_err <= 260){dac_speed = min_forward;}
	}
	else if(_linear_x < 0.0){
		motor_fb_.forward_motor = false;
		if(_pid_data->sum_err <= 260){dac_speed = min_backward;}
	}
	else{
		motor_fb_.forward_motor = false;
	}

	motor_fb_.bldc_kmph_fb = _pv; // don't change it's for controllino

	motor_fb_.bldc_mps_fb = dac_speed; //in real situation the feedback should be speed_output for accurate speed output.

}

void DACControlSpeed(double _linear_x) {
	PIDControl_BLDC(&_MotorBLDC, _linear_x, _hallBLDC.kmh, 0.5);

	if(!hold_bldc_sp && motor_fb_.forward_motor && speed_output>=1600){
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed_output);
	}
	else if(!hold_bldc_sp && !motor_fb_.forward_motor && speed_output>=1350){
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed_output);
	}
	else if(hold_bldc_sp){
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		if(_hallBLDC.kmh < 0.2){
			HAL_Delay(500);
			hold_bldc_sp = false;
		}
	}
	else{
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	}

	motor_fb_.bldc_rpm_fb = _MotorBLDC.sum_err;
}

void Steering_Control(int sp, int ap ,int dt){
	time = HAL_GetTick();
	int err_dt = 0;
	int desire_pose_tmp = 0;
	int desire_pose = sp;

	if(desire_pose != desire_pose_tmp){dt_init = true; desire_pose_tmp = sp; /*_MotorSteering.sum_err = 0; _MotorSteering.err_l = 0;*/}
	if(dt_init){time_tmp = time; dt_init = false;}

	_MotorSteering.err = sp-ap;
	if (_MotorSteering.err > 1000.0){_MotorSteering.err = 1000.0;}
	if (_MotorSteering.err < -1000.0){_MotorSteering.err = -1000.0;}

	if(time - time_tmp >= dt){
		if(abs(_MotorSteering.err)>=10){
		_MotorSteering.sum_err += _MotorSteering.err;
		}
		err_dt = (_MotorSteering.err - _MotorSteering.err_l)/(dt/1000);
		_MotorSteering.err_l = _MotorSteering.err;
		time_tmp = time;
	}

	_encoder.steering_speed = (_MotorSteering.kp*_MotorSteering.err)
			+ (_MotorSteering.ki*_MotorSteering.sum_err)
			+ (_MotorSteering.kd * err_dt);

	if(_encoder.steering_speed > 0){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	}
	else if(_encoder.steering_speed < 0){
		_encoder.steering_speed *= -1;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else if(_encoder.steering_speed == 0){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	}

//	sprintf((char*)val,"Cureent_speed %d\r\n", _encoder.steering_speed);
//	HAL_UART_Transmit(&huart3, val, strlen((char*)val), 1000);

	if(_encoder.steering_speed > 1000) _encoder.steering_speed=1000;
	else if(_encoder.steering_speed<200 && _encoder.steering_speed>0) _encoder.steering_speed=200;

	if(abs(_MotorSteering.err)<=10){
		_encoder.steering_speed = 0;
	}
	TIM4->CCR1 = _encoder.steering_speed;
//	if(_MotorSteering.sum_err > -650.0 && _MotorSteering.sum_err < 650.0)steering_PID_state = true;
//	if(_MotorSteering.sum_err < -650.0 && _MotorSteering.sum_err > 650.0)steering_PID_state = false;

	motor_fb_.steering_angle_sp = sp;
	motor_fb_.steering_angular_fb = _encoder.encoder_pose;
	motor_fb_.steering_rpm_fb = TIM4->CCR1;
//	motor_fb_.bldc_mps_fb = _MotorSteering.err;
//	motor_fb_.bldc_kmph_fb = _MotorSteering.sum_err;
//	motor_fb_.bldc_rpm_fb = _encoder.steering_speed;

}

void Steering_Init(){
	bool set_centure = true;
	bool set_wheel = false;
	bool seton_right = true;
	bool seton_left = false;
	int go_right, go_left = 0;

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.0);
	while(set_centure){
		if(seton_right){ //6
			while(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)){
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
				TIM4->CCR1 = 300;
			}
			TIM4->CCR1 = 0;
			go_right = _encoder.encoder_pose;
			seton_left = true;
			seton_right	= false;
		}
		HAL_Delay(600);
		if(seton_left){ //5
			while(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)){
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
				TIM4->CCR1 = 300;
			}
			TIM4->CCR1 = 0;
			go_left = _encoder.encoder_pose;
			setpoint_middle = (go_left + go_right)/2;
			seton_left = false;
			seton_right	= false;
		}
		set_wheel = true;
		set_centure = false;
	}
	HAL_Delay(600);
	while(set_wheel){
		Steering_Control(setpoint_middle,_encoder.encoder_pose,100);
		if(_encoder.encoder_pose<=setpoint_middle+5 && _encoder.encoder_pose>=setpoint_middle-5){
			htim3.Instance->CNT = (setpoint_middle-htim3.Instance->CNT)+500;
			TIM4->CCR1 = 0;
			setpoint_middle = 500; // +- ~190
			setpoint_left = 500 - ((go_right - go_left)/2); //300 - left
			setpoint_right = 500 + ((go_right - go_left)/2); //~700 - right
			set_wheel = false;

//			sprintf((char*)val,"Cureent %d, Left %d,Right %d\r\n", htim3.Instance->CNT, setpoint_left, setpoint_right);
//			HAL_UART_Transmit(&huart3, val, strlen((char*)val), 1000);
		}

	}
	steering_init = true;
}

void ros_setup(void) {
	nh_.initNode();
	nh_.advertise(speed_fb_pub_);
	nh_.subscribe(speed_sp_sub_);

	motor_fb_.bldc_rpm_sp = 0.0;
	motor_fb_.bldc_rpm_fb = 0.0;
	motor_fb_.bldc_mps_fb = 0.0;
	motor_fb_.bldc_kmph_fb = 0.0;
	motor_fb_.steering_angle_sp = 0.0;
	motor_fb_.steering_angular_fb = 0.0;
	motor_fb_.steering_rpm_fb = 0.0;
	motor_fb_.forward_motor = true;

	ros::Time ros_time_setup = nh_.now();

	for (int i = 0; i < 4; ++i) {
		last_time_[i] = ros_time_setup;
	}
}

void ros_loop(void) {
	ros::Time time_now = nh_.now();
	if ((time_now - last_time_[SPEED_SP]).toSec() > 1.0) {
		_cmd_vel.bldc_sp = 0.0;
		_cmd_vel.angle_sp = 0.0;
	}

	if ((time_now - last_time_[PUBLISH]).toSec() > 0.03) {
		speed_fb_pub_.publish(&motor_fb_);
		last_time_[PUBLISH] = time_now;
	}

	if ((time_now - last_time_[BLDC]).toSec() > 0.01) {
		DACControlSpeed(_cmd_vel.bldc_sp);
		last_time_[BLDC] = time_now;
	}

	if ((time_now - last_time_[STEERING]).toSec() > 0.01) {
		if(_cmd_vel.angle_sp > 37){_cmd_vel.angle_sp = 37;} // limit right
		if(_cmd_vel.angle_sp < -37){_cmd_vel.angle_sp = -37;} // limit left
		Steering_Control(int((_cmd_vel.angle_sp*10.89)+500),_encoder.encoder_pose,100);
		last_time_[STEERING] = time_now;
	}

	nh_.spinOnce();
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

// ROS
	ros_setup();

	// Interrupt
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim5);

	//PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	// Hall & Encoder
	htim3.Instance->CNT = 500;
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	// DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

	//PID Parameter Initial
	_MotorBLDC.kp = 80;
	_MotorBLDC.ki = 10;
	_MotorBLDC.kd = 0;
	_MotorBLDC.sum_err = 0;

	_MotorSteering.kp = 1.5;
	_MotorSteering.ki = 0.001;
	_MotorSteering.kd = 5;
	_MotorSteering.sum_err = 0.0;

	//Set the mid point of the encoder before initiate
	htim3.Instance->CNT = 500;
	Steering_Init();

	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  steering_PID_state = false;
//	  number_testing = 0;


	   if(initial_loop){pre_bldc_sp = _cmd_vel.bldc_sp; initial_loop = false;}
	   lastest_bldc_sp = _cmd_vel.bldc_sp;
	   if(pre_bldc_sp>0 && lastest_bldc_sp <=0){hold_bldc_sp = true; pre_bldc_sp = lastest_bldc_sp;}
	   if(pre_bldc_sp<=0 && lastest_bldc_sp >0){hold_bldc_sp = true; pre_bldc_sp = lastest_bldc_sp;}

	   ros_loop();


//	  sprintf((char*)param1,"yooooooooo....\r\n");
//	  HAL_UART_Transmit(&huart3, param1, strlen((char*)param1), 1000);
//	  Steering_Control(int((10*10.89)+500),_encoder.encoder_pose,100);
//	  while(!steering_PID_state){
//		  Steering_Control(int((-6*10.89)+500),_encoder.encoder_pose,100);
//		  TIM4->CCR1 = _encoder.steering_speed;
//		  sprintf((char*)val,"Return 0..\r\n");
//		  HAL_UART_Transmit(&huart3, val, strlen((char*)val), 1000);
//	  }


//	  HAL_Delay(1000);
//	  Steering_Control(int((0*10.89)+500),_encoder.encoder_pose,100);
//	  HAL_Delay(1000);

	  //ros_loop();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  htim1.Init.Period = 60000;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 540-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 540-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		_hallBLDC.delta_encoder = htim1.Instance->CNT - 30000;
		_hallBLDC.pps = abs(_hallBLDC.delta_encoder) * 2.0f;
		_hallBLDC.rpm = _hallBLDC.pps * 60 / 200.0;
		_hallBLDC.kmh = (_hallBLDC.rpm * 60 * 1.276096f)/1000;

//		sprintf((char*)val,"%d\r\n", _hallBLDC.delta_encoder);
//		HAL_UART_Transmit(&huart3, val, strlen((char*)val), 1000);

		htim1.Instance->CNT = 30000;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}
	if(htim->Instance == TIM5){
		if(steering_init){
			if(htim3.Instance -> CNT > setpoint_right && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)){
				htim3.Instance -> CNT = setpoint_right;
			}
			if(htim3.Instance -> CNT < setpoint_left && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)){
				htim3.Instance -> CNT = setpoint_left;
			}
		}
		_encoder.encoder_pose = htim3.Instance -> CNT;
//		sprintf((char*)val,"%d\r\n", _encoder.encoder_pose);
//		HAL_UART_Transmit(&huart3, val, strlen((char*)val), 1000);
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
