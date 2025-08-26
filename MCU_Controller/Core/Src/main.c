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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "FreeRTOS.h"	// Real Time task 세팅을 위해 사용
#include "arm_math.h"	// 선형 대수 연산을 위해 사용
#include "queue.h"		// CAN 통신 수신을 위해 사용
#include "task.h"		// Real Time task 세팅을 위해 사용
#include <string.h>
#include <stdlib.h>   // strtof
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart3;

/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DataLogging */
osThreadId_t DataLoggingHandle;
const osThreadAttr_t DataLogging_attributes = {
  .name = "DataLogging",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// Setting for CAN communication
uint8_t TxData[8], RxData[8];

// Setting for UART communication
// ===== UART3 RX 링버퍼 (뮤텍스/FreeRTOS 객체 없이 SPSC) =====
#define UART3_RBUF_SIZE    2048      // 총 수신 버퍼 크기 (2의 거듭제곱 권장)
#define UART3_LINE_MAX     1024      // 한 줄 최대 길이

static uint8_t  uart3_rbuf[UART3_RBUF_SIZE];
static volatile uint16_t uart3_widx = 0;   // ISR가 증가 (writer)
static volatile uint16_t uart3_ridx = 0;   // Task가 증가 (reader)
static uint8_t uart3_rx_byte;              // HAL 1바이트 수신용

// 한 줄 조립용 작업버퍼 (Task 전용)
static char    uart3_line[UART3_LINE_MAX];
static size_t  uart3_line_len = 0;

// 수신 메시지 필드 수: [t, x, y, z, xKp, xKi, xKd, xCut, xAw, yKp, yKi, yKd, yCut, yAw, zKp, zKi, zKd, zCut, zAw]
#define PC_MSG_FIELDS 19

// Setting for Control
#define NUM_MOTORS 3 // motor 3개
#define NUM_TASK_DEG 3 // task space 자유도 3

const float32_t Ts        = 0.002;			// time interval (sec)
const float32_t pi        = 3.141592;       // 원주율 설정
const float32_t g         = 9.80665;		// gravity acceleration (m/s^2)
uint32_t ctrl_time_ms     = 0;				// current control time (millisec)
uint32_t ctrl_time_ms_old = 0;  			// previous control time (millisec)
uint32_t logging_time_ms  = 0;   			// current logging time (millisec)

// Individual motor control reference
float32_t target_pos[NUM_MOTORS] = {
    0,  // Motor 0: rad
	0,
	0
};
float32_t target_vel[NUM_MOTORS] = {
    0,  // Motor 0: rad/s
	0,
	0
};

// Motor Structure Definition
typedef struct {
	// motor state definition
	int id;					// motor CAN id
	int current_motor_mode;		// motor enable: 1, disable: 0
	QueueHandle_t canRxQueue;  // motor CAN communication data
	float32_t encoder_pulses;
	float32_t gear_ratio;
	float32_t Kt;
	float32_t sensor_cutoff;
	float32_t pos_ref; 		// motor position reference
	float32_t vel_ref;		    // motor velocity reference
	float32_t pos;				// motor current position
	float32_t pos_old;			// motor 1 step previous position
	float32_t vel;				// motor current velocity
	float32_t vel_old;			// motor 1 step previous velocity
	float32_t acc;				// motor current acceleration
	float32_t acc_old;			// motor 1 step previous acceleration
	float32_t pos_error;		// motor current position error
	float32_t pos_error_old;	// motor 1 step previous position error
	float32_t pos_P_term;
	float32_t pos_I_term; // motor current accumulate error
	float32_t pos_I_term_old; // motor 1 step previous accumulate error
	float32_t pos_D_term; // motor current derivative error
	float32_t pos_D_term_old; // motor 1 step previous derivative error
	float32_t vel_error;		// motor current velocity error
	float32_t vel_error_old;   // motor 1 step previous velocity error
	float32_t vel_P_term;
	float32_t vel_I_term;  // motor current derivative error
	float32_t vel_I_term_old; // motor 1 step previous derivative error
	// motor pid gain setting
	float32_t pos_kp;
	float32_t pos_kd;
	float32_t cutoff_pos_pid;
	float32_t pos_ki;
	float32_t vel_kp;
	float32_t vel_ki;
	// motor control input
	float32_t control_input;
	float32_t control_input_old;
	// motor current limit
	float32_t upper_CL;
	float32_t lower_CL;
	// motor saturated term
	float32_t control_input_excess;
} Motor;

// robot control reference
float32_t homing_q_bi_buffer[NUM_MOTORS] = {
    0 * (pi/180),  		// q1: rad
	60 * (pi/180),  	// qm: rad
	- 60 * (pi/180),  	// qb: rad
};
arm_matrix_instance_f32 homing_q_bi;

float32_t homing_posXYZ_buffer[NUM_TASK_DEG] = {
    0.46,  	// x: m
	0,  	// y: m
	0.176   // z: m
};
arm_matrix_instance_f32 homing_posXYZ;

float32_t target_posXYZ_buffer[NUM_TASK_DEG] = {
    0.46,  	// x: m
	0,  	// y: m
	0.176   // z: m
};
arm_matrix_instance_f32 target_posXYZ;

// robot control gains
float32_t taskspace_p_gain[NUM_TASK_DEG]      = {180, 180, 180};
float32_t taskspace_i_gain[NUM_TASK_DEG]      = {30, 30, 30};
float32_t taskspace_d_gain[NUM_TASK_DEG]      = {3, 3, 3};
float32_t taskspace_windup_gain[NUM_TASK_DEG] = {0, 0, 0};
float32_t taskspace_pid_cutoff[NUM_TASK_DEG]  = {150, 150, 150};

// Manipulator Structure Definition
typedef struct {
	// manipulator safety state definition
	int current_robot_mode;		// robot enable: 1, disable: 0
	int desired_robot_mode;

	// Motor structure definition
	Motor motors[NUM_MOTORS];

	// manipulator joint state definition
	float32_t axis_configuration[NUM_MOTORS]; // 모터 축 방향 설정
	arm_matrix_instance_f32 q;
	float32_t q_buffer[NUM_MOTORS];
	float32_t q_upper_ROM[NUM_MOTORS];
	float32_t q_lower_ROM[NUM_MOTORS];
	arm_matrix_instance_f32 q_bi;
	float32_t q_bi_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 q_bi_old;
	float32_t q_bi_old_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 qdot_bi;
	float32_t qdot_bi_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 qdot_bi_old;
	float32_t qdot_bi_old_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 qddot_bi;
	float32_t qddot_bi_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 qddot_bi_old;
	float32_t qddot_bi_old_buffer[NUM_MOTORS];

	// manipulator taskspace state definition
	arm_matrix_instance_f32 posXYZ_ref;
	float32_t posXYZ_ref_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 posXYZ_ref_old;
	float32_t posXYZ_ref_old_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 posXYZ;
	float32_t posXYZ_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 posXYZ_old;
	float32_t posXYZ_old_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 velXYZ;
	float32_t velXYZ_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 velXYZ_old;
	float32_t velXYZ_old_buffer[NUM_TASK_DEG];

	// manipulator model params definition
	float32_t m1, m2, m3;
	float32_t l1, l2, l3;
	float32_t J1, J2, J3;
	float32_t d2, d3;
	arm_matrix_instance_f32 jacb_bi;
	float32_t jacb_bi_buffer[NUM_TASK_DEG * NUM_MOTORS];
	arm_matrix_instance_f32 jacb_bi_inv;
	float32_t jacb_bi_inv_buffer[NUM_TASK_DEG * NUM_MOTORS];
	arm_matrix_instance_f32 jacb_bi_trans;
	float32_t jacb_bi_trans_buffer[NUM_TASK_DEG * NUM_MOTORS];
	arm_matrix_instance_f32 jacb_bi_trans_inv;
	float32_t jacb_bi_trans_inv_buffer[NUM_TASK_DEG * NUM_MOTORS];
	arm_matrix_instance_f32 M_bi;
	float32_t M_bi_buffer[NUM_MOTORS * NUM_MOTORS];
	arm_matrix_instance_f32 C_bi;
	float32_t C_bi_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 G_bi;
	float32_t G_bi_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 M_bi_task;
	float32_t M_bi_task_buffer[NUM_TASK_DEG * NUM_TASK_DEG];
	arm_matrix_instance_f32 M_bi_task_nominal;
	float32_t M_bi_task_nominal_buffer[NUM_TASK_DEG * NUM_TASK_DEG];

	// manipulator pd gain setting
	float32_t pos_kp[NUM_TASK_DEG];
	float32_t pos_ki[NUM_TASK_DEG];
	float32_t pos_kd[NUM_TASK_DEG];
	float32_t pos_k_windup[NUM_TASK_DEG];
	float32_t cutoff_pos_pid[NUM_TASK_DEG];

	// manipulator task space pid control state definition
	arm_matrix_instance_f32 pos_error;
	float32_t pos_error_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_error_old;
	float32_t pos_error_old_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_P_term;
	float32_t pos_P_term_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_I_term;
	float32_t pos_I_term_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_I_term_old;
	float32_t pos_I_term_old_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_D_term;
	float32_t pos_D_term_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_D_term_old;
	float32_t pos_D_term_old_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_pid_output;
	float32_t pos_pid_output_buffer[NUM_TASK_DEG];

	// manipulator task space DOB control state definition
	float32_t cut_off_DOB[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_lhs;
	float32_t DOB_lhs_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_lhs_old;
	float32_t DOB_lhs_old_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_filtered_lhs;
	float32_t DOB_filtered_lhs_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_filtered_lhs_old;
	float32_t DOB_filtered_lhs_old_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_rhs;
	float32_t DOB_rhs_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_rhs_old;
	float32_t DOB_rhs_old_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_filtered_rhs;
	float32_t DOB_filtered_rhs_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 DOB_filtered_rhs_old;
	float32_t DOB_filtered_rhs_old_buffer[NUM_MOTORS];

	// manipulator control input
	arm_matrix_instance_f32 tau_bi;
	float32_t tau_bi_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 tau_bi_old;
	float32_t tau_bi_old_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 tau_bi_excess;
	float32_t tau_bi_excess_buffer[NUM_MOTORS];
	arm_matrix_instance_f32 pos_pid_output_excess;
	float32_t pos_pid_output_excess_buffer[NUM_TASK_DEG];
	arm_matrix_instance_f32 pos_pid_output_excess_old;
	float32_t pos_pid_output_excess_old_buffer[NUM_TASK_DEG];
} Manipulator;

Manipulator strawberry_robot;

// Setting for Debug
int sta = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
void ControlTask(void *argument);
void DataLoggingTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Console Display Functions ----------------------------------------------------
int _write(int file, char* p, int len){
	/*for(int i=0; i<len; i++){
		ITM_SendChar((*p++));
	}*/
	HAL_UART_Transmit(&huart3, (uint8_t*)p, len, HAL_MAX_DELAY);
	return len;
}

// Safety Button Functions ----------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (strawberry_robot.current_robot_mode == 0)
	{
		strawberry_robot.desired_robot_mode = 1;
	}
	else
	{
		strawberry_robot.desired_robot_mode = 0;
	}
}

void BSP_PB_Callback(Button_TypeDef Button)
{
    if (Button == BUTTON_USER)
    {
        HAL_GPIO_EXTI_Callback(GPIO_PIN_13);  // 내부 콜백 호출 강제 연결
    }
}

// CAN Communication Functions ----------------------------------------------------
int float32_t_to_uint(float32_t x, float32_t x_min, float32_t x_max, unsigned int bits)
{
	// Converts a float to an unsigned int, given range and number of bits
    float32_t span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;

    return (unsigned int)((x - x_min) * ((float32_t)((1ULL << bits) - 1) / span));
}

float32_t uint_to_float32_t(unsigned int x_int, float32_t x_min, float32_t x_max, unsigned int bits)
{
	// converts unsigned int to float, given range and number of bits
    float32_t span = x_max - x_min;
    float32_t offset = x_min;

    return ((float32_t)x_int) * span / ((float32_t)((1ULL << bits) - 1)) + offset;
}

void MIT_reset_origin(const uint16_t motor_id){
	uint8_t buffer[8];   // transmit buffer

	buffer[0]=0xff;
	buffer[1]=0xff;
	buffer[2]=0xff;
	buffer[3]=0xff;
	buffer[4]=0xff;
	buffer[5]=0xff;
	buffer[6]=0xff;
	buffer[7]=0xfe;

	TxHeader.Identifier = motor_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.MessageMarker = 0;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buffer) != HAL_OK) {
		sta = 1;
		Error_Handler();
	}
}

void MIT_enter_control_mode(const uint16_t motor_id){
	uint8_t buffer[8]; // motor control buffer

	buffer[0]=0xff;
	buffer[1]=0xff;
	buffer[2]=0xff;
	buffer[3]=0xff;
	buffer[4]=0xff;
	buffer[5]=0xff;
	buffer[6]=0xff;
	buffer[7]=0xfc;

	TxHeader.Identifier = motor_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.MessageMarker = 0;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buffer) != HAL_OK) {
		sta = 1;
		Error_Handler();
	}
}

void MIT_exit_control_mode(const uint16_t motor_id){
	uint8_t buffer[8]; // motor control buffer

	buffer[0]=0xff;
	buffer[1]=0xff;
	buffer[2]=0xff;
	buffer[3]=0xff;
	buffer[4]=0xff;
	buffer[5]=0xff;
	buffer[6]=0xff;
	buffer[7]=0xfd;

	TxHeader.Identifier = motor_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.MessageMarker = 0;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buffer) != HAL_OK) {
		sta = 1;
		Error_Handler();
	}
}

void MIT_TxData(uint8_t* buffer, int16_t number) {
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	buffer[4] = 0;
	buffer[5] = 0;
	buffer[6] =(number&0x0f00) >> 8;
	buffer[7] = number&0x00ff;
}

void MIT_Mode(const uint16_t motor_id, float current_ref){
	uint8_t buffer[8];
	const float32_t I_MIN = -25.0f;
	const float32_t I_MAX = 25.0f;
	current_ref = fmin(fmax(I_MIN, current_ref), I_MAX);
	int i_int = float32_t_to_uint(current_ref, I_MIN, I_MAX, 12);

	MIT_TxData(buffer, (int16_t)(i_int));

	TxHeader.Identifier = motor_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.MessageMarker = 0;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, buffer) != HAL_OK) {
		sta = 1;
		Error_Handler();
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        {
            sta = 2;
            Error_Handler();
        }

        uint8_t id = RxData[0];  // RxData[0]에 모터 ID가 있음

        // 해당 ID와 일치하는 motor 찾기
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            if (strawberry_robot.motors[i].id == id)
            {
                // 해당 motor의 수신 큐에 RxData[8] 통째로 넣기
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(strawberry_robot.motors[i].canRxQueue, RxData, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                break;
            }
        }

        HAL_GPIO_TogglePin(GPIOE, LED2_PIN);  // 수신 표시
    }
}

// Filter Functions ----------------------------------------------------
float32_t tustin_derivative(float32_t input, float32_t input_old, float32_t output_old, float32_t cutoff_freq)
{
    float32_t time_const = 1 / (2 * pi * cutoff_freq);
    float32_t output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

float32_t tustin_integrate(float32_t input, float32_t input_old, float32_t output_old)
{
  float32_t output = 0;

  output = (Ts * (input + input_old) + 2 * output_old) / 2;

  return output;
}

float32_t lowpassfilter(float32_t input, float32_t input_old, float32_t output_old, float32_t cutoff_freq)
{
    float32_t time_const = 1 / (2 * pi * cutoff_freq);
    float32_t output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

// Single Motor Controller Functions ----------------------------------------------------
void motor_encoder_read(Motor *m, float32_t cutoff)
{
	// sensor cutoff resetting
	m->sensor_cutoff = cutoff;

	// state update
	m->pos_old = m->pos;
	m->vel_old = m->vel;
	m->acc_old = m->acc;
	m->pos_error_old = m->pos_error;
	m->pos_I_term_old = m->pos_I_term;
	m->pos_D_term_old = m->pos_D_term;
	m->vel_error_old = m->vel_error;
	m->vel_I_term_old = m->vel_I_term;

	m->control_input_old = m->control_input;

	const float32_t P_MIN = -32768, P_MAX = 32768;
	uint8_t buf[8];
	if (xQueueReceive(m->canRxQueue, buf, pdMS_TO_TICKS(1)) == pdPASS)
	{
		// CAN 메시지가 이미 수신되었거나 1ms 이내 수신 성공 시
		unsigned int p_int = ((buf[1]<<8)|buf[2]);
		float32_t pulses = (float32_t) uint_to_float32_t(p_int, P_MIN, P_MAX, 16);
		//printf("motor pulses: %f", pulses);
		m->pos = (pulses * (2 * pi /m->encoder_pulses)) ; // load단 position (rad) 값 피드백
	}
	else
	{
		sta = 2;
		Error_Handler();
		// 수신 실패 시에도 이전 pos 값을 그대로 유지
		//m->pos = m->pos_old;
		//printf("Warning: can not read encoder position of ID %d", m->id);
	}
	// 어쨌든 vel, acc 업데이트는 수행
	m->vel = tustin_derivative(m->pos, m->pos_old, m->vel_old, m->sensor_cutoff); // rad/s 값 계산
	m->acc = tustin_derivative(m->vel, m->vel_old, m->acc_old, m->sensor_cutoff); // rad/s^2 값 계산
}

void motor_pos_pid_gain_setting(Motor *m, float32_t kp, float32_t kd, float32_t ki, float32_t cutoff)
{
	m->pos_kp = kp;
	m->pos_kd = kd;
	m->pos_ki = ki;
	m->cutoff_pos_pid = cutoff;
}

void motor_vel_pi_gain_setting(Motor *m, float32_t kp, float32_t ki)
{
	m->vel_kp = kp;
	m->vel_ki = ki;
}

void motor_pos_pid(Motor *m, float32_t pos_ref)
{
	float32_t tau = 1 / (2 * pi * m->cutoff_pos_pid);

	m->pos_ref = pos_ref;

	m->pos_error = m->pos_ref - m->pos;

	m->pos_P_term = m->pos_kp * m->pos_error;
	m->pos_I_term = m->pos_ki * Ts / 2.0 * (m->pos_error + m->pos_error_old) + m->pos_I_term_old;
	m->pos_D_term = 2.0 * m->pos_kd / (2.0 * tau + Ts) * (m->pos_error - m->pos_error_old) - (Ts - 2.0 * tau) / (2.0 * tau + Ts) * m->pos_D_term_old;

	m->control_input = (m->pos_P_term + m->pos_I_term + m->pos_D_term) /m->gear_ratio /m->Kt; // motor torque -> load torque -> current converting

	// 매 주기 anti-windup term 리셋 (추후 saturation이 발생하게 되면 값이 덧씌워짐)
	m->control_input_excess = 0.0f;
	if (m->control_input > m->upper_CL) {
		m->control_input_excess = (m->control_input - m->upper_CL) * m->Kt * m->gear_ratio;
		m->control_input = m->upper_CL; // upper bound saturation (rated current limit)
	}
	if (m->control_input < m->lower_CL) {
		m->control_input_excess = (m->control_input - m->lower_CL) * m->Kt * m->gear_ratio;
		m->control_input = m->lower_CL; // lower bound saturation (rated current limit)
	}
}

void motor_vel_pid(Motor *m, float32_t vel_ref)
{
	m->vel_ref = vel_ref;

	m->vel_error = m->vel_ref - m->vel;

	m->vel_P_term = m->vel_kp * m->vel_error;
	m->vel_I_term = m->vel_ki * Ts / 2.0 * (m->vel_error + m->vel_error_old) + m->vel_I_term_old;

	m->control_input = (m->vel_P_term + m->vel_I_term) /m->gear_ratio /m->Kt; // motor torque -> load torque -> current converting

	// 매 주기 anti-windup term 리셋 (추후 saturation이 발생하게 되면 값이 덧씌워짐)
	m->control_input_excess = 0.0f;
	if (m->control_input > m->upper_CL) {
		m->control_input_excess = (m->control_input - m->upper_CL) * m->Kt * m->gear_ratio;
		m->control_input = m->upper_CL; // upper bound saturation (rated current limit)
	}
	if (m->control_input < m->lower_CL) {
		m->control_input_excess = (m->control_input - m->lower_CL) * m->Kt * m->gear_ratio;
		m->control_input = m->lower_CL; // lower bound saturation (rated current limit)
	}
}

void motor_feedforward_torque(Motor *m, float32_t tor_ref)
{
	m->control_input = tor_ref /m->gear_ratio /m->Kt; // motor torque -> load torque -> current converting

	// 매 주기 anti-windup term 리셋 (추후 saturation이 발생하게 되면 값이 덧씌워짐)
	m->control_input_excess = 0.0f;
	if (m->control_input > m->upper_CL) {
		m->control_input_excess = (m->control_input - m->upper_CL) * m->Kt * m->gear_ratio;
		m->control_input = m->upper_CL; // upper bound saturation (rated current limit)
	}
	if (m->control_input < m->lower_CL) {
		m->control_input_excess = (m->control_input - m->lower_CL) * m->Kt * m->gear_ratio;
		m->control_input = m->lower_CL; // lower bound saturation (rated current limit)
	}
}

// 3-DoF Manipulator Task Space Controller Functions ----------------------------------------------------
void robot_forward_kinematics_cal(Manipulator *r)
{
	// 1. pre-term calculation
	const float32_t s_1 = sinf(r->q_bi.pData[0]);
	const float32_t c_1 = cosf(r->q_bi.pData[0]);
	const float32_t s_m = sinf(r->q_bi.pData[1]);
	const float32_t c_m = cosf(r->q_bi.pData[1]);
	const float32_t s_b = sinf(r->q_bi.pData[2]);
	const float32_t c_b = cosf(r->q_bi.pData[2]);
	// 2. task space state update
	for (int i = 0; i < NUM_TASK_DEG; ++i) {
		r->posXYZ_ref_old.pData[i] = r->posXYZ_ref.pData[i];
		r->posXYZ_old.pData[i] = r->posXYZ.pData[i];
		r->velXYZ_old.pData[i] = r->velXYZ.pData[i];
	}
	r->posXYZ.pData[0] = c_1 * (r->l2 * c_m + r->l3 * c_b);
	r->posXYZ.pData[1] = s_1 * (r->l2 * c_m + r->l3 * c_b);
	r->posXYZ.pData[2] = r->l1 + r->l2 * s_m + r->l3 * s_b;
	if (arm_mat_mult_f32(&r->jacb_bi, &r->qdot_bi, &r->velXYZ) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
}

void robot_model_param_cal(Manipulator *r)
{
	// 1. pre-term calculation
	const float32_t s_1 = sinf(r->q_bi.pData[0]);
	const float32_t c_1 = cosf(r->q_bi.pData[0]);
	const float32_t s_m = sinf(r->q_bi.pData[1]);
	const float32_t c_m = cosf(r->q_bi.pData[1]);
	const float32_t s_b = sinf(r->q_bi.pData[2]);
	const float32_t c_b = cosf(r->q_bi.pData[2]);
	const float32_t s_bm = sinf(r->q_bi.pData[2] - r->q_bi.pData[1]);
	const float32_t c_bm = cosf(r->q_bi.pData[2] - r->q_bi.pData[1]);

	// 2. model params update (Jacobian 채우기)
	r->jacb_bi.pData[0 * r->jacb_bi.numCols + 0] = -s_1 * (r->l2 * c_m + r->l3 * c_b);
	r->jacb_bi.pData[0 * r->jacb_bi.numCols + 1] = -r->l2 * c_1 * s_m;
	r->jacb_bi.pData[0 * r->jacb_bi.numCols + 2] = -r->l3 * c_1 * s_b;
	r->jacb_bi.pData[1 * r->jacb_bi.numCols + 0] = c_1 * (r->l2 * c_m + r->l3 * c_b);
	r->jacb_bi.pData[1 * r->jacb_bi.numCols + 1] = -r->l2 * s_1 * s_m;
	r->jacb_bi.pData[1 * r->jacb_bi.numCols + 2] = -r->l3 * s_1 * s_b;
	r->jacb_bi.pData[2 * r->jacb_bi.numCols + 0] = 0.0f;
	r->jacb_bi.pData[2 * r->jacb_bi.numCols + 1] = r->l2 * c_m;
	r->jacb_bi.pData[2 * r->jacb_bi.numCols + 2] = r->l3 * c_b;
	if (arm_mat_trans_f32(&r->jacb_bi, &r->jacb_bi_trans) != ARM_MATH_SUCCESS)  { sta=4; Error_Handler(); }

	// 3. model params update (Manipulator Dynamics model 채우기)
	float32_t l2_cm_d3_cb = r->l2 * c_m + r->d3 * c_b;
	r->M_bi.pData[0 * r->M_bi.numCols + 0] = r->J1
											+ r->m2 * (r->d2 * c_m) * (r->d2 * c_m)
											+ r->m3 *  l2_cm_d3_cb * l2_cm_d3_cb;
	r->M_bi.pData[0 * r->M_bi.numCols + 1] = - r->m2 * r->d2 * r->d2 * s_m * c_m
											- r->m3 * l2_cm_d3_cb * r->l2 * s_m;
	r->M_bi.pData[0 * r->M_bi.numCols + 2] = - r->m3 * l2_cm_d3_cb * r->d3 * s_b;
	r->M_bi.pData[1 * r->M_bi.numCols + 0] = r->M_bi.pData[0 * r->M_bi.numCols + 1];
	r->M_bi.pData[1 * r->M_bi.numCols + 1] = r->J2
											+ r->m3 * r->l2 * r->l2;
	r->M_bi.pData[1 * r->M_bi.numCols + 2] = r->m3 * r->l2 * r->d3 * c_bm;
	r->M_bi.pData[2 * r->M_bi.numCols + 0] = r->M_bi.pData[0 * r->M_bi.numCols + 2];
	r->M_bi.pData[2 * r->M_bi.numCols + 1] = r->M_bi.pData[1 * r->M_bi.numCols + 2];
	r->M_bi.pData[2 * r->M_bi.numCols + 2] = r->J3;
	r->C_bi.pData[0]= 2.0f * (- r->m2 * r->d2 * r->d2 * c_m * s_m * r->qdot_bi.pData[1]
								- r->m3 * (r->l2 * c_m + r->d3 * c_b) * ((r->l2 * s_m + r->d3 * s_b) * r->qdot_bi.pData[1] + r->d3 * s_b * (r->qdot_bi.pData[2] - r->qdot_bi.pData[1]))
							 ) * r->qdot_bi.pData[0];
	r->C_bi.pData[1] = - r->m3 * r->l2 * r->d3 * s_bm * (r->qdot_bi.pData[2] + r->qdot_bi.pData[1]) * (r->qdot_bi.pData[2] - r->qdot_bi.pData[1]);
	r->C_bi.pData[2] = r->m3 * r->l2 * r->d3 * s_bm * (r->qdot_bi.pData[1] * r->qdot_bi.pData[1]);
	r->G_bi.pData[0] = 0.0f;
	r->G_bi.pData[1] = g * (r->m2 * r->d2 + r->m3 * r->l2) * c_m;
	r->G_bi.pData[2] = g * r->m3 * r->d3 * c_b;

	// 5.Singular Point에 가까운지 여부에 따라 Jacobian Inverse와 Taskspace Mass Matrix 분리해서 계산
	// L = l2*cos(qm) + l3*cos(qb)
	float32_t L = r->l2 * c_m + r->l3 * c_b;
	// Δ = cos(qb) * sin(qm) - cos(qm) * sin(qb) = sin(qm - qb)
	float32_t Delta = c_b * s_m - c_m * s_b;
	// L, Delta 계산 직후 크기가 너무 작지 않은지 확인 (Singular Point에 가까운지 확인)
	float32_t epsL = fmaxf(1e-6f*(r->l2 + r->l3), FLT_EPSILON*(r->l2 + r->l3));
	float32_t epsD = fmaxf(1e-6f,               FLT_EPSILON);
	if (fabsf(L) < epsL || fabsf(Delta) < epsD) // Singular Point에 가까우면 0으로 나누게 되는 Fault 상황이 발생하기 때문에 해당 경우에는 DLS 사용
	{
	    // ----- DLS fallback: J^T (J J^T + λ^2 I)^{-1} -----
		// Jacobian의 Inverse 계산
	    float32_t JJt_buf[9], JJt_d_buf[9], invJJt_buf[9];
	    arm_matrix_instance_f32 JJt, JJt_d, invJJt;
	    arm_mat_init_f32(&JJt,   3,3, JJt_buf);
	    arm_mat_init_f32(&JJt_d, 3,3, JJt_d_buf);
	    arm_mat_init_f32(&invJJt,3,3, invJJt_buf);
	    if (arm_mat_mult_f32(&r->jacb_bi, &r->jacb_bi_trans, &JJt) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
	    float32_t tr = JJt_buf[0] + JJt_buf[4] + JJt_buf[8];
	    float32_t lambda = 0.05f * (tr/3.0f + 1e-6f);
	    float32_t lambda2 = lambda*lambda;
	    for (int i=0;i<9;i++) JJt_d_buf[i] = JJt_buf[i];
	    JJt_d_buf[0]+=lambda2; JJt_d_buf[4]+=lambda2; JJt_d_buf[8]+=lambda2;
	    if (arm_mat_inverse_f32(&JJt_d, &invJJt) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
	    if (arm_mat_mult_f32(&r->jacb_bi_trans, &invJJt, &r->jacb_bi_inv) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
	    // Jacobian의 Inverse의 Transpose 계산
		if (arm_mat_trans_f32(&r->jacb_bi_inv, &r->jacb_bi_trans_inv) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
		// Taskspace Mass Matrix 계산
		float32_t Minv_buf[9], A_buf[9], Ad_buf[9], Lambda_buf[9], tmp_buf[9];
		arm_matrix_instance_f32 Minv, A, Ad, Lambda, tmp;
		arm_mat_init_f32(&Minv,  3,3, Minv_buf);
		arm_mat_init_f32(&A,     3,3, A_buf);
		arm_mat_init_f32(&Ad,    3,3, Ad_buf);
		arm_mat_init_f32(&Lambda,3,3, Lambda_buf);
		arm_mat_init_f32(&tmp,   3,3, tmp_buf);
		if (arm_mat_inverse_f32(&r->M_bi, &Minv) == ARM_MATH_SUCCESS) { // M이 특이행렬이 아니면 계산하고, 특이행렬이면 안전하게 이전값 유지
		    // A = J * Minv * J^T
		    if (arm_mat_mult_f32(&r->jacb_bi, &Minv, &tmp) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
		    if (arm_mat_mult_f32(&tmp, &r->jacb_bi_trans, &A) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
		    // 댐핑(선택): A_d = A + μ^2 I
		    for (int i=0;i<9;i++) Ad_buf[i] = A_buf[i];
		    float32_t mu2 = 0.0f; // 필요 시 1e-4 ~ 1e-2 범위
		    Ad_buf[0]+=mu2; Ad_buf[4]+=mu2; Ad_buf[8]+=mu2;
		    if (arm_mat_inverse_f32(&Ad, &Lambda) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
		    // 최종 작업공간 관성 M_bi_task = Lambda
		    for (int i=0;i<9;i++) r->M_bi_task.pData[i] = Lambda_buf[i];
		}
		// DOB를 위한 Nominal Taskspace Mass Matrix 계산
		r->M_bi_task_nominal.pData[0 * r->M_bi_task_nominal.numCols + 0] = r->M_bi_task.pData[0 * r->M_bi_task.numCols + 0];
		r->M_bi_task_nominal.pData[0 * r->M_bi_task_nominal.numCols + 1] = 0;
		r->M_bi_task_nominal.pData[0 * r->M_bi_task_nominal.numCols + 2] = 0;
		r->M_bi_task_nominal.pData[1 * r->M_bi_task_nominal.numCols + 0] = 0;
		r->M_bi_task_nominal.pData[1 * r->M_bi_task_nominal.numCols + 1] = r->M_bi_task.pData[1 * r->M_bi_task.numCols + 1];
		r->M_bi_task_nominal.pData[1 * r->M_bi_task_nominal.numCols + 2] = 0;
		r->M_bi_task_nominal.pData[2 * r->M_bi_task_nominal.numCols + 0] = 0;
		r->M_bi_task_nominal.pData[2 * r->M_bi_task_nominal.numCols + 1] = 0;
		r->M_bi_task_nominal.pData[2 * r->M_bi_task_nominal.numCols + 2] = r->M_bi_task.pData[2 * r->M_bi_task.numCols + 2];
	}
	else // Singular Point에 가깝지 않으면 직접 Jacobian의 Inverse 계산
	{
		// Jacobian의 Inverse 계산
	    r->jacb_bi_inv.pData[0*r->jacb_bi_inv.numCols + 0] = -s_1 / L;
	    r->jacb_bi_inv.pData[0*r->jacb_bi_inv.numCols + 1] =  c_1 / L;
	    r->jacb_bi_inv.pData[0*r->jacb_bi_inv.numCols + 2] =  0.0f;
	    r->jacb_bi_inv.pData[1*r->jacb_bi_inv.numCols + 0] = -c_1 * c_b / (r->l2 * Delta);
	    r->jacb_bi_inv.pData[1*r->jacb_bi_inv.numCols + 1] = -c_b * s_1 / (r->l2 * Delta);
	    r->jacb_bi_inv.pData[1*r->jacb_bi_inv.numCols + 2] = -s_b / (r->l2 * Delta);
	    r->jacb_bi_inv.pData[2*r->jacb_bi_inv.numCols + 0] =  c_1 * c_m / (r->l3 * Delta);
	    r->jacb_bi_inv.pData[2*r->jacb_bi_inv.numCols + 1] =  c_m * s_1 / (r->l3 * Delta);
	    r->jacb_bi_inv.pData[2*r->jacb_bi_inv.numCols + 2] =  s_m / (r->l3 * Delta);
	    // Jacobian의 Inverse의 Transpose 계산
	    if (arm_mat_trans_f32(&r->jacb_bi_inv, &r->jacb_bi_trans_inv) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
	    // Taskspace Mass Matrix 계산
		float32_t MJI_buf[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
		arm_matrix_instance_f32 MJI;
		arm_mat_init_f32(&MJI,  3, 3, MJI_buf);
		if (arm_mat_mult_f32(&r->M_bi, &r->jacb_bi_inv, &MJI) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
		if (arm_mat_mult_f32(&r->jacb_bi_trans_inv, &MJI, &r->M_bi_task) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
		// DOB를 위한 Nominal Taskspace Mass Matrix 계산
		r->M_bi_task_nominal.pData[0 * r->M_bi_task_nominal.numCols + 0] = r->M_bi_task.pData[0 * r->M_bi_task.numCols + 0];
		r->M_bi_task_nominal.pData[0 * r->M_bi_task_nominal.numCols + 1] = 0;
		r->M_bi_task_nominal.pData[0 * r->M_bi_task_nominal.numCols + 2] = 0;
		r->M_bi_task_nominal.pData[1 * r->M_bi_task_nominal.numCols + 0] = 0;
		r->M_bi_task_nominal.pData[1 * r->M_bi_task_nominal.numCols + 1] = r->M_bi_task.pData[1 * r->M_bi_task.numCols + 1];
		r->M_bi_task_nominal.pData[1 * r->M_bi_task_nominal.numCols + 2] = 0;
		r->M_bi_task_nominal.pData[2 * r->M_bi_task_nominal.numCols + 0] = 0;
		r->M_bi_task_nominal.pData[2 * r->M_bi_task_nominal.numCols + 1] = 0;
		r->M_bi_task_nominal.pData[2 * r->M_bi_task_nominal.numCols + 2] = r->M_bi_task.pData[2 * r->M_bi_task.numCols + 2];
	}
}

void robot_state_update(Manipulator *r)
{
	// 1. joint state update
	for (int i = 0; i < NUM_MOTORS; ++i) {
		r->q_bi.pData		 [i] = r->axis_configuration[i] * r->motors[i].pos + homing_q_bi.pData[i];
		r->q_bi_old.pData	 [i] = r->axis_configuration[i] * r->motors[i].pos_old + homing_q_bi.pData[i];
		r->qdot_bi.pData	 [i] = r->axis_configuration[i] * r->motors[i].vel;
		r->qdot_bi_old.pData [i] = r->axis_configuration[i] * r->motors[i].vel_old;
		r->qddot_bi.pData	 [i] = r->axis_configuration[i] * r->motors[i].acc;
		r->qddot_bi_old.pData[i] = r->axis_configuration[i] * r->motors[i].acc_old;
		r->tau_bi_excess.pData[i] = r->axis_configuration[i] * r->motors[i].control_input_excess;
	}

	// 2. Range of Motion Checking
	r->q.pData[0] = r->q_bi.pData[0];
	r->q.pData[1] = r->q_bi.pData[1];
	r->q.pData[2] = r->q_bi.pData[2] - r->q_bi.pData[1];
	for (int i = 0; i < NUM_MOTORS; ++i) {
		if (r->q.pData[i] > r->q_upper_ROM[i]) { sta=5; Error_Handler(); }
		else if (r->q.pData[i] < r->q_lower_ROM[i]) { sta=5; Error_Handler(); }
	}

	// 2. model params update
	robot_model_param_cal(r);

	// 3. task space state update
	robot_forward_kinematics_cal(r);

	// 4. manipulator task space pid control state update
	for (int i = 0; i < NUM_TASK_DEG; ++i) {
		r->pos_error_old.pData[i] = r->pos_error.pData[i];
		r->pos_I_term_old.pData[i] = r->pos_I_term.pData[i];
		r->pos_D_term_old.pData[i] = r->pos_D_term.pData[i];
	}

	// 5. manipulator task space DOB control state update
	for (int i = 0; i < NUM_MOTORS; ++i) {
		r->DOB_lhs_old.pData[i] = r->DOB_lhs.pData[i];
		r->DOB_filtered_lhs_old.pData[i] = r->DOB_filtered_lhs.pData[i];
		r->DOB_rhs_old.pData[i] = r->DOB_rhs.pData[i];
		r->DOB_filtered_rhs_old.pData[i] = r->DOB_filtered_rhs.pData[i];
	}

	// 6. manipulator control input update
	for (int i = 0; i < NUM_MOTORS; ++i) {
		r->tau_bi_old.pData[i] = r->tau_bi.pData[i];
		r->pos_pid_output_excess_old.pData[i] = r->pos_pid_output_excess.pData[i];
	}

	// 7. anti-windup term update
	if (arm_mat_mult_f32(&r->jacb_bi_trans_inv, &r->tau_bi_excess, &r->pos_pid_output_excess) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
}

void robot_pos_pid_gain_setting(Manipulator *r, float32_t* kp, float32_t* kd, float32_t* ki, float32_t* k_windup, float32_t* cutoff)
{
	for (int i = 0; i < NUM_TASK_DEG; ++i) {
		r->pos_kp[i] = kp[i];
		r->pos_kd[i] = kd[i];
		r->pos_ki[i] = ki[i];
		r->pos_k_windup[i] = k_windup[i];
		r->cutoff_pos_pid[i] = cutoff[i];
	}
}

void robot_pos_pid(Manipulator *r, arm_matrix_instance_f32 pos_ref)
{
	for (int i = 0; i < NUM_TASK_DEG; ++i) {
		float32_t tau = 1 / (2 * pi * r->cutoff_pos_pid[i]);

		r->posXYZ_ref.pData[i] = pos_ref.pData[i];

		r->pos_error.pData[i] = r->posXYZ_ref.pData[i] - r->posXYZ.pData[i];

		r->pos_P_term.pData[i] = r->pos_kp[i] * r->pos_error.pData[i];
		r->pos_I_term.pData[i] = r->pos_ki[i] * Ts / 2.0 * (r->pos_error.pData[i] - r->pos_k_windup[i] * r->pos_pid_output_excess.pData[i] + r->pos_error_old.pData[i] - r->pos_k_windup[i] * r->pos_pid_output_excess_old.pData[i]) + r->pos_I_term_old.pData[i];
		r->pos_D_term.pData[i] = 2.0 * r->pos_kd[i] / (2.0 * tau + Ts) * (r->pos_error.pData[i] - r->pos_error_old.pData[i]) - (Ts - 2.0 * tau) / (2.0 * tau + Ts) * r->pos_D_term_old.pData[i];

		r->pos_pid_output.pData[i] = (r->pos_P_term.pData[i] + r->pos_I_term.pData[i] + r->pos_D_term.pData[i]);
	}

	if (arm_mat_mult_f32(&r->jacb_bi_trans, &r->pos_pid_output, &r->tau_bi) != ARM_MATH_SUCCESS) { sta=4; Error_Handler(); }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    // 링버퍼에 바이트 저장 (넘치면 가장 오래된 바이트를 버림)
    uint16_t next = (uart3_widx + 1) & (UART3_RBUF_SIZE - 1);
    if (next == uart3_ridx) {
      // 버퍼 풀 → reader를 한 칸 앞으로 밀어 가장 오래된 것 1바이트 drop
      uart3_ridx = (uart3_ridx + 1) & (UART3_RBUF_SIZE - 1);
    }
    uart3_rbuf[uart3_widx] = uart3_rx_byte;
    uart3_widx = next;

    // 다음 바이트 수신 재개
    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}

// 링버퍼에서 1바이트 pop (읽을 게 없으면 0 반환)
static int uart3_rb_pop(uint8_t *out)
{
  if (uart3_ridx == uart3_widx) return 0;  // empty
  *out = uart3_rbuf[uart3_ridx];
  uart3_ridx = (uart3_ridx + 1) & (UART3_RBUF_SIZE - 1);
  return 1;
}

// 좌우 공백 제거
static inline void trim_spaces(char *s) {
  char *p = s;
  while (*p==' ' || *p=='\t') ++p;
  if (p!=s) memmove(s, p, strlen(p)+1);
  for (int i=(int)strlen(s)-1; i>=0 && (s[i]==' ' || s[i]=='\t'); --i) s[i]='\0';
}

// 한 줄을 파싱: [ ... 19개 ... ] 에서 float들 추출
static int parse_pc_line_to_floats(char *line, float vals[], int maxn)
{
  // 대괄호 범위 찾기
  char *L = strchr(line, '[');
  char *R = strrchr(line, ']');
  if (!L || !R || R <= L) return 0;

  *R = '\0';   // ']' 대신 문자열 끝
  ++L;         // '[' 다음부터

  int count = 0;
  char *save = NULL;
  char *tok = strtok_r(L, ",", &save);
  while (tok && count < maxn) {
    trim_spaces(tok);
    vals[count++] = strtof(tok, NULL);
    tok = strtok_r(NULL, ",", &save);
  }
  return count;
}

// 파싱 결과를 시스템 파라미터에 반영 (요청대로 DataLoggingTask에서 직접 반영)
static void apply_pc_floats(const float v[PC_MSG_FIELDS])
{
	// 인덱스 매핑
	const int T   = 0;
	const int tx  = 1,  ty  = 2,  tz  = 3;
	const int xKp = 4,  xKi = 5,  xKd = 6,  xCf = 7,  xAw = 8;
	const int yKp = 9,  yKi =10,  yKd =11,  yCf =12,  yAw =13;
	const int zKp =14,  zKi =15,  zKd =16,  zCf =17,  zAw =18;

	// 간단한 유효성 (원하면 강화)
	if (v[xCf] <= 0 || v[yCf] <= 0 || v[zCf] <= 0) return;

	// 1) taskTime
	//  gTaskTime_s = v[T];

	// 2) 타겟 위치 (사용자 전역/객체)
	//  target_posXYZ.pData[0] = v[tx];
	//  target_posXYZ.pData[1] = v[ty];
	//  target_posXYZ.pData[2] = v[tz];

	// 3) XYZ 게인/컷오프/안티윈드업
	taskspace_p_gain[0]     	   = v[xKp];
	taskspace_i_gain[0]     	   = v[xKi];
	taskspace_d_gain[0]     	   = v[xKd];
	taskspace_pid_cutoff[0]        = v[xCf];
	taskspace_windup_gain[0]       = v[xAw];

	taskspace_p_gain[1]     	   = v[yKp];
	taskspace_i_gain[1]     	   = v[yKi];
	taskspace_d_gain[1]     	   = v[yKd];
	taskspace_pid_cutoff[1]        = v[yCf];
	taskspace_windup_gain[1]       = v[yAw];

	taskspace_p_gain[2]     	   = v[zKp];
	taskspace_i_gain[2]    	       = v[zKi];
	taskspace_d_gain[2]     	   = v[zKd];
	taskspace_pid_cutoff[2]        = v[zCf];
	taskspace_windup_gain[2]       = v[zAw];
}

// 링버퍼에서 줄 단위로 꺼내 처리 (CR 무시, LF로 완료)
static void uart3_poll_and_process_lines(void)
{
  uint8_t b;
  while (uart3_rb_pop(&b)) {
    char c = (char)b;
    if (c == '\r') continue;

    if (c == '\n') {
      if (uart3_line_len > 0) {
        uart3_line[uart3_line_len] = '\0';

        float vals[PC_MSG_FIELDS];
        int n = parse_pc_line_to_floats(uart3_line, vals, PC_MSG_FIELDS);
        if (n == PC_MSG_FIELDS) {
          apply_pc_floats(vals);
        } else {
          // 형식 불일치 시 무시(필요하면 printf로 경고)
          // printf("UART parse fail: got %d fields\r\n", n);
        }
        uart3_line_len = 0;
      }
    } else {
      if (uart3_line_len < UART3_LINE_MAX - 1) {
        uart3_line[uart3_line_len++] = c;
      } else {
        // 라인 과길이 → 드롭 & 리셋
        uart3_line_len = 0;
      }
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
	// 모터 객체 불변 파라미터 초기화
	for (int i = 0; i < NUM_MOTORS; ++i) {
		strawberry_robot.motors[i].id = i + 1;  // ID 1, 2, 3, ...
		strawberry_robot.motors[i].current_motor_mode = 0;
		strawberry_robot.motors[i].encoder_pulses = 16384;
		strawberry_robot.motors[i].gear_ratio = 10;
		strawberry_robot.motors[i].Kt = 0.123;
		strawberry_robot.motors[i].canRxQueue = xQueueCreate(8, sizeof(uint8_t[8]));  // 8바이트 버퍼
		if (strawberry_robot.motors[i].canRxQueue == NULL) {
			sta = 3;
			Error_Handler();
		}
		strawberry_robot.motors[i].upper_CL = 7.2;
		strawberry_robot.motors[i].lower_CL = -7.2;
	}
	// 로봇 객체 불변 파라미터 초기화
	arm_mat_init_f32(&homing_q_bi, NUM_MOTORS, 1, homing_q_bi_buffer);
	arm_mat_init_f32(&homing_posXYZ, NUM_TASK_DEG, 1, homing_posXYZ_buffer);
	arm_mat_init_f32(&target_posXYZ, NUM_TASK_DEG, 1, target_posXYZ_buffer);

	strawberry_robot.current_robot_mode = 0;
	strawberry_robot.desired_robot_mode = 0;

	strawberry_robot.axis_configuration[0] = -1;
	strawberry_robot.axis_configuration[1] = -1;
	strawberry_robot.axis_configuration[2] = 1;

	strawberry_robot.q_lower_ROM[0] = -pi;
	strawberry_robot.q_upper_ROM[0] = pi;
	strawberry_robot.q_lower_ROM[1] = 0;
	strawberry_robot.q_upper_ROM[1] = 85 * (pi/180);
	strawberry_robot.q_lower_ROM[2] = -160 * (pi/180);
	strawberry_robot.q_upper_ROM[2] = -40 * (pi/180);

	// link length setting
	strawberry_robot.l1 = 0.176;
	strawberry_robot.l2 = 0.46;
	strawberry_robot.l3 = 0.46;

	// link mass setting
	strawberry_robot.m1 = 3.93949;
	strawberry_robot.m2 = 0;
	strawberry_robot.m3 = 0;

	// link CoM position setting
	strawberry_robot.d2 = 0;
	strawberry_robot.d3 = 0;

	// link inertia setting
	strawberry_robot.J1 = 0;
	strawberry_robot.J2 = 0;
	strawberry_robot.J3 = 0;

	// 로봇 joint state matrix 연결
	arm_mat_init_f32(&strawberry_robot.q, NUM_MOTORS, 1, strawberry_robot.q_buffer);
	arm_mat_init_f32(&strawberry_robot.q_bi, NUM_MOTORS, 1, strawberry_robot.q_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.q_bi_old, NUM_MOTORS, 1, strawberry_robot.q_bi_old_buffer);
	arm_mat_init_f32(&strawberry_robot.qdot_bi, NUM_MOTORS, 1, strawberry_robot.qdot_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.qdot_bi_old, NUM_MOTORS, 1, strawberry_robot.qdot_bi_old_buffer);
	arm_mat_init_f32(&strawberry_robot.qddot_bi, NUM_MOTORS, 1, strawberry_robot.qddot_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.qddot_bi_old, NUM_MOTORS, 1, strawberry_robot.qddot_bi_old_buffer);
	// 로봇 task space state matrix 연결
	arm_mat_init_f32(&strawberry_robot.posXYZ_ref, NUM_TASK_DEG, 1, strawberry_robot.posXYZ_ref_buffer);
	arm_mat_init_f32(&strawberry_robot.posXYZ_ref_old, NUM_TASK_DEG, 1, strawberry_robot.posXYZ_ref_old_buffer);
	arm_mat_init_f32(&strawberry_robot.posXYZ, NUM_TASK_DEG, 1, strawberry_robot.posXYZ_buffer);
	arm_mat_init_f32(&strawberry_robot.posXYZ_old, NUM_TASK_DEG, 1, strawberry_robot.posXYZ_old_buffer);
	arm_mat_init_f32(&strawberry_robot.velXYZ, NUM_TASK_DEG, 1, strawberry_robot.velXYZ_buffer);
	arm_mat_init_f32(&strawberry_robot.velXYZ_old, NUM_TASK_DEG, 1, strawberry_robot.velXYZ_old_buffer);
	// 로봇 model params matrix 연결
	arm_mat_init_f32(&strawberry_robot.jacb_bi, NUM_TASK_DEG, NUM_MOTORS, strawberry_robot.jacb_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.jacb_bi_inv, NUM_MOTORS, NUM_TASK_DEG, strawberry_robot.jacb_bi_inv_buffer);
	arm_mat_init_f32(&strawberry_robot.jacb_bi_trans, NUM_TASK_DEG, NUM_MOTORS, strawberry_robot.jacb_bi_trans_buffer);
	arm_mat_init_f32(&strawberry_robot.jacb_bi_trans_inv, NUM_MOTORS, NUM_TASK_DEG, strawberry_robot.jacb_bi_trans_inv_buffer);
	arm_mat_init_f32(&strawberry_robot.M_bi, NUM_MOTORS, NUM_MOTORS, strawberry_robot.M_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.C_bi, NUM_MOTORS, 1, strawberry_robot.C_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.G_bi, NUM_MOTORS, 1, strawberry_robot.G_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.M_bi_task, NUM_TASK_DEG, NUM_TASK_DEG, strawberry_robot.M_bi_task_buffer);
	arm_mat_init_f32(&strawberry_robot.M_bi_task_nominal, NUM_TASK_DEG, NUM_TASK_DEG, strawberry_robot.M_bi_task_nominal_buffer);
	// 로봇 task space pid control state matrix 연결
	arm_mat_init_f32(&strawberry_robot.pos_error, NUM_TASK_DEG, 1, strawberry_robot.pos_error_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_error_old, NUM_TASK_DEG, 1, strawberry_robot.pos_error_old_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_P_term, NUM_TASK_DEG, 1, strawberry_robot.pos_P_term_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_I_term, NUM_TASK_DEG, 1, strawberry_robot.pos_I_term_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_I_term_old, NUM_TASK_DEG, 1, strawberry_robot.pos_I_term_old_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_D_term, NUM_TASK_DEG, 1, strawberry_robot.pos_D_term_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_D_term_old, NUM_TASK_DEG, 1, strawberry_robot.pos_D_term_old_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_pid_output, NUM_TASK_DEG, 1, strawberry_robot.pos_pid_output_buffer);

	// manipulator task space DOB control state definition
	arm_mat_init_f32(&strawberry_robot.DOB_lhs, NUM_MOTORS, 1, strawberry_robot.DOB_lhs_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_lhs_old, NUM_MOTORS, 1, strawberry_robot.DOB_lhs_old_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_filtered_lhs, NUM_MOTORS, 1, strawberry_robot.DOB_filtered_lhs_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_filtered_lhs_old, NUM_MOTORS, 1, strawberry_robot.DOB_filtered_lhs_old_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_rhs, NUM_MOTORS, 1, strawberry_robot.DOB_rhs_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_rhs_old, NUM_MOTORS, 1, strawberry_robot.DOB_rhs_old_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_filtered_rhs, NUM_MOTORS, 1, strawberry_robot.DOB_filtered_rhs_buffer);
	arm_mat_init_f32(&strawberry_robot.DOB_filtered_rhs_old, NUM_MOTORS, 1, strawberry_robot.DOB_filtered_rhs_old_buffer);

	// manipulator control input
	arm_mat_init_f32(&strawberry_robot.tau_bi, NUM_MOTORS, 1, strawberry_robot.tau_bi_buffer);
	arm_mat_init_f32(&strawberry_robot.tau_bi_old, NUM_MOTORS, 1, strawberry_robot.tau_bi_old_buffer);
	arm_mat_init_f32(&strawberry_robot.tau_bi_excess, NUM_MOTORS, 1, strawberry_robot.tau_bi_excess_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_pid_output_excess, NUM_TASK_DEG, 1, strawberry_robot.pos_pid_output_excess_buffer);
	arm_mat_init_f32(&strawberry_robot.pos_pid_output_excess_old, NUM_TASK_DEG, 1, strawberry_robot.pos_pid_output_excess_old_buffer);
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // UART3 1바이트 인터럽트 수신 시작
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Control */
  ControlHandle = osThreadNew(ControlTask, NULL, &Control_attributes);

  /* creation of DataLogging */
  DataLoggingHandle = osThreadNew(DataLoggingTask, NULL, &DataLogging_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 31;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 64;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 4;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 16;
  hfdcan1.Init.TxBuffersNbr = 16;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)!= HAL_OK)
  	{
	    sta = 3;
  		Error_Handler();
  	}

  	if ( HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  	{
  		sta = 3;
  		Error_Handler();
  	}
  	for (int i = 0; i < NUM_MOTORS; ++i)
	{
		// 모터를 Control Disable 모드로 전환
		if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
			MIT_exit_control_mode(strawberry_robot.motors[i].id);
			strawberry_robot.motors[i].current_motor_mode = 0;
			//printf("Motor %d: Stopped and exited control mode.\r\n", strawberry_robot.motors[i].id);
		}
	}
  /* USER CODE END FDCAN1_Init 2 */

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
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ControlTask */
/**
  * @brief  Function implementing the Control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	// 1) 현재 커널 틱 수를 읽어와 기준 시점으로 저장
	TickType_t ctrl_tick_reference = xTaskGetTickCount();
	// 2) 2 ms 를 틱 단위로 환산하여 주기 변수에 저장
	const TickType_t ctrl_tick_period = pdMS_TO_TICKS(2);  // 2 ms
  /* Infinite loop */
	for (;;)
	{
		// 3) 2 ms 주기로 블록 → 이 시점이 매 2 ms마다 실행됨
		vTaskDelayUntil(&ctrl_tick_reference, ctrl_tick_period);

		// 4) 실제 경과 시간(틱)으로부터 ms 환산하여 누적
		// portTICK_PERIOD_MS 는 1 틱이 ms 단위로 몇 ms인지 정의 (보통 1)
		ctrl_time_ms_old = ctrl_time_ms;
		ctrl_time_ms += (ctrl_tick_period * portTICK_PERIOD_MS);

		// 5) LED1 토글: 주기가 잘 유지되는지 육안으로 확인
		HAL_GPIO_TogglePin(GPIOB, LED1_PIN);

		// 6) 현재 로봇이 Enable 상태인지, Disable 상태인지 판단
		if (strawberry_robot.current_robot_mode == 1) // Robot이 Enable 상태일 때
		{
			if (strawberry_robot.desired_robot_mode == 0) // Robot의 Disable 명령이 들어오면
			{
				for (int i = 0; i < NUM_MOTORS; ++i)
				{
					// 0. 로봇의 상태 전환 LED로 표시
					HAL_GPIO_TogglePin(GPIOB, LED2_PIN);
					// 1. 제어 입력 초기화
					strawberry_robot.motors[i].control_input = 0.0;
					if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
						MIT_Mode(strawberry_robot.motors[i].id, strawberry_robot.motors[i].control_input);
					}
					// 2. 모터를 Control Disable 모드로 전환
					if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
						MIT_exit_control_mode(strawberry_robot.motors[i].id);
						strawberry_robot.motors[i].current_motor_mode = 0;
						//printf("Motor %d: Stopped and exited control mode.\r\n", strawberry_robot.motors[i].id);
					}
				}
				// 3. 로봇의 상태를 Control Disable 상태로 초기화
				strawberry_robot.current_robot_mode = 0;
			}
			else // Robot의 Disable 명령이 들어오지 않으면
			{
				// 0. 각 모터의 엔코더 값 센싱 및 모터 상태 업데이트
				for (int i = 0; i < NUM_MOTORS; ++i)
				{
					motor_encoder_read(&strawberry_robot.motors[i], 70.0);
				}
				// 1. 로봇의 상태 업데이트
				robot_state_update(&strawberry_robot);
				// 2. 로봇의 task space PID값 설정
				robot_pos_pid_gain_setting(&strawberry_robot, taskspace_p_gain, taskspace_d_gain, taskspace_i_gain, taskspace_windup_gain, taskspace_pid_cutoff);
				// 3. 로봇의 Control Input 계산
				target_posXYZ.pData[0] = homing_posXYZ.pData[0] + 0.2f * sinf(2.0f * pi * 5.0f * ((float32_t)ctrl_time_ms) / 1000.0f);
				robot_pos_pid(&strawberry_robot, target_posXYZ);
				for (int i = 0; i < NUM_MOTORS; ++i)
				{
					// 4. 로봇에서 계산한 Control Input을 모터 레벨로 내리기
					motor_feedforward_torque(&strawberry_robot.motors[i], strawberry_robot.tau_bi.pData[i] * strawberry_robot.axis_configuration[i]);
					// 5. CAN 통신 레지스터에 여유 슬롯이 있으면 현재 모터 제어값을 전송
					if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
						MIT_Mode(strawberry_robot.motors[i].id, strawberry_robot.motors[i].control_input);
					}
				}
			}
		}
		else // Robot이 Disable 상태일 때
		{
			if (strawberry_robot.desired_robot_mode == 1) // Robot의 Enable 명령이 들어오면
			{
				for (int i = 0; i < NUM_MOTORS; ++i)
				{
					// 0. 로봇의 상태 전환 LED로 표시
					HAL_GPIO_TogglePin(GPIOB, LED2_PIN);
					ctrl_time_ms = 0;
					ctrl_time_ms_old = 0;
					// 1. 현재 위치 원점으로 초기화
					if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
						MIT_reset_origin(strawberry_robot.motors[i].id);
					}
					// 2. CAN Rx 버퍼가 남아 있으면 모두 버림
					uint8_t dump[8];
					while (xQueueReceive(strawberry_robot.motors[i].canRxQueue, dump, 0) == pdPASS) {
						/* drop */
					}
					// 3. 모터를 Control Enable 모드로 전환
					if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
						MIT_enter_control_mode(strawberry_robot.motors[i].id);
						strawberry_robot.motors[i].current_motor_mode = 1;
						//printf("Motor %d: Initialized and started.\r\n", strawberry_robot.motors[i].id);
					}
					// 4. 모터 제어 입력 초기화
					strawberry_robot.motors[i].control_input = 0.0;
					strawberry_robot.motors[i].control_input_old = 0.0;
					strawberry_robot.motors[i].control_input_excess = 0.0;
					// 5. 모터 엔코더 값 초기화
					strawberry_robot.motors[i].pos = 0.0;
					strawberry_robot.motors[i].pos_old = strawberry_robot.motors[i].pos;
					strawberry_robot.motors[i].vel = 0.0;
					strawberry_robot.motors[i].vel_old = strawberry_robot.motors[i].vel;
					// 6. 모터 Desired 값 초기 설정
					strawberry_robot.motors[i].pos_ref = target_pos[i];
					strawberry_robot.motors[i].vel_ref = target_vel[i];
					// 7. 모터 Position 제어 관련 오차 초기화
					strawberry_robot.motors[i].pos_error = strawberry_robot.motors[i].pos_ref - strawberry_robot.motors[i].pos;
					strawberry_robot.motors[i].pos_error_old = strawberry_robot.motors[i].pos_error;
					strawberry_robot.motors[i].pos_P_term = 0.0;
					strawberry_robot.motors[i].pos_I_term = 0.0;
					strawberry_robot.motors[i].pos_I_term_old = strawberry_robot.motors[i].pos_I_term;
					strawberry_robot.motors[i].pos_D_term = 0.0;
					strawberry_robot.motors[i].pos_D_term_old = strawberry_robot.motors[i].pos_D_term;
					// 8. 모터 Velocity 제어 관련 오차 초기화
					strawberry_robot.motors[i].vel_error = strawberry_robot.motors[i].vel_ref - strawberry_robot.motors[i].vel;
					strawberry_robot.motors[i].vel_error_old = strawberry_robot.motors[i].vel_error;
					strawberry_robot.motors[i].vel_P_term = 0.0;
					strawberry_robot.motors[i].vel_I_term = 0.0;
					strawberry_robot.motors[i].vel_I_term_old = strawberry_robot.motors[i].vel_I_term;
					// 9. 로봇 상태 값 초기화
					strawberry_robot.qdot_bi.pData[i] = 0.0;
					strawberry_robot.qddot_bi.pData[i] = 0.0;
					// 10. manipulator task space DOB control state 초기화
					strawberry_robot.DOB_lhs.pData[i] = 0.0;
					strawberry_robot.DOB_filtered_lhs.pData[i] = 0.0;
					strawberry_robot.DOB_rhs.pData[i] = 0.0;
					strawberry_robot.DOB_filtered_rhs.pData[i] = 0.0;
					// 11. manipulator control input 초기화
					strawberry_robot.tau_bi.pData[i] = 0.0;
					strawberry_robot.tau_bi_excess.pData[i] = 0.0;
				}
				for (int i = 0; i < NUM_TASK_DEG; ++i)
				{
					// 12. manipulator taskspace state 초기화
					strawberry_robot.posXYZ_ref.pData[i] = target_posXYZ.pData[i];
					strawberry_robot.posXYZ.pData[i] = 0.0;
					strawberry_robot.velXYZ.pData[i] = 0.0;

					// 13. manipulator task space pid control state 초기화
					strawberry_robot.pos_error.pData[i] = 0.0;
					strawberry_robot.pos_P_term.pData[i] = 0.0;
					strawberry_robot.pos_I_term.pData[i] = 0.0;
					strawberry_robot.pos_D_term.pData[i] = 0.0;
					strawberry_robot.pos_pid_output.pData[i] = 0.0;
					strawberry_robot.pos_pid_output_excess.pData[i] = 0.0;
					strawberry_robot.pos_pid_output_excess_old.pData[i] = 0.0;
				}
				for (int i=0;i<9;i++) strawberry_robot.M_bi_task.pData[i] = 0.0f;
				strawberry_robot.M_bi_task.pData[0]=1.0f;
				strawberry_robot.M_bi_task.pData[4]=1.0f;
				strawberry_robot.M_bi_task.pData[8]=1.0f;
				for (int i=0;i<9;i++) strawberry_robot.M_bi_task_nominal.pData[i] = 0.0f;
				strawberry_robot.M_bi_task_nominal.pData[0]=1.0f;
				strawberry_robot.M_bi_task_nominal.pData[4]=1.0f;
				strawberry_robot.M_bi_task_nominal.pData[8]=1.0f;
				// 14. 로봇의 남은 과거 상태 파라미터 초기화
				robot_state_update(&strawberry_robot);
				// 15. 로봇의 상태를 Control Enable 상태로 초기화
				strawberry_robot.current_robot_mode = 1;
			}
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DataLoggingTask */
/**
* @brief Function implementing the DataLogging thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataLoggingTask */
void DataLoggingTask(void *argument)
{
  /* USER CODE BEGIN DataLoggingTask */
	// 1) 현재 커널 틱 수를 읽어와 기준 시점으로 저장
	TickType_t logging_tick_reference = xTaskGetTickCount();

	// 2) 2 ms 를 틱 단위로 환산하여 주기 변수에 저장
	const TickType_t logging_tick_period = pdMS_TO_TICKS(2);  // 2 ms

	  /* Infinite loop */
		for (;;)
		{
			// 3) 2 ms 주기로 블록 → 이 시점이 매 2 ms마다 실행됨
			vTaskDelayUntil(&logging_tick_reference, logging_tick_period);

			// 4) 실제 경과 시간(틱)으로부터 ms 환산하여 누적
			// portTICK_PERIOD_MS 는 1 틱이 ms 단위로 몇 ms인지 정의 (보통 1)
			logging_time_ms += (logging_tick_period * portTICK_PERIOD_MS);

			// 5) 여기서 PC로부터 들어온 명령을 처리
			uart3_poll_and_process_lines();

			// 6) 현재 로봇의 상태를 Serial 통신을 통해 PC로 전송
			if (strawberry_robot.current_robot_mode == 1) // 로봇의 현재 상태가 Control Enable인 경우
			{
				printf("[%8.3f, %8.3f, %8d, %8d, %8d, %8.3f, %8d, %8d, %8.3f, %8d, %8d, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f]\r\n",
						(float32_t) ctrl_time_ms/1000, (float32_t) (ctrl_time_ms - ctrl_time_ms_old)/1000, strawberry_robot.current_robot_mode,
						strawberry_robot.motors[0].id, strawberry_robot.motors[0].current_motor_mode, strawberry_robot.motors[0].control_input,
						strawberry_robot.motors[1].id, strawberry_robot.motors[1].current_motor_mode, strawberry_robot.motors[1].control_input,
						strawberry_robot.motors[2].id, strawberry_robot.motors[2].current_motor_mode, strawberry_robot.motors[2].control_input,
						strawberry_robot.q_bi.pData[0], strawberry_robot.q_bi.pData[1], strawberry_robot.q_bi.pData[2],
						strawberry_robot.posXYZ_ref.pData[0], strawberry_robot.posXYZ_ref.pData[1], strawberry_robot.posXYZ_ref.pData[2],
						strawberry_robot.posXYZ.pData[0], strawberry_robot.posXYZ.pData[1], strawberry_robot.posXYZ.pData[2],
						strawberry_robot.velXYZ.pData[0], strawberry_robot.velXYZ.pData[1], strawberry_robot.velXYZ.pData[2],
						strawberry_robot.pos_I_term.pData[0], strawberry_robot.pos_I_term.pData[1], strawberry_robot.pos_I_term.pData[2],
						strawberry_robot.pos_pid_output.pData[0], strawberry_robot.pos_pid_output.pData[1], strawberry_robot.pos_pid_output.pData[2]);
			}
			else // 로봇의 현재 상태가 Control Disable인 경우
			{
				printf("[%8.3f, %8.3f, %8d, %8d, %8d, %8.3f, %8d, %8d, %8.3f, %8d, %8d, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f]\r\n",
				(float32_t) ctrl_time_ms/1000, (float32_t) (ctrl_time_ms - ctrl_time_ms_old)/1000, strawberry_robot.current_robot_mode,
				strawberry_robot.motors[0].id, strawberry_robot.motors[0].current_motor_mode, 0.0f,
				strawberry_robot.motors[1].id, strawberry_robot.motors[1].current_motor_mode, 0.0f,
				strawberry_robot.motors[2].id, strawberry_robot.motors[2].current_motor_mode, 0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f);
			}
		}
  /* USER CODE END DataLoggingTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	if (strawberry_robot.current_robot_mode == 1)
  	{
  		for (int i = 0; i < NUM_MOTORS; ++i)
  		{
  			HAL_GPIO_TogglePin(GPIOB, LED2_PIN);
  			// 제어 입력 초기화
  			strawberry_robot.motors[i].control_input = 0.0;
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
				MIT_Mode (strawberry_robot.motors[i].id, strawberry_robot.motors[i].control_input);
			}
			if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
				MIT_exit_control_mode(strawberry_robot.motors[i].id);
			}
			//printf("Motor %d: Stopped and exited control mode.\r\n", strawberry_robot.motors[i].id);
  		}
		strawberry_robot.current_robot_mode = 0;
  	}
	switch(sta) {
	  case 1: printf("Error Code: %d, This is CAN Tx Error.\r\n", sta); break;
	  case 2: printf("Error Code: %d, This is CAN Rx Error.\r\n", sta); break;
	  case 3: printf("Error Code: %d, This is MCU Initialization Error.\r\n", sta); break;
	  case 4: printf("Error Code: %d, This is Matrix Calculation Error.\r\n", sta); break;
	  case 5: printf("Error Code: %d, This is Range of Motion Error.\r\n", sta); break;
	  default: printf("Error Code: Unknown");
	}
  while (1)
  {
	  for (int i = 0; i < sta; i++) {
	      HAL_GPIO_TogglePin(GPIOB, LED3_PIN);  // LED 반전
	      HAL_Delay(200);                       // 200ms 간격 (필요시 조절)
	      HAL_GPIO_TogglePin(GPIOB, LED3_PIN);  // LED 원래 상태 복귀
	      HAL_Delay(200);
	  }
	  HAL_Delay(1000);
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
