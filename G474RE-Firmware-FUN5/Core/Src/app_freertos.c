/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"

#include <math.h>
#include <string.h>
#include <mpu6050.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>
#include <robotic_interfaces/srv/keyboard.h>
#include <imu_interfaces/srv/imu_calibration.h>

#include <micro_ros_utilities/string_utilities.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef struct {
	double x;
	double y;
	double z;
} offset3d_t;

typedef struct {
	double roll;
	double pitch;
	double yaw;
} rotation_t;

typedef struct {
	double Q_angle;
	double Q_bias;
	double R_measure;

	double angle;
	double bias;
	double rate;

	double P[2][2];
} KalmanFilter_t;

typedef struct {
	GPIO_PinState A;
	GPIO_PinState B;
	GPIO_PinState C;
	GPIO_PinState D;
	GPIO_PinState K;
} controller_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GRAVITY 9.80665
#define DEG_TO_RAD M_PI / 180.0
#define RAD_TO_DEG 180.0 / M_PI

#define ADC_MAX 4095.0f
#define ADC_MIN 0.0f
#define OUTPUT_MAX 1.0f
#define OUTPUT_MIN -1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_node_t node;

rcl_publisher_t mpu6050_publisher;
sensor_msgs__msg__Imu mpu6050_msg;

rcl_publisher_t cmd_vel_publisher;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t cmd_vell_publisher;
geometry_msgs__msg__Twist cmd_vell_msg;

rcl_service_t imu_calibration_server;
imu_interfaces__srv__ImuCalibration_Request imu_calibration_request;
imu_interfaces__srv__ImuCalibration_Response imu_calibration_response;

rcl_service_t imu_status_server;
std_srvs__srv__SetBool_Request imu_status_request;
std_srvs__srv__SetBool_Response imu_status_response;

rcl_client_t robot_controller_client;
robotic_interfaces__srv__Keyboard_Request keyboard_request;
robotic_interfaces__srv__Keyboard_Response keyboard_response;

rcl_client_t robot_controller_Ref_client;
robotic_interfaces__srv__Keyboard_Request ref_request;

rcl_client_t robot_controller_saved_client;
std_srvs__srv__Trigger_Request save_request;

MPU6050_t MPU6050;
uint16_t cc = 0;
uint16_t cs = 0;
uint16_t ct = 0;
uint16_t cq = 0;

offset3d_t accl_offset;
offset3d_t gyro_offset;

rotation_t rotation_gyro;
rotation_t rotation_accl;
rotation_t rotation_kalm;
rotation_t rotation_real;

KalmanFilter_t kalmanX, kalmanY;

bool is_calib = false;
bool on_calib = false;

uint16_t ADCBuffer[80]={0};
int ADC_Average[2] = {0};
int ADC_SumAPot[2] = {0};

controller_t joy;

int main_Mode = 0;
uint8_t state_keep_B = 0;
uint8_t B_count = 0;
uint8_t state_keep_D = 0;
uint8_t D_count = 0;
uint8_t wait = 0;
uint8_t auto_on = 0;
uint8_t state_keep_K = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void calculate_gyro_angles(double Ax, double Ay, double Az, double Gx, double Gy, double Gz, float DT);
void calculate_accl_angles(double Ax, double Ay, double Az, double Gx, double Gy, double Gz, float DT);
void calculate_kalm_angles(double Ax, double Ay, double Az, double Gx, double Gy, double Gz, float DT);

void Kalman_Init(KalmanFilter_t* kf);
double Kalman_Angle(KalmanFilter_t* kf, double new_angle, double new_rate, float DT);

void ADC_Averaged();
void Read_Buttons();
float map_adc_to_output(int adc_value);

void calculate_gyro_angles(double Ax, double Ay, double Az, double Gx, double Gy, double Gz, float DT) {

	rotation_gyro.roll += (Gx * RAD_TO_DEG) * DT;
	rotation_gyro.pitch += (Gy * RAD_TO_DEG) * DT;
	rotation_gyro.yaw += (Gz * RAD_TO_DEG) * DT;

    if (rotation_gyro.yaw > 180.0){
    	rotation_gyro.yaw -= 360.0;
    }
    else if (rotation_gyro.yaw < -180.0){
    	rotation_gyro.yaw += 360.0;
    }
}

void calculate_accl_angles(double Ax, double Ay, double Az, double Gx, double Gy, double Gz, float DT) {

    double roll_acc = atan2(Ay, Az) * RAD_TO_DEG;
    double pitch_acc = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * RAD_TO_DEG;
    double yaw_acc = (Gz * RAD_TO_DEG) * DT;

    rotation_accl.roll = roll_acc;
    rotation_accl.pitch = pitch_acc;
    rotation_accl.yaw += yaw_acc;

    if (rotation_accl.yaw > 180.0){
    	rotation_accl.yaw -= 360.0;
    }
    else if (rotation_accl.yaw < -180.0){
    	rotation_accl.yaw += 360.0;
    }
}

void calculate_kalm_angles(double Ax, double Ay, double Az, double Gx, double Gy, double Gz, float DT) {

	double angleX = atan(Ay / sqrt(Ax * Ax + Az * Az)) * RAD_TO_DEG;
	double angleY = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * RAD_TO_DEG;

	double ratedX = Gx;
	double ratedY = Gy;

	rotation_kalm.roll = Kalman_Angle(&kalmanX, angleX, ratedX, DT);
	rotation_kalm.pitch = Kalman_Angle(&kalmanY, angleY, ratedY, DT);
	rotation_kalm.yaw += (Gz * RAD_TO_DEG) * DT;

    if (rotation_kalm.yaw > 180.0){
    	rotation_kalm.yaw -= 360.0;
    }
    else if (rotation_kalm.yaw < -180.0){
    	rotation_kalm.yaw += 360.0;
    }
}

void Kalman_Init(KalmanFilter_t* kf) {
    kf->Q_angle = 0.001f;
    kf->Q_bias  = 0.003f;
    kf->R_measure = 0.03f;

    kf->angle = 0.0f;
    kf->bias = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

double Kalman_Angle(KalmanFilter_t* kf, double new_angle, double new_rate, float DT){

    kf->rate = new_rate - kf->bias;
    kf->angle += DT * kf->rate;

    kf->P[0][0] += DT * (DT * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= DT * kf->P[1][1];
    kf->P[1][0] -= DT * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * DT;

    double S = kf->P[0][0] + kf->R_measure;
    double K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    double y = new_angle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    double P00_temp = kf->P[0][0];
    double P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

float map_adc_to_output(int adc_value) {
	float mapped = ((adc_value - ADC_MIN) * (OUTPUT_MAX - OUTPUT_MIN) / (ADC_MAX - ADC_MIN)) + OUTPUT_MIN;
	float output = (fabs(mapped) < 0.03) ? 0.0 : mapped;
	return output;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		if (main_Mode == 1)
		{
			uint32_t i2cError = HAL_I2C_GetError(&hi2c1);
			if (i2cError == HAL_I2C_ERROR_NONE){
				if (is_calib || on_calib){
					MPU6050_Read_All(&hi2c1, &MPU6050);

					double Ax = (GRAVITY * MPU6050.Ax) - accl_offset.x;
					double Ay = (GRAVITY * MPU6050.Ay) - accl_offset.y;
					double Az = (GRAVITY * MPU6050.Az) - accl_offset.z;

					double Gx = (DEG_TO_RAD * MPU6050.Gx) - gyro_offset.x;
					double Gy = (DEG_TO_RAD * MPU6050.Gy) - gyro_offset.y;
					double Gz = (DEG_TO_RAD * MPU6050.Gz) - gyro_offset.z;

					mpu6050_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
					mpu6050_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();

					mpu6050_msg.linear_acceleration.x = Ax;
					mpu6050_msg.linear_acceleration.y = Ay;
					mpu6050_msg.linear_acceleration.z = Az;

					mpu6050_msg.angular_velocity.x = Gx;
					mpu6050_msg.angular_velocity.y = Gy;
					mpu6050_msg.angular_velocity.z = Gz;

					rcl_ret_t ret = rcl_publish(&mpu6050_publisher, &mpu6050_msg, NULL);
					if (ret != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);

					rotation_real.roll = MPU6050.KalmanAngleX;
					rotation_real.pitch = MPU6050.KalmanAngleY;

					calculate_gyro_angles(Ax/GRAVITY, Ay/GRAVITY, Az/GRAVITY, Gx, Gy, Gz, 0.01);
					calculate_accl_angles(Ax/GRAVITY, Ay/GRAVITY, Az/GRAVITY, Gx, Gy, Gz, 0.01);
					calculate_kalm_angles(Ax/GRAVITY, Ay/GRAVITY, Az/GRAVITY, Gx, Gy, Gz, 0.01);

					cmd_vel_msg.linear.x = rotation_kalm.roll * DEG_TO_RAD;
					cmd_vel_msg.angular.z = -(rotation_kalm.pitch * DEG_TO_RAD);

					rcl_ret_t rett = rcl_publish(&cmd_vel_publisher, &cmd_vel_msg, NULL);
					if (rett != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);

				}
			}
			else
			{
				static uint32_t timestamp = 0;
				if (timestamp <= HAL_GetTick()){
					timestamp = HAL_GetTick() + 1000;
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					HAL_I2C_DeInit(&hi2c1);
					HAL_I2C_Init(&hi2c1);
					MPU6050_Init(&hi2c1);
				}
			}
		}

		else if (main_Mode == 0)
		{
			if (joy.B == GPIO_PIN_RESET && state_keep_B == 0)
			{
				wait++;
				state_keep_B = 1;
				state_keep_K = 0;
			}

			if (joy.B == GPIO_PIN_SET && state_keep_B == 1 && B_count == 0 && wait > 0)
			{
				keyboard_request.mode.data = "TOB";

				int64_t sq_num;
				rcl_ret_t ret = rcl_send_request(&robot_controller_client, &keyboard_request, &sq_num);

				if (ret != RCL_RET_OK) cq++;

				B_count = 1;
				D_count = 0;
				state_keep_B = 0;
				state_keep_K = 0;
			}

			else if (joy.B == GPIO_PIN_SET && state_keep_B == 1 && B_count == 1 && wait >= 2)
			{
				keyboard_request.mode.data = "AUT";

				int64_t sq_num;
				rcl_ret_t ret = rcl_send_request(&robot_controller_client, &keyboard_request, &sq_num);

				if (ret != RCL_RET_OK) cq++;

				B_count = 0;
				state_keep_B = 0;
				state_keep_K = 0;
			}

			if (joy.D == GPIO_PIN_RESET)
			{
				wait++;
				state_keep_D  = 1;
				state_keep_K = 0;
			}

			if (joy.D == GPIO_PIN_SET && state_keep_D == 1 && B_count == 1 && D_count == 0 && wait > 2)
			{
				keyboard_request.mode.data = "TOE";

				int64_t sq_numI;
				rcl_ret_t ret = rcl_send_request(&robot_controller_client, &keyboard_request, &sq_numI);
				if (ret != RCL_RET_OK) cq++;

				int64_t sq_numII;
				rcl_ret_t rett = rcl_send_request(&robot_controller_Ref_client, &keyboard_request, &sq_numII);
				if (rett != RCL_RET_OK) cq++;

				D_count = 1;
				state_keep_D = 0;
				state_keep_K = 0;
			}

			else if (joy.D == GPIO_PIN_SET && state_keep_D == 1 && B_count == 1 && D_count == 1 && wait > 2)
			{
				keyboard_request.mode.data = "TOB";

				int64_t sq_numI;
				rcl_ret_t ret = rcl_send_request(&robot_controller_client, &keyboard_request, &sq_numI);
				if (ret != RCL_RET_OK) cq++;

				int64_t sq_numII;
				rcl_ret_t rett = rcl_send_request(&robot_controller_Ref_client, &keyboard_request, &sq_numII);
				if (rett != RCL_RET_OK) cq++;

				D_count = 0;
				state_keep_D = 0;
				state_keep_K = 0;
			}

			if (joy.K == GPIO_PIN_RESET)
			{
				wait++;
				state_keep_K  = 1;
			}

			if (joy.K == GPIO_PIN_SET && state_keep_K == 1 && wait > 2)
			{
				int64_t sq_numI;
				rcl_ret_t ret = rcl_send_request(&robot_controller_saved_client, &save_request, &sq_numI);
				if (ret != RCL_RET_OK) cq++;

				state_keep_K = 0;
			}

			ADC_Averaged();
			Read_Buttons();

			float zp = (joy.A == GPIO_PIN_SET) ? 0 : 0.2;
			float zm = (joy.C == GPIO_PIN_SET) ? 0 : -0.2;

			cmd_vel_msg.linear.x = (map_adc_to_output(ADC_Average[1])) * 0.2;
			cmd_vel_msg.linear.y = (map_adc_to_output(ADC_Average[0])) * 0.2;
			cmd_vel_msg.linear.z = zp + zm;

			rcl_ret_t rett = rcl_publish(&cmd_vel_publisher, &cmd_vel_msg, NULL);
			if (rett != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
		}

		else if (main_Mode == 2)
		{
			uint32_t i2cError = HAL_I2C_GetError(&hi2c1);
			if (i2cError == HAL_I2C_ERROR_NONE){
				if (is_calib || on_calib){
					MPU6050_Read_All(&hi2c1, &MPU6050);

					double Ax = (GRAVITY * MPU6050.Ax) - accl_offset.x;
					double Ay = (GRAVITY * MPU6050.Ay) - accl_offset.y;
					double Az = (GRAVITY * MPU6050.Az) - accl_offset.z;

					double Gx = (DEG_TO_RAD * MPU6050.Gx) - gyro_offset.x;
					double Gy = (DEG_TO_RAD * MPU6050.Gy) - gyro_offset.y;
					double Gz = (DEG_TO_RAD * MPU6050.Gz) - gyro_offset.z;

					mpu6050_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
					mpu6050_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();

					mpu6050_msg.linear_acceleration.x = Ax;
					mpu6050_msg.linear_acceleration.y = Ay;
					mpu6050_msg.linear_acceleration.z = Az;

					mpu6050_msg.angular_velocity.x = Gx;
					mpu6050_msg.angular_velocity.y = Gy;
					mpu6050_msg.angular_velocity.z = Gz;

					rotation_real.roll = MPU6050.KalmanAngleX;
					rotation_real.pitch = MPU6050.KalmanAngleY;

					calculate_gyro_angles(Ax/GRAVITY, Ay/GRAVITY, Az/GRAVITY, Gx, Gy, Gz, 0.01);
					calculate_accl_angles(Ax/GRAVITY, Ay/GRAVITY, Az/GRAVITY, Gx, Gy, Gz, 0.01);
					calculate_kalm_angles(Ax/GRAVITY, Ay/GRAVITY, Az/GRAVITY, Gx, Gy, Gz, 0.01);

					cmd_vel_msg.linear.x = rotation_kalm.roll * DEG_TO_RAD;
					cmd_vel_msg.angular.z = -(rotation_kalm.pitch * DEG_TO_RAD);

				}
			}
			else
			{
				static uint32_t timestamp = 0;
				if (timestamp <= HAL_GetTick()){
					timestamp = HAL_GetTick() + 1000;
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					HAL_I2C_DeInit(&hi2c1);
					HAL_I2C_Init(&hi2c1);
					MPU6050_Init(&hi2c1);
				}
			}

			ADC_Averaged();

			cmd_vell_msg.linear.x = (map_adc_to_output(ADC_Average[1])) * 0.2;
			cmd_vell_msg.linear.y = (map_adc_to_output(ADC_Average[0])) * 0.2;

//			rcl_ret_t ret = rcl_publish(&cmd_vel_publisher, &cmd_vel_msg, NULL);
//			if (ret != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
//			rcl_ret_t rett = rcl_publish(&cmd_vel_II_publisher, &cmd_vel_II_msg, NULL);
//			if (rett != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
		}

	    HAL_IWDG_Refresh(&hiwdg);
	    cc++;
	}
}

void imu_calib_service_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  imu_interfaces__srv__ImuCalibration_Request * req_in =
    (imu_interfaces__srv__ImuCalibration_Request *) request_msg;
  imu_interfaces__srv__ImuCalibration_Response * res_in =
    (imu_interfaces__srv__ImuCalibration_Response *) response_msg;

  size_t accl_size = sizeof(req_in->imu_calib.linear_acceleration_covariance);
  size_t gyro_size = sizeof(req_in->imu_calib.angular_velocity_covariance);
  memcpy(mpu6050_msg.linear_acceleration_covariance, req_in->imu_calib.linear_acceleration_covariance, accl_size);
  memcpy(mpu6050_msg.angular_velocity_covariance, req_in->imu_calib.angular_velocity_covariance, gyro_size);

  accl_offset.x = req_in->imu_calib.linear_acceleration.x;
  accl_offset.y = req_in->imu_calib.linear_acceleration.y;
  accl_offset.z = req_in->imu_calib.linear_acceleration.z;

  gyro_offset.x = req_in->imu_calib.angular_velocity.x;
  gyro_offset.y = req_in->imu_calib.angular_velocity.y;
  gyro_offset.z = req_in->imu_calib.angular_velocity.z;

  is_calib = true;
  res_in->success = true;

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

}

void imu_status_service_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  std_srvs__srv__SetBool_Request * req_in =
    (std_srvs__srv__SetBool_Request *) request_msg;
  std_srvs__srv__SetBool_Response * res_in =
    (std_srvs__srv__SetBool_Response *) response_msg;

  if (req_in->data){
	  uint32_t i2cError = HAL_I2C_GetError(&hi2c1);
	  if (i2cError == HAL_I2C_ERROR_NONE) {
	      res_in->success = true;
	      res_in->message.data = "MPU6050 is connected.";
		  cs++;
	  }
	  else {
	      res_in->success = false;
	      res_in->message.data = "MPU6050 is not connect, Error with I2C interfaces.";
		  ct++;
	  }
  }
  else{
      res_in->success = false;
      res_in->message.data = "Request false.";
  }

}

void ADC_Averaged()
{
	for (int i = 0; i < 40; i++)
	{
		ADC_SumAPot[0] += ADCBuffer[2*i];
		ADC_SumAPot[1] += ADCBuffer[1+(2*i)];
	}

	for (int i = 0; i < 2; i++)
	{
		ADC_Average[i] = ADC_SumAPot[i] / 40;
		ADC_SumAPot[i] = 0;
	}
}

void Read_Buttons()
{
	joy.A = HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin);
	joy.B = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin);
	joy.C = HAL_GPIO_ReadPin(C_GPIO_Port, C_Pin);
	joy.D = HAL_GPIO_ReadPin(D_GPIO_Port, D_Pin);
	joy.K = HAL_GPIO_ReadPin(K_GPIO_Port, K_Pin);
}

void keyboard_callback(const void * response_msg){
  // Cast response message to expected type
//  robotic_interfaces__srv__Keyboard_Response * msgin =
//    (robotic_interfaces__srv__Keyboard_Response *) response_msg;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10)
	{
		if (main_Mode == 0) main_Mode = 1;
		else if (main_Mode == 1) main_Mode = 2;
		else if (main_Mode == 2) main_Mode = 0;
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  while (MPU6050_Init(&hi2c1) == 1);
  Kalman_Init(&kalmanX);
  Kalman_Init(&kalmanY);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, ADCBuffer, 80);
  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	// micro-ROS configuration

	rmw_uros_set_custom_transport(
		true,
		(void *) &hlpuart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  printf("Error on default allocators (line %d)\n", __LINE__);
	}

	GPIO_PinState B1 = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

	// micro-ROS app
	rcl_timer_t mpu6050_timer;
	rclc_support_t support;
	rclc_executor_t executor;
	rcl_allocator_t allocator;
	rcl_init_options_t init_options;

	const unsigned int timer_period = RCL_MS_TO_NS(10);
	const int timeout_ms = 5000;
	int executor_num = 3;

	// Get message type support
	const rosidl_message_type_support_t * imu_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);

	const rosidl_message_type_support_t * cmd_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

	const rosidl_service_type_support_t * imu_calib_type_support =
	  ROSIDL_GET_SRV_TYPE_SUPPORT(imu_interfaces, srv, ImuCalibration);

	const rosidl_service_type_support_t * imu_status_type_support =
	  ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);

	const rosidl_service_type_support_t * keyboard_type_support =
	  ROSIDL_GET_SRV_TYPE_SUPPORT(robotic_interfaces, srv, Keyboard);

	const rosidl_service_type_support_t * trigger_type_support =
	  ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);

	allocator = rcl_get_default_allocator();

	executor = rclc_executor_get_zero_initialized_executor();

	init_options = rcl_get_zero_initialized_init_options();

	RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 69));

	//create support init_options
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	//create timer
	rclc_timer_init_default(&mpu6050_timer, &support, timer_period, timer_callback);

	// create node
	rclc_node_init_default(&node, "G474RE_MPU6050_node", "", &support);

	// create publisher
	rclc_publisher_init_best_effort(&mpu6050_publisher, &node, imu_type_support, "mpu6050_publisher");
	rclc_publisher_init_default(&cmd_vel_publisher, &node, cmd_type_support, "cmd_vel");
	rclc_publisher_init_default(&cmd_vell_publisher, &node, cmd_type_support, "cmd_vell");
	//create subscriber


	//create service server
	if (B1 == GPIO_PIN_RESET) {
		rclc_service_init_default(&imu_calibration_server, &node, imu_calib_type_support, "imu_calibration");
		executor_num++;
	}
	else{
		on_calib = true;
	}

	rclc_service_init_default(&imu_status_server, &node, imu_status_type_support, "imu/status");

	//create service client
	rclc_client_init_default(&robot_controller_client, &node, keyboard_type_support, "Mode");
	rclc_client_init_default(&robot_controller_Ref_client, &node, keyboard_type_support, "Ref");
	rclc_client_init_default(&robot_controller_saved_client, &node, trigger_type_support, "SavePath");

	//create executor
	rclc_executor_init(&executor, &support.context, executor_num, &allocator);

	rclc_executor_add_timer(&executor, &mpu6050_timer);
	if (B1 == GPIO_PIN_RESET) rclc_executor_add_service(&executor, &imu_calibration_server, &imu_calibration_request, &imu_calibration_response, imu_calib_service_callback);
	rclc_executor_add_service(&executor, &imu_status_server, &imu_status_request, &imu_status_response, imu_status_service_callback);

	rclc_executor_spin(&executor);

	rmw_uros_sync_session(timeout_ms);

	//create message
	mpu6050_msg.header.frame_id = micro_ros_string_utilities_init("imu_frame");

	for(;;)
	{
		osDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

