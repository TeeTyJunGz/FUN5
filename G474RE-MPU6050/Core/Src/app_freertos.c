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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GRAVITY 9.80665
#define DEG_TO_RAD M_PI / 180.0
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

rcl_service_t imu_calibration_server;
imu_interfaces__srv__ImuCalibration_Request imu_calibration_request;
imu_interfaces__srv__ImuCalibration_Response imu_calibration_response;

MPU6050_t MPU6050;
uint16_t cc = 0;

offset3d_t accl_offset;
offset3d_t gyro_offset;

bool is_calib = false;
bool on_calib = false;
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

	if (timer != NULL) {
		if (is_calib || on_calib){
			MPU6050_Read_All(&hi2c1, &MPU6050);

			mpu6050_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
			mpu6050_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();

			mpu6050_msg.linear_acceleration.x = (GRAVITY * MPU6050.Ax) - accl_offset.x;
			mpu6050_msg.linear_acceleration.y = (GRAVITY * MPU6050.Ay) - accl_offset.y;
			mpu6050_msg.linear_acceleration.z = (GRAVITY * MPU6050.Az) - accl_offset.z;

			mpu6050_msg.angular_velocity.x = (DEG_TO_RAD * MPU6050.Gx) - gyro_offset.x;
			mpu6050_msg.angular_velocity.y = (DEG_TO_RAD * MPU6050.Gy) - gyro_offset.y;
			mpu6050_msg.angular_velocity.z = (DEG_TO_RAD * MPU6050.Gz) - gyro_offset.z;

			rcl_ret_t ret = rcl_publish(&mpu6050_publisher, &mpu6050_msg, NULL);

			if (ret != RCL_RET_OK)
			{
			  printf("Error publishing (line %d)\n", __LINE__);
			}

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
	const int timeout_ms = 1000;
	int executor_num = 1;

	// Get message type support
	const rosidl_message_type_support_t * imu_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);

	const rosidl_service_type_support_t * imu_calib_type_support =
	  ROSIDL_GET_SRV_TYPE_SUPPORT(imu_interfaces, srv, ImuCalibration);

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

	//create subscriber


	//create service server
	if (B1 == GPIO_PIN_RESET) {
		rclc_service_init_default(&imu_calibration_server, &node, imu_calib_type_support, "imu_calibration");
		executor_num++;
	}
	else{
		on_calib = true;
	}
	//create service client


	//create executor
	rclc_executor_init(&executor, &support.context, executor_num, &allocator);

	rclc_executor_add_timer(&executor, &mpu6050_timer);
	if (B1 == GPIO_PIN_RESET) rclc_executor_add_service(&executor, &imu_calibration_server, &imu_calibration_request, &imu_calibration_response, imu_calib_service_callback);

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

