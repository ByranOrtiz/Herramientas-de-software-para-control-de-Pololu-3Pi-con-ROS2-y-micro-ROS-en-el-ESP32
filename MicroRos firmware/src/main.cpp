// ================================================================================
// Dependencias y librerías
// ================================================================================
#include <Arduino.h>
#include <tinycbor.h>
#include <stdio.h>
#include <micro_ros_platformio.h>

extern "C" {
  #include <rcl/rcl.h>
  #include <rcl/error_handling.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <geometry_msgs/msg/twist.h>
  #include <std_msgs/msg/int8.h>
  #include <std_msgs/msg/int32.h>
}

// ================================================================================
// Variables básicas del robot
// ================================================================================
uint8_t uart_send_buffer[32] = {0};
static const unsigned int control_time_ms = 100; 
volatile float phi_ell = 0; 
volatile float phi_r = 0; 

// ================================================================================
// Clientes ROS2
// ================================================================================
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ================================================================================
// Conversión cmd_vel → velocidades RPM
// ================================================================================
const float l = 0.048;    
const float r = 0.016;    
const float Pi = 3.14159;

float radsecLeft = 0, radsecRight = 0;  
float motorSpeedLeft = 0, motorSpeedRight = 0; 

// ================================================================================
// Macros de comprobación ROS2
// ================================================================================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ================================================================================
// Funciones auxiliares
// ================================================================================
void error_loop() {
  while(1) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Limita el valor 'value' a un máximo 'maxLimit'
 */
float limitToMaxValue(float value, float maxLimit) {
  return (value > maxLimit) ? maxLimit : value;
}

// ================================================================================
// Envío de velocidades de ruedas por UART en formato CBOR
// ================================================================================
void encode_send_wheel_speeds_task(void * p_params) {
  TickType_t last_control_time;
  const TickType_t control_freq_ticks = pdMS_TO_TICKS(control_time_ms);
  last_control_time = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&last_control_time, control_freq_ticks);

    Serial.print(" phi ell rpm = ");
    Serial.print(phi_ell);
    Serial.print(" phi r rpm = ");
    Serial.print(phi_r);

    TinyCBOR.Encoder.init(uart_send_buffer, sizeof(uart_send_buffer));
    TinyCBOR.Encoder.create_array(2);
    TinyCBOR.Encoder.encode_float(phi_ell);
    TinyCBOR.Encoder.encode_float(phi_r);
    TinyCBOR.Encoder.close_container();
    Serial2.write(TinyCBOR.Encoder.get_buffer(), TinyCBOR.Encoder.get_buffer_size());        
  }
}

// ================================================================================
// Callback de ROS2: conversión de cmd_vel a velocidades por rueda
// ================================================================================
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  radsecLeft = ((linear - l * angular) / r);
  radsecRight = ((linear + l * angular) / r);

  motorSpeedLeft = 60 * radsecLeft / (2 * Pi);
  motorSpeedRight = 60 * radsecRight / (2 * Pi);
}

// ================================================================================
// Actualización de velocidades y saturación
// ================================================================================
void ros2_cmdvel_task(void * p_params) {
  while(1) {
    phi_ell = limitToMaxValue(motorSpeedLeft, 800);
    phi_r = limitToMaxValue(motorSpeedRight, 800);

    Serial.println("Hello");
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ================================================================================
// Inicialización del sistema y configuración ROS2
// ================================================================================
void setup() {
  // Dirección IP del agente (máquina virtual conectada a la red).
  // Verificar, ya que el router puede asignar una IP diferente.
  IPAddress agent_ip(192, 168, 50, 3);
  set_microros_wifi_transports("Robotat", "iemtbmcit116", agent_ip, 8888);
 
  Serial.begin(115200);
  Serial2.begin(115200);
  TinyCBOR.init();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "MicroROS_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_13")); //para usar teleoperación se carga cmd_vel, para el nodo de control
	//se carga cmd_vel_ID, respectivo a cada robot.

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  xTaskCreate(encode_send_wheel_speeds_task, "encode_send_wheel_speeds_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(ros2_cmdvel_task, "ros2_cmdvel_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}

// ================================================================================
// Bucle principal
// ================================================================================
void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
}
