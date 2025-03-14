// ======== Include necessary Micro-ROS and ESP32 libraries ========
#include <micro_ros_arduino.h>           // Biblioteca de Micro-ROS para ESP32
#include <rcl/rcl.h>                     // Biblioteca principal de ROS 2
#include <rcl/error_handling.h>          // Manejo de errores en ROS 2
#include <rclc/rclc.h>                   // Biblioteca cliente de Micro-ROS
#include <rclc/executor.h>               // Ejecutor para manejar callbacks
#include <std_msgs/msg/float32.h>        // Tipo de mensaje Float32
#include <rmw_microros/rmw_microros.h>   // Funciones de middleware para Micro-ROS
#include <stdio.h>                       // Biblioteca de entrada/salida estándar para depuración
#include <micro_ros_utilities/string_utilities.h>  // Utilidades para manejo de strings en Micro-ROS

// ======== Declaración de entidades de Micro-ROS ========
rclc_support_t support;       // Contexto de ejecución de Micro-ROS
rclc_executor_t executor;     // Administrador de tareas (timers, suscriptores)
rcl_allocator_t allocator;    // Manejador de memoria

rcl_node_t control_node;    // Definición del nodo de ROS 2

rcl_publisher_t motor_output_publisher;    // Publicador de la velocidad del motor
rcl_subscription_t set_point_subscriber;   // Suscriptor de la referencia de velocidad
rcl_timer_t timer;                   // Temporizador para publicar la velocidad

// ======== Declaración de mensajes de Micro-ROS ========
std_msgs__msg__Float32 motor_output_msg;    // Mensaje de salida de velocidad del motor
std_msgs__msg__Float32 set_point_msg;       // Mensaje de referencia de velocidad

const double timer_timeout = 0.09398;  // Periodo del temporizador en milisegundos

// ======== Variables para el encoder ========
volatile int32_t tiempo_act = 0,
                 tiempo_ant = 0,
                 delta_tiempo = 2e9,
                 contador = 0;
int32_t revoluciones;

float posicion=0, posactual = 0, posanterior = 0,
      velocidad = 0, loop_vel = 0,
      resolucion = 0.7891;  // Resolución del encoder

int pulsos = 456,       //Número de pulsos a la salida del motorreductor
    sign = 0;

volatile bool encoderDirection = false;

float rpm2rads = 0.104719;  // Factor de conversión de RPM a rad/s

// ======== Variables para el controlador PID ========
float currentTime = 0.0, elapsedTime = 0.0, previousTime = 0.0,
      pwm_set_point = 0.0, error = 0.0, integral = 0.0, derivative = 0.0, lastError = 0.0, output = 0.0,
      kp = 0.10, ki = 0.0857, kd = 0.00455;

int pwm_signal = 0, pwm_prev = 0;

// ======== Definición de macros para control y depuración ========
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ======== Definición de pines de hardware ========
#define PWM_PIN  26 // Pin de salida PWM para el motor
#define PWM_FRQ 5000 // Frecuencia de PW
#define PWM_RES 8 // Resolución de PWM (2^8 = 256 niveles)
#define PWM_CHNL 0 // Canal de PWM

#define In1  14 // Pin de control del puente H
#define In2  27 // Pin de control del puente H

#define EncA  33 // Pin de la señal A del encoder
#define EncB  32 // Pin de la señal B del encoder

#define OMEGA_MIN -29.321
#define OMEGA_MAX 29.321

// ======== Máquinad de estados para conexión de Micro-ROS ========
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

// ======== Declaración de funciones ========
bool create_entities();
void destroy_entities();

// ======== Funciones ========
void IRAM_ATTR Encoder(){
  if (digitalRead(EncB) == digitalRead(EncA)){
    contador++;
    encoderDirection = true;
  }else{
    contador--;
    encoderDirection = false;
  }
  tiempo_act = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant = tiempo_act;
}

void velocidad_w(){
  if (encoderDirection){
    sign = 1;
  }
  else{
    sign = -1;
  }
  
  //Cálculo de la velocidad
  if(delta_tiempo > 250){
    velocidad = 60000000/(pulsos * delta_tiempo); // Conversión a segundos
    velocidad = velocidad*rpm2rads*sign; // Conversión a rad/s
  }
  
  if (micros() - tiempo_act > 100000) {  // 100,000 us = 100 ms
    velocidad = 0.0;
  }
}

void publish_vel(){
  motor_output_msg.data = velocidad;
  rcl_publish(&motor_output_publisher, &motor_output_msg, NULL);
}

// ======== Timer Callback: Publicación de la velocidad angular del motor ========
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    velocidad_w();
    publish_vel();
  }
}
// ======== Subscriber Callback: Lectura de señal de referencia ========
void subscription_callback(const void * msgin) { 
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  pwm_set_point = constrain(msg->data, OMEGA_MIN, OMEGA_MAX);

  error = pwm_set_point - velocidad; // Referencia - medición del encoder

  if (fabs(error) >= 0.1){

  integral += error * 0.09398;

  derivative = (error-lastError) / 0.09398;

  output = kp*error + ki*integral + kd * derivative;

  lastError = error;

  if (output < 0){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }else if(output > 0){
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }else{
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  }

  pwm_signal = (int)(fabs(output / 12)*130)+ 125; // Mapeo a PWM (Con numero de PWM disponible)
  pwm_prev = pwm_signal;
  }else{
    pwm_signal = pwm_prev;
  }
  ledcWrite(PWM_CHNL, pwm_signal);
}

// ======== Configuración inicial del sistema ========
void setup(){
  set_microros_transports();  // Inicialización de communicación con Micro Ros
  
  // Configuración de pines de saida para el puente H
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);

  // Configuración de canal de señal PWM
  pinMode(PWM_PIN, OUTPUT);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  // Entradas del encoder del motor
  pinMode(EncA, INPUT_PULLUP);    // Señal A del encoder como entrada con pull-up
  pinMode(EncB, INPUT_PULLUP);    // Señal B del encoder como entrada con pull-up
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);
}

// ======== Bucle principal ========
void loop(){
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        //velocidad_w();
        //publish_vel();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
  
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    
    default:
      break;
  }
}

// ======== Funciones de creación y limpieza de entidades de ROS 2 ========
bool create_entities() {
  // Iniciaización de  Micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&control_node, "control_node_ROSario", "", &support));

  // Publicador
  RCCHECK(rclc_publisher_init_default(
    &motor_output_publisher,
    &control_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output_ROSario"));

  // Suscriptor
  RCCHECK(rclc_subscription_init_default(
    &set_point_subscriber,
    &control_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "set_point_ROSario"));

  // Timer
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &set_point_subscriber, &set_point_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&set_point_subscriber, &control_node);
  rcl_publisher_fini(&motor_output_publisher, &control_node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&control_node);
  rclc_support_fini(&support);
}