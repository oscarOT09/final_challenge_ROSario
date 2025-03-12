// ======== Include necessary Micro-ROS and ESP32 libraries ========
#include <micro_ros_arduino.h>           // Micro-ROS library for ESP32
#include <rcl/rcl.h>                     // ROS 2 client library (Core)
#include <rcl/error_handling.h>          // ROS 2 error handling utilities
#include <rclc/rclc.h>                   // Micro-ROS client library
#include <rclc/executor.h>               // Executor to handle callbacks
#include <std_msgs/msg/float32.h>        // Float message type for LED brightness
#include <rmw_microros/rmw_microros.h>   // Middleware functions for Micro-ROS
#include <stdio.h>                       // Standard I/O for debugging
#include <micro_ros_utilities/string_utilities.h>  // Utilities for handling strings

// ======== Micro-ROS Entity Declarations ========
rclc_support_t support;       // Micro-ROS execution context
rclc_executor_t executor;     // Manages execution of tasks (timers, subscribers)
rcl_allocator_t allocator;    // Handles memory allocation

rcl_node_t control_node;              // Defines the ROS 2 node

rcl_publisher_t motor_output_publisher;    // Publishes motor output signal
rcl_publisher_t tmp_publisher;
rcl_subscription_t set_point_subscriber;   // Subscribes to set point input signal
rcl_timer_t timer;                   // Periodically publishes button state

// ======== Micro-ROS Messages Declarations ========
std_msgs__msg__Float32 motor_output_msg;    // Holds the button state ("active" or "inactive")
std_msgs__msg__Float32 set_point_msg;      // Holds the LED brightness value

std_msgs__msg__Float32 tmp_msg;

const double timer_timeout = 0.09398;  // Timer period (ms)

// =========== Variables para el encoder

volatile int32_t tiempo_act = 0,
        tiempo_ant = 0,
        delta_tiempo = 2e9,
        contador = 0;
int32_t revoluciones;

float posicion=0, posactual = 0, posanterior = 0,
      velocidad = 0, loop_vel = 0,
      resolucion = 0.7891;  //Definir resolución del encoder

int pulsos = 456, sign = 0;      //Número de pulsos a la salida del motorreductor

volatile bool BSet = 0,
              ASet = 0,
              encoderDirection = false;

float rpm2rads = 0.104719;

// ======= Variables para el controlador PID
float currentTime = 0.0, elapsedTime = 0.0, previousTime = 0.0,
      pwm_set_point = 0.0, error = 0.0, integral = 0.0, derivative = 0.0, lastError = 0.0, output = 0.0,
      kp = 0.682256, ki = 33.575487, kd = 0.001333;

// ======== Macro Definitions ========
// Error handling macros
//Executes fn and returns false if it fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ======== Hardware Pin Definitions ========
#define PWM_PIN  26 // EnA Pin in Motor Driver
#define PWM_FRQ 5000 //Frecuencia de PWM
#define PWM_RES 8 //Resolución de PWM 2^10 = 1024
#define PWM_CHNL 0 // Canal de PWM

#define In1  14 // Pin de salida digital 
#define In2  27 // Pin de salida digital

#define EncA  33 // GPIO para señal A del encoder
#define EncB  32 // GPIO para señal B del encoder

#define OMEGA_MIN -29.321
#define OMEGA_MAX 29.321

// ======== Micro-ROS Connection State Machine ========
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

// ======== Function Prototypes ========
bool create_entities();
void destroy_entities();

void IRAM_ATTR Encoder()
{
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);
  if (BSet == ASet)
  {
    contador++;
    encoderDirection = true;
  }
  //Si ambas señales leídas son distintas, el motor gira en sentido horario
  //y se decrementa un contador para saber el número de lecturas
  else
  {
    contador--;
    encoderDirection = false;
  }
  tiempo_act = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant = tiempo_act;
}

void velocidad_w(){
  if (encoderDirection){
      posicion = contador * resolucion; //Convertir a grados
      if (contador >= pulsos) //Contar por revoluciones
      {
        revoluciones++;
        contador = 0;
      }
      sign = 1;
    }
    else{
      posicion = contador * resolucion; //Convertir a grados 
      if (contador <= -pulsos) //Contar por revoluciones
      {
        revoluciones--;
        contador = 0;
      }
      sign = -1;
    }
    
    //Cálculo de la velocidad mediante un delta de tiempo  
    if(delta_tiempo > 250){
      velocidad = 60000000/(pulsos * delta_tiempo); //  Convertir a segundos nuevamente multiplicando por 60000000
      loop_vel = velocidad*rpm2rads*sign;
    }
}

void publish_vel(){
  motor_output_msg.data = loop_vel;
  rcl_publish(&motor_output_publisher, &motor_output_msg, NULL);
}

// ======== Timer Callback: Publishes motor angular velocity Periodically ========
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    /*
    tiempo_act = micros();
    delta_tiempo = tiempo_act - tiempo_ant;
    tiempo_ant = tiempo_act;

    float delta_tiempo_seg = delta_tiempo / 1e6;

    // Calcular velocidad en rad/s
    float vueltas = contador / (float)pulsos;  
    velocidad = (vueltas / delta_tiempo_seg) * 2 * M_PI;  // rad/s  
    
    contador = 0;

    motor_output_msg.data = velocidad;
    */
    publish_vel();
  }
}
// ======== Subscriber Callback: Adjusts LED Brightness ========
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  pwm_set_point = constrain(msg->data, OMEGA_MIN, OMEGA_MAX);

  if (pwm_set_point < 0){
      digitalWrite(In1, HIGH);
      digitalWrite(In2, LOW);
  }else if(pwm_set_point > 0){
      digitalWrite(In1, LOW);
      digitalWrite(In2, HIGH); 
  }else{
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  }
  
  currentTime = millis();
  elapsedTime = currentTime - previousTime;

  error = pwm_set_point - velocidad; // Referencia - medición del encoder
  
  integral += error * (elapsedTime/1000);
  

  derivative = (error-lastError) / (elapsedTime/1000);

  output = kp*error + ki*integral + kd * derivative;

  lastError = error;
  previousTime = currentTime;

  int pwm_signal = (int)(fabs(output/12) * 255); // Mapeo a PWM
  
 // int pwm_signal = (int)(fabs(pwm_set_point/OMEGA_MAX) * 255);

  ledcWrite(PWM_CHNL, pwm_signal);
  //tmp_msg.data = velocidad;
  //rcl_publish(&tmp_publisher, &tmp_msg, NULL);
}

void setup(){
  set_microros_transports();  // Initialize Micro-ROS communication

  // Setup Motor Driver pins 
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);

  // Setup MOTOR PWM
  pinMode(PWM_PIN, OUTPUT);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  // Setup Encoder inputs
  pinMode(EncA, INPUT_PULLUP);    // Señal A del encoder como entrada con pull-up
  pinMode(EncB, INPUT_PULLUP);    // Señal B del encoder como entrada con pull-up
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);
}

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
        velocidad_w();
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

// ======== ROS 2 Entity Creation and Cleanup Functions ========
bool create_entities() {
  // Initialize Micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&control_node, "control_node_ROSario", "", &support));

  // Initialize Button Publisher
  RCCHECK(rclc_publisher_init_default(
      &motor_output_publisher,
      &control_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "motor_output_ROSario"));

  RCCHECK(rclc_publisher_init_default(
      &tmp_publisher,
      &control_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "pwm_ROSario"));

  // Initialize LED Subscriber
  RCCHECK(rclc_subscription_init_default(
      &set_point_subscriber,
      &control_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "set_point_ROSario"));

  // Initialize Timer
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));
  
  // Initialize Executor
  // create zero initialised executor (no configured) to avoid memory problems
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