#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <cmath>

// Wi-Fi Credentials
char ssid[] = "28125D";
char password[] = "RKKT1999!1988$";

// Micro-ROS Agent Details
// Last octet is set per-environment in platformio.ini (esp32dev = WiFi PC
// interface .195, esp32dev_ethernet = Ethernet PC interface .172) — flash
// with `./pio_build.sh run -e <env> -t upload` to pick which one.
#ifndef AGENT_IP_LAST_OCTET
#define AGENT_IP_LAST_OCTET 195
#endif
IPAddress agent_ip(10, 51, 51, AGENT_IP_LAST_OCTET); // IP of your ROS2 PC/Agent
const uint16_t agent_port = 8888;      // Standard micro-ROS UDP port
// ROS node configuration
const char* kNodeName = "esp32_differential_drive";
const char* kPublisherTopic = "wheel_states";
const char* kSubscriberTopic = "wheel_commands";
const int kExecutorTimeout = 100;  // ms
const size_t kDomainId = 0;  // ROS domain ID

const double TICKS_PER_REVOLUTION = 240; // Updated for your 576 TPR encoders

// --- Encoder Pin Configurations ---
const int LEFT_ENC_A = 4;   // Rob 4  Titus 4
const int LEFT_ENC_B = 5;  // Rob 5  Titus 5
const int RIGHT_ENC_A = 7;  // Rob 7  Titus 7
const int RIGHT_ENC_B = 6;  // Rob 6

// --- Motor Driver H-Bridge Configurations ---
const int LEFT_PWM = 15;   // rob 15  Titus 9
const int LEFT_DIR_1 = 16;  // Rob 16
const int LEFT_DIR_2 = 17;  // Rob 17

const int RIGHT_PWM = 9;   // Rob 9  Titus 15
const int RIGHT_DIR_1 = 10;  // Rob 10  Titus 16
const int RIGHT_DIR_2 = 11;  // Rob 11  Titus 17

// --- Status LED (onboard WS2812, most ESP32-S3 DevKit boards) ---
const int LED_PIN = 48;
const uint8_t LED_BRIGHTNESS = 40;
// How recently a /wheel_commands message must have arrived for the LED to
// show "actively receiving" (green) instead of just "connected" (blue).
const unsigned long CMD_RECEIVED_LED_WINDOW_MS = 300;

// --- PID Tuning Gains ---
double Kp = 1.5;
double Ki = 5.0;
double Kd = 0.05;

// Target velocities set by the micro-ROS subscriber callback (rad/s)
double target_vel_left = 0.0;
double target_vel_right = 0.0;

// PID Tracking States: Array index 0 = Left, 1 = Right
double prev_error[2] = {0.0, 0.0};
double integral_error[2] = {0.0, 0.0};

// --- micro-ROS Infrastructure ---
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Publisher Objects (Wheel States -> PC)
rcl_publisher_t wheel_state_pub;
sensor_msgs__msg__JointState wheel_state_msg;

// Subscriber Objects (PC -> Wheel Commands)
rcl_subscription_t wheel_cmd_sub;
std_msgs__msg__Float64MultiArray wheel_cmd_msg;

// Global Raw Counters (Modified by hardware interrupts)
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Local Memory Storage Arrays for Publisher
double wheel_pos[2] = {0.0, 0.0};
double wheel_vel[2] = {0.0, 0.0};

// Local Memory Storage Array for Incoming Subscriber Commands
double cmd_data[2] = {0.0, 0.0};

// Time Tracking Anchors
unsigned long last_time = 0;
long last_left_ticks = 0;
long last_right_ticks = 0;

// Watchdog: last time a /wheel_commands message was received. Also drives
// the status LED's "actively receiving commands" (green) state.
unsigned long last_cmd_received_ms = 0;
const unsigned long CMD_WATCHDOG_TIMEOUT_MS = 500;

// --- ATOMIC ACCESS GUARD MUTEX ---
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// Non-fatal check used during entity creation: on failure, bail out of
// createEntities() and let the reconnection state machine retry, instead of
// freezing the whole firmware.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { return false; } }

#define EXECUTE_EVERY_N_MS(MS, X)            \
  do {                                       \
    static volatile int64_t init = -1;       \
    if (init == -1) { init = millis(); }     \
    if (millis() - init > (MS)) {            \
      X;                                     \
      init = millis();                       \
    }                                        \
  } while (0)

enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
State state = WAITING_AGENT;

bool createEntities();
void destroyEntities();
void stopMotors();
void controlLoop();
void updateStatusLED();

// --- Encoder Interrupt Service Routines ---
void IRAM_ATTR readLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) { left_encoder_ticks++; }
  else { left_encoder_ticks--; }
}

void IRAM_ATTR readRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) { right_encoder_ticks++; }
  else { right_encoder_ticks--; }
}

// --- SUBSCRIBER CALLBACK FUNCTION ---
void wheelCmdCallback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;

  target_vel_left = msg->data.data[0];
  target_vel_right = msg->data.data[1];
  last_cmd_received_ms = millis();
}

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  // Configure Pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR_1, OUTPUT);
  pinMode(LEFT_DIR_2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR_1, OUTPUT);
  pinMode(RIGHT_DIR_2, OUTPUT);
  analogWriteFrequency(20000);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, RISING);

  stopMotors();
  last_time = millis();
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
      });
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        state = AGENT_CONNECTED;
      } else {
        destroyEntities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // 300ms timeout x 3 attempts (~900ms worst case) and a 1s poll
      // interval — loosened from an initial 100ms/1-attempt/200ms profile
      // that was tripping false disconnects on this Docker+WiFi path, which
      // has observed /wheel_states gaps up to ~0.6-1.0s even when healthy.
      EXECUTE_EVERY_N_MS(1000, {
        state = (rmw_uros_ping_agent(300, 3) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      });
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        controlLoop();
      }
      break;

    case AGENT_DISCONNECTED:
      stopMotors();
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }

  updateStatusLED();
}

// Creates all micro-ROS entities (node, publisher, subscriber, executor).
// Returns false on any failure instead of blocking forever, so the caller
// can fall back to WAITING_AGENT and retry once the agent is reachable again.
bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCSOFTCHECK(rclc_node_init_default(&node, "esp32_wheel_node", "", &support));

  // 1. Initialize JointState Publisher (/wheel_states)
  RCSOFTCHECK(rclc_publisher_init_default(
    &wheel_state_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/wheel_states"));

  // 2. Initialize Float64MultiArray Subscriber (/wheel_commands)
  // Best-effort QoS to match the ROS2-side publisher (see
  // DiffBotSystemHardware::on_init) — a reliable subscriber is
  // DDS-incompatible with a best-effort publisher and would receive nothing.
  RCSOFTCHECK(rclc_subscription_init(
    &wheel_cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/wheel_commands",
    &rmw_qos_profile_sensor_data));

  // 3. Configure Micro-ROS Executor (1 Handle for the single Array Subscriber)
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCSOFTCHECK(rclc_executor_add_subscription(&executor, &wheel_cmd_sub, &wheel_cmd_msg, &wheelCmdCallback, ON_NEW_DATA));

  // 4. Dynamic Memory Allocations for JointState Publisher Strings — static
  // storage, only needs to be wired up once even across reconnects.
  static bool msg_memory_initialized = false;
  if (!msg_memory_initialized) {
    static rosidl_runtime_c__String joint_names[2];
    joint_names[0].data = (char*)"left_wheel_joint";
    joint_names[0].size = strlen(joint_names[0].data);
    joint_names[0].capacity = joint_names[0].size + 1;

    joint_names[1].data = (char*)"right_wheel_joint";
    joint_names[1].size = strlen(joint_names[1].data);
    joint_names[1].capacity = joint_names[1].size + 1;

    wheel_state_msg.name.data = joint_names;
    wheel_state_msg.name.size = 2;
    wheel_state_msg.name.capacity = 2;

    wheel_state_msg.position.data = wheel_pos;
    wheel_state_msg.position.capacity = 2;
    wheel_state_msg.velocity.data = wheel_vel;
    wheel_state_msg.velocity.capacity = 2;

    // 5. Allocate Memory Array Links to receive incoming subscription details
    wheel_cmd_msg.data.capacity = 2;
    wheel_cmd_msg.data.size = 2; // Pre-set to match the expected 2-element payload
    wheel_cmd_msg.data.data = cmd_data;

    msg_memory_initialized = true;
  }

  last_time = millis();
  // Seed as already-stale (not "just received") so the watchdog still zeros
  // any leftover target velocity immediately, but the status LED doesn't
  // falsely flash green for a command that was never actually received.
  last_cmd_received_ms = millis() - CMD_WATCHDOG_TIMEOUT_MS;
  return true;
}

// Tears down micro-ROS entities. Safe to call even if creation only got
// partway through, and safe to call repeatedly.
void destroyEntities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  // Don't block trying to gracefully close a session with an agent we can no
  // longer reach.
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&wheel_state_pub, &node);
  rcl_subscription_fini(&wheel_cmd_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void stopMotors() {
  digitalWrite(LEFT_DIR_1, LOW);  digitalWrite(LEFT_DIR_2, LOW);
  digitalWrite(RIGHT_DIR_1, LOW); digitalWrite(RIGHT_DIR_2, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  target_vel_left = 0.0;
  target_vel_right = 0.0;
}

void updateStatusLED() {
  if (state != AGENT_CONNECTED) {
    neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0); // RED — not connected
  } else if (millis() - last_cmd_received_ms < CMD_RECEIVED_LED_WINDOW_MS) {
    neopixelWrite(LED_PIN, 0, LED_BRIGHTNESS, 0); // GREEN — actively receiving /wheel_commands
  } else {
    neopixelWrite(LED_PIN, 0, 0, LED_BRIGHTNESS); // BLUE — connected, idle
  }
}

// Runs the ~20 Hz encoder/PID/motor/publish cycle. Only called while
// AGENT_CONNECTED, i.e. rcl entities are guaranteed to be valid.
void controlLoop() {
  unsigned long current_time = millis();
  unsigned long time_delta_ms = current_time - last_time;

  if (time_delta_ms >= 50) {
    double dt = (double)time_delta_ms / 1000.0;

    // Command watchdog: stop driving if no /wheel_commands message has
    // arrived recently (e.g. the PC-side stack went away but the micro-ROS
    // session itself is still alive).
    if (millis() - last_cmd_received_ms > CMD_WATCHDOG_TIMEOUT_MS) {
      target_vel_left = 0.0;
      target_vel_right = 0.0;
    }

    long safe_left_ticks;
    long safe_right_ticks;
    portENTER_CRITICAL(&encoderMux);
    safe_left_ticks = left_encoder_ticks;
    safe_right_ticks = right_encoder_ticks;
    portEXIT_CRITICAL(&encoderMux);

    wheel_pos[0] = ((double)safe_left_ticks / TICKS_PER_REVOLUTION) * 2.0 * M_PI;
    wheel_pos[1] = ((double)safe_right_ticks / TICKS_PER_REVOLUTION) * 2.0 * M_PI;

    long delta_left_ticks = safe_left_ticks - last_left_ticks;
    long delta_right_ticks = safe_right_ticks - last_right_ticks;

    wheel_vel[0] = (((double)delta_left_ticks / TICKS_PER_REVOLUTION) * 2.0 * M_PI) / dt;
    wheel_vel[1] = (((double)delta_right_ticks / TICKS_PER_REVOLUTION) * 2.0 * M_PI) / dt;

    // --- LEFT WHEEL PID ---
    double error_l = target_vel_left - wheel_vel[0];
    integral_error[0] += error_l * dt;
    integral_error[0] = constrain(integral_error[0], -50.0, 50.0);
    double derivative_l = (error_l - prev_error[0]) / dt;
    double output_l = (Kp * error_l) + (Ki * integral_error[0]) + (Kd * derivative_l);
    prev_error[0] = error_l;

    // --- RIGHT WHEEL PID ---
    double error_r = target_vel_right - wheel_vel[1];
    integral_error[1] += error_r * dt;
    integral_error[1] = constrain(integral_error[1], -50.0, 50.0);
    double derivative_r = (error_r - prev_error[1]) / dt;
    double output_r = (Kp * error_r) + (Ki * integral_error[1]) + (Kd * derivative_r);
    prev_error[1] = error_r;

    // --- H-BRIDGE OUTPUT EXECUTION ---
    if (output_l > 0) {
      digitalWrite(LEFT_DIR_1, HIGH); digitalWrite(LEFT_DIR_2, LOW);
    } else if (output_l < 0) {
      digitalWrite(LEFT_DIR_1, LOW);  digitalWrite(LEFT_DIR_2, HIGH);
    } else {
      digitalWrite(LEFT_DIR_1, LOW);  digitalWrite(LEFT_DIR_2, LOW);
    }

    if (output_r > 0) {
      digitalWrite(RIGHT_DIR_1, HIGH); digitalWrite(RIGHT_DIR_2, LOW);
    } else if (output_r < 0) {
      digitalWrite(RIGHT_DIR_1, LOW);  digitalWrite(RIGHT_DIR_2, HIGH);
    } else {
      digitalWrite(RIGHT_DIR_1, LOW);  digitalWrite(RIGHT_DIR_2, LOW);
    }

    int left_pwm = constrain(abs(output_l), 0, 255);
    int right_pwm = constrain(abs(output_r), 0, 255);

    analogWrite(LEFT_PWM, left_pwm);
    analogWrite(RIGHT_PWM, right_pwm);

    wheel_state_msg.position.size = 2;
    wheel_state_msg.velocity.size = 2;

    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    wheel_state_msg.header.stamp.sec = tv.tv_sec;
    wheel_state_msg.header.stamp.nanosec = tv.tv_nsec;

    rcl_publish(&wheel_state_pub, &wheel_state_msg, NULL);

    last_left_ticks = safe_left_ticks;
    last_right_ticks = safe_right_ticks;
    last_time = current_time;
  }
}
