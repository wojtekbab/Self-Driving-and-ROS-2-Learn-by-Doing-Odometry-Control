#include <Arduino.h>
#include <PID_v1.h>

// encoder pins
#define GPIO_MOT1_ENCA 10
#define GPIO_MOT1_ENCB 13
#define GPIO_MOT2_ENCA 27
#define GPIO_MOT2_ENCB 9

// pwm pins for control motors
#define GPIO_MOT1_PWM_R 5
#define GPIO_MOT1_PWM_L 2
#define GPIO_MOT2_PWM_R 25
#define GPIO_MOT2_PWM_L 26

static const BaseType_t cpu_0 = 0;
static const BaseType_t cpu_1 = 1;

// FreeRTOS kernel objects
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t mutex_state_variables = NULL;

// values used by interrupts
volatile long encoder_counter_mot1_global = 0;
volatile long encoder_counter_mot2_global = 0;

// relative speed od motor axle
float omega_rot_R_global = 0.0f;
float omega_rot_L_global = 0.0f;

void IRAM_ATTR ISR_callback_mot1();
void IRAM_ATTR ISR_callback_mot2();
void speed_calculation_and_publishing(void *pvParameters);
void motors_task_and_reference_handle(void *pvParameters);
void decodeCommand(String cmd, double &right_speed, double &left_speed);
float saturation(float value, float limit);

void setup()
{
  // serial
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Serial initialized!");

  // interrupts
  pinMode(GPIO_MOT1_ENCA, INPUT_PULLUP);
  pinMode(GPIO_MOT1_ENCB, INPUT_PULLUP);
  pinMode(GPIO_MOT2_ENCA, INPUT_PULLUP);
  pinMode(GPIO_MOT2_ENCB, INPUT_PULLUP);

  attachInterrupt(GPIO_MOT1_ENCA, ISR_callback_mot1, CHANGE);
  attachInterrupt(GPIO_MOT1_ENCB, ISR_callback_mot1, CHANGE);
  attachInterrupt(GPIO_MOT2_ENCA, ISR_callback_mot2, CHANGE);
  attachInterrupt(GPIO_MOT2_ENCB, ISR_callback_mot2, CHANGE);

  // FreeRTOS objects
  mutex_state_variables = xSemaphoreCreateMutex();

  // tasks
  Serial.println("Initializing tasks...");
  xTaskCreatePinnedToCore(speed_calculation_and_publishing, "reference_task", 2048, NULL, 1, NULL, cpu_0);
  xTaskCreatePinnedToCore(motors_task_and_reference_handle, "motors_task", 4096, NULL, 4, NULL, cpu_1);
}

void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void IRAM_ATTR ISR_callback_mot1()
{
  static int8_t mot1_last_state = 0;
  int8_t mot1_current_state = (digitalRead(GPIO_MOT1_ENCA) << 1) | digitalRead(GPIO_MOT1_ENCB);

  switch ((mot1_last_state << 2) | mot1_current_state)
  {
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot1_global++;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  case 0b0010:
  case 0b0100:
  case 0b1011:
  case 0b1101:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot1_global--;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  }
  mot1_last_state = mot1_current_state;
}

void IRAM_ATTR ISR_callback_mot2()
{
  static int8_t mot2_last_state = 0;
  int8_t mot2_current_state = (digitalRead(GPIO_MOT2_ENCA) << 1) | digitalRead(GPIO_MOT2_ENCB);

  switch ((mot2_last_state << 2) | mot2_current_state)
  {
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot2_global--;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  case 0b0010:
  case 0b0100:
  case 0b1011:
  case 0b1101:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot2_global++;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  }
  mot2_last_state = mot2_current_state;
}

void speed_calculation_and_publishing(void *pvParameters)
{
  // local consts
  u_int16_t const IMPULSES_PER_ROT = 1320; // [IMPULSES]
  // float const WHEEL_RADIUS = 0.045;        // [m]
  // float const WHEELS_DISTANCE = 0.245f;    // [m]

  // local vars
  long encoder_counter_mot1 = 0.0f;
  long encoder_counter_mot2 = 0.0f;

  float theta_rot_R = 0.0f; // rotor (relative) angle (right motor)
  float theta_rot_L = 0.0f;
  float theta_rot_R_last = 0.0f;
  float theta_rot_L_last = 0.0f;
  float omega_rot_R = 0.0f; // rotor (relative) angular speed (right motor)
  float omega_rot_L = 0.0f;

  char right_motor_vel_sign = ' ';
  char left_motor_vel_sign = ' ';

  // State_variables_struct state_variables;

  unsigned long last_time = 0;
  float dt = 0.0f;

  while (1)
  {

    dt = (float)(micros() - last_time) / 1000000.0f;
    last_time = micros();

    // read values
    portENTER_CRITICAL(&spinlock); // disable interrupts for a while
    encoder_counter_mot1 = encoder_counter_mot1_global;
    encoder_counter_mot2 = encoder_counter_mot2_global;
    portEXIT_CRITICAL(&spinlock);

    // calculations
    theta_rot_R = ((float)encoder_counter_mot1 / IMPULSES_PER_ROT) * 2.0f * PI;
    theta_rot_L = ((float)encoder_counter_mot2 / IMPULSES_PER_ROT) * 2.0f * PI;
    omega_rot_R = (theta_rot_R - theta_rot_R_last) / dt;
    omega_rot_L = (theta_rot_L - theta_rot_L_last) / dt;
    theta_rot_R_last = theta_rot_R;
    theta_rot_L_last = theta_rot_L;

    // write values
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    omega_rot_R_global = omega_rot_R;
    omega_rot_L_global = omega_rot_L;
    xSemaphoreGive(mutex_state_variables);
    if (omega_rot_R >= 0.0f)
    {
      right_motor_vel_sign = 'p';
    }
    else
    {
      right_motor_vel_sign = 'n';
    }
    if (omega_rot_L >= 0.0f)
    {
      left_motor_vel_sign = 'p';
    }
    else
    {
      left_motor_vel_sign = 'n';
    }
    Serial.printf("r%c%05.2f,l%c%05.2f, SEND\r\n",
              right_motor_vel_sign, fabs(omega_rot_R),
              left_motor_vel_sign,  fabs(omega_rot_L));

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void motors_task_and_reference_handle(void *pvParameters)
{
  // setting PWM properties
  const int freq = 5000;
  const int chanel_mot1_A = 0;
  const int chanel_mot1_B = 1;
  const int chanel_mot2_A = 2;
  const int chanel_mot2_B = 3;
  const int resolution = 10;

  double omega_rot_R = 0.0f; // rotor (relative) angular speed
  double omega_rot_L = 0.0f;
  double reference_omega_rot_R = 0.0f; // rotor (relative) angular speed
  double reference_omega_rot_L = 0.0f;

  String buffer = "";

  double control_signal_R = 0.0f;
  double control_signal_L = 0.0f;

  // configure LED PWM functionalities
  ledcSetup(chanel_mot1_A, freq, resolution);
  ledcSetup(chanel_mot1_B, freq, resolution);
  ledcSetup(chanel_mot2_A, freq, resolution);
  ledcSetup(chanel_mot2_B, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(GPIO_MOT1_PWM_R, chanel_mot1_A);
  ledcAttachPin(GPIO_MOT1_PWM_L, chanel_mot1_B);
  ledcAttachPin(GPIO_MOT2_PWM_R, chanel_mot2_A);
  ledcAttachPin(GPIO_MOT2_PWM_L, chanel_mot2_B);

  // Tuning
  double Kp_r = 40;
  double Ki_r = 30;
  double Kd_r = 0.1;
  double Kp_l = 40;
  double Ki_l = 30;
  double Kd_l = 0.1;

  // Controller
  PID rightMotor(&omega_rot_R, &control_signal_R, &reference_omega_rot_R, Kp_r, Ki_r, Kd_r, DIRECT);
  PID leftMotor(&omega_rot_L, &control_signal_L, &reference_omega_rot_L, Kp_l, Ki_l, Kd_l, DIRECT);
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  while (1)
  {
    while (Serial.available())
    {
      char c = Serial.read();

      if (c != '\r') // ignoruj carriage return
        buffer += c;

      if (c == '\n')
      {
        decodeCommand(buffer, reference_omega_rot_R, reference_omega_rot_L);
        Serial.printf("%s REC\r\n", buffer);
        buffer = "";
      }
    }

    // read values (protect globals with semaphore), then compute based on fresh inputs
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    omega_rot_R = omega_rot_R_global;
    omega_rot_L = omega_rot_L_global;
    xSemaphoreGive(mutex_state_variables);

    // compute control after inputs are updated
    rightMotor.Compute();
    leftMotor.Compute();

    // // saturate signal with limit  +/-12V
    double PWM_mot_R = saturation(control_signal_R, 1024.0f);
    double PWM_mot_L = saturation(control_signal_L, 1024.0f);

    // Write PWM to driver MDD3A, its require 2 PWMs for each motor
    if (PWM_mot_R >= 0)
    {
      ledcWrite(chanel_mot1_A, (int)PWM_mot_R);
      ledcWrite(chanel_mot1_B, (int)0);
    }
    else
    {
      ledcWrite(chanel_mot1_A, (int)0);
      ledcWrite(chanel_mot1_B, (int)-PWM_mot_R);
    }

    if (PWM_mot_L >= 0)
    {
      ledcWrite(chanel_mot2_A, (int)PWM_mot_L);
      ledcWrite(chanel_mot2_B, (int)0);
    }
    else
    {
      ledcWrite(chanel_mot2_A, (int)0);
      ledcWrite(chanel_mot2_B, (int)-PWM_mot_L);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void decodeCommand(String cmd, double &right_speed, double &left_speed)
{
  cmd.trim();

  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);

  // 1st part: "rp1.5"
  String rpPart = cmd.substring(0, firstComma);
  // 2nd part: "ln2.0"
  String lpPart = cmd.substring(firstComma + 1, secondComma);

  // --- RIGHT WHEEL ---
  char sign_r = rpPart.charAt(1); // 'p' or 'n'
  float value_r = rpPart.substring(2).toFloat();
  if (sign_r == 'n')
    value_r = -value_r;

  // --- LEFT WHEEL ---
  char sign_l = lpPart.charAt(1);
  float value_l = lpPart.substring(2).toFloat();
  if (sign_l == 'n')
    value_l = -value_l;

  right_speed = value_r;
  left_speed = value_l;
}

float saturation(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  else if (value < -limit)
  {
    return -limit;
  }
  else
  {
    return value;
  }
}