#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

// ----------------- ROS -----------------
ros::NodeHandle nh;
void cmdVelCb(const geometry_msgs::Twist &twist);
void pumpCb(const std_msgs::Bool &msg);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);
ros::Subscriber<std_msgs::Bool> pumpSub("/pump_control", &pumpCb);

// ----------------- ROS Publisher -----------------
std_msgs::Bool encoderStuckMsg;
ros::Publisher encoderStuckPub("/encoder_stuck", &encoderStuckMsg);

// ----------------- Motor & Encoder Pins -----------------
#define PWM_PIN_R 19
#define DIR_PIN1_R 26
#define DIR_PIN2_R 18
#define ENCODER_PIN_A_R 34
#define ENCODER_PIN_B_R 35

#define PWM_PIN_L 32
#define DIR_PIN1_L 2
#define DIR_PIN2_L 15
#define ENCODER_PIN_A_L 36
#define ENCODER_PIN_B_L 39

#define PUMP_PIN 25
#define LED_PIN_1 27
#define LED_PIN_2 14

// ----------------- Robot Params -----------------
const float PPR = 374;
const float WHEEL_DIAMETER_CM = 8.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.1416 * WHEEL_DIAMETER_CM;
const float WHEEL_BASE_CM = 33.0;

ESP32Encoder encoderR;
ESP32Encoder encoderL;

// ----------------- Global Velocity Inputs -----------------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile float linear_cm_s = 0;
volatile float angular_rad_s = 0;
volatile float targetVelocity_R = 0;
volatile float targetVelocity_L = 0;

// ----------------- PID Config -----------------
float alpha = 0.3;
float K_line = 2.2;

float Kp = 0.68, Ki = 0.35, Kd = 0.08, ff = 0.12;
float eprevR = 0, eintegralR = 0, dFilterR = 0;
float eprevL = 0, eintegralL = 0, dFilterL = 0;
float dAlpha = 0.3;
float integralMin = -150, integralMax = 150;
long prevTimeR = 0, prevTimeL = 0;
long lastCountR = 0, lastCountL = 0;
float measured_R = 0, measured_L = 0;

// ----------------- Callback: Twist -----------------
void cmdVelCb(const geometry_msgs::Twist &twist) {
  portENTER_CRITICAL(&mux);
  linear_cm_s = twist.linear.x * 100.0;
  angular_rad_s = twist.angular.z;
  portEXIT_CRITICAL(&mux);
}

// ----------------- Callback: Pump Control -----------------
void pumpCb(const std_msgs::Bool &msg) {
  if (msg.data) {
    digitalWrite(PUMP_PIN, LOW); // Turn pump ON
  } else {
    digitalWrite(PUMP_PIN, HIGH); // Turn pump OFF
  }
}

// ----------------- Motor Control Task -----------------
void motorControlTask(void *pvParameters) {
  for (;;) {
    float v, w;
    portENTER_CRITICAL(&mux);
    v = linear_cm_s;
    w = angular_rad_s;
    portEXIT_CRITICAL(&mux);

    float vR = v + (w * WHEEL_BASE_CM / 2.0);
    float vL = v - (w * WHEEL_BASE_CM / 2.0);
    targetVelocity_R = vR;
    targetVelocity_L = vL;

    // --- Right Wheel PID ---
    long currTimeR = millis();
    float dtR = (currTimeR - prevTimeR) / 1000.0;
    long encR = encoderR.getCount();
    long deltaR = encR - lastCountR;
    float speedR = (deltaR / (PPR * dtR)) * WHEEL_CIRCUMFERENCE_CM;
    measured_R = alpha * speedR + (1 - alpha) * measured_R;

    float errR = vR - measured_R;
    float dedtR = (errR - eprevR) / dtR;
    dFilterR = dAlpha * dedtR + (1 - dAlpha) * dFilterR;
    eintegralR += errR * dtR;
    eintegralR = constrain(eintegralR, integralMin, integralMax);
    float uR = Kp * errR + Ki * eintegralR + Kd * dFilterR + ff * vR;
    float pwmR = constrain(fabs(uR), 0, 255);

    digitalWrite(DIR_PIN1_R, uR > 0);
    digitalWrite(DIR_PIN2_R, uR <= 0);
    analogWrite(PWM_PIN_R, pwmR);

    eprevR = errR;
    prevTimeR = currTimeR;
    lastCountR = encR;

    // --- Left Wheel PID ---
    long currTimeL = millis();
    float dtL = (currTimeL - prevTimeL) / 1000.0;
    long encL = encoderL.getCount();
    long deltaL = encL - lastCountL;
    float speedL = (deltaL / (PPR * dtL)) * WHEEL_CIRCUMFERENCE_CM;
    measured_L = alpha * speedL + (1 - alpha) * measured_L;

    float errL = vL - measured_L;
    float dedtL = (errL - eprevL) / dtL;
    dFilterL = dAlpha * dedtL + (1 - dAlpha) * dFilterL;
    eintegralL += errL * dtL;
    eintegralL = constrain(eintegralL, integralMin, integralMax);
    float uL = Kp * errL + Ki * eintegralL + Kd * dFilterL + ff * vL;
    float pwmL = constrain(fabs(uL), 0, 255);

    digitalWrite(DIR_PIN1_L, uL > 0);
    digitalWrite(DIR_PIN2_L, uL <= 0);
    analogWrite(PWM_PIN_L, pwmL);

    eprevL = errL;
    prevTimeL = currTimeL;
    lastCountL = encL;

    // --- Stuck Check ---
    const float STUCK_THRESHOLD = 0.5; // Minimum encoder delta to consider the motor not stuck
    bool isStuck = (fabs(deltaR) < STUCK_THRESHOLD && fabs(deltaL) < STUCK_THRESHOLD && fabs(linear_cm_s) > 1.0);
    if (isStuck) {
      encoderStuckMsg.data = true;
      encoderStuckPub.publish(&encoderStuckMsg);
      /*delay(4000); // Delay to avoid flooding the topic
      encoderStuckMsg.data = false;
      encoderStuckPub.publish(&encoderStuckMsg);*/
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ----------------- Straight Line Correction -----------------
void lineCorrectionTask(void *pvParameters) {
  for (;;) {
    if (fabs(angular_rad_s) < 0.01 && fabs(linear_cm_s) > 1.0) {
      long deltaL = encoderL.getCount();
      long deltaR = encoderR.getCount();
      long diff = deltaL - deltaR;
      float err = (diff / PPR) * WHEEL_CIRCUMFERENCE_CM;
      float correction = K_line * err;

      float baseV = linear_cm_s;
      float vL = constrain(baseV - correction, -255, 255);
      float vR = constrain(baseV + correction, -255, 255);

      portENTER_CRITICAL(&mux);
      targetVelocity_L = vL;
      targetVelocity_R = vR;
      portEXIT_CRITICAL(&mux);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ----------------- Stuck Detection Task -----------------
/*
void stuckDetectionTask(void *pvParameters) {
  for (;;) {
    // Removed isStuck check from here
    vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
  }
}
*/
// ----------------- LED Blink -----------------
void ledBlinkTask(void *pvParameters) {
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  for (;;) {
    digitalWrite(LED_PIN_1, !digitalRead(LED_PIN_1));
    digitalWrite(LED_PIN_2, !digitalRead(LED_PIN_2));
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ----------------- ROS Task -----------------
void rosTask(void *pvParameters) {
  nh.initNode();
  nh.subscribe(cmdVelSub);
  nh.subscribe(pumpSub);
  nh.advertise(encoderStuckPub); 
  while (true) {
    nh.spinOnce();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);

  // Right wheel
  pinMode(PWM_PIN_R, OUTPUT);
  pinMode(DIR_PIN1_R, OUTPUT);
  pinMode(DIR_PIN2_R, OUTPUT);
  encoderR.attachFullQuad(ENCODER_PIN_A_R, ENCODER_PIN_B_R);
  encoderR.clearCount();

  // Left wheel
  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(DIR_PIN1_L, OUTPUT);
  pinMode(DIR_PIN2_L, OUTPUT);
  encoderL.attachFullQuad(ENCODER_PIN_A_L, ENCODER_PIN_B_L);
  encoderL.clearCount();

  // Pump: initially OFF
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH); 

        
  xTaskCreatePinnedToCore(rosTask, "ROSTask", 2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "MotorControl", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(lineCorrectionTask, "LineCorrection", 2048, NULL, 2, NULL, 0);
  //xTaskCreatePinnedToCore(stuckDetectionTask, "StuckDetection", 2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(ledBlinkTask, "LEDBlink", 1024, NULL, 1, NULL, 0);
}

void loop() {}