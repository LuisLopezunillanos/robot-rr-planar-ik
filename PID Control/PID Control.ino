/*
* POSITION CONTROL: 2 DC MOTORS (PID) + 1 SG90 SERVO
* ESP32 - INTEGRATION CORRECTED
*/

#include <ESP32Servo.h>

// --- PIN CONFIGURATION ---

// MOTOR 1
#define M1_ENC_A 4
#define M1_ENC_B 5
#define M1_IN1   18
#define M1_IN2   19
#define M1_PWM   21

// MOTOR 2
#define M2_ENC_A 32
#define M2_ENC_B 33
#define M2_IN1   25
#define M2_IN2   26
#define M2_PWM   27

// SERVO
static const int servoPin = 13; // Tu pin de confianza

// --- GENERAL PARAMETERS ---
#define PPR 1650.0            
const double DEGREES_PER_PULSE = 360.0 / PPR;
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
const int PWM_MIN_MOVING = 150;
const double TOLERANCE_DEGREES = 1.0;
const double Ts = 0.02; // 20ms

// --- GLOBAL VARIABLES ---
volatile long pulseCount1 = 0;
volatile long pulseCount2 = 0;

Servo miServo; // Servo Object

// --- INTERRUPT ROUTINES (ISRs) ---
void IRAM_ATTR readEncoder1() {
  if (digitalRead(M1_ENC_B) == LOW) pulseCount1++;
  else pulseCount1--;
}

void IRAM_ATTR readEncoder2() {
  if (digitalRead(M2_ENC_B) == LOW) pulseCount2++;
  else pulseCount2--;
}

// ==========================================
// PID CONTROLLER CLASS
// ==========================================
class MotorPID {
  private:
    int pinIN1, pinIN2, pinPWM;
    volatile long* encoderCountPtr; 
    
    double Kp, Ki, Kd;
    double integralSum = 0.0;
    double lastError = 0.0;
    double targetAngle = 0.0;
    double currentAngle = 0.0;
    
  public:
    MotorPID(int in1, int in2, int pwm, volatile long* encCount, double _Kp, double _Ki, double _Kd) {
      pinIN1 = in1;
      pinIN2 = in2;
      pinPWM = pwm;
      encoderCountPtr = encCount;
      Kp = _Kp; Ki = _Ki; Kd = _Kd;
    }

    void begin() {
      pinMode(pinIN1, OUTPUT);
      pinMode(pinIN2, OUTPUT);
      // Configure PWM for DC motors
      ledcAttach(pinPWM, PWM_FREQ, PWM_RESOLUTION);
      stopMotor();
    }

    void setTarget(double angle) {
      targetAngle = angle;
      integralSum = 0.0; 
      lastError = 0.0;
    }

    double getCurrentAngle() { return currentAngle; }
    double getTargetAngle() { return targetAngle; }

    void update() {
      noInterrupts();
      long pulses = *encoderCountPtr;
      interrupts();
      currentAngle = (double)pulses * DEGREES_PER_PULSE;

      double error = targetAngle - currentAngle;
      double abs_error = abs(error);
      double pid_output = 0.0;

      if (abs_error <= TOLERANCE_DEGREES) {
        stopMotor();
        integralSum = 0.0;
      } else {
        // PID
        double P_term = Kp * error;
        double I_term_step = (Ki * Ts / 2.0) * (error + lastError);
        double D_term = Kd * (error - lastError) / Ts;

        pid_output = P_term + (integralSum + I_term_step) + D_term;

        // Anti-Windup
        double abs_output = abs(pid_output);
        if (abs_output >= PWM_MAX) {
          if ( (error > 0 && I_term_step < 0) || (error < 0 && I_term_step > 0) ) {
             integralSum += I_term_step;
          }
        } else {
          integralSum += I_term_step;
        }

        int pwm_signal = (int)constrain(abs(pid_output), 0, PWM_MAX);
        if (pwm_signal > 0) {
          pwm_signal = constrain(pwm_signal, PWM_MIN_MOVING, PWM_MAX);
        }
        driveMotor(pid_output, pwm_signal);
      }
      lastError = error;
    }

  private:
    void driveMotor(double pid_out, int pwm_val) {
      ledcWrite(pinPWM, pwm_val);
      if (pid_out > 0) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
      } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
      }
    }

    void stopMotor() {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      ledcWrite(pinPWM, 0);
    }
};

// ==========================================
// GLOBAL INSTANCES
// ==========================================
MotorPID motor1(M1_IN1, M1_IN2, M1_PWM, &pulseCount1, 2.84, 2.7, 0.9);
MotorPID motor2(M2_IN1, M2_IN2, M2_PWM, &pulseCount2, 3.67, 2.05, 0.5); 

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

// --- SERVO CONFIGURATION ---
// Reserve the timer before anything else related to PWM
  
  ESP32PWM::allocateTimer(0);
  miServo.setPeriodHertz(50); 
  miServo.attach(servoPin, 100, 1000); 
  
 // Initial position of the servo
  miServo.write(90);

// --- MOTOR AND ENCODER CONFIGURATION ---
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), readEncoder2, RISING);

  motor1.begin();
  motor2.begin();

  Serial.println("SISTEMA LISTO. Formato: ANG1,ANG2,SERVO (Ej: 48,67,90)");
}

// ==========================================
// LOOP
// ==========================================
void loop() {
// --- SERIAL READING (M1, M2, SERVO) ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 

    // We are looking for the two commas
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      // We parse the 3 values
      String strM1 = input.substring(0, firstComma);
      String strM2 = input.substring(firstComma + 1, secondComma);
      String strServo = input.substring(secondComma + 1);
      
      double targetM1 = strM1.toDouble();
      double targetM2 = strM2.toDouble();
      int targetServo = strServo.toInt();

      // 1. Assign Targets to PID Engines
      motor1.setTarget(targetM1);
      motor2.setTarget(targetM2);

      // 2. Move Servo immediately
      targetServo = constrain(targetServo, 0, 180); // Segurity
      miServo.write(targetServo);

      // (Optional) Visual confirmation only upon receiving command
      // Serial.print("CMD RECEIVED -> Servo: "); Serial.println(targetServo);
    }
  }

  // --- PID CONTROL (50Hz) AND SERIAL PLOTTER ---
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  
  if (currentMillis - previousMillis >= 20) { 
    previousMillis = currentMillis;
    
    // Update PID calculations
    motor1.update();
    motor2.update();

    // --- SERIAL OUTPUT (Identical to the original for your graphics card) ---
    Serial.print(motor1.getTargetAngle()); Serial.print(",");
    Serial.print(motor1.getCurrentAngle()); Serial.print(",");
    Serial.print(motor2.getTargetAngle()); Serial.print(",");
    Serial.println(motor2.getCurrentAngle());
    
  }
}
