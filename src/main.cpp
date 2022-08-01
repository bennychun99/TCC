// DC Motor wiring:
// Red Wire - positive power supply of motor(+)(change positive and negative of motor the rotation will change)
// White Wire - negative power supply of motor(-)(change positive and negative of motor the rotation will change))
// Yellow Wire - signal feedback (motor one turn has 11 signals)
// Green Wire - signal feedback (motor one turn has 11 signals)
// Blue Wire - positive of encoder power supply(+)(3.3-5V),cannot be wrong
// Black Wire - negative of encoder power supply(-)(3.3-5V),cannot be wrong

#include <Arduino.h>

#define ENC_COUNTS_REVOLUTION 11
#define GEARBOX_REDUCTION 46.8
// Motor A
#define ENA 14
#define IN1 27
#define IN2 26
// Encoder A
#define ENCA_A 23
#define ENCA_B 22
// 1 = Forward; -1 = Reverse; 0 = Stop
int direction_A = 0;
// Keep track of the number of ENCA pulses
volatile long ENCA_pulse_count = 0;
// Variable for RPM measuerment
float previousENCA_rpm = 0;
float ENCA_rpm = 0;
// Interval for measurements
int interval = 1000;
// Counters for microseconds during interval
long previousMicros = 0;
long currentMicros = 0;
// Variable for angular velocity measurement
float ENCA_ang_velocity = 0;
float ENCA_ang_velocity_deg = 0;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm, pwmVal);
  if (dir == 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Increment the number of pulses by 1
void increment_pulse()
{

  // Reads the value from encoder
  int val = digitalRead(ENCA_B);

  if (val == LOW)
  {
    direction_A = false; // Reverse
  }
  else
  {
    direction_A = true; // Forward
  }

  if (direction_A)
  {
    ENCA_pulse_count++;
  }
  else
  {
    ENCA_pulse_count--;
  }
}

void read_encoder()
{
  // If interval has passed, print the number of pulses
  if (currentMicros - previousMicros > interval)
  {
    previousMicros = currentMicros;

    // Calculate revolutions per minute
    previousENCA_rpm = ENCA_rpm;

    ENCA_rpm = (float)(ENCA_pulse_count * 60 / ENC_COUNTS_REVOLUTION);

    // Serial.print(" Speed(RPM): ");
    // Serial.print(ENCA_rpm);
    // Serial.print(", Mean: ");
    // Serial.print((float)((ENCA_rpm + previousENCA_rpm) / 2));
    // Serial.println();

    ENCA_pulse_count = 0;
  }
}
void setup()
{
  Serial.begin(9600);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCA_A, INPUT);
  pinMode(ENCA_B, INPUT);

  // Set pin states of the encoder
  pinMode(ENCA_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENCA_A), increment_pulse, RISING);
}

void loop()
{
  // Setpoint (onda senoidal)
  float vt = 100 * (float)abs(sin(micros() / 1e6));
  float kp = 100;
  float e = vt - ENCA_rpm;
  float u = kp * e; // controller

  // Applies controller
  int dir = -1;
  if (u < 0)
  {
    dir = 1;
  }
  int pwmVal = (int)fabs(u);
  if (pwmVal > 255)
  {
    pwmVal = 255;
  }
  setMotor(1, 255, ENA, IN1, IN2);
  // Record the time
  currentMicros = micros();
  read_encoder();

  Serial.print("Setpoint:");
  Serial.print(vt);
  Serial.print(",");
  Serial.print("RPM:");
  Serial.println(ENCA_rpm);
}
