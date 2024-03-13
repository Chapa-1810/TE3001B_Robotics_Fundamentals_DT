#define LED_PIN 12
#define PWM_PIN 27
#define ENCODER_PIN 32
#define IN1 25
#define IN2 26

#define FREQUENCY 5000
#define MOTOR_CHANNEL 0
#define RESOLUTION 8

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), updateEncoder, RISING);
  ledcSetup(MOTOR_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(PWM_PIN, MOTOR_CHANNEL);
  ledcWrite(MOTOR_CHANNEL, 0);
  Serial.begin(115200);
}