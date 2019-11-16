#define PWM 11
#define INA 6
#define INB 4
int mot_speed = 255;
int rotDirection = 0;
int pwm_speed = 50;

void setup() {
  //pinMode(PWM, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

Serial.begin(9600);
  
  // Set initial rotation direction
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
  int pwm_speed = map (mot_speed,0,100,0,255); 
}
void loop() {
  // put your main code here, to run repeatedly:
    analogWrite(PWM, mot_speed);
    Serial.println(pwm_speed);
}
