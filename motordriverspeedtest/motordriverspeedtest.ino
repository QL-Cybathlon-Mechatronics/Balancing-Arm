void setup() {
  #define PWM 11
  #define INA 6
  #define INB 4
  
  //pinMode(PWM, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  int mot_speed = 255;
  int rotDirection = 0;
  int pwm_speed = 50;

  Serial.begin(9600);
  
  // Set initial rotation direction
  // set_pt = (encoder reader function) // manually hold the link vertically at balanced point to obtain desired setpoint
  // Serial.println(set_pt);            // display desired encoder position
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
  int pwm_speed = map (mot_speed,0,100,0,255); //Ask software team, how will the motor know to change spin direction? 
}

void loop() {
  // put your main code here, to run repeatedly:
    analogWrite(PWM, mot_speed);
    Serial.println(pwm_speed);
    // write code that prompts user for desired values of Kp, Ki, Kd, and a new setpoint (in Deg)
    // need a function to relate degrees to changes in encoder values
    
    PID_control(5, 0, 0, set_pt); //Kp, Ki, Kd values are temp. subject to user change
    
}

void PID_control(Kp, Ki, Kd, set_pt){
  // input desired PID constants and setpoint
  
  // Define controller variables
  int16_t pos = 0;
  int16_t error = 0;
  int16_t dif = 0;
  int16_t old_error = 0;
  int16_t e_sum = 0;
  int16_t prop =0;
  int16_t diff =0;
  int16_t integral =0;
  int16_t act =0;
  
  for (i=0; i<5000; i++) // Run the controller for 5 seconds
  {
    /* pos = (__insert encoder reader function to obtain initial position__); // Determine current motor angular position */
    
    error = set_pt - pos;  // obtain positional error by comparing to desired angular position
    
    e_dif = (error - old_error)/0.1;  // Set up Differential parameters: How the error is changing per run 
    old_error = error;  // Store error for next loop
  
    e_sum = e_sum + error;  // Set up integral parameters: The sum of the error throughout the program
    
    prop = Kp*error;  // Calculate proportional control
  
    diff = Kd*e_dif;  // Calculate differential control
  
    integral = Ki*e_sum;  // Calculate integral control
  
    act = prop+diff+integral;
    
    analogWrite(PWM, act); // Assuming this is the output control signal to motor PWM
    delay(1); //sets function to run once, per ms
  }  
}


  

