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
  int16_t pos;  // Measured encoder angular position
  int16_t error = 0;  // Difference between desired angular position and actual angular position
  int16_t e_dif = 0;  // Time derivative of error, used for differential control
  int16_t old_error = 0; // Calculated error from the last controller loop
  int16_t e_sum = 0; // Error summation
  int16_t Pout = 0;  // Proportional control output
  int16_t Dout = 0;  // Derivative control output
  int16_t Iout = 0;  // Integral control output
  int16_t act = 0;  // actuation signal to motor
  int8_t dt = 0.001; // in seconds
  int16_t max_act = 5; // Set this based on testing
  int16_t min_act = 0; // Set this based on testing
  
  for (i=0; i<5000; i++) // Run the controller for 5 seconds
  {
    /* pos = (__insert encoder reader function to obtain initial position__); // Determine current motor angular position */
    
    error = set_pt - pos;  // obtain positional error by comparing to desired angular position
    
    e_dif = (error - old_error)/dt;  // Set up Differential parameters: How the error is changing per run 
    old_error = error;  // Store error for next loop
  
    e_sum = e_sum + error*dt;  // Set up integral parameters: The sum of the error throughout the program
    
    Pout = Kp*error;  // Calculate proportional control
  
    Dout = Kd*e_dif;  // Calculate differential control
  
    Iout = Ki*e_sum;  // Calculate integral control
  
    act = Pout+Dout+Iout;

    // Saturate the actuation signal
    if (act>max_act){
      act = max_act;
    }
    else if(act<min_act){
      act = min_act;
    }
    
    analogWrite(PWM, act); // Assuming this is the output control signal to motor PWM
    delay(dt*1000); //sets function to run once, per ms
  }  
}


  

