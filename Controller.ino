  #define PWM 11
  #define INA 6
  #define INB 4

  #define outputA 3
  #define outputB 2

  int INA_State;
  int INB_State;
  int16_t i;

  volatile int counter = 0;

  // Define controller variables
  int16_t set_pt;  // Desired angular position
  int16_t pos;  // Measured encoder angular position
  int16_t error = 0;  // Difference between desired angular position and actual angular position
  int16_t e_dif = 0;  // Time derivative of error, used for differential control
  int16_t old_error = 0; // Calculated error from the last controller loop
  int16_t e_sum = 0; // Error summation
  double Kp = 0.3;  // Controller gains
  double Kd = 0.5;
  double Ki = 0.0015;
  int16_t Pout = 0;  // Proportional control output
  int16_t Dout = 0;  // Derivative control output
  int16_t Iout = 0;  // Integral control output
  int16_t act = 0;  // actuation signal to motor
  int8_t dt = 1; // in milliseconds
  int16_t max_act = 100; // Set this based on testing
  int16_t min_act = 0; // Set this based on testing

void setup() {
  
  pinMode(INA, OUTPUT);  // UNO pins that control motor rotation direction
  pinMode(INB, OUTPUT);

  pinMode(outputA, INPUT);      // Encoders A and B
  pinMode(outputB, INPUT);

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(outputA), A_Rise, RISING);  // Generate interrupts when encoder signals rise from low to high
  attachInterrupt(digitalPinToInterrupt(outputB), B_Rise, RISING);
}

void loop() {

  // Reset all control loop variables
  i = 0; 
  e_sum = 0;
  old_error = 0;
  
  set_pt = random(-2500, 2500); // Choose a random setpoint each loop
  //set_pt = 800;           

  for (i=0; i<1000; i++) // Run the controller for 5 seconds
    { 
      pos = counter; // Determine current motor angular position 
      
      error = set_pt - pos;  // obtain positional error by comparing to desired angular position
      
      e_dif = (error - old_error)/dt;  // Set up Differential parameters: How the error is changing per run 
      old_error = error;  // Store error for next loop
    
      e_sum = e_sum + error*dt;  // Set up integral parameters: The sum of the error throughout the program
      
      Pout = Kp*error;  // Calculate proportional control
    
      Dout = Kd*e_dif;  // Calculate differential control
    
      Iout = Ki*e_sum;  // Calculate integral control
    
      act = Pout+Dout+Iout; // Calculate actuation magnitude (PWM)

      // Set rotation direction and negate act if necessary
      rot_direction();
      
      // Saturate the actuation signal
      if (act>max_act){
        act = max_act;
      }
      else if(act<min_act){
        act = min_act;
      }
      
      analogWrite(PWM, act); // Output control signal to motor PWM
      Serial.print(set_pt);
      Serial.print(", ");
      Serial.print(counter);
      Serial.println(" ");
      delay(dt); //sets function to run once, per ms
    }
}

// Truth table for encoder signals
// A rising and B low = CW motion (-)
// A rising and B high = CCW motion (+)
// B rising and A low = CCW motion (+)
// B rising and A high = CW motion (-)

void A_Rise() { // Interrupt Service Routine for when encoder A reading transitions from low to high (Rising edge)
  if (digitalRead(outputB)){  // If encoder B is high (and A is rising), the motor is moving Counter Clock Wise
    counter ++; // CCW = increment
  }
  else{
    counter --; // CW = decrement
  }
}

void B_Rise() { // Interrupt Service Routine for when encoder B reading transitions from low to high (Rising edge)
  if (digitalRead(outputA)){  // If encoder A is high (and B is rising), the motor is moving Clock Wise
    counter --; // CW = decrement
  }
  else{
    counter ++; // CCW = increment
  }
}

void rot_direction(){
  
  if (act<=0){
    INA_State = HIGH;
    INB_State = LOW;
    act *= -1;
  }
  else if(act>0){
    INA_State = LOW;
    INB_State = HIGH;
  }
  
  digitalWrite(INA, INA_State);
  digitalWrite(INB, INB_State);
}
