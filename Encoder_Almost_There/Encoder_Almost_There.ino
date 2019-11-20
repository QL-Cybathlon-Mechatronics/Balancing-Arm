  #define PWM 11
  #define INA 6
  #define INB 4

  #define outputA 3
  #define outputB 5
  
  int mot_speed = 255;
  int rotDirection = 0;
  int pwm_speed =20;

  int counter = 0;

  int shake;
  int bake;
  int aState;
  int revsCount = 0;
  int ana_Speed = map (pwm_speed,0,100,0,255); //Ask software team, how will the motor know to change spin direction? 

void setup() {

  
  //pinMode(PWM, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  pinMode(outputA, INPUT);      // Encoders A and B
  pinMode(outputB, INPUT);

  Serial.begin(9600);
  
  // Set initial rotation direction
  // set_pt = (encoder reader function) // manually hold the link vertically at balanced point to obtain desired setpoint
  // Serial.println(set_pt);            // display desired encoder position
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
 
  shake = digitalRead(outputA);
 // bake = digitalRead(outputB);
    analogWrite(PWM, 20);
}

void loop() {
  // put your main code here, to run repeatedly:
 


if (revsCount == 5) {
  //Serial.println(revsCount);
  analogWrite(PWM, 0);
  Serial.println("Motor 1 rev");
  Serial.println(revsCount);
}
//Serial.println(revsCount);

    // write code that prompts user for desired values of Kp, Ki, Kd, and a new setpoint (in Deg)
    // need a function to relate degrees to changes in encoder values
    
  // PID_control(5, 0, 0, set_pt); //Kp, Ki, Kd values are temp. subject to user change


 /*  aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != shake){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");
     Serial.println(counter);
   } 

   shake = aState;

*/
   attachInterrupt(digitalPinToInterrupt(outputA), blink, RISING);  
}

void blink() {
  counter++;
  //Serial.println(counter);

     if(counter == 399){
      revsCount++;
      counter = 0;   
   }
}
