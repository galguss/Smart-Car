#include <Servo.h>                
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

unsigned char old_SL,old_SM,old_SR;
unsigned char Bluetooth_val;

// pin list
int servopin = 3;   //defining digital port pin 3, connecting to signal line of servo motor
#define inputPin A0  // ultrasonic module   ECHO to A0
#define outputPin A1  // ultrasonic module  TRIG to A1
#define SensorLeft    6   //input pin of left sensor
#define SensorMiddle  9   //input pin of middle sensor
#define SensorRight   11   //input pin of right sensor
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  10    //pin of controlling speed---- ENA of motor driver board
#define pinLB 2            //pin of controlling diversion----IN1 of motor driver board
#define pinLF 4            //pin of controlling diversion----IN2 of motor driver board
#define pinRB 7            //pin of controlling diversion----IN3 of motor driver board
#define pinRF 8 //pin of controlling diversion----IN4 of motor driver board
#define RECV_PIN   12


unsigned char SL;        //state of left sensor
unsigned char SM;        //state of middle sensor
unsigned char SR;        //state of right sensor
unsigned char Lpwm_val =180;//the speed of left wheel at 180 in initialization
unsigned char Rpwm_val = 180;//the speed of right wheel at 180 in initialization
int Car_state=0;             //state of car moving

void Sensor_IO_Config(){
  pinMode(SensorLeft,INPUT);
  pinMode(SensorMiddle,INPUT);
  pinMode(SensorRight,INPUT);
}

void Sensor_Scan(){
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);

void LCD1602_init(){
  lcd.init();                     
  lcd.backlight(); 
  lcd.clear();  
}

void show_state(){ //showing current state of the car
  lcd.setCursor(0, 1); //showing from second row
   switch(Car_state)
   {
     case 1:lcd.print(" Go  ");Serial.print("\n GO");
     break;
     case 2:lcd.print("Back ");Serial.print("\n Back");
     break;
     case 3:lcd.print("Left ");Serial.print("\n Left");
     break;
     case 4:lcd.print("Right");Serial.print("\n Right");
     break;
     case 5:lcd.print("Stop ");Serial.print("\n Stop"); 
     break;
     default:
     break;
   }
}

void Show_V(unsigned char V){
     lcd.setCursor(11, 0);
     lcd.print("V=    ");
     lcd.setCursor(13, 0);
     lcd.print(V,DEC);
     Serial.print("\n Speed = ");
     Serial.print(V,DEC); 
}

void Show_DuoJiao(unsigned char Jiao){
     lcd.setCursor(6,1);
     lcd.print("C=    ");
     lcd.setCursor(8, 1);
     lcd.print(Jiao,DEC);
     Serial.print("\n JiaoDu = ");
     Serial.print(Jiao,DEC); 
}

//Motors
void M_Control_IO_config(){ //initialized function of IO of motor driver
  pinMode(pinLB,OUTPUT); // pin 2--IN1 of motor driver board
  pinMode(pinLF,OUTPUT); // pin 4--IN2 of motor driver board
  pinMode(pinRB,OUTPUT); // pin 7--IN3 of motor driver board
  pinMode(pinRF,OUTPUT); // pin 8--IN4 of motor driver board
  pinMode(Lpwm_pin,OUTPUT); // pin 5  (PWM) --ENA of motor driver board
  pinMode(Rpwm_pin,OUTPUT); // pin 10 (PWM) --ENB of motor driver board  
}

void Set_Speed(unsigned char Left,unsigned char Right){ //setting function of speed
  analogWrite(Lpwm_pin,Left);   
  analogWrite(Rpwm_pin,Right);
}

void advance(){ // going forwards
  digitalWrite(pinRB,LOW);  // making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);  //  making motor move towards left rear
  digitalWrite(pinLF,HIGH); 
  Car_state = 1; 
  show_state();   
}

void turnR(){ //turning on the right(dual wheels)
  digitalWrite(pinRB,LOW);  //making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);  //making motor move towards left front
  Car_state = 4;
  show_state();
}

void turnL(){ //turning on the left(dual wheels)
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW );   //making motor move towards right front
  digitalWrite(pinLB,LOW);   //making motor move towards left rear
  digitalWrite(pinLF,HIGH);
  Car_state = 3;
  show_state();
}

void back(){ //back
  digitalWrite(pinRB,HIGH);  //making motor move towards right rear
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);  //making motor move towards left rear
  digitalWrite(pinLF,LOW);
  Car_state = 2;
  show_state();    
}

void stopp(){  //stop
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
  Car_state = 5;
  show_state();
}

//Remote Control
IRrecv irrecv(RECV_PIN);
decode_results results;
#define IR_Go      0x00ff629d
#define IR_Back    0x00ffa857
#define IR_Left    0x00ff22dd
#define IR_Right   0x00ffc23d
#define IR_Stop    0x00ff02fd
#define IR_ESC     0x00ff52ad
//Situation management
unsigned long modeSelection;
#define NUM_ONE     0x00ff6897    // Activates marked track mode
#define NUM_TWO     0x00ff9867    // Activates remote control mode
#define NUM_THREE   0x00ffbd4f    // Activates obstacle detection mode
#define NUM_FOUR    0x00ff30cf    // Activates control mode via Bluetooth


void IR_Control(){
   unsigned long Key;
   lcd.setCursor(0,0);  //setting cursor in the first row and column
   lcd.print("IR_Ctr");
  while(Key!=IR_ESC)
  {
   if(irrecv.decode(&results)) //judging if serial port receives data   
 {
     Key = results.value;
    switch(Key)
     {
       case IR_Go:advance();   //UP
       break;
       case IR_Back: back();   //back
       break;
       case IR_Left:turnL();   //Left    
       break;
       case IR_Right:turnR(); //Righ
       break;
       case IR_Stop:stopp();   //stop
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
    } 
  }
  lcd.clear();
  lcd.setCursor(0, 0);  //setting cursor in the first row and column，
  lcd.print("  Wait  Signal  ");
  stopp();
}
 
//The motor that moves the distance sensor

int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao=60;    //initialized angle of motor at 60°

void servopulse(int servopin,int myangle){ //defining a function of pulse
pulsewidth=(myangle*11)+500; //converting angle into pulse width value at 500-2480 
digitalWrite(servopin,HIGH); //increasing the level of motor interface to upmost
delayMicroseconds(pulsewidth); //delaying microsecond of pulse width value
digitalWrite(servopin,LOW); //decreasing the level of motor interface to the least
delay(20-pulsewidth/1000);
}

void Set_servopulse(int set_val){
 for(int i=0;i<=10;i++)  //giving motor enough time to turn to assigning point
   servopulse(servopin,set_val); //invokimg pulse function
}



//distance sensor 

void Self_Control(){ //self-going, ultrasonic obstacle avoidance
   int H;
   lcd.setCursor(0, 0);  //setting cursor in the first row and column
   lcd.print("Self_Ctr        ");
   Show_V(Lpwm_val);
   Set_servopulse(DuoJiao);
   Show_DuoJiao(DuoJiao); 
   H = Ultrasonic_Ranging(1);
   delay(300);   
   if(Ultrasonic_Ranging(1) < 35)         
   {
       stopp();              
       delay(100);
       back();               
       delay(50);
    }
           
  if(Ultrasonic_Ranging(1) < 60)        
      {
        stopp();  
        delay(100);            
        Set_servopulse(5);
        Show_DuoJiao(5);
        int L = ask_pin_L(2);
        delay(300);      
         Set_servopulse(177);
         Show_DuoJiao(177);
        int R = ask_pin_R(3);
        delay(300);      

        if(ask_pin_L(2) > ask_pin_R(3))   
        {
         back(); 
        delay(100);      
        turnL();
       delay(400);                  
       stopp();  
       delay(50);            
       Set_servopulse(DuoJiao);
       Show_DuoJiao(DuoJiao); 
       H = Ultrasonic_Ranging(1);
       delay(500); 
        }
        
      if(ask_pin_L(2)  <= ask_pin_R(3))   
      {
       back();  
       delay(100);  
       turnR(); 
       delay(400);   
       stopp();  
       delay(50);            
       Set_servopulse(DuoJiao);
       Show_DuoJiao(DuoJiao); 
       H = Ultrasonic_Ranging(1);
       delay(300);        
        }   
        if (ask_pin_L(2)  < 35 && ask_pin_R(3)< 35)   
        {
       stopp();            
       delay(50);
       back(); 
       delay(50);                   
        }          
      }
      else                      
      {
      advance();                
      }                 
}

int Ultrasonic_Ranging(unsigned char Mode){ //function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situationn
  int old_distance;
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  int distance = pulseIn(inputPin, HIGH);  // reading the duration of high level
  distance= distance/58;   // Transform pulse time to distance   
  if(Mode==1){
         lcd.setCursor(11, 1);
         lcd.print("H=    ");
         lcd.setCursor(13, 1);
         lcd.print(distance,DEC);
         Serial.print("\n H = ");
         Serial.print(distance,DEC); 
         return distance;
  }
   else  return distance;
} 

int ask_pin_L(unsigned char Mode){ 
  int old_Ldistance;
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  int Ldistance = pulseIn(inputPin, HIGH); 
  Ldistance= Ldistance/58;   // Transform pulse time to distance   
  if(Mode==2){
         lcd.setCursor(11, 1);
         lcd.print("L=    ");
         lcd.setCursor(13, 1);
         lcd.print(Ldistance,DEC);
         Serial.print("\n L = ");
         Serial.print(Ldistance,DEC); 
         return Ldistance;
  }
   else  return Ldistance;
} 

int ask_pin_R(unsigned char Mode){ 
  int old_Rdistance;
  digitalWrite(outputPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(outputPin, HIGH); // 
  delayMicroseconds(10); 
  digitalWrite(outputPin, LOW);    
  int Rdistance = pulseIn(inputPin, HIGH); 
  Rdistance= Rdistance/58;   // Transform pulse time to distance   
  if(Mode==3){
         lcd.setCursor(11, 1);
         lcd.print("R=    ");
         lcd.setCursor(13, 1);
         lcd.print(Rdistance,DEC);
         Serial.print("\n R = ");
         Serial.print(Rdistance,DEC); 
         return Rdistance;
  }
   else  return Rdistance;
} 



//start for loop
void routeStatusMarked(){      // 12 Line Tracking Car
    Sensor_Scan();
   if (SM == HIGH)// middle sensor in black area
  {
  if (SL == LOW & SR == HIGH) // black on left, white on right, turn left
  {
     turnR();
  }
  else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
  {
     turnL();
  }
  else // white on both sides, going forward
    {
       advance();
    }
  }
  else // middle sensor on white area
  {
    if (SL== LOW & SR == HIGH)// black on left, white on right, turn left
    {
      turnR();
    }
    else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
    {
      turnL();
    }
    else // all white, stop
    {
      back();
      delay(100);
      stopp() ; 
    }
  }
}
void ThroughRemoteControl(){   // 13 Remote Control Car
  if (irrecv.decode(&results)) {
      if(results.value == IR_Stop )IR_Control();
      irrecv.resume(); // Receive the next value   
  } 
}
void distanceControl(){        // 14 Obstacle_Avoidance_Car
  Self_Control();  
}
void BluetoothControl(){       // 15 Bluetooth_Remote_Control_Car
  lcd.setCursor(0, 0);  //setting cursor in the first row and column
   lcd.print("BluetoothControl");
 
   if(Serial.available()) //to judge whether the serial port receives the data.
    {
     Bluetooth_val=Serial.read();  //reading (Bluetooth) data of serial port,giving the value of val;
    switch(Bluetooth_val)
     {
       case 'U':advance(); //UP
       break;
       case 'D': back();   //back
       break;
       case 'L':turnL();   //Left
       break;
       case 'R':turnR();  //Right
       break;
       case 'S':stopp();    //stop
       break;   
     }
    } 
}

void setup() {
   Serial.begin(9600);   //initializing serial port, Bluetooth used as serial port, setting baud ratio at 9600 
   pinMode(servopin,OUTPUT); //setting motor interface as output
   pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
   pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
   LCD1602_init();
   Sensor_IO_Config();
   M_Control_IO_config();        //motor controlling the initialization of IO
   Set_Speed(Lpwm_val,Rpwm_val); //setting initialization of speed
   irrecv.enableIRIn(); // Start the receiver
   lcd.clear();
   lcd.setCursor(0, 0);  //cursor set in first row and first column，
   lcd.print("  Wait  Signal  ");
   stopp(); 
}


void loop() {
   if (irrecv.decode(&results)) {
        modeSelection = results.value;
        switch(modeSelection)
        {
          case NUM_ONE:
              routeStatusMarked();
          break;
          case  NUM_TWO:
              ThroughRemoteControl();
          break;
          case NUM_THREE:
            distanceControl();
          break;
          case NUM_FOUR:
            BluetoothControl();
          break;
          default: 
           break; 
        }
      irrecv.resume(); // Receive the next value
  }   
}
