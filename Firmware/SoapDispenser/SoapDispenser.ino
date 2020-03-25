#include <Servo.h>
#include <Ultrasonic.h>

//#define DEBUG_MODE 1

#define SERVO_OUTPUT        2
#define ULTRASONIC_TRIGGER  12
#define ULTRASONIC_ECHO     13
#define LED_A               6
#define LED_B               7

#define SERVO_MIN 0   //IRM Initial servo degrees
#define SERVO_MAX 37  //IRM Max servo rotation degrees

#define DELAY_SERVO 300 //IRM pause between servo movements (SERVO_MIN, SERVO_MAX)


//IRM pause (miliseconds) between gesture detection
//Avoids false positive triggers
#define DELAY_BETWEEN_GESTURES 500

//IRM how many times a gesture is checked before triggering an event
#define GESTURE_DETECTION_ATTEMPTS 5

//IRM delay (miliseconds) between gesture detection events 
#define DELAY_BETWEEN_DETECTION_ATTEMPTS 150  



//IRM range within a gesture is detected
#define MIN_GESTURE_DIST 6
#define MAX_GESTURE_DIST 30

//IRM how many times the soap should be poured onto the hands
#define POURING_TIMES 3


/*
================================
IRM Global variables & instances
================================
*/
Ultrasonic distSensor(ULTRASONIC_TRIGGER, ULTRASONIC_ECHO);
Servo servoMotor;

/*
=======================
IRM function prototypes
=======================
*/
void ioSetup(void);
void mainLoop(void);

unsigned int computeDistance(void);
unsigned int detectGesture(void);
unsigned int withinRange(unsigned int dist, unsigned int minDist, unsigned int maxDist);
void pourSoap(void);
void dispenseSoap(unsigned int times);


/*
=======================
IRM I/O Initialization
=======================
*/
void ioSetup(void){
  #ifdef DEBUG_MODE
    Serial.begin(115200); //IRM Init serial port
  #endif
  
  //IRM Initializing IOs
  distSensor.init();
  
  servoMotor.attach(SERVO_OUTPUT);
  servoMotor.write(SERVO_MIN);

  pinMode(LED_A, OUTPUT);
  digitalWrite(LED_A, 0);

  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, 0); 
}


/*
===================================================================
IRM measure distance using ultrasonic sensor and return value in cm
*** THIS IS A BLOCKING FUNCTION *** Use it carefully
===================================================================
*/
unsigned int computeDistance(void){
  unsigned int d;
  d = distSensor.distance(1); //IRM Mode 1 returns distance in cm

  #ifdef DEBUG_MODE
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm.");
  #endif

  return(d);
}

/*
===================================================================
IRM Detects whether a gesture has been triggered by the user or not
*** THIS IS A BLOCKING FUNCTION *** as it's based on computeDistance
===================================================================
*/
unsigned int detectGesture(void){
  unsigned int event = 0;
  unsigned int dist = 0;
  unsigned int inRange = 0;

  unsigned int i;

  delay(DELAY_BETWEEN_GESTURES); //IRM Avoids multiple triggers within a short time range
  
  while(event < 1){ //IRM keeps waiting until a gesture is detected
    digitalWrite(LED_A, 0); //IRM no individual gestures detected yet
    digitalWrite(LED_B, 0);
    //IRM measure "GESTURE_DETECTION_ATTEMPTS" times while a valid range is detected
    //IRM if an invalid range is detected, no trigger is sent. Otherwise, a (int)1 is returned
    dist = computeDistance();
    inRange = withinRange(dist, MIN_GESTURE_DIST, MAX_GESTURE_DIST);
    i = GESTURE_DETECTION_ATTEMPTS;
    while((inRange) && (i)){
      dist = computeDistance();
      inRange = withinRange(dist, MIN_GESTURE_DIST, MAX_GESTURE_DIST);
      if(inRange){
        i--;
      }
      digitalWrite(LED_A, 1); //IRM Warn the user a gesture is being detected
      #ifdef DEBUG_MODE
        Serial.print("i =");
        Serial.println(i);
      #endif
      delay(DELAY_BETWEEN_DETECTION_ATTEMPTS);
    }

    if(i < 1){ //IRM if the prior loop ran the expected times, a valid event is detected
      event = 1;   
      #ifdef DEBUG_MODE
        Serial.println("Event!");
      #endif      
      digitalWrite(LED_B, 1); //IRM A valid event detected. Pouring soap soon.   
    }
    
  }
  return(event);
}


/*
===================================================================
IRM Determines whether a distance value falls within a valid range
Distances are including lower and upper bounds
===================================================================
*/
unsigned int withinRange(unsigned int dist, unsigned int minDist, unsigned int maxDist){
  if((dist >= minDist) && (dist <= maxDist)){
    return(1);
  }
  return(0);
}


/*
===================================================================
IRM Moves the servo from the starting position to the final position
and the other way around. It's done only once.
===================================================================
*/
void pourSoap(void){

  #ifdef DEBUG_MODE
    Serial.println("Pouring soap!");
  #endif
  
  servoMotor.write(SERVO_MIN);
  delay(DELAY_SERVO);
  servoMotor.write(SERVO_MAX);
  delay(DELAY_SERVO);
  servoMotor.write(SERVO_MIN);
}


/*
===================================================================
IRM Executes soap pouring procedure "times" times
===================================================================
*/
void dispenseSoap(unsigned int times){
  int i = 0;
  for(i = 0; i < times; i++){
    pourSoap();
  }
}

/*
===================================================================
IRM Main loop executes individual procedures as whole algorithm
===================================================================
*/
void mainLoop(void){
  if(detectGesture())
    dispenseSoap(POURING_TIMES);
}


/*
==============================
IRM ARDUINO required functions
==============================
*/
void setup() {
  ioSetup();
}

void loop() {
  mainLoop();
}
