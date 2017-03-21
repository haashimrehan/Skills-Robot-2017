#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>
#include <TPixy.h>
#include <Servo.h>


//Driving
int slowR = 92.33; // Slow Forward Values
int slowL = 88.3;
int forwardR = 88.33; //forward
int forwardL = 91.9;  //forward
int reverseR = 100; //reverse
int reverseL = 79; //reverse
int stopL = 84.99999618811;   //stop
int stopR = 96;   //stop

//Camera
Pixy pixy;
Block blocks[10];
int midPos;
boolean oneBlock;
float mid = 0, sum = 0;
int high = 117 ; //Mid Range
int low = 109; // Mid Range

//Ultrasonic Sensor
const int trigPin = 2; //pins
const int echoPin = 7;
long dist = 0L;
int timing = 20;
int exactDist;

//Line Sensor
int sensorL = A2; // The line sensors are connected to A0, A1 and A2
int sensorC = A1;
int sensorR = A0;
int colour = 800; //middle of black and white
int Lval;
int Cval;
int Rval;

//State Controller
int state = 5; // runs through different states for each part of the field  (set to 1 to enable)


class Drive {

    //  /http://arduino.stackexchange.com/questions/1321/servo-wont-stop-rotating
    int leftServoPin;
    int rightServoPin;
    int armPin;
    Servo leftServo;
    Servo rightServo;
    Servo armServo;

  public:
    Drive (int leftServo, int rightServo, int armServo) {
      leftServoPin = 9;//leftServo;
      rightServoPin = 5; //rightServo;
      armPin = 10;//armServo;
    }

    void attachServos() {
      //initialzes Servos
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      armServo.attach (armPin);
      //Put Servos back in the middle
      leftServo.write (86.3);//86.3
      rightServo.write (96);//96
      armServo.write (90);
      //Serial.println ("LeftServo");
      Serial.print (leftServo.read());
    }

    //Driving
    void setLeft (int speed) {
      leftServo.write (speed);
    }
    void setRight (int speed) {
      rightServo.write (speed);
    }

    void stopServos() {  //Full Stop
      setLeft(84.99999618811); //84.99999618811
      setRight(95.999); //95.999
    }

    void driveStraight () {//forward Value; Turns Very Slightly Right
      setRight (87.33);
      setLeft(94); //92.3
    }
    void driveSlow () { //Slow Forward Value
      setLeft(88.99999);
      setRight(92.3);
    }

    void driveFast() {
      setLeft(95);
      setRight(85);
      }

    void reverse() {
      setRight(100); //reverse
      setLeft(79);
    }

    void turnLeft () { //Turn Left
      setRight (92);//93
      setLeft (81.99999618811);//turn left 82
    }

    void turnRight() { //Turn Right
      setRight (98);// turn right.    old value = 97
      setLeft (87.99999618811);     //old value =86
    }

    //Arm Servo
    void drop () { //Opens Claw
      armServo.write (130);
      // Serial.println(armServo.read());
      delay(2000);
    }
    void grab () {//Closes Claw
      armServo.write (10);
      //Serial.println (armServo.read());
      delay(2000);
    }

    //Line Follower
    void sensorRead () { //needs to be enabled for Line Follower
      Lval = analogRead (sensorL);
      Cval = analogRead (sensorC);
      Rval = analogRead (sensorR);
    }

    void lineFollower() { //follow lines, needs sensorRead(); to be enabled
      //    if ( Lval > colour && Cval > colour &&  Rval > colour) { // if detects all black

      if ( Lval < colour && Cval > colour &&  Rval < colour)  { //goes straight when sees line in center
        setLeft(slowL);
        setRight(slowR);
        if ( Lval < colour && Cval < colour &&  Rval < colour) { // if detects White
          stopServos();
          grab();
        }
      } else if ( Lval > colour && Cval < colour &&  Rval < colour) { //turns right when line is to the left
        setRight(forwardR);
        setLeft(reverseL);
        delay(200);
      } else if ( Lval < colour && Cval < colour && Rval > colour) { //turns left when the line is to far to the right
        setLeft(forwardL);
        setRight(reverseR);
        delay(200);
      }
    }

    //States
    void straight() { // what to do when through defenders (state 2)
      stopServos(); //testing right now
      grab();
      drop();
    }

    void getThrough() {  // get through defenders (state 1)
      if (blocks) { // if camera sees blocks
        driveStraight(); //drive straight
        delay (2800);
        stopServos();
        if (mid != 53 || mid != 52 || mid != 54) {

        } else if (mid > 52 && mid < 54)
          driveStraight();
      } else {
        stopServos();
      }
      /* if (mid < low) {
         setLeft(forwardL); //left
         setRight(slowR);
         delay(10);
        } else if (mid > high) { //turn right
         setRight(forwardR);
         setLeft(slowL);
         delay(10);
        } else if (mid > low && mid < high) { // forward
         driveStraight();
         delay(2000);
        }
        if (mid < low) {
         setLeft(forwardL);
         setRight(slowR);
         } else if (mid > high) {
         setRight(forwardR);
         setLeft(slowL);
         } else if (mid > low && mid < high) {
         driveStraight();
         delay(4000);
         state = 2;
         }*/
    }

    //Ultrasonic Sensor
    void ping() {
      pinMode(trigPin, OUTPUT);
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
    }

};

class Camera {
    Pixy pixy;
    Drive robot;
  public:
    Camera(int leftServo, int rightServo, int armPin) : robot (leftServo, rightServo, armPin) { //Have to do this whole mess in order to have an object in a class
      intialize();
    }

    void intialize () {

      pixy.init();//Initializes Camera
      pixy.setServos (500, 500);//Center Servos
    }

    /*void tiltServosUp (int signiture) {
      // Block p =  getSpecialBlocks (signiture);
      //Block specialBlocks [pixy.getBlocks()] = {p};//Populates the array of blocks using the pointer
      Block specialBlocks [pixy.getBlocks()];//Populates the array of blocks using the pointer
      Block backWall = specialBlocks [3];
      int currentTilt = 500;
      if (backWall.x != NULL) {
        while (backWall.signature != 1 || backWall.x == NULL) {
          currentTilt += 5;
          pixy.setServos (500, currentTilt);
        }
      }
      }*/

    float getMidpoint (Block _blocks []) {//Returns the midpoint of the top two blocks in an array
      float midpoint = (_blocks [0].x + _blocks [1].x) * 2;
      return midpoint;
    }

    boolean pointToBlock (Block target, int hedge) {//Points to a Block that is sent in
      int width = 320; // pixy cam width
      if (target.x > width / 2 + hedge) {
        //Turns Left until the blocks x val is within a range
        robot.setLeft(135);
      }
      else if (target.x < width / 2 - hedge) {
        //Turns Right until the blocks x val is within a range
        robot.setRight(135);
      }
      else if (target.x > width / 2 - hedge && target.x < width / 2 + hedge) {
        //Stops once the block is within a range
        robot.stopServos();
        return true;
      }
      return false;
    }
};

void getSpecialBlocks (int signiture) {
  //Block blocks [pixy.getBlocks()] = {NULL};//Makes an Array of Blocks
  //Block blocks [pixy.getBlocks()];//Makes an Array of Blocks
  int numYellow = 0;//Ctr to count how many yellow blocks it has seen
  for (int i = 0; i < pixy.getBlocks(); i++) {
    if (pixy.blocks[i].signature == signiture) {//Checks if signiture of current block matches signiture sent in
      //Pushes Yellow Block into array created earlier
      blocks [numYellow] = (pixy.blocks[i]);
      numYellow++;
    }
  }

  Block swap;
  boolean sorted ;
  while (!sorted) {
    //Bubble Sorts Blocks By x val   --Height--
    sorted = true;
    for (int i = 0; i < pixy.getBlocks(); i++) {
      if (!blocks [i + 1].x == NULL) {//Breaks Out if next block is null
        if (blocks [i].width > blocks [i + 1].width) { //was height
          //Swaps the index of blocks one is larger then the next one
          swap = blocks [i];
          blocks [i + 1] = blocks [i];
          blocks [i + 1] = swap;
          sorted = false;
        }
      }
    }
  }

  //return blocks;
}
Camera cam (9, 10, 6);
Drive robot (9, 10, 6);  //Arm is on 6

void setup() {
  Serial.begin (9600);
  robot.attachServos();
}

void loop() {
//  robot.driveFast();
  sum = 0.0;// Camera Midpoint

  robot.sensorRead(); //Line Sensors

  //Ultrasonic Sensor
  exactDist = timing * dist;
  long duration, cm;
  robot.ping(); //triggers sensor
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  // convert the time into a distance
  //  cm = microsecondsToCentimeters(duration);
  dist = duration / 50; //gets average

  //dist * time = exactDist

  //Camera Midpoint
  for (int x = 0; x < 100; x++) {
    getSpecialBlocks(1);
    sum = sum + cam.getMidpoint(blocks);
  }
  mid = sum / 200.0; //120
  // delay(10);

  Serial.println(dist); //'mid' For Camera; 'dist' For Ping Sensor; 'Cval' For Line Sensor;
  //midPos = mid;
  //boolean point = poinToBlock(blocks, 10);

  //delay(2600); turns 90Degrees left


  //What States Do
  if (state == 1) { // Get to first wall
    robot.grab();
    robot.stopServos();
    delay(500);
    robot.driveFast();
    delay(1800);
    robot.turnLeft();
    delay(2000); //turns to much at 2300
    robot.stopServos();
    delay(500);
    robot.driveFast();
    delay(1000);
    state = 2;


  } else if (state == 2) {
    if (dist < 17) {//16 - 17
      robot.turnRight();
      delay(1800);
      robot.stopServos();
      delay(1000);
      robot.driveFast();
      delay(13000);
      robot.stopServos();
      delay(500);
      state = 3;
    } else if (dist > 17) {
      robot.driveStraight();
    }

  } else if (state == 3) {
    if (dist > 44) {//43-44
      robot.driveStraight();
    } else if (dist < 44) {
      robot.stopServos();
      state = 8;
    }
  } else if (state == 4) { // Pickup Football
    robot.drop();
    // delay(0);

    robot.driveSlow();
    if (Cval > colour || Lval > colour || Rval > colour) {
      robot.driveSlow();
      delay(1500);

      if (dist > 14) {
        robot.turnLeft();
      } else if (dist < 14) {
        robot.stopServos();
        state = 4;
      }
    } else {
      ///robot.driveSlow();
    }
  } else if (state == 5) {//Pickups footballs and reverses (uses Sonar)
    robot.stopServos();
    robot.drop();
    if (dist > 7 ) {
      robot.driveStraight();
      delay(exactDist);
    } else if (dist < 8 ) {
      robot.stopServos();
      robot.grab();
      delay(500);
      robot.reverse();
      delay(2000);
      robot.stopServos();
      delay(500);
      robot.turnRight();
      delay(2900);
      robot.stopServos();
      delay(500);
      robot.driveStraight();
      state = 5;
    }
  } else if (state == 6) {

    robot.stopServos();
  }


}

// Distance to Centimeters for Ultrasonic Sensor
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

