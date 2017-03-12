#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>
#include <TPixy.h>
#include <Servo.h>
Pixy pixy;
Block blocks[10];
int slowR = 92.33;
int slowL = 88.3;
int forwardR = 88.33; //forward
int forwardL = 92.3;  //forward
int reverseR = 100; //reverse
int reverseL = 79; //reverse
int stopL = 85;   //stop
int stopR = 95;   //moves slightly backward
int midPos;
int state = 1; // runs through different states for each part of the field  (set to 1 to enable)
float mid = 0, sum = 0;
int high = 11; // 35 is mid
int low = 9;
boolean oneBlock;

int sensorL = A2; // The line sensors are connected to A0, A1 and A2
int sensorC = A1;
int sensorR = A0;
int colour = 800; //middle of black and white

int Lval;
int Cval;
int Rval;

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
      leftServoPin = leftServo;
      rightServoPin = rightServo;
      armPin = armServo;
    }

    void attachServos() {
      //initialzes Servos
      leftServo.attach(leftServoPin);
      rightServo.attach(rightServoPin);
      armServo.attach (armPin);
      //Put Servos back in the middle
      leftServo.write (86.3);
      rightServo.write (96);
      armServo.write (90);
      //Serial.println ("LeftServo");
      Serial.print (leftServo.read());
    }

    void setLeft (int speed) {
      leftServo.write (speed);
    }
    void setRight (int speed) {
      rightServo.write (speed);
    }
    void stopServos() {
      setLeft(90);
      setRight(90);
    }

    void driveStraight () {
      setRight (88.33);
      setLeft(92.3);
    }
    
    void grab () {
      armServo.write (100);
      //Serial.println(armServo.read());
      delay(2000);
    }
    void drop () {
      armServo.write (10);
      //Serial.println (armServo.read());
      delay(2000);
    }

    void sensorRead () {
      Lval = analogRead (sensorL);
      Cval = analogRead (sensorC);
      Rval = analogRead (sensorR);
    }

    void getThrough() {  // get through defenders (state 1)
      if (midPos < low)  {// align robot
        setRight(slowR);
        setLeft(stopL);
      } else if (midPos > high) {
        setRight(stopR);
        setLeft(slowL);
      } else if (midPos < high && midPos > low) {
        driveStraight(); //move forward
        delay (2000);
        state = 2;
        /*       if (midPos < low)  { // align again
                 setRight(slowR);
                 setLeft(stopL);
               } else if (midPos > high) {
                 setRight(stopR);
                 setLeft(slowL);
               } else if (midPos < high && midPos > low) {
                 driveStraight();
                 delay (1000); // get through defenders and change state
                 stopServos();
                 state = 2; // go to next state
               }*/
      }
    }


   void straight() { // what to do when through defenders (state 2)
      grab();
      drop();
      /*
        if (oneBlock = true) {
        setRight(forwardR);
        setLeft(forwardL);
        delay(2000);
        setRight(stopR);
        setLeft(stopL);
        } else if (oneBlock = true) {
        state = 1;
        }
      */
    }

    /*
      void lineFollower() { //follow lines
driveStraight();
      if ( Lval < colour && Cval > colour &&  Rval < colour)  { //goes straight when sees line in center
        setLeft(forwardL);
        setRight(forwardR);
      } else if ( Lval > colour && Cval < colour &&  Rval < colour) { //turns right when line is to the left
        setRight(forwardR);
        setLeft(reverseL);
        delay(200);
      } else if ( Lval < colour && Cval < colour && Rval > colour) { //turns left when the line is to far to the right
        setRight(reverseR);
        setLeft(forwardL);
        delay(200);
      }
      }
    */

    
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
      float midpoint = (_blocks [0].x + _blocks [1].x) / 2 ;
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
    //Bubble Sorts Blocks By Height
    sorted = true;
    for (int i = 0; i < pixy.getBlocks(); i++) {
      if (!blocks [i + 1].x == NULL) {//Breaks Out if next block is null
        if (blocks [i].height > blocks [i + 1].height) {
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
  sum = 0.0;

  for (int x = 0; x < 100; x++) {
    getSpecialBlocks(1);
    sum = sum + cam.getMidpoint(blocks);
  }
  mid = sum / 300.0;
  delay(10);
  Serial.println(mid);

  //boolean point = poinToBlock(blocks, 10);

    if (state == 1) { //What states do
    robot.getThrough();
  } else if (state == 2) {
    robot.straight();
  } else {
    robot.stopServos();
    }
}
