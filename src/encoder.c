#include "main.h"

static int leftEncoder;
static int rightEncoder;
static double leftInches;
static double rightInches;
static double prevLeftInches;
static double prevRightInches;
static double velocity;

static int x;
static int y;
static double theta;

/*void leftHandler(unsigned char pin) {
	leftEncoder++;
}

void rightHandler(unsigned char pin){
  rightEncoder++;
}*/

// This method gets called in initialize()
void initEncoder() {
	/*pinMode(LEFT_ENCODER, INPUT);
	ioSetInterrupt(LEFT_ENCODER, INTERRUPT_EDGE_FALLING, leftHandler);

  pinMode(RIGHT_ENCODER, INPUT);
	ioSetInterrupt(RIGHT_ENCODER, INTERRUPT_EDGE_FALLING, rightHandler);*/
}

void tick(){
  imeGet(RIGHT_ENCODER, &rightEncoder);
  imeGet(LEFT_ENCODER, &leftEncoder);
  leftEncoder = -leftEncoder;

  leftInches = 2*PI*WHEEL_RADIUS*leftEncoder/360;
  rightInches = 2*PI*WHEEL_RADIUS*rightEncoder/360;

  theta = (leftInches-rightInches)/WHEEL_DISTANCE;
  velocity = ((leftInches-prevLeftInches)+(rightInches-prevRightInches))/2;
  x += velocity*cos(theta);
  y += velocity*sin(theta);

  prevLeftInches = leftInches;
  prevRightInches = rightInches;
}
