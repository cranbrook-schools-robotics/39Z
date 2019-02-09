#include "robot-config.h"
#include <math.h>
#include <time.h>

long startTime;

double theta;
double thetaOffset = 0;
double modTheta;
double x;
double y;
double targetX;
double targetY;

bool breaking = false;
bool prevA;
bool prevB;
bool prevY;
bool prevX;

double distance;
double print;
double lift;
int step;
double flipPosition;
double flipperOffset = 0.397;
StartingPosition start = RED_FLAGS;

PID liftPID = PID(170,0,0);
PID flipPID = PID(150,0,0);
PID targetPID = PID(6,0,0);

const double PI = 3.141592653589793;

competition Competition;

double autonLength = 10;
double autonX[] = {-1  , -3, -3, 34 + 2,    -1   ,    23, -2,   -4,    -1     , 30.11  , 16.78, -1, 40};
double autonY[] = {PI/4,  0, 0, 72.576 + 3, -PI/1.5 , 35 ,2.756, 1.5 , -PI, 37.087 , 51.386, 0 , 46.576};

//double autonX[] = {-1  , -3, -3, 34, -1   , 10.5, -1, 16.78, -1, 40};
//double autonY[] = {-PI/4,  0, 0, 44, PI/2 , 66  , PI, 65.19, 0 , 70};

/*
31.753872, 38.713623
10.439629, 12.179567
27.404026, 62.63777
5.437307, 69.59753
29.143965, 81.77709
56.113007, 81.5596
*/


void autonomous(void){
    //unhitch();
    /*for(step = 0; step < autonLength; step++){
        if(autonX[step] > 0){
            do {
                goTo(autonX[step],autonY[step]);
            } while(distance > 2);
        }
        else if(autonX[step] == -1){
            do {
                turnTo(autonY[step]);
            } while(fabs(modTheta-autonY[step]) > 0.2);
        }
        else if(autonX[step] == -2){
                lift = autonY[step];
        }
        else if(autonX[step] == -3){
                flip();
        }
        else if(autonX[step] == -4){
            double startTime = time();
            double i = 0;
            while(time() < startTime+autonY[step]){
                  i++;  
            }
        }
    }*/
    while(true){
        while(runAuton()){
            print = 1;
            goTo(18.269+26,58.288);
        }
    }
}

bool runAuton(){
    return Competition.isAutonomous() && Competition.isEnabled();
}
    
void pre_auton(){
    
    //targetX = 34.58;
    //targetY = 35.23;
    
    switch(start){
        case RED_FLAGS:
            x = 18.269;
            y = 58.288;
            thetaOffset = 0;
            break;
        case RED_EXPANSION:
            x = 18.269;
            y = 105.701;
            thetaOffset = 0;
            break;
        case BLUE_FLAGS:
            x = 122.448;
            y = 58.288;
            thetaOffset = PI;
            break;
        case BLUE_EXPANSION:
            x = 122.448;
            y = 105.701;
            thetaOffset = PI;
            break;
    }
    
    control.ButtonUp.pressed(buttonUp);
    control.ButtonDown.pressed(buttonDown);
    control.ButtonLeft.pressed(buttonLeft);
    control.ButtonRight.pressed(buttonRight);
    control.ButtonA.pressed(buttonA);
    control.ButtonB.pressed(buttonB);
        
    rightBase.spin(directionType::fwd);
    leftBase.spin(directionType::fwd);
    rightBase.setVelocity(0,velocityUnits::pct);
    leftBase.setVelocity(0,velocityUnits::pct);
    
    rightLift.spin(directionType::fwd);
    leftLift.spin(directionType::fwd);
    rightLift.setVelocity(0,velocityUnits::pct);
    leftLift.setVelocity(0,velocityUnits::pct);
    
    flipper.spin(directionType::fwd);
    flipper.setVelocity(0,velocityUnits::pct);
    startTime = clock();
    
}


void usercontrol(void){
    
    
    while(true){
        if(!runAuton()){
        rightBase.setVelocity((100/128.0)*control.Axis2.value(),vex::velocityUnits::pct);
        leftBase.setVelocity((100/128.0)*control.Axis3.value(),vex::velocityUnits::pct);
        }
        //targetX = x + cos(theta)*control.Axis3.value() - sin(theta)*control.Axis4.value();
        //targetY = y + sin(theta)*control.Axis3.value() + cos(theta)*control.Axis4.value();
        //goTo(targetX,targetY);
        if(!prevA && control.ButtonA.pressing()){
            buttonA();
        }
        if(!prevB && control.ButtonB.pressing()){
            buttonB();
        }
        if(!prevY && control.ButtonY.pressing()){
            buttonY();
        }
        if(!prevX && control.ButtonX.pressing()){
            buttonX();
        }
        
        if(breaking){
            goTo(targetX,targetY);
        }
        
        
        prevA = control.ButtonA.pressing();
        prevB = control.ButtonB.pressing();
        prevY = control.ButtonY.pressing();
        prevX = control.ButtonX.pressing();
        
        print = lift;
        
        task::sleep(20);
        
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    
    //Run the pre-autonomous function. 
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    //Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}

int values[] = {0,0,0,0,0,0};
int i = 0;

void display(){
    while(true){
        Brain.Screen.clearLine();
        Brain.Screen.print("%f, %f",x,y);
        control.Screen.clearLine(0);
        control.Screen.clearLine(1);
        control.Screen.clearLine(2);
        control.Screen.setCursor(0,0);
        control.Screen.print("%f, %f",x,y);
        control.Screen.newLine();
        control.Screen.print("%f",theta);
        control.Screen.newLine();
        //control.Screen.print("%d%d%d, %d%d%d",values[0],values[1],values[2],values[3],values[4],values[5]);
        //control.Screen.print("%f, %f",autonX[step],autonY[step]);
        control.Screen.print("%f",print);
        task::sleep(33);
    }
}

void buttonUp(){
    values[i]++;
    if(values[i] > 9)
        values[i] = 0;
}

void buttonDown(){
    values[i]--;
    if(values[i] < 0)
        values[i] = 9;
}

void buttonLeft(){
    i--;
    if(i < 0)
        i = 0;
}

void buttonRight(){
    i++;
    if(i >= 6)
        i = 5;
}

void buttonA(){
    print = flipPosition;
    flip();
    //targetX = 100*values[0] + 10*values[1] + values[2];
    //targetY = 100*values[3] + 10*values[4] + values[5];
}

void buttonB(){
    targetX = x;
    targetY = y;
    breaking ^= true;
}

void buttonY(){
    lift = 1.87;
}

void buttonX(){
    unhitch();
}

void positionLoop(){
    double prevTime = time();
    
    while(true){
        double currentTime = time();
        double dt = currentTime-prevTime;
        
        theta = 0.718202679482081*(leftBase.rotation(rotationUnits::rev)-rightBase.rotation(rotationUnits::rev)) + thetaOffset;
        double velocity = 1.065*dt*(leftBase.velocity(velocityUnits::rpm)+rightBase.velocity(velocityUnits::rpm));
        modTheta = fmod(theta,2*PI);
        x += velocity*cos(theta);
        y += velocity*sin(theta);
        
        prevTime = currentTime;
        task::sleep(1);
    }
}

double time(){
    return (clock()-startTime)/(double)CLOCKS_PER_SEC;
}

void goTo(double targetX, double targetY){
    double dtheta = atan2(targetY-y,targetX-x)-theta;
    if(cos(dtheta) < 0 ){
        dtheta = -dtheta;
    }
    dtheta += PI/4;
    
    distance = sqrt((x-targetX)*(x-targetX) + (y-targetY)*(y-targetY));
    double velocity = targetPID.apply(-distance);
    setBase(velocity*squareSin(dtheta),velocity*squareCos(dtheta));
}

void turnTo(double target){
    double velocity = fmin(100,fabs(modTheta-target)*100);
    if(modTheta-target > 0){
        velocity = -velocity;
    }
    setBase(velocity,-velocity);
}

void setBase(double left, double right){
    leftBase.setVelocity(left,velocityUnits::pct);
    rightBase.setVelocity(right,velocityUnits::pct);
}

double squareSin(double x){
    return (cos(x) >= 0 ? 1 : -1) * fmin(1,fmax(-1,tan(x)));
}

double squareCos(double x){
    return (sin(x) >= 0 ? 1 : -1) * fmin(1,fmax(-1,-tan(x+PI/2)));
}

PID::PID(double p, double i, double d){
    //if(p < 0 || i < 0 || d < 0)
        //throw "A negative PID weight value is inherently counterproductive."; //TODO exception throwing.
    kP = p;
    kI = i;
    kD = d;
    //initialize prevTime?
    prevTime = time();
}

double PID::apply(double measured){
    double error = target-measured;
    double proportional = kP*error;
    
    
    return proportional;
}

void PID::setTarget(double t){
    target = t;
}

void liftLoop(){
    double prevTime = time();
    
    while(true){
        double currentTime = time();
        double dt = currentTime - prevTime;
        
        if(control.ButtonL1.pressing()){
           lift += dt*30;
        }
        else if(control.ButtonL2.pressing()){
            lift -= dt*30;
        }
        
        if(control.ButtonR1.pressing()){
           lift += dt*15;
        }
        else if(control.ButtonR2.pressing()){
            lift -= dt*15;
        }
        
        lift = fmin(2.756,fmax(0,lift));
        
        liftPID.setTarget(lift);
        double power = liftPID.apply((rightLift.rotation(rotationUnits::rev) + leftLift.rotation(rotationUnits::rev))/2.);
        power = fmin(100,fmax(-100,power));
        rightLift.setVelocity(power, velocityUnits::pct);
        leftLift.setVelocity(power, velocityUnits::pct);
        
        flipPID.setTarget(flipPosition);
        double flipPower = flipPID.apply(flipper.rotation(rotationUnits::rev));
        flipPower = fmin(100,fmax(-100,flipPower));
        flipper.setVelocity(flipPower, velocityUnits::pct);
        
        //print = (rightLift.rotation(rotationUnits::rev) + leftLift.rotation(rotationUnits::rev))/2.;
        
        prevTime = currentTime;

        print = flipPosition;
        task::sleep(1);
    }
}

void flip(){
    if(flipPosition == 0.5 + flipperOffset){
        flipPosition = 0 + flipperOffset;
    }
    else{
        flipPosition = 0.5 + flipperOffset;
    }
}

void unhitch(){
    flipPosition = flipperOffset/2.;
}