using namespace vex;

void positionLoop();
void liftLoop();

void goTo(double targetX, double targetY);
double normalize(double val1, double val2);
void setBase(double left, double right);
double squareSin(double x);
double squareCos(double x);
void display();
void turnTo(double target);
double time();

bool runAuton();

void flip();
void unhitch();

void buttonUp();
void buttonDown();
void buttonLeft();
void buttonRight();
void buttonA();
void buttonB();
void buttonX();
void buttonY();

void pre_auton();
void autonomous();
void usercontrol();

thread displayThread(display);
thread positionThread(positionLoop);
thread liftThread(liftLoop);
thread driverThread(usercontrol);
thread autonThread(autonomous);

enum StartingPosition{
    RED_EXPANSION,
    RED_FLAGS,
    BLUE_EXPANSION,
    BLUE_FLAGS
};

class PID {
public:
    double kP;
    double kI;
    double kD;
    
    double prevTime;
    double integral;
    double prevValue;
    
    double target;
    
    PID(double p, double i, double d);
    void setTarget(double t);
    double apply(double measured);
};

brain Brain;

motor rightBase(vex::PORT10, vex::gearSetting::ratio18_1, true);
motor leftBase(vex::PORT20, vex::gearSetting::ratio18_1, false);

motor rightLift(vex::PORT1, vex::gearSetting::ratio18_1, false);
motor leftLift(vex::PORT11, vex::gearSetting::ratio18_1, true);

motor flipper(vex::PORT9, vex::gearSetting::ratio18_1, false);

controller control(vex::controllerType::primary);