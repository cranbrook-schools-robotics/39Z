static volatile double velocity;
static volatile int x;
static volatile int y;
static volatile double theta;

void encoderHandler(int pin);

// This method gets called in initialize()
void initEncoder();
void tick();

int getEncoderTicks();

void resetEncoder();
