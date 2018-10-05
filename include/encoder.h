static double velocity;
static int x;
static int y;
static double theta;
static int leftEncoder;
static int rightEncoder;

void encoderHandler(int pin);

// This method gets called in initialize()
void initEncoder();
void tick();

int getEncoderTicks();

void resetEncoder();
