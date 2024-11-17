constexpr struct
{
    int pinA;
    int pinB;
} MOTOR_PINS[] = {
    {0, 1},
    {2, 3},
    {4, 5},
    {6, 7}
};

constexpr struct
{
    int pinTrig;
    int pinDir;
} ENCODER_PINS[] = {
    {34, 35},
    {32, 33},
    {25, 26},
    {14, 27}
};

constexpr struct
{
    int pinSCK;
    int pinDT;
} HX711_PINS[] = {
    {23, 16},
    {23, 17},
    {23, 18},
    {23, 19}
};

constexpr struct 
{
    int sda;
    int scl;
} IMU_PINS = {
    21, 22
};

