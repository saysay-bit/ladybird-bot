//Header file for RDaC

#define GRID_ROWS 12  //one grid box would be 5x5 cm
#define GRID_COLS 9
#define INT_MAX 100

//Ultrasonic sensor class creation =======================================================================
class UltrasonicSensor {
  private:
    int pin;
    unsigned long startTime;
    unsigned long endTime;

  public:
    //Constructor
    UltrasonicSensor(int pinNum) : pin(pinNum) {}

    //function to measure distance
    float DistanceMeasurement() {
      
      //Send the trigger pulse
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delayMicroseconds(2);
        digitalWrite(pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pin, LOW);

      //Switch to input to listen for echo
        pinMode(pin, INPUT);

      //Measure the duration of the echo signal
        unsigned long duration = pulseIn(pin, HIGH, 25000);  // 25ms timeout
      /*
        Serial.print("Sensor Duration ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(duration);


      if (duration == 0) {
        Serial.println("pulseIn timed out!");
      }
      */
      float distance = duration * 0.0343 / 2;
      return distance;
    }
};

class Motor {
  private:
    PinName PWM_Pin;
    PinName DIR_Pin;

  public:
    //Constructor
    Motor(PinName pwmPin, PinName dirPin) : PWM_Pin(pwmPin), DIR_Pin(dirPin) {}

};


float closest90Fix(float angle_to_fix);
void fix_position();

void stop();

void move_forward(float speedIn, float targetDistance);
void move_backward(float speedIn, float targetDistance);

void turn(float speedIn, float targetAngle, bool turnRight);


void turn_right(float speedIn, float targetAngle);
void turn_left(float speedIn, float targetAngle);

void countEncoderR();
void countEncoderL();

float calculate_deltaDelta();

void updateMazeMapObstacles();

void anglesAndCoordsUpdate();

int dijkstra(int maze[GRID_ROWS][GRID_COLS], struct Cell begin, struct Cell goal);
