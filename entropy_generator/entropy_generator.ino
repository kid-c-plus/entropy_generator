#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>

// Constants

// Delay inbetween successive accelerometer samples (here, 1 second)
unsigned long SAMPLE_DELAY = 1000;

// Minimum sum of absolute values of differences between this and last sample needed to store sample
// This helps prevent a buildup of extremely predictable values from a lack of motion
// Here, 100, but, you know, we'll see
int16_t DIFFERENCE_THRESHOLD = 1000;

// Size of acceleration data array, in int16_t (here, 1 KB worth, hopefully not too much)
// I store acceleration data as all three dimensional values, xored together
size_t ACCELERATION_DATA_ARRAY_SIZE = 1024 / sizeof(int8_t);

// Just local versions of INT16_MAX and INT16_MIN
int16_t ACCELERATION_DIMENSION_MAX = 0x7fff;
int16_t ACCELERATION_DIMENSION_MIN = (-ACCELERATION_DIMENSION_MAX - 1);

// Global Variables

// Accelerometer object
Adafruit_MMA8451 accelerometer = Adafruit_MMA8451();

// Clock time (in milliseconds) of previous sample
unsigned long prevMillis = millis();

// Current LED state (either HIGH or LOW)
int ledState = LOW;

// Acceleration data array
int8_t * dataArray;

// Read and write points to array queue
int dataArrayRead = 0;
int dataArrayWrite = 0;

// Previous accepted acceleration values (for evaluating difference)
int16_t xPrev = 0;
int16_t yPrev = 0;
int16_t zPrev = 0;

// Command char array and index within
char * commandArray;
int commandIndex = 0;

// Methods

// setup - start function, run when code is loaded
void setup(void) {
  // Set built-in LED pin to output and turn it off
  // It'll light up whenever the accelerometer is sampled
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  
  // Begin serial communication at 9600 baud
  Serial.begin(9600);

  // Sets 10-second timeout
  Serial.setTimeout(10000);
  
  Serial.println("~ Entropy Generator ~");

  if (!accelerometer.begin()) {
     // Couldn't communicate with MMA8451
     Serial.println("ERROR STARTING ENTROPY GENERATOR: COULDN'T START ACCELEROMETER");
     // A busy wait, is the best way to do this
     while (1);
  }

  // Set max acceleration it can register before clipping, is 2 times Earth's gravitation (could also be 4 or 8)
  // I want something that fills the range of all possible values pretty well, without too much clipping
  accelerometer.setRange(MMA8451_RANGE_2_G);

  // Allocate data array
  dataArray = new int8_t[ACCELERATION_DATA_ARRAY_SIZE];
}

// sendRandomNumbers - convenience method to write a given number of random 8-bit integers 
//                     (if 0, or really any garbage data) all stored integers are sent), 
//                     after "y" signifying successful request or "n" signifying insufficient entropy
// @param quantity - number of 16-bit integers to send, encoded
void sendRandomNumbers(String quantity) {
  int numIntegers = quantity.toInt();

  // Send all values collected
  if (numIntegers <= 0) {
    numIntegers = (dataArrayWrite - dataArrayRead) % ACCELERATION_DATA_ARRAY_SIZE;
  }

  // Device has enough samples to service request
  if (numIntegers != 0 && (dataArrayWrite - dataArrayRead) % ACCELERATION_DATA_ARRAY_SIZE >= numIntegers) {

    // Request successful
    Serial.println("y");
    for (int i = 0; i < numIntegers; i++) {
      Serial.println(dataArray[dataArrayRead]);
      dataArrayRead = (dataArrayRead + 1) % ACCELERATION_DATA_ARRAY_SIZE;
    }
  }
  
  // Device has insufficient samples
  else {
      Serial.println("n");
  }
}

// sendPseudorandomNumbers - convenience method to write a given number of pseudorandom 8-bit
//                           integers (generated using the onboard PRNG) within a given range,
//                           after "y" signifying successful request or "n" signifying
//                           insufficient entropy. PRNG is seeded with two 16-bit samples
//                           concatenated into a long
// @param args - "<quantity> <minimum> <maximum>" as String
void sendPseudorandomNumbers(String args) {
  
  // Parse out string
  int indices[] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    if (i > 0) {
      indices[i] = args.indexOf(' ', indices[i - 1]);
      if (indices[i] == -1) {
        Serial.println("ERROR SENDING PSEUDORANDOM NUMBERS: IMPROPER ARGUMENT STRING");
        return;
      }
    }
    while (indices[i] < args.length() && args[indices[i]] == ' ') {
      indices[i]++;
    }
    if (indices[i] == args.length()) {
      Serial.println("ERROR SENDING PSEUDORANDOM NUMBERS: IMPROPER ARGUMENT STRING");
      return;
    }
  }

  // Check to see if the values are acceptable
  int quantity = args.substring(indices[0], indices[1]).toInt();
  int minimum = args.substring(indices[1], indices[2]).toInt();
  int maximum = args.substring(indices[2]).toInt();
  if (quantity <= 0 || minimum >= maximum) {
    Serial.println("ERROR SENDING PSEUDORANDOM NUMBERS: IMPROPER ARGUMENT STRING");
    return;
  }
  
  // We need to have at least 4 samples
  if ((dataArrayWrite - dataArrayRead) % ACCELERATION_DATA_ARRAY_SIZE >= 4) {

    // Request successful
    Serial.println("y");

    // Strings together four 8-bit random samples
    long seed = ((long) dataArray[dataArrayRead]) << 24 | ((long) dataArray[(dataArrayRead + 1) % ACCELERATION_DATA_ARRAY_SIZE]) << 16 |
                ((long) dataArray[(dataArrayRead + 2) % ACCELERATION_DATA_ARRAY_SIZE]) << 8 | ((long) dataArray[(dataArrayRead + 3) % ACCELERATION_DATA_ARRAY_SIZE]);
    dataArrayRead = (dataArrayRead + 4) % ACCELERATION_DATA_ARRAY_SIZE;

    // Generate and send seeded random numbers
    randomSeed(seed);
    for (int i = 0; i < quantity; i++) {
      // Kills upper byte of random value (that's fine)
      Serial.println((int8_t) (random(minimum, maximum)));
    }
  }

  // Device has insufficient samples
  else {
    Serial.println("n");
  }
}

// sendNumSamples - convenience method to write number of 8-bit integer
//                  samples available to read
void sendNumSamples() {
  Serial.println((dataArrayWrite - dataArrayRead) % ACCELERATION_DATA_ARRAY_SIZE);
}

// loop - work function, run continuously
void loop() {
  while (Serial.available() > 0) {
    // Read one line from machine
    String op = Serial.readStringUntil(';');
    if (op.length() > 0) {
      switch (op[0]) {
        case ('R', 'r') :
          sendRandomNumbers(op.substring(2));
          break;
        case ('P', 'p') :
          sendPseudorandomNumbers(op.substring(2));
          break;
        case ('N', 'n') :
          sendNumSamples();
          break;
        default :
          Serial.println("ERROR READING COMMAND: UNRECOGNIZED OPCODE");
          break;
      }
    } else {
      Serial.println("ERROR READING COMMAND: INCOMPLETE COMMAND");
    }
  }
  unsigned long currentMillis = millis();
  
  // Eventually it's gotta zero out again, right?
  if (currentMillis < prevMillis) {
    prevMillis = 0;
  }

  // Time to sample again
  if (currentMillis - prevMillis >= SAMPLE_DELAY) {
    accelerometer.read();
    int16_t difference = abs(accelerometer.x - xPrev) + abs(accelerometer.y - yPrev) + abs(accelerometer.z - zPrev);
    // None of the values are clipped, it's sufficiently different for us to add it, or the queue is empty
    if (max(max(accelerometer.x, accelerometer.y), accelerometer.z) < ACCELERATION_DIMENSION_MAX && 
        min(min(accelerometer.x, accelerometer.y), accelerometer.z) > ACCELERATION_DIMENSION_MIN && 
        difference >= DIFFERENCE_THRESHOLD || dataArrayWrite == dataArrayRead) {
      // Write new sample to data array
      int8_t currData = (int8_t) (accelerometer.x ^ accelerometer.y ^ accelerometer.z);
      dataArray[dataArrayWrite] = currData;
      xPrev = accelerometer.x;
      yPrev = accelerometer.y;
      zPrev = accelerometer.z;

      // Iterate write point
      dataArrayWrite = (dataArrayWrite + 1) % ACCELERATION_DATA_ARRAY_SIZE;
      
      // The queue is full, and the write point has looped back around to the read point
      // Start eliminating oldest values
      if (dataArrayWrite == dataArrayRead) {
        dataArrayRead = (dataArrayRead + 1) % ACCELERATION_DATA_ARRAY_SIZE;
      }
      
      ledState = HIGH;
      digitalWrite(LED_BUILTIN, ledState);
    }
    // It's still been one cycle since the last sample, so I'll turn off the sample LED
    else if (ledState == HIGH) {
      ledState = LOW;
      digitalWrite(LED_BUILTIN, ledState);
    }
    
    prevMillis = currentMillis;
  }
}
