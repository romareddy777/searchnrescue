/*
 * Complete code for a sound-following robot with four motors.
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

// Motor control pins
const int leftMotor1ForwardPin = 7;
const int leftMotor1BackwardPin = 8;
const int rightMotor1ForwardPin = 4;
const int rightMotor1BackwardPin = 2;
const int leftMotor2ForwardPin = 10; // New motor
const int leftMotor2BackwardPin = 11; // New motor
const int rightMotor2ForwardPin = 12; // New motor
const int rightMotor2BackwardPin = 13; // New motor

// Microphone pins
const int mic1pin = A0;
const int mic2pin = A1;

// Sound sampling variables
const int sample_window = 2000; // Sample time in milliseconds
int mic1, mic2;
int mic1_max, mic1_min, mic2_max, mic2_min;
int amp1, amp2;
int difference;
unsigned long start_time, current_time;

void setup() {
  // Initialize all motor control pins as outputs
  pinMode(leftMotor1ForwardPin, OUTPUT);
  pinMode(leftMotor1BackwardPin, OUTPUT);
  pinMode(rightMotor1ForwardPin, OUTPUT);
  pinMode(rightMotor1BackwardPin, OUTPUT);
  pinMode(leftMotor2ForwardPin, OUTPUT);
  pinMode(leftMotor2BackwardPin, OUTPUT);
  pinMode(rightMotor2ForwardPin, OUTPUT);
  pinMode(rightMotor2BackwardPin, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  writefirstline();
  writelistening();
  //writeleft();
  //writeright();
  display.display();
}

void loop() {
  // Stop motors before sound sampling
  stopDriving();
  delay(500); // Wait for motors to stop completely

  // Reset max and min values for microphones
  mic1_min = mic1_max = mic2_min = mic2_max = 0;
  start_time = millis();

  // Sound sampling
  while (millis() - start_time < sample_window) {
    mic1 = analogRead(mic1pin);
    mic2 = analogRead(mic2pin);
    mic1_min = min(mic1, mic1_min);
    mic1_max = max(mic1, mic1_max);
    mic2_min = min(mic2, mic2_min);
    mic2_max = max(mic2, mic2_max);
  }

  // Calculate amplitudes and difference
  amp1 = mic1_max - mic1_min;
  amp2 = mic2_max - mic2_min;
  difference = amp1 - amp2 ;

  // Debugging: Print amplitudes and difference
  Serial.print("Mic 1 amplitude: ");
  Serial.print(amp1);
  Serial.print(" | Mic 2 amplitude: ");
  Serial.print(amp2);
  Serial.print(" | Difference = ");
  Serial.println(difference);

  // Movement decision based on sound direction
  if (abs(difference) < 50) { // Sound is straight ahead
    //driveForward();
  } else if (difference > 50) { // Sound is to the left
    display.clearDisplay();
    writefirstline();
    writeleft();
    display.display();
    // turnLeft();
    turnRight();
    delay(200);
    
  } else if (difference < -50) { // Sound is to the right
    display.clearDisplay();
    writefirstline();
    writeright();
    display.display();
    // turnRight();
    turnLeft();
    delay(200);
  }
  
  // Delay before next action
  delay(1000);
  display.clearDisplay();
  writefirstline();
  writelistening();
  display.display();
}

void driveForward() {
  digitalWrite(leftMotor1ForwardPin, HIGH);
  digitalWrite(leftMotor1BackwardPin, LOW);
  digitalWrite(rightMotor1ForwardPin, HIGH);
  digitalWrite(rightMotor1BackwardPin, LOW);
  digitalWrite(leftMotor2ForwardPin, HIGH);
  digitalWrite(leftMotor2BackwardPin, LOW);
  digitalWrite(rightMotor2ForwardPin, HIGH);
  digitalWrite(rightMotor2BackwardPin, LOW);
}

void turnRight() {
  digitalWrite(leftMotor1ForwardPin, HIGH);
  digitalWrite(leftMotor1BackwardPin, LOW);
  digitalWrite(rightMotor1ForwardPin, LOW);
  digitalWrite(rightMotor1BackwardPin, HIGH);
  digitalWrite(leftMotor2ForwardPin, HIGH);
  digitalWrite(leftMotor2BackwardPin, LOW);
  digitalWrite(rightMotor2ForwardPin, LOW);
  digitalWrite(rightMotor2BackwardPin, HIGH);
  delay(200); // Move for 200 milliseconds
  stopDriving(); // Stop motors
}

void turnLeft() {
  digitalWrite(leftMotor1ForwardPin, LOW);
  digitalWrite(leftMotor1BackwardPin, HIGH);
  digitalWrite(rightMotor1ForwardPin, HIGH);
  digitalWrite(rightMotor1BackwardPin, LOW);
  digitalWrite(leftMotor2ForwardPin, LOW);
  digitalWrite(leftMotor2BackwardPin, HIGH);
  digitalWrite(rightMotor2ForwardPin, HIGH);
  digitalWrite(rightMotor2BackwardPin, LOW);
  delay(200); // Move for 200 milliseconds
  stopDriving(); // Stop motors
}

void stopDriving() {
  digitalWrite(leftMotor1ForwardPin, LOW);
  digitalWrite(leftMotor1BackwardPin, LOW);
  digitalWrite(rightMotor1ForwardPin, LOW);
  digitalWrite(rightMotor1BackwardPin, LOW);
  digitalWrite(leftMotor2ForwardPin, LOW);
  digitalWrite(leftMotor2BackwardPin, LOW);
  digitalWrite(rightMotor2ForwardPin, LOW);
  digitalWrite(rightMotor2BackwardPin, LOW);
}

void writefirstline(void) {
  //display.clearDisplay();

  display.setTextSize(1.8);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(30, 0);
  display.println(F("Roma's Robot"));
}

void writelistening(void) {
  //display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(15, 30);
  display.println(F("Listening"));
}
void writeleft(void) {
  //display.clearDisplay();
  display.drawCircle(10, 50, 30, SSD1306_WHITE);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(1, 40);
  display.println(F("Left"));
}
void writeright(void) {
  //display.clearDisplay();
  display.drawCircle(display.width()-10, 50, 30, SSD1306_WHITE);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(95, 40);
  display.println(F("Right"));
}
