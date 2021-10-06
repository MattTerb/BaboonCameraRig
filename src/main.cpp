/**********************************************************************
   File: BaboonCameraRig
   Modified by: Matthew Terblanche
   Date: 5 September 2021
 **********************************************************************/

#include <Arduino.h>
#include <Encoder.h>
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define OUT_A 2 // D2 with interrupt
#define OUT_B 4 // D4 no interrupt
#define OUT_Z 3 // D3 with interrupt

#define BUZZER 5

#define START 6
#define STOP 7

#define DATA_LED 8
#define SYNC_LED 9

#define CS 10

SdFat SD;

File myFile;

volatile bool writeToFile = false;

volatile bool index_triggered = false;
void index_ISR()
{
  index_triggered = true;
}

long lastDebounceTime = 0;
const int debounceDelay = 500; // ms

const int startBeepFreq = 2800;
const int stopBeepFreq = 2800;
//const int regularBeepFreq = 3000;

float angle = 0;

unsigned int dataSet;

size_t n;       // Length of returned field with delimiter.
char str[20];   // Must hold longest field with delimiter and zero byte.
char *ptr;      // Test for valid field.
char delim = 0; // Delimiter from previous line. Start with no delimiter.

const long beepInterval = 500000; // Minimum gap between beeps

const long regularBeepTime = 10000000; // Beep every 10s

byte setOrigin = LOW;

byte fullRev = LOW;

int beepCount = 0; // Used for scheduling beeps

unsigned int freqArray[6]; // Used for scheduling beep frequency

unsigned long durArray[6]; // Used for scheduling beep duration

unsigned long currentMillis = 0; // stores the value of millis() in each iteration of loop()
unsigned long previousBeepMillis = 0;

unsigned long currentLedMillis = 0; // stores the value of millis() in each iteration of loop()
unsigned long previousLedMillis = 0;

unsigned long previousBeepTime = 0;

unsigned long readMicros = 0;

long newPosition;

Encoder myEnc(OUT_A, OUT_B);

byte buzzerState = LOW;

byte startWriting = LOW;

unsigned int writeCount = 0;

byte ledState = LOW;

byte openFile = HIGH;

unsigned long beepMicros;

byte longFlash = LOW;

byte error = LOW;

unsigned long dur = 250000;

// Function Declarations
void updateBuzzerState();
void beep(unsigned int freq, unsigned long duration, bool regularBeep);
void resetPos();
void startDataSet();
void endDataSet();
void readEncoder();
void saveEncoderData();

// Read last entry of the file
size_t readField(File *file, char *str, size_t size, const char *delim)
{
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1)
  {
    // Delete CR.
    if (ch == '\r')
    {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch))
    {
      break;
    }
  }
  str[n] = '\0';
  return n; // length of field including terminating delimiter
}

void setup()
{

  pinMode(DATA_LED, OUTPUT);
  pinMode(SYNC_LED, OUTPUT);

  digitalWrite(SYNC_LED, LOW);
  digitalWrite(DATA_LED, LOW);

  pinMode(OUT_Z, INPUT_PULLUP);

  pinMode(START, INPUT_PULLUP); // Start
  pinMode(STOP, INPUT_PULLUP);  // Stop

  pinMode(BUZZER, OUTPUT); // Buzzer

  // TIMER SETUP- the timer interrupt allows preceise timed measurements of the reed switch
  //for mor info about configuration of arduino timers see http://arduino.cc/playground/Code/Timer1
  //cli(); //stop interrupts

  //set timer1 interrupt at 60Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B

  // Set CTC mode
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);

  // Set CS12, C11, C10 bits for 8 prescaler
  TCCR1B &= ~(1 << CS12);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TCNT1 = 0; //initialize counter value to 0;
  // set timer count for 60hz increments
  OCR1A = 33332; // = (16*10^6) / (60*8) - 1
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); //allow interrupts
  //END TIMER SETUP

  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Serial.print("Initializing SD card...");

  if (!SD.begin(CS))
  {
    //   Serial.println("initialization failed!");
    error = HIGH;
    return;
  }

  //Serial.println("initialization done.");

  if (SD.exists("dataset.csv"))
  {

    // Serial.println("dataset.csv exists.....");

    myFile = SD.open("dataset.csv", FILE_READ); // open "dataset.csv" to read data

    while (true)
    {

      n = readField(&myFile, str, sizeof(str), "\n"); // Read last entry from file

      // Read error or at EOF.
      if (n == 0)
      {
        break;
      }

      dataSet = strtol(str, &ptr, 10); // Convert entry to data type long

      // Skip any blanks after number.
      while (*ptr == ' ')
      {
        ptr++;
      }
      // Save delimiter.
      delim = *ptr;
    }

    myFile.close(); // Close file to save data
  }
  else
  {

    dataSet = 1;
    //  Serial.println("dataset.csv does not exist.....");

    // Write Data Set Number
    myFile = SD.open("dataset.csv", FILE_WRITE);
    if (myFile)
    {
      //    Serial.print("Writing to dataset.csv...");
      myFile.println("Dataset Number");
      myFile.println(dataSet); // Add '1' to new dataSet file
      // close the file:
      myFile.close();
      //  Serial.println("done.");
    }
    else
    {
      // if the file didn't open, print an error:
      // Serial.println("error opening dataset.csv");
      error = HIGH;
    }
  }

  attachInterrupt(digitalPinToInterrupt(OUT_Z), index_ISR, RISING); // Add interrupt for detecting origin
}

void loop()
{

  if (error){
    digitalWrite(SYNC_LED,HIGH);
  }

  currentMillis = micros();

  currentLedMillis = micros();

  //unsigned long currentBeepTime = micros();

  updateBuzzerState();

  if (index_triggered)
  {
    resetPos(); // If receive pulse from OUT Z then set position to 0
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {

    if (digitalRead(START) == LOW) // Start logging data
    {

      Serial.println("START");
      lastDebounceTime = millis();

      writeToFile = true;
      if (error == LOW)
      {
        digitalWrite(DATA_LED, HIGH);
        startDataSet();
      }
    }

    if (digitalRead(STOP) == LOW) // Stop logging data
    {

      Serial.println("STOP");
      lastDebounceTime = millis();

      digitalWrite(DATA_LED, LOW);

      // Beep order : LIFO
      beep(stopBeepFreq, 800, false);
      beep(stopBeepFreq, 400, false);
      beep(stopBeepFreq, 400, false);

      writeToFile = false;

      endDataSet();
    }

  }

  // if (currentBeepTime - previousBeepTime >= regularBeepTime)
  // {

  //   beep(regularBeepFreq, 400, true);
  //   // Serial.println("Beep");

  //   previousBeepTime = currentBeepTime;
  // }

  if (ledState == LOW) // If led is off
  {

    if (currentLedMillis - previousLedMillis > 500000) // interval
    {
      ledState = HIGH;
      digitalWrite(SYNC_LED, HIGH);
      previousLedMillis = currentLedMillis;

      longFlash = !longFlash;
    }
  }
  else
  {
    if (longFlash)
    {
      dur = 650000;
    }
    else
    {
      dur = 250000;
    }

    if (currentLedMillis - previousLedMillis > dur) // duration
    {
      ledState = LOW;
      digitalWrite(SYNC_LED, LOW);
      previousLedMillis = currentLedMillis;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{ //Interrupt at freq of 90Hz to save data to SD

  if ((writeToFile == true) && (startWriting == HIGH))
  {
    readEncoder();
    saveEncoderData();
  }
}

void updateBuzzerState() // Continously check and update state of buzzer
{

  if (beepCount > 0) // Check if a beep is scheduled
  {

    if (buzzerState == LOW) // If buzzer is off
    {

      if (currentMillis - previousBeepMillis > beepInterval)
      {

        tone(BUZZER, freqArray[beepCount - 1], durArray[beepCount - 1]); // Play beep

        Serial.println("Beep");

        buzzerState = HIGH;

        previousBeepMillis = currentMillis;
      }
    }
    else
    {

      if (currentMillis - previousBeepMillis > durArray[beepCount - 1] * 1000)
      {

        buzzerState = LOW;
        beepCount -= 1; // Remove beep from schedule

        previousBeepMillis = currentMillis;
      }
    }
  }
}

void beep(unsigned int freq, unsigned long duration, bool regularBeep) // Add beep to schedule
{

  if (regularBeep == true && beepCount > 0) // Don't play regular beep if another beep is already scheduled
  {
    // Serial.println("SKIP BEEP");
  }
  else
  {

    beepCount += 1; // Increase beep count

    freqArray[beepCount - 1] = freq; // Add frequency to scheduled beep

    durArray[beepCount - 1] = duration; // Add duration to scheduled beep
  }
}

void resetPos() // Set position to 0
{

  if ((setOrigin == LOW) || (fullRev == HIGH))
  {
    Serial.println("ORIGIN");
    setOrigin = HIGH;
    fullRev = LOW;
    myEnc.write(0); // Set encoder to 0
    index_triggered = false;
    beep(1800, 200, false); // Beep when at origin
  }
}

void startDataSet() // Save column titles to file
{

  writeCount = 0;

  myFile = SD.open("encoder.csv", FILE_WRITE);
  if (myFile)
  {

    beep(startBeepFreq, 400, false);
    beep(startBeepFreq, 400, false);
    beep(startBeepFreq, 400, false);

    //  Serial.print("Writing to encoder.csv...");
    myFile.println("Dataset Number,Time,Rotation,LED State"); // Write column title to file
    // close the file:
    myFile.close();
    //  Serial.println("done.");

    startWriting = HIGH;
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("START error opening encoder.csv");
    error = HIGH;
  }
}

void endDataSet() // Write dataset number to dataset file
{

  if (myFile)
  {
    myFile.close();
  }
  startWriting = LOW;

  writeCount = 0;
  dataSet += 1;

  openFile = HIGH;

  // Write Dataset Number
  myFile = SD.open("dataset.csv", FILE_WRITE);
  if (myFile)
  {
    //   Serial.print("Writing to dataset.csv...");
    myFile.println(dataSet);
    // close the file:
    myFile.close();

    //  Serial.println("done.");
  }
  else
  {
    // if the file didn't open, print an error:
    // Serial.println("error opening dataset.csv");
    error = HIGH;
  }
}

void readEncoder() // Read encoder value from encoder and convert to degrees
{

  ledState = digitalRead(SYNC_LED);

  newPosition = myEnc.read();

  readMicros = micros();

  angle = ((newPosition / 4096) * 360) / 3;

  if (abs(round(angle)) == 360)
  {
    fullRev = HIGH;
  }
}

void saveEncoderData() // Save encoder position to sd card
{

  if (openFile == HIGH)
  {
    myFile = SD.open("encoder.csv", O_CREAT | O_APPEND | O_WRITE);
    openFile = LOW;
  }
  if (myFile)
  {
    myFile.print(dataSet);
    myFile.print(',');
    myFile.print(readMicros);
    myFile.print(',');
    myFile.print(newPosition);
    myFile.print(',');
    myFile.println(ledState);

    writeCount++;

    if (writeCount == 512)
    {
      Serial.println("FLUSH");
      myFile.flush();
      //  myFile.close();

      writeCount = 0;
    }
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("SAVE error opening encoder.csv");
    error = HIGH;
  }
}