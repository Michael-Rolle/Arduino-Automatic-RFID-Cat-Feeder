#include <SoftwareSerial.h>
#include <Stepper.h>
#include <EEPROM.h>
#include <LowPower.h>
#include <NewPing.h>


const byte BUFFER_SIZE = 14; //RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const byte DATA_SIZE = 10; //10byte data (2byte version + 8byte tag)
const byte DATA_VERSION_SIZE = 2; //2byte version
const byte DATA_TAG_SIZE = 8; //8byte tag
const byte CHECKSUM_SIZE = 2; //2byte checksum (redundancy for error checking)
long storedTag;
byte eeAddress = 0;
long receivedTag;
bool haveReadTag;
bool configureTag;
int distance;
long duration;
bool checkProximity;
bool getTag;
bool gateOpen;
bool multipleRead;
unsigned long prevTime;
unsigned long currTime;

unsigned interruptCount;

uint8_t buffer[BUFFER_SIZE]; //Array of bytes, used to store an incoming data frame
int buffer_index = 0;

const byte rxPin = 4; //Pin 4 will be used to receive the RFID code
const byte LEDPin = 5; //Pin 5 is used for the LED
const byte configureButtonPin = 2; // Pin 2 is used as an interrupt with a push button (with a pull down resistor) to configure a new tag
const byte trigPin = 6;
const byte echoPin = 7;
const byte limitSwitchClosed = 12;
const byte limitSwitchOpen = 13;

const int maxDistance = 10000;
const int stepsPerRevolution = 2038;

NewPing sonar(trigPin, echoPin, maxDistance);
SoftwareSerial ssrfid(rxPin, 3);
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() 
{
  Serial.begin(9600);
  ssrfid.begin(9600); //Baud rate of the RFID module
  ssrfid.listen();
  
  pinMode(rxPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(configureButtonPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(limitSwitchClosed, INPUT);
  pinMode(limitSwitchOpen, INPUT);
  
  EEPROM.get(eeAddress, storedTag);
  Serial.print("Stored tag: ");
  Serial.println(storedTag);
  
  digitalWrite(LEDPin, LOW);
  digitalWrite(trigPin, LOW);
  
  haveReadTag = false;
  checkProximity = true;
  getTag = false;
  configureTag = false;
  gateOpen = false;
  interruptCount = 0;

  attachInterrupt(digitalPinToInterrupt(configureButtonPin), configure_new_tag, RISING);

  myStepper.setSpeed(10); //10 RPM

  Serial.println("Closing gate...");
  while(digitalRead(limitSwitchClosed) == LOW)
  {
    myStepper.step(-10);
  }
  while(digitalRead(limitSwitchClosed) == HIGH)
  {
    myStepper.step(10); //Move the gate slightly away from the switch to deactivate it
  }
  Serial.println("Gate Closed");

  currTime = millis();
  prevTime = currTime;
}

void loop() 
{
  if(checkProximity == true) //Main thing that will happen is checking the proximity
  {
    //delay(4000); //Check proximity every 4 seconds
    currTime = millis();

    if(currTime - prevTime > 4000)
    {
      prevTime = currTime;
      distance = measure_distance();

      if(distance <= 10 && distance != 0 && gateOpen != true)
      {
        flashLED();
        getTag = true;
      }
      if((distance >= 15 || distance == 0) && gateOpen == true)
      {
        closeGate();
      }
    }   
  }
  
  if(getTag == true)
  {
    Serial.println("Reading tag...");
    currTime = millis();
    prevTime = currTime;
    while(haveReadTag != true && configureTag != true)
    {
      read_tag_into(receivedTag);
      delay(2);
      currTime = millis();
      if(currTime - prevTime > 4000)
      {
        prevTime = currTime;
        distance = measure_distance();
        if(distance >= 15)
        {
          getTag = false;
          haveReadTag = false;
          break;
        }
        Serial.println("Reading tag...");
        flashLED();
      }
    }
    
    if(haveReadTag == true)
    {
      if(receivedTag == storedTag)
      {
        digitalWrite(LEDPin, HIGH);
        openGate();
        digitalWrite(LEDPin, LOW);
      }
      haveReadTag = false;
      clearRFID();    
    }
    getTag = false;
    checkProximity = true;
  } 
    
  if(configureTag == true) //Flag set true from interrupt
  {
    Serial.println("Configuring new tag, present tag...");
    haveReadTag = false;
    digitalWrite(LEDPin, HIGH); //Light up LED
    delay(200);
    while(haveReadTag != true)
    {
      read_tag_into(storedTag);
      delay(2);
      if(interruptCount == 2)
        break;
    }
    if(haveReadTag == true && interruptCount != 2)
    {
      Serial.print("Succesfully configured new tag: ");
      Serial.println(storedTag);
      EEPROM.put(eeAddress, storedTag);
    }
    else if(interruptCount == 2)
    {
      Serial.println("Cancelled Configuration");
      delay(200);
    }
    haveReadTag = false;
    digitalWrite(LEDPin, LOW); //Turn off LED
    configureTag = false;
    clearRFID();
  }
}


//Functions
void configure_new_tag() //Lights up LED, reads tag, stores tag in EEPROM, turns off LED
{
  if(interruptCount == 2)
  {
    interruptCount = 0;
  }
  else
  {
    interruptCount++;
  }
  configureTag = true;
}

void read_tag_into(long &placeHolder) //Reads the tag, returns 0 if there is an error
{
  if(ssrfid.available() > 0)
  {
    bool call_extract_tag = false;

    int ssvalue = ssrfid.read(); //Read one byte
    if(ssvalue == -1) //No data was read
    {
      return;
    }

    if(ssvalue == 2) //RDM6300 found a tag => tag incoming
    {
      buffer_index = 0;
    }
    else if (ssvalue == 3) //Tag has been fully transmitted
    {
      call_extract_tag = true; //Extract tag at the end of the function call
    }

    if(buffer_index >= BUFFER_SIZE) //Checking for buffer overflow
    {
      Serial.println("Error: Buffer overflow detected!");
      return;
    }

    buffer[buffer_index++] = ssvalue; //everything is alright, copy current value to buffer

    if(call_extract_tag == true)
    {
      if(buffer_index == BUFFER_SIZE)
      {
        haveReadTag = true;
        long tag = extract_tag();
        if(tag != 0)
        {
          Serial.print("Succesfully read tag: ");
          Serial.println(tag);
        }
        placeHolder = tag;
        return;
      }
      else //Something is wrong... start again looking for preamble (value: 2)
      {
        buffer_index = 0;
        return;
      }
    }
  }
}

long extract_tag()
{
  uint8_t msg_head = buffer[0];
  uint8_t *msg_data = buffer + 1; //10 byte => data contains 2byte version + 8byte tag
  uint8_t *msg_data_version = msg_data;
  uint8_t *msg_data_tag = msg_data + 2;
  uint8_t *msg_checksum = buffer + 11; //2byte
  uint8_t msg_tail = buffer[13];

  long checksum = 0;
  for(int i = 0; i < DATA_SIZE; i+= CHECKSUM_SIZE)
  {
    long val = hexstr_to_value((char*)(msg_data + i), CHECKSUM_SIZE);
    checksum ^= val;
  }
  if(checksum != hexstr_to_value((char*)msg_checksum, CHECKSUM_SIZE)) //Compare calculated checksum to retrieved checksum
  {
    Serial.println("CHECKSUMS DO NOT MATCH"); //Checksums do not match
    return 0;
  }
  
  return hexstr_to_value((char*)msg_data_tag, DATA_TAG_SIZE);
}

long hexstr_to_value(char *str, unsigned int length) //Converts HEX value (encoded as ASCII string) to a numeric value
{
  char* copy = (char*)malloc((sizeof(char) * length) + 1);
  memcpy(copy, str, sizeof(char) * length);
  copy[length] = '\0'; //The variable 'copy' is a copy of the parameter 'str'. 'Copy' has an additional '\0' element to make sure that 'str' is null-terminated.

  long value = strtol(copy, NULL, 16); //strtol converts a null-terminated string to a long value
  free(copy); //clean up
  return value;
}

int measure_distance()
{
  int dist = sonar.ping_cm();
  
  Serial.print("Distance: ");
  Serial.println(dist);

  return dist;
}

void flashLED()
{
  digitalWrite(LEDPin, HIGH);
  delay(300);
  digitalWrite(LEDPin, LOW);
}

void openGate()
{
  Serial.println("Opening gate...");
  while(digitalRead(limitSwitchOpen) == LOW)
  {
    myStepper.step(10);
  }
  while(digitalRead(limitSwitchOpen) == HIGH)
  {
    myStepper.step(-10); //Move gate slightly so that switch deactivates
  }
  Serial.println("Gate Open");
  gateOpen = true;
}

void closeGate()
{
  Serial.println("Closing gate...");
  while(digitalRead(limitSwitchClosed) == LOW)
  {
    myStepper.step(-10);
  }
  while(digitalRead(limitSwitchClosed) == HIGH)
  {
    myStepper.step(10); //Move gate slightly to deactivate the switch
  }
  Serial.println("Gate Closed");
  gateOpen = false;
}

/*void clearRFID()
{ 
  int ssvalue = 0;
  while(ssvalue != -1)
  {
    ssvalue = ssrfid.read(); //Read one byte, returns -1 if no data is read
  }
  ssvalue = ssrfid.read();
}*/

void clearRFID()
{
  currTime = millis();
  prevTime = currTime;
  byte count = 0;
  while(count != 10)
  {
    currTime = millis();
    if(currTime - prevTime > 10000) //time out in case stuck in loop
    {
      return;
    }
    ssrfid.read();
    if(ssrfid.available() == 0)
    {
      count++;
      delay(50);
    }
  }
  Serial.println("RFID is clear");
  ssrfid.read();
}
