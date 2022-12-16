#include <SoftwareSerial.h>
#include <Stepper.h>
#include <EEPROM.h>

const byte rxPin = 2; //Pin 2 will be used to receive the RFID code
const byte BUFFER_SIZE = 14; //RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const byte DATA_SIZE = 10; //10byte data (2byte version + 8byte tag)
const byte DATA_VERSION_SIZE = 2; //2byte version
const byte DATA_TAG_SIZE = 8; //8byte tag
const byte CHECKSUM_SIZE = 2; //2byte checksum (redundancy for error checking)
long storedTag;
byte eeAddress = 0;
long receivedTag;

SoftwareSerial ssrfid(rxPin, 8);

uint8_t buffer[BUFFER_SIZE]; //Array of bytes, used to store an incoming data frame
int buffer_index = 0;

void setup() 
{
  ssrfid.begin(9600); //Baud rate of the RFID module
  ssrfid.listen();
  pinMode(rxPin, INPUT);
  EEPROM.get(eeAddress, storedTag);
  Serial.begin(9600);
}

void loop() 
{
  receivedTag = read_tag();
  Serial.println(receivedTag);
  delay(1000);
}

void configure_new_tag() //Lights up LED, reads tag, stores tag in EEPROM, turns off LED
{
  //Light up LED
  //...
  do
  {
    storedTag = read_tag();
  }while(storedTag != 0);
  Serial.print("Succesfully configured new tag: ");
  Serial.println(storedTag);
  EEPROM.put(eeAddress, storedTag);
  //Turn off LED
}

long read_tag() //Reads the tag, returns 0 if there is an error
{
  bool tagRead = false;
  Serial.println("Reading tag...");
  while(tagRead == false)
  {
    if(ssrfid.available() > 0)
    {
      bool call_extract_tag = false;
  
      int ssvalue = ssrfid.read(); //Read one byte
      if(ssvalue == -1) //No data was read
      {
        continue;
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
        continue;
      }
  
      buffer[buffer_index++] = ssvalue; //everything is alright, copy current value to buffer
  
      if(call_extract_tag == true)
      {
        if(buffer_index == BUFFER_SIZE)
        {
          tagRead = true;
          long tag = extract_tag();
          if(tag != 0)
          {
            Serial.println("Succesfully read tag");
          }
          return tag;
        }
        else //Something is wrong... start again looking for preamble (value: 2)
        {
          buffer_index = 0;
          continue;
        }
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
