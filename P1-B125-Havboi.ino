#include "SoftwareSerial.h"
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include "Akeru.h"

// DEBUG SECTION //

//#define debugNoMessage  // Enables no sending mode    Undefine to disable
#define serialPrinting  // Enables serial printing    Undefine to disable
//#define spoofVol 10.57  // Sets a custom voltage      Undefine to disable

// END OF DEBUG //

// CONFIG SECTON //

#define GPS_fet_pin 2   // Pin for GPS switch fet
#define v_read_pin A0   // Pin for battery readout

#define batLowest 10
#define batWarning 12

#define good_val 16     // Value for no anomaly
#define badd_val 17     // Value for detected collision

#define aref 5.02       // Analog reference voltage
#define jolt_max 2000   // Max allowed jolt for bouey
#define gps_samp 60     // GPS samples for each recording
#define attempts 60     // Allowed failed attempts at GPS lock
#define gps_interval 24 // Hours between GPS phone-home

#define R1 38.3         // R1 resistance in k Ohm 
#define R2 10.0         // R2 resistance in k Ohm
#define RA 100000       // ADC resistance in k Ohm

// END OF CONFIG //

// INITIALIZATION OF COMMS //

SoftwareSerial  GPS      (10,9);
Akeru           akeru    (4, 5);

// END OF INITIALIZATION //

// GLOBAL VARIABLES //

uint8_t sanity_val = 0;

int loop_time = 0;
int loop_delta = 0;

long gpsTimer = 0; 
long gpsTimerPrev = 0; 

float aX, aY, aZ, LSB;

struct transmitData {
    String time;
    String latitude;
    String longitude;
    uint8_t voltage;
    uint8_t alarm;
} data;

struct gpsData{
    //String time;
    //String date;
    String latitude;
    String lathem;
    String longitude;
    String longhem;
} pulled;

// END OF GLOBAL VALS //

// Pulls the GPS data and places it in gpsData struct variable
void gpsPull(struct gpsData *output) {
  int pos = 0;                    // Variable to hold position
  String tempMsg = GPS.readStringUntil('\n'); // Temporary output for $GPRMC
  String place[12]; // String array to hold each entry
  int stringstart = 0;        // Variable to hold start location of string

  GPS.listen(); // Listen to GPS serial connection
  GPS.flush(); // Serial connection is flushed
  while (GPS.available() > 0) // While GPS is available
  {
    GPS.read(); // Read entire output
  }
  
  if (GPS.find("$GPRMC,")) // If Minimum Configuration is present...
  {
    for (int i = 0; i < tempMsg.length(); i++)  // For-loop to go through each character
    {
      if (tempMsg.substring(i, i + 1) == ",") // If the next character in $GPRMC is a comma
      {
        place[pos] = tempMsg.substring(stringstart, i); // Substring is written to array
        stringstart = i + 1; // Comma is skipped by increasing i by 1
        pos++; // Switch to next position in array 
      }
      if (i == tempMsg.length() - 1) // If end of message is reached
      {
        place[pos] = tempMsg.substring(stringstart, i); // Substring is written to array
      }
    }
  }

  // Source : http://www.davidjwatts.com/youtube/GPS-software-serial.ino
  // Finally relevant strings are exported using pointers
  output -> latitude = place[2];   // latitude
  output -> lathem = place[3];     // lat hem
  output -> longitude = place[4];  // longitude
  output -> longhem = place[5];    // long hem
}

// Formats the GPS data for sending. From string to string
void gpsFormat(struct gpsData *input,struct transmitData *output) {

    //   H900000000 - H is 1 for positive, 2 for negative   // 10 chars

    String latfirst, latsecond, latString, lathem; // Substrings
    latfirst = input -> latitude.substring(0,4); // Get numbers before comma
    latsecond = input -> latitude.substring(5,10); // Get numbers after comma


         if (input -> lathem == "N") lathem = "1"; // If north, number is 1
    else if (input -> lathem == "S") lathem = "2"; // If south, number is 2
    

    latString = lathem + latfirst + latsecond; // numbers are combined
    output -> latitude = latString;  // Exported to output

    //   H1800000000 - H is 1 for positive, 2 for negative  // 11 chars

    String longfirst, longsecond, longString, longhem; // Substrings
    longfirst = input -> longitude.substring(0,5); // Get numbers before comma
    longsecond = input -> longitude.substring(6,11); // Get numbers after comma

         if (input -> longhem == "E") longhem = "1"; // If east, number is 1
    else if (input -> longhem == "W") longhem = "2"; // If west, number is 2

    longString = longhem + longfirst + longsecond; // numbers are combined
    output -> longitude = longString; // Exported to output
}

// Formats the voltage to a uint8 with no decimal place
uint8_t formatVolt(float voltageFloat){
  uint8_t voltage; // variable to hold final voltage
  voltage = (int)round(voltageFloat * 10); // voltage is convrted so xx.yy becomes xxy

  if (voltage < 16){ // Is voltage is less than 16 (due to lowest 2 char hex value)
    voltage = 16; // set to 16
  }
  return voltage; // return voltage
}

// Calculates the voltage of the battery according to voltage divider 
float getVoltage(int samples = 100) {
  float voltage = 0;      // Sum of voltage average
  float RE = (R2 * RA) / (R2 + RA); // Replacement resistance for R2

  for (int i = 0; i < samples; i++) { // For loop to do multiple samples
    voltage += analogRead(v_read_pin); // Analog value is added to itself
    delay(10); // slight delay for stability
  }

  #ifndef spoofVol
  voltage = 1.0035 * ((voltage / samples) * ( aref / 1024 ) / ( RE / (R1 + RE))); // Voltage divider calc and ADC conv
  #endif
  #ifdef spoofVol
    voltage = spoofVol; // Allows for voltage spoofing for testing purposes
  #endif
  return voltage;
}

// Prints formatted and raw long, lat and voltage to serial monitor
void printOutput(struct gpsData *input, struct transmitData *output) {
  #ifdef serialPrinting
    Serial.print("lat pulled: ");Serial.println(input -> latitude);
    Serial.print("lat format: ");Serial.println(output -> latitude);
    Serial.println("");
  
    Serial.print("long pulled: ");Serial.println(input -> longitude);
    Serial.print("long format: ");Serial.println(output -> longitude);
    Serial.println("");

    Serial.print("Raw voltage:  ");Serial.println(getVoltage());
    Serial.print("form voltage: ");Serial.println(output -> voltage);
    Serial.println("");
    Serial.println("");  
  #endif
}

// Converts a number string to uint64
uint64_t turnToUint64(String stringy, uint8_t pow = 10) {
  uint8_t strln = stringy.length(); // Note length of input string
  uint64_t sum = 0; // Sum used to hold output number
    for (uint8_t i = 0 ; i < strln ; i++) { // For loop for each number in string
      uint64_t power = 1; // Power of 1 is default
      for (uint8_t j = 0 ; j < strln - 1 - i ; j++){ // Power is dependant on position of number
        power = power * pow; // power is multiplied by (10) set amount of times
      }
      sum += stringy.substring(i,i+1).toInt() * power; // Numbers are sequentially added together
    }
  return sum; // return final sum
}

// Makes sure the recieved data is valid
bool sanityCheck(struct gpsData *input) {

  #ifdef serialPrinting
  Serial.print("Longitude length : "); Serial.println(pulled.longitude.length());
  Serial.print("Latitude  length : ");Serial.println(pulled.latitude.length());
  Serial.print("Battery voltage  : ");Serial.println(data.voltage);
  #endif 

  bool longitudeChk = false; // sanity check failed by defailt
  bool latitudeChk = false;
  bool batteryChk = false;

  if (pulled.longitude.length() == 11) // If longitude is of length 11.. 
  {
    longitudeChk = true; //.. it passes the test
  }

  if (pulled.latitude.length() == 10) // If latitude is of length 10.. 
  {
    latitudeChk = true; //.. it passes the test
  }

  if (data.voltage >= 16 && data.voltage <= 255) //if converted voltage is between 16 and 255
  {
    batteryChk = true; //.. it passes the test
  }

  if (longitudeChk == true && latitudeChk == true){ // if all checks pass..
    #ifdef serialPrinting
    Serial.println("Sanity check passed!");
    #endif
    return true; // ..return true
  } 

  else if (longitudeChk != true || latitudeChk != true){ // if some checks do not pass..
    #ifdef serialPrinting
    Serial.println("Sanity check failed");
    #endif
    return false; // ..return false
  }
}

// Converts a uint64 to a string of hex numbers
String uint64ToString(uint64_t input) {
  String result = "";
  uint8_t base = 16;
  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

// Initializes SigFox (Unused)
void SigFoxSetup() {
  // Check TD1208 communication
  if (!akeru.begin()) //If not working send KO
  {
    #ifdef serialPrinting
    Serial.println("TD1208 KO");
    #endif
    while (1);
  }
  //akeru.echoOn(); // uncomment this line to see AT commands
}

// Sends the formated values using the SigFox module
void SigFoxSend(uint64_t latt, uint64_t lonn, uint8_t volt, uint8_t alarm) {
  //Convert all data to hex strings and combine them in msg
  #ifdef serialPrinting
  Serial.print("Hex latitude  : "); Serial.println(uint64ToString(latt));
  Serial.print("Hex longitude : "); Serial.println(uint64ToString(lonn));
  Serial.print("Hex voltage   : "); Serial.println(uint64ToString(volt));
  Serial.print("Hex alarm     : "); Serial.println(uint64ToString(alarm));
  #endif

  #ifndef debugNoMessage
  const uint8_t trashbit = 1; // bit of 1 allows for correct formatting
  #endif
  #ifdef debugNoMessage
  const uint8_t trashbit = 16; // bit of 16 cant send. Wrong format
  #endif

  // Final message consists of all data formatted using uint64 to hex-string function
  String msg = uint64ToString(latt) + uint64ToString(lonn) + uint64ToString(volt) + uint64ToString(alarm) + uint64ToString(trashbit);
  #ifdef serialPrinting
  Serial.print("MSG:");
  Serial.println(msg); // Print final message
  Serial.println("Sending");
  #endif
  //akeru.listen();
  if (akeru.sendPayload(msg)) //Send data
  {
    #ifdef serialPrinting
    Serial.println("Message sent !");
    #endif
  }
}

// Initializes the MPU
void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  //Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.write(0b00101000); //Setting CYCLE register to 1 (Power save mode), and disables temp sensor
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x6C); //Accessing the register 6C - Power Management 2 (Sec. 4.29)
  //Wire.write(0b00000111); //Setting FREQ register to 1.25 Hz, and disables gyroscope
  //Wire.write(0b01000111); //Setting FREQ register to 5 Hz, and disables gyroscope
  //Wire.write(0b10000111); //Setting FREQ register to 20 Hz, and disables gyroscope
  Wire.write(0b11000111); //Setting FREQ register to 40 Hz, and disables gyroscope
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  //Wire.write(0b00000000); LSB = 16384.0; //Setting the accel to +/- 2g
  //Wire.write(0b00001000); LSB = 8192.0; //Setting the accel to +/- 4g
  //Wire.write(0b00010000); LSB = 4096.0; //Setting the accel to +/- 8g
  Wire.write(0b00011000); LSB = 2048.0; //Setting the accel to +/- 16g
  Wire.endTransmission();
}

// Calculates a Jolt value from the MPU
float jolty() {
  float oldX = aX; // sets previous values in their own variables
  float oldY = aY;
  float oldZ = aZ;

  // Pulls accelerometer registers
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  long accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  long accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  long accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ

  // Converts from g-force to acceleration
  aX = (accelX / LSB) * 9.82;
  aY = (accelY / LSB) * 9.82;
  aZ = (accelZ / LSB) * 9.82;

  // Length of Jolt vector is determined
  float jolt = sqrt(sq((oldX - aX)/0.025) + sq((oldY - aY)/0.025) + sq((oldZ - aZ)/0.025));
  return jolt; // return value length of jolt
}

// Checks wether the jolt value is out of range
void collisionCheck(uint8_t *alarm, float jolt) {
    if (jolt < jolt_max) { // Everything good
      data.alarm = good_val; // alarm value is set to bad value
      #ifdef serialPrinting
      Serial.print("No collision - Good val      : "); Serial.println(jolt);
      #endif
    }
    else if (jolt >= jolt_max) { // Jolt value is too high
      data.alarm = badd_val; // alarm value is set to good value
      for (int i = 0 ; i < 5 ; i++){
        #ifdef serialPrinting
        Serial.print("COLLISION DETECTED - BAD VAL : "); Serial.println(jolt);
        #endif
      }
    }
}

// Converts GPS data from DMS (degree,minute,second) to DDM (degree,decimal minute)
String gpsConvert(String gpsData) {
  String temp1; //Create temporary string
  String temp2; //Create temporary string
  
  if(gpsData.length() == 10)      //Check if the string is lattitude
  {
    //Cut out minutes from the string, convert to float, divide by 6.000.000 then convert to string                                                        
    temp1 = String((gpsData.substring(3,gpsData.length()+1).toFloat()/6000000),7);
  }
  else if(gpsData.length() == 11) //Check if the string is longtitude
  {
    //Cut out minutes from the string, convert to float, divide by 6.000.000 then convert to string
    temp1 = String((gpsData.substring(4,gpsData.length()+1).toFloat()/6000000),7);
  }

  //Count i up to check where "." is in the string.
  for(uint8_t i = 0; i < gpsData.length(); i++) 
  {
    if(temp1.substring(i,i+1) == ".")   //Check if the i placement in the string is "."
    {
      temp2 = temp1.substring(i-1,i+8); //Cut off "." and everything before it 
    } 
  }

  if(gpsData.length() == 10)      //Check if the string is lattitude
  {
    //Combine and return the first 2 places from the original string with the 7 places from the converted minutes
    return(gpsData.substring(0,3) + temp2.substring(2,10)); 
  }
  else if(gpsData.length() == 11) //Check if the string is longtitude
  {
    //Combine and return the first 2 places from the original string with the 7 places from the converted minutes
    return(gpsData.substring(0,4) + temp2.substring(2,10));
  }
}

// Calculates and prints the loop time to the console
void print_loop_time(){
  
  loop_delta = millis() - loop_time; // Calculates difference between current time and previous
  loop_time = millis(); // Determine new current time
  #ifdef serialPrinting
  Serial.print("Loop time : "); Serial.print(loop_delta); Serial.println(" ms");
  #endif
}

// Program setup
void setup() {

  #ifdef serialPrinting
  Serial.begin(115200);             // Begins serial comms if enabled
  #endif

  Wire.begin();                     // Wire used wit accelerometer
  setupMPU();                       // Setup for accelerometer

  GPS.begin(9600);                  // Sets baud rate for GPS comms
  pinMode(GPS_fet_pin,OUTPUT);      // GPS fet pin set as output
  digitalWrite(GPS_fet_pin,LOW);    // Sets the GPS fet pin low

  pinMode(v_read_pin,INPUT);        // Pin bruges

  SigFoxSetup();                    // Does nothing lmao
}

// Program loop (MAIN LOOP)
void loop() {
  gpsTimer = millis(); // Time variable used to determine when to activate GPS

  collisionCheck(&data.alarm, jolty()); // Checks for collision
  delay(25); // 25 milliseconds is required for the 40hz MPU

  // 10 seconds before GPS measurements, mosfet is switched in
  if (gpsTimer > (gpsTimerPrev + 3600000*gps_interval - 10000)) { 
    digitalWrite(GPS_fet_pin,HIGH);
    GPS.listen();
    GPS.flush();
    GPS.read();
  }

  // If the timer has passed gps_interval minutes or a jolt is detected, collect data
  if (gpsTimer > gpsTimerPrev + 3600000*gps_interval || data.alarm == badd_val){ 
   
    uint64_t latsum = 0;
    uint64_t lonsum = 0;

    // voltage value is set to formatted voltage
    data.voltage = formatVolt(getVoltage());

    gpsTimerPrev = gpsTimer; // Previous GPS time is set to current time

    digitalWrite(GPS_fet_pin,HIGH);   // Mosfet is switched on (if it wasnt previously)
    int failed = 0;
    for (int j = 0 ; j < gps_samp ; j ++){ // Multiple samples are collected gps_samp times

      #ifdef serialPrinting
      Serial.print("Taking sample   # "); Serial.print(j+1); Serial.print(" out of "); Serial.println(gps_samp);
      Serial.print("Failed attempts # "); Serial.print(failed); Serial.print(" out of "); Serial.println(attempts);
      Serial.println("\n");
      #endif

      // Data from GPS is pulled and placed in struct
      gpsPull(&pulled);
      // If data passes sanity check
      if (sanityCheck(&pulled) == true)
      {
        // Data is formatted
        gpsFormat(&pulled, &data);
        
        // GPS data is added together
        latsum += turnToUint64(gpsConvert(data.latitude));
        lonsum += turnToUint64(gpsConvert(data.longitude));
      }
      else { // If GPS checks fail..
        j = j - 1;
        failed++; // .. fail counter goes up
      }
      if (failed > attempts) { // If failed attemps surpass limit
        j = gps_samp; // For loop is broken
        latsum = 3000000000*gps_samp; // latitude is set to unreal value
        lonsum = 30000000000*gps_samp; // longitude is set to unreal value
      }
    }
    digitalWrite(GPS_fet_pin,LOW);   // Switches off mosfet to disable GPS

    latsum = latsum / gps_samp; // average of latitude measurements is taken
    lonsum = lonsum / gps_samp; // average of longitude measurements is taken

    // Prints the GPS data
    printOutput(&pulled, &data);

    // All data is sent using Sigfox
    SigFoxSend( latsum , lonsum , data.voltage, data.alarm);
    failed = 0; // Failed attempts is reset
    aX = 0; // acceleration values work better if reset
    aY = 0;
    aZ = 0;
  }
}