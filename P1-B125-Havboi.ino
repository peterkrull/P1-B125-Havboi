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
void gpsPull(struct gpsData *input) {
  int pos = 0;                    // Variabel til at holde positionen i arrayet place[]
  String tempMsg = GPS.readStringUntil('\n'); // GPS output for $GPRMC linjen placeres i en tempMsg string
  String place[12]; // String array til at holde hvert formateret output
  int stringstart = 0;        // Variabel til at holde den læste position i GPS-output

  GPS.listen();

  Serial.flush(); // De serielle forbindelser flushes for bedre aflæsning
  GPS.flush();
  while (GPS.available() > 0) // Imens GPS er tilgængelig
  {
    GPS.read();
  }
  
  if (GPS.find("$GPRMC,")) // Hvis Minimum Configuration linjen findes...
  {
    for (int i = 0; i < tempMsg.length(); i++)  // For loop tæller op til antallet af karakterer i tempMsg
    {
      if (tempMsg.substring(i, i + 1) == ",") // Hvis karakteren i $GPRMC beskeden har et komma
      {
        place[pos] = tempMsg.substring(stringstart, i); // Nuværende karakter skrives ind i array
        stringstart = i + 1; // Komma springes over, noteres ikke
        pos++; // Skifter til næste position i arrayet 
      }
      if (i == tempMsg.length() - 1) // Hvis enden af tempMsg er nået
      {
        place[pos] = tempMsg.substring(stringstart, i);
      }
    }
  }

  // Kilde : http://www.davidjwatts.com/youtube/GPS-software-serial.ino

  //input -> time = place[0]; // time
  //input -> date = place[8]; // date
  input -> latitude = place[2]; // latitude
  input -> lathem = place[3]; // lat hem
  input -> longitude = place[4]; // longitude
  input -> longhem = place[5]; // long hem
}

// Formats the GPS data for sending. From string to string
void gpsFormat(struct gpsData *input,struct transmitData *output) {

    //   H900000000 - H is 1 for positive, 2 for negative   // 10 karakterer

    String latfirst, latsecond, latString, lathem;
    latfirst = input -> latitude.substring(0,4);
    latsecond = input -> latitude.substring(5,10);


         if (input -> lathem == "N") lathem = "1";
    else if (input -> lathem == "S") lathem = "2";
    

    latString = lathem + latfirst + latsecond;
    output -> latitude = latString; 

    //   H1800000000 - H is 1 for positive, 2 for negative  // 11 karakterer

    String longfirst, longsecond, longString, longhem;

    longfirst = input -> longitude.substring(0,5);
    longsecond = input -> longitude.substring(6,11);

         if (input -> longhem == "E") longhem = "1";
    else if (input -> longhem == "W") longhem = "2";

    longString = longhem + longfirst + longsecond;
    output -> longitude = longString; 
}

// Formats the voltage to a uint8 with no decimal place
uint8_t formatVolt(float voltageFloat){

  // 18.543 returns 186

  uint8_t voltage;
  voltage = (int)round(voltageFloat * 10);

  if (voltage < 16){
    voltage = 16;
  }
  return voltage;
}

// Calculates the voltage of the battery according to voltage divider 
float getVoltage(int samples = 100) {
  float voltage = 0;      // Sum af samlede spændinger
  float RE = (R2 * RA) / (R2 + RA); // erstatning for R2

  for (int i = 0; i < samples; i++) {
    voltage += analogRead(v_read_pin);
    delay(10);
  }



  voltage = 1.0035 * ((voltage / samples) * ( aref / 1024 ) / ( RE / (R1 + RE)));
  #ifndef spoofVol
 
  #endif
  #ifdef spoofVol
    voltage = spoofVol;
  #endif
  return voltage;
}

// Prints formatted and raw long, lat and voltage to serial monitor
void printOutput(struct gpsData *input, struct transmitData *output) {
  #ifdef serialPrinting
  /*
    Serial.print("date pulled: ");Serial.println(input -> date);
    Serial.print("time pulled: ");Serial.println(input -> time);
    Serial.print("time format: ");Serial.println(output -> time);
    Serial.println("");
  */
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
  uint8_t strln = stringy.length();
  uint64_t sum = 0;
    for (uint8_t i = 0 ; i < strln ; i++) {
      uint64_t power = 1;
      for (uint8_t j = 0 ; j < strln - 1 - i ; j++){
        power = power * pow;
      }
      sum += stringy.substring(i,i+1).toInt() * power;
    }
  return sum;
}

// Makes sure the recieved data is valid
bool sanityCheck(struct gpsData *input) {

  #ifdef serialPrinting
  Serial.print("Longitude length : "); Serial.println(pulled.longitude.length());
  Serial.print("Latitude  length : ");Serial.println(pulled.latitude.length());
  Serial.print("Battery voltage  : ");Serial.println(data.voltage);
  #endif 

  bool longitudeChk = false;
  bool latitudeChk = false;
  bool batteryChk = false;

  if (pulled.longitude.length() == 11)
  {
    longitudeChk = true;
  }

  if (pulled.latitude.length() == 10)
  {
    latitudeChk = true;
  }

  if (data.voltage >= 16 && data.voltage <= 254)
  {
    batteryChk = true;
  }

  if (longitudeChk == true && latitudeChk == true){// & batteryChk == true){
    #ifdef serialPrinting
    Serial.println("Sanity check passed!");
    #endif
    return true;
  } 

  else if (longitudeChk != true || latitudeChk != true){// || batteryChk != true){
    #ifdef serialPrinting
    Serial.println("Sanity check failed");
    #endif
    /*#ifdef serialPrinting
    if (longitudeChk != true){
      Serial.println("Longitude is incorrect");
    }
    if (latitudeChk != true){
      Serial.println("Latitude is incorrect");
    }
    if (batteryChk != true){
      Serial.println("Battery is out of range");
    }
    #endif*/
    return false;
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
  const uint8_t trashbit = 1;
  #endif
  #ifdef debugNoMessage
  const uint8_t trashbit = 16;
  #endif

  String msg = uint64ToString(latt) + uint64ToString(lonn) + uint64ToString(volt) + uint64ToString(alarm) + uint64ToString(trashbit);
  #ifdef serialPrinting
  Serial.print("MSG:");
  Serial.println(msg);
  Serial.println("Sending");
  #endif
  //akeru.listen();
  if (akeru.sendPayload(msg)) //Send data
  {
    #ifdef serialPrinting
    Serial.println("Message sent !");
    #endif
  }
  // akeru. not working!
}

// Checks the checksum of the GPS data (Unused, not finished)
char checkSum(String theseChars) {
  char check = 0;
  // iterate over the string, XOR each byte with the total sum:
  for (int c = 0; c < theseChars.length(); c++) {
    check = char(check ^ theseChars.charAt(c));
  } 
  // return the result
  return check;
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
  float oldX = aX;
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

  float jol = sqrt(sq((oldX - aX)/0.025) + sq((oldY - aY)/0.025) + sq((oldZ - aZ)/0.025));
  return jol;
}

// Checks wether the jolt value is out of range
void collisionCheck(uint8_t *alarm, float jolt) {
    if (jolt < jolt_max) { // FUCKING ALT GODT MAIN
      data.alarm = good_val;
      #ifdef serialPrinting
      Serial.print("No collision - Good val      : "); Serial.println(jolt);
      #endif
    }
    else if (jolt >= jolt_max) {
      data.alarm = badd_val;
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
  
  loop_delta = millis() - loop_time;
  loop_time = millis();
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

  //print_loop_time();
  //gpsTimer = millis(); // Timer sættes lig nuværende tidspunkt
  gpsTimer = millis() + 3600000 - 12000; // Timer sættes lig nuværende tidspunkt

  collisionCheck(&data.alarm, jolty()); // Checker efter kollision
  delay(25);

  if (gpsTimer > (gpsTimerPrev + 3600000*gps_interval - 10000)) { // 10 sekunder før GPS målinger, mosfet tændes
    digitalWrite(GPS_fet_pin,HIGH);
    GPS.listen();
    GPS.flush();
    GPS.read();
  }

  if (gpsTimer > gpsTimerPrev + 3600000*gps_interval || data.alarm == badd_val){ // Hvis der er gået mere end gps_interval antal minutter udføres dette
   
    uint64_t latsum = 0;
    uint64_t lonsum = 0;

    // Sætter spændingen som en værdi
    data.voltage = formatVolt(getVoltage());

    gpsTimerPrev = gpsTimer; // Forrige måling sættes lige den nuværende måling

    digitalWrite(GPS_fet_pin,HIGH);   // Giver strøm til GPS-modulet gennem mosfet
    int failed;
    for (int j = 0 ; j < gps_samp ; j ++){

      #ifdef serialPrinting
      Serial.print("Taking sample   # "); Serial.print(j+1); Serial.print(" out of "); Serial.println(gps_samp);
      Serial.print("Failed attempts # "); Serial.print(failed); Serial.print(" out of "); Serial.println(attempts);
      Serial.println("\n");
      #endif

      // Data fra GPS-modulet hentes og leveres i allData struct til nem aflæsning
      gpsPull(&pulled);
      // Hvis GPS-data opfylder basale krav
      if (sanityCheck(&pulled) == true)
      {
        // Data formateres til send-bart format
        gpsFormat(&pulled, &data);
        

        // GPS-data summeres som uint64_t variabel, og bliver konverteret til andet GPS-format
        latsum += turnToUint64(gpsConvert(data.latitude));
        lonsum += turnToUint64(gpsConvert(data.longitude));
      }
      else { // Hvis GPS-data ikke opfylder krav, startes der forfra.
        j = j - 1;
        failed++;
      }
      if (failed > attempts) {
        j = gps_samp;
        latsum = 3000000000*gps_samp;
        lonsum = 30000000000*gps_samp;
      }
    }
    digitalWrite(GPS_fet_pin,LOW);   // Giver strøm til GPS-modulet gennem mosfet

    latsum = latsum / gps_samp;
    lonsum = lonsum / gps_samp;

    // Printer GPS-data
    printOutput(&pulled, &data);

    // uint64_t sendlat = data.latitude.toInt();
    SigFoxSend( latsum , lonsum , data.voltage, data.alarm);
    failed = 0;
    aX = 0;
    aY = 0;
    aZ = 0;
  }
}

void loop2(){
  #ifdef serialPrinting
  Serial.print(""); Serial.println(getVoltage());
  #endif
}