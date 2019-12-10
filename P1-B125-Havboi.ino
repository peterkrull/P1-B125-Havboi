#include "SoftwareSerial.h"
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include "Akeru.h"

#define GPS_fet_pin 2
#define v_read_pin A0

#define good_val 16
#define badd_val 17

#define aref 5.05
#define jolt_val 5000
#define gps_samp 60
#define gps_interval 1 // Hours between GPS capture

// Et delay under 10 sender ikke
#define delayMinutes 10

#define R1 383.0
#define R2 100.0

#define serialPrinting

uint8_t sanity_val = 0;

int loop_time = 0;
int loop_delta = 0;

long accelX, accelY, accelZ;
float aX, aY, aZ, LSB;
long gpsTimer = 0; 
long gpsTimerPrev = 0; 

SoftwareSerial  GPS      (10,9);
Akeru           akeru    (4, 5);

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

// Trækker data ud af GPS-modulets serielle output og placerer det i et struct
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

// Formaterer GPS data til afsending. Fra streng til streng
void gpsFormat(struct gpsData *input,struct transmitData *output, float voltageFloat) {

    //              SECTION FOR FORMATTING TIME             //
    //                    YY-MM-DD-HH-MM                    //
    // Raw date format is : DD-MM-YY
    // Raw time format is : HH-MM-SS
    /*
    String day, mon, yea;
    String hou, min, sec;
    
    day = input -> date.substring(0,2);
    mon = input -> date.substring(2,4);
    yea = input -> date.substring(4,6);

    hou = input -> time.substring(0,2);
    min = input -> time.substring(2,4);
    sec = input -> time.substring(4,6);

    String dateString = yea + mon + day + hou + min;

    output -> time = dateString; 
    */
    //            SECTION FOR FORMATTING LATITUDE           //
    //   H900000000 - H is 1 for positive, 2 for negative   // 10 karakterer

    String latfirst, latsecond, latString, lathem;
    latfirst = input -> latitude.substring(0,4);
    latsecond = input -> latitude.substring(5,10);


         if (input -> lathem == "N") lathem = "1";
    else if (input -> lathem == "S") lathem = "2";
    

    latString = lathem + latfirst + latsecond;
    output -> latitude = latString; 

    //            SECTION FOR FORMATTING LONGITUDE          // 
    //   H1800000000 - H is 1 for positive, 2 for negative  // 11 karakterer

    String longfirst, longsecond, longString, longhem;

    longfirst = input -> longitude.substring(0,5);
    longsecond = input -> longitude.substring(6,11);

         if (input -> longhem == "E") longhem = "1"; //Serial.println("lathem is 1");
    else if (input -> longhem == "W") longhem = "2"; //Serial.println("lathem is 2");

    longString = longhem + longfirst + longsecond;
    output -> longitude = longString; 

    //             SECTION FOR FORMATTING VOLTAGE           // 
    //            SECTION FOR FORMATTING LONGITUDE          // 3 karakterer

    uint8_t voltage;
    voltage = (int)round(voltageFloat * 10);

    if (voltage < 100){
      voltage = 100;
    }
    output -> voltage = voltage; 
}

// Udregner spænding på batteriet, ud fra et gennemsnit af målinger 
float getVoltage(int samples = 100) {
  float voltage = 0;      // Sum af samlede spændinger
  uint16_t RA = 1000000;  // Intern modstand i Arduino
  float RE = (R2 * RA) / (R2 + RA); // erstatning for R2

  for (int i = 0; i < samples; i++) {
    voltage += analogRead(v_read_pin);
    delay(10);
  }
  voltage = 1.0035 * ((voltage / samples) * ( aref / 1024 ) / ( RE / R1 + RE));
  return voltage;
}

// Printer den hentede information til den serielle monitor
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

// Konverterer en streng bestående af tal om til en uint64
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

// Konverterer en streng bestående af tal om til en int
int turnToInt(String stringy, uint8_t strln, uint8_t pow = 10) {
  int sum = 0;
    for (uint8_t i = 0 ; i < strln ; i++) {
      int power = 1;
      for (uint8_t j = 0 ; j < strln - 1 - i ; j++){
        power = power * pow;
      }
      sum += stringy.substring(i,i+1).toInt() * power;
    }
    return sum;
}

// Kontrolerer at den modtagede data giver mening
bool sanityCheck(struct gpsData *input) {
  bool longitudeChk = false;
  bool latitudeChk = false;
  bool batteryChk = false;

  if (pulled.longitude.length() == 11)
  {
    longitudeChk = true;
    #ifdef serialPrinting
    Serial.print("Longitude : "); Serial.println(pulled.longitude.length());
    #endif
  }

  if (pulled.latitude.length() == 10)
  {
    latitudeChk = true;
    #ifdef serialPrinting
    Serial.print("Latitude : ");Serial.println(pulled.latitude.length());
    #endif
  }

  if (data.voltage > 90 && data.voltage < 210 || true)
  {
    batteryChk = true;
    #ifdef serialPrinting
    Serial.print("Battery : ");Serial.println(data.voltage);
    #endif
  }

  if (longitudeChk == true && latitudeChk == true & batteryChk == true){
    #ifdef serialPrinting
    Serial.println("Sanity check passed!");
    #endif
    return true;
  }
  

  else if (longitudeChk != true || latitudeChk != true || batteryChk != true){
    #ifdef serialPrinting
    Serial.println("Sanity check failed");
    if (longitudeChk != true){
      Serial.println("Longitude is incorrect");
      Serial.print("Longitude string length : "); Serial.println(pulled.longitude.length());
    }
    if (latitudeChk != true){
      Serial.println("Latitude is incorrect");
      Serial.print("Latitude string length : "); Serial.println(pulled.latitude.length());
    }
    if (batteryChk != true){
      Serial.println("Battery is out of range");
      Serial.print("Battery string length : "); Serial.println(pulled.longitude.length());
    }
    #endif
    return false;
  }
}

// Konverterer en uint64 til en hex streng
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

// Initialiserer SigFox
void SigFoxSetup() {
  // Check TD1208 communication
  if (!akeru.begin()) //If not working send KO
  {
    Serial.println("TD1208 KO");
    while (1);
  }
  //akeru.echoOn(); // uncomment this line to see AT commands
}

// Sender den formaterede data til SigFox-modulet
void SigFoxSend(uint64_t latt, uint64_t lonn, uint8_t volt, uint8_t alarm) {
  //Convert all data to hex strings and combine them in msg
  #ifdef serialPrinting
  Serial.print("Hex latitude  : "); Serial.println(uint64ToString(latt));
  Serial.print("Hex longitude : "); Serial.println(uint64ToString(lonn));
  Serial.print("Hex voltage   : "); Serial.println(uint64ToString(volt));
  Serial.print("Hex alarm     : "); Serial.println(uint64ToString(alarm));
  #endif
  const uint8_t trashbit = 1;
  String msg = uint64ToString(latt) + uint64ToString(lonn) + uint64ToString(volt) + uint64ToString(alarm) + uint64ToString(trashbit);
  #ifdef serialPrinting
  //akeru.listen();
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

// Bruges ikke
char checkSum(String theseChars) {
  char check = 0;
  // iterate over the string, XOR each byte with the total sum:
  for (int c = 0; c < theseChars.length(); c++) {
    check = char(check ^ theseChars.charAt(c));
  } 
  // return the result
  return check;
}

// Initialiserer MPU
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

// Registrerer data fra MPU'en
void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
}

// Udregner en jolt-værdi for ud fra MPU-data
float jolty() {
  float oldX = aX;
  float oldY = aY;
  float oldZ = aZ;
  // Converts from raw data to g-force
  float gForceX = accelX / LSB;
  float gForceY = accelY / LSB;
  float gForceZ = accelZ / LSB;
  // Converts from g-force to acceleration
  aX = gForceX * 9.82;
  aY = gForceY * 9.82;
  aZ = gForceZ * 9.82;

  float acc = sqrt(sq(oldX - aX) + sq(oldY - aY) + sq(oldZ - aZ));

  return acc;
}

// Ændrer alarm variblen for et jolt
void collisionCheck(uint8_t *alarm) {
  float jolt = jolty();
    if (jolt < jolt_val) { // FUCKING ALT GODT MAIN
      data.alarm = good_val;
    }
    else if (jolt >= jolt_val) {
      data.alarm = badd_val;
    }
}

// Konverterer GPS data fra DMS til DDS
String gpsConvert(String gpsData) {
  String temp1;
  String temp2;
  temp1 = String((gpsData.substring(4,gpsData.length()+1).toFloat()/60.0));
  for(uint8_t i = 0; i < gpsData.length(); i++)
  {
    if(temp1.substring(i,i+1) == ".")
    {
      //Serial.println("1");
      temp2 = temp1.substring(0,i);
      temp2 += temp1.substring(i+1,i+3);
    } 
  }
  return(gpsData.substring(0,4) + temp2); 
}

void setup() {
    #ifdef serialPrinting
    Serial.begin(115200);
    #endif
    Wire.begin();
    GPS.begin(9600);
    pinMode(v_read_pin,INPUT);        // Pin bruges
    pinMode(GPS_fet_pin,OUTPUT);      // Pin bruges
    digitalWrite(GPS_fet_pin,LOW);    // Giver strøm til GPS-modulet gennem mosfet
    setupMPU();
    //SigFoxSetup();
}

void loopori() {  
  uint64_t latsum = 0;
  uint64_t lonsum = 0;

  // Registrerer MPU data
  //recordAccelRegisters();

  // Kontrolerer MPU data
  collisionCheck(&data.alarm);
  for (int j = 0 ; j < gps_samp ; j ++){
    // Data fra GPS-modulet hentes og leveres i allData struct til nem aflæsning
    gpsPull(&pulled);

    // Hvis GPS-data opfylder basale krav
    if (sanityCheck(&pulled) == true)
    {
      // Data formateres til send-bart format
      gpsFormat(&pulled, &data, getVoltage() );

      // GPS-data summeres som uint64_t variabel, og bliver konverteret til andet GPS-format
      latsum += turnToUint64(gpsConvert(data.latitude));
      lonsum += turnToUint64(gpsConvert(data.longitude));
      #ifdef serialPrinting
      Serial.print("Sample nr. : "); Serial.println(j);
      #endif
    }
    else { // Hvis GPS-data ikke opfylder krav, startes der forfra.
      j = -1;
    }
  }
  // gennemsnittet findes ved at dividere summen med antal samples
  latsum = latsum / gps_samp;
  lonsum = lonsum / gps_samp;

  // Hvis GPS-data er korrekt
  if (sanityCheck(&pulled) == true)
  {
      

      // Venter 10 minutter
      for (int sec = 0 ; sec < delayMinutes*60 ; sec ++){
        delay(1000);
    }
  }
  else { // Hvis sanityCheck fejler gøres intet. 
    #ifdef serialPrinting
    Serial.println("Sanity check failed!");
    Serial.print("Error code : "); Serial.println(sanity_val);
    #endif

    delay(5000);
  }
}

void print_loop_time(){
  loop_delta = millis() - loop_time;
  loop_time = millis();
  Serial.print("Loop time : "); Serial.print(loop_delta); Serial.println(" ms");
}

void loop() {

print_loop_time();

  gpsTimer = millis() + 3600000 - 12000; // Timer sættes lig nuværende tidspunkt

  collisionCheck(&data.alarm); // Checker efter kollision
Serial.println("gpsTimer     : "); Serial.println(gpsTimer);
Serial.println("gpsTimerPrev : "); Serial.println(gpsTimerPrev);

  if (gpsTimer > (gpsTimerPrev + 3600000*gps_interval - 10000)) { // 10 sekunder før GPS målinger, mosfet tændes
    digitalWrite(GPS_fet_pin,HIGH);
    GPS.listen();
    GPS.flush();
    GPS.read();
  }

  if (gpsTimer > gpsTimerPrev + 3600000*gps_interval || data.alarm == badd_val){ // Hvis der er gået mere end gps_interval antal minutter udføres dette
   
  uint64_t latsum = 0;
  uint64_t lonsum = 0;

    gpsTimerPrev = gpsTimer; // Forrige måling sættes lige den nuværende måling

    digitalWrite(GPS_fet_pin,HIGH);   // Giver strøm til GPS-modulet gennem mosfet

    for (int j = 0 ; j < gps_samp ; j ++){
    // Data fra GPS-modulet hentes og leveres i allData struct til nem aflæsning
      gpsPull(&pulled);

      // Hvis GPS-data opfylder basale krav
      if (sanityCheck(&pulled) == true)
      {
        // Data formateres til send-bart format
        gpsFormat(&pulled, &data, data.voltage);

        // GPS-data summeres som uint64_t variabel, og bliver konverteret til andet GPS-format
        latsum += turnToUint64(gpsConvert(data.latitude));
        lonsum += turnToUint64(gpsConvert(data.longitude));
      }
      else { // Hvis GPS-data ikke opfylder krav, startes der forfra.
        j = j - 1;
      }
    }
    digitalWrite(GPS_fet_pin,LOW);   // Giver strøm til GPS-modulet gennem mosfet

    latsum = latsum / gps_samp;
    lonsum = lonsum / gps_samp;

    // Printer GPS-data
    printOutput(&pulled, &data);

    // Sætter spændingen som en værdi
    data.voltage = getVoltage();

    // uint64_t sendlat = data.latitude.toInt();
    SigFoxSend( latsum , lonsum , data.voltage, data.alarm);
  }
  else { // Hvis sanityCheck fejler gøres intet. 
    #ifdef serialPrinting
    Serial.println("Not ready to send");
    Serial.print("Error code : "); Serial.println(sanity_val);
    #endif
  }
}