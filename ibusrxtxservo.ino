#include <string.h>
#include <iBUSTelemetry.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <EEPROM.h>

//iBUS
#define UPDATE_INTERVAL 500
#define IBUS_BUFFSIZE 32
#define IBUS_MAXCHANNELS 10
iBUSTelemetry telemetry(10);
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = { 0 };
uint32_t prevMillis = 0;

//PWM & servo
static uint16_t rcValue[IBUS_MAXCHANNELS];
static boolean rxFrameDone;
int ch_width_1;
int ch_width_2;
int ch_width_3;
//int ch_width_4;
//int ch_width_5;
int ch_width_6;
int ch_width_7;
int ch_width_8;
int ch_width_9;
int ch_width_10;

//GPS
static const int RXPin = 15, TXPin = 14;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
float latitude;
float longitude;
float heading;

long coordinates[6][2];
long kordinata[6][2];
// long coordinateStore[6][2];
// long readCoordinates[6][2];
// long testCoord[2];
// long testReadCoord[2];
long coordinateStore[6][2] = {
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};
double distance;
float course;
int index = 0;
bool button8Pressed = false;
bool button10Pressed = false;

int g = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(9600);

  // Sample coordinates
  coordinates[0][0] = 476138080;
  coordinates[0][1] = 189508450;
  coordinates[1][0] = 476132940;
  coordinates[1][1] = 189504330;
  coordinates[2][0] = 476123990;
  coordinates[2][1] = 189528740;
  coordinates[3][0] = 394567800;
  coordinates[3][1] = -686543200;
  coordinates[4][0] = 385678900;
  coordinates[4][1] = -675432100;
  coordinates[5][0] = 375678900;
  coordinates[5][1] = -615432100;



  telemetry.begin();

  telemetry.addSensor(IBUS_MEAS_TYPE_S85);
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_STATUS);
  telemetry.addSensor(IBUS_MEAS_TYPE_CMP_HEAD);
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LAT);
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LON);
  telemetry.addSensor(IBUS_MEAS_TYPE_GPS_DIST);
  telemetry.addSensor(IBUS_MEAS_TYPE_COG);
}

void loop() {
  readRx();
    while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      displayInfo();

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
  updateValues();
  telemetry.run();

}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 7);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 7);
    float latitudeEP = static_cast<float>(gps.location.lat());
    float longitudeEP = static_cast<float>(gps.location.lng());
    float latitude = (gps.location.lat(), 7);
    //EEPROM.put(eepromAddressLat, 47.6138080);
    //EEPROM.put(eepromAddressLng, 18.9508450);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
  Serial.println(gps.sentencesWithFix());
  Serial.println();
  float storedLatitude, storedLongitude;
  //EEPROM.get(eepromAddressLat, storedLatitude);
  //EEPROM.get(eepromAddressLng, storedLongitude);
  Serial.print(F("Stored Latitude: "));
  Serial.print(storedLatitude, 7);
  Serial.print(F(", Stored Longitude: "));
  Serial.println(storedLongitude, 7);
  //EEPROM.get(0, readCoordinates);
  // EEPROM.get(index, testReadCoord);
  // Serial.println(EEPROM.length());
    // Calculate and print available EEPROM space
  //int availableSpace = EEPROM.length() - eepromAddressLng;
  // Serial.print(F("Available EEPROM space: "));
  // Serial.print(availableSpace);
  // Serial.println(F(" bytes"));
    // Serial.println("Beolvasott adatok:");
  // for (int i = 0; i < 6; i++) {
  //   Serial.print("P");
  //   Serial.print(i);
  //   Serial.print(": ");
  //   Serial.print(testReadCoords[i][0]);
  //   Serial.print(", ");
  //   Serial.println(testReadCoords[i][1]);
  // }
  // for (int i = 0; i<6; i++){
  //   Serial.print("P");
  //   Serial.print(i);
  //   Serial.print(": ");
  //   Serial.print(EEPROM.get(index, testReadCoord[0]));
  //   Serial.print(", ");
  //   Serial.println(EEPROM.get(index, testReadCoord[1]));
  // }
  EEPROM.get(0, coordinateStore);

  Serial.println("Adatok betöltése az EEPROM-ból:");
  for (int i = 0; i < 6; i++) {
    Serial.print("P");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(coordinateStore[i][0]);
    Serial.print(", ");
    Serial.println(coordinateStore[i][1]);
  }
}

void readRx() {
  rxFrameDone = false;

  if (Serial2.available()) {
    uint8_t val = Serial2.read();
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20) {
      ibusIndex = 0;
      return;
    }
    if (ibusIndex == 1 && val != 0x40) {
      ibusIndex = 0;
      return;
    }

    if (ibusIndex == IBUS_BUFFSIZE) {
      ibusIndex = 0;
      int high = 3;
      int low = 2;
      for (int i = 0; i < IBUS_MAXCHANNELS; i++) {
        rcValue[i] = (ibus[high] << 8) + ibus[low];
        high += 2;
        low += 2;
      }
      
      ch_width_6 = map(rcValue[5], 1000, 2000, 1000, 2000);
      ch_width_10 = map(rcValue[9], 1000, 2000, 1000, 2000);
      ch_width_8 = map(rcValue[7], 1000, 2000, 1000, 2000);
      if (ch_width_6 >= 950 && ch_width_6 <= 1050) {
        index = 0;

      } else if (ch_width_6 >= 1260 && ch_width_6 <= 1360) {
        index = 1;
      } else if (ch_width_6 >= 1390 && ch_width_6 <= 1490) {
        index = 2;
      } else if (ch_width_6 >= 1510 && ch_width_6 <= 1610) {
        index = 3;
      } else if (ch_width_6 >= 1640 && ch_width_6 <= 1740) {
        index = 4;
      } else if (ch_width_6 >= 1950 && ch_width_6 <= 2050) {
        index = 5;
      } else {
        index = -1; // Érvénytelen tartomány esetén
      }
      //Serial.println(ch_width_10);

      if(ch_width_10 > 1900 && !button10Pressed){
        // testCoord[0] = gps.location.lat()*10000000;
        // testCoord[1] = gps.location.lng()*10000000;

        // EEPROM.put(index, testCoord);
      coordinateStore[index][0] = gps.location.lat()*10000000;
      coordinateStore[index][1] = gps.location.lng()*10000000;
      EEPROM.put(0, coordinateStore);  // Módosított adatok írása az EEPROM-ba
      Serial.print("Adatok tárolva a ");
      Serial.print(index);
      Serial.println(". tömbben.");
      button10Pressed = true;
}
            if(ch_width_8 > 1900 && !button8Pressed){
        // testCoord[0] = gps.location.lat()*10000000;
        // testCoord[1] = gps.location.lng()*10000000;

        // EEPROM.put(index, testCoord);
      coordinateStore[index][0] = 0;
      coordinateStore[index][1] = 0;
      EEPROM.put(0, coordinateStore);  // Módosított adatok írása az EEPROM-ba
      button8Pressed = true;
            }


      //EEPROM.get(0, readCoordinates);

      //Serial.println("Beolvasott adatok:");
      // for (int i = 0; i < 6; i++) {
      //   Serial.print("P");
      //   Serial.print(i);
      //   Serial.print(": ");
      //   Serial.print(readCoordinates[i][0]);
      //   Serial.print(", ");
      //   Serial.println(readCoordinates[i][1]);
      // }
EEPROM.get(0, kordinata);
      distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), kordinata[index][0]/10000000.0, kordinata[index][1]/10000000.0);
      course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), kordinata[index][0]/10000000.0, kordinata[index][1]/10000000.0);
      
      rxFrameDone = true;
      return;
    } else {
      ibus[ibusIndex] = val;
      ibusIndex++;
    }
  }
  if (ch_width_8 < 1100) {
    button8Pressed = false;
  }
  if (ch_width_10 < 1100) {
    button8Pressed = false;
  }
}
void updateValues()
{
    EEPROM.get(0, coordinateStore);
    uint32_t currMillis = millis();
    if (currMillis - prevMillis >= UPDATE_INTERVAL) { // Code in the middle of these brackets will be performed every 500ms.
      prevMillis = currMillis;


      telemetry.setSensorValueFP(1, index); //S85
      //telemetry.setSensorValue(2, telemetry.gpsStateValues(3, 8)); // GPS Status
      telemetry.setSensorValue(2, gps.satellites.value());
      telemetry.setSensorValue(3, heading); // Heading
      telemetry.setSensorValue(4, coordinateStore[index][0]); //Latitude
      telemetry.setSensorValue(5, coordinateStore[index][1]); // Longitude
      //telemetry.setSensorValue(4, gps.location.lat()*10000000); //Latitude
      //telemetry.setSensorValue(5, gps.location.lng()*10000000); // Longitude
      telemetry.setSensorValue(6, distance); // Distance
      telemetry.setSensorValueFP(7, course); // COG
    }
}