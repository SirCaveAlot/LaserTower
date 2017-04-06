
#include <Wire.h>
#include <LIDARLite.h>


LIDARLite LIDAR;
const int ledPin = LED_BUILTIN;
bool shinelamp;


int last_time;
int current_time;
int count;
int dist;

void setup()
{

  current_time = 0;
  count = 0;
  Serial.begin(115200); // Initialize serial connection to display distance readings
  LIDAR.begin(0, true); // Set configuration to default and I2C to 400 kHz
  LIDAR.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
  LIDAR.write(0x04, 0b00000100); // Use non-default reference acquisition count
  LIDAR.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)
}



void loop()
{
   frequency_test();   

   dist = distanceFast(true);

   //Serial.write(dist);
}





void frequency_test()
{
  if(count%1000 == 0)
  {

      current_time = millis();

      
      Serial.println("frequency:");
      Serial.println(1/(current_time-last_time));
      last_time = current_time;
  }
  
   ++count;
}






int distanceFast(bool biasCorrection)
{
  byte isBusy = 1;
  int distance;
  int loopCount;

  // Poll busy bit in status register until device is idle
  while(isBusy)
  {
    // Read status register
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
    isBusy = Wire.read();
    isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

    loopCount++; // Increment loop counter
    // Stop status register polling if stuck in loop
    if(loopCount > 9999)
    {
      break;
    }
  }

  // Send measurement command
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0X00); // Prepare write to register 0x00
  if(biasCorrection == true)
  {
    Wire.write(0X04); // Perform measurement with receiver bias correction
  }
  else
  {
    Wire.write(0X03); // Perform measurement without receiver bias correction
  }
  Wire.endTransmission();

  // Immediately read previous distance measurement data. This is valid until the next measurement finishes.
  // The I2C transaction finishes before new distance measurement data is acquired.
  // Prepare 2 byte read from registers 0x0f and 0x10
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0x8f);
  Wire.endTransmission();

  // Perform the read and repack the 2 bytes into 16-bit word
  Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
  distance = Wire.read();
  distance <<= 8;
  distance |= Wire.read();

  // Return the measured distance
  return distance;
}
