
#include <Wire.h>
#include <LIDARLite.h>


LIDARLite LIDAR;
const int ledPin = LED_BUILTIN;

const int LIDAR_enable_pin = 3;
char incoming_mode = 'T';

bool shinelamp;


int last_time;
int current_time;
int count;
uint16_t dist;

void setup()
{
  pinMode(LIDAR_enable_pin,OUTPUT);
  current_time = 0;
  count = 0;

  digitalWrite(LIDAR_enable_pin, HIGH);
  Serial.begin(115200); // Initialize serial connection to display distance readings
  LIDAR.begin(0, true); // Set configuration to default and I2C to 400 kHz
  LIDAR.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
  LIDAR.write(0x04, 0b00000100); // Use non-default reference acquisition count
  LIDAR.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)
}




void loop()
{
    //frequency test-------
    //int lidar_dist;
    //frequency_and_accuracy_test(true, lidar_dist);
    //----------------------
    
    
    if(Serial.available() > 0)
    {
        incoming_mode = Serial.read();
        Clear_UART_buffer();
    }

    if(incoming_mode == 'T')
    {
        dist = 0x0142;//distanceFast(false);
        dist = (dist<<1);
        Serial.write(0xFF);
        Serial.write(dist);
        Serial.write(dist>>8);
        
        delay(200);
       
    }
    else if(incoming_mode == 'L')
    {
        digitalWrite(ledPin, HIGH);
    }
    else if(incoming_mode == 'M')
    {
        //Single_LIDAR_measurement();
        digitalWrite(ledPin, LOW);
    }


}



//makes a single measurement
void Single_LIDAR_measurement()
{

    Serial.write(LIDAR.distance());
  
}

//clears the UART input buffer from the 
void Clear_UART_buffer()
{

    while(Serial.available() > 0 )
    {
        Serial.read();
    }
  
}

//call this function to check frequency and accuracy.
void frequency_and_accuracy_test(bool biascorrection, int lidar_dist)
{   

    lidar_dist = distanceFast(false);//LIDAR.distance(true);//
    frequency_test(lidar_dist);
    show_difference(lidar_dist);
    ++count;
  
}
long total_dist = 0;

//Check frequency
void frequency_test(int dist)
{
  total_dist += dist;
  if(count%1000 == 0)
  {
      
      current_time = millis();
      
      
      Serial.println("frequency:");
      Serial.println(1000000/(current_time - last_time));
      Serial.println("average: ");
      Serial.println(total_dist / 1000);

      total_dist = 0;
      last_time = current_time;
  }

}


int lower_limit = 0;
int higher_limit = 0;

//check highest ad lowest measurement out of 5000.
void show_difference( int LIDAR_dist)
{   
      
      if((count % 5000) == 0)
      {

          Serial.println("lowest measurement:");
          Serial.println(lower_limit);
          Serial.println("highest measurement:");
          Serial.println(higher_limit);          

          
          lower_limit = LIDAR_dist;
          higher_limit =  LIDAR_dist;


          
      }
      
      if(LIDAR_dist < lower_limit)
      {
          lower_limit = LIDAR_dist;
      }
             
      if(LIDAR_dist > higher_limit)
      {
           higher_limit = LIDAR_dist;
      }

   
            
}

   

    






//Returns LIDAR values very fast.
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
