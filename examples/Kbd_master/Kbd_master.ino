
#include <Wire.h>

#define RKP_I2C_ADDR         0x18
#define RKP_REG_KEYDEF       0x10
#define RKP_REG_KEY_PRESSED  0x20
#define RKP_NBR_KEYS         25

//
//                              {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O' }
char key_vector[RKP_NBR_KEYS] = {'9', 'E', '2', '3', '4', '0', '4', '7', '8', '9', 'D', 'C', 'C', 'D', 'E', '2', '8', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O' };
void setup() {
    delay(2000);
    while(!Serial);
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(RKP_I2C_ADDR);
    Wire.write(RKP_REG_KEYDEF);
    for (uint8_t i=0; i< RKP_NBR_KEYS; i++){
        Wire.write(key_vector[i]);
        Serial.print(key_vector[i]);
    }
    Wire.endTransmission();
    delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

   Wire.requestFrom(RKP_I2C_ADDR ,1);    
   char c = Wire.read();
   if (c) { 
       Serial.println(c);
   }   
   delay(5); 
}
