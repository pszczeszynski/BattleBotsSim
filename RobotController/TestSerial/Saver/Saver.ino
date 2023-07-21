// arduino program that dumps 1,2,3,4 to a memory location

#include <EEPROM.h>

void setup()
{
  Serial.println("Starting");
  EEPROM.write(0, 1);
  EEPROM.write(1, 2);
  EEPROM.write(2, 3);
  EEPROM.write(3, 4);
  Serial.println("Done");
}

void loop()
{

}