#include <EEPROM.h>

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting");
    for (int i = 0; i < 100; i ++)
    {
        Serial.print((char) EEPROM.read(i));
    }
    Serial.println("Done");
}

void loop()
{
    //Serial.println("hello");
}