To Set Up the Teensys with all their communication and IMU Shenanigans, there are a few steps you need to follow:

1. If you haven't already, install the Arduino IDE from https://www.arduino.cc/en/software

2. Now you need to install Teensyduino, to be able to program the Teensys through the Arduino IDE. 
    - Go to File > Preferences and under "Additional boards manager URLs" add the following link: https://www.pjrc.com/teensy/package_teensy_index.json. 
    - Now go to Boards Manager and search Teensy. Install "Teensy by Paul Stoffregen". He's the genius behind Teensyduino.
    - You can now program Teensys!

3. Next, install the RF24 radio library we use to communicate between the Teensys.
    - We use these radio modules: https://protosupplies.com/product/nrf24l01palna-2-4ghz-rf-wireless-module/
    - In Library Manager, search for RF24. Scroll down until you find "RF24 by TMRh20" and install the latest version. See this Arduino site for reference (The description should match) https://reference.arduino.cc/reference/en/libraries/rf24/
    - Now you can include <RF24.h> in your sketches and use the library. You can now try the sample code by the radio module manufacturer that's on the site above.

4. Now it's time to install the IMU library.
    - We use these IMUs: https://www.sparkfun.com/products/15335
    - To get readings from the IMU, we use this library: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary. You can find example code in here as well.
    - Go to the repository, click clone, then download as ZIP
    - In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library, then select the ZIP folder you just downloaded
    - Now you can include "ICM_20948.h" in your sketches and read from the IMU.
    