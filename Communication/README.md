To Set Up the Teensys with all their communication and IMU Shenanigans, there are a few steps you need to follow:

1. If you haven't already, install vscode and get the platformio extension

2. Open the Communication folder in vscode (the folder this README is in) 

3. Open the platformio menu on the left side (the alien icon) and project tasks should appear
    - The `Unified` target will automatically determine whether tx/rx/position depending on ID pins. IE it can be flashed to all boards.
    - To force a board to act as a specific board regardless of ID pins the Transmitter/Receiver targets may be used.=

4. To use the debug serial interface, download tycommander [here](https://github.com/Koromix/tytools).
    - Note that it must be disabled (click on the Serial icon in the top toolbar) while uploading code to free up the interface
    - The debug serial interface may be used in parallel with the USBHID interface for transmitter boards.