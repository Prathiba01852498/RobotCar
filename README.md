# RobotCar
/Self driving robot car
/**Aim**:To design a robot car which can drive itself surrounding an object such as a trash can or one gallon milk bottle 
(1) Use the Arduino to control the two DC motors to drive the car moving forward, backward,
turn left, or turn right.
(2) Use the ultrasonic distance sensor to measure the distance between the car and a
nearby object such as a trash can.
(3) Implement a control algorithm to drive the robot car such that the car cycles around a
trash can (or other object placed on the floor) but never touches it. You can refer to the
demo video (reference [5]).
(4) use the sound sensor module to sample pre-recorded tones and convert the sound to
digital values
(5) use FFT library to process the audio samples and detect peak frequencies
(6) compare the peak frequencies detected against know patterns. The pattern to match is
music notes “C4” and “A4” (with frequency of 262Hz, 440Hz, respectively). If the note is
“C4”, stop the robot car. If the note is “A4”, allow the robot car to continue its movement.
To match with the frequencies, an error of 2% is allowed. For example, you can
recognize the note as C4 if the peak frequency is in the range of between 257Hz and
267Hz.



**MATERIALS REQUIRED:**
| Part Name                                                                     |Quantity   |
|-------------------------------------------------------------------------------|-----------|
|                                                                               |           |
| Arduino board                                                                 | 1         |
| DC motor                                                                      | 1         |
| Power supply module                                                           | 1         |
| L293D                                                                         | 1         |
| Button                                                                        | 1         |
| Sound module                                                                  | 1         |
| Robot kit(includes top/bottom plates,batterycharger,tt motors,wheels setsetc) | 1         |
| assorted jumper wires                                                         | as needed |



References:

\-1. Arduino IDE: https://www.arduino.cc/en/main/software
\-2. The resource files (sample code and tutorials) from Elegoo:
https://www.elegoo.com/download/
\-3. For EECE.5520 students, you need to install “arduinoFFT” library through the IDE’s library
manager tool.
\-4. For EECE.5520 students, you need to refer to the robot car assembly guide to build the twowheel
robot car: https://github.com/ACANETS/eece-4520-5520-
labs/blob/2039441de8a069a14cadbb58f175f142d48f87c0/lab3/assembly%20guide.md
\-5. Hugging bot demo video: https://youtu.be/7CNUAesYvLU
![image](https://github.com/Prathiba01852498/RobotCar/assets/157857568/cff0b341-70ac-467e-ba7d-ed0fc2f4e77a)


