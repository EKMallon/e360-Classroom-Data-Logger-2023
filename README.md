![banner image](https://github.com/EKMallon/The-e360-Student-Built-Data-Logger/assets/7884030/48f75f8c-3236-4a68-9ae2-6572afddc3ce)
# The e360 Student Data Logger 2023 REPO is still under construction
![image](https://github.com/EKMallon/The-e360-Student-Built-Data-Logger/assets/7884030/1073ed17-0470-4bda-b54a-d84adefa1c2b)
This program supports an ongoing series of DIY 'Classroom Logger' tutorials from Edward Mallon & Dr. Patricia Beddows at the Cave Pearl Project. The idea is to provide a starting point for student projects in environmental monitoring courses and/or thesis level research.<br/>
<br/>The tutorial that matches this code can be found at:<br/>
 post link goes here
<br/>with a detailed building guide video at:<br/>
video link goes here

This 'low power' 2-module iteration runs the logger from a CR2032 coin cell and uses  EEprom memory to store sensor readings. This necessarily involves several power optimization steps which add significant complexity to the base code (as compared to previous versions) but hopefully everyone can read through the code and understand what is happening from the extensive comments. There are several manual configuration settings controlled by #define statements at the start of the program, and the logger will not be able to read the coincell voltage properly until you tweak the InternalReferenceConstant. <br/> <br/>
Data download & logger control are managed  through the IDE's serial monitor window at 500000 baud. 
The logger WILL NOT START taking readings until those serial handshakes are completed via the UART connection.<br/><br/>

The most important rule to follow when adding new sensors is that the buffer arrays can only handle 'powers of 2' additions of 1, 2, 4, or 8 bytes.
Odd byte quantities per sensor record (other than one) and you end up with page boundary issues in the EEprom.

---

