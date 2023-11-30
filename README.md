![banner image](https://github.com/EKMallon/The-e360-Student-Built-Data-Logger/assets/7884030/48f75f8c-3236-4a68-9ae2-6572afddc3ce)
# The e360 Student Built Data Logger 2023
---
This program supports an ongoing series of DIY 'Classroom Logger' tutorials from Edward Mallon & Dr. Patricia Beddows at the Cave Pearl Project. The idea is to provide a starting point for student projects in environmental monitoring courses and/or thesis level research.<br/>
<br/>The tutorial that matches this code can be found at:<br/>
 post link goes here
<br/>with a detailed building guide video at:<br/>
video link goes here
<img   align="right" width="400" src="[https://github.com/EKMallon/2-Part-ProMini-EEprom-Data-Logger_2022/blob/main/images/2-PartEEpromLogger_CavePearlProject_2022.jpg]">
---
<img   align="right" width="400" src="[https://github.com/EKMallon/The-e360-Student-Built-Data-Logger/blob/main/images/20231128_e360_schematic_b-01.jpg]">

This 'low power' 2-module iteration runs the logger from a CR2032 coin cell and uses  EEprom memory to store sensor readings. This necessarily involves several power optimization steps which add significant complexity to the base code (as compared to previous versions) but hopefully everyone can read through the code and understand what is happening from the extensive comments. There are several manual configuration settings controlled by #define statements at the start of the program, and the logger will not be able to read the coincell voltage properly until you tweak the InternalReferenceConstant. <br/> <br/>
Data download & logger control are managed  through the IDE's serial monitor window at 500000 baud. 
The logger WILL NOT START taking readings until those serial handshakes are completed via the UART connection.<br/><br/>
Note that all the readings are initially buffered in opdDataBuffer[16] & sensorDataBuffer[16] arrays so there won't be any data in the EEprom until those ram buffers get transfered.  With the 1-byte RTCtemp only default configuration you will have to wait 16* sampleInterval minutes before that happens. The default 4k eeprom on the rtc module stores 4096 of those readings so takes ~2.8 days at 1min interval before it is full. (at which point the logger shuts down)<br/><br/>
The most important rule to follow when adding new sensors is that the buffer arrays can only handle 'powers of 2' additions of 1, 2, 4, or 8 bytes.
Odd byte quantities per sensor record (other than one) and you end up with page boundary issues in the EEprom.

---

Note: This script will still run on the 3-module "Modules & Jumper Wires"  loggers described in the original Sensors paper: http://www.mdpi.com/1424-8220/18/2/530 
and provides a 'no SD card' method of extending lifespan on the 2020 classroom logger described at https://thecavepearlproject.org/2020/10/22/pro-mini-classroom-datalogger-2020-update/  where multiple I2C eeproms can be added easily via the breadboard(s)

---
Addendum 2023-10-25: Code revisions are *currently underway* to support the use of these loggers in enviro-sci course curriculum & some elements of the old code may disappear for a while until our students have progressed further through the lab sequence. Those elements will then be restored to the base code. This may cause discrepancies between the text in the 2-part logger blog post and the options in the code posted here.

