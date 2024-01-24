# Overview 
Embedded System Course's assignments   focused on ESP32 platform and ESP-IDF framework

* Lab 1: Led blinking
* Lab 2: Displaying OLED SSD1306
  - Displaying text
  - Displaying school logo 
* Lab 3: BLE basics: broadcasting, read, write, notify
  - Broadcast advertising messages
  - Create communication between ESP32 and LightBlue app on mobile phone
* Lab 4: MQTT basics: connect, subcribing, publishing
  
   Create communication following the below model
  
   Mobilephone <----_bluetooth_----> ESP32 <----_MQTT Broker_----> PC
* Lab 5:
  Create an IoT system with the following requirements:
  - NODE ESP32:
    + Get temperature, humidity data from sensor
    + Display information onto OLED SSD1306
    + Send data to server periodically
  - SERVER:
    + Create a database (MySQL)
    + Receive data 
    + Save data into database
    + Allow local access to its web-app
  - END-USER:
    + Display information on web-site with graphs
   
      ![image](https://github.com/yunevo/labs-from-embedded-course/assets/156734673/782d1280-ace4-42c3-bd95-f8c9d3d9f133)
