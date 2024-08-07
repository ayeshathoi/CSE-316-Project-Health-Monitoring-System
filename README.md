# Health Monitoring System
The main aim of the project is to monitor heart rate, body temperature and blood oxygen saturation level of a patient body. It also monitors room humidity and room temperature of the patient room.This system designed by using Arduino Uno. Pulse oxymeter are being used here to measure temperature, blood oxygen level and pulse rate of body.  Temperature and humidity sensor are being used to measure room temperature and room humidity. Corona Patient can be benefitted from this technology  the most. SMS will be sent  to the doctor in case of any critical situation by passing signal to GSM module. A buzzer will be activated so that doctor can pay more attention to the situation.

Made for Level-3 Term-1,
CSE316: Microprocessors, Microcontrollers, and Embedded Systems Sessional,
created by  Ayesha Binte Mostofa (1805062),Zarin Tasnim (1805071) and Nusrat Noor Labiba(1805081) 
###### Under the kind supervision of Dr. A.B.M. Alim Al Islam,Dr. Md. Monirul Islam & Md. Masum Mushfiq Sir 
 
This is a Hardware project. See our Project Demonstration : [Youtube Link](https://youtu.be/Gx7eTR-YP74)

We have added block diagram and circuit diagram in the [Project Report](https://github.com/ayeshathoi/CSE-316-Project-Health-Monitoring-System/blob/main/Project%20Report%20-%20Health%20Monitoring%20System%20(%2062%2C71%2C81).pdf).
![image](https://github.com/ayeshathoi/CSE-316-Project-Health-Monitoring-System/assets/79919256/82d21085-314a-43a0-a39e-99647d297b3f)


### Tools for project
<li>Arduino Uno</li>
<li>Max 30102</li>
<li>LCD 1602</li>
<li>I2C Adapter</li>
<li>GSM Module 900A</li>
<li>DHT11</li>
<li>Buzzer</li>
<li>Power Bank - 5V 2A </li>
<li>BreadBoard</li>
<li>Jumper Wire</li>



<h2>Features of Project: </h2>
<li> Patient can know his/her body temperature, pulse rate and blood oxygen saturation level</li>
<li> Patient can also know room humidity and room temperature .</li>
<li> He/She can contact doctor through GSM module if pulse rate<60 or pulse rate > 125 or blood oxygen level<90 .</li>
<li> He/she can also contact for room condition when room humidity<40% and room temperature > 36C .</li>
<li> A buzzer will be activated when message will be sent so that doctor can pay more attention to the situation.</li>
 <li> A LCD Display will continuously show data for every activity.</li>
 
 <h3>Arduino Uno</h3>
 <li> Code is uploaded here</li>
 <li> All components are connected to Arduino via jumper wires and breadboard.</li>
 <li> Gives 5V & 3.3V to the components</li>
 
 <h3>Max 30102</h3>
 <li> Measure Body Temperature</li>
 <li> Measure heart beats per minnute</li>
 <li>  Measure Blood oxygen Saturation level</li>
 
 <h3>DHT11</h3>
 <li>Measure Room Temperature</li>
 <li>Measure Room Humidity</li>
 
<h3>LCD</h3>
<li>1. It shows if the sensors are connected or not. </li>
<li>Finger placed on sensor or not</li>
<li>The data read by DHT11 & Max30102 </li>
<li>Message Sending situation</li>
