# Smart Security Control Room

An embedded smart security system built on **STM32F4** using **FreeRTOS** for concurrent task management.  
The system integrates ultrasonic sensing, servo control, buzzer alarms, LED indicators, and a BH1750 light sensor to provide adaptive security monitoring.

---

## Features
- **Ultrasonic Distance Measurement**  
  - Detects intrusions using HC-SR05 ultrasonic sensor.  
  - Input capture interrupts on STM32F4 for accurate distance calculation.  

- **Servo-Controlled Gate**  
  - PWM-based servo (SG90) to open/close the gate automatically.  
  - Only allows access when environmental conditions are satisfied.  

- **Buzzer Alarm & LEDs**  
  - Visual and audible alerts when intruders are detected.  
  - Mute functionality with timers for user convenience.  

- **Light Intensity Monitoring (BH1750)**  
  - Custom I2C driver for BH1750.  
  - Gate only opens if lights are on (room occupied), blocks access if dark.  
  - Implements hysteresis thresholds to avoid false triggers.  

- **FreeRTOS Multitasking**  
  - Event groups and software timers for task synchronization.  
  - Concurrent tasks: distance sensing, light sensing, servo control, buzzer, LED indication.  

---

## Hardware Used
- STM32F407G-DISC1 Development Board  
- HC-SR05 Ultrasonic Sensor  
- SG90 Servo Motor  
- Passive Buzzer  
- BH1750 Light Sensor (I2C)  
- On-board LEDs  

---

## Project Structure
