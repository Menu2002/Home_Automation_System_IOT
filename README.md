# Home_Automation_System_IOT

1 Overview
  
1.1 Purpose
The purpose of this project is to build a smart home device which can be used to control the home appliances via internet. The home automation device that we build can be integrated with almost all the home appliances and can be used to control them remotely from any part of the world over internet.

1.2 Scope 
  
The Home automation should be simulated on the picsimlab simulator, Blynk iot mobile applications used to control the devices. We Should be able to control the lights, temperature of the home , inflow and outflow of water in the water tank. 

2. Functional Requirements

2.1 Garden lights control

Description : 

Read the LDR sensor value, based on the reading from LDR, vary the brightness of the led, which resembles controlling garden lights based on the availability of sunlight.


2.2 Temperature Control System

Description : 

The temperature control system consists of a heating resistor, an LM35 temperature sensor, and a cooler. Which resembles the temperature control system at home. Read the temperature from the temperature sensor LM35 and display it on the CLCD. Control the temperature of the system by turning ON/OFF the heater and cooler through the Blynk IOT mobile app .

2.3 water tank inlet and outlet valve control

Description : 

Read the volume of the water in the tank through Serial Communication and display it on the CLCD, control the volume of the water in the tank by controlling the inlet and outlet valve, by sending commands through serial communication. Display the volume of water in the tank on the CLCD.

2.3 User Interfaces

BLYNK Application

Create widgets on Mobile blynk application 

Button widgets to control heater, cooler, inlet valve , outlet value.

Gauge widgets to display temperature and volume of the water in the tank on the mobile application

Terminal widgets to display the notifications whenever threshold is crossed like
“Temperature is more than 35 degrees”, “” turning OFF the heater”, “water level is full “
“Water inflow disabled”

![image](https://github.com/Menu2002/Home-Automation-System/assets/145160069/08cdd991-5386-4ced-881f-33d3e96336ee)


![image](https://github.com/Menu2002/Home-Automation-System/assets/145160069/f268882a-dbd0-49b1-9aa2-98d6b5e7d349)


3. Conclusion: 

Using BLYNK Iot  application and Picsimlab simulator, We simulated home automation, where LED, temperature system, Serial tank resembles  Light, Heater, Cooler and Water tank in real time.
CLCD acts like a dash board used for displaying the events, Widgets from Blynk Iot app like button widgets are used to control heater, cooler and inlet valve, outlet valve.
Gauge widgets to display the temperature and volume of the water.





Code:


/*************************************************************
Title         :   Home automation using blynk
Description   :   To control light's brigntness with brightness,monitor temperature , monitor water level in the tank through blynk app
Pheripherals  :   Arduino UNO , Temperature system, LED, LDR module, Serial Tank, Blynk cloud, Blynk App.
 *************************************************************/

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "TMPL3OUsnrled"
#define BLYNK_DEVICE_NAME "Home Automation"
#define BLYNK_AUTH_TOKEN "uYM2jtmNf4fNSjhzJ1oVXGG_QGxGqcPt"


// Comment this out to disable prints 
//#define BLYNK_PRINT Serial

#include <SPI.h>
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
/* lib for clcd*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "main.h"
#include "temperature_system.h"
#include "ldr.h"
#include "serial_tank.h"

char auth[] = BLYNK_AUTH_TOKEN;
bool heater_sw,cooler_sw,outlet_sw,inlet_sw;
unsigned int tank_volume;

BlynkTimer timer;

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// This function is called every time the Virtual Pin 0 state changes
/*To turn ON and OFF cooler based virtual PIN value*/
BLYNK_WRITE(COOLER_V_PIN)
{
  // TO READ THE VAL ON VIRTUAL PIN CONNECTED TO COOLER
  cooler_sw = param.asInt();

  if(cooler_sw)
  {
    cooler_control(ON);
    lcd.setCursor(7,0);
    lcd.print("C0_LR ON ");
  }
  else
  {
    cooler_control(OFF); 
    lcd.setCursor(7,0);
    lcd.print("C0_LR OFF");
  }

  
}
/*To turn ON and OFF heater based virtual PIN value AND PRINT NOTIFICATION ON CLCD*/
BLYNK_WRITE(HEATER_V_PIN )
{
  // TO READ THE VAL ON VIRTUAL PIN CONNECTED TO HEATER
  heater_sw = param.asInt();

  if(heater_sw)
  {
    heater_control(ON);
    lcd.setCursor(7,0);
    lcd.print("HT_R ON  ");
  }
  else
  {
    heater_control(OFF); 
    lcd.setCursor(7,0);
    lcd.print("HT_R OFF ");
  }
  
}
/*To turn ON and OFF inlet vale based virtual PIN value*/
BLYNK_WRITE(INLET_V_PIN)
{
   //to read the values on the inlet pin
   inlet_sw = param.asInt(); 
   if(inlet_sw)
   {
     enable_inlet();
     lcd.setCursor(7,1);
     lcd.print("IN_FLOW_ON ");
   }
   else
   {
     disable_inlet();
     lcd.setCursor(7,1);
     lcd.print("IN_FLOW_OFF");
   }
}

/*To turn ON and OFF outlet value based virtual switch value*/
BLYNK_WRITE(OUTLET_V_PIN)
{
  // to read the vals from outlet pin
  outlet_sw = param.asInt(); 
   if(outlet_sw)
   {
     enable_outlet();
     lcd.setCursor(7,1);
     lcd.print("OT_FLOW_ON ");
   }
   else
   {
     disable_outlet();
     lcd.setCursor(7,1);
     lcd.print("OT_FLOW_OFF");
   }
  
}

/* To display temperature and water volume as gauge on the Blynk App*/  
void update_temperature_reading()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(TEMPERATURE_GAUGE, read_temperature());
  Blynk.virtualWrite(WATER_VOL_GAUGE , volume());

}

/*To turn off the heater if the temperature raises above 35 deg C*/
void handle_temp(void)
{
  /* read temp  and compare with 35 and check if heater is ON*/
  if((read_temperature() > float(35)) && heater_sw)
  {
    heater_sw = 0;
    /* to turn off th heater*/
    heater_control(OFF);
    /* to print heater status on the dashboard*/
    lcd.setCursor(7,0);
    lcd.print("HT_R OFF");

    // to print notification on the blynk app
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN, "temperature is above 35 degree celsius\n");
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN, " turning off the heater\n");


    // to reflect the status on the button widget on the heater pin
    Blynk.virtualWrite(HEATER_V_PIN, 0);
 
  }
 
}

/*To control water volume above 2000ltrs*/
void handle_tank(void)
{
  // to check if the vol ofthe water is les than 2000 and inlet is off, then enable the inlet valve
  if((tank_volume < 2000) && (inlet_sw == 0))
  {
    enable_inlet();
    inlet_sw = 1;
// to print notification status on clcd
    lcd.setCursor(7,1);
    lcd.print("IN_FL_ON");

// to print the notification on the mobile app
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"volume of water in the tank is less than 2000");
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN, " turning on the inlet valve\n");

// REFLECTING the status on the button widget
    Blynk.virtualWrite(INLET_V_PIN, 1);
 

  }

  // if the tank is full and inlet valve is on then turn off the inlet valve
  if((tank_volume == 3000) && (inlet_sw == 1))
  {
    disable_inlet();
    inlet_sw = 0;
// to print notification status on clcd
    lcd.setCursor(7,1);
    lcd.print("IN_FL_OFF");

// to print the notification on the mobile app
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN,"tank is full");
    Blynk.virtualWrite(BLYNK_TERMINAL_V_PIN, " turning off the inlet valve\n");

// REFLECTING the status on the button widget
    Blynk.virtualWrite(INLET_V_PIN, 0);
 
  
  }
  

}


void setup(void)
{
  /* to config garden light as op */
  init_ldr();

  /* to init.. clcd*/
  lcd.init();
  /*to turn the back light*/
  lcd.backlight();
  /* to clear clcd*/
  lcd.clear();
  /* to set cursor to the first pos*/
  lcd.home();


  /*to display the string*/
  lcd.setCursor(0,0);
  lcd.print("T=");

  // to set cursor at second line first pos
  lcd.setCursor(0,1);
  lcd.print("V=");


/* TO CONNECT arduino to the blynk cloud*/
  Blynk.begin(auth);

  /*to init temp sys*/
  init_temperature_system();

  // to init serial tank
  init_serial_tank();

  /* to update temp to blynk app for every 0.5 sec*/
  timer.setInterval(500L, update_temperature_reading);
    
}

void loop(void) 
{
  Blynk.run();
  // keep timer running
  timer.run();
  /* to control the brightness of the led*/
  brightness_control();
 


  /* to read the temp and display on clcd*/
  String temperature;
  temperature = String(read_temperature(), 2);
  lcd.setCursor(2,0);
  lcd.print(temperature);

  // to read the volume of water and display it on the clcd
  tank_volume = volume();
  lcd.setCursor(2,1);
  lcd.print(tank_volume);

  // to maintain volume of water for 2000
  handle_tank();

  // to maintain threshold temp
  handle_temp();
}




Ldr Source File:

#include "ldr.h"
#include "Arduino.h"
#include "main.h"

void init_ldr(void)
{
   pinMode(GARDEN_LIGHT, OUTPUT);
   
}
  unsigned int ldr_val;
void brightness_control(void)
{
    ldr_val = analogRead(LDR_SENSOR);


  /* mapping 0 to 1023--> 255 to 0*/
  /*scaling down 0 to 1023 -->255 to 0*/

  ldr_val = (1023 - ldr_val)/4;

  /* to assign duty cycle LED based on LDR val*/
  analogWrite(GARDEN_LIGHT,ldr_val);
  delay(100);
   
}


Ldr Header File:

#ifndef LDR_H
#define LDR_H


#define LDR_SENSOR       A1
#define GARDEN_LIGHT     3



void init_ldr(void);
void brightness_control(void);

#endif


Main:

#ifndef MAIN_H
#define MAIN_H


#define ON    1
#define OFF   0

#define TEMPERATURE_GAUGE       V1
#define COOLER_V_PIN            V0
#define HEATER_V_PIN            V2
#define WATER_VOL_GAUGE         V3
#define INLET_V_PIN             V4
#define OUTLET_V_PIN            V5
#define BLYNK_TERMINAL_V_PIN    V6


#endif



Serial Tank Source File:

#include "serial_tank.h"
#include "Arduino.h"
#include "main.h"

unsigned int volume_value;
unsigned char valueh, valuel;

void init_serial_tank(void) 
{
  /* to begin the serial communication between serial tank and arduino board*/
    Serial.begin(19200);
    Serial.write(0xFF); //sincroniza comunicação
    Serial.write(0xFF);
    Serial.write(0xFF);   
}

unsigned int volume(void)
{
  
    // to read the volume of water
    Serial.write(VOLUME);
    //wait for data
    while(!Serial.available());
    // reading higher byte
    valueh = Serial.read();
    while(!Serial.available());
    // reading lowest byte
    valuel = Serial.read();
    // combining higher byte and lower byte
    volume_value = valueh << 8 | valuel ;

    // return the volume of the water
    return volume_value;
    
}
void enable_inlet(void)
{
    Serial.write(INLET_VALVE);
    Serial.write(ENABLE);
}  
void disable_inlet(void)
{
    Serial.write(INLET_VALVE);
    Serial.write(DISABLE);
    
}  
void enable_outlet(void)
{  
    Serial.write(OUTLET_VALVE);
    Serial.write(ENABLE);
    
}
void disable_outlet(void)
{  
    Serial.write(OUTLET_VALVE);
    Serial.write(DISABLE);
    
}


Serial Tank Header File:

#ifndef SERIAL_TANK_H
#define SERIAL_TANK_H


//input digital
#define INLET_VALVE  0x00
#define OUTLET_VALVE 0x01

//sensors digital
#define HIGH_FLOAT 0x10
#define LOW_FLOAT  0x11

//sensor analog
#define VOLUME 0x30

#define   ENABLE  0x01
#define   DISABLE 0x00

void init_serial_tank(void);
void enable_inlet(void);
void enable_outlet(void);
void disable_inlet(void);
void disable_outlet(void);
unsigned int volume(void);

#endif



Temperature System Source File:

#include "temperature_system.h"
#include "Arduino.h"
#include "main.h"


void init_temperature_system(void)
{
  /* to config heater and cooler pin as op pin*/
  pinMode(HEATER,OUTPUT);
  pinMode(COOLER,OUTPUT); 

  /* TO TURN OFF*/
  digitalWrite(HEATER,LOW);
  digitalWrite(COOLER,LOW);

   
}

float read_temperature(void)
{
  float temperature;
  /* to read analog vals and converting the voltage and then to temp*/
  temperature = (((analogRead(TEMPERATURE_SENSOR)*(float)5/1024)) / (float)0.01);
  return temperature;
}

/* function to control the cooler*/
void cooler_control(bool control)
{
  if(control)
  {
     digitalWrite(COOLER, HIGH);
  }
  else
  {
    digitalWrite(COOLER,LOW);
  }
  
}
void heater_control(bool control)
{
   if(control)
  {
     digitalWrite(HEATER, HIGH);
  }
  else
  {
    digitalWrite(HEATER,LOW);
  }
    
}


Temperature System Header File:


#ifndef TEMPERATURE_SYSTEM_H
#define TEMPERATURE_SYSTEM_H

#define HEATER                5
#define COOLER                4


#define TEMPERATURE_SENSOR    A0


float read_temperature(void);
void init_temperature_system(void);
void cooler_control(bool control);
void heater_control(bool control);
#endif








