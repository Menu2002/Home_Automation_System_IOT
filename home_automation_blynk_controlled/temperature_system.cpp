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
