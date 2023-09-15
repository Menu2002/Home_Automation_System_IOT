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
