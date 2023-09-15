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
