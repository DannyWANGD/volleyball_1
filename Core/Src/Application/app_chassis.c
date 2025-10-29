#include "app_chassis.h"
#include "module_chassis.h"
#include "cmsis_os.h"



void Chassis_Task(void const * argument) {
 
  for(;;) {
			
		Chassis_Control();
    Chassis_Trans_ControlData();
    osDelay(2);
    
  }
}

