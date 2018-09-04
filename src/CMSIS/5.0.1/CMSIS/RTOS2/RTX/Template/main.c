/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "../../../../../../CMSIS/5.0.1/CMSIS/RTOS2/Include/cmsis_os2.h"
#include "../../../../../../CMSIS/5.0.1/CMSIS/RTOS2/RTX/Library/IAR/IDE/RTE_Components.h"
 
#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
#ifdef RTE_Compiler_EventRecorder
  // Initialize and start Event Recorder
  EventRecorderInitialize(EventRecordError, 1U);
#endif
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
