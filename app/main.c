#include "stm32f10x.h"
#include "debug.h"
#include "usb_lib.h"


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    SystemInit();
    DebugInit();
    USB_PreInit();
    USB_Init();

    DEBUG(DEBUG_INFO,"init over\n");
    while(1)
    {
    
    }
}

