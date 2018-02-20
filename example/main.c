//#############################################################################
//
//  AS5048A library example for C2000: LaunchXL-F28027F
//  Communication using SPI with FIFO
//!
//#############################################################################


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/spi.h"
#include "f2802x_common/include/wdog.h"
#include "AS5048A.h"

CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
SPI_Handle mySpi;

AS5048A_Vars_t gAs5048aVars;

void main(void)
{



    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    // Initialize all the handles needed for this application    
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    mySpi = SPI_init((void *)SPIA_BASE_ADDR, sizeof(SPI_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    // Perform basic system initialization    
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();
    
    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);
    
    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    // If running from flash copy RAM only functions to RAM   
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif   

    AS5048A_gpio_init(myGpio);    // Initialize the SPI GPIOs

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

    AS5048A_spi_init(mySpi,myClk);         // Initialize SPI
    AS5048A_fifo_init(mySpi);    // Initialize the SPI FIFOs

    for(;;) {

        // If error flag is high, read the error register and clear the flag
        if (gAs5048aVars.errorFlag){
            AS5048A_clearErrorReg(mySpi,&gAs5048aVars);
            DELAY_US(500);
        }
        else{
            //Get angle
            AS5048A_getAngle(mySpi,&gAs5048aVars);
            DELAY_US(500);
            //Get magnitude
            AS5048A_getMagnitude(mySpi,&gAs5048aVars);
            DELAY_US(500);
            //Read the diagnose register
            AS5048A_readDiagnoseReg(mySpi,&gAs5048aVars);
            DELAY_US(500);
            //Write to the offset register. Examplevalue = 15.50
            AS5048A_writeZeroPosReg(mySpi,15.50);
            DELAY_US(500);
            //Read the offset register.
            AS5048A_readZeroPosReg(mySpi,&gAs5048aVars);
            DELAY_US(500);
            //Read the programming register
            AS5048A_readProgrammingReg(mySpi,&gAs5048aVars);
            DELAY_US(500);
        }

    }
}

//===========================================================================
// No more.
//===========================================================================


