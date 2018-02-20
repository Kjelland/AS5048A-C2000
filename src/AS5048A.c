

#include <AS5048A.h>
AS5048A_Obj as5048a;

//! \brief     Sets up the SPI for communication with the AS5048A sensor
//! \param[in] SPI_Handle, CLK_Handle
void AS5048A_spi_init(SPI_Handle spiHandle,CLK_Handle clkHandle)
{
    CLK_enableSpiaClock(clkHandle);

    // Reset on, rising edge, 16-bit char bits
    SPI_setCharLength(spiHandle, SPI_CharLength_16_Bits);

    // Enable master mode, normal phase,
    SPI_setMode(spiHandle, SPI_Mode_Master);
    SPI_enableTx(spiHandle);

    SPI_setBaudRate(spiHandle, SPI_BaudRate_1_MBaud);
    SPI_setClkPhase(spiHandle,SPI_ClkPhase_Normal);
    SPI_setClkPolarity(spiHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);

    SPI_disableLoopBack(spiHandle);
    SPI_enable(spiHandle);

    // Set so breakpoints don't disturb xmission
    SPI_setPriority(spiHandle, SPI_Priority_FreeRun);//SPI_Priority_FreeRun

}

//! \brief     Sets up the SPI FIFO for communication with the AS5048A sensor
//! \param[in] SPI_Handle
void AS5048A_fifo_init(SPI_Handle spiHandle)
{

    // Initialize SPI FIFO registers
    SPI_enableChannels(spiHandle);
    SPI_enableFifoEnh(spiHandle);
    SPI_resetTxFifo(spiHandle);
    SPI_clearTxFifoInt(spiHandle);
    SPI_resetRxFifo(spiHandle);
    SPI_clearRxFifoInt(spiHandle);
    SPI_setRxFifoIntLevel(spiHandle, SPI_FifoLevel_4_Words);

}

//! \brief     Sets up the SPI GPIO for communication with the AS5048A sensor
//! \param[in] GPIO_Handle
void AS5048A_gpio_init(GPIO_Handle gpioHandle){
    // Initalize GPIO
    GPIO_setPullUp(gpioHandle, GPIO_Number_16, GPIO_PullUp_Enable);
    GPIO_setPullUp(gpioHandle, GPIO_Number_17, GPIO_PullUp_Enable);
    GPIO_setPullUp(gpioHandle, GPIO_Number_18, GPIO_PullUp_Enable);
    GPIO_setPullUp(gpioHandle, GPIO_Number_19, GPIO_PullUp_Enable);
    GPIO_setQualification(gpioHandle, GPIO_Number_16, GPIO_Qual_ASync);
    GPIO_setQualification(gpioHandle, GPIO_Number_17, GPIO_Qual_ASync);
    GPIO_setQualification(gpioHandle, GPIO_Number_18, GPIO_Qual_ASync);
    GPIO_setQualification(gpioHandle, GPIO_Number_19, GPIO_Qual_ASync);
    GPIO_setMode(gpioHandle, GPIO_Number_16, GPIO_16_Mode_SPISIMOA);
    GPIO_setMode(gpioHandle, GPIO_Number_17, GPIO_17_Mode_SPISOMIA);
    GPIO_setMode(gpioHandle, GPIO_Number_18, GPIO_18_Mode_SPICLKA);
    GPIO_setMode(gpioHandle, GPIO_Number_19, GPIO_19_Mode_SPISTEA_NOT);
}

//! \brief     Calculate the even parity of the command
//! \param[in] uint16_t value
uint16_t AS5048A_CalculateEvenParity(const uint16_t value){
    uint16_t x = value;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return ((~x) & 1)<<15;
}

//! \brief     Calculate the odd parity of the command
//! \param[in] uint16_t value
uint16_t AS5048A_CalculateOddParity(const uint16_t value){
    uint16_t x = value;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return ((x) & 1)<<15;
}

//! \brief     Sends a read command for a register and then sends an empty command to read out the feedback
//! \param[in] SPI_Handle, (uint16_t) address
uint16_t AS5048A_read(SPI_Handle spiHandle, const uint16_t address,bool *error){
    //Adds parity
    uint16_t recivedData;
    uint16_t ctrlAddress;
    ctrlAddress =address | AS5048A_CalculateEvenParity(address);
    //Adds read mask
    ctrlAddress |= AS5048A_RW_MASK;
    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(spiHandle);
    SPI_enableRxFifo(spiHandle);
    //Write address to read from
    SPI_write(spiHandle,ctrlAddress);
    while(SPI_getRxFifoStatus(spiHandle) == SPI_FifoStatus_Empty){}
    //Read last message
    SPI_read(spiHandle) ;
        //Write NOP to recive the new value
    SPI_write(spiHandle,AS5048A_NOP);

    while(SPI_getRxFifoStatus(spiHandle) == SPI_FifoStatus_Empty){}

    //Read the value from the addressed register
    recivedData=SPI_read(spiHandle);
    if (recivedData & 0x4000){
        *error=1;
    }

    return recivedData & ~0xC000;
}

//! \brief     Reads the programming register on the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_readProgrammingReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars){
    uint16_t newData = AS5048A_read(spiHandle,AS5048A_PROGRAMMING_CONTROL,&Spi_AS5048A_Vars->errorFlag);
    Spi_AS5048A_Vars->ProgrammingRegister.PRG_Enable = (bool)(newData & (uint16_t)AS5048A_PROGRAMMING_ENABLE)?1:0;
    Spi_AS5048A_Vars->ProgrammingRegister.PRG_burn = (bool)(newData & (uint16_t)AS5048A_PROGRAMMING_BURN)?1:0;
    Spi_AS5048A_Vars->ProgrammingRegister.PRG_verify = (bool)(newData & (uint16_t)AS5048A_PROGRAMMING_VERIFY)?1:0;
}

//! \brief     Reads the error register and clear the error flag on the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_clearErrorReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars){
    uint16_t newData = AS5048A_read(spiHandle,AS5048A_CLEAR_ERROR_FLAG,&Spi_AS5048A_Vars->errorFlag);
    Spi_AS5048A_Vars->ErrorRegister.ERR_framingError = (bool)(newData & (uint16_t)AS5048A_ERROR_FRAME)?1:0;
    Spi_AS5048A_Vars->ErrorRegister.ERR_invalidCommand = (bool)(newData & (uint16_t)AS5048A_ERROR_INVALID_COMMAND)?1:0;
    Spi_AS5048A_Vars->ErrorRegister.ERR_parityError = (bool)(newData & (uint16_t)AS5048A_ERROR_PARITY)?1:0;
    Spi_AS5048A_Vars->errorFlag=0;
}

//! \brief     Reads the diagnostics register from the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_readDiagnoseReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars){
    uint16_t newData = AS5048A_read(spiHandle,AS5048A_DIAG_AGC,&Spi_AS5048A_Vars->errorFlag);
    Spi_AS5048A_Vars->DiagnoseRegister.DIAG_AGC_automaticGain = (char)newData&0xFF;
    Spi_AS5048A_Vars->DiagnoseRegister.DIAG_AGC_ocf = (bool)(newData & (uint16_t)AS5048A_DIAG_AGC_OCF)?1:0;
    Spi_AS5048A_Vars->DiagnoseRegister.DIAG_AGC_cof = (bool)(newData & (uint16_t)AS5048A_DIAG_AGC_COF)?1:0;
    Spi_AS5048A_Vars->DiagnoseRegister.DIAG_AGC_Comp_Low = (bool)(newData & (uint16_t)AS5048A_DIAG_AGC_COMP_LOW)?1:0;
    Spi_AS5048A_Vars->DiagnoseRegister.DIAG_AGC_Comp_High = (bool)(newData & (uint16_t)AS5048A_DIAG_AGC_COMP_HIGH)?1:0;
}

//! \brief     Reads the zero position from the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_readZeroPosReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars){
    uint16_t newDataHigh = AS5048A_read(spiHandle,0x0016,&Spi_AS5048A_Vars->errorFlag);
    uint16_t newDataLow = AS5048A_read(spiHandle,0x0017,&Spi_AS5048A_Vars->errorFlag);

    Spi_AS5048A_Vars->ZeroPosition.ZERO_POS_HI   = (uint16_t)newDataHigh & (uint16_t)0xFF;
    Spi_AS5048A_Vars->ZeroPosition.ZERO_POS_LOW  = (uint16_t)newDataLow & (uint16_t)0x3F;
    Spi_AS5048A_Vars->zeroAngle  = (float)((uint16_t)(Spi_AS5048A_Vars->ZeroPosition.ZERO_POS_HI<<6) | (uint16_t)(Spi_AS5048A_Vars->ZeroPosition.ZERO_POS_LOW))*360.0/16384.0;
}

//! \brief     Write the zero offset position to the AS5048A
//! \param[in] SPI_Handle, float zeroAngle(degrees)
void AS5048A_writeZeroPosReg(SPI_Handle spiHandle, const float zeroAngle){
    uint16_t data;
    uint16_t low;
    uint16_t high;
    data =  ((zeroAngle / 360.0)* 16384.0);
    low=0x003F & data;
    high=(0x3FC0 & data)>>6;
    AS5048A_write(spiHandle,AS5048A_OTP_REGISTER_ZERO_POS_HIGH, high);
    AS5048A_write(spiHandle,AS5048A_OTP_REGISTER_ZERO_POS_LOW ,low );

}

//! \brief     Returns the absolute (multiturn) rotation. This is uses calculation from the MCU
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_getAngle(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars){
    float angle=AS5048A_read(spiHandle,AS5048A_ANGLE,&Spi_AS5048A_Vars->errorFlag) * (360.0 / 16384.0);

    float angleDiff=angle- Spi_AS5048A_Vars->angleLastCycle;
    if (angleDiff>200){
        Spi_AS5048A_Vars->count--;
    }
    else if(angleDiff<-200){
        Spi_AS5048A_Vars->count++;
    }

    Spi_AS5048A_Vars->angleLastCycle=angle;
    Spi_AS5048A_Vars->angleSingleturn = angle;
    Spi_AS5048A_Vars->angleMultiturn=Spi_AS5048A_Vars->count*360.0+Spi_AS5048A_Vars->angleSingleturn;
}

//! \brief     Sends a read command for a register and read the recive register.(last package)
//! \param[in] SPI_Handle, (uint16_t) address
uint16_t AS5048A_stream(SPI_Handle spiHandle, uint16_t address){
    //Adds read mask
    address = address | AS5048A_RW_MASK;
    //Adds parity
    address |= AS5048A_CalculateEvenParity(address);
    SPI_write(spiHandle,address);
    while(SPI_getRxFifoStatus(spiHandle) == SPI_FifoStatus_Empty){}
    return SPI_read(spiHandle) & ~0xC000;
}

//! \brief     Returns the magnitude directly from the AS5048A.
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_getMagnitude(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars){
    Spi_AS5048A_Vars->magnitude=AS5048A_read(spiHandle,AS5048A_MAGNITUDE,&Spi_AS5048A_Vars->errorFlag) / 16384.0;
}

//! \brief     Sends a write to a register address and then sends the data to write in it
//! \param[in] SPI_Handle, (uint16_t) address, (uint16_t) data
void AS5048A_write(SPI_Handle spiHandle,const uint16_t address,const uint16_t data) {
    uint16_t ctrlAddress;
    uint16_t ctrlData;
    uint16_t n;

    //Adds parity
    ctrlAddress = address | AS5048A_CalculateOddParity(address);
    ctrlData = data | AS5048A_CalculateOddParity(data);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(spiHandle);
    SPI_enableRxFifo(spiHandle);

    SPI_write(spiHandle,ctrlAddress);
    // wait for registers to update
      for(n=0;n<0xdf;n++)
       asm(" NOP");
    while(SPI_getRxFifoStatus(spiHandle) == SPI_FifoStatus_Empty){}
    //Read last message
    SPI_read(spiHandle);


    SPI_write(spiHandle,ctrlData);
    // wait for registers to update
      for(n=0;n<0xdf;n++)
        asm(" NOP");
    while(SPI_getRxFifoStatus(spiHandle) == SPI_FifoStatus_Empty){}
    SPI_read(spiHandle);
}

