#ifndef _AS5048A_H_
#define _AS5048A_H_
// **************************************************************************
// the includes
#include <stdint.h>

// drivers

//#include "sw/drivers/spi/src/32b/f28x/f2802x/spi.h" <-- dont use
//#include "sw/drivers/gpio/src/32b/f28x/f2802x/gpio.h" <-- dont use
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
//#include "f2802x_common/include/spi.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/spi.h"
#include "f2802x_common/include/clk.h"

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

//! \brief Defines the R/W mask
//!
#define AS5048A_RW_MASK                     (1<<14)
//! \brief Defines the No Operation mask
//!
#define AS5048A_NOP                         (0x00)
//! \brief Defines the clear error mask
//!
#define AS5048A_CLEAR_ERROR_FLAG            (0x0001)
//! \brief Defines the programming control mask
//!
#define AS5048A_PROGRAMMING_CONTROL         (0x0003 )
//! \brief Defines the zero position (high) mask
//!
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH  (0x0016 )
//! \brief Defines the zero position (low) mask
//!
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW   (0x0017)
//! \brief Defines the Diagnostics + AutomaticGain Control (AGC) a mask
//!
#define AS5048A_DIAG_AGC                    (0x3FFD)
//! \brief Defines the magnitude control mask
//!
#define AS5048A_MAGNITUDE                   (0x3FFE)
//! \brief Defines the angle control mask
//!
#define AS5048A_ANGLE                       (0x3FFF)
//! \brief Defines the framing error bit in the error register
//!
#define AS5048A_ERROR_FRAME          (1 << 0)
//! \brief Defines the invalid command bit in the error register
//!
#define AS5048A_ERROR_INVALID_COMMAND          (1 << 1)
//! \brief Defines the parity error bit in the error register
//!
#define AS5048A_ERROR_PARITY         (1 << 2)
//! \brief Defines the mode bit in the programming register
//!
#define AS5048A_PROGRAMMING_ENABLE          (1 << 0)
//! \brief Defines the reserved 1 bit in the programming register
//!
#define AS5048A_PROGRAMMING_RESV1          (1 << 1)
//! \brief Defines the reserved 12 bit in the programming register
//!
#define AS5048A_PROGRAMMING_RESV2          (1 << 2)
//! \brief Defines the burn bit in the programming register
//!
#define AS5048A_PROGRAMMING_BURN          (1 << 3)
//! \brief Defines the reserved 3 bit in the programming register
//!
#define AS5048A_PROGRAMMING_RESV3          (1 << 4)
//! \brief Defines the reserved 4 bit in the programming register
//!
#define AS5048A_PROGRAMMING_RESV4          (1 << 5)
//! \brief Defines the verify bit in the programming register
//!
#define AS5048A_PROGRAMMING_VERIFY            (1 << 6)
//! \brief Defines the  OCF bit in the Diagnostics + AutomaticGain Control register
//!
#define AS5048A_DIAG_AGC_OCF            (1 << 8)
//! \brief Defines the  COF bit in the Diagnostics + AutomaticGain Control register
//!
#define AS5048A_DIAG_AGC_COF            (1 << 9)
//! \brief Defines the  Comp Low bit in the Diagnostics + AutomaticGain Control register
//!
#define AS5048A_DIAG_AGC_COMP_LOW            (1 << 10)
//! \brief Defines the  Comp High bit in the Diagnostics + AutomaticGain Control register
//!
#define AS5048A_DIAG_AGC_COMP_HIGH            (1 << 11)


//! \brief Enumeration for the Error register. All errors are cleared by access
//!
typedef enum
{
  ERR_framingError     = AS5048A_ERROR_FRAME,     //!< Framing error
  ERR_invalidCommand   = AS5048A_ERROR_INVALID_COMMAND,     //!< Command Invalid
  ERR_parityError      = AS5048A_ERROR_PARITY,     //!< Parity Error
} AS5048A_errorRegister_e;

//! \brief Enumeration for the programming register.
//! Programming must be enabled before burning the fuse(s). After programming is a
//! verification mandatory.
//!
typedef enum
{
  PRG_Enable            = AS5048A_PROGRAMMING_ENABLE,     //!< Enable programming
  PRG_RSV1              = AS5048A_PROGRAMMING_RESV1,     //!< Reserved
  PRG_RSV2              = AS5048A_PROGRAMMING_RESV2,     //!< Reserved
  PRG_burn              = AS5048A_PROGRAMMING_BURN,     //!< Burn
  PRG_RSV3              = AS5048A_PROGRAMMING_RESV3,     //!< Reserved
  PRG_RSV4              = AS5048A_PROGRAMMING_RESV4,     //!< Reserved
  PRG_verify            = AS5048A_PROGRAMMING_VERIFY,     //!< Verify
} AS5048A_programmingRegister_e;

//! \brief Enumeration for the Diagnostics + AutomaticGain Control register.
//!
typedef enum
{
  DIAG_AGC_ocv            = AS5048A_DIAG_AGC_OCF,     //!< Offset Compensation Finished
  DIAG_AGC_cof            = AS5048A_DIAG_AGC_COF,     //!< CORDIC Overflow
  DIAG_AGC_Comp_Low       = AS5048A_DIAG_AGC_COMP_LOW,     //!< indicates a high magnetic field. It is recommended to monitor in addition the magnitudevalue.
  DIAG_AGC_Comp_High      = AS5048A_DIAG_AGC_COMP_HIGH,     //!<  indicated a weak magnetic field. It is  recommended to monitor the magnitude value.
} AS5048A_Diag_AgcRegister_e;

//! \brief Object for the programming register
//!
typedef struct _AS5048A_Error_
{
  bool      ERR_framingError;  // Bit 0
  bool      ERR_invalidCommand;               // Bit 1
  bool      ERR_parityError;               // Bit 2
}AS5048A_Error_;



//! \brief Object for the programming register
//!
typedef struct _AS5048A_Programming_
{
  bool      PRG_Enable;  // Bit 0
  bool      PRG_RSV1;               // Bit 1
  bool      PRG_RSV2;               // Bit 2
  bool      PRG_burn;               // Bit 3
  bool      PRG_RSV3;               // Bit 4
  bool      PRG_RSV4;               // Bit 5
  bool      PRG_verify;             // Bit 6

}AS5048A_Programming_;
//! \brief Object for the Diagnostics + AutomaticGain Control (AGC) register
//!
typedef struct _AS5048A_Diag_Agc_
{
  char      DIAG_AGC_automaticGain;  // Bit 0-7
  bool      DIAG_AGC_ocf;            // Bit 8
  bool      DIAG_AGC_cof;            // Bit 9
  bool      DIAG_AGC_Comp_Low;       // Bit 10
  bool      DIAG_AGC_Comp_High;      // Bit 11
}AS5048A_Diag_Agc_;


//! \brief Object for the OTP Register Zero Position Low register
//!
typedef struct _AS5048A_Zero_Position_
{
    uint16_t                          ZERO_POS_HI;    // Bits 7-0
    uint16_t                          ZERO_POS_LOW;    // Bits 5-0
}AS5048A_Zero_Position_;

//! \brief Object for the AS5048A registers and commands
//!
typedef struct _AS5048A_Vars_t_
{
    AS5048A_Error_              ErrorRegister;
    AS5048A_Programming_        ProgrammingRegister;
    AS5048A_Zero_Position_   ZeroPosition;
    AS5048A_Diag_Agc_           DiagnoseRegister;
    int16_t count;
    float angleLastCycle;
    float angleSingleturn;
    float angleMultiturn;
    float magnitude;
    float zeroAngle;
    bool errorFlag;
}AS5048A_Vars_t;
typedef struct _AS5048A_Obj_
{
    float a;
}AS5048A_Obj;
//! \brief Defines the AS5048A handle
//!
typedef struct _AS5048A_Obj_ *AS5048A_Handle;

// **************************************************************************
// the functions
//! \brief     Sets up the SPI for communication with the AS5048A sensor
//! \param[in] SPI_Handle, CLK_Handle
void AS5048A_spi_init(SPI_Handle spiHandle,CLK_Handle clkHandle);

//! \brief     Sets up the SPI FIFO for communication with the AS5048A sensor
//! \param[in] SPI_Handle
void AS5048A_fifo_init(SPI_Handle spiHandle);

//! \brief     Sets up the SPI GPIO for communication with the AS5048A sensor
//! \param[in] GPIO_Handle
void AS5048A_gpio_init(GPIO_Handle gpioHandle);

//! \brief     Calculate the even parity of the command
//! \param[in] uint16_t value
uint16_t AS5048A_CalculateEvenParity(const uint16_t value);

//! \brief     Calculate the odd parity of the command
//! \param[in] uint16_t value
uint16_t AS5048A_CalculateOddParity(const uint16_t value);

//! \brief     Sends a read command for a register and then sends an empty command to read out the feedback
//! \param[in] SPI_Handle, (uint16_t) address
uint16_t AS5048A_read(SPI_Handle spiHandle, const uint16_t address,bool *error);

//! \brief     Reads the programming register on the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_readProgrammingReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars);

//! \brief     Reads the error register and clear the error flag on the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_clearErrorReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars);

//! \brief     Reads the diagnostics register from the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_readDiagnoseReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars);

//! \brief     Reads the zero position from the AS5048A
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_readZeroPosReg(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars);

//! \brief     Write the zero offset position to the AS5048A
//! \param[in] SPI_Handle, float zeroAngle(degrees)
void AS5048A_writeZeroPosReg(SPI_Handle spiHandle, const float zeroAngle);

//! \brief     Returns the absolute (multiturn) rotation. This is uses calculation from the MCU
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_getAngle(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars);

//! \brief     Sends a read command for a register and read the recive register.(last package)
//! \param[in] SPI_Handle, (uint16_t) address
uint16_t AS5048A_stream(SPI_Handle spiHandle, uint16_t address);

//! \brief     Returns the magnitude directly from the AS5048A.
//! \param[in] SPI_Handle, AS5048A_Vars_t
void AS5048A_getMagnitude(SPI_Handle spiHandle, AS5048A_Vars_t *Spi_AS5048A_Vars);

//! \brief     Sends a write to a register address and then sends the data to write in it
//! \param[in] SPI_Handle, (uint16_t) address, (uint16_t) data
void AS5048A_write(SPI_Handle spiHandle, const uint16_t address, const uint16_t data);

extern AS5048A_Obj as5048a;









#endif

