

#ifndef __ADXL312_H__
#define __ADXL312_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif



//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
void I2CSendString(uint32_t slave_addr, char array[]);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __ADXL312_H__
