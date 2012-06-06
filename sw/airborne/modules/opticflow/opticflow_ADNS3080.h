/** \file opticflow_ADNS3080.h
 *  \brief Handling of ADNS3080 optic flow sensor
 *
 */

#ifndef OPTFLOW_ADNS3080_H
#define OPTFLOW_ADNS3080_H


#ifdef USE_OPTFLOW_ADNS3080

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

//register addresses
#define OPTFLOW_ADNS3080_ADDR_PROD_ID		0x00
#define OPTFLOW_ADNS3080_ADDR_REV_ID		0x01
#define OPTFLOW_ADNS3080_ADDR_MOTION		0x02
#define OPTFLOW_ADNS3080_ADDR_DX			0x03
#define OPTFLOW_ADNS3080_ADDR_DY			0x04
#define OPTFLOW_ADNS3080_ADDR_SQUAL			0x05
#define OPTFLOW_ADNS3080_ADDR_PIX_SUM		0x06
#define OPTFLOW_ADNS3080_ADDR_MAX_PIX		0x07
#define OPTFLOW_ADNS3080_ADDR_CONF			0x0a
#define OPTFLOW_ADNS3080_ADDR_EXCONF		0x0b
#define OPTFLOW_ADNS3080_ADDR_DO_LOW		0x0c
#define OPTFLOW_ADNS3080_ADDR_DO_UP			0x0d
#define OPTFLOW_ADNS3080_ADDR_SHUT_LO		0x0e
#define OPTFLOW_ADNS3080_ADDR_SHUT_UP		0x0f
#define OPTFLOW_ADNS3080_ADDR_FP_LO			0x10
#define OPTFLOW_ADNS3080_ADDR_FP_UP			0x11
#define OPTFLOW_ADNS3080_ADDR_MOTION_CLR	0x12
#define OPTFLOW_ADNS3080_ADDR_FRAMECAP		0x13
#define OPTFLOW_ADNS3080_ADDR_SROM_ENABLE	0x14
#define OPTFLOW_ADNS3080_ADDR_FP_MAX_B_LOW  0x19
#define OPTFLOW_ADNS3080_ADDR_FP_MAX_B_UP	0x1a
#define OPTFLOW_ADNS3080_ADDR_FP_MIN_B_LOW	0x1b
#define OPTFLOW_ADNS3080_ADDR_FP_MIN_B_UP   0x1c
#define OPTFLOW_ADNS3080_ADDR_SHUT_MAX_B_LOW 0x1d
#define OPTFLOW_ADNS3080_ADDR_SHUT_MAX_B_UP 0x1e
#define OPTFLOW_ADNS3080_ADDR_SROM_ID		0x1f
#define OPTFLOW_ADNS3080_ADDR_OBSERV		0x3d
#define OPTFLOW_ADNS3080_ADDR_INV_PROD_ID	0x3f
#define OPTFLOW_ADNS3080_ADDR_PIX_BURST		0x40
#define OPTFLOW_ADNS3080_ADDR_MOTION_BURST  0x50
#define OPTFLOW_ADNS3080_ADDR_SROM_LOAD		0x60


#define OPTFLOW_ADNS3080_CONF_LED_MODE		6
#define OPTFLOW_ADNS3080_CONF_SYSTEST		5
#define OPTFLOW_ADNS3080_CONF_RESOLUTION	4

#define OPTFLOW_ADNS3080_EXCONF_NO_PULLUP	2
#define OPTFLOW_ADNS3080_EXCONF_NO_AGC		1
#define OPTFLOW_ADNS3080_EXCONF_FIXED_FR	0


#define OPTFLOW_ADNS3080_FP_UP_6469		0x0E
#define OPTFLOW_ADNS3080_FP_LO_6469		0x7E
#define OPTFLOW_ADNS3080_FP_UP_5000		0x12
#define OPTFLOW_ADNS3080_FP_LO_5000		0xC0
#define OPTFLOW_ADNS3080_FP_UP_3000		0x1F
#define OPTFLOW_ADNS3080_FP_LO_3000		0x40
#define OPTFLOW_ADNS3080_FP_UP_2000		0x2E
#define OPTFLOW_ADNS3080_FP_LO_2000		0xE0

#define OPTFLOW_ADNS3080_US_BETWEEN_WRITES	40
#define OPTFLOW_ADNS3080_MIN_SQUAL		20

#define OPTFLOW_ADNS3080_FOV                    0.202458  // 11.6 degrees, according to ardupilot
#define OPTFLOW_ADNS3080_XRES                   30
#define OPTFLOW_ADNS3080_YRES                   30


/** parameters for the 2nd order low-pass filter
 * calculated in MATLAB using
 *   [num,den] = butter(2,(cutoff_freq/(sample_freq/2)))
 */
/*
#define OFS_BUTTER_NUM_1 +0.067455273889072
#define OFS_BUTTER_NUM_2 +0.134910547778144
#define OFS_BUTTER_NUM_3 +0.067455273889072
//warning, ACCEL_BUTTER_DEN_1 is always one for this filter, so it is omitted here.
#define OFS_BUTTER_DEN_2 -1.142980502539901
#define OFS_BUTTER_DEN_3 +0.412801598096189
*/

#define OFS_BUTTER_NUM_1 +0.139939286866852
#define OFS_BUTTER_NUM_2 +0.279878573733703
#define OFS_BUTTER_NUM_3 +0.139939286866852
//warning, ACCEL_BUTTER_DEN_1 is always one for this filter, so it is omitted here.
#define OFS_BUTTER_DEN_2 -0.699738028273365
#define OFS_BUTTER_DEN_3 +0.259495175740772

#include "math/pprz_algebra_int.h"

void optflow_ADNS3080_init(void);
void optflow_ADNS3080_spi_conf(void);
uint8_t optflow_ADNS3080_readRegister( uint8_t addr);
void optflow_ADNS3080_writeRegister(uint8_t addr, uint8_t val);
void optflow_ADNS3080_writeSROM(void);
void optflow_ADNS3080_captureFrame(void);

void optflow_ADNS3080_test(void);

//h2w
void optflow_ADNS3080_read_OF(void); 
extern bool_t opticflow_data_available;
extern int8_t dx,dy;
extern int8_t dx_filtered, dy_filtered;
extern uint8_t squal;
extern struct Int8Vect2 OF_p;
extern struct Int8Vect2 dOF_p;
extern int8_t ddx;
extern int8_t ddy;
extern int8_t dx_prev;
extern int8_t dy_prev;
extern float dx_scaled;
extern float dy_scaled;
extern float dx_fused;
extern float dy_fused;

#define OpticFlowEvent(_handler) { \
  if (opticflow_data_available) { \
    _handler(); \
    opticflow_data_available = FALSE; \
  } \
}

#endif // USE_OPTFLOW_ADNS3080

#endif // OPTFLOW_ADNS3080_H
