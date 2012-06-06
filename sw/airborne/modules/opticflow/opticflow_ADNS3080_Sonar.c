/** \file opticflow_ADNS3080.h
 *  \brief Handling of ADNS3080 optic flow sensor
 *
 */

/* @TODO:
 * after finishing raw data:
 * compensate for altitude
 * compensate for pitch/roll
 * ignore bad-quality measurements
 * include butterworth filter
 */
#include "std.h"
#include "modules/opticflow/opticflow_ADNS3080.h"
#include "modules/opticflow/opticflow_ADNS3080_srom.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/sys_time.h"

//h2w
#include "modules/sonar/sonar_maxbotix_MB12XX_PWM.h"
#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "math/pprz_algebra.h"

#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

//The CS of the SPI1 header is connected to the Overo.
//So we use the PC4 (SPI1 DRDY) pin as Slave-select for the optic flow sensor.
//Connect the NCS pin of the optic flow breakout to the DRDY pin on the SPI1 header.
#define OfUnselect() GPIOC->BSRR = GPIO_Pin_4
#define OfSelect()   GPIOC->BRR = GPIO_Pin_4

float ofs_meas_dx_prev = 0;
float ofs_meas_dx_prev_prev = 0;
float ofs_filter_val_dx = 0;
float ofs_filter_val_dx_prev = 0;
float ofs_filter_val_dx_prev_prev = 0;

float ofs_meas_dy_prev = 0;
float ofs_meas_dy_prev_prev = 0;
float ofs_filter_val_dy = 0;
float ofs_filter_val_dy_prev = 0;
float ofs_filter_val_dy_prev_prev = 0;

int32_t ofs_itgr_x = 0;
int32_t ofs_itgr_y = 0;

uint8_t isWritingSROM = 0;

//h2w
bool_t opticflow_data_available;
uint8_t squal;
int8_t dx,dy;
int8_t dx_filtered = 0;
int8_t dy_filtered = 0;
#define FORTYFIVE_DEGREES 0.78539816

float dx_scaled = 0;
float dy_scaled = 0;
float dx_fused = 0;
float dy_fused = 0;
const float conv_factor = 0.006156; // conv_factor = (1.0 / (float)(num_pixels * scaler)) * 2.0 * tan(field_of_view / 2.0);
				     // num_pixels=30 pixels; scaler=1.1; FOV=11.6 deg=0.202458rad
const float radians_to_pixels = 162.9968; //radians_to_pixels = (num_pixels * scaler) / field_of_view;
float exp_change_x = 0;
float exp_change_y = 0;
float change_x = 0;
float change_y = 0;
float diff_roll = 0;
float diff_pitch = 0;
float roll = 0;
float pitch = 0;
float _last_roll = 0;
float _last_pitch = 0;
float x_of_m = 0.0;
float y_of_m = 0.0;

int8_t dx_prev = 0;
int8_t dy_prev = 0;
int8_t ddx = 0;
int8_t ddy = 0;

struct Int8Vect2 OF_p;
struct Int8Vect2 dOF_p;

void optflow_ADNS3080_init( void ) {
	opticflow_data_available = FALSE;
	optflow_ADNS3080_spi_conf();
	isWritingSROM = 1;
	optflow_ADNS3080_writeSROM();
	isWritingSROM = 0;

	//config register:
	// 0 0 0 1 0 0 0 0  but OR with reset val: 0x09:
	// 0 0 0 1 1 0 0 1 => 0x19
	//       ^-- 1600 CPI
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_CONF, 0x09|(1<<OPTFLOW_ADNS3080_CONF_RESOLUTION));
	sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);

	//ext_config register:
	// 0 0 0 0 0 0 0 1  => 0x01
	//               ^--- fixed frame rate
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_EXCONF,1<<OPTFLOW_ADNS3080_EXCONF_FIXED_FR);
	sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);

	//optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_EXCONF,0<<OPTFLOW_ADNS3080_EXCONF_FIXED_FR);
	//sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);

	//6469 FPS
	/*optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MIN_B_LOW,OPTFLOW_ADNS3080_FP_LO_6469);
	sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MIN_B_UP,OPTFLOW_ADNS3080_FP_UP_6469);
	sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);*/

	//2000 FPS
        optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MIN_B_LOW,OPTFLOW_ADNS3080_FP_LO_2000);
        sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);
        optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MIN_B_UP,OPTFLOW_ADNS3080_FP_UP_2000);
        sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);

	/*optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MIN_B_LOW,OPTFLOW_ADNS3080_FP_LO_2000);
	sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MIN_B_UP,OPTFLOW_ADNS3080_FP_UP_2000);
	sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);

	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MAX_B_LOW,OPTFLOW_ADNS3080_FP_LO_5000);
        sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);
        optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FP_MAX_B_UP,OPTFLOW_ADNS3080_FP_UP_5000);
        sys_time_usleep(OPTFLOW_ADNS3080_US_BETWEEN_WRITES);*/
}

void optflow_ADNS3080_spi_conf( void ) {
	GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

	//SS config
    OfUnselect();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    OfUnselect();

    //Enable SPI1 Periph clock and GPIOA + AFIO
    //@fixme: is AFIO actually required?
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);

	//Configure GPIOs: SCK (PA5), MISO (PA6) and MOSI(PA7)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_Cmd(SPI1, ENABLE);


	//configure SPI
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	//8 bit MSB first (byte is: 76543210)
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//prescaler 32 is 2MHz SCK... wait... or 64 @FIXME check with scope
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
}


void optflow_ADNS3080_periodic( void ) {
	if (isWritingSROM) {
		return;
	}
	//OfSelect();


	      //ignore the motion register, we don' t need it
	      SPI_I2S_ReceiveData(SPI1);


	      //those are (two's complement) SIGNED integers dx and dy
	      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	      SPI_I2S_SendData(SPI1, 0x00);
	      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	      dx = SPI_I2S_ReceiveData(SPI1);

	      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	      SPI_I2S_SendData(SPI1, 0x00);
	      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	      dy = SPI_I2S_ReceiveData(SPI1);

	      //and the surface quality
	      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	      SPI_I2S_SendData(SPI1, 0x00);
	      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	      squal = SPI_I2S_ReceiveData(SPI1);

	      OfUnselect();


	      ofs_filter_val_dx = OFS_BUTTER_NUM_1*dx
                + OFS_BUTTER_NUM_2*ofs_meas_dx_prev
                + OFS_BUTTER_NUM_3*ofs_meas_dx_prev_prev
                - OFS_BUTTER_DEN_2*ofs_filter_val_dx_prev;
                - OFS_BUTTER_DEN_3*ofs_filter_val_dx_prev_prev;

              ofs_meas_dx_prev_prev = ofs_meas_dx_prev;
              ofs_meas_dx_prev = dx;
              ofs_filter_val_dx_prev_prev = ofs_filter_val_dx_prev;
              ofs_filter_val_dx_prev = ofs_filter_val_dx;

              ofs_filter_val_dy = OFS_BUTTER_NUM_1*dy
                + OFS_BUTTER_NUM_2*ofs_meas_dy_prev
                + OFS_BUTTER_NUM_3*ofs_meas_dy_prev_prev
                - OFS_BUTTER_DEN_2*ofs_filter_val_dy_prev;
                - OFS_BUTTER_DEN_3*ofs_filter_val_dy_prev_prev;

              ofs_meas_dy_prev_prev = ofs_meas_dy_prev;
              ofs_meas_dy_prev = dy;
              ofs_filter_val_dy_prev_prev = ofs_filter_val_dy_prev;
              ofs_filter_val_dy_prev = ofs_filter_val_dy;

              ofs_itgr_x += dx;
              ofs_itgr_y += dy;
	     
	      //h2w 

// 	      EULERS_FLOAT_OF_BFP(ahrs_float.ltp_to_body_euler,ahrs.ltp_to_body_euler);
// 	      roll = ahrs_float.ltp_to_body_euler.phi;
// 	      pitch = ahrs_float.ltp_to_body_euler.theta;
// 	      
// 	      diff_roll     = roll  - _last_roll;
// 	      diff_pitch    = pitch - _last_pitch;

	// only update position if surface quality is good and angle is not over 45 degrees
//         if( squal >= 10 && fabs(roll) <= FORTYFIVE_DEGREES && fabs(pitch) <= FORTYFIVE_DEGREES ) {
// 	      exp_change_x = -diff_pitch*radians_to_pixels;
// 	      exp_change_y = diff_roll*radians_to_pixels;
// 
// 	      change_x = ofs_filter_val_dx - exp_change_x;
// 	      change_y = ofs_filter_val_dy - exp_change_y;
// 	      
     	  dx_scaled = -ofs_filter_val_dy;
     	  dy_scaled = -ofs_filter_val_dx;

//#ifdef SONAR_MAXBOTIX12_H
// 	      dx_scaled = ofs_filter_val_dx*sonar_filtered*conv_factor;
// 	      dy_scaled = ofs_filter_val_dy*sonar_filtered*conv_factor;
//#else
// 	      dx_scaled = ofs_filter_val_dx;
// 	      dy_scaled = ofs_filter_val_dy;
//#endif
// 	      dx_fused = (int8_t)change_x*sonar_filtered*conv_factor;
// 	      dy_fused = (int8_t)change_y*sonar_filtered*conv_factor;
// 	      
// 	      x_of_m += dx_fused;
// 	      y_of_m += dy_fused;
// 	}
// 	      _last_roll = roll;
// 	      _last_pitch = pitch;
// 	      
 	     // 	  RunOnceEvery(50,DOWNLINK_SEND_OFLOW_DATA(DefaultChannel, DefaultDevice, &dx, &dy,&squal););
 	     //	      DOWNLINK_SEND_OFLOW_DATA(DefaultChannel, DefaultDevice, &dx, &dy, &squal);
 	      DOWNLINK_SEND_OFLOW_FILTERED(DefaultChannel, DefaultDevice, &dx_scaled, &dy_scaled, &squal);

 	      opticflow_data_available = TRUE;
//h2w
// 	      int8_t ab = 0;
	     //DOWNLINK_SEND_OFLOW_DATA(DefaultChannel, DefaultDevice, &dx, &dy,&squal, &ab,&ab,&ab,&ab,&ab,&ab);
	    // DOWNLINK_SEND_OFLOW_DATA(DefaultChannel, DefaultDevice, &dx, &dy,&squal, &dx_scaled,&dy_scaled,&dx_fused,&dy_fused,&diff_pitch,&diff_roll);


	//dx      		 = (int8_t)optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_DX);
	//dy	 		 = (int8_t)optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_DY);

//	squal	 		 = optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_SQUAL);

	/*ofs_filter_val_dx = OFS_BUTTER_NUM_1*dx
          + OFS_BUTTER_NUM_2*ofs_meas_dx_prev
          + OFS_BUTTER_NUM_3*ofs_meas_dx_prev_prev
          - OFS_BUTTER_DEN_2*ofs_filter_val_dx_prev;
          - OFS_BUTTER_DEN_3*ofs_filter_val_dx_prev_prev;


        ofs_filter_val_dy = OFS_BUTTER_NUM_1*dy
          + OFS_BUTTER_NUM_2*ofs_meas_dy_prev
          + OFS_BUTTER_NUM_3*ofs_meas_dy_prev_prev
          - OFS_BUTTER_DEN_2*ofs_filter_val_dy_prev;
          - OFS_BUTTER_DEN_3*ofs_filter_val_dy_prev_prev;*/

	//@TODO: maybe add filter?
	//@TODO: maybe check squal?
	//if (squal > OPTFLOW_ADNS3080_MIN_SQUAL)*/

	//now we pull the CS low, and we should keep it low until the transfer is completed!
	/*OfSelect();

	       //wait until the TX buffer is sent. should not be necessary after SS toggle, just for safety
	        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	        SPI_I2S_SendData(SPI1, OPTFLOW_ADNS3080_ADDR_MOTION_BURST);
	        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	        ////while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	        //maybe?
	        //sys_time_usleep(50);

	        //sys_time_usleep(75);
	        SPI_I2S_ReceiveData(SPI1);
	        //sys_time_usleep(45);
	        //sys_time_usleep(25);
	        //sys_time_usleep(2);


	        //sys_time_usleep(2);
	         while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	         SPI_I2S_SendData(SPI1, 0x00);
	         //sys_time_usleep(25);
	         //sys_time_usleep(5);
	         while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	         //sys_time_usleep(75);
	         //sys_time_usleep(5);
	         dx = SPI_I2S_ReceiveData(SPI1);
	         sys_time_usleep(2);
	                    //      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	                     //     SPI_I2S_SendData(SPI1, 0x00);
	         //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	                         //sys_time_usleep(75);
	                         //sys_time_usleep(5);
	                         dy = SPI_I2S_ReceiveData(SPI1);

	                       // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	                       //                                    SPI_I2S_SendData(SPI1, 0x00);
	                          while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	                                                          //sys_time_usleep(75);
	                                                          //sys_time_usleep(5);
	                                                          squal = SPI_I2S_ReceiveData(SPI1);*/
        //sys_time_usleep(12);*/
	//dx=optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_MOTION_BURST);
	//optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_MOTION_BURST);
	//dy=optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_MOTION_BURST);
	//squal=optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_MOTION_BURST);
	//sys_time_usleep(75);
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	//SPI_I2S_SendData(SPI1, 0xff);
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	//dy = SPI_I2S_ReceiveData(SPI1);
	//sys_time_usleep(75);
	//squal = SPI_I2S_ReceiveData(SPI1);
	//dy=optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_DX);
	//dx=optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_DX);


      OfSelect();

      //wait until the TX buffer is sent. should not be necessary after SS toggle, just for safety
      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

      SPI_I2S_SendData(SPI1, OPTFLOW_ADNS3080_ADDR_MOTION_BURST);
      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
      //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);


      sys_time_usleep(2);

      //destroy garabage
      SPI_I2S_ReceiveData(SPI1);

      //sys_time_usleep(45);
      //sys_time_usleep(25);
      //sys_time_usleep(2);
      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
      SPI_I2S_SendData(SPI1, 0x00);
      //sys_time_usleep(25);
      //sys_time_usleep(5);
      //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    //sys_time_usleep(75);
      //sys_time_usleep(5);
}



void optflow_ADNS3080_test( void ) {
	if (isWritingSROM) {
		return;
	}

	uint8_t prodId,revId,motionReg,isMotion,motionOverflow,motionResolution,srom_id;


	prodId 			 = optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_PROD_ID);
	revId 			 = optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_REV_ID);

	motionReg 		 = optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_MOTION);
	isMotion 		 = (motionReg & (1<<7)) > 0;
	motionOverflow 	 = (motionReg & (1<<4)) > 0;
	motionResolution = motionReg & (0x01);

	//those are (two's complement) SIGNED integers
	dx		 		 = (int8_t)optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_DX);
	dy		 		 = (int8_t)optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_DY);

	squal	 		 = optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_SQUAL);

	srom_id	 		 = optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_SROM_ID);

	//DOWNLINK_SEND_OFLOW_DBG(DefaultChannel, &prodId,&revId,&isMotion,&motionOverflow,&motionResolution,&dx,&dy,&squal,&srom_id);
	//optflow_ADNS3080_captureFrame();
}

uint8_t optflow_ADNS3080_readRegister( uint8_t addr) {
  OfSelect();

  //wait until the TX buffer is sent. should not be necessary after SS toggle, just for safety
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  SPI_I2S_SendData(SPI1, addr);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  ////while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  //maybe?
  //sys_time_usleep(50);

  sys_time_usleep(2);
  //destroy garabage
  SPI_I2S_ReceiveData(SPI1);
  //sys_time_usleep(45);
  //sys_time_usleep(25);
  //sys_time_usleep(2);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, 0x00);
  //sys_time_usleep(25);
  //sys_time_usleep(5);
  //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  sys_time_usleep(75);
  //sys_time_usleep(5);
  uint8_t val = SPI_I2S_ReceiveData(SPI1);
  //OfUnselect();
  return val;
}


void optflow_ADNS3080_writeRegister(uint8_t addr, uint8_t val) {
  OfSelect();
  //the MSB of the address should be 1, in order to indicate write mode
  SPI_I2S_SendData(SPI1, (addr|(1<<7)));
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, val);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
  //maybe this can be less than 40us, check with scope!
  sys_time_usleep(40);
  OfUnselect();
}


void optflow_ADNS3080_writeSROM(void) {
	//magic sequence for initiating SROM upload
	optflow_ADNS3080_writeRegister(0x20,0x44);
	optflow_ADNS3080_writeRegister(0x23,0x07);
	optflow_ADNS3080_writeRegister(0x24,0x88);

	//We have to wait one frame period. The minimum framerate is 2000 fps, so one frame lasts 0.0005 s =  500us
	//Now to be sure that we wait long enough, we wait 600us instead of 500us.
	sys_time_usleep(600);

	//Write 0x18 to the SROM_ENABLE register to initiate the burst upload
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_SROM_ENABLE,0x18);

	//now we pull the CS low, and we should keep it low until the transfer is completed!
	OfSelect();
	//the MSB of the address should be 1, in order to indicate write mode
	SPI_I2S_SendData(SPI1, (OPTFLOW_ADNS3080_ADDR_SROM_LOAD|(1<<7)));
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	for (int i=0; i<1986; i++) {
		SPI_I2S_SendData(SPI1, adns3080_srom[i]);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		//we have to sleep 10us before writing the next byte, but for safety we sleep 12us
		sys_time_usleep(12);
	}

	//now pull CS high for 10us, to exit the burst mode and return to normal operation
	OfUnselect();
	sys_time_usleep(10);
}

void optflow_ADNS3080_captureFrame(void) {
	//optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_REV_ID);
	//optflow_ADNS3080_readRegister(OPTFLOW_ADNS3080_ADDR_MOTION);
	uint8_t frame[90];
	frame[3] = 111;
    //after capturing frames, the module has to be resetted/powercycled before it can resume normal operation!

	//initialize frame capture mode
	optflow_ADNS3080_writeRegister(OPTFLOW_ADNS3080_ADDR_FRAMECAP,0x83);

	//We have to wait 3 frame periods + 10us.
	//The minimum framerate is 2000 fps, so one frame lasts 0.0005 s =  500us
	//So we have to wait 3*500+10=1510us
    //Now to be sure that we wait long enough, we wait 1520us
	sys_time_usleep(1520);

	//we can not use the readRegister function to get the image, as we have to keep CS low to stay in burst mode

	//now we pull the CS low, and we should keep it low until the transfer is completed!
	OfSelect();
	SPI_I2S_SendData(SPI1, OPTFLOW_ADNS3080_ADDR_PIX_BURST);

	//@FIXME: maybe send 0x00 first?
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, 0x00);

	//for now, we will send onlyone frame (900 pixels) a time.
	//@FIXME: change this to read 1 2/3 frame (1537 pixels), to get a higher framerate
	for(int i=0;i<90;i++) {
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		frame[i] = SPI_I2S_ReceiveData(SPI1);
		//we have to sleep 10us before writing the next byte, but for safety we sleep 12us
		sys_time_usleep(12);
	}
	//now pull CS high for 10us, to stop this read session
	//this does *not* mean that the sensor returns to normal operation!
	//to resume to normal operation after burst captue, the sensor has to be power-cycled!
	OfUnselect();


	sys_time_usleep(10); //can we skip this? does the downlink send action take enough time? @todo check with scope

	//DOWNLINK_SEND_OFLOW_FRAMECAP(DefaultChannel,90,frame);
	
	return;
}

//read the optical flows and compute their derivatives
void optflow_ADNS3080_read_OF(void) {
    optflow_ADNS3080_periodic();
    dx_filtered = (int8_t)dx_scaled;
    dy_filtered = (int8_t)dy_scaled;
	VECT2_ASSIGN(OF_p,dx_filtered,dy_filtered);
	ddx = dx_filtered-dx_prev;
	ddy = dy_filtered-dy_prev;
	
	VECT2_ASSIGN(dOF_p,ddx,ddy);
	dx_prev = dx_filtered;
	dy_prev = dy_filtered;
	
// 	DOWNLINK_SEND_GUIDANCE_OF(DefaultChannel, DefaultDevice, &dx, &dy, &ddx,&ddy)
// 	RunOnceEvery(10,DOWNLINK_SEND_GUIDANCE_OF(DefaultChannel, DefaultDevice, &(OF_p.x), &(OF_p.y), &(dOF_p.x),&(dOF_p.y)));
//  	DOWNLINK_SEND_GUIDANCE_OF(DefaultChannel, DefaultDevice, &(OF_p.x), &(OF_p.y), &(dOF_p.x),&(dOF_p.y));
}

