#include "modules/sonar/sonar_maxbotix_MB12XX_PWM.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/tim.h>
#include <stm32/misc.h>

#include "mcu_periph/uart.h"
#include "messages.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/ins.h"

//#include "sys_time.h"

uint16_t sonar_meas;
uint16_t sonar_meas_raw;
uint32_t sonar_filtered;


int32_t sonar_meas_prev = 0;
int32_t sonar_meas_prev_prev = 0;
int32_t sonar_filter_val = 0;
int32_t sonar_filter_val_prev = 0;
int32_t sonar_filter_val_prev_prev = 0;

uint8_t sonar_spike_cnt = 0;

int32_t sonar_meas_real, sonar_alt_cm_bfp;

bool_t sonar_data_available;

void SONAR_MAXBOTIX12_IRQ_HANDLER(void);

void
maxbotix12_init(void)
{
  sonar_meas_raw = 0;
  sonar_data_available = FALSE;

  //GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);


  /* TIM3 channel 4 pin (PB1) configuration */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = SONAR_MAXBOTIX12_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SONAR_MAXBOTIX12_GPIO, &GPIO_InitStructure);

  //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
  //GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

  /* TIM5 clock enable */
  //RCC_APB1PeriphClockCmd(SONAR_MAXBOTIX12_TIM_PERIPH, ENABLE);
  RCC_APB2PeriphClockCmd(SONAR_MAXBOTIX12_TIM_PERIPH, ENABLE);

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(SONAR_MAXBOTIX12_GPIO_PERIPH, ENABLE);

  /* Time Base configuration */

  /* The sensor has a pulse duration of 58us per cm.
   * It can measure a range from about 20 cm to 1068 cm, that is a pulsewidth between 1.160 ms and 61.944 ms.
   *
   * Calculation example:
   * Our clock is running at 72 MHz
   * At 16 bits resolution, the maximum pulse we could measure would be
   * (0xffff / 72 000 000) * 1000 = 0.910 ms
   *
   * So our prescaler has to be:
   *  (58 * (10^(-6)) * 1068) / (0xffff / 72 000 000) = 68,054
   *
   *  We don't want it to overflow when it's almost at it's maximum, as that would cause the system to think
   *  the A/C is on the ground.
   *+
   *  So we want prescaler 69. We have to se 68 as value, as the clock is divided by ( 1 + prescaler ).
   *
   *  @TODO make this dependant of the APB_CLK value
   */

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = SONAR_MAXBOTIX12_TIM_PRESCALER;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InternalClockConfig(SONAR_MAXBOTIX12_TIM);
  TIM_TimeBaseInit(SONAR_MAXBOTIX12_TIM, &TIM_TimeBaseStructure);

  /* TIM3 configuration
   * Input signal is connected to TIM3 CH4 pin (PB1)
   * The Rising edge is used as active edge.
   */
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = SONAR_MAXBOTIX12_TIM_CHANNEL;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x00;

  /* Initialize PWM Input measurement
   * See page 301 of the STM32F103 reference manual. */
  TIM_PWMIConfig(SONAR_MAXBOTIX12_TIM, &TIM_ICInitStructure);

  /* Enable the TIM3 global Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SONAR_MAXBOTIX12_IRQ_CHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 16;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* we can use TI2FP2 because it is bound to TIM channel 2 , which we are using */
  TIM_SelectInputTrigger(SONAR_MAXBOTIX12_TIM, SONAR_MAXBOTIX12_TIM_TS);

  TIM_SelectSlaveMode(SONAR_MAXBOTIX12_TIM, TIM_SlaveMode_);
  TIM_SelectMasterSlaveMode(SONAR_MAXBOTIX12_TIM, TIM_MasterSlaveMode_Enable);

  /* TIM3 enable counter */
  TIM_Cmd(SONAR_MAXBOTIX12_TIM, ENABLE);

  /* Enable the IRQ
   * We need CC1 only. CC2 would contain the PWM period (that is, 10Hz)
   */
  TIM_ITConfig(SONAR_MAXBOTIX12_TIM, TIM_IT_CC1, ENABLE);
}

void SONAR_MAXBOTIX12_IRQ_HANDLER(void)
{
  if (TIM_GetITStatus(SONAR_MAXBOTIX12_TIM, TIM_IT_CC1) == SET)
    {
      TIM_ClearITPendingBit(SONAR_MAXBOTIX12_TIM, TIM_IT_CC1);
      sonar_meas = TIM_GetCapture1(SONAR_MAXBOTIX12_TIM);
      /* pulse_cnts to actual pulse width in us:
       *   pulse width = pulse_cnts * (prescaler+1)/(actual clock)
       *   with 58us per centimeter, the alt in cm is:
       */
      //int32_t alt_mm = pulse_cnts * 10 * (69/72) / 58;
      //sonar_alt

      //if ((sonar_meas > (sonar_filter_val + 1500) || sonar_meas < (sonar_filter_val - 1500))
      if ((sonar_meas > (sonar_filter_val + 3000) || sonar_meas < (sonar_filter_val - 3000))
          && (sonar_spike_cnt < 5)) {
        sonar_spike_cnt++;
        sonar_meas = sonar_filter_val;
      }
      else {
          sonar_spike_cnt = 0;
      }

      //@TODO: check for angle > 10 deg ==> don't use value

      sonar_filter_val = SONAR_MAXBOTIX12_BUTTER_NUM_1*sonar_meas
      + SONAR_MAXBOTIX12_BUTTER_NUM_2*sonar_meas_prev
    //  + SONAR_MAXBOTIX12_BUTTER_NUM_3*sonar_meas_prev_prev
      - SONAR_MAXBOTIX12_BUTTER_DEN_2*sonar_filter_val_prev;
    //  - SONAR_MAXBOTIX12_BUTTER_DEN_3*sonar_filter_val_prev_prev;

      sonar_meas_prev_prev = sonar_meas_prev;
      sonar_meas_prev = sonar_meas;
      sonar_filter_val_prev_prev = sonar_filter_val_prev;
      sonar_filter_val_prev = sonar_filter_val;

      /** using float
      float factor_cm = 0.0165229885;
      float alt_cm_flt = sonar_meas * factor_cm * (1<<8);
      sonar_meas_real = alt_cm_flt;*/


      //((69 / 72) / 58) * (2^16) = 1082.85057
      //uint16_t conv_factor_cm = 1083;
      //sonar_alt_cm_bfp = conv_factor_cm*sonar_filter_val;
      //sonar_filtered = conv_factor_cm*sonar_filter_val;

      //((69 / 72) / 58) / 100 * (2^8) = 0.0422988506
      float conv_factor_m = 0.0422988506;
      float alt_m_bfp_fl = conv_factor_m*sonar_filter_val;
      sonar_filtered = alt_m_bfp_fl;
      //ins_sonar_initialised = TRUE;

      ins_ext_alt = -sonar_filtered; //NED
//       ins_ext_alt_active = TRUE;
      ins_update_module_altimeter();
      //sonar_filtered = conv_factor_cm*sonar_filter_val;
      //DOWNLINK_SEND_VFF(DefaultChannel, &alt_mm_flt,0,0,0,0,0,0);
      //DOWNLINK_SEND_INS_Z(DefaultChannel, DefaultDevice,&sonar_filtered,&sonar_spike_cnt,0,0);
    }
  DOWNLINK_SEND_SONAR_Z(DefaultChannel, DefaultDevice, &sonar_meas, &sonar_filtered); //h2w
}

bool_t SonarAlmostGroundDetect(void) {
  if (sonar_filtered != 0 && sonar_filtered < 51) {
    return 1;
  }
  return 0;
}
