/**
 * @file    adc_driver.h
 * @brief   STM32F4xx ADC Driver — Public API
 * @version 2.0.0
 *
 * @details ADC1 driver supporting:
 *          - Single conversion (polling)
 *          - Continuous conversion (interrupt)
 *          - Scan mode for multiple channels
 *          - DMA-based continuous sampling for high-speed data acquisition
 *          - Configurable resolution (12/10/8/6 bit)
 *          - Configurable sample time per channel
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADC Register Bit Definitions
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* SR */
#define ADC_SR_AWD (1 << 0)
#define ADC_SR_EOC (1 << 1) /*!< End of conversion         */
#define ADC_SR_JEOC (1 << 2)
#define ADC_SR_JSTRT (1 << 3)
#define ADC_SR_STRT (1 << 4) /*!< Regular channel started   */
#define ADC_SR_OVR (1 << 5)  /*!< Overrun                   */

/* CR1 */
#define ADC_CR1_AWDCH_MASK (0x1F)
#define ADC_CR1_EOCIE (1 << 5) /*!< EOC interrupt enable      */
#define ADC_CR1_AWDIE (1 << 6)
#define ADC_CR1_JEOCIE (1 << 7)
#define ADC_CR1_SCAN (1 << 8) /*!< Scan mode enable          */
#define ADC_CR1_AWDSGL (1 << 9)
#define ADC_CR1_JAUTO (1 << 10)
#define ADC_CR1_DISCEN (1 << 11)
#define ADC_CR1_JDISCEN (1 << 12)
#define ADC_CR1_DISCNUM_MASK (0x7 << 13)
#define ADC_CR1_JAWDEN (1 << 22)
#define ADC_CR1_AWDEN (1 << 23)
#define ADC_CR1_RES_MASK (0x3 << 24) /*!< Resolution mask           */
#define ADC_CR1_OVRIE (1 << 26)      /*!< Overrun interrupt enable  */

/* CR2 */
#define ADC_CR2_ADON (1 << 0)   /*!< ADC ON/OFF                */
#define ADC_CR2_CONT (1 << 1)   /*!< Continuous conversion     */
#define ADC_CR2_DMA (1 << 8)    /*!< DMA access mode           */
#define ADC_CR2_DDS (1 << 9)    /*!< DMA disable selection     */
#define ADC_CR2_EOCS (1 << 10)  /*!< End of conversion selection */
#define ADC_CR2_ALIGN (1 << 11) /*!< Data alignment            */
#define ADC_CR2_JEXTSEL_MASK (0xF << 16)
#define ADC_CR2_JEXTEN_MASK (0x3 << 20)
#define ADC_CR2_JSWSTART (1 << 22)
#define ADC_CR2_EXTSEL_MASK (0xF << 24)
#define ADC_CR2_EXTEN_MASK (0x3 << 28)
#define ADC_CR2_SWSTART (1 << 30) /*!< Start conversion          */

/* Common CCR */
#define ADC_CCR_ADCPRE_MASK (0x3 << 16)
#define ADC_CCR_TSVREFE (1 << 23) /*!< Temp sensor/VREF enable   */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  ADC_RES_12BIT = 0x00,
  ADC_RES_10BIT = 0x01,
  ADC_RES_8BIT = 0x02,
  ADC_RES_6BIT = 0x03
} adc_resolution_t;

typedef enum {
  ADC_SAMPLETIME_3 = 0x00,   /*!< 3 cycles                  */
  ADC_SAMPLETIME_15 = 0x01,  /*!< 15 cycles                 */
  ADC_SAMPLETIME_28 = 0x02,  /*!< 28 cycles                 */
  ADC_SAMPLETIME_56 = 0x03,  /*!< 56 cycles                 */
  ADC_SAMPLETIME_84 = 0x04,  /*!< 84 cycles                 */
  ADC_SAMPLETIME_112 = 0x05, /*!< 112 cycles                */
  ADC_SAMPLETIME_144 = 0x06, /*!< 144 cycles                */
  ADC_SAMPLETIME_480 = 0x07  /*!< 480 cycles                */
} adc_sample_time_t;

typedef enum {
  ADC_PRESCALER_DIV2 = 0x00,
  ADC_PRESCALER_DIV4 = 0x01,
  ADC_PRESCALER_DIV6 = 0x02,
  ADC_PRESCALER_DIV8 = 0x03
} adc_prescaler_t;

typedef enum { ADC_ALIGN_RIGHT = 0, ADC_ALIGN_LEFT = 1 } adc_alignment_t;

typedef enum {
  ADC_CHANNEL_0 = 0,
  ADC_CHANNEL_1,
  ADC_CHANNEL_2,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_5,
  ADC_CHANNEL_6,
  ADC_CHANNEL_7,
  ADC_CHANNEL_8,
  ADC_CHANNEL_9,
  ADC_CHANNEL_10,
  ADC_CHANNEL_11,
  ADC_CHANNEL_12,
  ADC_CHANNEL_13,
  ADC_CHANNEL_14,
  ADC_CHANNEL_15,
  ADC_CHANNEL_16,
  ADC_CHANNEL_17,
  ADC_CHANNEL_18
} adc_channel_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*adc_callback_t)(void *context, uint16_t value);

/** @brief ADC configuration */
typedef struct {
  adc_resolution_t resolution; /*!< Conversion resolution             */
  adc_prescaler_t prescaler;   /*!< ADC clock prescaler               */
  adc_alignment_t alignment;   /*!< Data alignment (left/right)       */
  uint8_t continuous;          /*!< Continuous mode (0 or 1)          */
  uint8_t scan_mode;           /*!< Scan mode enable (0 or 1)         */
  uint8_t num_channels;        /*!< Number of channels in scan seq    */
} ADC_Config_t;

typedef struct {
  ADC_RegDef_t *pADCx;     /*!< ADC peripheral base address       */
  ADC_Config_t config;     /*!< Configuration                     */
  adc_callback_t callback; /*!< EOC callback                      */
  void *cb_context;        /*!< User context                      */
} ADC_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Initialization */
drv_status_t ADC_Init(ADC_Handle_t *pHandle);
drv_status_t ADC_DeInit(ADC_RegDef_t *pADCx);
void ADC_PeriClockControl(uint8_t en);

/* Channel Configuration */
void ADC_ConfigChannel(ADC_RegDef_t *pADCx, adc_channel_t channel, uint8_t rank,
                       adc_sample_time_t sample_time);

/* Polling (Blocking) — Single Conversion */
drv_status_t ADC_StartConversion(ADC_RegDef_t *pADCx);
uint16_t ADC_ReadValue(ADC_RegDef_t *pADCx);
uint16_t ADC_ReadChannel(ADC_Handle_t *pHandle, adc_channel_t channel,
                         adc_sample_time_t sample_time);

/* Interrupt (Non-Blocking) */
drv_status_t ADC_StartConversion_IT(ADC_Handle_t *pHandle);
void ADC_IRQHandler(ADC_Handle_t *pHandle);

/* DMA Continuous Sampling */
drv_status_t ADC_StartContinuousDMA(ADC_Handle_t *pHandle, uint16_t *pBuffer,
                                    uint32_t len);
void ADC_StopContinuousDMA(ADC_Handle_t *pHandle);

/* Control */
void ADC_Enable(ADC_RegDef_t *pADCx);
void ADC_Disable(ADC_RegDef_t *pADCx);
void ADC_RegisterCallback(ADC_Handle_t *pHandle, adc_callback_t cb, void *ctx);

/* Internal temperature sensor */
void ADC_EnableTempSensor(void);
uint16_t ADC_ReadTemperature(ADC_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DRIVER_H */
