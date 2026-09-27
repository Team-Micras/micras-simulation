/**
 * @file
 *
 * @brief The STM32 handle types and HAL constants a fake Cube layer builds on.
 *
 * @note The micras_hal headers name STM32 handle types in their configurations,
 *       and a robot's target.hpp names handles, channels and SPI modes. On a PC
 *       none of the vendor HAL exists, so a robot's fake Cube layer includes this
 *       file from its main.h and adds only what is its own: its handles, pins,
 *       init functions and clock tree. The types carry just the fields the host
 *       backend reads or writes; the constants keep the vendor's values where the
 *       firmware could compare them.
 */

#ifndef STM32_HOST_H
#define STM32_HOST_H

#include <cstdint>

/*****************************************
 * GPIO
 *****************************************/

/**
 * @brief A GPIO port, identified by its address alone.
 */
struct GPIO_TypeDef {
    /**
     * @brief Letter of the port, for messages.
     */
    char name;
};

///@{
inline constexpr uint16_t GPIO_PIN_0{0x0001U};
inline constexpr uint16_t GPIO_PIN_1{0x0002U};
inline constexpr uint16_t GPIO_PIN_2{0x0004U};
inline constexpr uint16_t GPIO_PIN_3{0x0008U};
inline constexpr uint16_t GPIO_PIN_4{0x0010U};
inline constexpr uint16_t GPIO_PIN_5{0x0020U};
inline constexpr uint16_t GPIO_PIN_6{0x0040U};
inline constexpr uint16_t GPIO_PIN_7{0x0080U};
inline constexpr uint16_t GPIO_PIN_8{0x0100U};
inline constexpr uint16_t GPIO_PIN_9{0x0200U};
inline constexpr uint16_t GPIO_PIN_10{0x0400U};
inline constexpr uint16_t GPIO_PIN_11{0x0800U};
inline constexpr uint16_t GPIO_PIN_12{0x1000U};
inline constexpr uint16_t GPIO_PIN_13{0x2000U};
inline constexpr uint16_t GPIO_PIN_14{0x4000U};
inline constexpr uint16_t GPIO_PIN_15{0x8000U};

///@}

/*****************************************
 * Timers
 *****************************************/

/**
 * @brief The registers of a timer the host backend models.
 *
 * @note kernel_clock is not a register: it is the frequency the timer counts
 *       at, which on the chip comes from the clock tree the init functions
 *       configure. The fake init function writes it.
 */
struct TIM_TypeDef {
    uint32_t CR1;
    uint32_t CCER;
    uint32_t PSC;
    uint32_t ARR;
    uint32_t kernel_clock;
};

/**
 * @brief Base configuration of a timer, as the init function fills it.
 */
struct TIM_Base_InitTypeDef {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
};

/**
 * @brief State of a timer handle.
 */
enum HAL_TIM_StateTypeDef : uint8_t {
    HAL_TIM_STATE_RESET = 0,
    HAL_TIM_STATE_READY = 1,
};

/**
 * @brief A timer handle.
 */
struct TIM_HandleTypeDef {
    TIM_TypeDef*         Instance;
    TIM_Base_InitTypeDef Init;
    HAL_TIM_StateTypeDef State;
};

///@{
inline constexpr uint32_t TIM_CHANNEL_1{0x00000000U};
inline constexpr uint32_t TIM_CHANNEL_2{0x00000004U};
inline constexpr uint32_t TIM_CHANNEL_3{0x00000008U};
inline constexpr uint32_t TIM_CHANNEL_4{0x0000000CU};
inline constexpr uint32_t TIM_CHANNEL_ALL{0x0000003CU};
///@}

///@{
inline constexpr uint32_t TIM_CR1_CMS{0x3U << 5U};
inline constexpr uint32_t TIM_CCER_CC1P{0x1U << 1U};
inline constexpr uint32_t TIM_COUNTERMODE_UP{0x00000000U};
inline constexpr uint32_t TIM_COUNTERMODE_CENTERALIGNED1{0x1U << 5U};

///@}

/*****************************************
 * ADC
 *****************************************/

/**
 * @brief Configuration of an ADC, as the init function fills it.
 */
struct ADC_InitTypeDef {
    uint32_t Resolution;
    uint32_t NbrOfConversion;
};

///@{
inline constexpr uint32_t HAL_ADC_STATE_RESET{0x00000000U};
inline constexpr uint32_t HAL_ADC_STATE_READY{0x00000001U};

///@}

/**
 * @brief An ADC handle.
 */
struct ADC_HandleTypeDef {
    ADC_InitTypeDef Init;
    uint32_t        State;
};

/*****************************************
 * SPI
 *****************************************/

///@{
inline constexpr uint32_t SPI_POLARITY_LOW{0x00000000U};
inline constexpr uint32_t SPI_POLARITY_HIGH{0x1U << 25U};
inline constexpr uint32_t SPI_PHASE_1EDGE{0x00000000U};
inline constexpr uint32_t SPI_PHASE_2EDGE{0x1U << 24U};

///@}

/**
 * @brief Configuration of an SPI bus, as the init function fills it.
 */
struct SPI_InitTypeDef {
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
};

/**
 * @brief State of an SPI handle.
 */
enum HAL_SPI_StateTypeDef : uint8_t {
    HAL_SPI_STATE_RESET = 0,
    HAL_SPI_STATE_READY = 1,
};

/**
 * @brief An SPI handle.
 */
struct SPI_HandleTypeDef {
    SPI_InitTypeDef      Init;
    HAL_SPI_StateTypeDef State;
};

/*****************************************
 * UART
 *****************************************/

/**
 * @brief State of a UART handle.
 */
enum HAL_UART_StateTypeDef : uint8_t {
    HAL_UART_STATE_RESET = 0x00,
    HAL_UART_STATE_READY = 0x20,
    HAL_UART_STATE_BUSY_RX = 0x22,
};

/**
 * @brief Configuration of a UART, as the init function fills it.
 */
struct UART_InitTypeDef {
    uint32_t BaudRate;
};

/**
 * @brief A UART handle.
 */
struct UART_HandleTypeDef {
    UART_InitTypeDef      Init;
    HAL_UART_StateTypeDef gState;
    HAL_UART_StateTypeDef RxState;
};

/*****************************************
 * CRC
 *****************************************/

///@{
inline constexpr uint8_t  DEFAULT_POLYNOMIAL_ENABLE{0x00U};
inline constexpr uint8_t  DEFAULT_POLYNOMIAL_DISABLE{0x01U};
inline constexpr uint8_t  DEFAULT_INIT_VALUE_ENABLE{0x00U};
inline constexpr uint8_t  DEFAULT_INIT_VALUE_DISABLE{0x01U};
inline constexpr uint32_t CRC_POLYLENGTH_32B{0x00000000U};
inline constexpr uint32_t CRC_POLYLENGTH_16B{0x1U << 3U};
inline constexpr uint32_t CRC_POLYLENGTH_8B{0x2U << 3U};
inline constexpr uint32_t CRC_POLYLENGTH_7B{0x3U << 3U};
inline constexpr uint32_t CRC_INPUTDATA_INVERSION_NONE{0x00000000U};
inline constexpr uint32_t CRC_INPUTDATA_INVERSION_BYTE{0x1U << 5U};
inline constexpr uint32_t CRC_INPUTDATA_INVERSION_HALFWORD{0x2U << 5U};
inline constexpr uint32_t CRC_INPUTDATA_INVERSION_WORD{0x3U << 5U};
inline constexpr uint32_t CRC_OUTPUTDATA_INVERSION_DISABLE{0x00000000U};
inline constexpr uint32_t CRC_OUTPUTDATA_INVERSION_ENABLE{0x1U << 7U};
inline constexpr uint32_t CRC_INPUTDATA_FORMAT_BYTES{0x00000001U};
inline constexpr uint32_t CRC_INPUTDATA_FORMAT_HALFWORDS{0x00000002U};
inline constexpr uint32_t CRC_INPUTDATA_FORMAT_WORDS{0x00000003U};
inline constexpr uint32_t DEFAULT_CRC32_POLY{0x04C11DB7U};
inline constexpr uint32_t DEFAULT_CRC_INITVALUE{0xFFFFFFFFU};

///@}

/**
 * @brief Configuration of the CRC unit, as the init function fills it.
 */
struct CRC_InitTypeDef {
    uint8_t  DefaultPolynomialUse;
    uint8_t  DefaultInitValueUse;
    uint32_t GeneratingPolynomial;
    uint32_t CRCLength;
    uint32_t InitValue;
    uint32_t InputDataInversionMode;
    uint32_t OutputDataInversionMode;
};

/**
 * @brief The CRC unit's handle.
 */
struct CRC_HandleTypeDef {
    CRC_InitTypeDef Init;
    uint32_t        InputDataFormat;
};

/*****************************************
 * FMAC
 *****************************************/

/**
 * @brief The filter accelerator's handle; nothing on the host uses it.
 */
struct FMAC_HandleTypeDef {
    uint32_t State;
};

#endif  // STM32_HOST_H
