#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "flash_map.hpp"

#include "app_framework.hpp"
#include <xrobot_main.hpp>

using namespace LibXR;

/* User Code Begin 1 */
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/* DMA Resources */
static uint16_t adc1_buf[16];
static uint8_t usart1_tx_buf[128];
static uint8_t usart1_rx_buf[128];
static uint8_t i2c1_buf[32];

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */

  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32Timebase timebase;
  PlatformInit();
  STM32PowerManager power_manager;

  /* GPIO Configuration */

  STM32ADC adc1(&hadc1, adc1_buf, {ADC_CHANNEL_0}, 3.3);
  auto adc1_adc_channel_0 = adc1.GetChannel(0);
  UNUSED(adc1_adc_channel_0);


  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  /* I2C1: PB6 = SCL, PB7 = SDA。DMA1_CH6 = I2C1_TX、DMA1_CH7 = I2C1_RX ——
     STM32F103 上这两条通道的请求映射是硬连线的，原先归 USART2_RX/TX 使用；为给
     I2C1 让路，USART2 及其 DMA 已从 Clock.ioc 移除（PA2/PA3 随之释放）。

     第三个实参 dma_enable_min_size = 2：LibXR 的判定条件是
     `size > dma_enable_min_size` 走 DMA 分支，因此：
       1~2 字节 -> BLOCK 阻塞路径（HAL_I2C_Mem_Read 在这两个长度上已验证正确）；
       >=3 字节 -> DMA 路径，绕开 STM32F1 HAL 阻塞接收在 Size>2 时把末字节复制成
                   前一字节的缺陷（该缺陷会让 DMP 固件的 16 字节回读校验必然失败）。
     MPU6050/DMP 的多字节传输为 3/6/12/16/32 字节，全部落在 DMA 分支内。
     缓冲区仍取 32 字节（= DMP FIFO 整包长度）；若日后加大 DMP 特征掩码使包长超过
     32 字节，必须同步增大 i2c1_buf 与 libxr_config.yaml 的 i2c1.buffer_size。 */
  STM32I2C i2c1(&hi2c1, i2c1_buf, 2);

  /* Terminal Configuration */
  /* 把 STDIO 输出接到 USART1（PA9/PA10, 115200），用于 RamFS 调试命令回显。 */
  STDIO::read_ = nullptr;
  STDIO::write_ = usart1.write_port_;

  RamFS ramfs("XRobot");

  LibXR::HardwareContainer peripherals{
    LibXR::Entry<LibXR::PowerManager>({power_manager, {"power_manager"}}),
    LibXR::Entry<LibXR::ADC>({adc1_adc_channel_0, {"adc1_adc_channel_0"}}),
    LibXR::Entry<LibXR::UART>({usart1, {"usart1"}}),
    LibXR::Entry<LibXR::I2C>({i2c1, {"i2c_mpu6050", "i2c1", "I2C1"}}),
    LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}})
  };

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  XRobotMain(peripherals);
  /* User Code End 3 */
}
