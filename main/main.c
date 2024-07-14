/**
 * @brief This example demonstrates how to read 16 bits parallel data abusing the camera peripheral and using GDMA.
 * @author odelot
 * @date 2021-07-01
 *
 * The goal is be able to fast read data in parallel without CPU load using (G)DMA. G stands for General Purpose DMA
 *
 * This example helps understand how we can use the camera peripheral to read data in parallel and use GDMA to transfer the data to memory.
 *
 * It is set to low speed (157480 Hz) to make it easier to understand the code and debug it. If you want to increase the read speed you need
 * to change the number of DMA descriptors and the size of each descriptor. Also, you will need to remove code that is used for debugging / testing.
 *
 * The way cam_bit_order and cam_byte_order are set, together with the fact that ESP32 is a little-endian chip, makes the data be read as unsigned words
 * where the first configured input pin (in this case GPIO_NUM_1) is the most significant bit of the word.
 *
 * Example with this code configuration (cam_bit_order = 1 and cam_byte_order = 0):
 *
 *  16 bit word to read: 0x0F51 (or 3921 in decimal)
 *    Binary to be read: 0   0   0   0   1   1   1   1   0   1   0   1   0   0   0   1
 *            GPIO pins: 1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16
 *
 *
 */
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/gpio.h>
#include <rom/gpio.h>

#include <sdkconfig.h>
#include <esp_log.h>

#include <hal/gpio_ll.h>
#include <hal/clk_gate_ll.h>
#include <hal/dma_types.h>

#include <soc/lcd_cam_struct.h>
#include <soc/gdma_reg.h>
#include <soc/gdma_struct.h>
#include <soc/gdma_periph.h>

#if __has_include("esp_private/gdma.h")
#include "esp_private/gdma.h"
#endif

/**
 * @brief Configuration for the camera peripheral and DMA
 */

#if CONFIG_LCD_CAM_ISR_IRAM_SAFE
#define CAMERA_ISR_IRAM_FLAG ESP_INTR_FLAG_IRAM
#else
#define CAMERA_ISR_IRAM_FLAG 0
#endif

#define ALIGNMENT_SRAM 4

// dma config variables
dma_descriptor_t *dma_descriptors;
uint8_t *dma_buffer;
int dma_channel_id = -1;

// N DMA descriptors of M bytes each (you can change this - it matters for higher read speeds, since it impacts the number of interrupts generated)
// the max value for DMA_node_buffer_size is 4092, but you can use many descriptors until you fill the DMA memory limit (480KB of internal memory at most)
uint32_t dma_node_buffer_size = 2; // max 4092
uint32_t dma_node_cnt = 2;         // max 117 for internal ram - I set to 2 for testing, since it easy to spot the DMA descriptors flipping (from 0 to 1 and then back to 0 as it is a circular buffer)

/**
 * @brief snippet variables used for debugging / testing
 */

// variables for debugging / testing
volatile uint16_t last_word_to_send = 0;
volatile uint16_t word_to_send = 0;
volatile uint32_t dma_eof_address = 0;
volatile int dma_descriptor_idx = -1;

const char *MAIN_TAG = "16bits_parallel_read_dma";

/**
 * @brief static functions declarations to handle DMA allocation and interruptions
 */

// allocate DMA descriptors
static dma_descriptor_t *allocate_dma_descriptors(uint32_t descriptorCount, uint16_t size, uint8_t *buffer)
{
  dma_descriptor_t *descriptors = (dma_descriptor_t *)heap_caps_aligned_calloc(ALIGNMENT_SRAM, 1, descriptorCount * sizeof(dma_descriptor_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  if (descriptors == NULL)
  {
    return descriptors;
  }

  for (int x = 0; x < descriptorCount; x++)
  {
    descriptors[x].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
    descriptors[x].dw0.suc_eof = 0;
    descriptors[x].next = &descriptors[x + 1];
    descriptors[x].dw0.size = size;
    descriptors[x].dw0.length = descriptors[x].dw0.size;
    descriptors[x].buffer = (buffer + size * x);
  }

  // we are using a ring dma buffer, so the last descriptor should point to the first one
  descriptors[descriptorCount - 1].next = &descriptors[0];
  return descriptors;
}

// empty handler for vsync interrupt
static void IRAM_ATTR vsyncHandler(void *arg)
{
}

// DMA in (read) suc (success) eof (end of file) interrupt handler
static void IRAM_ATTR dma_in_suc_eof_isr(void *arg)
{
  int dma_channel = ((int *)arg)[0];
  if (GDMA.channel[dma_channel].in.int_st.in_suc_eof)
  {
    // for debugging / testing, we are storing the last word read and the descriptor address that triggered the interrupt
    dma_eof_address = GDMA.channel[dma_channel].in.suc_eof_des_addr;
    uint16_t *eof_dma_buffer = (uint16_t *)((dma_descriptor_t *)dma_eof_address)->buffer;

    // find the descriptor index of the last word read - REMOVE THIS FOR HIGHER SPEEDS - it is just for debugging / understanding what is going on
    for (int i = 0; i < dma_node_cnt; i++)
    {
      if (((uint32_t)&dma_descriptors[i]) == (dma_eof_address))
      {
        dma_descriptor_idx = i;
      }
    }
    word_to_send = eof_dma_buffer[0];
    // send to queue if you need to process the data in another task and want to avoid reading DMA data outside the interrupt (not recommended)
  }
  GDMA.channel[dma_channel].in.int_clr.in_suc_eof = 1; // clear interrupt
}

void app_main(void)
{
  // define input and output pins for using camera peripheral
  // for this example we are using just the master clock pin
  int masterClockPin = GPIO_NUM_0;
  // int pixelClockPin = GPIO_NUM_17;
  // int vSyncPin = GPIO_NUM_18;
  // int hSyncPin = GPIO_NUM_21;
  // int hSyncEnablePin = GPIO_NUM_33;

  lcd_cam_dev_t *dev = &LCD_CAM;

  // config input data pins
  const int pin_data[] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_16};

  // auxiliar variables
  int32_t _freq_read; // Hz
  uint32_t dma_buffer_size;

  // following the steps found on the technical reference manual of the ESP32-S3

  // 1. Configure clock according to Section 29.3.3. Note that in slave mode, the module clock frequency
  // should be two times faster than the PCLK frequency of the image sensor.

  // configure to read at _freq_read Hz

  // choose source clock for camera peripheral (40, 160 or 240 MHz)  // 0= disabled  // 1=XTAL CLOCK 40MHz / 2=240MHz / 3=160MHz
  uint8_t clksel = 1;

  // define clk_freq_mhz according to clksel just for logging / defining the _freq_read variable
  uint8_t clk_freq_mhz = 40;
  if (clksel == 2)
  {
    clk_freq_mhz = 240;
  }
  else if (clksel == 3)
  {
    clk_freq_mhz = 160;
  }

  // define div_a, div_b and div_n according to the clock frequency
  uint32_t div_a, div_b, div_n;
  div_a = 0;   // max 64
  div_b = 0;   // max 64
  div_n = 254; // max 256

  _freq_read = (clk_freq_mhz * 1000 * 1000) / div_n; // just used for logging - ignoring the div_a and div_b for now
  ESP_LOGD(MAIN_TAG, "freq_read %d Hz", _freq_read);
  ESP_LOGD(MAIN_TAG, "clock div a %d", div_a);
  ESP_LOGD(MAIN_TAG, "clock div b %d", div_b);
  ESP_LOGD(MAIN_TAG, "clock div n %d", div_n);

  // enable lcd / camera module
  if (!periph_ll_periph_enabled(PERIPH_LCD_CAM_MODULE))
  {
    periph_ll_disable_clk_set_rst(PERIPH_LCD_CAM_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_LCD_CAM_MODULE);
  }

  dev->cam_ctrl.val = 0;  // reset all values
  dev->cam_ctrl1.val = 0; // reset all values

  dev->cam_ctrl1.cam_reset = 1; // reset camera bus

  // config camera clock
  dev->cam_ctrl.cam_clk_sel = clksel;
  dev->cam_ctrl.cam_clkm_div_a = div_a;
  dev->cam_ctrl.cam_clkm_div_b = div_b;
  dev->cam_ctrl.cam_clkm_div_num = div_n;

  // 2. Configure signal pins according to Table 29-1.

  // pixel clock input (for master mode)
  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pixelClockPin], PIN_FUNC_GPIO);
  // gpio_set_direction((gpio_num_t)pixelClockPin, GPIO_MODE_INPUT);
  // gpio_set_pull_mode((gpio_num_t)pixelClockPin, GPIO_FLOATING);
  // gpio_matrix_in(pixelClockPin, CAM_PCLK_IDX, false);

  // vsync interrupt input
  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[vSyncPin], PIN_FUNC_GPIO);
  // gpio_set_direction((gpio_num_t)vSyncPin, GPIO_MODE_INPUT);
  // gpio_set_pull_mode((gpio_num_t)vSyncPin, GPIO_FLOATING);
  // gpio_matrix_in(vSyncPin, CAM_V_SYNC_IDX, false);
  // gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT, CAM_V_SYNC_IDX, false);

  // hsync enable input pin
  /*PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[hSyncEnablePin], PIN_FUNC_GPIO);
  gpio_set_direction((gpio_num_t)hSyncEnablePin, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)hSyncEnablePin, GPIO_FLOATING);
  gpio_matrix_in(hSyncEnablePin, CAM_H_ENABLE_IDX, false);*/
  gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT, CAM_H_ENABLE_IDX, false);

  // hsync interrupt input pin
  // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[hSyncPin], PIN_FUNC_GPIO);
  // gpio_set_direction((gpio_num_t)hSyncPin, GPIO_MODE_INPUT);
  // gpio_set_pull_mode((gpio_num_t)hSyncPin, GPIO_FLOATING);
  // gpio_matrix_in(hSyncPin, CAM_H_SYNC_IDX, false);
  gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT, CAM_H_SYNC_IDX, false);

  // master clock output pin
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[masterClockPin], PIN_FUNC_GPIO);
  gpio_set_direction((gpio_num_t)masterClockPin, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_pull_mode((gpio_num_t)masterClockPin, GPIO_FLOATING);
  gpio_matrix_out(masterClockPin, CAM_CLK_IDX, false, false);
  // redirect master clock to pixel clock input (O.o) - lord bless io mux
  gpio_matrix_in(masterClockPin, CAM_PCLK_IDX, false);

  // initialize input data pins
  for (int i = 0; i < 16; i++)
  {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_data[i]], PIN_FUNC_GPIO);
    gpio_set_direction((gpio_num_t)pin_data[i], GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)pin_data[i], GPIO_FLOATING);
    gpio_matrix_in(pin_data[i], CAM_DATA_IN0_IDX + i, false);
  }

  // 3. Set or clear LCD_CAM_CAM_VH_DE_MODE_EN according to the control signal HSYNC.
  //  1: Input control signals are CAM_DE and CAM_HSYNC. CAM_VSYNC is 1.
  //  0: Input control signals are CAM_DE and CAM_VSYNC. CAM_HSYNC and CAM_DE are all 1 at the the same time.

  // not relevant for us since we are not using vsync nor hsync
  dev->cam_ctrl1.cam_vh_de_mode_en = 0;

  // 4. Set needed RX channel mode and RX data mode, then set the bit LCD_CAM_CAM_UPDATE.
  //  maybe it is not needed, old reference to i2s rx mode?, doc is not clear
  dev->cam_ctrl.cam_update = 1;

  // 5. Reset RX control unit (Camera_Ctrl) and Async Rx FIFO as described in Section 29.3.4.
  dev->cam_ctrl1.cam_reset = 1;
  dev->cam_ctrl1.cam_reset = 0;

  dev->cam_ctrl1.cam_afifo_reset = 1;
  dev->cam_ctrl1.cam_afifo_reset = 0;

  // 6. Enable corresponding interrupts, see Section 29.5.
  //    LCD_CAM_CAM_HS_INT: triggered when the total number of received lines by camera is greater than or equal to LCD_CAM_CAM_LINE_INT_NUM + 1.
  //    LCD_CAM_CAM_VSYNC_INT: triggered when the camera received a VSYNC signal.

  // we are not using vsync nor line interrupt
  dev->cam_ctrl.cam_line_int_en = 0;         // disable line interrupt //LCD_CAM_CAM_HS_INT
  dev->lc_dma_int_ena.cam_vsync_int_ena = 0; // disable vsync interrupt //LCD_CAM_CAM_VSYNC_INT

  // we are not using vsync interrupt but apparently it is needed to configure it with ESP_INTR_FLAG_INTRDISABLED
  intr_handle_t cam_intr_handle;

  // Allocate LCD_CAM interrupt, keep it disabled
  esp_intr_alloc(ETS_LCD_CAM_INTR_SOURCE,
                 ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                 &vsyncHandler, NULL, &cam_intr_handle);

  if (esp_intr_enable(cam_intr_handle) != ESP_OK)
  {
    ESP_LOGE(MAIN_TAG, "Failed to disable LCD_CAM interrupt");
  }

  // 7. Configure GDMA inlink, and set the length of RX data in LCD_CAM_CAM_REC_DATA_BYTELEN.

  // allocate GDMA

  dma_buffer_size = dma_node_cnt * dma_node_buffer_size; // dma size will be number of DMA nodes * size of each node

  dma_buffer = NULL;
  dma_descriptors = NULL;

  // allocate dma buffer for all dma descriptors
  dma_buffer = (uint8_t *)heap_caps_malloc(dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
  if (NULL == dma_buffer)
  {
    ESP_LOGE(MAIN_TAG, "%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
             (int)dma_buffer_size, (int)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  }

  // clear DMA buffer for debugging / testing
  for (int i = 0; i < dma_buffer_size; i++)
  {
    dma_buffer[i] = 0;
  }

  // allocate DMA descriptors in a circular buffer
  dma_descriptors = allocate_dma_descriptors(dma_node_cnt, dma_node_buffer_size, dma_buffer);
  if (dma_descriptors == NULL)
  {
    ESP_LOGE(MAIN_TAG, "DMA descriptor malloc failed");
  }

  // alloc rx gdma channel
  gdma_channel_alloc_config_t rx_alloc_config = {
      .direction = GDMA_CHANNEL_DIRECTION_RX};
  rx_alloc_config.sibling_chan = NULL;

  gdma_channel_handle_t dma_channel_handle; // handle for the GDMA channel
  esp_err_t ret = gdma_new_channel(&rx_alloc_config, &dma_channel_handle);
  if (ret != ESP_OK)
  {
    ESP_LOGE(MAIN_TAG, "Can't find available GDMA channel");
  }

  ret = gdma_get_channel_id(dma_channel_handle, &dma_channel_id); // get the DMA channel number
  if (ret != ESP_OK)
  {
    ESP_LOGE(MAIN_TAG, "Can't get GDMA channel number");
  }
  ESP_LOGD(MAIN_TAG, "DMA Channel=%d", dma_channel_id);

  // enable GDMA module
  if (!periph_ll_periph_enabled(PERIPH_GDMA_MODULE))
  {
    periph_ll_disable_clk_set_rst(PERIPH_GDMA_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_GDMA_MODULE);
  }

  // configuring interrupt for GDMA success eof
  esp_err_t ret_gmda_intr = ESP_OK;
  intr_handle_t gdma_intr_handle;
  ret_gmda_intr = esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[dma_channel_id].rx_irq_id,
                                            ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | CAMERA_ISR_IRAM_FLAG,
                                            (uint32_t)&GDMA.channel[dma_channel_id].in.int_st, GDMA_IN_SUC_EOF_CH0_INT_ST_M,
                                            dma_in_suc_eof_isr, &dma_channel_id, &gdma_intr_handle);
  if (ret_gmda_intr != ESP_OK)
  {
    ESP_LOGE(MAIN_TAG, "DMA interrupt allocation of camera failed");
  }

  if (esp_intr_enable(gdma_intr_handle) != ESP_OK)
  {
    ESP_LOGE(MAIN_TAG, "Failed to enable GDMA interrupt");
  }

  // configure GDMA channel with the first DMA descriptor
  GDMA.channel[dma_channel_id].in.link.addr = ((uint32_t)&dma_descriptors[0]) & 0xfffff;

  // reset GDMA channel
  GDMA.channel[dma_channel_id].in.int_clr.val = ~0;
  GDMA.channel[dma_channel_id].in.int_ena.val = 0;

  GDMA.channel[dma_channel_id].in.conf0.val = 0;
  GDMA.channel[dma_channel_id].in.conf0.in_rst = 1;
  GDMA.channel[dma_channel_id].in.conf0.in_rst = 0;

  // no need to use burst mode until now
  GDMA.channel[dma_channel_id].in.conf0.indscr_burst_en = 0;
  GDMA.channel[dma_channel_id].in.conf0.in_data_burst_en = 0;

  GDMA.channel[dma_channel_id].in.conf1.in_check_owner = 0; // disabling check owner

  GDMA.channel[dma_channel_id].in.peri_sel.sel = 5; // peripheral number 5 is LCD_CAM

  GDMA.channel[dma_channel_id].in.link.start = 1; // start GDMA channel

  dev->cam_ctrl.cam_stop_en = 0;    // do not stop camera when GDMA is full
  dev->cam_ctrl.cam_bit_order = 1;  // both flags makes input 0 to map bit 0, input 1 map bit 1, from left to right
  dev->cam_ctrl.cam_byte_order = 0; // reading the buffer as unsigned word - remember ESP32 is little-endian chip
  dev->cam_ctrl.cam_vs_eof_en = 0;  // 1: CAM_VSYNC to generate in_suc_eof. 0: in_suc_eof is controlled by cam_rec_data_bytelen  - we are using in_suc_eof

  // this is important - the number of bytes to read before generating the GMDA in_suc_eof interrupt
  // cam_rec_data_bytelen: Configure camera received data byte length. When the length of received data reaches this value + 1, GDMA in_suc_eof_int is triggered.
  dev->cam_ctrl1.cam_rec_data_bytelen = dma_node_buffer_size - 1;

  dev->cam_ctrl1.cam_line_int_num = 0;    // The number of hsyncs that generate hs interrupts
  dev->cam_ctrl1.cam_clk_inv = 0;         // do not invert master clock
  dev->cam_ctrl1.cam_vsync_filter_en = 0; // 1: enable vsync filter | 0: disable - disabling since we are not using vsync interrupt
  dev->cam_ctrl1.cam_2byte_en = 1;        // 1: 16bits input | 0: 8 bits input
  dev->cam_ctrl1.cam_de_inv = 0;          // irrelevant for us, we are not using de signal
  dev->cam_ctrl1.cam_hsync_inv = 0;       // irrelevant for us, we are not using hsync signal
  dev->cam_ctrl1.cam_vsync_inv = 0;       // irrelevant for us, we are not using vsync signal

  // make sure we are not using image converter
  dev->cam_rgb_yuv.val = 0;
  dev->cam_rgb_yuv.cam_conv_bypass = 0; // bypass converter // doesnt need this because of the above line

  // 8. Start receiving data:
  //   In master mode, when the slave is ready, set LCD_CAM_CAM_START to start receiving data.
  //   In slave mode, set LCD_CAM_CAM_START. Receiving data starts after the master provides clock signal and control signal.

  // start camera
  dev->cam_ctrl1.cam_start = 0;

  GDMA.channel[dma_channel_id].in.int_clr.in_suc_eof = 1;
  GDMA.channel[dma_channel_id].in.int_ena.in_suc_eof = 1; // enable interrupt on success eof for GMDA

  dev->cam_ctrl1.cam_reset = 1;
  dev->cam_ctrl1.cam_reset = 0;
  dev->cam_ctrl1.cam_afifo_reset = 1;
  dev->cam_ctrl1.cam_afifo_reset = 0;

  // 9. Receive data and store the data to the specified address of ESP32-S3 memory. Then corresponding interrupts set in Step 6 will be generated.

  dev->cam_ctrl.cam_update = 1;
  dev->cam_ctrl1.cam_start = 1;

  // Notes:
  //   No matter in which operation mode, camera master RX mode or camera slave RX mode, the rules below must be followed when accessing internal memory via GDMA:
  //
  // – If 8-bit parallel data input mode is selected, then
  //  * pixel clock frequency must be less than 80 MHz.
  //  * if YUV-RGB format conversion is used meanwhile, the pixel clock frequency must be less than 60 MHz.
  //
  // – If 16-bit parallel data input mode is selected, then
  //  * pixel clock frequency must be less than 40 MHz.
  //  * if YUV-RGB format conversion is used meanwhile, the pixel clock frequency must be less than 30 MHz.
  //
  // • If an external camera and an external LCD are connected simultaneously, ensure that the maximum data
  // throughput on the interface is less than GDMA total data bandwidth of 80 MB/s. Note the default
  // frequency of APB_CLK is 80 MHz here. For more information, see Chapter 7 Reset and Clock.

  // workaround to start the camera reading, eventhough we are not using the vsync interrupt ¯\_(ツ)_/¯
  esp_rom_delay_us(100);
  gpio_matrix_in(GPIO_MATRIX_CONST_ZERO_INPUT, CAM_V_SYNC_IDX, false);
  esp_rom_delay_us(100);
  gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT, CAM_V_SYNC_IDX, false);

  ESP_LOGI(MAIN_TAG, "16bit parallel read started at %d Hz", _freq_read);

  // loop for testing / dubugging / understanding the code - you can remove this
  while (1)
  {
    // print the last word read from DMA interrupt if it is different from the last one
    if (word_to_send != last_word_to_send)
    {
      // log the word read and its descriptor index
      ESP_LOGI(MAIN_TAG, "word to send: %d  dma_descriptor %d:%p", word_to_send, dma_descriptor_idx, dma_eof_address);
      last_word_to_send = word_to_send;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}