#include "esp_camera.h"
#include "esp_timer.h"
#include "Arduino.h"
#include "img_converters.h"
#include "image_util.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

#include "TFTRelated.h"


/* Modify the following line according to your project name
   Do not forget to import the library using "Sketch">"Include Library">"Add .ZIP Library..."
*/
#include <Car_Detection_inferencing.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

int pictureNumber = 0;

int64_t time_start_here;
int64_t time_end_here;

camera_fb_t * fb = NULL;

dl_matrix3du_t *resized_matrix = NULL;
dl_matrix3du_t *rgb888_matrix = NULL;
uint16_t *rgb565 = (uint16_t *) malloc(120 * 120);

size_t out_len = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
ei_impulse_result_t result = {0};
String label = "uncertain";
bool sd_card = true;



void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);

  camera_init();

  resized_matrix = dl_matrix3du_alloc(1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
  rgb888_matrix = dl_matrix3du_alloc(1, 240, 240, 3);
}

void camera_init()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 10; //设置为10可以避免fmt2rgb888解码出错的问题
  config.fb_count = 1;

  // Init Camera
  esp_err_t err = esp_camera_init(&config);

   if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void take_picture()
{
  // --- 拍照 ---

  time_start_here = esp_timer_get_time();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  time_end_here = esp_timer_get_time();
  Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb->len), (uint32_t)((time_end_here - time_start_here) / 1000));

  // --- 保存原始照片，不影响接下来的功能 ---
  saveOriginalPicture();

  // --- 将图片转成RGB888并缩小，然后提供给模型使用 ---
  convertPicture2RGB888NResize();

  // --- Call classifier function ---

  classify();

  // --- Convert back the resized RGB888 frame to JPG to save on SD card ---
/*
  Serial.println("Converting resized RGB888 frame to JPG...");
  time_start_here = esp_timer_get_time();
  uint8_t * _jpg_buf = NULL;
  fmt2jpg(resized_matrix->item, out_len, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, PIXFORMAT_RGB888, 10, &_jpg_buf, &out_len);
  time_end_here = esp_timer_get_time();
  Serial.printf("Done in %ums\n", (uint32_t)((time_end_here - time_start_here) / 1000));

  uint8_t *rgb565 = (uint8_t *) malloc(240 * 240 * 3);
  jpg2rgb565(fb->buf, fb->len, rgb565, JPG_SCALE_2X); // scale to half size
*/
  // --- Free memory ---
//  dl_matrix3du_free(resized_matrix);


  //--- Save resized Picture ---
/*
  if (sd_card) {
    // Path where new picture will be saved in SD Card
    String path = "/" + String(label) + ".predicted_" + String(pictureNumber) + ".jpg";
    label = "uncertain";

    fs::FS &fs = SD_MMC;
    Serial.printf("Picture file name: %s\n", path.c_str());

    File file = fs.open(path.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(_jpg_buf, out_len); // payload (image), payload length
      Serial.printf("Saved file to path: %s\n", path.c_str());
    }
    file.close();
  }

  // --- Free memory ---
  free(_jpg_buf);
  _jpg_buf = NULL;
*/

  // --- Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4 ---
//  pinMode(4, OUTPUT);
//  digitalWrite(4, LOW);
//  rtc_gpio_hold_en(GPIO_NUM_4);
//  delay(2000);

  
  // --- Deep sleep ---
//  Serial.println("Going to sleep now");
  delay(2000);
//  esp_deep_sleep_start();
//  Serial.println("This will never be printed");
}

//将拍下来的文件保存下来
void saveOriginalPicture()
{
  //Serial.println("Starting SD Card");
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    sd_card = false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    sd_card = false;
  }
  
  if (sd_card) {
    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);
    pictureNumber = EEPROM.read(0) + 1;

    // Path where new picture will be saved in SD Card
    String path = "/original_" + String(pictureNumber) + ".jpg";

    fs::FS &fs = SD_MMC;
    Serial.printf("Picture file name: %s\n", path.c_str());

    File file = fs.open(path.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.printf("Saved file to path: %s\n", path.c_str());
      EEPROM.write(0, pictureNumber);
      EEPROM.commit();
    }
    file.close();
  }

  SD_MMC.end();
}

//这个方法主要是将图片转成RGB888，并缩小图片，等待模型来取缩小后的图片
void convertPicture2RGB888NResize()
{
  bool s;

  // --- Convert frame to RGB888  ---
  Serial.println("Converting to RGB888...");
  time_start_here = esp_timer_get_time();
  Serial.println("get start time");
  // Allocate rgb888_matrix buffer
  //dl_matrix3du_t *rgb888_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  //Serial.println("allocate memory");
  s = fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_matrix->item);
  Serial.println("converted");
  time_end_here = esp_timer_get_time();
  Serial.println("get end time");
  Serial.printf("Done in %ums\n", (uint32_t)((time_end_here - time_start_here) / 1000));

  // --- Resize the RGB888 frame to 96x96 in this example ---

  Serial.println("Resizing the frame buffer...");
  time_start_here = esp_timer_get_time();
  //resized_matrix = dl_matrix3du_alloc(1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
  image_resize_linear(resized_matrix->item, rgb888_matrix->item, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3, fb->width, fb->height);
  time_end_here = esp_timer_get_time();
  Serial.printf("Done in %ums\n", (uint32_t)((time_end_here - time_start_here) / 1000));

  dl_matrix3du_t *scaled_matrix = dl_matrix3du_alloc(1, 120, 120, 3);
  image_zoom_in_twice(scaled_matrix->item, 120, 120, 3, rgb888_matrix->item, 240, 3);

  
  //image_rgb888_to_565(rgb565, scaled_matrix->item, 120 * 120);
  //uint16_t *rgb565 = 
  raw_rgb565_get_data(0, scaled_matrix);
  
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("start draw");
  tft.drawRGBBitmap(3, 0, rgb565, 120, 120);
  Serial.println("draw done");

//  Serial.println("free");
  // --- Free memory ---
  dl_matrix3du_free(scaled_matrix);
//  dl_matrix3du_free(rgb888_matrix);
  esp_camera_fb_return(fb);


  //free(rgb565);
}

int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr)
{
  size_t pixel_ix = offset * 3;
  size_t bytes_left = out_len;
  size_t out_ptr_ix = 0;

  // read byte for byte
  while (bytes_left != 0) {
    // grab the values and convert to r/g/b
    uint8_t r, g, b;
    r = resized_matrix->item[pixel_ix];
    g = resized_matrix->item[pixel_ix + 1];
    b = resized_matrix->item[pixel_ix + 2];

    // then convert to out_ptr format
    float pixel_f = (r << 16) + (g << 8) + b;
    signal_ptr[out_ptr_ix] = pixel_f;

    // and go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    bytes_left--;
  }
  return 0;
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  Serial.println("In tft_output");
  // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;
  Serial.println("In tft_output go on");
  // This function will clip the image block rendering automatically at the TFT boundaries
  //tft.pushImage(x, y, w, h, bitmap);

  // This might work instead if you adapt the sketch to use the Adafruit_GFX library
   tft.drawRGBBitmap(x, y, bitmap, w, h);

  // Return 1 to decode next block
  return 1;
}

void classify()
{
  Serial.println("Getting signal...");
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_WIDTH;
  signal.get_data = &raw_feature_get_data;

  Serial.println("Run classifier...");
  // Feed signal to the classifier

  // Run the classifier
  ei_impulse_result_t result = { 0 };
  
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }

        ei_printf("    %s (", bb.label);
        ei_printf_float(bb.value);
        ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
    }

    if (!bb_found) {
        ei_printf("    No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");
        if(result.classification[ix].value > 0.8){
          label = String(result.classification[ix].label);
        }
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif
#endif
}

void loop() {
  take_picture();

  delay(5000);
}


uint16_t RGB888_TO_RGB565(uint32_t rgb888){    
  uint8_t r,g,b;    
  uint16_t rgb565=0;    
  r=rgb888>>16;    
  g=rgb888>>8;    
  b=rgb888>>0;    
  r>>=3; //分量5    
  g>>=2; //分量6    
  b>>=3; //分量5    
  rgb565=r<<11|g<<5|b<<0;    
  return rgb565;
}

int raw_rgb565_get_data(size_t offset, dl_matrix3du_t *rgb888_matrix)
{
  size_t pixel_ix = offset * 3;
  size_t bytes_left = 120 * 120;
  size_t out_ptr_ix = 0;

  // read byte for byte
  while (bytes_left != 0) {
    // grab the values and convert to r/g/b
    uint8_t r, g, b;
    r = rgb888_matrix->item[pixel_ix];
    g = rgb888_matrix->item[pixel_ix + 1];
    b = rgb888_matrix->item[pixel_ix + 2];

//    r>>=3; //分量5    
//    g>>=2; //分量6    
//    b>>=3; //分量5    

    // then convert to out_ptr format
    //rgb565[out_ptr_ix] = r<<11|g<<5|b<<0;

    rgb565[out_ptr_ix] = (((r & 0xf8) | (g >>5 )) << 8) | (((g << 3) & 0xE0) |( b >> 3));

    // and go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    bytes_left--;
//    Serial.print(out_ptr_ix);
//    Serial.print(":");
//    Serial.println(bytes_left);
  }

  Serial.println("to rgb565 done");
  
  return 0;
}
