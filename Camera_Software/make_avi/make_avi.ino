/*

  TimeLapseAvi

  ESP32-CAM Video Recorder

  This program records an AVI video on the SD Card of an ESP32-CAM.
  
  by James Zahary July 20, 2019
     jamzah.plc@gmail.com

  https://github.com/jameszah/ESP32-CAM-Video-Recorder
    jameszah/ESP32-CAM-Video-Recorder is licensed under the
    GNU General Public License v3.0

  Acknowlegements:

  1.  https://robotzero.one/time-lapse-esp32-cameras/
      Timelapse programs for ESP32-CAM version that sends snapshots of screen.
  2.  https://github.com/nailbuster/esp8266FTPServer
      ftp server (slightly modifed to get the directory function working)
  3.  https://github.com/ArduCAM/Arduino/tree/master/ArduCAM/examples/mini
      ArduCAM Mini demo (C)2017 LeeWeb: http://www.ArduCAM.com
      I copied the structure of the avi file, some calculations.

  The is Arduino code, with standard setup for ESP32-CAM
    - Board ESP32 Wrover Module
    - Partition Scheme Huge APP (3MB No OTA)
    
  This program records an AVI video on the SD Card of an ESP32-CAM.
  
  It will record realtime video at limited framerates, or timelapses with the full resolution of the ESP32-CAM.
  It is controlled by a web page it serves to stop and start recordings with many parameters, and look through the viewfinder.
  
  You can control framesize (UXGA, VGA, ...), quality, length, and fps to record, and fps to playback later, etc.

  framesize can be UXGA, SVGA, VGA, CIF (default VGA)
  length is length in seconds of the recording 0..3600 (default 1800)
  interval is the milli-seconds between frames (default 200)
  quality is a number 5..50 for the jpeg  - smaller number is higher quality with bigger and more detailed jpeg (default 10)
  repeat is a number of who many of the same recordings should be made (default 100)
  speed is a factor to speed up realtime for a timelapse recording - 1 is realtime (default 1)
  gray is 1 for a grayscale video (default 0 - color)

  These factors have to be within the limit of the SD chip to receive the data.
  For example, using a LEXAR 300x 32GB microSDHC UHS-I, the following works for me:

  UXGA quality 10,  2 fps (or interval of 500ms)
  SVGA quality 10,  5 fps (200ms)
  VGA  quality 10, 10 fps (100ms)
  CIG  quality 10, 20 fps (50ms)

  If you increase fps, you might have to reduce quality or framesize to keep it from dropping frames as it writes all the data to the SD chip.

  Also, other SD chips will be faster or slower.  I was using a SanDisk 16GB microSDHC "Up to 653X" - which was slower and more unpredictable than the LEXAR ???

  Search for "zzz" to find places to modify the code for:
    1.  Your wifi name and password
    2.  Your preferred ip address (with default gateway, etc)
    3.  Your Timezone for use in filenames
    4.  Defaults for framesize, quality, ... and if the recording should start on reboot of the ESP32 without receiving a command
  
*/



#include "esp_camera.h"

// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "SD_MMC.h"
#include "esp_vfs_fat.h"

//#define LED_BUILTIN 4 //define for flash light

long current_millis;
long last_capture_millis = 0;
static esp_err_t cam_err;
static esp_err_t card_err;
char strftime_buf[64];
int file_number = 0;
bool internet_connected = false;
struct tm timeinfo;
time_t now;

char *filename ;
char *stream ;
int newfile = 0;
int frames_so_far = 0;
FILE *myfile;
long bp;
long ap;
long bw;
long aw;
long totalp;
long totalw;
float avgp;
float avgw;
int overtime_count = 0;

// CAMERA_MODEL_AI_THINKER
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


// GLOBALS
#define BUFFSIZE 512



// these are just declarations -- look below to edit defaults 

int capture_interval = 200; // microseconds between captures
int total_frames = 300;     // default updated below
int recording = 1;          // default start recording on reboot
int framesize = 6;          // vga
int repeat = 100;           // capture 100 videos
int quality = 10;
int xspeed = 1;
int xlength = 3;
int gray = 0;


#define AVIOFFSET 240 // AVI main header length

unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
unsigned long idx_offset = 0;

uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t   dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"

uint8_t  vga_w[2] = {0x80, 0x02}; // 640
uint8_t  vga_h[2] = {0xE0, 0x01}; // 480
uint8_t  cif_w[2] = {0x90, 0x01}; // 400
uint8_t  cif_h[2] = {0x28, 0x01}; // 296
uint8_t svga_w[2] = {0x20, 0x03}; //
uint8_t svga_h[2] = {0x58, 0x02}; //
uint8_t uxga_w[2] = {0x40, 0x06}; // 1600
uint8_t uxga_h[2] = {0xB0, 0x04}; // 1200


const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x67,
  0x6D, 0x61, 0x69, 0x6C, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};


//
// Writes an uint32_t in Big Endian at current file position
//
static void inline print_quartet(unsigned long i, FILE * fd)
{
  uint8_t x[1];

  x[0] = i % 0x100;
  size_t i1_err = fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  size_t i2_err = fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  size_t i3_err = fwrite(x , 1, 1, fd);
  i = i >> 8;  x[0] = i % 0x100;
  size_t i4_err = fwrite(x , 1, 1, fd);
}

char the_page[3000];

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


void setup() {


  Serial.begin(115200);

  Serial.setDebugOutput(true);
  


  pinMode(33, OUTPUT);    // little red led on back of chip
  //pinMode (LED_BUILTIN, OUTPUT); // flash light

  digitalWrite(33, LOW);           // turn on the red LED on the back of chip

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

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 1;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;         // svga 12 fails due to jpg 60000
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  cam_err = esp_camera_init(&config);
  if (cam_err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", cam_err);
    major_fail();
    return;
  }

  // SD camera init
  card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    major_fail();
    return;
  }

  // 200 ms x 150 frames = 30 seconds     is 3 MB indoors
  // 200 ms x 300 frames = 1 minute       is about 6MD indoor
  // 20 ms x 3000 frames = 10 minute      is 60 MB indoor
  // burst 1000 frames gives 8 fps rather than 5, so 2 minutues 20 MB indoor

  digitalWrite(33, HIGH);


  //
  //  startup defaults  -- EDIT HERE 
  //  zzz

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 10);
  do_fb();                  // do a couple captures to make sure camera has new config
  do_fb();

  framesize = 6;            // vga
  repeat = 0;             // video + 1
  xspeed = 1;               // playback at 1 x realtime
  gray = 0;                 // not gray

  quality = 10;             // quality 10 - pretty good.  Goes from 0..63, but 0-5 sometimes fails on bright scenery (jpg too big for ESP32CAM system)
  capture_interval = 66;   //  milli-secconds per frame
  total_frames = 937;       // total_frames x capture_interval = record_time
  
  xlength = total_frames * capture_interval / 1000;

  newfile = 0;    // no file is open  // don't fiddle with this!
  recording = 1;  // start recording on reboot without sending a command

}

//
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//
void major_fail() {
  while (1) {
    digitalWrite(33, LOW);
    delay(100);
    digitalWrite(33, HIGH);
    delay(100);

    digitalWrite(33, LOW);
    delay(100);
    digitalWrite(33, HIGH);
    delay(100);

    digitalWrite(33, LOW);
    delay(100);
    digitalWrite(33, HIGH);
    //delay(100);

    delay(1000);

    digitalWrite(33, LOW);
    delay(500);
    digitalWrite(33, HIGH);
    delay(500);

    digitalWrite(33, LOW);
    delay(500);
    digitalWrite(33, HIGH);
    delay(500);

    digitalWrite(33, LOW);
    delay(500);
    digitalWrite(33, HIGH);
    //delay(500);

    delay(1000);
  }
}

static esp_err_t init_sdcard()
{
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 10,
  };
  sdmmc_card_t *card;

  Serial.println("Mounting SD card...");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    Serial.println("SD card mount successfully!");
  }  else  {
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
    major_fail();
  }

  Serial.print("SD_MMC Begin: "); Serial.println(SD_MMC.begin());   // required by ftp system ??
}

//////////////////////////////////////////////////////////////
// save photo stuff not currently used

static esp_err_t save_photo_numbered()
{
  file_number++;
  Serial.print("Taking picture: ");
  Serial.print(file_number);
  camera_fb_t *fb = esp_camera_fb_get();

  char *filename = (char*)malloc(21 + sizeof(int));
  sprintf(filename, "/sdcard/capture_%d.jpg", file_number);

  Serial.println(filename);
  FILE *file = fopen(filename, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    Serial.printf("File saved: %s\n", filename);
  }  else  {
    Serial.println("Could not open file");
  }
  fclose(file);
  esp_camera_fb_return(fb);
  free(filename);
}



static esp_err_t save_photo_dated()
{
  Serial.println("Taking a picture...");
  camera_fb_t *fb = esp_camera_fb_get();

  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%F__%H%M%S", &timeinfo);

  char fname[100];

  if (framesize == 6) {
    sprintf(fname, "/sdcard/%s_vga_%d.jpg", strftime_buf, quality);
  } else if (framesize == 7) {
    sprintf(fname, "/sdcard/%s_svga_%d.jpg", strftime_buf, quality);
  } else if (framesize == 10) {
    sprintf(fname, "/sdcard/%s_uxga_%d.jpg", strftime_buf, quality);
  } else  if (framesize == 5) {
    sprintf(fname, "/sdcard/%s_cif_%d.jpg", strftime_buf, quality);
  } else {
    Serial.println("Wrong framesize");
    sprintf(fname, "/sdcard/%s_xxx_%d.jpg", strftime_buf, quality);
  }

  FILE *file = fopen(fname, "w");
  if (file != NULL)  {
    size_t err = fwrite(fb->buf, 1, fb->len, file);
    Serial.printf("File saved: %s\n", fname);

  }  else  {
    Serial.println("Could not open file");
  }
  fclose(file);
  esp_camera_fb_return(fb);

}

void save_photo()
{
  if (timeinfo.tm_year < (2016 - 1900) || internet_connected == false) { // if no internet or time not set
    save_photo_numbered(); // filenames in numbered order
  } else {
    save_photo_dated(); // filenames with date and time
  }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Make the avi move in 4 pieces
//
// make_avi() called in every loop, which calls below, depending on conditions
//   start_avi() - open the file and write headers
//   another_pic_avi() - write one more frame of movie
//   end_avi() - write the final parameters and close the file

// global variable used by these pieces

char str[20];
uint16_t n;
uint8_t buf[BUFFSIZE];

static int i = 0;
uint8_t temp = 0, temp_last = 0;
unsigned long fileposition = 0;
uint16_t frame_cnt = 0;
uint16_t remnant = 0;
uint32_t length = 0;
uint32_t startms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;
bool is_header = false;
long bigdelta = 0;

camera_fb_t * fb = NULL;

FILE *avifile = NULL;
FILE *idxfile = NULL;

void make_avi( ) {

  // we are recording, but no file is open

  if (newfile == 0 && recording == 1) {                                     // open the file

    //save_photo_dated();

    start_avi();

    digitalWrite(33, HIGH);
    newfile = 1;
    totalp = 0;
    totalw = 0;
    frame_cnt = 0;
    frames_so_far = 0;

  } else {

    // we have a file open, but not recording

    if (newfile == 1 && recording == 0) {                                  // got command to close file
      digitalWrite(33, LOW);

      end_avi();

      Serial.println("Done capture due to command");

      frames_so_far = total_frames;

      newfile = 0;    // file is closed
      recording = 0;  // DO NOT start another recording

    } else {

      if (newfile == 1 && recording == 1) {                            // regular recording

        if (frames_so_far == total_frames)  {                                // we are done the recording

          Serial.println("Done capture for total frames!");

          digitalWrite(33, LOW);                                                       // close the file
          end_avi();

          frames_so_far = 0;
          newfile = 0;          // file is closed

          if (repeat > 0) {
            recording = 1;        // start another recording
            repeat = repeat - 1;
          } else {
            recording = 0;
          }
          
        } else if ((millis() - startms) > (total_frames * capture_interval)) {

          Serial.println (" "); Serial.println("Done capture for time");
          Serial.print("Time Elapsed: "); Serial.print(millis() - startms); Serial.print(" Frames: "); Serial.println(frame_cnt);
          Serial.print("Config:       "); Serial.print(total_frames * capture_interval ) ; Serial.print(" (");
          Serial.print(total_frames); Serial.print(" x "); Serial.print(capture_interval);  Serial.println(")");

          digitalWrite(33, LOW);                                                       // close the file

          end_avi();

          frames_so_far = 0;
          newfile = 0;          // file is closed
          if (repeat > 0) {
            recording = 1;        // start another recording
            repeat = repeat - 1;
          } else {
            recording = 0;
          }

        } else  {                                                            // regular

          current_millis = millis();

          if (current_millis - last_capture_millis > capture_interval) {                       // Take another picture - fixed interval

            //if ((current_millis - startms) > (frame_cnt * capture_interval)) {                   // Take another picture - with catch up

            last_capture_millis = millis();

            frames_so_far = frames_so_far + 1;
            frame_cnt++;

            another_pic_avi();
          }
        }
      }
    }
  }
}

static esp_err_t start_avi() {

  //Serial.println("Starting an avi ");

  do_fb();  // start the camera ... warm it up
  delay(1000);
  do_fb();
  delay(1000);
  do_fb();

  time(&now);
  localtime_r(&now, &timeinfo);

  strftime(strftime_buf, sizeof(strftime_buf), "%F__%H-%M-%S", &timeinfo);

  char fname[100];

  if (framesize == 6) {
    sprintf(fname, "/sdcard/%s_vga_Q%d_I%d_L%d_S%d.avi", strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 7) {
    sprintf(fname, "/sdcard/%s_svga_Q%d_I%d_L%d_S%d.avi", strftime_buf, quality, capture_interval, xlength, xspeed);
  } else if (framesize == 10) {
    sprintf(fname, "/sdcard/%s_uxga_Q%d_I%d_L%d_S%d.avi", strftime_buf, quality, capture_interval, xlength, xspeed);
  } else  if (framesize == 5) {
    sprintf(fname, "/sdcard/%s_cif_Q%d_I%d_L%d_S%d.avi", strftime_buf, quality, capture_interval, xlength, xspeed);
  } else {
    Serial.println("Wrong framesize");
    sprintf(fname, "/sdcard/%s_xxx_Q%d_I%d_L%d_S%d.avi", strftime_buf, quality, capture_interval, xlength, xspeed);
  }

  Serial.print("\nFile name will be >"); Serial.print(fname); Serial.println("<");

  avifile = fopen(fname, "w");
  idxfile = fopen("/sdcard/idx.tmp", "w");

  if (avifile != NULL)  {

    //Serial.printf("File open: %s\n", fname);

  }  else  {
    Serial.println("Could not open file");
    major_fail();
  }

  if (idxfile != NULL)  {

    //Serial.printf("File open: %s\n", "/sdcard/idx.tmp");

  }  else  {
    Serial.println("Could not open file");
    major_fail();
  }

  // -- open large file and initialize  -- doesn't help much

  //fseek(avifile, 1 * 1024 * 1024, SEEK_SET);              // start with 1 MB file
  //size_t ze_err = fwrite(zero_buf, 1, 4, avifile);
  //fseek(avifile, 0, SEEK_SET);


  for ( i = 0; i < AVIOFFSET; i++)
  {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }

  size_t err = fwrite(buf, 1, AVIOFFSET, avifile);

  if (framesize == 6) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(vga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(vga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(vga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(vga_h, 1, 2, avifile);

  } else if (framesize == 10) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(uxga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(uxga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(uxga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(uxga_h, 1, 2, avifile);

  } else if (framesize == 7) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(svga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(svga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(svga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(svga_h, 1, 2, avifile);

  }  else if (framesize == 5) {

    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(cif_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(cif_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(cif_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(cif_h, 1, 2, avifile);
  }

  fseek(avifile, AVIOFFSET, SEEK_SET);

  Serial.print(F("\nRecording "));
  Serial.print(total_frames);
  Serial.println(F(" video frames ...\n"));

  startms = millis();
  bigdelta = millis();
  totalp = 0;
  totalw = 0;
  overtime_count = 0;
  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;

} // end of start avi


static esp_err_t another_pic_avi() {

  digitalWrite(33, LOW);

  bp = millis();
  fb = esp_camera_fb_get();
  totalp = totalp + millis() - bp;

  jpeg_size = fb->len;
  movi_size += jpeg_size; // Update totals
  uVideoLen += jpeg_size;   

  size_t dc_err = fwrite(dc_buf, 1, 4, avifile);
  size_t ze_err = fwrite(zero_buf, 1, 4, avifile);

  bw = millis();
  size_t err = fwrite(fb->buf, 1, fb->len, avifile);

  totalw = totalw + millis() - bw;

  digitalWrite(33, HIGH);  // red led is on durng the photo and the write

  if (millis() - bigdelta > capture_interval * 1.25  ) {        // how many are 25 % overtime
    overtime_count = overtime_count + 1;
    //Serial.print(millis()-bigdelta);Serial.print(",");
  }

  if (millis() - bigdelta > capture_interval * 2 ) {           // if we are 100% overtime

    //if (frame_cnt % 10 == 1) {

    Serial.print("Frame: "); Serial.print(frame_cnt);
    Serial.print(" size ");  Serial.print(err);
    Serial.print(" delta "); Serial.print(millis() - bigdelta);
    Serial.print(" > "); Serial.print(capture_interval);

    Serial.print(" avg pic "); Serial.print( totalp / frame_cnt );
    Serial.print(" avg wrt "); Serial.print( totalw / frame_cnt );

    Serial.print(" overtime "); Serial.print( overtime_count ); Serial.print(" "); Serial.print( 100.0 * overtime_count / frame_cnt, 1 ); Serial.println(" %");

  }
  bigdelta = millis();

  remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

  print_quartet(idx_offset, idxfile);
  print_quartet(jpeg_size, idxfile);

  idx_offset = idx_offset + jpeg_size + remnant + 8;

  jpeg_size = jpeg_size + remnant;
  movi_size = movi_size + remnant;
  if (remnant > 0) {
    size_t rem_err = fwrite(zero_buf, 1, remnant, avifile);
  }
  esp_camera_fb_return(fb);

  fileposition = ftell (avifile);       // Here, we are at end of chunk (after padding)
  fseek(avifile, fileposition - jpeg_size - 4, SEEK_SET);    // Here we are the the 4-bytes blank placeholder

  print_quartet(jpeg_size, avifile);    // Overwrite placeholder with actual frame size (without padding)

  fileposition = ftell (avifile);

  fseek(avifile, fileposition + 6, SEEK_SET);    // Here is the FOURCC "JFIF" (JPEG header)
  // Overwrite "JFIF" (still images) with more appropriate "AVI1"

  size_t av_err = fwrite(avi1_buf, 1, 4, avifile);

  fileposition = ftell (avifile);
  fseek(avifile, fileposition + jpeg_size - 10 , SEEK_SET);


} // end of another_pic_avi

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static esp_err_t end_avi() {

  unsigned long current_end = 0;

  current_end = ftell (avifile);

  Serial.println("End of avi - closing the files");

  elapsedms = millis() - startms;
  float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms) * xspeed;
  float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
  uint8_t iAttainedFPS = round(fRealFPS);
  uint32_t us_per_frame = round(fmicroseconds_per_frame);


  //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

  fseek(avifile, 4 , SEEK_SET);
  print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

  fseek(avifile, 0x20 , SEEK_SET);
  print_quartet(us_per_frame, avifile);

  unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt; 

  fseek(avifile, 0x24 , SEEK_SET);
  print_quartet(max_bytes_per_sec, avifile);
  
  fseek(avifile, 0x30 , SEEK_SET);
  print_quartet(frame_cnt, avifile);

  fseek(avifile, 0x84 , SEEK_SET); 
  print_quartet((int)iAttainedFPS, avifile);

  fseek(avifile, 0xe8 , SEEK_SET);
  
  print_quartet(movi_size + frame_cnt * 8 + 4, avifile); 

  Serial.println(F("\n*** Video recorded and saved ***\n"));
  Serial.print(F("Recorded "));
  Serial.print(elapsedms / 1000);
  Serial.print(F("s in "));
  Serial.print(frame_cnt);
  Serial.print(F(" frames\nFile size is "));
  Serial.print(movi_size + 12 * frame_cnt + 4);
  Serial.print(F(" bytes\nActual FPS is "));
  Serial.print(fRealFPS, 2);
  Serial.print(F("\nMax data rate is "));
  Serial.print(max_bytes_per_sec);
  Serial.print(F(" byte/s\nFrame duration is "));  Serial.print(us_per_frame);  Serial.println(F(" us"));
  Serial.print(F("Average frame length is "));  Serial.print(uVideoLen / frame_cnt);  Serial.println(F(" bytes"));
  Serial.print("Average picture time (ms) "); Serial.println( totalp / frame_cnt );
  Serial.print("Average write time (ms) "); Serial.println( totalw / frame_cnt );
  Serial.print("Frames 25%+ late % ");  Serial.println( 100.0 * overtime_count / frame_cnt, 1 );

  Serial.println("Writing the index");

  fseek(avifile, current_end, SEEK_SET);

  fclose(idxfile);

  size_t i1_err = fwrite(idx1_buf, 1, 4, avifile);

  print_quartet(frame_cnt * 16, avifile);

  idxfile = fopen("/sdcard/idx.tmp", "r");

  if (idxfile != NULL)  {

    //Serial.printf("File open: %s\n", "/sdcard/idx.tmp");

  }  else  {
    Serial.println("Could not open file");
    //major_fail();
  }

  char * AteBytes;
  AteBytes = (char*) malloc (8);

  for (int i = 0; i < frame_cnt; i++) {

    size_t res = fread ( AteBytes, 1, 8, idxfile);
    size_t i1_err = fwrite(dc_buf, 1, 4, avifile);
    size_t i2_err = fwrite(zero_buf, 1, 4, avifile);
    size_t i3_err = fwrite(AteBytes, 1, 8, avifile);

  }

  free(AteBytes);

  fclose(idxfile);
  fclose(avifile);

  Serial.println("---");
//  WiFi.printDiag(Serial);
  Serial.println("---");
  
}

//~~~~~~~~~~~~~~~~~~
static esp_err_t do_fb() {

  camera_fb_t * fb = esp_camera_fb_get();

  Serial.print("Pic, len="); Serial.println(fb->len);

  esp_camera_fb_return(fb);
}

////////////////////////////////////////////////////////////////////////////////////
//
// some globals for the loop()
//

long wakeup;
long last_wakeup = 0;


void loop()
{
  //digitalWrite(LED_BUILTIN, LOW); //flash light
  make_avi();

}