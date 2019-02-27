/**
 * Based on: 
 * - https://github.com/wilhelmzeuschner/arduino_thermal_camera_with_sd_and_img_processing.
 * - https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example
 * 
 * Hardware:
 * - ESP32: https://www.espressif.com/en/products/hardware/esp32-devkitc/overview
 * - Sensor: https://shop.pimoroni.com/products/mlx90640-thermal-camera-breakout
 * - Display: https://www.amazon.de/gp/product/B07DPMV34R/, https://www.pjrc.com/store/display_ili9341.html
 * 
 * Needs an ESP32 core fork:
 * - https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example/issues/2
 * - https://github.com/stickbreaker/arduino-esp32
 */


/*
 *  Changes:
 *  no GCC warnings
 *  only one float digit/Nachkommastelle
 *  show degree ° character
 *  avoid constants like 768, 32 x 24
 *  plausibility check [PLAUSI_MIN,PLAUSI_MAX] to avoid strange printing on TFT
 *
 *  TODO:
 *  check why sometimes not plausible values
 *
 */


 
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"


#define C_BLUE Display.color565(0,0,255)
#define C_RED Display.color565(255,0,0)
#define C_GREEN Display.color565(0,255,0)
#define C_WHITE Display.color565(255,255,255)
#define C_BLACK Display.color565(0,0,0)
#define C_LTGREY Display.color565(200,200,200)
#define C_DKGREY Display.color565(80,80,80)
#define C_GREY Display.color565(127,127,127)

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air
paramsMLX90640 mlx90640;

#define PLAUSI_MAX_TEMP (+300.0)
#define PLAUSI_MIN_TEMP ( -40.0)

TFT_eSPI Display = TFT_eSPI();


// Added for measure Temp
boolean measure = true;

// start with some initial colors
float minTemp = 0.0;
float maxTemp = 10.0;


// variables for interpolated colors
byte red, green, blue;

// variables for row/column interpolation
float intPoint, val, a, b, c, d, ii;
int x, y, i, j;


// array for the 32 x 24 measured pixels
#define MLX90640_pixel_x 32
#define MLX90640_pixel_y 24
static float pixels[ MLX90640_pixel_x * MLX90640_pixel_y ];



void setup() {
  Serial.begin(115200);
  Serial.println("Hello. nova netzbasteln ThermoCam");

  // Connect thermal sensor.
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  Wire.beginTransmission((uint8_t) MLX90640_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring.");
  }
  else {
    Serial.println("MLX90640 online!");
  }
  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("Parameter extraction failed");
  // Set refresh rate
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); // Set rate to 8Hz effective - Works at 800kHz
  // Once EEPROM has been read at 400kHz we can increase
  Wire.setClock(800000);

  // Set up Display.
  pinMode(TFT_DC, OUTPUT);
  SPI.begin();
  SPI.setFrequency(80000000L);
  Display.begin();
  //Display.setRotation(3);
  Display.fillScreen(C_BLACK);
}


void loop() {
  readPixels();
  if (setTempScale() == 0) {
    drawPicture();
    drawMeasurement();
  }
}


// Read pixel data from MLX90640.
void readPixels() {
  float emissivity = 0.95;
  
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature  

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels);
  }
}


// Show 32x24 sensor data on the top part of the 240x320 screen.
void drawPicture() {
  for (y=0; y < MLX90640_pixel_y; y++) {
    for (x=0; x < MLX90640_pixel_x; x++) {
      Display.fillRect(8 + x*7, 8 + y*7, 7, 7, getColor(pixels[(MLX90640_pixel_x -1 - x) + (y * MLX90640_pixel_x)]));
    }
  }
}


// Get color for temp value.
uint16_t getColor(float val) {
  /*
    pass in value and figure out R G B
    several published ways to do this I basically graphed R G B and developed simple linear equations
    again a 5-6-5 color display will not need accurate temp to R G B color calculation

    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html
    
  */

  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  if ((val > minTemp) & (val < a)) {
    green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c)) {
    green = 255;
  }
  else if (val > c) {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a)) {
    green = 0;
  }

  if (val <= b) {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d)) {
    blue = 0;
  }
  else if (val > d) {
    blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return Display.color565(red, green, blue);
}


int setTempScale() {
  minTemp = PLAUSI_MAX_TEMP;
  maxTemp = PLAUSI_MIN_TEMP;

  for (i = 0; i < sizeof(pixels)/sizeof(pixels[0]) ; i++) {
    minTemp = min(minTemp, pixels[i]);
    maxTemp = max(maxTemp, pixels[i]);
  }

/* avoid strange reading values:
 *  e.g. : 
 *  min 255.0 max 622.7
 *  min -173.5 max -131.6
*/
  if ( minTemp < PLAUSI_MIN_TEMP || maxTemp > PLAUSI_MAX_TEMP) { return 1; }
  // get the cutoff points for the color interpolation routines
  // note this function called when the temp scale is changed
  setAbcd();
  drawLegend();
  return 0;
}


// Function to get the cutoff points in the temp vs RGB graph.
void setAbcd() {
  a = minTemp + (maxTemp - minTemp) * 0.2121;
  b = minTemp + (maxTemp - minTemp) * 0.3182;
  c = minTemp + (maxTemp - minTemp) * 0.4242;
  d = minTemp + (maxTemp - minTemp) * 0.8182;
}


// Draw a legend.
void drawLegend() {
  float inc = (maxTemp - minTemp) / 224.0;
  j = 0;
  for (ii = minTemp; ii < maxTemp; ii += inc) {
    Display.drawFastVLine(8 + j++, 292, 20, getColor(ii));
  }

  Display.setTextFont(2);  // Small 16 pixel high font
  Display.setTextSize(1);
  Display.setCursor(8, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(minTemp,1));

  Display.setCursor(202, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(maxTemp,1));
  
  // Serial.println( "min " + String(minTemp,1) + " max " + String(maxTemp,1));
}


// Draw a circle + measured value.
void drawMeasurement() {
  float centerTemp;

  // Mark center measurement
  Display.drawCircle(120, 8+84, 3, TFT_WHITE);

  // Measure and print center temperature
  int pixel_row = MLX90640_pixel_y / 2;
  int pixel_col = MLX90640_pixel_x / 2;

  centerTemp = ( pixels[MLX90640_pixel_x * (pixel_row-1) + pixel_col -1] + pixels[MLX90640_pixel_x * (pixel_row-1) + pixel_col]
               + pixels[MLX90640_pixel_x *  pixel_row    + pixel_col -1] + pixels[MLX90640_pixel_x *  pixel_row    + pixel_col] ) / 4;
  // Serial.println(" center " + String(centerTemp,2));

  Display.setCursor(60, 218);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.setTextFont(1); // font GLCD to display degree char
  Display.setTextSize(4);
  Display.print(String(centerTemp,1));
  #define GLCD_CHAR_DEGREE 247
  Display.print(char(GLCD_CHAR_DEGREE));
  Display.print("C");
}
