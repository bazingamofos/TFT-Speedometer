#include <SoftwareSerial.h>
#include <SPI.h>
#include <TimerOne.h>   // library for timer interrupt
#define USE_SDFAT
#include <SdFat.h>            
SdFatSoftSpi<12, 11, 13> SD;  //Bit-Bang on the Shield pins

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

// Global variables for graphics
int randomvalue = -90;
float i = 0;
float angle, angle1, angle2, angle3, x, x1, x2, y, y1, y2;
float from = -90;
float till = 90;

#include <FreeDefaultFonts.h>

#define BLACK 0x0000
#define RED 0xF800
#define GREEN 0x07E0
#define WHITE 0xFFFF
#define GREY 0x8410
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0

#define SD_CS 10
#define PALETTEDEPTH 0
#define BMPIMAGEOFFSET 54
#define BUFFPIXEL 20

File root;
int pathlen;

float val[181];

float wantedval;
int wantedpos;
char speedC[8];
int k = 0;

// Speed calculations related constants and variables

const int irSensorPin = 20;  
//unsigned long lastTime = 6006;
unsigned long lastPulseTime = 0;
volatile int pulseCount = 0;
const float radius = 0.03;
float speed = 0;

void setup() {
  uint16_t ID;

  // IR sensor pin and interrupt that attaches itself everytime IR sensor goes low
  pinMode(irSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(irSensorPin), pulseDetected, FALLING);
  
  Timer1.initialize(1000000);  // Timer interval in microseconds (1 second in this case)
  Timer1.attachInterrupt(calculateSpeed);  // interrupt gets attached every 1 second
  
  Serial.begin(9600);

  Serial.println("Show BMP files on TFT with ID:0x");
  ID = tft.readID();
  Serial.println(ID, HEX);
  if (ID == 0x0D3D3) ID = 0x9481;
  tft.begin(ID);
  tft.fillScreen(BLACK);
  tft.setTextColor(0xFFFF, 0x0000);
  bool good = SD.begin(SD_CS);
  if (!good) {
    Serial.println(F("cannot start SD"));
    while (1)
      ;
  }

  tft.setRotation(1);
  showBMP("/speedo~1.bmp", 0, 0);

  tft.fillCircle(158, 150, 13, RED);
  angle = (-90 / 57.2958) - 1.57;
  x = 170 + cos(angle) * 80;
  y = 200 + sin(angle) * 80;
  angle1 = angle - 1.57;
  angle2 = angle + 1.57;
  angle3 = angle + 3.14;

  x1 = 160 + cos(angle1) * 6;
  y1 = 150 + sin(angle1) * 6;
  x2 = 160 + cos(angle2) * 6;
  y2 = 150 + sin(angle2) * 6;
  
  tft.fillCircle(160, 239, 75, BLACK);
 
  tft.fillTriangle(x, y, x1, y1, x2, y2, RED);

  showmsgXY(137, 215, 2, NULL, "KMPH");

  //define and initialize dummy array for speed
  float j = 0;
  for ( k = 0; k < 181; k++) {
    val[k] = j;
    j = j + 0.667;
  }
  k=0;
}

void loop(void) {

  float tolerance = 0.6;
  wantedval = speed;

  for (int q = 0; q < 181; q++) {
    if (fabs(wantedval - val[q]) <= tolerance) {
      wantedpos = q;
      break;
    }
  }
  
  till = wantedpos - 90;
  wantedpos = 0;

  if (till > from) {
    for (int j = from; j <= till; j++) {
      angle = (j / 57.2958) - 1.57;
      x = 170 + cos(angle) * (80);
      y = 200 + sin(angle) * (80);
      angle1 = angle - 1.57;
      angle2 = angle + 1.57;
      angle3 = angle + 3.14;

      x1 = 160 + cos(angle1) * 6;
      y1 = 150 + sin(angle1) * 6;
      x2 = 160 + cos(angle2) * 6;
      y2 = 150 + sin(angle2) * 6;
      tft.fillTriangle(x, y, x1, y1, x2, y2, BLACK);

      angle = ((j + 2) / 57.2958) - 1.57;
      x = 170 + cos(angle) * (80);
      y = 200 + sin(angle) * (80);
      angle1 = angle - 1.57;
      angle2 = angle + 1.57;
      angle3 = angle + 3.14;

      x1 = 160 + cos(angle1) * 6;
      y1 = 150 + sin(angle1) * 6;
      x2 = 160 + cos(angle2) * 6;
      y2 = 150 + sin(angle2) * 6;
      tft.fillTriangle(x, y, x1, y1, x2, y2, RED);

      if (val[k] < 100)
        val[k] = floor(10 * val[k]) / 10;
      else
        val[k] = floor(val[k]);
      
      dtostrf(val[k], 5, 2, speedC);

      k++;
      showmsgXY(115, 183, 3, NULL, speedC);
      tft.fillCircle(158, 150, 13, RED);

      from = till;
    }
  } else {
    for (int j = from; j >= till; j--) {
      angle = ((j + 2) / 57.2958) - 1.57;
      x = 170 + cos(angle) * (80);
      y = 200 + sin(angle) * (80);
      angle1 = angle - 1.57;
      angle2 = angle + 1.57;
      angle3 = angle + 3.14;

      x1 = 160 + cos(angle1) * 6;
      y1 = 150 + sin(angle1) * 6;
      x2 = 160 + cos(angle2) * 6;
      y2 = 150 + sin(angle2) * 6;
      tft.fillTriangle(x, y, x1, y1, x2, y2, BLACK);

      angle = (j / 57.2958) - 1.57;
      x = 170 + cos(angle) * (80);
      y = 200 + sin(angle) * (80);
      angle1 = angle - 1.57;
      angle2 = angle + 1.57;
      angle3 = angle + 3.14;

      x1 = 160 + cos(angle1) * 6;
      y1 = 150 + sin(angle1) * 6;
      x2 = 160 + cos(angle2) * 6;
      y2 = 150 + sin(angle2) * 6;
      tft.fillTriangle(x, y, x1, y1, x2, y2, RED);

      if (val[k] < 100)
        val[k] = floor(10 * val[k]) / 10;
      else
        val[k] = floor(val[k]);
      
      dtostrf(val[k], 5, 2, speedC);

      k--;
      if( k < 0 )
        k = 0;

      showmsgXY(115, 183, 3, NULL, speedC);
      tft.fillCircle(158, 150, 13, RED);

      from = till;
    }
  }
}


uint16_t read16(File& f) {
  uint16_t result;  // read little-endian
  f.read((uint8_t*)&result, sizeof(result));
  return result;
}

uint32_t read32(File& f) {
  uint32_t result;
  f.read((uint8_t*)&result, sizeof(result));
  return result;
}

uint8_t showBMP(char* nm, int x, int y) {
  File bmpFile;
  int bmpWidth, bmpHeight;          // W+H in pixels
  uint8_t bmpDepth;                 // Bit depth (currently must be 24, 16, 8, 4, 1)
  uint32_t bmpImageoffset;          // Start of image data in file
  uint32_t rowSize;                 // Not always = bmpWidth; may have padding
  uint8_t sdbuffer[3 * BUFFPIXEL];  // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[(1 << PALETTEDEPTH) + BUFFPIXEL], *palette = NULL;
  uint8_t bitmask, bitshift;
  boolean flip = true;  // BMP is stored bottom-to-top
  int w, h, row, col, lcdbufsiz = (1 << PALETTEDEPTH) + BUFFPIXEL, buffidx;
  uint32_t pos;           // seek position
  boolean is565 = false;  //

  uint16_t bmpID;
  uint16_t n;  // blocks read
  uint8_t ret;

  if ((x >= tft.width()) || (y >= tft.height()))
    return 1;  // off screen

  bmpFile = SD.open(nm);             // Parse BMP header
  bmpID = read16(bmpFile);           // BMP signature
  (void)read32(bmpFile);             // Read & ignore file size
  (void)read32(bmpFile);             // Read & ignore creator bytes
  bmpImageoffset = read32(bmpFile);  // Start of image data
  (void)read32(bmpFile);             // Read & ignore DIB header size
  bmpWidth = read32(bmpFile);
  bmpHeight = read32(bmpFile);
  n = read16(bmpFile);                                         // # planes -- must be '1'
  bmpDepth = read16(bmpFile);                                  // bits per pixel
  pos = read32(bmpFile);                                       // format
  if (bmpID != 0x4D42) ret = 2;                                // bad ID
  else if (n != 1) ret = 3;                                    // too many planes
  else if (pos != 0 && pos != 3) ret = 4;                      // format: 0 = uncompressed, 3 = 565
  else if (bmpDepth < 16 && bmpDepth > PALETTEDEPTH) ret = 5;  // palette
  else {
    bool first = true;
    is565 = (pos == 3);  // ?already in 16-bit format
    // BMP rows are padded (if needed) to 4-byte boundary
    rowSize = (bmpWidth * bmpDepth / 8 + 3) & ~3;
    if (bmpHeight < 0) {  // If negative, image is in top-down order.
      bmpHeight = -bmpHeight;
      flip = false;
    }

    w = bmpWidth;
    h = bmpHeight;
    if ((x + w) >= tft.width())  // Crop area to be loaded
      w = tft.width() - x;
    if ((y + h) >= tft.height())  //
      h = tft.height() - y;

    if (bmpDepth <= PALETTEDEPTH) {  // these modes have separate palette
      //bmpFile.seek(BMPIMAGEOFFSET); //palette is always @ 54
      bmpFile.seek(bmpImageoffset - (4 << bmpDepth));  //54 for regular, diff for colorsimportant
      bitmask = 0xFF;
      if (bmpDepth < 8)
        bitmask >>= bmpDepth;
      bitshift = 8 - bmpDepth;
      n = 1 << bmpDepth;
      lcdbufsiz -= n;
      palette = lcdbuffer + lcdbufsiz;
      for (col = 0; col < n; col++) {
        pos = read32(bmpFile);  //map palette to 5-6-5
        palette[col] = ((pos & 0x0000F8) >> 3) | ((pos & 0x00FC00) >> 5) | ((pos & 0xF80000) >> 8);
      }
    }

    // Set TFT address window to clipped image bounds
    tft.setAddrWindow(x, y, x + w - 1, y + h - 1);
    for (row = 0; row < h; row++) {  // For each scanline...
      // Seek to start of scan line.  It might seem labor-
      // intensive to be doing this on every line, but this
      // method covers a lot of gritty details like cropping
      // and scanline padding.  Also, the seek only takes
      // place if the file position actually needs to change
      // (avoids a lot of cluster math in SD library).
      uint8_t r, g, b, *sdptr;
      int lcdidx, lcdleft;
      if (flip)  // Bitmap is stored bottom-to-top order (normal BMP)
        pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
      else  // Bitmap is stored top-to-bottom
        pos = bmpImageoffset + row * rowSize;
      if (bmpFile.position() != pos) {  // Need seek?
        bmpFile.seek(pos);
        buffidx = sizeof(sdbuffer);  // Force buffer reload
      }

      for (col = 0; col < w;) {  //pixels in row
        lcdleft = w - col;
        if (lcdleft > lcdbufsiz) lcdleft = lcdbufsiz;
        for (lcdidx = 0; lcdidx < lcdleft; lcdidx++) {  // buffer at a time
          uint16_t color;
          // Time to read more pixel data?
          if (buffidx >= sizeof(sdbuffer)) {  // Indeed
            bmpFile.read(sdbuffer, sizeof(sdbuffer));
            buffidx = 0;  // Set index to beginning
            r = 0;
          }
          switch (bmpDepth) {  // Convert pixel from BMP to TFT format
            case 32:
            case 24:
              b = sdbuffer[buffidx++];
              g = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              if (bmpDepth == 32) buffidx++;  //ignore ALPHA
              color = tft.color565(r, g, b);
              break;
            case 16:
              b = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              if (is565)
                color = (r << 8) | (b);
              else
                color = (r << 9) | ((b & 0xE0) << 1) | (b & 0x1F);
              break;
            case 1:
            case 4:
            case 8:
              if (r == 0)
                b = sdbuffer[buffidx++], r = 8;
              color = palette[(b >> bitshift) & bitmask];
              r -= bmpDepth;
              b <<= bmpDepth;
              break;
          }
          lcdbuffer[lcdidx] = color;
        }
        tft.pushColors(lcdbuffer, lcdidx, first);
        first = false;
        col += lcdidx;
      }                                                          // end cols
    }                                                            // end rows
    tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1);  //restore full screen
    ret = 0;                                                     // good render
  }
  bmpFile.close();
  //return (ret);
}

void showmsgXY(int x, int y, int sz, const GFXfont* f, const char* msg) {
  int16_t x1, y1;
  uint16_t wid, ht;
  tft.setFont(f);
  tft.setCursor(x, y);
  tft.setTextColor(GREEN, BLACK);
  tft.setTextSize(sz);
  tft.print(msg);
}

void pulseDetected() {
  // ISR for IR sensor pulse detection
  if (millis() - lastPulseTime >= 15) {  // Minimum time between pulses to avoid multiple counts
    pulseCount++;
    lastPulseTime = millis();
  }
}

void calculateSpeed() {
  detachInterrupt(digitalPinToInterrupt(irSensorPin));
  // calculate rpm and speed
  int rpm = (pulseCount * 60);
  speed = (2 * 3.14 * radius * rpm) * 60.0 / 1000.0;
  Serial.println("RPM: " + String(rpm));

  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.println(" kmph");

  pulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(irSensorPin), pulseDetected, FALLING);

}
