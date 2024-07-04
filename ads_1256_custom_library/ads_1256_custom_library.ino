#include <SPI.h>

//(other stuff for getting the ADS1526 to work is in the next tab
#define ADS_RST_PIN    8 //ADS1256 reset pin
#define ADS_RDY_PIN    22 //ADS1256 data ready
#define ADS_CS_PIN    21 //ADS1256 chip select


/* 
    CLK  - pin 13
    DIN  - pin 11 (MOSI)
    DOUT - pin 12 (MISO)
*/
#include "SdFat.h"
#include "RingBuf.h"

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 1000

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE 10*25000*600  // 150,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512
#define LOG_FILENAME "SdioLogger.csv"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;




//put the ADC constants here

double resolution = 8388608.; //2^23-1

//this needs to match the setting in the ADC init function in the library tab
double Gain = 64.; //be sure to have a period 

double vRef = 5.0; //reference voltage

//we'll calculate this in setup
double bitToVolt = 0.;




void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("booting");
  //initialize the ADS
  pinMode(ADS_CS_PIN, OUTPUT);

  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);

  SPI.begin();

  initADS();
  Serial.println("done init");

  //determine the conversion factor
    //do some calculations for the constants
  bitToVolt = resolution*Gain/vRef;
}



int32_t val1;
int32_t val2;
int32_t val3;



void loop() {
  clearSerialInput();
  Serial.println("Type any character to start");
  while (!Serial.available()) {};
  clearSerialInput();
  logData();
}






void logData() {
  
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
     Serial.println("preAllocate failed\n");
     file.close();
     return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.println("Type any character to stop");

  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX;

  // Start time.
  uint32_t logTime = micros();
  // Log data until Serial input or file full.
  while (!Serial.available()) {

for (int i = 0; i <1; i++){
  read_three_values();
}
    
    // Amount of data in ringBuf.
    size_t n = rb.bytesUsed();
    if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
      Serial.println("File full - quitting.");
      break;
    }
    if (n > maxUsed) {
      maxUsed = n;
    }
    if (n >= 512 && !file.isBusy()) {
      // Not busy only allows one sector before possible busy wait.
      // Write one sector from RingBuf to file.
      if (512 != rb.writeOut(512)) {
        Serial.println("writeOut failed");
        break;
      }
    }
    // Time for next point.
    logTime += LOG_INTERVAL_USEC;
    int32_t spareMicros = logTime - micros();
    if (spareMicros < minSpareMicros) {
      minSpareMicros = spareMicros;
    }
    if (spareMicros <= 0) {
      Serial.print("Rate too fast ");
      Serial.println(spareMicros);
      break;
    }
    // Wait until time to log data.
    while (micros() < logTime) {}

    // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
//    uint16_t adc = analogRead(0);
    // Print spareMicros into the RingBuf as test data.
    rb.print(spareMicros);
    rb.write(',');
    // Print adc into RingBuf.
    rb.println(val1,8);
    if (rb.getWriteError()) {
      // Error caused by too few free bytes in RingBuf.
      Serial.println("WriteError");
      break;
    }
  }
  // Write any RingBuf data to file.
  rb.sync();
  file.truncate();
  file.rewind();
  // Print first twenty lines of file.
  Serial.println("spareMicros,ADC0");
  for (uint8_t n = 0; n < 20 && file.available();) {
    int c = file.read();
    if (c < 0) {
      break;
    }
    Serial.write(c);
    if (c == '\n') n++;
  }
  Serial.print("fileSize: ");
  Serial.println((uint32_t)file.fileSize());
  Serial.print("maxBytesUsed: ");
  Serial.println(maxUsed);
  Serial.print("minSpareMicros: ");
  Serial.println(minSpareMicros);
  file.close();
}

void clearSerialInput() {
  for (uint32_t m = micros(); micros() - m < 10000;) {
    if (Serial.read() >= 0) {
      m = micros();
    }
  }
}


