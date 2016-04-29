/**
 * This program logs data to a binary file.  Functions are included
 * to convert the binary file to a csv text file.
 *
 * Samples are logged at regular intervals.  The maximum logging rate
 * depends on the quality of your SD card and the time required to
 * read sensor data.  This example has been tested at 500 Hz with
 * good SD card on an Uno.  4000 HZ is possible on a Due.
 *
 * If your SD card has a long write latency, it may be necessary to use
 * slower sample rates.  Using a Mega Arduino helps overcome latency
 * problems since 13 512 byte buffers will be used.
 *
 * Data is written to the file using a SD multiple block write command.
 */
 
#include "ADC.h"
#include "RingBuffer.h"
#include <IntervalTimer.h>

#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
//------------------------------------------------------------------------------
// User data functions.  Modify these functions for your data items.
#include "UserDataType.h"

//Calibration Data: Based on 1 G Tests and data sheet
const uint16_t Xzero = 32770; //LSB per milliG
const int XGain = 6102; //LSB per milliG
const uint16_t Yzero = 32844; //LSB per milliG
const int YGain = 6102; //LSB per milliG
const uint16_t Zzero = 32803; //LSB per milliG
const int ZGain = 6102; //LSB per milliG


IntervalTimer timer0, timer1, timer2; // timers

RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
RingBuffer *buffer1 = new RingBuffer;
RingBuffer *buffer2 = new RingBuffer;

int startTimerValue0 = 0, startTimerValue1 = 0, startTimerValue2 = 0;

char serialInput;


//==============================================================================
// Start of configuration constants.
//==============================================================================
//Interval between data records in microseconds.
const uint32_t LOG_INTERVAL_USEC = 500;
//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = 9;

const uint8_t recordSwitchPin = 0;

//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = 13;


ADC *adc = new ADC(); // adc object


// Acquire a data record.
void acquireData(data_t* data) {
  if(!buffer2->isEmpty() && !buffer1->isEmpty() && !buffer0->isEmpty() ) {
    data->time = micros();
    data->adc[0] = uint16_t(buffer0->read());
    data->adc[1] = uint16_t(buffer1->read());
    data->adc[2] = uint16_t(buffer2->read());
  }
}

// Print a data record.
void printData(Print* pr, data_t* data) {
  pr->print(data->time);
  
  pr->print(",");
  pr->print((data->adc[0] - Xzero)*XGain);
  pr->print(",");
  pr->print((data->adc[1] - Yzero)*YGain);
  pr->print(",");
  pr->print((data->adc[2] - Zzero)*ZGain); 
  pr->println();
}

// Print data header.
void printHeader(Print* pr) {
  pr->print("Time");
  
  pr->print(",");
  pr->print("X Accel");
  pr->print(",");
  pr->print("Y Accel");
  pr->print(",");
  pr->print("Z Accel"); 
  pr->println();
  
  pr->print("[microsec]");
  
  pr->print(",");
  pr->print("[micro Gs]");
  pr->print(",");
  pr->print("[micro Gs]");
  pr->print(",");
  pr->print("[micro Gs]"); 
  pr->println();
}

//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 2048000;

// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "data"
//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional
// buffers.
//
#ifndef RAMEND
// Assume ARM. Use total of nine 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 8;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND
//==============================================================================
// End of configuration constants.
//==============================================================================
// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "tmp_log.bin"

// Size of file base name.  Must not be larger than six.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

boolean  recording = false;

SdFat sd;

SdBaseFile binFile;

char binName[13] = FILE_BASE_NAME "00.bin";

// Number of data records in a block.
const uint16_t DATA_DIM = (512 - 4)/sizeof(data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM*sizeof(data_t);

struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};

const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 2;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;

// Advance queue index.
inline uint8_t queueNext(uint8_t ht) {
  return ht < (QUEUE_DIM - 1) ? ht + 1 : 0;
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) errorFlash(F(msg))
//------------------------------------------------------------------------------
void errorFlash(const __FlashStringHelper* msg) {
  sd.errorPrint(msg);
  fatalBlink();
}
//------------------------------------------------------------------------------
//
void fatalBlink() {
  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
    }
  }
}
//==============================================================================
// Convert binary file to csv file.
void binaryToCsv() {
  uint8_t lastPct = 0;
  block_t block;
  uint32_t t0 = millis();
  uint32_t syncCluster = 0;
  SdFile csvFile;
  char csvName[13];

  if (!binFile.isOpen()) {
    Serial.println();
    Serial.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  // Create a new csvFile.
  strcpy(csvName, binName);
  strcpy(&csvName[BASE_NAME_SIZE + 3], "csv");

  if (!csvFile.open(csvName, O_WRITE | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  Serial.println();
  Serial.print(F("Writing: "));
  Serial.print(csvName);
  Serial.println(F(" - type any character to stop"));
  printHeader(&csvFile);
  uint32_t tPct = millis();
  while (!Serial.available() && binFile.read(&block, 512) == 512) {
    uint16_t i;
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      csvFile.print(F("OVERRUN,"));
      csvFile.println(block.overrun);
    }
    for (i = 0; i < block.count; i++) {
      printData(&csvFile, &block.data[i]);
    }
    if (csvFile.curCluster() != syncCluster) {
      csvFile.sync();
      syncCluster = csvFile.curCluster();
    }
    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
      }
    }
    if (Serial.available()) {
      break;
    }
  }
  csvFile.close();
  Serial.print(F("Done: "));
  Serial.print(0.001*(millis() - t0));
  Serial.println(F(" Seconds"));
}
//------------------------------------------------------------------------------
// read data file and check for overruns
void checkOverrun() {
  bool headerPrinted = false;
  block_t block;
  uint32_t bgnBlock, endBlock;
  uint32_t bn = 0;

  if (!binFile.isOpen()) {
    Serial.println();
    Serial.println(F("No current binary file"));
    return;
  }
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  binFile.rewind();
  Serial.println();
  Serial.println(F("Checking overrun errors - type any character to stop"));
  while (binFile.read(&block, 512) == 512) {
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      if (!headerPrinted) {
        Serial.println();
        Serial.println(F("Overruns:"));
        Serial.println(F("fileBlockNumber,sdBlockNumber,overrunCount"));
        headerPrinted = true;
      }
      Serial.print(bn);
      Serial.print(',');
      Serial.print(bgnBlock + bn);
      Serial.print(',');
      Serial.println(block.overrun);
    }
    bn++;
  }
  if (!headerPrinted) {
    Serial.println(F("No errors found"));
  } else {
    Serial.println(F("Done"));
  }
}
//------------------------------------------------------------------------------
// dump data file to Serial
void dumpData() {
  block_t block;
  if (!binFile.isOpen()) {
    Serial.println();
    Serial.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  Serial.println();
  Serial.println(F("Type any character to stop"));
  delay(1000);
  printHeader(&Serial);
  while (!Serial.available() && binFile.read(&block , 512) == 512) {
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      Serial.print(F("OVERRUN,"));
      Serial.println(block.overrun);
    }
    for (uint16_t i = 0; i < block.count; i++) {
      printData(&Serial, &block.data[i]);
    }
  }
  Serial.println(F("Done"));
}
//------------------------------------------------------------------------------
// log data
// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;
void logData() {
  uint32_t bgnBlock, endBlock;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT];
  block_t* curBlock = 0;
  Serial.println();

  // Find unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(binName)) {
    if (binName[BASE_NAME_SIZE + 1] != '9') {
      binName[BASE_NAME_SIZE + 1]++;
    } else {
      binName[BASE_NAME_SIZE + 1] = '0';
      if (binName[BASE_NAME_SIZE] == '9') {
        error("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }
  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME)) {
    Serial.println(F("Deleting tmp file"));
    if (!sd.remove(TMP_FILE_NAME)) {
      error("Can't remove tmp file");
    }
  }
  // Create new file.
  Serial.println(F("Creating new file"));
  binFile.close();
  if (!binFile.createContiguous(sd.vwd(),
                                TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
    error("createContiguous failed");
  }
  // Get the address of the file on the SD.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  // Use SdFat's internal buffer.
  uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
  if (cache == 0) {
    error("cacheClear failed");
  }

  // Flash erase all data in the file.
  Serial.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock) {
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock) {
      endErase = endBlock;
    }
    if (!sd.card()->erase(bgnErase, endErase)) {
      error("erase failed");
    }
    bgnErase = endErase + 1;
  }
  // Start a multiple block write.
  if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) {
    error("writeBegin failed");
  }
  // Initialize queues.
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;

  // Use SdFat buffer for one block.
  emptyQueue[emptyHead] = (block_t*)cache;
  emptyHead = queueNext(emptyHead);

  // Put rest of buffers in the empty queue.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyQueue[emptyHead] = &block[i];
    emptyHead = queueNext(emptyHead);
  }
  Serial.println(F("Logging - type any character to stop"));
  // Wait for Serial Idle.
  Serial.flush();
  delay(10);
  uint32_t bn = 0;
  uint32_t t0 = millis();
  uint32_t t1 = t0;
  uint32_t overrun = 0;
  uint32_t overrunTotal = 0;
  uint32_t count = 0;
  uint32_t maxLatency = 0;
  int32_t diff;
  // Start at a multiple of interval.
  uint32_t logTime = micros()/LOG_INTERVAL_USEC + 1;
  logTime *= LOG_INTERVAL_USEC;
  bool closeFile = false;
  while (1) {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;
    if (Serial.available()) {
      closeFile = true;
    }
    if (!digitalRead(recordSwitchPin))
    {
      closeFile = true;
    }
    if (closeFile) {
      if (curBlock != 0 && curBlock->count >= 0) {
        // Put buffer in full queue.
        fullQueue[fullHead] = curBlock;
        fullHead = queueNext(fullHead);
        curBlock = 0;
      }
    } else {
      if (curBlock == 0 && emptyTail != emptyHead) {
        curBlock = emptyQueue[emptyTail];
        emptyTail = queueNext(emptyTail);
        curBlock->count = 0;
        curBlock->overrun = overrun;
        overrun = 0;
      }
      do {
        diff = logTime - micros();
      } while(diff > 0);
      if (diff < -10) {
        error("LOG_INTERVAL_USEC too small");
      }
      if (curBlock == 0) {
        overrun++;
      } else {
        acquireData(&curBlock->data[curBlock->count++]);
        if (curBlock->count == DATA_DIM) {
          fullQueue[fullHead] = curBlock;
          fullHead = queueNext(fullHead);
          curBlock = 0;
        }
      }
    }

    if (fullHead == fullTail) {
      // Exit loop if done.
      if (closeFile) {
        break;
      }
    } else if (!sd.card()->isBusy()) {
      // Get address of block to write.
      block_t* pBlock = fullQueue[fullTail];
      fullTail = queueNext(fullTail);
      // Write block to SD.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t*)pBlock)) {
        error("write data failed");
      }
      usec = micros() - usec;
      t1 = millis();
      if (usec > maxLatency) {
        maxLatency = usec;
      }
      count += pBlock->count;

      // Add overruns and possibly light LED.
      if (pBlock->overrun) {
        overrunTotal += pBlock->overrun;
        if (ERROR_LED_PIN >= 0) {
          digitalWrite(ERROR_LED_PIN, HIGH);
        }
      }
      // Move block to empty queue.
      emptyQueue[emptyHead] = pBlock;
      emptyHead = queueNext(emptyHead);
      bn++;
      if (bn == FILE_BLOCK_COUNT) {
        // File full so stop
        break;
      }
    }
  }
  if (!sd.card()->writeStop()) {
    error("writeStop failed");
  }
  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT) {
    Serial.println(F("Truncating file"));
    if (!binFile.truncate(512L * bn)) {
      error("Can't truncate file");
    }
  }
  if (!binFile.rename(sd.vwd(), binName)) {
    error("Can't rename file");
  }
  Serial.print(F("File renamed: "));
  Serial.println(binName);
  Serial.print(F("Max block write usec: "));
  Serial.println(maxLatency);
  Serial.print(F("Record time sec: "));
  Serial.println(0.001*(t1 - t0), 3);
  Serial.print(F("Sample count: "));
  Serial.println(count);
  Serial.print(F("Samples/sec: "));
  Serial.println((1000.0)*count/(t1-t0));
  Serial.print(F("Overruns: "));
  Serial.println(overrunTotal);
  binaryToCsv();
  Serial.println(F("Done"));
}
//------------------------------------------------------------------------------
void setup(void) {
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }
  pinMode(recordSwitchPin,INPUT_PULLUP);
  recording = digitalRead(recordSwitchPin);
  digitalWrite(ERROR_LED_PIN,HIGH);
  
  Serial.begin(9600);
  delay(1000);

  Serial.print(F("FreeRam: "));
  Serial.println(FreeRam());
  Serial.print(F("Records/block: "));
  Serial.println(DATA_DIM);
  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }
  // initialize file system.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    fatalBlink();
    sd.initErrorPrint();
    
  }
  
   
    ///// ADC0 ////
    // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
    //adc->setReference(ADC_REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->setAveraging(8); // set number of averages
    adc->setResolution(16); // set bits of resolution

    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    // see the documentation for more information
    adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS); // change the conversion speed
    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    adc->setSamplingSpeed(ADC_HIGH_SPEED); // change the sampling speed
    // with 16 averages, 12 bits resolution and ADC_HIGH_SPEED conversion and sampling it takes about 32.5 us for a conversion
    
    Serial.println("Starting Timers");

    // start the timers, if it's not possible, startTimerValuex will be false
    startTimerValue0 = timer0.begin(timer0_callback, LOG_INTERVAL_USEC);
    // wait enough time for the first timer conversion to finish (depends on resolution and averaging),
    // with 16 averages, 12 bits, and ADC_MED_SPEED in both sampling and conversion speeds it takes about 36 us.
    delayMicroseconds(50); // if we wait less than 36us the timer1 will interrupt the conversion
    // initiated by timer0. The adc_isr will restart the timer0's measurement.

    // You can check with an oscilloscope:
    // Pin 14 corresponds to the timer0 initiating a measurement
    // Pin 15 the same for the timer1
    // Pin 16 is the adc_isr when there's a new measurement on readpin0
    // Pin 17 is the adc_isr when there's a new measurement on readpin1

    // Timer0 starts a comversion and 25 us later timer1 starts a new one, "pausing" the first, about 36 us later timer1's conversion
    // is done, and timer0's is restarted, 36 us later timer0's conversion finishes. About 14 us later timer0 starts a new conversion again.
    // (times don't add up to 120 us because the timer_callbacks and adc_isr take time to execute, about 2.5 us and 1 us, respectively)
    // so in the worst case timer0 gets a new value in about twice as long as it would take alone.
    // if you change the periods, make sure you don't go into a loop, with the timers always interrupting each other
    startTimerValue1 = timer1.begin(timer1_callback, LOG_INTERVAL_USEC);
    delayMicroseconds(50); 
    startTimerValue2 = timer2.begin(timer2_callback, LOG_INTERVAL_USEC);
    
    adc->enableInterrupts(ADC_0);

    Serial.println("Timers started.");
}
//------------------------------------------------------------------------------
void loop(void) {
  // discard any input
  
  while (Serial.read() >= 0) {}
  Serial.println();
  Serial.println(F("type:"));
  Serial.println(F("c - convert file to csv"));
  Serial.println(F("d - dump data to Serial"));
  Serial.println(F("e - overrun error details"));
  Serial.println(F("r - record data"));
  
  while(!Serial.available()) {
      if (digitalRead(recordSwitchPin)) {
        delay(30);
        if (digitalRead(recordSwitchPin)) logData();
      }
  }
  serialInput = tolower(Serial.read());
  
  // Discard extra Serial data.
  do {
    delay(10);
  } while (Serial.read() >= 0);

  if (ERROR_LED_PIN >= 0) {
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  if (serialInput == 'c') {
    binaryToCsv();
  } else if (serialInput == 'd') {
    dumpData();
  } else if (serialInput == 'e') {
    checkOverrun();
  } else  if (serialInput == 'r') {
    logData();
  
  } else {
    Serial.println(F("Invalid entry"));
  }
}


// This function will be called with the desired frequency
// start the measurement
void timer0_callback(void) {
    adc->startSingleRead(readPins[0], ADC_0); // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential
}
void timer1_callback(void) {
    adc->startSingleRead(readPins[1], ADC_0);
}
void timer2_callback(void) {
     adc->startSingleRead(readPins[2], ADC_0);
}

// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel
   
    // add value to correct buffer
    if(pin==readPins[0]) {
         buffer0->write(adc->readSingle());
    } 
    else if(pin==readPins[1]) {
         buffer1->write(adc->readSingle());
     }
    else if(pin==readPins[2]) {
        buffer2->write(adc->readSingle());
      } 
    else { // clear interrupt anyway
        adc->readSingle();
    }

    // restore ADC config if it was in use before being interrupted by the analog timer
    if (adc->adc0->adcWasInUse) {
        // restore ADC config, and restart conversion
        adc->adc0->loadConfig(&adc->adc0->adc_config);
        // avoid a conversion started by this isr to repeat itself
        adc->adc0->adcWasInUse = false;
    }
}
