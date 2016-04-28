/*
    This example shows how to use the IntervalTimer library and the ADC library in the tensy 3.0/3.1

    The three important objects are: the ADC, the (one or more) IntervalTimer and the same number of RingBuffer.

    The ADC sets up the internal ADC, you can change the settings as usual. Enable interrupts.
    The IntervalTimer (timerX) executes a function every 'period' microseconds.
    This function, timerX_callback, starts the desired ADC measurement, for example:
        startSingleRead, startSingleDifferential, startSynchronizedSingleRead, startSynchronizedSingleDifferential
    you can use the ADC0 or ADC1 (only for Teensy 3.1).

    When the measurement is done, the adc_isr is executed:
        - If you have more than one timer per ADC module you need to know which pin was measured.
        - Then you store/process the data
        - Finally, if you have more than one timer you can check whether the last measurement interruped a
          a previous one (using adc->adcX->adcWasInUse), and restart it if so.
          The settings of the interrupted measurement are stored in the adc->adcX->adc_config struct.


*/



#include "ADC.h" // Converts analog to digital. Part of Teensyduino
#include "RingBuffer.h" //
#include <IntervalTimer.h>


#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>

// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "analog"

// SD chip select pin.
const uint8_t SD_CS_PIN = 10;

//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional
// buffers.  QUEUE_DIM must be a power of two larger than
//(BUFFER_BLOCK_COUNT + 1).
//
#if RAMEND < 0X8FF
//#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 4;  // Must be a power of two!
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 8;  // Must be a power of two!
//
#elif RAMEND < 0X40FF
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 16;  // Must be a power of two!
//
#else  // RAMEND
// Use total of 29 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 28;
// Dimension for queues of 512 byte SD blocks.
const uint8_t QUEUE_DIM = 32;  // Must be a power of two!
#endif  // RAMEND

// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "tmp_log.bin"

SdFat sd;

SdBaseFile binFile;

char binName[13] = FILE_BASE_NAME "00.bin";
// Size of file base name.  Must not be larger than six.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

const size_t SAMPLES_PER_BLOCK = DATA_DIM16/PIN_COUNT;
typedef block16_t block_t;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
volatile uint8_t fullHead;  // volatile insures non-interrupt code sees changes.
uint8_t fullTail;

// queueNext assumes QUEUE_DIM is a power of two
inline uint8_t queueNext(uint8_t ht) {
  return (ht + 1) & (QUEUE_DIM -1);
}
//==============================================================================
// Interrupt Service Routines

// Pointer to current buffer.
block_t* isrBuf;

// Need new buffer if true.
bool isrBufNeeded = true;

// overrun count
uint16_t isrOver = 0;


const int ledPin = LED_BUILTIN;
const int unusedPin = 25;

const int readPin0 = A0; // X Axis Accel
const int period0 = 200; // us (200us = 5000Hz)

const int readPin1 = A1; // Y Axis Accel
const int period1 = 200; // us

const int readPin2 = A2; // Z Axis Accel
const int period2 = 200; // us

const uint32_t readPeriod = 100000; // us

ADC *adc = new ADC(); // adc object

IntervalTimer timer0, timer1, timer2; // timers

RingBuffer *buffer0 = new RingBuffer; // buffers to store the values
RingBuffer *buffer1 = new RingBuffer;
RingBuffer *buffer2 = new RingBuffer;

int startTimerValue0 = 0, startTimerValue1 = 0, startTimerValue2 = 0;

void setup() {

    pinMode(ledPin, OUTPUT); // led blinks every loop

//    pinMode(unusedPin+1, OUTPUT); // timer0 starts a measurement
//    pinMode(unusedPin+2, OUTPUT); // timer1 starts a measurement
//    pinMode(unusedPin+3, OUTPUT); // adc0_isr, measurement finished for readPin0
//    pinMode(unusedPin+4, OUTPUT); // adc0_isr, measurement finished for readPin1
//    pinMode(unusedPin+5, OUTPUT); // adc0_isr, measurement finished for readPin0
//    pinMode(unusedPin+6, OUTPUT); // adc0_isr, measurement finished for readPin1

    pinMode(readPin0, INPUT); pinMode(readPin1, INPUT), pinMode(readPin2, INPUT);
    
    digitalWrite(ledPin,HIGH);
    Serial.begin(9600);

    delay(1000);

    ///// ADC0 ////
    // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
    //adc->setReference(ADC_REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->setAveraging(8); // set number of averages
    adc->setResolution(16); // set bits of resolution

    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    // see the documentation for more information
    adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS); // change the conversion speed
    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    adc->setSamplingSpeed(ADC_MED_SPEED); // change the sampling speed
    // with 16 averages, 12 bits resolution and ADC_HIGH_SPEED conversion and sampling it takes about 32.5 us for a conversion

    Serial.println("Starting Timers");

    // start the timers, if it's not possible, startTimerValuex will be false
    startTimerValue0 = timer0.begin(timer0_callback, period0);
    // wait enough time for the first timer conversion to finish (depends on resolution and averaging),
    // with 16 averages, 12 bits, and ADC_MED_SPEED in both sampling and conversion speeds it takes about 36 us.
    delayMicroseconds(50); // if we wait less than 36us the timer1 will interrupt the conversion
    // initiated by timer0. The adc_isr will restart the timer0's measurement.

 
    // Timer0 starts a comversion and 25 us later timer1 starts a new one, "pausing" the first, about 36 us later timer1's conversion
    // is done, and timer0's is restarted, 36 us later timer0's conversion finishes. About 14 us later timer0 starts a new conversion again.
    // (times don't add up to 120 us because the timer_callbacks and adc_isr take time to execute, about 2.5 us and 1 us, respectively)
    // so in the worst case timer0 gets a new value in about twice as long as it would take alone.
    // if you change the periods, make sure you don't go into a loop, with the timers always interrupting each other
    startTimerValue1 = timer1.begin(timer1_callback, period1);
    delayMicroseconds(50); 
    startTimerValue2 = timer2.begin(timer2_callback, period2);
    
    adc->enableInterrupts(ADC_0);

    Serial.println("Timers started.");
  
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
        Serial.println("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }
  
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
  // Write metadata.
  if (!sd.card()->writeData((uint8_t*)&block[0])) {
    error("Write metadata failed");
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
  // Give SD time to prepare for big write.
  delay(1000);
  Serial.println(F("Logging - type any character to stop"));
  // Wait for Serial Idle.
  Serial.flush();
  delay(10);
  uint32_t bn = 1;
  uint32_t t0 = millis();
  uint32_t t1 = t0;
  uint32_t overruns = 0;
  uint32_t count = 0;
  uint32_t maxLatency = 0;

}

int value = 0;
char c=0;

void loop() {

    if(startTimerValue0==false) {
            Serial.println("Timer0 setup failed");
    }
    if(startTimerValue1==false) {
            Serial.println("Timer1 setup failed");
    }
    if(startTimerValue2==false) {
            Serial.println("Timer2 setup failed");
    }

    if(!buffer0->isEmpty()) { // read the values in the buffer
        Serial.print("Read pin 0: ");
        Serial.println(uint16_t(buffer0->read()));
        //Serial.println("New value!");
    }
    if(!buffer1->isEmpty()) { // read the values in the buffer
        Serial.print("Read pin 1: ");
        Serial.println(uint16_t(buffer1->read()));
        //Serial.println("New value!");
    }
    if(!buffer1->isEmpty()) { // read the values in the buffer
        Serial.print("Read pin 2: ");
        Serial.println(uint16_t(buffer2->read()));
        //Serial.println("New value!");
    }

    if (Serial.available()) {
        c = Serial.read();
        if(c=='s') { // stop timer
            Serial.println("Stop timer1");
            timer1.end();
        } else if(c=='r') { // restart timer
            Serial.println("Restart timer1");
            startTimerValue1 = timer1.begin(timer1_callback, period1);
        } else if(c=='p') { // restart timer
            Serial.print("isContinuous: ");
            Serial.println(adc->adc0->isContinuous());
        }
    }

    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );

    delayMicroseconds(readPeriod);
}

// This function will be called with the desired frequency
// start the measurement
// also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential

void timer0_callback(void) {
    adc->startSingleRead(readPin0, ADC_0); }
void timer1_callback(void) {
    adc->startSingleRead(readPin1, ADC_0);
}
void timer2_callback(void) {
    adc->startSingleRead(readPin2, ADC_0);
}

// when the measurement finishes, this will be called
// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {
    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

    // add value to correct buffer
    if(pin==readPin0) {
        buffer0->write(adc->readSingle());
    } 
    else if(pin==readPin1) {
        buffer1->write(adc->readSingle());
    }
    else if(pin==readPin2) {
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
void adc1_isr() {

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel
 
    // add value to correct buffer
    if(pin==readPin0) {
        buffer0->write(adc->readSingle());
    } 
    else if(pin==readPin1) {
        buffer1->write(adc->readSingle());
    }
    else if(pin==readPin2) {
        buffer2->write(adc->readSingle());
    } 
    else { // clear interrupt anyway
        adc->readSingle();
    }

    // restore ADC config if it was in use before being interrupted by the analog timer
    if (adc->adc1->adcWasInUse) {
        // restore ADC config, and restart conversion
        adc->adc1->loadConfig(&adc->adc1->adc_config);
        // avoid a conversion started by this isr to repeat itself
        adc->adc1->adcWasInUse = false;
    }

}
