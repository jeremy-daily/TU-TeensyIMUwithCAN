#ifndef UserDataType_h
#define UserDataType_h
const uint8_t ADC_DIM = 3;
struct data_t {
  uint32_t time;
  uint16_t adc[ADC_DIM];
};
const uint8_t readPins[ADC_DIM] = {14,15,16};

#endif  // UserDataType_h
