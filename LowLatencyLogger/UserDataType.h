#ifndef UserDataType_h
#define UserDataType_h
const uint8_t ADC_DIM = 3;
struct data_t {
  uint32_t time;
  uint16_t adc[3];
  int16_t accel[3];
  int16_t euler[3];
  int16_t rateGyro[3];
  int16_t quat[4];
};
const uint8_t readPins[ADC_DIM] = {14,15,16};

#endif  // UserDataType_h
