#include <Arduino.h>
#include <Wire.h>

namespace Magneto {
    enum Mode { MODE_STANDBY = 0, MODE_CONTINUOUS };

    enum DataRate {
        DATARATE_10HZ = 0,
        DATARATE_50HZ,
        DATARATE_100HZ,
        DATARATE_200HZ
    };

    enum Range { RANGE_2GA = 0, RANGE_8GA };
    enum OverSample {
        OVERSAMPLE_512 = 0,
        OVERSAMPLE_256,
        OVERSAMPLE_128,
        OVERSAMPLE_64,
    };

    bool begin(Mode m, DataRate odr, Range rng, OverSample osr);
    void setCalibrationData(int16_t *xyz);
    void getCalibrationData(int16_t *xyz);
    void runCalibration(int16_t *offsetXYZ);
    bool getRawData(int16_t *xyz);
    bool getRawDataWithCal(int16_t *xyz);
    bool getGauss(float *xyz);
    void getHeading(float *heading);
} // namespace Magneto