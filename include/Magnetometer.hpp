#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

namespace Magneto
{
    enum Mode
    {
        MODE_STANDBY = 0,
        MODE_CONTINUOUS
    };

    enum DataRate
    {
        DATARATE_10HZ = 0,
        DATARATE_50HZ,
        DATARATE_100HZ,
        DATARATE_200HZ
    };

    enum Range
    {
        RANGE_2GA = 0,
        RANGE_8GA
    };
    enum OverSample
    {
        OVERSAMPLE_512 = 0,
        OVERSAMPLE_256,
        OVERSAMPLE_128,
        OVERSAMPLE_64,
    };

    struct CalibrationData
    {
        int16_t offsetX;
        int16_t offsetY;
        int16_t offsetZ;
        float gainX;
        float gainY;
        float gainZ;
    };

    bool begin(Mode m, DataRate odr, Range rng, OverSample osr);
    void setCalibrationData(CalibrationData *calibration);
    void getCalibrationData(CalibrationData *calibration);
    // void runCalibration(CalibrationData *calibration);
    void runCalibration(CalibrationData *calibration, TFT_eSPI *tft); // Show calibration data on TFT
    inline String getCalibrationDataString(CalibrationData *calibration)
    {
        return "OffsetX: " + String(calibration->offsetX) + " OffsetY: " + String(calibration->offsetY) +
               " OffsetZ: " + String(calibration->offsetZ) + " GainX: " + String(calibration->gainX) +
               " GainY: " + String(calibration->gainY) + " GainZ: " + String(calibration->gainZ);
    }

    bool getRawData(int16_t *xyz);
    bool getRawDataWithCal(int16_t *xyz);
    bool getGauss(float *xyz);
    float getHeading();

} // namespace Magneto