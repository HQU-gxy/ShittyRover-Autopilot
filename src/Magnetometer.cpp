#include <EEPROM.h>

#include "CMSIS_DSP.h"
#include "Magnetometer.hpp"
#include "config.h"

namespace Magneto
{
    static TwoWire i2c1(I2C1_SDA, I2C1_SCL);
    static CalibrationData calibration;

    constexpr uint8_t QMC5883_ADDR = 0x0D;
    constexpr uint8_t QMC5883_REG_X_LSB = 0x00;
    constexpr uint8_t QMC5883_REG_STATUS = 0x06;
    constexpr uint8_t QMC5883_REG_CONFIG1 = 0x09;
    constexpr uint8_t QMC5883_REG_CONFIG2 = 0x0A;
    constexpr uint8_t QMC5883_REG_SET_RESET_PERIOD = 0x0B;
    constexpr uint8_t QMC5883_REG_CHIP_ID = 0x0D;

    constexpr uint8_t QMC5883_INIT_TIMEOUT = 100;
    constexpr uint8_t QMC5883_READ_TIMEOUT = 100;

    constexpr uint8_t QMC5883_CAL_ADDR = 0x69;

    void writeRegister(uint8_t reg, uint8_t *val, uint8_t len = 1)
    {
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.write(val, len);
        i2c1.endTransmission();
    }

    bool readRegister(uint8_t reg, uint8_t *data, uint8_t len = 1)
    {
        i2c1.flush();
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.endTransmission();
        i2c1.requestFrom(QMC5883_ADDR, len);

        auto startTime = millis();
        while (millis() - startTime < QMC5883_READ_TIMEOUT)
        {
            if (i2c1.available() < len) // Wait for required bytes
                continue;

            for (uint8_t i = 0; i < len; i++) // Read data
            {
                data[i] = i2c1.read();
            }
            return true;
        }
        return false;
    }

    inline bool dataReady()
    {
        uint8_t status;
        readRegister(QMC5883_REG_STATUS, &status);
        return status & 0x01;
    }

    bool begin(Mode m, DataRate odr, Range rng, OverSample osr)
    {
        i2c1.begin();
        uint8_t chipID;
        readRegister(QMC5883_REG_CHIP_ID, &chipID);
        if (chipID != 0xff) // It's always 0xff by normal
            return false;

        uint8_t cfgData = (osr << 6) | (rng << 4) | (odr << 2) | m;
        writeRegister(QMC5883_REG_CONFIG1, &cfgData);

        getCalibrationData(&calibration); // Load calibration data

        auto startTime = millis();
        uint8_t cfgRead;
        while (millis() - startTime < QMC5883_INIT_TIMEOUT)
        {
            // Check if the configuration is written
            readRegister(QMC5883_REG_CONFIG1, &cfgRead);
            if (cfgRead == cfgData)
            {
                return true;
            }
            delay(10);
        }
        return false;
    }

    bool getRawData(int16_t *xyz)
    {
        if (!dataReady()) // No data ready
            return false;

        uint8_t data[6];
        if (!readRegister(QMC5883_REG_X_LSB, data, 6))
            return false;

        // Combine the bytes into 16-bit signed integers
        for (uint8_t i = 0; i < 3; i++)
        {
            xyz[i] = (data[i * 2 + 1] << 8) | data[i * 2];
        }

        return true;
    }

    bool getRawDataWithCal(int16_t *xyz)
    {
        int16_t data[3];
        if (!getRawData(data))
            return false;

        int16_t offsets[] = {calibration.offsetX, calibration.offsetY, calibration.offsetZ};
        float32_t gains[] = {calibration.gainX, calibration.gainY, calibration.gainZ};

        float32_t temp[3];
        for (uint8_t i = 0; i < 3; i++)
        {
            temp[i] = (data[i] - offsets[i]);
        }
        arm_mult_f32(temp, gains, temp, 3);
        for (uint8_t i = 0; i < 3; i++)
        {
            xyz[i] = temp[i];
        }

        return true;
    }

    bool getGauss(float *xyz)
    {
        int16_t data[3];
        if (!getRawDataWithCal(data))
            return false;

        constexpr float QMC5883_16BIT_SENSITIVITY = 12000.0f;

        // This function will make the data in the range of -1 to 1
        arm_q15_to_float(data, xyz, 3);
        arm_scale_f32(xyz, 32768.0f / QMC5883_16BIT_SENSITIVITY, xyz, 3);

        return true;
    }

    void setCalibrationData(CalibrationData *cal)
    {
        eeprom_buffer_fill();
        for (uint8_t i = 0; i < sizeof(CalibrationData); i++)
        {
            eeprom_buffered_write_byte(QMC5883_CAL_ADDR + i, reinterpret_cast<uint8_t *>(cal)[i]);
        }

        eeprom_buffer_flush();                  // Write the data to EEPROM
        memcpy(&calibration, cal, sizeof(cal)); // Apply the calibration data
    }

    void getCalibrationData(CalibrationData *cal)
    {
        eeprom_buffer_fill();
        // cal->offsetX = eeprom_buffered_read_byte(QMC5883_CAL_ADDR) | (eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 1) << 8);
        // cal->gainX = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 2);

        // cal->offsetY = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 3) | (eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 4) << 8);
        // cal->gainY = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 5);

        // cal->offsetZ = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 6) | (eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 7) << 8);
        // cal->gainZ = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 8);

        for (uint8_t i = 0; i < sizeof(CalibrationData); i++)
        {
            reinterpret_cast<uint8_t *>(cal)[i] = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + i);
        }
    }

    void runCalibration(CalibrationData *cal, TFT_eSPI *tft = nullptr)
    {
        int16_t data[3];
        int16_t min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
        int16_t max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};

        // for (uint16_t i = 0; i < 1000; i++)
        while (1)
        {
            getRawData(data);
            for (uint8_t j = 0; j < 3; j++)
            {
                if (data[j] < min[j])
                    min[j] = data[j];
                if (data[j] > max[j])
                    max[j] = data[j];
            }
            if (tft)
            {
                tft->fillRect(0, 40, 160, 40, TFT_BLACK);
                tft->drawString("Max X: " + String(max[0]) + " Min X: " + String(min[0]), 8, 40);
                tft->drawString("Max Y: " + String(max[1]) + " Min Y: " + String(min[1]), 8, 50);
                tft->drawString("Max Z: " + String(max[2]) + " Min Z: " + String(min[2]), 8, 60);
            }
            if (Serial2.available() && Serial2.read() == 'e')
                break;
            delay(10);
        }

        cal->offsetX = (max[0] + min[0]) / 2;
        cal->offsetY = (max[1] + min[1]) / 2;
        cal->offsetZ = (max[2] + min[2]) / 2;

        cal->gainX = 1.0f;
        cal->gainY = static_cast<float>(max[1] - min[1]) / static_cast<float>(max[0] - min[0]);
        cal->gainZ = static_cast<float>(max[2] - min[2]) / static_cast<float>(max[0] - min[0]);
    }

    float getHeading()
    {
        int16_t xyz[3];
        getRawDataWithCal(xyz);
        float32_t heading;
        heading = atan2(xyz[1], xyz[0]); // This shitty version of CMSIS-DSP doesn't have atan2f
        if (heading < 0)
            heading += 2 * PI;

        arm_scale_f32(&heading, RAD_TO_DEG, &heading, 1);
        return heading;
    }

} // namespace Magneto