#include <EEPROM.h>

#include "CMSIS_DSP.h"
#include "Magnetometer.hpp"
#include "config.h"

namespace Magneto {
    static TwoWire i2c1(I2C1_SDA, I2C1_SCL);
    static int16_t calibration[3]{0};

    constexpr uint8_t QMC5883_ADDR = 0x0D;
    constexpr uint8_t QMC5883_REG_X_LSB = 0x00;
    constexpr uint8_t QMC5883_REG_X_MSB = 0x01;
    constexpr uint8_t QMC5883_REG_Y_LSB = 0x02;
    constexpr uint8_t QMC5883_REG_Y_MSB = 0x03;
    constexpr uint8_t QMC5883_REG_Z_LSB = 0x04;
    constexpr uint8_t QMC5883_REG_Z_MSB = 0x05;
    constexpr uint8_t QMC5883_REG_STATUS = 0x06;
    constexpr uint8_t QMC5883_REG_CONFIG1 = 0x09;
    constexpr uint8_t QMC5883_REG_CONFIG2 = 0x0A;
    constexpr uint8_t QMC5883_REG_SET_RESET_PERIOD = 0x0B;
    constexpr uint8_t QMC5883_REG_CHIP_ID = 0x0D;

    constexpr uint8_t QMC5883_INIT_TIMEOUT = 100;

    constexpr uint8_t QMC5883_CAL_ADDR = 0x69; // Offset: x->0, y->2, z->4

    void writeRegister(uint8_t reg, uint8_t *val, uint8_t len = 1) {
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.write(val, len);
        i2c1.endTransmission();
    }

    size_t readRegister(uint8_t reg, uint8_t *data, uint8_t len = 1) {
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.endTransmission();

        i2c1.requestFrom(QMC5883_ADDR, len);
        size_t i = 0;
        while (i2c1.available() && i < len) {
            data[i++] = i2c1.read();
        }
        i2c1.flush();
        return i;
    }

    inline bool dataReady() {
        uint8_t status;
        readRegister(QMC5883_REG_STATUS, &status);
        return status & 0x01;
    }

    bool begin(Mode m, DataRate odr, Range rng, OverSample osr) {
        i2c1.begin();
        uint8_t chipID;
        readRegister(QMC5883_REG_CHIP_ID, &chipID);
        if (chipID != 0xFF)
            return false;

        uint8_t cfgData = (osr << 6) | (rng << 4) | (odr << 2) | m;
        writeRegister(QMC5883_REG_CONFIG1, &cfgData);

        auto startTime = millis();
        uint8_t cfgRead;

        getCalibrationData(calibration);

        while (millis() - startTime < QMC5883_INIT_TIMEOUT) {
            readRegister(QMC5883_REG_CONFIG1, &cfgRead);
            if (cfgRead == cfgData) {
                return true;
            }
            delay(10);
        }
        return false;
    }

    bool getRawData(int16_t *xyz) {
        if (!dataReady()) {
            return false;
        }

        uint8_t data[6];
        if (readRegister(QMC5883_REG_X_LSB, data, 6) != 6)
            return false;

        for (uint8_t i = 0; i < 3; i++) {
            xyz[i] = static_cast<int16_t>(data[i * 2 + 1] << 8) | data[i * 2];
        }

        return true;
    }

    bool getRawDataWithCal(int16_t *xyz) {
        int16_t data[3];
        if (!getRawData(data))
            return false;
        for (uint8_t i = 0; i < 3; i++) {
            xyz[i] = data[i] - calibration[i];
        }
        return true;
    }

    bool getGauss(float *xyz) {
        int16_t data[3];
        if (!getRawDataWithCal(data))
            return false;

        constexpr float QMC5883_16BIT_SENSITIVITY = 12000.0f;

        // This function will make the data in the range of -1 to 1
        arm_q15_to_float(data, xyz, 3);
        arm_scale_f32(xyz, 32768.0f / QMC5883_16BIT_SENSITIVITY, xyz, 3);

        return true;
    }

    void setCalibrationData(int16_t *xyz) {
        eeprom_buffer_fill();
        eeprom_buffered_write_byte(QMC5883_CAL_ADDR, xyz[0] & 0xFF);
        eeprom_buffered_write_byte(QMC5883_CAL_ADDR + 1, xyz[0] >> 8);
        eeprom_buffered_write_byte(QMC5883_CAL_ADDR + 2, xyz[1] & 0xFF);
        eeprom_buffered_write_byte(QMC5883_CAL_ADDR + 3, xyz[1] >> 8);
        eeprom_buffered_write_byte(QMC5883_CAL_ADDR + 4, xyz[2] & 0xFF);
        eeprom_buffered_write_byte(QMC5883_CAL_ADDR + 5, xyz[2] >> 8);
        eeprom_buffer_flush();
        memcpy(calibration, xyz, sizeof(calibration));
    }

    void getCalibrationData(int16_t *xyz) {
        eeprom_buffer_fill();
        for (uint8_t i = 0; i < 6; i++) {
            xyz[i] = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 2 * i + 1)
                         << 8 |
                     eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 2 * i);
        }
    }

    void runCalibration(int16_t *offsetXYZ) {
        int16_t data[3];
        int16_t min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
        int16_t max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};

        for (int i = 0; i < 1000; i++) {
            getRawData(data);
            for (int j = 0; j < 3; j++) {
                if (data[j] < min[j])
                    min[j] = data[j];
                if (data[j] > max[j])
                    max[j] = data[j];

                delay(5);
            }
        }

        for (int i = 0; i < 3; i++) {
            offsetXYZ[i] = (max[i] + min[i]) / 2;
        }
    }

} // namespace Magneto