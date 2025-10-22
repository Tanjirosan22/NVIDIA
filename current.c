/**
 * @file ina219_reader.c
 * @brief INA219 I2C Power Monitor driver example for Raspberry Pi
 *
 * This program initializes the INA219 sensor over I2C and continuously
 * reads bus voltage, shunt voltage, current, and power values.
 *
 * Author: Your Name
 * Date  : YYYY-MM-DD
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>   // for uint8_t, uint16_t types

/* INA219 I2C Address */
#define INA219_ADDR              0x40

/* INA219 Register Addresses */
#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

/* INA219 Configuration Constants */
#define INA219_CONFIG_RESET          0x8000
#define INA219_CONFIG_BVOLTAGERANGE  0x2000
#define INA219_CONFIG_GAIN           0x1800
#define INA219_CONFIG_BADCRES        0x0780
#define INA219_CONFIG_SADCRES        0x0080
#define INA219_CONFIG_MODE           0x0078

/* Calibration Parameters */
#define SHUNT_RESISTOR   0.1f
#define CALIBRATION_VALUE 4096
#define CURRENT_LSB      (3.0f / 32768.0f)
#define POWER_LSB        (20.0f * CURRENT_LSB)

static int file;  // I2C file descriptor

/**
 * @brief Initialize the INA219 sensor over I2C
 *
 * Opens the I2C device (/dev/i2c-1), sets the slave address,
 * and configures INA219 registers.
 */
void initINA219(void)
{
    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, INA219_ADDR) < 0) {
        perror("Failed to access I2C slave address");
        exit(1);
    }

    /* Reset INA219 */
    unsigned char resetConfig[3] = {
        INA219_REG_CONFIG,
        (INA219_CONFIG_RESET >> 8) & 0xFF,
        INA219_CONFIG_RESET & 0xFF
    };

    if (write(file, resetConfig, 3) != 3) {
        perror("Failed to write reset command");
    }

    usleep(500000);  // 500ms delay after reset

    /* Configure INA219: voltage range, gain, resolution, and mode */
    unsigned int cfgValue = INA219_CONFIG_BVOLTAGERANGE |
                            INA219_CONFIG_GAIN |
                            INA219_CONFIG_BADCRES |
                            INA219_CONFIG_SADCRES |
                            INA219_CONFIG_MODE;

    unsigned char config[3] = {
        INA219_REG_CONFIG,
        (cfgValue >> 8) & 0xFF,
        cfgValue & 0xFF
    };

    if (write(file, config, 3) != 3) {
        perror("Failed to write configuration");
    }

    /* Write calibration value */
    unsigned char calib[3] = {
        INA219_REG_CALIBRATION,
        (CALIBRATION_VALUE >> 8) & 0xFF,
        CALIBRATION_VALUE & 0xFF
    };

    if (write(file, calib, 3) != 3) {
        perror("Failed to write calibration");
    }
}

/**
 * @brief Read 16-bit value from INA219 register
 *
 * @param reg Register address
 * @return int 16-bit register value or -1 on error
 */
int readRegister(uint8_t reg)
{
    if (write(file, &reg, 1) != 1) {
        perror("Failed to set register address");
        return -1;
    }

    unsigned char data[2];
    if (read(file, data, 2) != 2) {
        perror("Failed to read register");
        return -1;
    }

    return (data[0] << 8) | data[1];
}

/**
 * @brief Read bus voltage in volts
 *
 * @return float Bus voltage (V)
 */
float readBusVoltage(void)
{
    int value = readRegister(INA219_REG_BUS_VOLTAGE);
    return ((value >> 3) * 0.004f);  // 4mV per bit
}

/**
 * @brief Read shunt voltage in volts
 *
 * @return float Shunt voltage (V)
 */
float readShuntVoltage(void)
{
    int value = readRegister(INA219_REG_SHUNT_VOLTAGE);
    return (value * 0.00001f);  // 10uV per bit
}

/**
 * @brief Read current in amperes
 *
 * @return float Current (A)
 */
float readCurrent(void)
{
    int rawCurrent = readRegister(INA219_REG_CURRENT);
    return (rawCurrent * CURRENT_LSB);
}

/**
 * @brief Read power in watts
 *
 * @return float Power (W)
 */
float readPower(void)
{
    int rawPower = readRegister(INA219_REG_POWER);
    return (rawPower * POWER_LSB);
}

/**
 * @brief Main function: initialize INA219 and print readings
 */
int main(void)
{
    printf("Initializing INA219...\n");
    initINA219();
    printf("INA219 initialized successfully!\n");

    while (1) {
        float busVoltage   = readBusVoltage();
        float shuntVoltage = readShuntVoltage();
        float current      = readCurrent();
        float power        = readPower();

        printf("\n============================\n");
        printf("Bus Voltage   : %.3f V\n", busVoltage);
        printf("Shunt Voltage : %.3f mV\n", shuntVoltage * 1000);
        printf("Current       : %.3f A\n", current);
        printf("Power         : %.3f W\n", power);

        sleep(2);
    }

    close(file);
    return 0;
}

