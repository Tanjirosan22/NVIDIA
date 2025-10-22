#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define INA219_ADDRESS 0x40

// INA219 Register addresses
#define REG_CONFIG        0x00
#define REG_SHUNT_VOLTAGE 0x01
#define REG_BUS_VOLTAGE   0x02
#define REG_POWER         0x03
#define REG_CURRENT       0x04
#define REG_CALIBRATION   0x05

int i2c_fd;

// Write 16-bit value to register
void writeRegister(int reg, uint16_t value) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = value & 0xFF;
    write(i2c_fd, buffer, 3);
}

// Read 16-bit value from register
uint16_t readRegister(int reg) {
    uint8_t buffer[2];
    write(i2c_fd, &reg, 1);
    read(i2c_fd, buffer, 2);
    return (buffer[0] << 8) | buffer[1];
}

int main() {
    const char *i2cDevice = "/dev/i2c-1";
    if ((i2c_fd = open(i2cDevice, O_RDWR)) < 0) {
        perror("Failed to open I2C device");
        exit(1);
    }

    if (ioctl(i2c_fd, I2C_SLAVE, INA219_ADDRESS) < 0) {
        perror("Failed to set I2C address");
        exit(1);
    }

    // Calibration value (depends on your shunt resistor and expected current range)
    uint16_t calibrationValue = 4096;  
    writeRegister(REG_CALIBRATION, calibrationValue);

    printf("Reading INA219 sensor data...\n");

    while (1) {
        uint16_t rawBusVoltage = readRegister(REG_BUS_VOLTAGE);
        uint16_t rawShuntVoltage = readRegister(REG_SHUNT_VOLTAGE);
        uint16_t rawCurrent = readRegister(REG_CURRENT);
        uint16_t rawPower = readRegister(REG_POWER);

        // Calculate actual values
        float shuntVoltage_mV = (int16_t)rawShuntVoltage * 0.01;     // 10uV per bit
        float busVoltage_V = ((rawBusVoltage >> 3) * 4) * 0.001;     // 4mV per bit
        float current_mA = (int16_t)rawCurrent * 0.1;                // depends on calibration
        float power_mW = rawPower * 2.0;                             // depends on calibration

        printf("Bus Voltage: %.3f V | Shunt Voltage: %.3f mV | Current: %.3f mA | Power: %.3f mW\n",
               busVoltage_V, shuntVoltage_mV, current_mA, power_mW);

        usleep(1000000); // 1 second delay
    }

    close(i2c_fd);
    return 0;
}
