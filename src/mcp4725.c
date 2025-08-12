
#include "mcp4725.h"
#include <stdio.h>
#include <math.h>

bool mcp4725_begin(mcp4725_t *dev, uint8_t addr, i2c_inst_t* i2c_port,
                   uint16_t clk_speed, uint8_t sda_pin, uint8_t scl_pin, uint32_t timeout_us) 
{
    dev->i2c_addr     = addr;
    dev->i2c_port     = i2c_port;
    dev->clk_speed    = clk_speed;
    dev->sda_pin      = sda_pin;
    dev->scl_pin      = scl_pin;
    dev->i2c_timeout  = timeout_us;
    dev->serial_debug = false;
    dev->safety_check = true;
    dev->eeprom_write_time = 25;
    dev->lib_version = 100; // ejemplo

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    i2c_init(i2c_port, clk_speed * 1000);
    sleep_ms(50);

    return mcp4725_is_connected(dev);
}

void mcp4725_deinit_i2c(mcp4725_t *dev) {
    gpio_set_function(dev->sda_pin, GPIO_FUNC_NULL);
    gpio_set_function(dev->scl_pin, GPIO_FUNC_NULL);
    i2c_deinit(dev->i2c_port);
}

bool mcp4725_is_connected(mcp4725_t *dev) {
    uint8_t rx = 0;
    int ret = i2c_read_timeout_us(dev->i2c_port, dev->i2c_addr, &rx, 1, false, dev->i2c_timeout);
    return (ret > 0);
}

void mcp4725_set_reference_voltage(mcp4725_t *dev, float voltage) {
    if (voltage <= 0)
        dev->ref_voltage = MCP4725_REFERENCE_VOLTAGE;
    else
        dev->ref_voltage = voltage;
    dev->bits_per_volt = (float)MCP4725_STEPS / dev->ref_voltage;
}

float mcp4725_get_reference_voltage(mcp4725_t *dev) {
    return dev->ref_voltage;
}

bool mcp4725_set_input_code(mcp4725_t *dev, uint16_t code, mcp4725_cmd_t mode, mcp4725_power_t pwr) {
    if (dev->safety_check && code > MCP4725_MAX_VALUE)
        code = MCP4725_MAX_VALUE;
    return mcp4725_write_command(dev, code, mode, pwr);
}

bool mcp4725_set_voltage(mcp4725_t *dev, float voltage, mcp4725_cmd_t mode, mcp4725_power_t pwr) {
    uint16_t code;
    if (dev->safety_check) {
        if (voltage >= dev->ref_voltage) code = MCP4725_MAX_VALUE;
        else if (voltage <= 0) code = 0;
        else code = (uint16_t)(voltage * dev->bits_per_volt);
    } else {
        code = (uint16_t)(voltage * dev->bits_per_volt);
    }
    return mcp4725_write_command(dev, code, mode, pwr);
}

uint16_t mcp4725_get_input_code(mcp4725_t *dev) {
    uint16_t val = mcp4725_read_register(dev, MCP4725_ReadDACReg);
    return (val != MCP4725_ERROR) ? (val >> 4) : val;
}

float mcp4725_get_voltage(mcp4725_t *dev) {
    uint16_t code = mcp4725_get_input_code(dev);
    if (code == MCP4725_ERROR) return NAN;
    return code / dev->bits_per_volt;
}

uint16_t mcp4725_get_stored_input_code(mcp4725_t *dev) {
    uint16_t val = mcp4725_read_register(dev, MCP4725_ReadEEPROM);
    return (val != MCP4725_ERROR) ? (val & 0x0FFF) : val;
}

float mcp4725_get_stored_voltage(mcp4725_t *dev) {
    uint16_t code = mcp4725_get_stored_input_code(dev);
    if (code == MCP4725_ERROR) return NAN;
    return code / dev->bits_per_volt;
}

uint16_t mcp4725_get_power_type(mcp4725_t *dev) {
    uint16_t val = mcp4725_read_register(dev, MCP4725_ReadSettings);
    return (val != MCP4725_ERROR) ? ((val & 0x0006) >> 1) : val;
}

uint16_t mcp4725_get_stored_power_type(mcp4725_t *dev) {
    uint16_t val = mcp4725_read_register(dev, MCP4725_ReadEEPROM);
    return (val != MCP4725_ERROR) ? ((val << 1) >> 14) : val;
}

bool mcp4725_get_eeprom_busy_flag(mcp4725_t *dev) {
    uint16_t val = mcp4725_read_register(dev, MCP4725_ReadSettings);
    return (val != MCP4725_ERROR) ? (((val >> 7) & 0x01) == 1) : false;
}

bool mcp4725_write_command(mcp4725_t *dev, uint16_t code, mcp4725_cmd_t mode, mcp4725_power_t pwr) {
    uint8_t buf[3];
    int ret = 0;

    if (mode == MCP4725_FastMode) {
        buf[0] = mode | (pwr << 4) | ((code >> 8) & 0x0F);
        buf[1] = code & 0xFF;
        ret = i2c_write_timeout_us(dev->i2c_port, dev->i2c_addr, buf, 2, false, dev->i2c_timeout);
    } else {
        uint16_t shifted = code << 4;
        buf[0] = mode | (pwr << 1);
        buf[1] = (shifted >> 8) & 0xFF;
        buf[2] = shifted & 0xFF;
        ret = i2c_write_timeout_us(dev->i2c_port, dev->i2c_addr, buf, 3, false, dev->i2c_timeout);
    }

    if (ret < 1) return false;
    if (mode == MCP4725_EEPROM_Mode) {
        sleep_ms(dev->eeprom_write_time);
    }
    return true;
}

uint16_t mcp4725_read_register(mcp4725_t *dev, mcp4725_readtype_t type) {
    uint8_t buf[6];
    uint16_t val = MCP4725_ERROR;
    int ret = 0;

    switch (type) {
        case MCP4725_ReadSettings:
            ret = i2c_read_timeout_us(dev->i2c_port, dev->i2c_addr, buf, 1, false, dev->i2c_timeout);
            if (ret > 0) val = buf[0];
            break;
        case MCP4725_ReadDACReg:
            ret = i2c_read_timeout_us(dev->i2c_port, dev->i2c_addr, buf, 3, false, dev->i2c_timeout);
            if (ret > 0) val = (buf[1] << 8) | buf[2];
            break;
        case MCP4725_ReadEEPROM:
            ret = i2c_read_timeout_us(dev->i2c_port, dev->i2c_addr, buf, 5, false, dev->i2c_timeout);
            if (ret > 0) val = (buf[3] << 8) | buf[4];
            break;
    }
    return (ret > 0) ? val : MCP4725_ERROR;
}

void mcp4725_set_serial_debug_flag(mcp4725_t *dev, bool onOff) { dev->serial_debug = onOff; }
bool mcp4725_get_serial_debug_flag(mcp4725_t *dev) { return dev->serial_debug; }

void mcp4725_set_safety_check_flag(mcp4725_t *dev, bool onOff) { dev->safety_check = onOff; }
bool mcp4725_get_safety_check_flag(mcp4725_t *dev) { return dev->safety_check; }

bool mcp4725_general_call(mcp4725_t *dev, mcp4725_generalcall_t typeCall) {
    if (typeCall == MCP4725_GeneralCallAddress) return false;
    uint8_t data = (uint8_t)typeCall;
    int ret = i2c_write_timeout_us(dev->i2c_port, MCP4725_GENERAL_CALL_ADDRESS, &data, 1, false, dev->i2c_timeout);
    return (ret > 0);
}

uint16_t mcp4725_get_library_version_number(mcp4725_t *dev) { return dev->lib_version; }
uint16_t mcp4725_get_eeprom_write_time(mcp4725_t *dev) { return dev->eeprom_write_time; }
void mcp4725_set_eeprom_write_time(mcp4725_t *dev, uint16_t time_ms) { dev->eeprom_write_time = time_ms; }

