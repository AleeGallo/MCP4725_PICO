#ifndef MCP4725_H
#define MCP4725_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>
#include <stdint.h>

#define MCP4725_REFERENCE_VOLTAGE 5.0f
#define MCP4725_STEPS             4096
#define MCP4725_MAX_VALUE         4095
#define MCP4725_ERROR             0xFFFF
#define MCP4725_GENERAL_CALL_ADDRESS 0x00

typedef enum {
    MCP4725_FastMode       = 0x00,
    MCP4725_RegisterMode   = 0x40,
    MCP4725_EEPROM_Mode    = 0x60
} mcp4725_cmd_t;

typedef enum {
    MCP4725_PowerDown_OFF  = 0,
    MCP4725_PowerDown_1K   = 1,
    MCP4725_PowerDown_100K = 2,
    MCP4725_PowerDown_500K = 3
} mcp4725_power_t;

typedef enum {
    MCP4725_ReadSettings = 1,
    MCP4725_ReadDACReg   = 3,
    MCP4725_ReadEEPROM   = 5
} mcp4725_readtype_t;

typedef enum {
    MCP4725_GeneralCallAddress = 0x00,
    MCP4725_GeneralCallReset   = 0x06,
    MCP4725_GeneralCallWakeUp  = 0x09
} mcp4725_generalcall_t;

typedef struct {
    uint8_t  i2c_addr;
    i2c_inst_t *i2c_port;
    uint8_t  sda_pin;
    uint8_t  scl_pin;
    uint16_t clk_speed;
    uint32_t i2c_timeout;
    float    ref_voltage;
    float    bits_per_volt;
    bool     serial_debug;
    bool     safety_check;
    uint16_t eeprom_write_time;
    uint16_t lib_version;
} mcp4725_t;

bool mcp4725_begin(mcp4725_t *dev, uint8_t addr, i2c_inst_t* i2c_port,
                   uint16_t clk_speed, uint8_t sda_pin, uint8_t scl_pin, uint32_t timeout_us);

void mcp4725_deinit_i2c(mcp4725_t *dev);
bool mcp4725_is_connected(mcp4725_t *dev);

void mcp4725_set_reference_voltage(mcp4725_t *dev, float voltage);
float mcp4725_get_reference_voltage(mcp4725_t *dev);

bool mcp4725_set_input_code(mcp4725_t *dev, uint16_t code, mcp4725_cmd_t mode, mcp4725_power_t pwr);
bool mcp4725_set_voltage(mcp4725_t *dev, float voltage, mcp4725_cmd_t mode, mcp4725_power_t pwr);

uint16_t mcp4725_get_input_code(mcp4725_t *dev);
float mcp4725_get_voltage(mcp4725_t *dev);
uint16_t mcp4725_get_stored_input_code(mcp4725_t *dev);
float mcp4725_get_stored_voltage(mcp4725_t *dev);

uint16_t mcp4725_get_power_type(mcp4725_t *dev);
uint16_t mcp4725_get_stored_power_type(mcp4725_t *dev);

bool mcp4725_get_eeprom_busy_flag(mcp4725_t *dev);

bool mcp4725_write_command(mcp4725_t *dev, uint16_t code, mcp4725_cmd_t mode, mcp4725_power_t pwr);
uint16_t mcp4725_read_register(mcp4725_t *dev, mcp4725_readtype_t type);

void mcp4725_set_serial_debug_flag(mcp4725_t *dev, bool onOff);
bool mcp4725_get_serial_debug_flag(mcp4725_t *dev);

void mcp4725_set_safety_check_flag(mcp4725_t *dev, bool onOff);
bool mcp4725_get_safety_check_flag(mcp4725_t *dev);

bool mcp4725_general_call(mcp4725_t *dev, mcp4725_generalcall_t typeCall);

uint16_t mcp4725_get_library_version_number(mcp4725_t *dev);
uint16_t mcp4725_get_eeprom_write_time(mcp4725_t *dev);
void mcp4725_set_eeprom_write_time(mcp4725_t *dev, uint16_t time_ms);

#endif
