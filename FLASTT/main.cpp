#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#define MOTOR_PWM_PIN 2
#define MOTOR_FORWARDS_PIN 3
#define MOTOR_REVERSE_PIN 4

class AS5600
{
private:

    const int ZPOS = 0x01;
    const int AGC = 0x1A;
    const int RAWANGLE = 0x0C;
    const int ANGLE = 0x0E;
    const int STATUS = 0x00;

    i2c_inst_t* _i2c_inst;
    uint8_t _device_address;

    int read_blocking(uint8_t* destination, uint8_t register_address, size_t length, bool nostop)
    {
        ::i2c_write_blocking(this->_i2c_inst, this->_device_address, &register_address, 1, true);
        return ::i2c_read_blocking(this->_i2c_inst, this->_device_address, destination, length, nostop);
    }

    uint16_t ToUint16(uint8_t value[2])
    {
        return (static_cast<uint16_t>(value[1]) << 8) | value[0];
    }

public:

    AS5600(i2c_inst_t* i2c_inst, uint8_t device_address)
    {
        this->_i2c_inst = i2c_inst;
        this->_device_address = device_address;
    };

    int get_raw_angle()
    {
        uint8_t data[2];
        read_blocking(data, RAWANGLE+1, 2, false);
        return ToUint16(data);
    };
};

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    pwm_set_clkdiv(slice_num, 4.0f);
    pwm_set_wrap(slice_num, 255);
    pwm_set_enabled(slice_num, true);

    gpio_init(MOTOR_FORWARDS_PIN);
    gpio_set_dir(MOTOR_FORWARDS_PIN, GPIO_OUT);

    gpio_init(MOTOR_REVERSE_PIN);
    gpio_set_dir(MOTOR_REVERSE_PIN, GPIO_OUT);

    const int as5600_address = 0x36;
    AS5600 as5600 = AS5600(I2C_PORT, as5600_address);

    // move the motor
    gpio_put(MOTOR_FORWARDS_PIN, 1);
    gpio_put(MOTOR_REVERSE_PIN, 0);
    pwm_set_gpio_level(MOTOR_PWM_PIN, 96);


    int raw_angle;
    while (true) {
        sleep_ms(100);
        
        raw_angle = as5600.get_raw_angle();
        printf("raw_angle, %d\n", raw_angle);
    }
};
