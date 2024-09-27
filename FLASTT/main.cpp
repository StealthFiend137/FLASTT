#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/time.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#define MOTOR_PWM_PIN 2
#define MOTOR_FORWARDS_PIN 3
#define MOTOR_REVERSE_PIN 4

class AS5600
{
private:

    const int SENSOR_RESOLUTION = 4096;

    const int ZPOS = 0x01;
    const int AGC = 0x1A;
    const int RAWANGLE = 0x0C;
    const int ANGLE = 0x0E;
    const int STATUS = 0x00;

    uint16_t previous_raw_angle = 0;
    uint32_t previous_ms_since_boot = 0;

    i2c_inst_t* _i2c_inst;
    uint8_t _device_address;

    uint16_t ToUint16(uint8_t value[2])
    {
        return (static_cast<uint16_t>(value[0]) << 8) | value[1];
    }

    float ToRadians(uint16_t value)
    {
        float radians_per_unit = M_TWOPI / SENSOR_RESOLUTION;
        return value * radians_per_unit;
    }

    int read_blocking(uint8_t* destination, uint8_t register_address, size_t length, bool nostop)
    {
        ::i2c_write_blocking(this->_i2c_inst, this->_device_address, &register_address, 1, true);
        return ::i2c_read_blocking(this->_i2c_inst, this->_device_address, destination, length, nostop);
    }

    float get_angular_velocity(int currentPosition, int previousPosition, float deltaTime_s)
    {
        float thetaCurrent = ToRadians(currentPosition);
        float thetaPrevious = ToRadians(previousPosition);

        float deltaTheta = thetaCurrent = thetaPrevious;
        if(deltaTheta > M_PI) deltaTheta -= M_TWOPI;
        if(deltaTheta <  -M_PI) deltaTheta += M_TWOPI;


        return deltaTheta / deltaTime_s;
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
        read_blocking(data, RAWANGLE, 2, false);
        return ToUint16(data);
    };

    float get_radians()
    {
        int raw_angle = this->get_raw_angle();
        return ToRadians(raw_angle);
    }
    
    int uncorrected_angle()
    {
        const float degrees_per_unit = 0.08789;
        uint16_t raw_angle = this->get_raw_angle();
        return (raw_angle * degrees_per_unit);
    };

    float speed()
    {
        int raw_angle = this->get_raw_angle();
        uint32_t ms_since_boot = to_ms_since_boot(get_absolute_time());
        
        float deltaTime = (float)(ms_since_boot - previous_ms_since_boot) / 1000;
        printf("Raw Angle %d, Delta Time %.3f\n", raw_angle, deltaTime);

        float angular_velocity = this->get_angular_velocity(raw_angle, previous_raw_angle, deltaTime);
        
        previous_raw_angle = raw_angle;
        previous_ms_since_boot = ms_since_boot;

        return angular_velocity;
    }
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
        sleep_ms(200);
        
        uint16_t uncorrected_angle = as5600.uncorrected_angle();
        uint16_t raw_angle = as5600.get_raw_angle();
        printf("raw angle, %d, %d\n", uncorrected_angle, raw_angle);

        //float_t radians = as5600.get_radians();
        //printf("radians %9.6f\n", radians);

        //float angular_velocity = as5600.speed();

        //printf("speed %.6f rad/s\n", angular_velocity);
    }
};
