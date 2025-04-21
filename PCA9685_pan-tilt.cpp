
// 用PCA9685手动控制pan-tilt云台
// 启用了通道0 & 通道1
// PCA9685用i2c通信

// 接口示意
// pi5	PCA9685
// SCL	    SCL
// SDA 	    SDA
// 3.3V	    VCC
// 5V	    V+
// GND  	GND

// 舵机	PCA9685
// Pan 	PWM0
// Tilt	PWM1



#include <iostream>
#include <fcntl.h>      // open()
#include <unistd.h>     // close(), usleep(), read()
#include <sys/ioctl.h>  // ioctl()
#include <linux/i2c-dev.h>  // I2C 定义
#include <cstring>      // memset()

// PCA9685 的 I2C 默认地址
#define PCA9685_ADDR 0x40

// PCA9685 的寄存器地址
#define MODE1_REG     0x00
#define PRESCALE_REG  0xFE
#define LED0_ON_L     0x06

// PWM 控制范围（你可以根据实际舵机调整）
#define SERVO_MIN 150   // 对应最小角度（约 0°）
#define SERVO_MAX 600   // 对应最大角度（约 180°）

// 初始化 PCA9685 设置为 50Hz
void initPCA9685(int file) {
    uint8_t sleep_cmd[2] = {MODE1_REG, 0x10}; // 设置 sleep 模式
    write(file, sleep_cmd, 2);

    uint8_t prescale_cmd[2] = {PRESCALE_REG, 121}; // 50Hz 对应 prescale = 121
    write(file, prescale_cmd, 2);

    uint8_t wake_cmd[2] = {MODE1_REG, 0x20}; // 清除 sleep，设置 auto-increment
    write(file, wake_cmd, 2);

    usleep(500); // 稍作延迟
}

// 设置舵机角度
void setServoAngle(int file, int channel, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 映射角度为 PWM 值
    int pwm_value = static_cast<int>(SERVO_MIN + (angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
    int on = 0;
    int off = pwm_value;

    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buffer[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>((on >> 8) & 0xFF),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>((off >> 8) & 0xFF)
    };

    write(file, buffer, 5);
}

void disableChannel(int file, int channel) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t buffer[5] = {
        reg,
        0x00, 0x00,   // ON = 0
        0x00, 0x10    // OFF full ON bit（bit 4 of OFF_H = 1） = 完全关闭输出
    };
    write(file, buffer, 5);
}


int main() {
    const char *i2c_device = "/dev/i2c-1";  // 树莓派默认的 I2C 接口
    int file;

    // 打开 I2C 设备
    if ((file = open(i2c_device, O_RDWR)) < 0) {
        std::cerr << "无法打开 I2C 设备 " << i2c_device << std::endl;
        return 1;
    }

    // 设置 PCA9685 地址
    if (ioctl(file, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "无法设置 I2C 地址 " << std::endl;
        close(file);
        return 1;
    }

    // 初始化 PWM 控制器
    initPCA9685(file);

    std::cout << "实时舵机控制启动，输入通道号(0~15) 和角度(0~180)，输入 -1 退出。" << std::endl;

    int channel;
    float angle;

    // 主控制循环
    while (true) {
        std::cout << "\n请输入通道号 (0~15)，输入 -1 退出: ";
        std::cin >> channel;
        if (channel == -1) break;
        if (channel < 0 || channel > 15) {
            std::cerr << "无效通道号，请输入 0~15" << std::endl;
            continue;
        }

        std::cout << "请输入角度 (0~180): ";
        std::cin >> angle;
        if (angle < 0 || angle > 180) {
            std::cerr << "无效角度，请输入 0~180" << std::endl;
            continue;
        }

        setServoAngle(file, channel, angle);
        std::cout << "✅ 通道 " << channel << " 已设置为 " << angle << "°" << std::endl;
    }

    std::cout << "程序结束，关闭设备。" << std::endl;
    // 关闭通道 0
    disableChannel(file, 0);
    // 关闭通道 1
    disableChannel(file, 1);

    close(file);
    return 0;
}
