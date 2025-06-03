#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "lwip/ip4_addr.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/init.h"
#include "lwip/apps/mdns.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#define TEST_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1UL )

#define INT_PIN 9
#define SDA_PIN 10
#define SCL_PIN 11

#define MOTOR_LEFT_PWM 16
#define MOTOR_LEFT_A 18
#define MOTOR_LEFT_B 17

#define MOTOR_RIGHT_PWM 21
#define MOTOR_RIGHT_A 20
#define MOTOR_RIGHT_B 19

#define MOTOR_PWM_WRAP 5000

union vec3 {
    struct {
        float z;
        float y;
        float x;
    };
    struct {
        float yaw;
        float pitch;
        float roll;
    };
    float d[3];
};

using OrientationPacket = vec3;

struct PIDResponsePacket {
    float P;
    float I;
    float D;
};

struct PIDConfigurationPacket {
    float Kp;
    float Ki;
    float Kd;
};

enum class MotionPacketType : uint32_t {
    INVALID = 0,
    ORIENTATION = 1,
    PID_RESPONSE = 2,
    PID_CONFIGURATION = 3,
};

struct MotionPacket {
    MotionPacketType type;
    union {
        OrientationPacket orientation;
        PIDResponsePacket pid_response;
        PIDConfigurationPacket pid_configuration;
    };
};

void set_motor_left_speed(int16_t speed);
void set_motor_right_speed(int16_t speed);

TaskHandle_t taskIMU = nullptr;

void imu_irq_handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio_get_irq_event_mask(INT_PIN) & GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(INT_PIN, GPIO_IRQ_EDGE_FALL);
        if (taskIMU) {
            vTaskNotifyGiveFromISR(taskIMU, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// PID constants
volatile float Kp = 1.f / 5.f;
volatile float Ki = 0.025f;
volatile float Kd = 1.1f;

void imu_task(void *queue_) {
    QueueHandle_t queue = static_cast<QueueHandle_t>(queue_);

    // initialize I2C1 bus for the MPU6050
    auto rc = i2c_dma_init(&I2Cdev::i2c_dma, i2c1, 400 * 1000, SDA_PIN, SCL_PIN);
    if (rc != PICO_OK) {
        printf("failed to initialize I2C\n");
        vTaskDelete(NULL);
    }

    // mpu initialization
    MPU6050 mpu;
    mpu.initialize();
    if (mpu.dmpInitialize()) {
        printf("failed to initialize MPU6050 DMP\n");
        vTaskDelete(NULL);
    }

    mpu.setXAccelOffset(-387);
    mpu.setYAccelOffset(1135);
    mpu.setZAccelOffset(975);
    mpu.setXGyroOffset(106);
    mpu.setYGyroOffset(18);
    mpu.setZGyroOffset(7);

    mpu.setDMPEnabled(true);
    auto mpuIntStatus = mpu.getIntStatus();
    auto packetSize = mpu.dmpGetFIFOPacketSize();

    // PID state
    float previous_error = NAN;
    float I = 0.f;

    while (true) {
        auto fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) {
            // wait until IMU interrupt has been triggered
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        }

        // check for packets from the IMU
        fifoCount = mpu.getFIFOCount();
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)  {
            mpu.resetFIFO();
            continue;
        } else if (fifoCount < packetSize) {
            continue;
        }

        // read the fifo
        uint8_t fifoBuffer[64];
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // convert to euler angles
        Quaternion q;
        VectorFloat gravity;
        vec3 euler;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(euler.d, &q, &gravity);

        // convert to decimal and send to networking task
        euler.yaw = euler.yaw * 180.f / PI;
        euler.pitch = euler.pitch * 180.f / PI;
        euler.roll = euler.roll * 180.f / PI;

        // compute angle error
        float error = euler.pitch /*- 3.0*/;
        if (isnan(previous_error)) {
            previous_error = error;
        }

        // update balancing PID controller
        const float P = Kp * error;
        I += error;
        const float D = Kd * (error - previous_error);
        previous_error = error;

        float response_PD = P + D;
        float response = response_PD + (Ki * I);

        // backpropagate to solve I-term windup
        if (response > 1.f) {
            if (response_PD > 1.f) {
                I = 0.f;
            } else {
                I = (1.f - response_PD) / Ki;
            }
            response = 1.f;
        } else if (response < -1.f) {
            if (response_PD < -1.f) {
                I = 0.f;
            } else {
                I = (-1.f - response_PD) / Ki;
            }
            response = -1.f;
        }

        // update the motor inputs
        response *= (float) MOTOR_PWM_WRAP;
        set_motor_left_speed((int16_t) response);
        set_motor_right_speed((int16_t) response);

        // send state to network thread
        MotionPacket orientation_packet {
            .type = MotionPacketType::ORIENTATION,
            .orientation = euler
        };
        xQueueSend(queue, (void *) &orientation_packet, 0);

        // send PID packet to network thread
        MotionPacket pid_packet {
            .type = MotionPacketType::PID_RESPONSE,
            .pid_response = {
                .P = P,
                .I = I,
                .D = D
            }
        };
        xQueueSend(queue, (void *) &pid_packet, 0);
    }
}

void configuration_task(__unused void *params) {
    // setup udp receive socket for configuration
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(3000);
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_len = sizeof address;

    auto sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        printf("failed to create a udp socket\n");
        vTaskDelete(nullptr);
    }

    auto res = bind(sock, (struct sockaddr*) &address, sizeof address);
    if (res == -1) {
        printf("failed to bind configuration socket\n");
        close(sock);
        vTaskDelete(nullptr);
    }

    while (true) {
        struct MotionPacket packet;
        res = recv(sock, &packet, sizeof packet, 0);
        if (res < 0) {
            printf("recv encountered an error = %d\n", res);
            continue;
        }

        switch (packet.type) {
            case MotionPacketType::PID_CONFIGURATION:
                Kp = packet.pid_configuration.Kp;
                Ki = packet.pid_configuration.Ki;
                Kd = packet.pid_configuration.Kd;
                break;
            default:
                break;
        }
    }

    vTaskDelete(nullptr);
}

void main_task(__unused void *params) {
    // kick off the mpu thread
    QueueHandle_t queueIMU = xQueueCreate(10, sizeof(MotionPacket));
    xTaskCreate(imu_task, "IMUThread", 8192, (void *) queueIMU, TEST_TASK_PRIORITY, &taskIMU);

    // initialize wifi hardware
    if (cyw43_arch_init()) {
        printf("Failed to initialize Wi-Fi.\n");
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();

    // set the hostname
    const char *hostname = "pico-balancer";
    netif_set_hostname(&cyw43_state.netif[CYW43_ITF_STA], hostname);

    // connect to wifi network
    printf("Connecting to Wi-Fi...\n");
    while (true) {
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK,
                    30000)) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            break;
        }
    }

#if LWIP_MDNS_RESPONDER
    // setup mdns (a.k.a. avahi, bonjour, etc.)
    mdns_resp_init();
    mdns_resp_add_netif(&cyw43_state.netif[CYW43_ITF_STA], hostname);
#endif

    // setup udp socket
    // setup a sockaddr structure for IP 10.0.0.10, port 3000
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(3000);

    ip_addr_t ipaddr;
    if (ipaddr_aton("10.0.0.10", &ipaddr) == 0) {
        printf("failed to convert address to network format\n");
        vTaskDelete(NULL);
    }
    inet_addr_from_ip4addr(&address.sin_addr, &ipaddr);

    auto sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        printf("failed to create a udp socket\n");
        vTaskDelete(NULL);
    }

    auto res = connect(sock, (const struct sockaddr*) &address, sizeof address);
    if (res == -1) {
        printf("failed to connect to udp socket\n");
        close(sock);
        vTaskDelete(NULL);
    }
    printf("\nIP: %s, hostname: %s.local\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), hostname);

    TaskHandle_t taskConfiguration;
    xTaskCreate(configuration_task, "ConfigurationThread", 8192, nullptr, TEST_TASK_PRIORITY,
            &taskConfiguration);

    while (true) {
        // receive next orientation
        MotionPacket packet;
        if (xQueueReceive(queueIMU, (void *) &packet, portMAX_DELAY) != pdPASS) {
            continue;
        }

        // send packet
        res = send(sock, &packet, sizeof packet, 0);
        if (res < 0) {
            printf("send encountered an error = %d\n", res);
        }
    }

    // must delete task to "exit"
#if LWIP_MDNS_RESPONDER
    mdns_resp_remove_netif(&cyw43_state.netif[CYW43_ITF_STA]);
#endif
    cyw43_arch_deinit();
    vTaskDelete(NULL);
}

void setup_imu_interrupt() {
    // subscribe to imu interrupt
    gpio_init(INT_PIN);
    gpio_set_input_enabled(INT_PIN, true);
    gpio_pull_up(INT_PIN);
    gpio_add_raw_irq_handler(INT_PIN, imu_irq_handler);
    gpio_set_irq_enabled(INT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

void setup_motors() {
    // set up the motor GPIOs and PWM
    gpio_init(MOTOR_LEFT_A);
    gpio_set_dir(MOTOR_LEFT_A, GPIO_OUT);
    gpio_put(MOTOR_LEFT_A, false);

    gpio_init(MOTOR_LEFT_B);
    gpio_set_dir(MOTOR_LEFT_B, GPIO_OUT);
    gpio_put(MOTOR_LEFT_B, false);

    gpio_init(MOTOR_RIGHT_A);
    gpio_set_dir(MOTOR_RIGHT_A, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_A, false);

    gpio_init(MOTOR_RIGHT_B);
    gpio_set_dir(MOTOR_RIGHT_B, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_B, false);

    gpio_set_function(MOTOR_LEFT_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM), true);

    gpio_set_function(MOTOR_RIGHT_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM), true);
}

void set_motor_left_speed(int16_t speed) {
    if (speed > 5000) {
        speed = 5000;
    } else if (speed < -5000) {
        speed = -5000;
    }

    if (speed > 0) {
        gpio_put(MOTOR_LEFT_A, false);
        gpio_put(MOTOR_LEFT_B, true);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, speed);
    } else if (speed < 0) {
        gpio_put(MOTOR_LEFT_A, true);
        gpio_put(MOTOR_LEFT_B, false);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, -speed);
    } else {
        gpio_put(MOTOR_LEFT_A, false);
        gpio_put(MOTOR_LEFT_B, false);
    }
}

void set_motor_right_speed(int16_t speed) {
    if (speed > 5000) {
        speed = 5000;
    } else if (speed < -5000) {
        speed = -5000;
    }

    if (speed > 0) {
        gpio_put(MOTOR_RIGHT_A, true);
        gpio_put(MOTOR_RIGHT_B, false);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, speed);
    } else if (speed < 0) {
        gpio_put(MOTOR_RIGHT_A, false);
        gpio_put(MOTOR_RIGHT_B, true);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, -speed);
    } else {
        gpio_put(MOTOR_RIGHT_A, false);
        gpio_put(MOTOR_RIGHT_B, false);
    }
}

int main(void)
{
    stdio_init_all();
    setup_imu_interrupt();
    setup_motors();

    /* start the main thread */
    TaskHandle_t task;
    xTaskCreate(main_task, "MainThread", 8192, NULL, TEST_TASK_PRIORITY, &task);
    vTaskStartScheduler();
    return 0;
}