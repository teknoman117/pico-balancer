#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
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

#include "motors.hpp"
#include "packets.hpp"
#include "pid.hpp"
#include "utils.hpp"

#define TEST_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1UL )

#define INT_PIN 9
#define SDA_PIN 10
#define SCL_PIN 11

// Tilt PID Controller
constexpr float Kp = 0.100f;
constexpr float Ki = 1.000f;
constexpr float Kd = 0.010f;
PositionalPID tilt_pid(Kp, Ki, Kd);

// Velocity PID Controller
constexpr float velocity_Ka = 0.025f;
constexpr float velocity_Kp = 0.0013f;
constexpr float velocity_Ki = 0.0f;
constexpr float velocity_Kd = 0.00003f;
PositionalPID velocity_pid(velocity_Kp, velocity_Ki, velocity_Kd);

// Yaw PID Controller
constexpr float yaw_Kp = 0.02f;
constexpr float yaw_Ki = 0.001f;
constexpr float yaw_Kd = 0.001f;
volatile float yaw_rate = 0.f;
PositionalPID yaw_pid(yaw_Kp, yaw_Ki, yaw_Kd);

// Shared RTOS Objects
SemaphoreHandle_t input_mutex = nullptr;
QueueHandle_t packet_queue = nullptr;
TaskHandle_t imu_task_handle = nullptr;

// Handle IMU data
void imu_task(__unused void *params) {
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

    float velocity = 0.f;
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
        uint32_t timestamp = xTaskGetTickCount();
        const float timestamp_delta = 0.01f;

        // convert to euler angles (in degrees)
        Quaternion q;
        VectorFloat gravity;
        vec3 euler;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(euler.d, &q, &gravity);

        euler.yaw = euler.yaw * 180.f / PI;
        euler.pitch = euler.pitch * 180.f / PI;
        euler.roll = euler.roll * 180.f / PI;

        // if we've fallen over, turn off the motors
        if (fabs(euler.pitch) >= 30.f) {
            set_motor_left_speed(0);
            set_motor_right_speed(0);
            continue;
        }

        // compute tilt angle to achieve target velocity
        ScopedLock lock(input_mutex);
        float velocity_P, velocity_I, velocity_D;
        const float velocity_current = (float) get_motor_velocity() / timestamp_delta;
        velocity = (velocity_Ka * velocity_current) + (1.f - velocity_Ka) * velocity;
        const float control = velocity_pid.compute(velocity, timestamp_delta,
                &velocity_P, &velocity_I, &velocity_D);

        // compute motor response to achieve target tilt angle
        tilt_pid.set_setpoint(-3.f * control);
        float tilt_P, tilt_I, tilt_D;
        const float response = tilt_pid.compute(euler.pitch, timestamp_delta,
                &tilt_P, &tilt_I, &tilt_D);

        // compute motor response to achieve target yaw angle
        const float yaw_target = yaw_pid.get_setpoint() + yaw_rate * timestamp_delta;
        yaw_pid.set_setpoint(normalize_heading_degrees(yaw_target));
        float yaw_P, yaw_I, yaw_D;
        const float yaw_response = yaw_pid.compute(euler.yaw, timestamp_delta,
                &yaw_P, &yaw_I, &yaw_D,
                [] (float measured, float target) -> float {
            return normalize_heading_degrees(measured - target);
        });

        // mix tilt and yaw responses
        set_motor_left_speed(response + 0.33f * yaw_response);
        set_motor_right_speed(response - 0.33f * yaw_response);

        // send state to network thread
        MotionPacket state_packet {
            .type = MotionPacketType::STATE,
            .state = {
                .timestamp = timestamp,
                .orientation = euler,
                .orientation_target = tilt_pid.get_setpoint(),
                .P = tilt_P,
                .I = tilt_I,
                .D = tilt_D,
                .velocity = velocity,
                .velocity_target = velocity_pid.get_setpoint(),
                .vP = velocity_P,
                .vI = velocity_I,
                .vD = velocity_D,
                .yaw_target = yaw_pid.get_setpoint(),
                .yP = yaw_P,
                .yI = yaw_I,
                .yD = yaw_D
            }
        };
        xQueueSend(packet_queue, (void *) &state_packet, 0);
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

        ScopedLock lock(input_mutex);
        switch (packet.type) {
            case MotionPacketType::PID_CONFIGURATION_TILT:
                tilt_pid.set_constants(packet.pid_configuration.Kp, packet.pid_configuration.Ki,
                        packet.pid_configuration.Kd);
                break;
            case MotionPacketType::PID_CONFIGURATION_VELOCITY:
                velocity_pid.set_constants(packet.pid_configuration.Kp, packet.pid_configuration.Ki,
                        packet.pid_configuration.Kd);
                break;
            case MotionPacketType::PID_CONFIGURATION_YAW:
                yaw_pid.set_constants(packet.pid_configuration.Kp, packet.pid_configuration.Ki,
                        packet.pid_configuration.Kd);
                break;
            case MotionPacketType::CONTROL:
                velocity_pid.set_setpoint(packet.control.y * 1920.f);
                yaw_rate = packet.control.x * 180.f;
                break;
            default:
                break;
        }
    }

    vTaskDelete(nullptr);
}

void main_task(__unused void *params) {
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
        if (xQueueReceive(packet_queue, (void *) &packet, portMAX_DELAY) != pdPASS) {
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
    gpio_init(INT_PIN);
    gpio_set_input_enabled(INT_PIN, true);
    gpio_pull_up(INT_PIN);

    // subscribe to imu interrupt
    gpio_add_raw_irq_handler(INT_PIN, [] () {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (gpio_get_irq_event_mask(INT_PIN) & GPIO_IRQ_EDGE_FALL) {
            gpio_acknowledge_irq(INT_PIN, GPIO_IRQ_EDGE_FALL);
            vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    });
    gpio_set_irq_enabled(INT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

int main(void)
{
    stdio_init_all();

    /* allocate RTOS objects */
    packet_queue = xQueueCreate(10, sizeof(MotionPacket));
    input_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(input_mutex);

    /* setup hardware */
    setup_imu_interrupt();
    setup_encoders();
    setup_motors();

    /* start threads */
    TaskHandle_t main_task_handle;
    xTaskCreate(main_task, "MainThread", 8192, nullptr, TEST_TASK_PRIORITY, &main_task_handle);
    xTaskCreate(imu_task, "IMUThread", 8192, nullptr, TEST_TASK_PRIORITY, &imu_task_handle);
    vTaskStartScheduler();
    return 0;
}