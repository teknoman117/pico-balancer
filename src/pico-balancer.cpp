#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

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

#include "MPU6050_6Axis_MotionApps_V6_12.h"

#define TEST_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1UL )

#define INT_PIN 9
#define SDA_PIN 10
#define SCL_PIN 11

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

void imu_task(void *queue_) {
    QueueHandle_t queue = static_cast<QueueHandle_t>(queue_);

    // mpu initialization
    MPU6050 mpu;
    mpu.initialize();
    if (mpu.dmpInitialize()) {
        printf("failed to initialize MPU6050 DMP\n");
        vTaskDelete(NULL);
    }

    mpu.setDMPEnabled(true);
    auto mpuIntStatus = mpu.getIntStatus();
    auto packetSize = mpu.dmpGetFIFOPacketSize();

    while (true) {
        // wait until bytes in fifo are at least one packet
        auto fifoCount = mpu.getFIFOCount();
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
        xQueueSend(queue, (void *) &euler, 10 / portTICK_PERIOD_MS);
    }
}

void main_task(__unused void *params) {
    // kick off the mpu thread
    QueueHandle_t queueIMU = xQueueCreate(10, sizeof(vec3));
    TaskHandle_t taskIMU;
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

    while (true) {
        // receive next orientation
        vec3 orientation;
        if (xQueueReceive(queueIMU, (void *) &orientation, portMAX_DELAY) != pdPASS) {
            continue;
        }

        // send orientation
        char msg[64] = {0};
        int len = snprintf(msg, sizeof msg, "ypr: %f,\t %f,\t %f\n", orientation.yaw,
                orientation.pitch, orientation.roll);
        res = send(sock, msg, len, 0);
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

int main(void)
{
    stdio_init_all();

    // initialize I2C1 bus for the MPU6050
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    /* for now, wait for the usb connection */
    //while (!stdio_usb_connected()) {
    //    tight_loop_contents();
    //}

    sleep_ms(1000);

    /* start the main thread */
    TaskHandle_t task;
    xTaskCreate(main_task, "MainThread", 8192, NULL, TEST_TASK_PRIORITY, &task);
    vTaskStartScheduler();
    return 0;
}