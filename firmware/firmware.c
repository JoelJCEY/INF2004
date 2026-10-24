#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "wifi_credentials.h"

static void incoming_publish_cb(void *arg, const char *topic, u32_t total_len)
{
    (void)arg;
    printf("Incoming topic: %s, length: %lu\n",
           topic, (unsigned long)total_len);
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    (void)arg;

    if (len == 1 && (flags & MQTT_DATA_FLAG_LAST)) {
        printf("Received test byte: 0x%02X\n", data[0]);
    } else {
        printf("Ignored unexpected test message\n");
    }
}

static void subscribe_cb(void *arg, err_t result)
{
    (void)arg;
    printf("Subscribe result: %d\n", (int)result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status)
{
    (void)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connected to laptop broker!\n");

        mqtt_set_inpub_callback(client, incoming_publish_cb,
                                incoming_data_cb, NULL);

        err_t result = mqtt_subscribe(client, "is11/test/command",
                                      1, subscribe_cb, NULL);
        printf("Subscribe request result: %d\n", (int)result);
    } else {
        printf("MQTT connection status: %d\n", (int)status);
    }
}

int main(void)
{
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Wi-Fi connection failed\n");
        return 1;
    }
    printf("Wi-Fi connected\n");

    ip_addr_t broker_ip;
    if (!ipaddr_aton(MQTT_BROKER_IP, &broker_ip)) {
        printf("Invalid broker IP address\n");
        return 1;
    }

    static struct mqtt_connect_client_info_t client_info = {
        .client_id = "buddy1-pico-w",
        .client_user = MQTT_USERNAME,
        .client_pass = MQTT_PASSWORD,
        .keep_alive = 60
    };

    cyw43_arch_lwip_begin();

    mqtt_client_t *client = mqtt_client_new();
    err_t result = client
        ? mqtt_client_connect(client, &broker_ip, 1883,
                              mqtt_connection_cb, NULL, &client_info)
        : ERR_MEM;

    cyw43_arch_lwip_end();

    if (result != ERR_OK) {
        printf("Could not start MQTT connection: %d\n", (int)result);
        return 1;
    }

    printf("Connecting to laptop MQTT broker...\n");

    // Prototype: publish one test byte every 200 ms.
    absolute_time_t next_publish = make_timeout_time_ms(200);
    static const uint8_t test_byte = 0x01;

    while (true) {
        cyw43_arch_poll();

        if (time_reached(next_publish)) {
            next_publish = make_timeout_time_ms(200);

            if (mqtt_client_is_connected(client)) {
                cyw43_arch_lwip_begin();
                err_t pub_result = mqtt_publish(
                    client, "is11/test",
                    &test_byte, sizeof(test_byte),
                    0, 0, NULL, NULL);
                cyw43_arch_lwip_end();

                if (pub_result != ERR_OK) {
                    printf("Test publish failed: %d\n", (int)pub_result);
                }
            }
        }

        sleep_ms(10);
    }
}