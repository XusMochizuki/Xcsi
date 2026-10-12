/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modified for Xcsi (2026): HE20 / channel 1 / US, binary CSI frames over
 * USB-Serial-JTAG (csi_frame.h), gain-lock commands, chip temperature.
 * See README.md "相对 upstream 的改动".
 */
/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_csi_gain_ctrl.h"

#include "csi_frame.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL   1
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0))
#define CONFIG_WIFI_BAND_MODE               WIFI_BAND_MODE_2G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW20
#define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW40
#define CONFIG_WIFI_2G_PROTOCOL             (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX)
#define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#else
#define CONFIG_WIFI_BANDWIDTH           WIFI_BW40
#endif

#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HE20
#define CONFIG_WIFI_COUNTRY_CODE         "US"
#define CONFIG_ESP_NOW_RATE             WIFI_PHY_RATE_MCS0_LGI

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF                      0
#endif


#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

#if CONFIG_IDF_TARGET_ESP32C5
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        .ghz_5g = CONFIG_WIFI_5G_PROTOCOL
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));
    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        .ghz_5g = CONFIG_WIFI_5G_BANDWIDTHS
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
#elif (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) || CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));
    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
#else
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_country_code(CONFIG_WIFI_COUNTRY_CODE, false));
#if CONFIG_IDF_TARGET_ESP32C5
    if ((CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW20)
            || (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_5G_ONLY && CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW20)) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#elif (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) || CONFIG_IDF_TARGET_ESP32C61
    if (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW20) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#else
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW20) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}

static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,//  WIFI_PHY_RATE_MCS0_LGI,
        .ersu = false,
        .dcm = false
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));

}

typedef struct {
    csi_frame_hdr_t hdr;
    int8_t csi[CSI_FRAME_MAX_CSI_LEN];
    uint32_t crc;           /* placeholder; real CRC is written right after csi[csi_len] */
} __attribute__((packed)) csi_frame_t;

#define CSI_QUEUE_DEPTH     16

static QueueHandle_t s_csi_queue;
static volatile int8_t s_temp_c;

/* RX gain lock, driven by single-byte host commands (see csi_cmd_task). */
typedef enum {
    GAIN_AUTO,
    GAIN_COLLECT,       /* recording samples for the baseline, then force */
    GAIN_FORCED,
} gain_state_t;

#define GAIN_BASELINE_SAMPLES   100

static volatile gain_state_t s_gain_state = GAIN_AUTO;
static volatile bool s_gain_cmd_lock;
static volatile bool s_gain_cmd_auto;

/* Called from the CSI callback (Wi-Fi task), as the official examples do. */
static void gain_ctrl_update(uint8_t agc_gain, int8_t fft_gain)
{
    static int s_samples;

    if (s_gain_cmd_auto) {
        s_gain_cmd_auto = false;
        esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
        s_gain_state = GAIN_AUTO;
        ESP_EARLY_LOGI(TAG, "gain: auto");
    }
    if (s_gain_cmd_lock) {
        s_gain_cmd_lock = false;
        esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
        esp_csi_gain_ctrl_reset_rx_gain_baseline();
        s_samples = 0;
        s_gain_state = GAIN_COLLECT;
    }
    if (s_gain_state == GAIN_COLLECT) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
        if (++s_samples >= GAIN_BASELINE_SAMPLES) {
            uint8_t agc;
            int8_t fft;
            if (esp_csi_gain_ctrl_get_rx_gain_baseline(&agc, &fft) == ESP_OK) {
                esp_csi_gain_ctrl_set_rx_force_gain(agc, fft);
                s_gain_state = GAIN_FORCED;
                ESP_EARLY_LOGI(TAG, "gain: forced agc %d fft %d", agc, fft);
            }
        }
    }
}

/* Runs in the Wi-Fi task: copy only, no formatting, never block. */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    static csi_frame_t s_frame;
    static uint32_t s_frame_seq;

    if (!info || !info->buf) {
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        return;
    }

    if (info->len > CSI_FRAME_MAX_CSI_LEN) {
        ESP_EARLY_LOGW(TAG, "csi len %d > %d, dropped", info->len, CSI_FRAME_MAX_CSI_LEN);
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    csi_frame_hdr_t *hdr = &s_frame.hdr;

    hdr->magic[0]     = CSI_FRAME_MAGIC0;
    hdr->magic[1]     = CSI_FRAME_MAGIC1;
    hdr->version      = CSI_FRAME_VERSION;
    hdr->hdr_len      = sizeof(csi_frame_hdr_t);
    hdr->csi_len      = info->len;
    hdr->flags        = info->first_word_invalid ? CSI_FRAME_FLAG_FIRST_WORD_INVALID : 0;
    hdr->frame_seq    = s_frame_seq++;
    hdr->tx_seq       = 0;
    if (info->payload && info->payload_len >= 15 + sizeof(uint32_t)) {
        memcpy(&hdr->tx_seq, info->payload + 15, sizeof(uint32_t));
    }
    hdr->timestamp_us = rx_ctrl->timestamp;
    memcpy(hdr->mac, info->mac, 6);
    hdr->rssi         = rx_ctrl->rssi;
    hdr->noise_floor  = rx_ctrl->noise_floor;
    hdr->channel      = rx_ctrl->channel;
    hdr->second       = rx_ctrl->second;
    hdr->rate         = rx_ctrl->rate;
    hdr->bb_format    = rx_ctrl->cur_bb_format;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &hdr->agc_gain, &hdr->fft_gain);
    gain_ctrl_update(hdr->agc_gain, hdr->fft_gain);
    if (s_gain_state == GAIN_FORCED) {
        hdr->flags |= CSI_FRAME_FLAG_GAIN_FORCED;
    }
    hdr->rx_state     = rx_ctrl->rx_state;
    hdr->temp_c       = s_temp_c;
    hdr->sig_len      = rx_ctrl->sig_len;
    hdr->rx_seq       = info->rx_seq;
    hdr->he_siga1     = rx_ctrl->he_siga1;
    hdr->he_siga2     = rx_ctrl->he_siga2;
    hdr->reserved1    = 0;
    memcpy(s_frame.csi, info->buf, info->len);

    /* Queue full: frame is dropped, the host sees it as a frame_seq gap. */
    xQueueSend(s_csi_queue, &s_frame, 0);
}

static void csi_output_task(void *arg)
{
    static csi_frame_t s_frame;
    temperature_sensor_handle_t temp;
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_cfg, &temp));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp));
    TickType_t last_temp = 0;

    for (;;) {
        xQueueReceive(s_csi_queue, &s_frame, portMAX_DELAY);
        if (xTaskGetTickCount() - last_temp >= pdMS_TO_TICKS(1000)) {
            float c;
            if (temperature_sensor_get_celsius(temp, &c) == ESP_OK) {
                s_temp_c = (int8_t)(c + (c >= 0 ? 0.5f : -0.5f));
            }
            last_temp = xTaskGetTickCount();
        }
        size_t len = sizeof(csi_frame_hdr_t) + s_frame.hdr.csi_len;
        uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)&s_frame, len);
        memcpy((uint8_t *)&s_frame + len, &crc, sizeof(crc));
        usb_serial_jtag_write_bytes(&s_frame, len + sizeof(crc), portMAX_DELAY);
    }
}

/* Host commands, one byte each: 'g' = lock RX gain at the median of the next
 * GAIN_BASELINE_SAMPLES frames, 'a' = back to automatic gain. */
static void csi_cmd_task(void *arg)
{
    uint8_t c;

    for (;;) {
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c == 'g') {
            s_gain_cmd_lock = true;
        } else if (c == 'a') {
            s_gain_cmd_auto = true;
        }
    }
}

static void csi_output_init()
{
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_cfg.tx_buffer_size = 4096;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    /* Route console through the same driver so log lines and frames don't interleave mid-write. */
    usb_serial_jtag_vfs_use_driver();

    s_csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_frame_t));
    assert(s_csi_queue);
    xTaskCreate(csi_output_task, "csi_output", 3072, NULL, 5, NULL);
    xTaskCreate(csi_cmd_task, "csi_cmd", 2048, NULL, 4, NULL);
}

static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /**< default config */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = false,
        .acquire_csi_force_lltf   = CSI_FORCE_LLTF,
        .acquire_csi_ht20         = true,
        .acquire_csi_ht40         = true,
        .acquire_csi_vht          = false,
        .acquire_csi_su           = false,
        .acquire_csi_mu           = false,
        .acquire_csi_dcm          = false,
        .acquire_csi_beamformed   = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
        .reserved                 = false
    };
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = false,
        .acquire_csi_ht20       = false,
        .acquire_csi_ht40       = false,
        .acquire_csi_su         = true,
        .acquire_csi_mu         = false,
        .acquire_csi_dcm        = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc    = 2,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .reserved               = false
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main()
{
    /**
     * @brief Initialize NVS
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /**
     * @brief Initialize Wi-Fi
     */
    wifi_init();

    /**
     * @brief Initialize ESP-NOW
     *        ESP-NOW protocol see: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
     */

    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };

    wifi_esp_now_init(peer);

    csi_output_init();
    wifi_csi_init();
}
