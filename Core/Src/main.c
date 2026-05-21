/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SoundSentry - 3 Mic + OLED + Servo + RTC + SD
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "../../ai/artifacts/starter_model.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    DIR_LEFT = 0,
    DIR_CENTER = 1,
    DIR_RIGHT = 2
} direction_t;

typedef enum {
    EVT_IDLE = 0,
    EVT_IMPULSE,
    EVT_CONTINUOUS
} event_type_t;

typedef enum {
    THREAT_LOW = 0,
    THREAT_HIGH
} threat_t;

typedef struct {
    uint8_t active;
    uint32_t start_ms;
    uint32_t last_loud_ms;
    uint16_t peak_level;
    direction_t peak_direction;
    event_type_t type;
    threat_t threat;
    uint8_t capture_channel;
    uint8_t preview_sound_class;
    uint8_t preview_sound_hits;
    uint8_t locked_sound_class;
    uint32_t direction_sum[3];
    uint16_t direction_peak[3];
} event_state_t;

typedef struct {
    uint32_t frame_count;
    uint32_t zero_cross_count;
    uint32_t abs_sum;
    uint64_t sq_sum;
    uint16_t peak;
    int16_t prev_sample;
    uint8_t has_prev_sample;
    uint16_t window_count;
    int16_t window[128];
} ai_event_accumulator_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ring_color_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIC_COUNT            3
#define BLOCK_FRAMES         32
#define ADC_DMA_LEN          (MIC_COUNT * BLOCK_FRAMES * 2)

#define SERVO_LEFT_US        1100
#define SERVO_CENTER_US      1500
#define SERVO_RIGHT_US       1900

#define QUIET_THRESHOLD      90
#define EVENT_TRIGGER_LEVEL  240
#define EVENT_STRONG_TRIGGER_LEVEL 520
#define EVENT_TRIGGER_HOLD_MS 120
#define EVENT_RELEASE_LEVEL  95
#define EVENT_END_GAP_MS     280
#define EVENT_FORCE_END_MS   2600
#define EVENT_RESULT_HOLD_MS 5000
#define IMPULSE_MAX_MS       420
#define HIGH_THREAT_LEVEL    520
#define HIGH_THREAT_MS       900
#define DIRECTION_MARGIN     18
#define DIRECTION_HOLD_MS    20
#define DIRECTION_CAPTURE_MS 180
#define DIR_LEFT_GAIN_PCT    128
#define DIR_CENTER_GAIN_PCT  78
#define DIR_RIGHT_GAIN_PCT   128
#define SERVO_STEP_US        90
#define SERVO_UPDATE_MS      5
#define OLED_UPDATE_MS       80
#define OLED2_UPDATE_MS      350
#define OLED1_I2C_ADDR       SSD1306_I2C_ADDR
#define OLED2_I2C_ADDR       SSD1306_I2C_ADDR
#define BT_STATE_GPIO_PORT   GPIOB
#define BT_STATE_PIN         GPIO_PIN_12
#define BT_TX_TIMEOUT_MS     320
#define BT_HEARTBEAT_MS      10000
#define BT_CONNECT_STABLE_MS 1800
#define BT_TX_SETTLE_MS      500
#define BT_BOOT_DELAY_MS     2500
#define BT_DEBUG_PING_MS     1000
#define BT_DEBUG_PING_ENABLE 0
#define BT_BAUD_SWEEP_MS     5000
#define BT_BAUD_SWEEP_ENABLE 0
#define BT_REQUIRE_STATE_PIN 0
#define RING_LED_COUNT       12
#define RING_UPDATE_MS       80
#define RING_RESET_SLOTS     48
#define RING_PWM_ZERO        3
#define RING_PWM_ONE         6
#define RING_PWM_BUFFER_LEN  ((RING_LED_COUNT * 24U) + RING_RESET_SLOTS)
#define SD_SYNC_INTERVAL_MS  5000
#define SD_SYNC_EVENT_COUNT  4
#define AI_WINDOW_SIZE       128
#define AI_WINDOW_STRIDE     1
#define AI_SAMPLE_SCALE      2048.0f
#define AI_FALLBACK_CLASS    STARTER_MODEL_CLASS_COUNT
#define MIC_NOISE_INIT       12
#define NOISE_TRACK_MARGIN   90
#define MIC_CALIBRATION_MS   3200
#define BAR_FULL_SCALE_MIN   180
#define BAR_FULL_SCALE_MAX   1200

#define DS3231_ADDR          (0x68 << 1)

#define SD_ERR_NONE          0
#define SD_ERR_MOUNT         1
#define SD_ERR_OPEN          2
#define SD_ERR_WRITE         3
#define SD_ERR_SEEK          4
#define SD_ERR_HEADER        5

/* Set to 1 only once to program the RTC, then set back to 0 */
#define RTC_SET_TIME_ON_BOOT 0
#define RTC_SET_YEAR         26
#define RTC_SET_MONTH        5
#define RTC_SET_DATE         14
#define RTC_SET_DAY          1
#define RTC_SET_HOUR         22
#define RTC_SET_MIN          45
#define RTC_SET_SEC          0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_tim1_ch1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_dma_buf[ADC_DMA_LEN];
volatile uint8_t adc_half_ready = 0;
volatile uint8_t adc_full_ready = 0;

uint16_t mic_level[MIC_COUNT] = {0, 0, 0};
uint16_t mic_noise_floor[MIC_COUNT] = {MIC_NOISE_INIT, MIC_NOISE_INIT, MIC_NOISE_INIT};
uint16_t current_level = 0;
direction_t direction_idx = DIR_CENTER;
direction_t pending_direction_idx = DIR_CENTER;
uint32_t pending_direction_ms = 0;
uint16_t servo_pulse_current = SERVO_CENTER_US;

uint32_t last_ui_ms = 0;
uint32_t last_oled_ms = 0;
uint32_t last_oled2_ms = 0;
uint32_t last_rtc_ms = 0;
uint32_t last_servo_ms = 0;
uint32_t event_result_hold_until_ms = 0;
uint32_t mic_calibration_until_ms = 0;
uint32_t event_candidate_start_ms = 0;
uint16_t event_candidate_peak_level = 0;
uint8_t event_candidate_capture_channel = 1U;
uint16_t mic_bar_full_scale = BAR_FULL_SCALE_MIN;

uint8_t oled1_ready = 0;
uint8_t oled2_ready = 0;
uint32_t bt_tx_count = 0;
uint32_t bt_last_tx_ms = 0;
uint32_t last_bt_heartbeat_ms = 0;
uint32_t last_bt_ping_ms = 0;
uint32_t last_bt_baud_sweep_ms = 0;
uint32_t bt_state_changed_ms = 0;
uint32_t bt_connected_since_ms = 0;
uint8_t bt_baud_index = 0;
uint8_t bt_raw_connected = 0;
uint8_t bt_stable_connected = 0;
uint8_t bt_boot_sent = 0;
uint8_t bt_last_tx_ok = 0;
uint16_t ring_pwm_data[RING_PWM_BUFFER_LEN] = {0};
ring_color_t ring_pixels[RING_LED_COUNT] = {0};
volatile uint8_t ring_dma_busy = 0;
uint32_t last_ring_ms = 0;

uint8_t sd_ready = 0;
uint8_t sd_last_error = SD_ERR_NONE;
uint32_t sd_log_count = 0;
uint8_t sd_file_open = 0;
uint8_t sd_log_dirty = 0;
uint8_t sd_unsynced_events = 0;
uint32_t last_sd_sync_ms = 0;

event_state_t event_state = {0};
ai_event_accumulator_t ai_event_acc = {0};
event_type_t last_event_type = EVT_IDLE;
direction_t last_event_direction = DIR_CENTER;
threat_t last_event_threat = THREAT_LOW;

char last_event_text[32] = "BOOT";
char last_timestamp[24] = "RTC-NA";
char last_sound_text[24] = "OTHER";
uint8_t last_sound_class = AI_FALLBACK_CLASS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t bcd_to_dec(uint8_t val)
{
    return (uint8_t)(((val >> 4) * 10U) + (val & 0x0FU));
}

#if RTC_SET_TIME_ON_BOOT
static uint8_t dec_to_bcd(uint8_t val)
{
    return (uint8_t)(((val / 10U) << 4) | (val % 10U));
}

static HAL_StatusTypeDef ds3231_set_datetime(uint8_t year, uint8_t month, uint8_t date,
                                             uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    uint8_t data[7];

    data[0] = dec_to_bcd(sec);
    data[1] = dec_to_bcd(min);
    data[2] = dec_to_bcd(hour);
    data[3] = dec_to_bcd(day);
    data[4] = dec_to_bcd(date);
    data[5] = dec_to_bcd(month);
    data[6] = dec_to_bcd(year);

    return HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, data, 7, 100);
}
#endif

static void read_ds3231_timestamp(char *out, size_t out_len)
{
    uint8_t data[7];

    if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, data, 7, 100) == HAL_OK) {
        uint8_t sec   = bcd_to_dec(data[0] & 0x7F);
        uint8_t min   = bcd_to_dec(data[1] & 0x7F);
        uint8_t hour  = bcd_to_dec(data[2] & 0x3F);
        uint8_t date  = bcd_to_dec(data[4] & 0x3F);
        uint8_t month = bcd_to_dec(data[5] & 0x1F);
        uint8_t year  = bcd_to_dec(data[6]);

        snprintf(out, out_len, "20%02u-%02u-%02u %02u:%02u:%02u",
                 year, month, date, hour, min, sec);
    } else {
        snprintf(out, out_len, "RTC-ERR");
    }
}

static const char *dir_long(direction_t dir)
{
    if (dir == DIR_LEFT) return "LEFT";
    if (dir == DIR_RIGHT) return "RIGHT";
    return "CENTER";
}

static const char *event_short(event_type_t type)
{
    if (type == EVT_IMPULSE) return "IMP";
    if (type == EVT_CONTINUOUS) return "CONT";
    return "IDLE";
}

static const char *event_long(event_type_t type)
{
    if (type == EVT_IMPULSE) return "IMPULSE";
    if (type == EVT_CONTINUOUS) return "CONTINUOUS";
    return "IDLE";
}

static const char *threat_long(threat_t threat)
{
    return (threat == THREAT_HIGH) ? "HIGH" : "LOW";
}

static uint8_t bt_is_connected(void)
{
    return bt_stable_connected;
}

static uint32_t bt_current_baud(void)
{
    static const uint32_t baud_table[] = {9600U, 38400U, 115200U, 57600U};
    uint8_t index = bt_baud_index;

    if (index >= (sizeof(baud_table) / sizeof(baud_table[0]))) {
        index = 0U;
    }

    return baud_table[index];
}

static const char *bt_baud_label(void)
{
    uint32_t baud = bt_current_baud();

    if (baud == 9600U) return "9";
    if (baud == 38400U) return "38";
    if (baud == 57600U) return "57";
    if (baud == 115200U) return "115";
    return "?";
}

static void bt_baud_sweep_service(uint32_t now_ms)
{
#if BT_BAUD_SWEEP_ENABLE
    if ((now_ms - last_bt_baud_sweep_ms) >= BT_BAUD_SWEEP_MS) {
        last_bt_baud_sweep_ms = now_ms;
        bt_baud_index++;
        if (bt_baud_index >= 4U) {
            bt_baud_index = 0U;
        }
        if (huart1.Init.BaudRate != bt_current_baud()) {
            HAL_UART_DeInit(&huart1);
            huart1.Init.BaudRate = bt_current_baud();
            if (HAL_UART_Init(&huart1) != HAL_OK) {
                bt_last_tx_ok = 0U;
            }
        }
        bt_boot_sent = 0U;
        last_bt_ping_ms = now_ms - BT_DEBUG_PING_MS;
    }
#else
    (void)now_ms;
#endif
}

static const char *bt_status_short(void)
{
    if (bt_is_connected()) {
        return "ON";
    }

    if (bt_last_tx_ok &&
        ((HAL_GetTick() - bt_last_tx_ms) <= (BT_HEARTBEAT_MS + 1500U))) {
        return "TX";
    }

    return "--";
}

static const char *bt_display_status(void)
{
    if (bt_is_connected()) {
        return "ON";
    }

    if (bt_raw_connected) {
        return "CN";
    }

    if (bt_last_tx_ok &&
        ((HAL_GetTick() - bt_last_tx_ms) <= (BT_HEARTBEAT_MS + 1500U))) {
        return "TX";
    }

    if ((bt_tx_count > 0U) && (bt_last_tx_ok == 0U)) {
        return "ER";
    }

    return "--";
}

static void bt_poll_state(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t raw = (HAL_GPIO_ReadPin(BT_STATE_GPIO_PORT, BT_STATE_PIN) == GPIO_PIN_SET) ? 1U : 0U;

    if (raw != bt_raw_connected) {
        bt_raw_connected = raw;
        bt_state_changed_ms = now_ms;

        if (!raw) {
            bt_stable_connected = 0U;
            bt_boot_sent = 0U;
            bt_last_tx_ok = 0U;
        }
    }

    if (raw && !bt_stable_connected &&
        ((now_ms - bt_state_changed_ms) >= BT_CONNECT_STABLE_MS)) {
        bt_stable_connected = 1U;
        bt_connected_since_ms = now_ms;
        last_bt_heartbeat_ms = now_ms;
        bt_last_tx_ok = 0U;
    }
}

static uint8_t bt_can_transmit(void)
{
    bt_poll_state();

#if BT_REQUIRE_STATE_PIN
    if (!bt_stable_connected) {
        return 0U;
    }

    if ((HAL_GetTick() - bt_connected_since_ms) < BT_TX_SETTLE_MS) {
        return 0U;
    }
#else
    if (HAL_GetTick() < BT_BOOT_DELAY_MS) {
        return 0U;
    }
#endif

    return 1U;
}

static void format_oled_time_header(char *out, size_t out_len)
{
    uint32_t display_count = sd_log_count;

    if (display_count > 9999999UL) {
        display_count = 9999999UL;
    }

    if ((last_timestamp[0] == '2') &&
        (last_timestamp[13] == ':') &&
        (last_timestamp[16] == ':')) {
        snprintf(out, out_len, "%c%c:%c%c:%c%c N:%lu",
                 last_timestamp[11], last_timestamp[12],
                 last_timestamp[14], last_timestamp[15],
                 last_timestamp[17], last_timestamp[18],
                 (unsigned long)display_count);
    } else {
        snprintf(out, out_len, "RTC ERR N:%lu", (unsigned long)display_count);
    }
}

static void bt_send_line(const char *line)
{
    size_t len = strlen(line);

    if (len == 0U) {
        return;
    }

    if (!bt_can_transmit()) {
        return;
    }

    if (HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, BT_TX_TIMEOUT_MS) == HAL_OK) {
        bt_tx_count++;
        bt_last_tx_ms = HAL_GetTick();
        bt_last_tx_ok = 1U;
    } else {
        bt_last_tx_ok = 0U;
    }
}

static void bt_send_boot_message(void)
{
    char msg[96];

    snprintf(msg, sizeof(msg), "BOOT,SOUNDSENTRY,BAUD=%lu,TIME=%s,SD=%s,BT=%s\r\n",
             (unsigned long)bt_current_baud(),
             last_timestamp,
             sd_ready ? "OK" : "ERR",
             bt_status_short());
    bt_send_line(msg);
}

static void bt_send_status_message(void)
{
    char msg[112];

    snprintf(msg, sizeof(msg), "STAT,N=%lu,BAUD=%lu,TIME=%s,SD=%s,BT=%s\r\n",
             (unsigned long)sd_log_count,
             (unsigned long)bt_current_baud(),
             last_timestamp,
             sd_ready ? "OK" : "ERR",
             bt_status_short());
    bt_send_line(msg);
}

#if BT_DEBUG_PING_ENABLE
static void bt_send_ping_message(void)
{
    char msg[80];

    snprintf(msg, sizeof(msg), "PING,T=%lu,N=%lu,BT=%s\r\n",
             (unsigned long)(HAL_GetTick() / 1000U),
             (unsigned long)sd_log_count,
             bt_status_short());
    bt_send_line(msg);
}
#endif

static void bt_service(void)
{
    uint32_t now_ms = HAL_GetTick();

    bt_baud_sweep_service(now_ms);
    bt_poll_state();
    if (!bt_can_transmit()) {
        return;
    }

    if (!bt_boot_sent) {
        bt_send_boot_message();
        bt_boot_sent = 1U;
        last_bt_heartbeat_ms = now_ms;
        return;
    }

#if BT_DEBUG_PING_ENABLE
    if ((now_ms - last_bt_ping_ms) >= BT_DEBUG_PING_MS) {
        last_bt_ping_ms = now_ms;
        bt_send_ping_message();
    }
#endif

    if ((now_ms - last_bt_heartbeat_ms) >= BT_HEARTBEAT_MS) {
        last_bt_heartbeat_ms = now_ms;
        bt_send_status_message();
    }
}

static void ai_event_reset(void)
{
    memset(&ai_event_acc, 0, sizeof(ai_event_acc));
}

static void ai_event_capture_block(uint16_t *buf, uint32_t sample_count, const uint16_t *dc, uint8_t capture_channel)
{
    if (capture_channel >= MIC_COUNT) {
        capture_channel = 1U;
    }

    for (uint32_t i = 0; i < sample_count; i += MIC_COUNT) {
        int32_t sample = (int32_t)buf[i + capture_channel] - dc[capture_channel];
        uint32_t abs_sample = (sample < 0) ? (uint32_t)(-sample) : (uint32_t)sample;

        ai_event_acc.frame_count++;
        ai_event_acc.abs_sum += abs_sample;
        ai_event_acc.sq_sum += (uint64_t)((int64_t)sample * (int64_t)sample);

        if (abs_sample > ai_event_acc.peak) {
            ai_event_acc.peak = (uint16_t)abs_sample;
        }

        if (ai_event_acc.has_prev_sample) {
            if (((sample < 0) && (ai_event_acc.prev_sample >= 0)) ||
                ((sample >= 0) && (ai_event_acc.prev_sample < 0))) {
                ai_event_acc.zero_cross_count++;
            }
        } else {
            ai_event_acc.has_prev_sample = 1;
        }
        ai_event_acc.prev_sample = (int16_t)sample;

        if (ai_event_acc.window_count < AI_WINDOW_SIZE) {
            ai_event_acc.window[ai_event_acc.window_count++] = (int16_t)sample;
        }
    }
}

static void ai_compute_spectral_features(float *spec_centroid_norm,
                                         float *spec_rolloff_norm,
                                         float *spec_bandwidth_norm,
                                         float *spec_flatness,
                                         float *low_band_share,
                                         float *high_band_share)
{
    uint16_t n = ai_event_acc.window_count;
    float low_energy = 0.0f;
    float high_energy = 0.0f;
    float abs_sum = 0.0f;
    float hp_abs_sum = 0.0f;
    float diff_sq_sum = 0.0f;
    float hp_zero_cross = 0.0f;
    float lp = 0.0f;
    float prev_x = 0.0f;
    float prev_hp = 0.0f;

    *spec_centroid_norm = 0.0f;
    *spec_rolloff_norm = 0.0f;
    *spec_bandwidth_norm = 0.0f;
    *spec_flatness = 0.0f;
    *low_band_share = 0.0f;
    *high_band_share = 0.0f;

    if (n < 4U) {
        return;
    }

    lp = (float)ai_event_acc.window[0] / AI_SAMPLE_SCALE;
    prev_x = lp;
    prev_hp = 0.0f;

    for (uint16_t i = 0; i < n; ++i) {
        float x = (float)ai_event_acc.window[i] / AI_SAMPLE_SCALE;
        float hp;

        /* Cheap first-order split into low and high components. */
        lp = (0.78f * lp) + (0.22f * x);
        hp = x - lp;

        low_energy += lp * lp;
        high_energy += hp * hp;
        abs_sum += fabsf(x);
        hp_abs_sum += fabsf(hp);

        if (i > 0U) {
            float diff = x - prev_x;
            diff_sq_sum += diff * diff;
            if (((hp < 0.0f) && (prev_hp >= 0.0f)) ||
                ((hp >= 0.0f) && (prev_hp < 0.0f))) {
                hp_zero_cross += 1.0f;
            }
        }

        prev_x = x;
        prev_hp = hp;
    }

    {
        float total_energy = low_energy + high_energy + 1e-9f;
        float hp_zcr_norm = hp_zero_cross / (float)(n - 1U);

        *low_band_share = low_energy / total_energy;
        *high_band_share = high_energy / total_energy;
        *spec_rolloff_norm = *high_band_share;
        *spec_centroid_norm = hp_abs_sum / (abs_sum + 1e-6f);
        *spec_bandwidth_norm = sqrtf(diff_sq_sum / (float)(n - 1U));
        *spec_flatness = (0.60f * (*high_band_share)) + (0.40f * hp_zcr_norm);

        if (*spec_centroid_norm > 1.0f) *spec_centroid_norm = 1.0f;
        if (*spec_rolloff_norm > 1.0f) *spec_rolloff_norm = 1.0f;
        if (*spec_bandwidth_norm > 1.0f) *spec_bandwidth_norm = 1.0f;
        if (*spec_flatness > 1.0f) *spec_flatness = 1.0f;
    }
}

static void ai_build_feature_vector(float *features)
{
    float frame_count = (float)ai_event_acc.frame_count;
    float rms = 0.0f;
    float abs_mean = 0.0f;
    float peak = 0.0f;
    float zcr = 0.0f;
    float crest_factor = 0.0f;
    float spec_centroid_norm = 0.0f;
    float spec_rolloff_norm = 0.0f;
    float spec_bandwidth_norm = 0.0f;
    float spec_flatness = 0.0f;
    float low_band_share = 0.0f;
    float high_band_share = 0.0f;

    for (int i = 0; i < STARTER_MODEL_FEATURE_COUNT; ++i) {
        features[i] = 0.0f;
    }

    if (ai_event_acc.frame_count == 0U) {
        return;
    }

    abs_mean = ((float)ai_event_acc.abs_sum / frame_count) / AI_SAMPLE_SCALE;
    rms = sqrtf(((float)ai_event_acc.sq_sum / frame_count)) / AI_SAMPLE_SCALE;
    peak = ((float)ai_event_acc.peak / AI_SAMPLE_SCALE);

    if (ai_event_acc.frame_count > 1U) {
        zcr = (float)ai_event_acc.zero_cross_count / (float)(ai_event_acc.frame_count - 1U);
    }

    crest_factor = peak / (rms + 1e-6f);
    ai_compute_spectral_features(&spec_centroid_norm,
                                 &spec_rolloff_norm,
                                 &spec_bandwidth_norm,
                                 &spec_flatness,
                                 &low_band_share,
                                 &high_band_share);

    features[0] = rms;
    features[1] = abs_mean;
    features[2] = peak;
    features[3] = zcr;
    features[4] = crest_factor;
    features[5] = spec_centroid_norm;
    features[6] = spec_rolloff_norm;
    features[7] = spec_bandwidth_norm;
    features[8] = spec_flatness;
    features[9] = low_band_share;
    features[10] = high_band_share;
}

static const char *ai_sound_name(int sound_class)
{
    if ((sound_class < 0) || (sound_class >= STARTER_MODEL_CLASS_COUNT)) {
        return "OTHER";
    }

    return starter_model_class_name(sound_class);
}

static event_type_t event_type_from_duration(uint32_t duration_ms)
{
    return (duration_ms <= IMPULSE_MAX_MS) ? EVT_IMPULSE : EVT_CONTINUOUS;
}

static uint8_t ai_is_knock_like(const float *features, uint32_t duration_ms)
{
    return (uint8_t)(
        (duration_ms <= 260U) &&
        (features[9] > 0.82f) &&
        (features[3] < 0.030f) &&
        (features[7] < 0.045f)
    );
}

static uint8_t ai_is_glass_like(const float *features, uint32_t duration_ms)
{
    return (uint8_t)(
        (duration_ms <= 320U) &&
        (features[10] > 0.58f) &&
        (features[8] > 0.40f) &&
        (features[5] > 0.36f) &&
        (features[3] > 0.095f) &&
        (features[9] < 0.48f)
    );
}

static uint8_t ai_is_clap_like(const float *features, uint32_t duration_ms)
{
    return (uint8_t)(
        (duration_ms <= IMPULSE_MAX_MS) &&
        (features[2] > 0.040f) &&
        ((features[4] > 1.7f) ||
         (features[3] > 0.015f) ||
         (features[7] > 0.018f) ||
         (features[1] > 0.007f))
    );
}

static int ai_refine_sound_class(const float *features, uint32_t duration_ms, event_type_t event_type)
{
    int model_guess = starter_model_predict_class(features);
    float rms = features[0];
    float abs_mean = features[1];
    float peak = features[2];
    float zcr = features[3];
    float crest_factor = features[4];
    float low_band_share = features[9];

    if ((peak < 0.04f) || (rms < 0.003f)) {
        return AI_FALLBACK_CLASS;
    }

    if ((model_guess >= 0) && (model_guess < STARTER_MODEL_CLASS_COUNT)) {
        return model_guess;
    }

    /*
     * Lightweight heuristic classifier for the actual hardware.
     * This is intentionally conservative because the saved 1-NN model was only
     * a small starter model and did not generalize well to the live setup.
     */
    if ((duration_ms <= IMPULSE_MAX_MS) || (event_type == EVT_IMPULSE)) {
        if ((model_guess >= 0) && (model_guess <= 2)) {
            return model_guess;
        }

        if (ai_is_knock_like(features, duration_ms)) {
            return 1; /* KNOCK */
        }

        if (ai_is_glass_like(features, duration_ms)) {
            return 2; /* GLASS */
        }

        if (ai_is_clap_like(features, duration_ms)) {
            return 0; /* CLAP */
        }

        return 0; /* default impulse fallback -> CLAP */
    }

    if ((peak < 0.20f) &&
        (abs_mean < 0.018f) &&
        (duration_ms < 420U) &&
        (low_band_share > 0.68f)) {
        return 5; /* TICK */
    }

    if ((crest_factor > 6.0f) &&
        (zcr < 0.060f) &&
        (duration_ms < 900U)) {
        return 3; /* COUGH */
    }

    if ((duration_ms >= 600U) &&
        ((abs_mean > 0.016f) ||
         (zcr > 0.030f))) {
        return 4; /* LAUGH */
    }

    if ((model_guess >= 3) && (model_guess < STARTER_MODEL_CLASS_COUNT) && (duration_ms >= 500U)) {
        return model_guess;
    }
    return AI_FALLBACK_CLASS;
}

static int ai_guess_live_sound_class(uint32_t duration_ms)
{
    float features[STARTER_MODEL_FEATURE_COUNT];
    event_type_t live_type = event_type_from_duration(duration_ms);

    if (ai_event_acc.frame_count < 12U) {
        return AI_FALLBACK_CLASS;
    }

    ai_build_feature_vector(features);

    if (ai_is_knock_like(features, duration_ms)) {
        return 1;
    }

    if (ai_is_glass_like(features, duration_ms)) {
        return 2;
    }

    if ((live_type == EVT_IMPULSE) && ai_is_clap_like(features, duration_ms)) {
        return 0; /* CLAP */
    }

    return ai_refine_sound_class(features, duration_ms, live_type);
}

static void update_live_sound_lock(void)
{
    uint32_t live_duration_ms;
    int live_class;

    if (!event_state.active || (ai_event_acc.frame_count < 12U)) {
        return;
    }

    live_duration_ms = HAL_GetTick() - event_state.start_ms;
    if (live_duration_ms > IMPULSE_MAX_MS) {
        return;
    }

    live_class = ai_guess_live_sound_class(live_duration_ms);
    if ((live_class < 0) || (live_class > 2)) {
        return;
    }

    if ((uint8_t)live_class == event_state.preview_sound_class) {
        if (event_state.preview_sound_hits < 255U) {
            event_state.preview_sound_hits++;
        }
    } else {
        event_state.preview_sound_class = (uint8_t)live_class;
        event_state.preview_sound_hits = 1U;
    }

    if ((event_state.preview_sound_hits >= 2U) ||
        ((live_class == 0) && (live_duration_ms >= 70U))) {
        event_state.locked_sound_class = (uint8_t)live_class;
    }
}

static uint16_t direction_to_pulse(direction_t dir)
{
    if (dir == DIR_LEFT) return SERVO_LEFT_US;
    if (dir == DIR_RIGHT) return SERVO_RIGHT_US;
    return SERVO_CENTER_US;
}

static uint16_t weighted_level_for_channel(uint8_t channel)
{
    if (channel == 0U) {
        return (uint16_t)(((uint32_t)mic_level[0] * DIR_LEFT_GAIN_PCT) / 100U);
    }
    if (channel == 2U) {
        return (uint16_t)(((uint32_t)mic_level[2] * DIR_RIGHT_GAIN_PCT) / 100U);
    }
    return (uint16_t)(((uint32_t)mic_level[1] * DIR_CENTER_GAIN_PCT) / 100U);
}

static uint8_t select_capture_channel_from_levels(void)
{
    uint16_t left_level = weighted_level_for_channel(0U);
    uint16_t center_level = weighted_level_for_channel(1U);
    uint16_t right_level = weighted_level_for_channel(2U);
    uint16_t max_level = left_level;
    uint16_t side_margin;

    if (center_level > max_level) {
        max_level = center_level;
    }
    if (right_level > max_level) {
        max_level = right_level;
    }

    side_margin = (uint16_t)(DIRECTION_MARGIN + (max_level / 10U));

    if ((left_level >= right_level) && ((left_level + side_margin) >= center_level)) {
        return 0U;
    }
    if ((right_level > left_level) && ((right_level + side_margin) >= center_level)) {
        return 2U;
    }
    if ((center_level >= left_level) && (center_level >= right_level)) {
        return 1U;
    }
    return (left_level >= right_level) ? 0U : 2U;
}

static void sd_log_init(void)
{
    const char *header = "timestamp,peak,direction,event,threat,sound_label,sound_class\r\n";
    UINT header_len = (UINT)strlen(header);
    UINT bw = 0;

    sd_ready = 0;
    sd_last_error = SD_ERR_NONE;
    sd_log_count = 0;
    sd_file_open = 0;
    sd_log_dirty = 0;
    sd_unsynced_events = 0;
    last_sd_sync_ms = HAL_GetTick();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(20);

    if (f_mount(&USERFatFS, USERPath, 1) != FR_OK) {
        sd_last_error = SD_ERR_MOUNT;
        return;
    }

    if (f_open(&USERFile, "events.csv", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
        sd_last_error = SD_ERR_OPEN;
        return;
    }

    if (f_size(&USERFile) == 0) {
        if (f_write(&USERFile, header, header_len, &bw) != FR_OK || bw != header_len) {
            f_close(&USERFile);
            sd_last_error = SD_ERR_HEADER;
            return;
        }
    }

    if (f_lseek(&USERFile, f_size(&USERFile)) != FR_OK) {
        f_close(&USERFile);
        sd_last_error = SD_ERR_SEEK;
        return;
    }

    sd_file_open = 1;
    sd_ready = 1;
    sd_last_error = SD_ERR_NONE;
}

static void log_event_to_sd(const char *csv_line)
{
    UINT bw = 0;
    UINT len = (UINT)strlen(csv_line);

    if (!sd_ready) {
        return;
    }

    if (!sd_file_open) {
        if (f_open(&USERFile, "events.csv", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK) {
            sd_ready = 0;
            sd_last_error = SD_ERR_OPEN;
            return;
        }
        if (f_lseek(&USERFile, f_size(&USERFile)) != FR_OK) {
            f_close(&USERFile);
            sd_ready = 0;
            sd_last_error = SD_ERR_SEEK;
            return;
        }
        sd_file_open = 1;
    }

    if (f_write(&USERFile, csv_line, len, &bw) != FR_OK || bw != len) {
        f_close(&USERFile);
        sd_file_open = 0;
        sd_ready = 0;
        sd_last_error = SD_ERR_WRITE;
        return;
    }

    sd_last_error = SD_ERR_NONE;
    sd_log_dirty = 1U;
    sd_unsynced_events++;
    sd_log_count++;
}

static void sd_log_service(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (!sd_ready || !sd_file_open || !sd_log_dirty) {
        return;
    }

    if ((sd_unsynced_events < SD_SYNC_EVENT_COUNT) &&
        ((now_ms - last_sd_sync_ms) < SD_SYNC_INTERVAL_MS)) {
        return;
    }

    if (f_sync(&USERFile) != FR_OK) {
        sd_last_error = SD_ERR_WRITE;
        sd_ready = 0;
        f_close(&USERFile);
        sd_file_open = 0;
        return;
    }

    last_sd_sync_ms = now_ms;
    sd_log_dirty = 0U;
    sd_unsynced_events = 0U;
    sd_last_error = SD_ERR_NONE;
}

static void update_direction_from_levels(void)
{
    uint16_t left_level = weighted_level_for_channel(0U);
    uint16_t center_level = weighted_level_for_channel(1U);
    uint16_t right_level = weighted_level_for_channel(2U);
    uint16_t max_level = left_level;
    direction_t raw_direction = DIR_CENTER;
    uint32_t now_ms = HAL_GetTick();
    uint16_t side_max = left_level;
    uint16_t side_min = right_level;
    uint16_t side_gap;
    uint16_t side_margin = (uint16_t)(DIRECTION_MARGIN + (max_level / 10U));

    if (center_level > max_level) {
        max_level = center_level;
    }
    if (right_level > max_level) {
        max_level = right_level;
    }
    if (right_level > side_max) {
        side_max = right_level;
    }
    if (left_level < side_min) {
        side_min = left_level;
    }
    side_gap = (uint16_t)(side_max - side_min);
    side_margin = (uint16_t)(DIRECTION_MARGIN + (max_level / 10U));

    if (max_level < QUIET_THRESHOLD) {
        raw_direction = DIR_CENTER;
    } else if ((center_level > (side_max + side_margin + (max_level / 10U))) &&
               (side_gap < (uint16_t)(side_margin + (max_level / 12U)))) {
        raw_direction = DIR_CENTER;
    } else if ((left_level > right_level) &&
               (side_gap >= (side_margin / 2U)) &&
               ((left_level + side_margin) >= center_level)) {
        raw_direction = DIR_LEFT;
    } else if ((right_level > left_level) &&
               (side_gap >= (side_margin / 2U)) &&
               ((right_level + side_margin) >= center_level)) {
        raw_direction = DIR_RIGHT;
    } else if ((side_max + (side_margin / 2U)) >= center_level) {
        raw_direction = (left_level >= right_level) ? DIR_LEFT : DIR_RIGHT;
    } else if (left_level > right_level) {
        raw_direction = DIR_LEFT;
    } else if (right_level > left_level) {
        raw_direction = DIR_RIGHT;
    } else {
        raw_direction = DIR_CENTER;
    }

    current_level = max_level;

    if (raw_direction != pending_direction_idx) {
        pending_direction_idx = raw_direction;
        pending_direction_ms = now_ms;
    }

    if ((now_ms - pending_direction_ms) >= DIRECTION_HOLD_MS) {
        direction_idx = pending_direction_idx;
    }
}

static void accumulate_event_direction(void)
{
    uint16_t dir_level[3];
    uint32_t elapsed_ms;
    uint32_t weight = 1U;

    if (!event_state.active) {
        return;
    }

    elapsed_ms = HAL_GetTick() - event_state.start_ms;
    if ((elapsed_ms > DIRECTION_CAPTURE_MS) &&
        (current_level + EVENT_RELEASE_LEVEL < event_state.peak_level)) {
        return;
    }

    if (elapsed_ms <= 60U) {
        weight = 4U;
    } else if (elapsed_ms <= 120U) {
        weight = 3U;
    } else if (elapsed_ms <= DIRECTION_CAPTURE_MS) {
        weight = 2U;
    }

    dir_level[0] = weighted_level_for_channel(0U);
    dir_level[1] = weighted_level_for_channel(1U);
    dir_level[2] = weighted_level_for_channel(2U);

    for (int ch = 0; ch < MIC_COUNT; ++ch) {
        event_state.direction_sum[ch] += (uint32_t)(dir_level[ch] * weight);
        if (dir_level[ch] > event_state.direction_peak[ch]) {
            event_state.direction_peak[ch] = dir_level[ch];
        }
    }
}

static direction_t resolve_event_direction(void)
{
    uint64_t left_score = (uint64_t)event_state.direction_sum[0] + ((uint64_t)event_state.direction_peak[0] * 8U);
    uint64_t center_score = (uint64_t)event_state.direction_sum[1] + ((uint64_t)event_state.direction_peak[1] * 6U);
    uint64_t right_score = (uint64_t)event_state.direction_sum[2] + ((uint64_t)event_state.direction_peak[2] * 8U);
    uint64_t side_score = left_score;
    uint64_t far_score = right_score;
    uint64_t side_margin_score;
    uint64_t center_margin_score;
    direction_t side_dir = DIR_LEFT;
    uint16_t side_peak = event_state.direction_peak[0];
    uint16_t far_peak = event_state.direction_peak[2];
    uint16_t center_peak = event_state.direction_peak[1];

    if (right_score > side_score) {
        side_score = right_score;
        far_score = left_score;
        side_dir = DIR_RIGHT;
        side_peak = event_state.direction_peak[2];
        far_peak = event_state.direction_peak[0];
    }

    if ((side_peak < QUIET_THRESHOLD) && (center_peak < QUIET_THRESHOLD)) {
        return DIR_CENTER;
    }

    side_margin_score = (uint64_t)(DIRECTION_MARGIN * 12U) + ((uint64_t)event_state.peak_level * 6U);
    center_margin_score = side_margin_score + ((uint64_t)event_state.peak_level * 4U);

    if ((event_state.capture_channel == 0U) &&
        (left_score + side_margin_score >= center_score) &&
        (left_score > right_score + (side_margin_score / 2U))) {
        return DIR_LEFT;
    }

    if ((event_state.capture_channel == 2U) &&
        (right_score + side_margin_score >= center_score) &&
        (right_score > left_score + (side_margin_score / 2U))) {
        return DIR_RIGHT;
    }

    if ((center_score > side_score + center_margin_score) &&
        (center_peak > (uint16_t)(side_peak + DIRECTION_MARGIN + (event_state.peak_level / 8U))) &&
        (far_peak + DIRECTION_MARGIN >= side_peak)) {
        return DIR_CENTER;
    }

    if ((side_score > far_score + side_margin_score) &&
        (side_peak + DIRECTION_MARGIN >= center_peak)) {
        return side_dir;
    }

    if (side_score + side_margin_score >= center_score) {
        return side_dir;
    }

    if (center_score >= side_score) {
        return DIR_CENTER;
    }

    return side_dir;
}

static void bt_send_event_message(uint32_t sound_duration_ms)
{
    char msg[168];

    snprintf(msg, sizeof(msg),
             "EVENT,N=%lu,TIME=%s,SOUND=%s,DIR=%s,TYPE=%s,THREAT=%s,PEAK=%u,DUR=%lu,SD=%s,BT=%s\r\n",
             (unsigned long)sd_log_count,
             last_timestamp,
             last_sound_text,
             dir_long(event_state.peak_direction),
             event_long(event_state.type),
             threat_long(event_state.threat),
             event_state.peak_level,
             (unsigned long)sound_duration_ms,
             sd_ready ? "OK" : "ERR",
             bt_status_short());
    bt_send_line(msg);
}

static void finalize_event(uint32_t now_ms)
{
    uint32_t duration_ms;
    uint32_t sound_duration_ms;
    char csv_line[120];
    float features[STARTER_MODEL_FEATURE_COUNT];
    int sound_class;

    duration_ms = now_ms - event_state.start_ms;
    if (event_state.last_loud_ms >= event_state.start_ms) {
        sound_duration_ms = event_state.last_loud_ms - event_state.start_ms;
    } else {
        sound_duration_ms = duration_ms;
    }

    event_state.type = event_type_from_duration(sound_duration_ms);

    if ((event_state.peak_level >= HIGH_THREAT_LEVEL) || (sound_duration_ms >= HIGH_THREAT_MS)) {
        event_state.threat = THREAT_HIGH;
    } else {
        event_state.threat = THREAT_LOW;
    }

    event_state.peak_direction = resolve_event_direction();
    ai_build_feature_vector(features);
    sound_class = ai_refine_sound_class(features, sound_duration_ms, event_state.type);
    if ((event_state.locked_sound_class <= 2U) &&
        ((sound_class == AI_FALLBACK_CLASS) || (sound_class >= 3))) {
        sound_class = (int)event_state.locked_sound_class;
        event_state.type = EVT_IMPULSE;
    }
    last_event_type = event_state.type;
    last_event_direction = event_state.peak_direction;
    last_event_threat = event_state.threat;
    last_sound_class = (uint8_t)sound_class;
    snprintf(last_sound_text, sizeof(last_sound_text), "%s", ai_sound_name(sound_class));
    snprintf(last_event_text, sizeof(last_event_text), "%s %s",
             last_sound_text, event_short(event_state.type));

    snprintf(csv_line, sizeof(csv_line), "%s,%u,%s,%s,%s,%s,%u\r\n",
             last_timestamp,
             event_state.peak_level,
             dir_long(event_state.peak_direction),
             event_long(event_state.type),
             threat_long(event_state.threat),
             last_sound_text,
             (unsigned int)last_sound_class);

    log_event_to_sd(csv_line);
    bt_send_event_message(sound_duration_ms);
    event_result_hold_until_ms = now_ms + EVENT_RESULT_HOLD_MS;

    event_state.active = 0;
    event_state.peak_level = 0;
    event_state.peak_direction = DIR_CENTER;
    event_state.type = EVT_IDLE;
    event_state.threat = THREAT_LOW;
    event_state.capture_channel = 1U;
    event_state.preview_sound_class = AI_FALLBACK_CLASS;
    event_state.preview_sound_hits = 0U;
    event_state.locked_sound_class = AI_FALLBACK_CLASS;
    memset(event_state.direction_sum, 0, sizeof(event_state.direction_sum));
    memset(event_state.direction_peak, 0, sizeof(event_state.direction_peak));
    event_candidate_start_ms = 0;
    event_candidate_peak_level = 0;
    event_candidate_capture_channel = 1U;
}

static void reset_event_candidate(void)
{
    event_candidate_start_ms = 0;
    event_candidate_peak_level = 0;
    event_candidate_capture_channel = 1U;
}

static void update_event_engine(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (!event_state.active) {
        if (now_ms < mic_calibration_until_ms) {
            reset_event_candidate();
            return;
        }
        if (now_ms < event_result_hold_until_ms) {
            reset_event_candidate();
            return;
        }

        if (current_level < EVENT_TRIGGER_LEVEL) {
            reset_event_candidate();
            return;
        }

        if (event_candidate_start_ms == 0U) {
            event_candidate_start_ms = now_ms;
            event_candidate_peak_level = current_level;
            event_candidate_capture_channel = select_capture_channel_from_levels();
        } else {
            if (current_level > event_candidate_peak_level) {
                event_candidate_peak_level = current_level;
                event_candidate_capture_channel = select_capture_channel_from_levels();
            }
        }

        if ((current_level >= EVENT_STRONG_TRIGGER_LEVEL) ||
            ((now_ms - event_candidate_start_ms) >= EVENT_TRIGGER_HOLD_MS)) {
            event_state.active = 1;
            event_state.start_ms = event_candidate_start_ms;
            event_state.last_loud_ms = now_ms;
            event_state.peak_level = event_candidate_peak_level;
            event_state.peak_direction = direction_idx;
            event_state.type = EVT_IDLE;
            event_state.threat = THREAT_LOW;
            event_state.capture_channel = event_candidate_capture_channel;
            event_state.preview_sound_class = AI_FALLBACK_CLASS;
            event_state.preview_sound_hits = 0U;
            event_state.locked_sound_class = AI_FALLBACK_CLASS;
            memset(event_state.direction_sum, 0, sizeof(event_state.direction_sum));
            memset(event_state.direction_peak, 0, sizeof(event_state.direction_peak));
            reset_event_candidate();
            accumulate_event_direction();
        }
        return;
    }

    if (current_level > event_state.peak_level) {
        event_state.peak_level = current_level;
        event_state.peak_direction = direction_idx;
        if ((now_ms - event_state.start_ms) <= DIRECTION_CAPTURE_MS) {
            event_state.capture_channel = select_capture_channel_from_levels();
        }
    }

    if (current_level >= EVENT_RELEASE_LEVEL) {
        event_state.last_loud_ms = now_ms;
    }

    if ((now_ms - event_state.start_ms) >= EVENT_FORCE_END_MS) {
        finalize_event(now_ms);
        return;
    }

    if ((now_ms - event_state.last_loud_ms) >= EVENT_END_GAP_MS) {
        finalize_event(now_ms);
    }
}

static uint8_t mic_is_calibrating(void)
{
    return (HAL_GetTick() < mic_calibration_until_ms) ? 1U : 0U;
}

static void update_noise_floor_from_raw(const uint16_t *raw_level, uint8_t fast_calibration)
{
    for (int ch = 0; ch < MIC_COUNT; ch++) {
        uint16_t floor = mic_noise_floor[ch];

        if (fast_calibration) {
            floor = (uint16_t)(((uint32_t)floor + raw_level[ch] + 1U) / 2U);
        } else if (!event_state.active) {
            if (raw_level[ch] <= (uint16_t)(floor + (EVENT_TRIGGER_LEVEL / 2U) + NOISE_TRACK_MARGIN)) {
                floor = (uint16_t)(((uint32_t)floor * 15U + raw_level[ch]) / 16U);
            } else if (raw_level[ch] > floor) {
                uint16_t delta = (uint16_t)(raw_level[ch] - floor);
                uint16_t step = (uint16_t)(delta / 96U);
                if (step == 0U) {
                    step = 1U;
                }
                floor = (uint16_t)(floor + step);
            } else {
                floor = (uint16_t)(((uint32_t)floor * 31U + raw_level[ch]) / 32U);
            }
        }

        if (floor < MIC_NOISE_INIT) {
            floor = MIC_NOISE_INIT;
        }
        mic_noise_floor[ch] = floor;
    }
}

static void update_bar_scale(void)
{
    uint16_t max_level = mic_level[0];
    uint16_t target;

    if (mic_level[1] > max_level) {
        max_level = mic_level[1];
    }
    if (mic_level[2] > max_level) {
        max_level = mic_level[2];
    }

    target = (uint16_t)(max_level + (max_level / 2U) + 30U);
    if (target < BAR_FULL_SCALE_MIN) {
        target = BAR_FULL_SCALE_MIN;
    }
    if (target > BAR_FULL_SCALE_MAX) {
        target = BAR_FULL_SCALE_MAX;
    }

    if (target > mic_bar_full_scale) {
        mic_bar_full_scale = (uint16_t)(((uint32_t)mic_bar_full_scale * 3U + target) / 4U);
    } else {
        mic_bar_full_scale = (uint16_t)(((uint32_t)mic_bar_full_scale * 31U + target) / 32U);
    }
}

static void process_audio_block(uint16_t *buf, uint32_t sample_count)
{
    int32_t sum[MIC_COUNT] = {0};
    uint32_t acc[MIC_COUNT] = {0};
    uint16_t peak_abs[MIC_COUNT] = {0};
    uint32_t frames = sample_count / MIC_COUNT;
    uint16_t dc[MIC_COUNT];
    uint16_t raw_level[MIC_COUNT];
    uint8_t capture_channel = 1U;
    uint8_t hold_active = 0U;

    for (uint32_t i = 0; i < sample_count; i += MIC_COUNT) {
        sum[0] += buf[i];
        sum[1] += buf[i + 1];
        sum[2] += buf[i + 2];
    }

    for (int ch = 0; ch < MIC_COUNT; ch++) {
        dc[ch] = (uint16_t)(sum[ch] / (int32_t)frames);
    }

    for (uint32_t i = 0; i < sample_count; i += MIC_COUNT) {
        int32_t d0 = (int32_t)buf[i]     - dc[0];
        int32_t d1 = (int32_t)buf[i + 1] - dc[1];
        int32_t d2 = (int32_t)buf[i + 2] - dc[2];

        if (d0 < 0) d0 = -d0;
        if (d1 < 0) d1 = -d1;
        if (d2 < 0) d2 = -d2;

        acc[0] += (uint32_t)d0;
        acc[1] += (uint32_t)d1;
        acc[2] += (uint32_t)d2;

        if ((uint16_t)d0 > peak_abs[0]) peak_abs[0] = (uint16_t)d0;
        if ((uint16_t)d1 > peak_abs[1]) peak_abs[1] = (uint16_t)d1;
        if ((uint16_t)d2 > peak_abs[2]) peak_abs[2] = (uint16_t)d2;
    }

    for (int ch = 0; ch < MIC_COUNT; ch++) {
        uint16_t avg_abs = (uint16_t)(acc[ch] / frames);
        uint16_t transient = 0U;

        if (peak_abs[ch] > avg_abs) {
            transient = (uint16_t)(peak_abs[ch] - avg_abs);
        }

        /*
         * Blend steady energy with only part of the transient peak.
         * This stays sensitive to claps, but avoids treating normal idle spikes
         * as a permanent loud event.
         */
        raw_level[ch] = (uint16_t)(avg_abs + (transient / 8U));
    }

    update_noise_floor_from_raw(raw_level, mic_is_calibrating());

    for (int ch = 0; ch < MIC_COUNT; ch++) {
        if (raw_level[ch] > (uint16_t)(mic_noise_floor[ch] + QUIET_THRESHOLD)) {
            mic_level[ch] = (uint16_t)(raw_level[ch] - mic_noise_floor[ch]);
        } else {
            mic_level[ch] = 0U;
        }
    }
    update_bar_scale();

    update_direction_from_levels();
    if (mic_is_calibrating()) {
        update_event_engine();
        return;
    }
    if (event_state.active) {
        accumulate_event_direction();
    }
    if (event_state.active) {
        capture_channel = event_state.capture_channel;
    } else {
        capture_channel = select_capture_channel_from_levels();
    }
    hold_active = (HAL_GetTick() < event_result_hold_until_ms) ? 1U : 0U;
    if (!event_state.active && !hold_active && current_level >= EVENT_TRIGGER_LEVEL) {
        ai_event_reset();
        ai_event_capture_block(buf, sample_count, dc, capture_channel);
    } else if (event_state.active) {
        ai_event_capture_block(buf, sample_count, dc, capture_channel);
        update_live_sound_lock();
    }
    update_event_engine();
}

static uint16_t level_to_bar(uint16_t level)
{
    uint32_t scale = mic_bar_full_scale;
    uint32_t bar;

    if (scale < BAR_FULL_SCALE_MIN) {
        scale = BAR_FULL_SCALE_MIN;
    }

    bar = ((uint32_t)level * 96U) / scale;
    if (bar > 96U) {
        bar = 96U;
    }
    return (uint16_t)bar;
}

static uint8_t level_to_ring_brightness(uint16_t level)
{
    uint32_t capped = level;

    if (capped > HIGH_THREAT_LEVEL) {
        capped = HIGH_THREAT_LEVEL;
    }

    return (uint8_t)(10U + ((capped * 70U) / HIGH_THREAT_LEVEL));
}

static void ring_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= RING_LED_COUNT) {
        return;
    }

    ring_pixels[index].r = r;
    ring_pixels[index].g = g;
    ring_pixels[index].b = b;
}

static void ring_fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < RING_LED_COUNT; ++i) {
        ring_set_pixel(i, r, g, b);
    }
}

static direction_t ring_sector_for_led(uint8_t index)
{
    uint8_t third = (RING_LED_COUNT + 2U) / 3U;

    if (index < third) {
        return DIR_LEFT;
    }
    if (index < (uint8_t)(third * 2U)) {
        return DIR_CENTER;
    }
    return DIR_RIGHT;
}

static void ring_show_direction(direction_t dir, uint8_t high_threat, uint8_t brightness)
{
    uint8_t r = 0U;
    uint8_t g = 0U;
    uint8_t b = 0U;
    uint8_t dim = (uint8_t)(brightness / 8U);

    if (high_threat) {
        ring_fill(brightness, 0U, 0U);
        return;
    }

    if (dir == DIR_LEFT) {
        r = brightness;
        g = (uint8_t)(brightness / 3U);
    } else if (dir == DIR_RIGHT) {
        g = (uint8_t)(brightness / 4U);
        b = brightness;
    } else {
        g = brightness;
        r = (uint8_t)(brightness / 4U);
    }

    ring_fill(0U, 0U, dim);
    for (uint8_t i = 0; i < RING_LED_COUNT; ++i) {
        if (ring_sector_for_led(i) == dir) {
            ring_set_pixel(i, r, g, b);
        }
    }
}

static void ring_apply_status_pixel(void)
{
    if (!sd_ready) {
        ring_set_pixel(0U, 35U, 0U, 0U);
        return;
    }

    if (bt_is_connected()) {
        ring_set_pixel((uint8_t)(RING_LED_COUNT - 1U), 0U, 0U, 35U);
    } else if (bt_last_tx_ok &&
               ((HAL_GetTick() - bt_last_tx_ms) <= (BT_HEARTBEAT_MS + 1500U))) {
        ring_set_pixel((uint8_t)(RING_LED_COUNT - 1U), 0U, 0U, 16U);
    }
}

static void ring_build_pattern(uint32_t now_ms)
{
    if (mic_is_calibrating()) {
        uint8_t cursor = (uint8_t)((now_ms / 160U) % RING_LED_COUNT);
        ring_fill(2U, 2U, 0U);
        ring_set_pixel(cursor, 28U, 18U, 0U);
        ring_apply_status_pixel();
        return;
    }

    if (!sd_ready && ((now_ms / 250U) & 1U)) {
        ring_fill(35U, 0U, 0U);
        return;
    }

    if (event_state.active) {
        uint8_t brightness = level_to_ring_brightness(current_level);
        uint8_t high_threat = (current_level >= HIGH_THREAT_LEVEL) ? 1U : 0U;
        ring_show_direction(direction_idx, high_threat, brightness);
    } else if (now_ms < event_result_hold_until_ms) {
        uint8_t high_threat = (last_event_threat == THREAT_HIGH) ? 1U : 0U;
        ring_show_direction(last_event_direction, high_threat, high_threat ? 70U : 50U);
    } else if (current_level >= QUIET_THRESHOLD) {
        ring_show_direction(direction_idx, 0U, level_to_ring_brightness(current_level));
    } else {
        uint8_t pulse = (uint8_t)(3U + ((now_ms / 200U) % 5U));
        ring_fill(0U, 0U, pulse);
    }

    ring_apply_status_pixel();
}

static void ring_send_pixels(void)
{
    uint16_t pos = 0U;

    if (ring_dma_busy) {
        return;
    }

    for (uint8_t i = 0; i < RING_LED_COUNT; ++i) {
        uint8_t grb[3] = {ring_pixels[i].g, ring_pixels[i].r, ring_pixels[i].b};

        for (uint8_t byte_i = 0; byte_i < 3U; ++byte_i) {
            uint8_t value = grb[byte_i];

            for (uint8_t bit = 0; bit < 8U; ++bit) {
                ring_pwm_data[pos++] = (value & 0x80U) ? RING_PWM_ONE : RING_PWM_ZERO;
                value <<= 1;
            }
        }
    }

    while (pos < RING_PWM_BUFFER_LEN) {
        ring_pwm_data[pos++] = 0U;
    }

    ring_dma_busy = 1U;
    if (HAL_TIM_PWM_Start_DMA(&htim1,
                              TIM_CHANNEL_1,
                              (uint32_t *)ring_pwm_data,
                              RING_PWM_BUFFER_LEN) != HAL_OK) {
        ring_dma_busy = 0U;
    }
}

static void ring_light_init(void)
{
    ring_fill(0U, 0U, 0U);
    last_ring_ms = HAL_GetTick() - RING_UPDATE_MS;
}

static void ring_light_service(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_ring_ms) < RING_UPDATE_MS) {
        return;
    }
    last_ring_ms = now_ms;

    ring_build_pattern(now_ms);
    ring_send_pixels();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
        ring_dma_busy = 0U;
    }
}

static void select_oled1(void)
{
    SSD1306_SelectI2C(&hi2c1, OLED1_I2C_ADDR);
}

static void select_oled2(void)
{
    SSD1306_SelectI2C(&hi2c2, OLED2_I2C_ADDR);
}

static void draw_header(const char *title)
{
    SSD1306_DrawFilledRectangle(0, 0, 127, 10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(2, 1);
    SSD1306_Puts((char *)title, &Font_7x10, SSD1306_COLOR_BLACK);
    SSD1306_DrawLine(0, 11, 127, 11, SSD1306_COLOR_WHITE);
}

static void draw_big_centered(const char *text, uint8_t y)
{
    char shown[12];
    uint16_t len = (uint16_t)strlen(text);
    uint16_t width;
    uint16_t x;

    if (len > 11U) {
        len = 11U;
    }
    for (uint16_t i = 0; i < len; i++) {
        shown[i] = text[i];
    }
    shown[len] = '\0';

    width = (uint16_t)(len * Font_11x18.FontWidth);
    x = (width < SSD1306_WIDTH) ? (uint16_t)((SSD1306_WIDTH - width) / 2U) : 0U;

    SSD1306_GotoXY(x, y);
    SSD1306_Puts(shown, &Font_11x18, SSD1306_COLOR_WHITE);
}

static void draw_status_pill(uint8_t x, uint8_t y, uint8_t w, const char *text, uint8_t filled)
{
    SSD1306_DrawRectangle(x, y, w, 10, SSD1306_COLOR_WHITE);
    if (filled) {
        SSD1306_DrawFilledRectangle(x, y, w, 10, SSD1306_COLOR_WHITE);
    }
    SSD1306_GotoXY((uint16_t)(x + 3U), (uint16_t)(y + 1U));
    SSD1306_Puts((char *)text, &Font_7x10, filled ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
}

static void draw_dir_box(uint8_t x, uint8_t y, const char *label, uint8_t active)
{
    if (active) {
        SSD1306_DrawFilledRectangle(x, y, 29, 11, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY((uint16_t)(x + 11U), (uint16_t)(y + 1U));
        SSD1306_Puts((char *)label, &Font_7x10, SSD1306_COLOR_BLACK);
    } else {
        SSD1306_DrawRectangle(x, y, 29, 11, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY((uint16_t)(x + 11U), (uint16_t)(y + 1U));
        SSD1306_Puts((char *)label, &Font_7x10, SSD1306_COLOR_WHITE);
    }
}

static void draw_direction_strip(uint8_t y, direction_t dir)
{
    draw_dir_box(15, y, "L", (dir == DIR_LEFT));
    draw_dir_box(50, y, "C", (dir == DIR_CENTER));
    draw_dir_box(85, y, "R", (dir == DIR_RIGHT));
}

static void draw_bar_row(uint8_t y, const char *label, uint16_t level)
{
    char text[6];
    uint16_t bar = level_to_bar(level);
    uint16_t fill = (uint16_t)(((uint32_t)bar * 70U) / 96U);
    uint16_t shown_level = level;

    if (shown_level > 999U) {
        shown_level = 999U;
    }

    SSD1306_GotoXY(0, y);
    SSD1306_Puts((char *)label, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_DrawRectangle(15, (uint16_t)(y + 2U), 70, 5, SSD1306_COLOR_WHITE);
    if (fill > 0U) {
        SSD1306_DrawFilledRectangle(15, (uint16_t)(y + 2U), fill, 5, SSD1306_COLOR_WHITE);
    }
    SSD1306_DrawPixel(91, (uint16_t)(y + 4U), SSD1306_COLOR_WHITE);

    snprintf(text, sizeof(text), "%3u", shown_level);
    SSD1306_GotoXY(101, y);
    SSD1306_Puts(text, &Font_7x10, SSD1306_COLOR_WHITE);
}

static void update_servo_direction(void)
{
    uint16_t target_pulse = direction_to_pulse(direction_idx);
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_servo_ms) < SERVO_UPDATE_MS) {
        return;
    }
    last_servo_ms = now_ms;

    if (servo_pulse_current < target_pulse) {
        servo_pulse_current += SERVO_STEP_US;
        if (servo_pulse_current > target_pulse) {
            servo_pulse_current = target_pulse;
        }
    } else if (servo_pulse_current > target_pulse) {
        if (servo_pulse_current > (target_pulse + SERVO_STEP_US)) {
            servo_pulse_current -= SERVO_STEP_US;
        } else {
            servo_pulse_current = target_pulse;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo_pulse_current);
}

static void draw_mic_ui(void)
{
    char line[24];

    if (!oled1_ready) {
        return;
    }

    select_oled1();

    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (mic_is_calibrating()) {
        draw_header("ARRAY CAL");
    } else if (event_state.active) {
        snprintf(line, sizeof(line), "LIVE LV%u", (unsigned int)current_level);
        draw_header(line);
    } else {
        snprintf(line, sizeof(line), "ARRAY LV%u", (unsigned int)current_level);
        draw_header(line);
    }

    draw_direction_strip(14, direction_idx);
    draw_bar_row(29, "L", mic_level[0]);
    draw_bar_row(41, "C", mic_level[1]);
    draw_bar_row(53, "R", mic_level[2]);

    SSD1306_UpdateScreen();
}

static void draw_event_ui(void)
{
    char line[24];
    char header[20];
    const char *title = "READY";
    direction_t shown_dir = last_event_direction;

    if (!oled2_ready) {
        return;
    }

    select_oled2();
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (mic_is_calibrating()) {
        uint16_t nf0 = (mic_noise_floor[0] > 999U) ? 999U : mic_noise_floor[0];
        uint16_t nf1 = (mic_noise_floor[1] > 999U) ? 999U : mic_noise_floor[1];
        uint16_t nf2 = (mic_noise_floor[2] > 999U) ? 999U : mic_noise_floor[2];

        format_oled_time_header(header, sizeof(header));
        draw_header(header);
        draw_big_centered("QUIET", 14);

        draw_status_pill(3, 37, 39, "NOISE", 1U);

        snprintf(line, sizeof(line), "%u/%u/%u",
                 (unsigned int)nf0,
                 (unsigned int)nf1,
                 (unsigned int)nf2);
        SSD1306_GotoXY(48, 38);
        SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

        snprintf(line, sizeof(line), "BT:%s B:%s X:%lu",
                 bt_display_status(),
                 bt_baud_label(),
                 (unsigned long)(bt_tx_count % 1000UL));
        SSD1306_GotoXY(0, 53);
        SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();
        return;
    }

    format_oled_time_header(header, sizeof(header));
    draw_header(header);

    if (event_state.active) {
        uint32_t live_duration_ms = HAL_GetTick() - event_state.start_ms;
        direction_t live_dir = (direction_t)event_state.capture_channel;
        int live_class = ai_guess_live_sound_class(live_duration_ms);

        (void)live_duration_ms;
        shown_dir = live_dir;
        if (live_class == AI_FALLBACK_CLASS) {
            title = "LIVE";
        } else {
            title = ai_sound_name(live_class);
        }
    } else if ((sd_ready && sd_last_error == SD_ERR_NONE) || (sd_log_count > 0U)) {
        if (sd_log_count == 0U) {
            title = "READY";
        } else {
            title = last_sound_text;
        }
    } else {
        title = "SD ERR";
    }

    draw_big_centered(title, 13);
    draw_direction_strip(34, shown_dir);

    snprintf(line, sizeof(line), "BT:%s B:%s X:%lu",
             bt_display_status(),
             bt_baud_label(),
             (unsigned long)(bt_tx_count % 1000UL));
    SSD1306_GotoXY(0, 52);
    SSD1306_Puts(line, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_UpdateScreen();
}

static void init_oled_displays(void)
{
    select_oled1();
    oled1_ready = SSD1306_Init();

    select_oled2();
    oled2_ready = SSD1306_Init();

    draw_mic_ui();
    draw_event_ui();
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_half_ready = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_full_ready = 1;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

#if RTC_SET_TIME_ON_BOOT
  if (ds3231_set_datetime(RTC_SET_YEAR, RTC_SET_MONTH, RTC_SET_DATE,
                          RTC_SET_DAY, RTC_SET_HOUR, RTC_SET_MIN, RTC_SET_SEC) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(100);
#endif

  read_ds3231_timestamp(last_timestamp, sizeof(last_timestamp));
  sd_log_init();

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  servo_pulse_current = direction_to_pulse(direction_idx);
  update_servo_direction();

  mic_calibration_until_ms = HAL_GetTick() + MIC_CALIBRATION_MS + 500U;
  ring_light_init();
  init_oled_displays();
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_GetTick() - last_rtc_ms >= 1000U) {
      last_rtc_ms = HAL_GetTick();
      read_ds3231_timestamp(last_timestamp, sizeof(last_timestamp));
    }

    if (adc_half_ready) {
      adc_half_ready = 0;
      process_audio_block(&adc_dma_buf[0], ADC_DMA_LEN / 2);
    }

    if (adc_full_ready) {
      adc_full_ready = 0;
      process_audio_block(&adc_dma_buf[ADC_DMA_LEN / 2], ADC_DMA_LEN / 2);
    }

    if (HAL_GetTick() - last_ui_ms >= 40U) {
      last_ui_ms = HAL_GetTick();
      update_servo_direction();
    }

    if (HAL_GetTick() - last_oled_ms >= OLED_UPDATE_MS) {
      last_oled_ms = HAL_GetTick();
      draw_mic_ui();
    }

    if (HAL_GetTick() - last_oled2_ms >= OLED2_UPDATE_MS) {
      last_oled2_ms = HAL_GetTick();
      draw_event_ui();
    }

    sd_log_service();
    bt_service();
    ring_light_service();

    HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
