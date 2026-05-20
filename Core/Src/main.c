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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIC_COUNT            3
#define BLOCK_FRAMES         32
#define ADC_DMA_LEN          (MIC_COUNT * BLOCK_FRAMES * 2)

#define SERVO_LEFT_US        1100
#define SERVO_CENTER_US      1500
#define SERVO_RIGHT_US       1900

#define QUIET_THRESHOLD      5
#define EVENT_TRIGGER_LEVEL  20
#define EVENT_RELEASE_LEVEL  10
#define EVENT_END_GAP_MS     80
#define EVENT_FORCE_END_MS   700
#define EVENT_RESULT_HOLD_MS 4000
#define IMPULSE_MAX_MS       420
#define HIGH_THREAT_LEVEL    70
#define HIGH_THREAT_MS       900
#define DIRECTION_MARGIN     8
#define DIRECTION_HOLD_MS    20
#define DIRECTION_CAPTURE_MS 180
#define DIR_LEFT_GAIN_PCT    128
#define DIR_CENTER_GAIN_PCT  78
#define DIR_RIGHT_GAIN_PCT   128
#define SERVO_STEP_US        90
#define SERVO_UPDATE_MS      5
#define OLED_UPDATE_MS       60
#define SD_SYNC_INTERVAL_MS  5000
#define SD_SYNC_EVENT_COUNT  4
#define AI_WINDOW_SIZE       128
#define AI_WINDOW_STRIDE     1
#define AI_SAMPLE_SCALE      2048.0f
#define AI_FALLBACK_CLASS    STARTER_MODEL_CLASS_COUNT
#define MIC_NOISE_INIT       8
#define NOISE_TRACK_MARGIN   10

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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

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
uint32_t last_rtc_ms = 0;
uint32_t last_servo_ms = 0;
uint32_t event_result_hold_until_ms = 0;

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

char last_event_text[20] = "BOOT";
char last_timestamp[24] = "RTC-NA";
char last_sound_text[12] = "OTHER";
uint8_t last_sound_class = AI_FALLBACK_CLASS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
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

static const char *dir_short(direction_t dir)
{
    if (dir == DIR_LEFT) return "L";
    if (dir == DIR_RIGHT) return "R";
    return "C";
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

    /*
     * Lightweight heuristic classifier for the actual hardware.
     * This is intentionally conservative because the saved 1-NN model was only
     * a small starter model and did not generalize well to the live setup.
     */
    if ((duration_ms <= IMPULSE_MAX_MS) || (event_type == EVT_IMPULSE)) {
        if (ai_is_knock_like(features, duration_ms)) {
            return 1; /* KNOCK */
        }

        if (ai_is_glass_like(features, duration_ms)) {
            return 2; /* GLASS */
        }

        if (ai_is_clap_like(features, duration_ms)) {
            return 0; /* CLAP */
        }

        if (model_guess == 0) {
            return 0;
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
}

static void update_event_engine(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (!event_state.active) {
        if (now_ms < event_result_hold_until_ms) {
            return;
        }
        if (current_level >= EVENT_TRIGGER_LEVEL) {
            event_state.active = 1;
            event_state.start_ms = now_ms;
            event_state.last_loud_ms = now_ms;
            event_state.peak_level = current_level;
            event_state.peak_direction = direction_idx;
            event_state.type = EVT_IDLE;
            event_state.threat = THREAT_LOW;
            event_state.capture_channel = select_capture_channel_from_levels();
            event_state.preview_sound_class = AI_FALLBACK_CLASS;
            event_state.preview_sound_hits = 0U;
            event_state.locked_sound_class = AI_FALLBACK_CLASS;
            memset(event_state.direction_sum, 0, sizeof(event_state.direction_sum));
            memset(event_state.direction_peak, 0, sizeof(event_state.direction_peak));
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
        raw_level[ch] = (uint16_t)(avg_abs + (transient / 3U));
    }

    for (int ch = 0; ch < MIC_COUNT; ch++) {
        if (raw_level[ch] > mic_noise_floor[ch]) {
            mic_level[ch] = (uint16_t)(raw_level[ch] - mic_noise_floor[ch]);
        } else {
            mic_level[ch] = 0U;
        }
    }

    if (!event_state.active) {
        for (int ch = 0; ch < MIC_COUNT; ch++) {
            uint16_t track_limit = (uint16_t)(mic_noise_floor[ch] + EVENT_TRIGGER_LEVEL + NOISE_TRACK_MARGIN);

            if (raw_level[ch] <= track_limit) {
                mic_noise_floor[ch] = (uint16_t)(((uint32_t)mic_noise_floor[ch] * 15U + raw_level[ch]) / 16U);
            }

            if (mic_noise_floor[ch] < MIC_NOISE_INIT) {
                mic_noise_floor[ch] = MIC_NOISE_INIT;
            }
        }
    }

    update_direction_from_levels();
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
    uint32_t bar = level / 3U;
    if (bar > 96U) {
        bar = 96U;
    }
    return (uint16_t)bar;
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
    char line1[20];
    char line2[20];
    char line3[20];
    uint16_t bar_l = level_to_bar(mic_level[0]);
    uint16_t bar_c = level_to_bar(mic_level[1]);
    uint16_t bar_r = level_to_bar(mic_level[2]);
    uint8_t hold_active = (HAL_GetTick() < event_result_hold_until_ms) ? 1U : 0U;

    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (last_timestamp[0] == '2') {
        snprintf(line1, sizeof(line1), "%c%c:%c%c:%c%c N:%lu",
                 last_timestamp[11], last_timestamp[12],
                 last_timestamp[14], last_timestamp[15],
                 last_timestamp[17], last_timestamp[18],
                 (unsigned long)sd_log_count);
    } else {
        snprintf(line1, sizeof(line1), "RTC ERR N:%lu", (unsigned long)sd_log_count);
    }
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(line1, &Font_7x10, SSD1306_COLOR_WHITE);

    (void)hold_active;
    snprintf(line2, sizeof(line2), "DIR:%s LV:%3u", dir_short(direction_idx), current_level);

    SSD1306_GotoXY(0, 10);
    SSD1306_Puts(line2, &Font_7x10, SSD1306_COLOR_WHITE);

    if (event_state.active) {
        uint32_t live_duration_ms = HAL_GetTick() - event_state.start_ms;
        event_type_t live_type = event_type_from_duration(live_duration_ms);
        direction_t live_dir = (direction_t)event_state.capture_channel;
        int live_class = ai_guess_live_sound_class(live_duration_ms);

        if (live_class == AI_FALLBACK_CLASS) {
            snprintf(line3, sizeof(line3), "LIVE %s %s",
                     dir_short(live_dir),
                     event_short(live_type));
        } else {
            snprintf(line3, sizeof(line3), "LIVE %s %s %s",
                     ai_sound_name(live_class),
                     dir_short(live_dir),
                     event_short(live_type));
        }
    } else if (sd_ready && sd_last_error == SD_ERR_NONE) {
        if (sd_log_count == 0U) {
            snprintf(line3, sizeof(line3), "WAIT FOR SOUND");
        } else {
            snprintf(line3, sizeof(line3), "#%lu %s %s %s",
                     (unsigned long)sd_log_count,
                     last_sound_text,
                     dir_short(last_event_direction),
                     event_short(last_event_type));
        }
    } else {
        snprintf(line3, sizeof(line3), "SD:E%u %s",
                 (unsigned int)sd_last_error,
                 event_state.active ? "LIVE" : "CHECK");
    }

    SSD1306_GotoXY(0, 20);
    SSD1306_Puts(line3, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(0, 31);
    SSD1306_Puts("L", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_DrawRectangle(20, 33, 96, 5, SSD1306_COLOR_WHITE);
    if (bar_l > 0) {
        SSD1306_DrawFilledRectangle(20, 33, bar_l, 5, SSD1306_COLOR_WHITE);
    }

    SSD1306_GotoXY(0, 41);
    SSD1306_Puts("C", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_DrawRectangle(20, 43, 96, 5, SSD1306_COLOR_WHITE);
    if (bar_c > 0) {
        SSD1306_DrawFilledRectangle(20, 43, bar_c, 5, SSD1306_COLOR_WHITE);
    }

    SSD1306_GotoXY(0, 51);
    SSD1306_Puts("R", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_DrawRectangle(20, 53, 96, 5, SSD1306_COLOR_WHITE);
    if (bar_r > 0) {
        SSD1306_DrawFilledRectangle(20, 53, bar_r, 5, SSD1306_COLOR_WHITE);
    }

    SSD1306_UpdateScreen();
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
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

  SSD1306_Init();
  draw_mic_ui();
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

    sd_log_service();

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  ADC_ChannelConfTypeDef sConfig = {0};

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

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
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
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
