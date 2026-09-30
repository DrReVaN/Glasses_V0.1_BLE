#include "glasses_app.h"
#include "glasses_ota.h"
#include "main.h"
#include "STM32_Cap1203.h"
#include "ssd1306.h"
#include "ble.h"
#include <stdio.h>
#include <string.h>
#if CFG_LPM_SUPPORTED != 0
#error "Local HAL_GetTick clock requires SysTick to keep running; use CPU1 Sleep"
#endif

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
volatile uint8_t glasses_touch_irq;
GlassesRx glasses_rx;
GlassesClock glasses_clock;
static GlassesTouch touch;
static IWDG_HandleTypeDef watchdog;
/* Retained across software/watchdog resets and exposed in the diagnostics. */
__attribute__((section(".noinit"))) uint32_t glasses_fault[3];
uint32_t glasses_reset_flags;
static uint32_t touch_at, draw_at, activity_at, ready_at, vibrate_at, scroll_at;
static uint32_t pair_at, pair_window, ota_at;
static uint16_t pair_handle;
static uint32_t pair_value;
static bool connected, off, display_power, display_ready, dirty, cap_ready;
static bool pairing, ota_requested;
static char message[GLASSES_TEXT_SIZE];
static size_t scroll;
static bool reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static void display_off(void) {
    if (display_ready) ssd1306_SetDisplayOn(0);
    HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_RESET);
    display_power = display_ready = false;
}
static void wake(uint32_t now) { off = false; activity_at = now; dirty = true; }
void Glasses_WatchdogInit(void) {
    glasses_reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    watchdog.Instance = IWDG; watchdog.Init.Prescaler = IWDG_PRESCALER_256;
    watchdog.Init.Reload = 1023; watchdog.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) NVIC_SystemReset();
}
void Glasses_Init(void) {
    uint32_t now = HAL_GetTick();
    cap_ready = CAP1203_Init(&hi2c1) == HAL_OK;
    touch_at = now; activity_at = now; pair_window = now + 60000;
    dirty = true; message[0] = 0;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}
void Glasses_Connected(bool value) {
    connected = value; glasses_rx_reset(&glasses_rx);
    if (!value) { Glasses_PairingDone(); Glasses_OtaDisconnected(); }
    dirty = true;
}
bool Glasses_PairingRequest(uint16_t handle, uint32_t value) {
    uint32_t now = HAL_GetTick();
    if (pairing || reached(now, pair_window) || value > 999999) return false;
    pair_handle = handle; pair_value = value; pair_at = now;
    pairing = true; wake(now); return true;
}
void Glasses_PairingDone(void) { pairing = false; dirty = true; }
void Glasses_RequestOta(void) {
#ifndef SMARTGLASSES_BOOTLOADER
    ota_at = HAL_GetTick(); ota_requested = true; wake(ota_at);
#endif
}
void Glasses_Fatal(uint32_t code) {
    glasses_fault[0] = 0x53474631; glasses_fault[1] = code;
    glasses_fault[2] = __get_IPSR(); __DSB(); NVIC_SystemReset();
}
static void render(void) {
    char line[16];
    ssd1306_Fill(Black);
    if (pairing) {
        snprintf(line, sizeof(line), "%06lu", (unsigned long)pair_value);
        ssd1306_SetCursor(6, 56); ssd1306_WriteString(line, Font_6x8, White);
        ssd1306_SetCursor(2, 68); ssd1306_WriteString("1Yes3No", Font_6x8, White);
    } else if (ota_requested) {
        ssd1306_SetCursor(8, 56); ssd1306_WriteString("OTA?", Font_7x10, White);
        ssd1306_SetCursor(2, 68); ssd1306_WriteString("1Yes3No", Font_6x8, White);
    } else if (message[0]) {
        /* Keep the five-character window used by the working V1 firmware.
         * The RAM dimensions do not identify the panel's visible area. */
        char window[6] = {0}; size_t n = strlen(message), i;
        for (i = 0; i < 5 && scroll + i < n; ++i) window[i] = message[scroll + i];
        ssd1306_SetCursor(4, 61); ssd1306_WriteString(window, Font_7x10, White);
    } else {
#ifdef SMARTGLASSES_BOOTLOADER
        ssd1306_SetCursor(5, 56); ssd1306_WriteString("Update", Font_7x10, White);
        ssd1306_SetCursor(5, 70); ssd1306_WriteString(connected ? "BLE OK" : "Pair", Font_7x10, White);
#else
        if (glasses_clock.valid) snprintf(line, sizeof(line), "%02u:%02u", glasses_clock.hour, glasses_clock.minute);
        else strcpy(line, "--:--");
        ssd1306_SetCursor(6, 58); ssd1306_WriteString(line, Font_6x8, White);
        if (glasses_clock.valid) snprintf(line, sizeof(line), "%02u.%02u", glasses_clock.day, glasses_clock.month);
        else strcpy(line, "--.--");
        ssd1306_SetCursor(6, 68); ssd1306_WriteString(line, Font_6x8, White);
        if (!connected) { ssd1306_SetCursor(10, 82); ssd1306_WriteString("BLE?", Font_6x8, White); }
#endif
    }
    ssd1306_UpdateScreen();
}
void Glasses_Process(void) {
    uint32_t now = HAL_GetTick();
    GlassesTouchAction action = GLASSES_TOUCH_NONE;
    HAL_IWDG_Refresh(&watchdog);
    glasses_clock_tick(&glasses_clock, now); glasses_rx_expire(&glasses_rx, now);
    if (reached(now, touch_at)) {
        uint8_t pads;
        touch_at = now + (cap_ready ? 50 : 1000);
        /* Polling handles held ALERT# too. ISR never mutates application states. */
        glasses_touch_irq = 0;
        if (!cap_ready) {
            HAL_I2C_DeInit(&hi2c1); HAL_I2C_Init(&hi2c1);
            cap_ready = CAP1203_Init(&hi2c1) == HAL_OK;
        }
        if (cap_ready && CAP1203_ReadTouch(&pads) == HAL_OK) {
            action = glasses_touch_poll(&touch, pads, now);
            if (!off && pads && !display_power) wake(now);
        } else { cap_ready = false; memset(&touch, 0, sizeof(touch)); }
    }
    if (pairing && (uint32_t)(now - pair_at) >= 30000) {
        aci_gap_numeric_comparison_value_confirm_yesno(pair_handle, NO); Glasses_PairingDone();
    }
    if (ota_requested && (uint32_t)(now - ota_at) >= 30000) { ota_requested = false; dirty = true; }
    if (action == GLASSES_PAIR) { pair_window = now + 60000; wake(now); }
    else if (pairing && (action == GLASSES_HOME || action == GLASSES_MESSAGE)) {
        aci_gap_numeric_comparison_value_confirm_yesno(pair_handle, action == GLASSES_HOME ? YES : NO);
        Glasses_PairingDone();
    } else if (ota_requested && (action == GLASSES_HOME || action == GLASSES_MESSAGE)) {
        ota_requested = false; dirty = true;
        if (action == GLASSES_HOME) Glasses_OtaReboot();
    } else if (action == GLASSES_POWER) {
        if (off) wake(now);
        else { off = true; message[0] = 0; display_off(); __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); }
    } else if (!off && action == GLASSES_HOME) { message[0] = 0; wake(now); }
    else if (!off && action == GLASSES_MESSAGE && message[0]) { scroll = 0; scroll_at = now + 350; wake(now); }
#ifndef SMARTGLASSES_BOOTLOADER
    if (!off && !pairing && !ota_requested && !message[0] && glasses_rx_pop(&glasses_rx, message)) {
        scroll = 0; scroll_at = now + 1000; vibrate_at = now + 100;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 50); wake(now);
    }
    if (message[0] && !pairing && !ota_requested && reached(now, scroll_at)) {
        scroll_at = now + 350; dirty = true;
        if (++scroll >= strlen(message)) message[0] = 0;
        activity_at = now;
    }
#endif
    if (reached(now, vibrate_at)) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    if (off) return;
    if (!pairing && !ota_requested && !message[0] && (uint32_t)(now - activity_at) >= 15000) { display_off(); return; }
    if (!display_power) {
        HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_SET);
        display_power = true; ready_at = now + 100; return;
    }
    if (!display_ready) {
        if (!reached(now, ready_at)) return;
        ssd1306_Init(); ssd1306_SetContrast(50);
        display_ready = ssd1306_GetStatus() == HAL_OK; dirty = true;
    }
    if (display_ready && (dirty || reached(now, draw_at))) { render(); draw_at = now + 500; dirty = false; }
    if (ssd1306_GetStatus() != HAL_OK) {
        display_off(); HAL_SPI_DeInit(&hspi1); HAL_SPI_Init(&hspi1);
        display_power = true; ready_at = now + 1000;
        HAL_GPIO_WritePin(OLED_PWR_GPIO_Port, OLED_PWR_Pin, GPIO_PIN_SET);
    }
}
