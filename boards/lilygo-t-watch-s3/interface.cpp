#include "core/powerSave.h"
#include "core/utils.h"
#include <Wire.h>
#include <XPowersLib.h>
#include <interface.h>

XPowersAXP2101 axp192;
#include <TouchDrvFT6X36.hpp>
TouchDrvFT6X36 touch;

// Haptic
#include "SensorDRV2605.hpp"
SensorDRV2605 drv;

/***************************************************************************************
** Function name: _setup_gpio()
** Location: main.cpp
** Description:   initial setup for the device
***************************************************************************************/
void _setup_gpio() {
    pinMode(16, INPUT); // Touch IRQ

    // --- I2C bus recovery ---
    // The new ESP32 arduino core (i2c-ng / IDF 5.x i2c_master driver) returns
    // ESP_ERR_INVALID_STATE on every transmit if SDA/SCL are held low at boot.
    // The old core tolerated this; the new one refuses. Manually clock the bus
    // free (9 pulses) and release the lines before letting the drivers begin().
    auto i2c_recover = [](int sda, int scl) {
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, OUTPUT_OPEN_DRAIN);
        digitalWrite(scl, HIGH);
        delayMicroseconds(10);
        for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
            digitalWrite(scl, LOW);
            delayMicroseconds(10);
            digitalWrite(scl, HIGH);
            delayMicroseconds(10);
        }
        // generate a STOP condition
        pinMode(sda, OUTPUT_OPEN_DRAIN);
        digitalWrite(sda, LOW);
        delayMicroseconds(10);
        digitalWrite(scl, HIGH);
        delayMicroseconds(10);
        digitalWrite(sda, HIGH);
        delayMicroseconds(10);
        pinMode(sda, INPUT);
        pinMode(scl, INPUT);
    };
    i2c_recover(10, 11); // sensor bus (PMU/RTC/haptic)
    i2c_recover(39, 40); // touch bus
    delay(10);

    _rtc.setWire(&Wire); // Cplus uses Wire1 default, the lib had been changed to accept setting I2C bus
                         // StickCPlus uses BM8563 that is the same as PCF8536
    axp192.init(Wire, 10, 11); // first (and only) begin() on the sensor bus (SDA10/SCL11)
    axp192.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
    axp192.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_900MA);
    axp192.setSysPowerDownVoltage(2600);
    axp192.setALDO1Voltage(3300);
    axp192.setALDO2Voltage(3300);
    axp192.setALDO3Voltage(3300);
    axp192.setALDO4Voltage(3300);
    axp192.setBLDO2Voltage(3300);
    axp192.setDC3Voltage(3300);
    axp192.enableDC3(); // gps
    axp192.disableDC2();
    axp192.disableDC4();
    axp192.disableDC5();
    axp192.disableBLDO1();
    axp192.disableCPUSLDO();
    axp192.disableDLDO1();
    axp192.disableDLDO2();
    axp192.enableALDO1(); //! RTC VBAT
    axp192.enableALDO2(); //! TFT BACKLIGHT   VDD
    axp192.enableALDO3(); //! Screen touch VDD
    axp192.enableALDO4(); //! Radio VDD
    axp192.enableBLDO2(); //! drv2605 enable
    //  Set the time of pressing the button to turn off
    axp192.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    // Set the button power-on press time
    axp192.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    axp192.disableTSPinMeasure();
    // Enable internal ADC detection
    axp192.enableBattDetection();
    axp192.enableVbusVoltageMeasure();
    axp192.enableBattVoltageMeasure();
    axp192.enableSystemVoltageMeasure();
    // t-watch no chg led
    axp192.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    axp192.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Enable the required interrupt function
    axp192.enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |    // BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |     // POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ // CHARGE
    );

    // Clear all interrupt flags
    axp192.clearIrqStatus();
    // Set the precharge charging current
    axp192.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    // Set constant current charge current limit
    axp192.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_300MA);
    // Set stop charging termination current
    axp192.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    // Set charge cut-off voltage
    axp192.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V35);
    // Set RTC Battery voltage to 3.3V
    axp192.setButtonBatteryChargeVoltage(3300);
    axp192.enableButtonBatteryCharge();

    touch.begin(Wire1, FT6X36_SLAVE_ADDRESS, 39, 40);
    touch.setSwapXY(true);
    touch.interruptPolling();

    // Disable RF and NRF Menus for default
    bruceConfig.disabledMenus.push_back("RF");
    bruceConfig.disabledMenus.push_back("NRF24");

    // Haptic driver
    if (!drv.begin(Wire, 10, 11)) {
        Serial.println("Failed to find DRV2605.");
    } else {
        Serial.println("Init DRV2605 Sensor success!");
        drv.selectLibrary(1);
        drv.setMode(SensorDRV2605::MODE_INTTRIG);
        drv.useERM();

        // Startup buzz
        drv.setWaveform(0, 70);
        drv.setWaveform(1, 0);
        drv.run();
    }
    bruceConfigPins.gpsBaudrate = 38400;
}

/***************************************************************************************
** Function name: _post_setup_gpio()
** Location: main.cpp
** Description:   second stage gpio setup to make a few functions work
***************************************************************************************/
void _post_setup_gpio() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    ledcAttach(TFT_BL, TFT_BRIGHT_FREQ, TFT_BRIGHT_Bits);
    ledcWrite(TFT_BL, 255);
}

/***************************************************************************************
** Function name: getBattery()
** location: display.cpp
** Description:   Delivers the battery value from 1-100
***************************************************************************************/
int getBattery() {
    int percent = axp192.getBatteryPercent();
    return percent;
}

/*********************************************************************
** Function: setBrightness
** location: settings.cpp
** set brightness value
**********************************************************************/
void _setBrightness(uint8_t brightval) {
    int dutyCycle;
    if (brightval == 100) dutyCycle = 255;
    else if (brightval == 75) dutyCycle = 130;
    else if (brightval == 50) dutyCycle = 70;
    else if (brightval == 25) dutyCycle = 20;
    else if (brightval == 0) dutyCycle = 0;
    else dutyCycle = ((brightval * 255) / 100);

    // log_i("dutyCycle for bright 0-255: %d", dutyCycle);
    ledcWrite(TFT_BL, dutyCycle);
}

bool getTouched() { return digitalRead(16) == LOW; }
struct TP {
    int16_t x[1], y[1];
};
/*********************************************************************
** Function: InputHandler
** Handles the variables PrevPress, NextPress, SelPress, AnyKeyPress and EscPress
**********************************************************************/
// Swipe + tap navigation (replaces the invisible 3x3 tap-zone grid).
//   swipe up    -> next item      (DownPress)
//   swipe down  -> previous item  (UpPress)
//   tap         -> select         (SelPress)
//   swipe right -> back / exit    (EscPress)
// UpPress/DownPress are used (not Prev/NextPress) because loopOptions() runs a
// button-hold spinner loop on PrevPress that would stall on a quick swipe.
void InputHandler(void) {
    static bool touching = false;
    static int sx = 0, sy = 0;         // gesture start point
    static int cx = 0, cy = 0;         // latest point during the gesture
    static unsigned long lastFire = 0; // debounce between gestures

    TP t;

    if (getTouched()) {
        touch.getPoint(t.x, t.y, 1);
        if (bruceConfigPins.rotation == 3) { t.y[0] = (tftHeight + 20) - t.y[0]; }
        if (bruceConfigPins.rotation == 0) {
            int tmp = t.x[0];
            t.x[0] = tftWidth - t.y[0];
            t.y[0] = tftHeight - tmp;
        }
        if (bruceConfigPins.rotation == 2) {
            int tmp = t.x[0];
            t.x[0] = t.y[0];
            t.y[0] = tmp;
        }
        if (bruceConfigPins.rotation == 1) { t.x[0] = tftWidth - t.x[0]; }

        // A touch while the screen is asleep just wakes it; don't treat as a gesture.
        if (wakeUpScreen()) {
            touching = false;
            return;
        }
        AnyKeyPress = true;

        if (!touching) {
            touching = true;
            sx = t.x[0];
            sy = t.y[0];
        }
        cx = t.x[0];
        cy = t.y[0];

        // Keep the raw point live for direct-touch screens (on-screen keyboard, etc.)
        touchPoint.x = t.x[0];
        touchPoint.y = t.y[0];
        touchPoint.pressed = true;
    } else if (touching) {
        // Finger lifted -> classify the gesture.
        touching = false;
        touchPoint.pressed = false;

        int dx = cx - sx;
        int dy = cy - sy;
        int adx = dx < 0 ? -dx : dx;
        int ady = dy < 0 ? -dy : dy;

        const int TAP_MAX = 30;   // total movement <= this  => tap
        const int SWIPE_MIN = 45; // movement >= this along an axis => swipe

        if (millis() - lastFire < 120) return; // debounce

        if (adx <= TAP_MAX && ady <= TAP_MAX) {
            SelPress = true; // tap -> select highlighted item
        } else if (ady >= adx) {
            if (dy < -SWIPE_MIN) DownPress = true;     // swipe up   -> next item
            else if (dy > SWIPE_MIN) UpPress = true;   // swipe down -> previous item
            else return;                               // movement too small, ignore
        } else {
            if (dx > SWIPE_MIN) EscPress = true;       // swipe right -> back / exit
            else if (dx < -SWIPE_MIN) SelPress = true; // swipe left -> select (alt)
            else return;                               // movement too small, ignore
        }

        // CRITICAL: mark AnyKeyPress so the input task holds the flag (~75ms) instead
        // of clearing it on the next ~10ms tick before the menu can read it.
        AnyKeyPress = true;
        lastFire = millis();

        Serial.printf(
            "[NAV] dx=%d dy=%d -> %s\n", dx, dy,
            DownPress ? "NEXT(swipe-up)"
                      : UpPress   ? "PREV(swipe-down)"
                      : SelPress  ? "SELECT(tap)"
                      : EscPress  ? "BACK(swipe-right)"
                                  : "?"
        );
    }
}

/*********************************************************************
** Function: powerOff
** location: mykeyboard.cpp
** Turns off the device (or try to)
**********************************************************************/
void powerOff() {}

/*********************************************************************
** Function: checkReboot
** location: mykeyboard.cpp
** Btn logic to turn off the device (name is odd btw)
**********************************************************************/
void checkReboot() {}

/***************************************************************************************
** Function name: isCharging()
** Description:   Determines if the device is charging
***************************************************************************************/
bool isCharging() {
    return axp192.isCharging(); // Return the charging status from AXP
}
