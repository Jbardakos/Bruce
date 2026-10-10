#include "step_counter.h"

#ifdef T_WATCH_S3
#include "core/display.h"
#include "core/mykeyboard.h"
#include <SensorBMA423.hpp>
#include <Wire.h>
#include <globals.h>

// The BMA423 accelerometer sits on the shared sensor I2C bus (SDA=10, SCL=11),
// which is already brought up (with bus-recovery) during _setup_gpio() at boot.
static SensorBMA423 bma;
static bool bmaReady = false;

static bool initBMA() {
    if (bmaReady) return true;
    if (!bma.begin(Wire, BMA423_I2C_ADDR_SECONDARY, 10, 11)) return false;
    bma.configAccelerometer();
    bma.enableAccelerometer();
    bma.enablePedometer();
    bma.enableFeature(SensorBMA423::FEATURE_STEP_CNTR, 1);
    bmaReady = true;
    return true;
}

void stepCounter() {
    drawMainBorderWithTitle("STEP COUNTER");

    if (!initBMA()) {
        padprintln("");
        padprintln("BMA423 sensor not found.");
        padprintln("");
        padprintln("Press SEL or ESC to exit.");
        while (!check(EscPress) && !check(SelPress)) delay(50);
        return;
    }

    padprintln("");
    padprintln("SEL = reset");
    padprintln("ESC = exit");

    const int cy = tftHeight / 2;
    uint32_t last = 0xFFFFFFFFUL;

    while (true) {
        if (check(EscPress)) break;
        if (check(SelPress)) {
            bma.resetPedometer();
            last = 0xFFFFFFFFUL;
        }

        uint32_t steps = bma.getPedometerCounter();
        if (steps != last) {
            last = steps;
            tft.fillRect(0, cy - 28, tftWidth, 56, bruceConfig.bgColor);
            char buf[16];
            snprintf(buf, sizeof(buf), "%lu", (unsigned long)steps);
            tft.setTextSize(4);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            int w = tft.textWidth(buf);
            tft.setCursor((tftWidth - w) / 2, cy - 16);
            tft.print(buf);
        }
        delay(200);
    }
}
#else
void stepCounter() {}
#endif
