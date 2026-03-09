#include "Peripherals/Loadcells.h"
#include "Peripherals/IO_Map.h"
#include "Comms.h"
#include <Arduino.h>

HX711 loadcell1, loadcell2, loadcell3;

// ---------------------------------------------------------------------------
// Calibration helpers
// ---------------------------------------------------------------------------

static void wait_for_enter() {
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read(); // flush
}

static float read_known_weight() {
    Serial.println("  Enter known weight in grams, then press ENTER:");
    while (!Serial.available()) delay(10);
    float w = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    return w;
}

static void calibrate_one(HX711 &lc, const char *name) {
    Serial.print("\n-- ");
    Serial.print(name);
    Serial.println(" --");
    Serial.print("  Place a known weight on ");
    Serial.print(name);
    Serial.println(", then press ENTER...");
    wait_for_enter();

    float known_weight = read_known_weight();
    if (known_weight <= 0.0f) {
        Serial.println("  Invalid weight, skipping.");
        return;
    }

    lc.set_scale(1.0f); // reset scale so get_value() returns raw counts
    double raw = lc.get_value(20);
    float scale = (float)raw / known_weight;
    lc.set_scale(scale);

    Serial.print("  Calibration factor : ");
    Serial.println(scale, 4);
    Serial.print("  Verification       : ");
    Serial.print(lc.get_units(10));
    Serial.println(" g");
}

int loadcells_setup(void)
{

    loadcell1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    #if MY_ID == LIFT_THRUST_ID
        loadcell2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
        loadcell3.begin(LOADCELL3_DOUT_PIN, LOADCELL3_SCK_PIN);
    #endif
    
    loadcell1.tare(10); // tare with 10 samples
    #if MY_ID == LIFT_THRUST_ID
        loadcell2.tare(10);
        loadcell3.tare(10);
    #endif

    loadcell1.set_scale(LOADCELL1_SCALE);
    #if MY_ID == LIFT_THRUST_ID
        loadcell2.set_scale(LOADCELL2_SCALE);
        loadcell3.set_scale(LOADCELL3_SCALE);
    #endif
    return 0;
}

int read_loadcells(data_t *data)
{
    data->loadcells.loadcell1 = loadcell1.get_value();
    #if MY_ID == LIFT_THRUST_ID
        data->loadcells.loadcell2 = loadcell2.get_value();
        data->loadcells.loadcell3 = loadcell3.get_value();
    #endif
    return 0;
}

// ---------------------------------------------------------------------------
// Interactive serial calibration
// Procedure:
//   1. Remove all weight -> ENTER  (tares all loadcells)
//   2. For each loadcell: place known weight -> ENTER, type grams -> ENTER
//   3. Factors are printed — hardcode them as set_scale() in loadcells_setup()
// ---------------------------------------------------------------------------
void calibrate_loadcells() {
    Serial.println("\n==============================");
    Serial.println("   LOADCELL CALIBRATION");
    Serial.println("==============================");
    Serial.println("Step 1: Remove ALL weight from every loadcell, then press ENTER...");
    wait_for_enter();

    Serial.println("Taring (20 samples)...");
    loadcell1.tare(20);
    #if MY_ID == LIFT_THRUST_ID
        loadcell2.tare(20);
        loadcell3.tare(20);
    #endif
    Serial.println("Tare complete.");

    Serial.println("\nStep 2: Calibrate each loadcell with a known weight.");

    calibrate_one(loadcell1, "Loadcell 1");
    #if MY_ID == LIFT_THRUST_ID
        calibrate_one(loadcell2, "Loadcell 2");
        calibrate_one(loadcell3, "Loadcell 3");
    #endif

    Serial.println("\n==============================");
    Serial.println("   CALIBRATION COMPLETE");
    Serial.println("==============================");
    Serial.println("Hardcode the factors above with set_scale() in loadcells_setup().");
    Serial.println("Example:  loadcell1.set_scale(<factor>);");
}
