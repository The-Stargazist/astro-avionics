#include "BMP.h"

BMP::BMP(TwoWire& wire, uint8_t addr) {
    _wire = &wire;
    _addr = addr;
}

bool BMP::begin() {
    // Initialize sensor on given I2C bus
    if (!bmp.begin_I2C(_addr, _wire)) {
        return false;
    }

    // Configure sensor (Taken from drone example in Bosch BMP390 datasheet)
    // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // Allowed values 2,4,8,16. We dont want to oversample temp 
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1); // 0 - 127 set as 1 to reduce lag
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);  // 50Hz update rate

    // time to measure = time for pressure + time for temp + filtering
    // each sample takes ~ 2 - 2.5ms

    // Adafruit says initial reading is not accurate - discarding this data
    if (!bmp.performReading()) return false;
    delay(30);

    // Take initial reading to establish baseline
    if (!bmp.performReading()) return false;

    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0f;

    altitude = bmp.readAltitude(seaLevelPressure);

    // Store ground reference (IMPORTANT for flight logic)
    groundAltitude = altitude;

    lastAltitude = altitude;
    lastTime = millis();

    return true;
}

void BMP::read() {
    unsigned long now = millis();
    unsigned long dt_ms = now - lastTime;

    // Limit update rate to sensor ODR (~50Hz)
    // Calling faster than this just adds noise
    if (dt_ms < 20) return;
    
    if (!bmp.performReading()) return;

    float dt = dt_ms / 1000.0f;

    // Update raw sensor values
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0f;

    // Compute absolute altitude (WARNING: depends on sea level pressure)
    altitude = bmp.readAltitude(seaLevelPressure);

    // Compute vertical velocity (m/s)
    velocity = (altitude - lastAltitude) / dt;

    lastAltitude = altitude;
    lastTime = now;
}

void BMP::setSeaLevelPressure(float p) {
    // Set this BEFORE begin() ideally
    // Incorrect value will shift altitude readings significantly
    seaLevelPressure = p;
}

float BMP::getTemperature() {
    return temperature;
}

float BMP::getPressure() {
    return pressure;
}

float BMP::getAltitude() {
    // WARNING: Absolute altitude (not relative to ground)
    return altitude;
}

float BMP::getRelativeAltitude() {
    // Use this for flight logic (liftoff, apogee, etc.)
    return altitude - groundAltitude;
}

float BMP::getVelocity() {
    // Filtered vertical velocity (m/s)
    return velocity;
}
