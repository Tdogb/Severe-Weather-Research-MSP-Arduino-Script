#include "MSP.h"
#include "Adafruit_LPS35HW.h"
#include "Adafruit_SHT4x.h"
#include "ms4525do.h"

#define HUMIDITY_HZ 1
#define PRESSURE_HZ 75
#define AIRSPEED_HZ 100
#define MSP_HZ 100

MSP msp;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
bfs::Ms4525do pres;
msp_set_custom_sensors_t payload;

void setup() {
  Serial.begin(115200);
  msp.begin(Serial);
  
  if (!sht4.begin()) {
    Serial.println("Error communicating with SHT4");
    while (1) delay(1);
  } else {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
  }

  if (!lps35hw.begin_I2C()) {
    Serial.println("Couldn't find LPS35HW chip");
    while (1);
  } else {
    lps35hw.setDataRate(LPS35HW_RATE_75_HZ);
    lps35hw.resetPressure(); //Absolute pressure mode
  }
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while(1){}
  }
  payload.humidity = 0;
  payload.temp_SHT = 0;
  payload.pressure_lps = 0;
  payload.temp_lps = 0;
  payload.differential_pressure_up = 0;
  payload.up_die_temp = 0;
  payload.differential_pressure_forward = 0;
  payload.forward_die_temp = 0;
  payload.differential_pressure_side = 0;
  payload.side_die_temp = 0;
}

void loop() {
  static uint32_t last_time_sht = millis();
  static uint32_t last_time_lps = millis();
  static uint32_t last_time_airspeed = millis();
  static uint32_t last_time_msp = millis();

  uint32_t start_time = millis();
  if (start_time - last_time_sht > 1000/HUMIDITY_HZ) {
    sensors_event_t _humidity, _tempSHT;
    sht4.getEvent(&_humidity, &_tempSHT);
    payload.humidity = sht4.get_rh_ticks_raw();
    payload.temp_SHT = sht4.get_t_ticks_raw();
    last_time_sht = start_time;
  }
  if (start_time - last_time_lps > 1000/PRESSURE_HZ) {\
    payload.pressure_lps = lps35hw.readPressureRaw();
    payload.temp_lps = lps35hw.readTemperatureRaw();
    last_time_lps = start_time;
  }
  if (start_time - last_time_airspeed > 1000/AIRSPEED_HZ) {
    if (pres.Read()) {
      payload.differential_pressure_up = pres.pres_counts();
      payload.up_die_temp = pres.die_temp_counts();
    }
    last_time_airspeed = start_time;
  }
  if (start_time - last_time_msp > 1000/MSP_HZ) {
    msp.command(MSP_SET_CUSTOM_SENSORS, &payload, sizeof(payload));
    last_time_msp = start_time;
  }
}
