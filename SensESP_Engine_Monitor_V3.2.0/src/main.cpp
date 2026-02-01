/*
 * Apres Engine Monitor using SensESP V3
 * Copyright (c) 2026 Jason Greenwood 
 * https://www.youtube.com/@ApresSail
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Apres Engine Monitor using SensESP V3.
// Software version 3.2.0 - 20260118
// Changes for version 3.2.0
// 1. Add Alternator Current Sensing
//  a. Alternator Voltage, Current, Power
//  b. INA219
//
#include <memory>
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <Arduino.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/frequency.h"

using namespace sensesp;
using namespace sensesp::onewire;
//
//Fuel Rate lookup
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to M^3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, M^3/s));
    add_sample(CurveInterpolator::Sample(500, 0.00000473125946250));
    add_sample(CurveInterpolator::Sample(1000, 0.00000630834595000));
    add_sample(CurveInterpolator::Sample(1500, 0.00001261669190000));
    add_sample(CurveInterpolator::Sample(1800, 0.00001577086487500));
    add_sample(CurveInterpolator::Sample(2000, 0.00001955587244500));
    add_sample(CurveInterpolator::Sample(2100, 0.00002207921082500));
    add_sample(CurveInterpolator::Sample(2500, 0.00004100424867500));
    add_sample(CurveInterpolator::Sample(2600, 0.00004731259462500));
    add_sample(CurveInterpolator::Sample(2800, 0.00005677511355000));
    add_sample(CurveInterpolator::Sample(3000, 0.00006939180545000));
    add_sample(CurveInterpolator::Sample(3200, 0.00008831684330000));
    add_sample(CurveInterpolator::Sample(3400, 0.00010724188115000));  
  }
};
  // Oil Pressure lookup
class PressureInterpreter : public CurveInterpolator {
 public:
  PressureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our Pressure sender to Pascal
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownPascal));
    add_sample(CurveInterpolator::Sample(10, 0));
    add_sample(CurveInterpolator::Sample(21, 50000));
    add_sample(CurveInterpolator::Sample(31, 100000));
    add_sample(CurveInterpolator::Sample(42, 150000));
    add_sample(CurveInterpolator::Sample(52, 200000));
    add_sample(CurveInterpolator::Sample(71, 300000));
    add_sample(CurveInterpolator::Sample(90, 400000));
    add_sample(CurveInterpolator::Sample(107, 500000));
    add_sample(CurveInterpolator::Sample(124, 600000)); 
    add_sample(CurveInterpolator::Sample(140, 700000));
    add_sample(CurveInterpolator::Sample(156, 800000));
    add_sample(CurveInterpolator::Sample(163, 850000));
    add_sample(CurveInterpolator::Sample(170, 900000));
    add_sample(CurveInterpolator::Sample(184, 1000000)); 
  }
};
// BME280
  
    Adafruit_BME280 bme280;

  float read_temp_callback() { return (bme280.readTemperature() + 273.15);}
  float read_pressure_callback() { return (bme280.readPressure());}
  float read_humidity_callback() { return (bme280.readHumidity());}

// INA219
  Adafruit_INA219 ina219_A;

  const float RshuntA = 0.0005; //Vishay WSMS2908 Series from Newark 500 Micro Ohms -0.0005ohms

  //float read_A_current_callback() { return (ina219_A.getCurrent_mA() / 1000);}
  float read_A_current_callback() { return ((ina219_A.getShuntVoltage_mV() / 1000) / RshuntA);}
  float read_A_shuntvoltage_callback() { return (ina219_A.getShuntVoltage_mV() / 1000);} //drop across the shunt
  float read_A_busvoltage_callback() { return (ina219_A.getBusVoltage_V());} //downstream voltage to be supplied to the consumer
  float read_A_loadvoltage_callback() { return (ina219_A.getBusVoltage_V() + (ina219_A.getShuntVoltage_mV() / 1000));} //loadvoltage is the battery or supply voltage
  float read_A_power_callback() { return ((ina219_A.getBusVoltage_V() + (ina219_A.getShuntVoltage_mV() / 1000)) * (ina219_A.getCurrent_mA() / 1000));}
//INA219 end

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  Wire.begin(21,22);                // join i2c bus (address optional for master)

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Apres_Engine_Monitor_V3.2.0")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

/// 1-Wire Temp Sensors///
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(25); //digital 2

 // Oil temp (fasten to oil P sensor) - /propulsion/engine/oilTemperature
 auto oil_temp =
 new OneWireTemperature(dts, 5000, "/oilTemperature/oneWire");

ConfigItem(oil_temp)
 ->set_title("Oil Temperature")
 ->set_description("Temperature of the engine oil")
 ->set_sort_order(100);

auto oil_temp_calibration =
 new Linear(1.0, 0.0, "/oilTemperature/linear");

ConfigItem(oil_temp_calibration)
 ->set_title("Oil Temperature Calibration")
 ->set_description("Calibration for the oil temperature sensor")
 ->set_sort_order(200);

auto oil_temp_sk_output = new SKOutputFloat(
 "propulsion.engine.oilTemperature", "/oilTemperature/skPath");

ConfigItem(oil_temp_sk_output)
 ->set_title("oil Temperature Signal K Path")
 ->set_description("Signal K path for the oil temperature")
 ->set_sort_order(300);

oil_temp->connect_to(oil_temp_calibration)
 ->connect_to(oil_temp_sk_output);

// Aft Cabin temp - /environment/inside/aftCabin/temperature                             
auto aft_cabin_temp =
new OneWireTemperature(dts, 5000, "/aft_cabinTemperature/oneWire");

ConfigItem(aft_cabin_temp)
->set_title("Aft Cabin Temperature")
->set_description("Temperature of the Aft Cabin")
->set_sort_order(100);

auto aft_cabin_temp_calibration =
new Linear(1.0, 0.0, "/aft_cabinTemperature/linear");

ConfigItem(aft_cabin_temp_calibration)
->set_title("Aft Cabin Temperature Calibration")
->set_description("Calibration for the Aft Cabin temperature sensor")
->set_sort_order(200);

auto aft_cabin_temp_sk_output = new SKOutputFloat(
"environment.inside.aft_cabinTemperature", "/aft_cabinTemperature/skPath");

ConfigItem(aft_cabin_temp_sk_output)
->set_title("Aft Cabin Temperature Signal K Path")
->set_description("Signal K path for the Aft Cabin temperature")
->set_sort_order(300);

aft_cabin_temp->connect_to(aft_cabin_temp_calibration)
->connect_to(aft_cabin_temp_sk_output);

// Exhaust Elbow Temp sensor - /propulsion/engine/intakeManifoldTemperature
auto elbow_temp =
new OneWireTemperature(dts, 5000, "/elbowTemperature/oneWire");

ConfigItem(elbow_temp)
->set_title("elbow Temperature")
->set_description("Temperature of the engine exhaust elbow")
->set_sort_order(100);

auto elbow_temp_calibration =
new Linear(1.0, 0.0, "/elbowTemperature/linear");

ConfigItem(elbow_temp_calibration)
->set_title("Exhaust Elbow Temperature Calibration")
->set_description("Calibration for the elbow temperature sensor")
->set_sort_order(200);

auto elbow_temp_sk_output = new SKOutputFloat(
"propulsion.engine.intakeManifoldTemperature", "/elbowTemperature/skPath");

ConfigItem(elbow_temp_sk_output)
->set_title("Exhaust Elbow Temperature Signal K Path")
->set_description("Signal K path for the elbow temperature")
->set_sort_order(300);

elbow_temp->connect_to(elbow_temp_calibration)
->connect_to(elbow_temp_sk_output);

// Exhaust barrel sensor - /propulsion/engine/exhaustTemperature
auto exhaust_temp =
new OneWireTemperature(dts, 5000, "/exhaustTemperature/oneWire");

ConfigItem(exhaust_temp)
->set_title("exhaust Temperature")
->set_description("Temperature of the engine exhaust")
->set_sort_order(100);

auto exhaust_temp_calibration =
new Linear(1.0, 0.0, "/exhaustTemperature/linear");

ConfigItem(exhaust_temp_calibration)
->set_title("Exhaust Temperature Calibration")
->set_description("Calibration for the exhaust temperature sensor")
->set_sort_order(200);

auto exhaust_temp_sk_output = new SKOutputFloat(
"propulsion.engine.exhaustTemperature", "/exhaustTemperature/skPath");

ConfigItem(exhaust_temp_sk_output)
->set_title("Exhaust Temperature Signal K Path")
->set_description("Signal K path for the exhaust temperature")
->set_sort_order(300);

exhaust_temp->connect_to(exhaust_temp_calibration)
->connect_to(exhaust_temp_sk_output);

// Alternator Temperature - /electrical/alternator/temperature
auto alternator_temp =
new OneWireTemperature(dts, 5000, "/alternatorTemperature/oneWire");

ConfigItem(alternator_temp)
->set_title("alternator Temperature")
->set_description("Temperature of the engine alternator")
->set_sort_order(100);

auto alternator_temp_calibration =
new Linear(1.0, 0.0, "/alternatorTemperature/linear");

ConfigItem(alternator_temp_calibration)
->set_title("alternator Temperature Calibration")
->set_description("Calibration for the alternator temperature sensor")
->set_sort_order(200);

auto alternator_temp_sk_output = new SKOutputFloat(
"electrical.alternator.temperature", "/alternatorTemperature/skPath");

ConfigItem(alternator_temp_sk_output)
->set_title("alternator Temperature Signal K Path")
->set_description("Signal K path for the alternator temperature")
->set_sort_order(300);

alternator_temp->connect_to(alternator_temp_calibration)
->connect_to(alternator_temp_sk_output);

//RPM Application/////

const char* config_path_calibrate = "/Engine RPM/calibrate";
const char* config_path_skpath = "/Engine RPM/sk_path";
const float multiplier = 1.0;

auto* sensor = new DigitalInputCounter(16, INPUT_PULLUP, RISING, 500);

sensor->connect_to(new Frequency(multiplier, config_path_calibrate))  
// connect the output of sensor to the input of Frequency()
        ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath));  
        // connect the output of Frequency() to a Signal K Output as a number

sensor->connect_to(new Frequency(6))
// times by 6 to go from Hz to RPM
         ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
        ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/Engine Fuel/sk_path"));
// 5) Direct copy from old Main.ccp - End          

// 6) Direct copy from old Main.ccp - Start
/// BME280 SENSOR CODE - Temp/Humidity/Altitude/Pressure Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bme280.begin();
  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* bme280_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* bme280_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  auto* bme280_humidity = 
      new RepeatSensor<float>(60000, read_humidity_callback);     

  // Send the temperature to the Signal K server as a Float
  bme280_temp->connect_to(new SKOutputFloat("environment.inside.engineBay.temperature"));

  bme280_pressure->connect_to(new SKOutputFloat("environment.inside.engineBay.pressure"));

  bme280_humidity->connect_to(new SKOutputFloat("environment.inside.engineBay.relativeHumidity"));

///////////////////////////INA219 start
  ina219_A.begin();  // Initialize first board (default address 0x40)

//ina219_A - Alternator
  auto* ina219_A_current =
      new RepeatSensor<float>(1000, read_A_current_callback);

  auto* ina219_A_shuntvoltage = 
      new RepeatSensor<float>(1000, read_A_shuntvoltage_callback);

  auto* ina219_A_busvoltage = 
      new RepeatSensor<float>(1000, read_A_busvoltage_callback);   

  auto* ina219_A_loadvoltage = 
      new RepeatSensor<float>(1000, read_A_loadvoltage_callback); 

    auto* ina219_A_power = 
      new RepeatSensor<float>(1000, read_A_power_callback);    

  // Send the temperature to the Signal K server as a Float
  ina219_A_current->connect_to(new SKOutputFloat("electrical.alternators.A.current"));

  ina219_A_loadvoltage->connect_to(new SKOutputFloat("electrical.alternators.A.voltage"));

  //ina219_A_shuntvoltage->connect_to(new SKOutputFloat("electrical.alternators.A.shuntv"));

  //ina219_A_busvoltage->connect_to(new SKOutputFloat("electrical.alternators.A.vin-v"));

  ina219_A_power->connect_to(new SKOutputFloat("electrical.alternators.A.power"));

//// Pressure Sender Config ////

const float Vin = 3.45;
const float R1 = 47.0;
auto* analog_input = new AnalogInput(36, 500); //- Pin 36 is Analogue 0

analog_input->connect_to(new AnalogVoltage(Vin,Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Pressure/sender"))
      ->connect_to(new PressureInterpreter("/Engine Pressure/curve"))
      ->connect_to(new Linear(1.0, 0.0, "/Engine Pressure/calibrate"))
      ->connect_to(new SKOutputFloat("propulsion.engine.oilPressure", "/Engine Pressure/sk_path"));

//// Bilge Monitor /////
auto* bilge = new DigitalInputState(13, INPUT_PULLUP, 5000);  //- Pin 13 is Digital 7

	auto int_to_string_function = [](int input) ->String {
		 if (input == 1) {
		   return "Water present!";
		 } 
		 else { // input == 0
		   return "bilge clear";
		 }
	};

auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

bilge->connect_to(int_to_string_transform)
      ->connect_to(new SKOutputString("propulsion.engine.bilge"));

bilge->connect_to(new SKOutputFloat("propulsion.engine.bilge.raw"));

}

void loop() { event_loop()->tick(); }
