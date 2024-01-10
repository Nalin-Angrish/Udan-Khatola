#include <Wire.h>
#include <MS5611.h>

class Barometer {
private:
  const int MS5611_ADDR = 0x77;
  MS5611 ms5611;
  float basePressure;
  float temperature;

public:
  void setup(){
    // ==================== Initialize the MS5611 Module ==================== //
    ms5611 = MS5611(MS5611_ADDR);
    ms5611.begin();
    basePressure = ms5611.getPressure();
    temperature = ms5611.getTemperature() + 273.15; // in Kelvin
  }
  double getRelativeAltitude(){
    // Assumptions:
    // - Universal Gas Constant (R) = 8.314 J/(K mol)
    // - Mean Molar Mass (M) of the atmospheric gases = 0.02896 kg/mol 
    // - Acceleration due to gravity (g) = 9.8 m/s^2
    // - The aircraft is in troposphere, which lets us assume temperature 
    //   changes linearly according to the relation:
    //       T(h) = T(h0) - a*(h - h0)
    //   where a = 0.0065 K/m
    //   It can also be noted that this won't be a huge factor for a small
    //   remote control aircraft and we can neglect this.

    // The math can be understood from the file:
    //   docs/Altitude Measurement from Pressure Sensor.pdf

    // The final mathematical formula to obtain relative height from base becomes:
    // h = (T/a)*[1 - (p/p0)^(R*a/M*g)]
    // Substituting values of all constants, we get:
    // h = (T/0.0065)*[1 - (p/p0)^(0.1902632)]
    float pressure = ms5611.getPressure();
    double relativeAltitude = (temperature/0.0065)*(1 - pow((pressure/basePressure), 0.1902632));
    return relativeAltitude;
  }
};