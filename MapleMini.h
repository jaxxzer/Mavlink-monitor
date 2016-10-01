#ifndef MAPLE_MINI_H
#define MAPLE_MINI_H

#define VCC 3.3f
#define ADC_RESOLUTION 4096
#define ADC_VOLTAGE_SCALAR (VCC / ADC_RESOLUTION)

// Onboard LEDS
#define PIN_LED_MAPLE 33
#define PIN_LED_1 15
#define PIN_LED_2 16
#define PIN_LED_3 17

// Power module battery monitoring
#define PIN_ADC_VOLTAGE 10
#define PIN_ADC_CURRENT 11

// Onboard dip switch
#define PIN_DIP_0 21 //LSB
#define PIN_DIP_1 20
#define PIN_DIP_2 19
#define PIN_DIP_3 18 //MSB

// Additional sensor inputs
#define PIN_ADC_0 3
#define PIN_ADC_1 4
#define PIN_ADC_2 5

// Pins for Rangefinder_Ping
#define PIN_ECHO 30
#define PIN_TRIGGER 31

#define PIN_BUTTON 32

#define PIN_WATERDETECTOR PIN_ADC_0
#define PIN_TEMPSENSOR PIN_ADC_1



#endif
