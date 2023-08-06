/**
 ******************************************************************************
 * @file      config.h
 * @author    Neat Tech
 * @brief     Configuration file
 *
 *
 ******************************************************************************/


#ifndef SRC_CONFIG_INTERFACE_H_
#define SRC_CONFIG_INTERFACE_H_


#define FIRMWARE_VERSION "inno-sphere v0.1"
#define FW_VER 1

#define DISALBE_WDT
#define NO_DISPLAY
//#define NO_MEMORY
//#define NO_BUTTONS
//#define NO_LEDS
//#define NO_CHRG_STATE
//#define IGNORE_IMPEDANCE
//#define VIRTUAL_CAP
//#define DISABLE_OUTPUT_15V

//#define DISPLAY_DEBUG_ENABLE  //to enable display debig via uart, the errors will be written any way
//#define MEMORY_DEBUG_ENABLE
//#define STIMULATION_DEBUG_ENABLE

//#define DISABLE_ANALOG_THRESHOLD_TESTS //TODO: need more developing
#define ADC_DEBUG_ENABLE_DEFAULT 0


//TODO: remove internal pull down for push buttons

#endif /* SRC_CONFIG_INTERFACE_H_ */
