/*/***************************************************************************//**
 * @Yasir Aslam Shah
 * @IOT 2018 Spring
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 */
/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
//#include "main.h"
#include "em_gpio.h"
#include "graphics.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
/* Device initialization header */
#include "hal-config.h"

//#include "gpio.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

#include "graphics.h"

#define DISCONNECTED    0
#define SCANNING        1
#define FIND_SERVICE    2
#define FIND_CHAR       3
#define ENABLE_NOTIF    4
#define DATA_MODE       5
#define DISCONNECTING   6

//variables
char display_number[7];
uint32 display_uint32;
uint8_t BLE_CONNECT=0xFF;
static bool bonded = false;
uint8_t connect = 0xFF;
//service UUID-LUX VALUES
const uint8 serviceUUID[16] = {0x63, 0x89, 0xab, 0x71, 0xb7, 0x76, 0x41, 0x8e, 0xe7, 0x4c, 0x3b, 0x7f, 0x18, 0x9d, 0x1b, 0x93};
// light char UUID: 99 2a d5 8a- 0e 34- 40 bf- a4 3e- 5a 31 43 61 67 d3
const uint8 charUUID[16] = {0xd3, 0x67, 0x61, 0x43, 0x31, 0x5a, 0x3e, 0xa4, 0xbf, 0x40, 0x34, 0x0e, 0x8a, 0xd5, 0x2a, 0x99};

static uint8 _conn_handle = 0xFF;
static int _main_state;
static uint32 _service_handle;
static uint16 _char_handle;



static void reset_variables()
{
    _conn_handle = 0xFF;
    _main_state = DISCONNECTED;
    _service_handle = 0;
    _char_handle = 0;
}
//FUNCTION TO DISPLAY ON lcd
void display_lcd(char * string)
{
    GRAPHICS_Init();
    GRAPHICS_Clear();
    GRAPHICS_AppendString(string);
    GRAPHICS_Update();
}
void gpio_setup()
{
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(gpioPortF, 7, gpioModeInput, 1);
    GPIO_IntConfig(gpioPortF, 7, true, false, true);
    GPIO_PinModeSet(gpioPortF, 6, gpioModeInput, 1);
    GPIO_IntConfig(gpioPortF, 6, true, false, true);
    GPIO_IntEnable(gpioPortF);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    GPIO_DriveStrengthSet(gpioPortF, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(gpioPortF, gpioDriveStrengthWeakAlternateWeak);
	GPIO_DriveStrengthSet(gpioPortF, gpioDriveStrengthWeakAlternateWeak);//LED1 IS SET WITH WEAK DRIVE STRENGTH
	GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);//LED default is off as false
	GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);//LED default is off as false
}
static int process_scan_response(struct gecko_msg_le_gap_scan_response_evt_t *pResp)
{
    int i = 0;
    int ad_match_found = 0;
    int ad_len;
    int ad_type;
    char name[32];

    while (i < (pResp->data.len - 1))
    {
        ad_len  = pResp->data.data[i];
        ad_type = pResp->data.data[i+1];

        if (ad_type == 0x08 || ad_type == 0x09 )
        {
            memcpy(name, &(pResp->data.data[i+2]), ad_len-1);
            name[ad_len-1] = 0;
        }


        if (ad_type == 0x06 || ad_type == 0x07)
        {
            if(memcmp(serviceUUID, &(pResp->data.data[i+2]),16) == 0)
            {
                ad_match_found = 1;
            }
        }
        //jump to next AD record
        i = i + ad_len + 1;
    }
    return(ad_match_found);
}

void main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Initialize stack
  gecko_init(&config);
  void gpio_setup();
  char printbuf[128];
  //gpio_init();
//initialise LCD
  GRAPHICS_Init();

  while (1) {

    struct gecko_cmd_packet* evt;


    evt = gecko_wait_event();
    switch (BGLIB_MSG_ID(evt->header)) {

      case gecko_evt_system_boot_id:
    	  	  	gecko_cmd_sm_delete_bondings();
    	      	gecko_cmd_sm_configure(0x07, sm_io_capability_displayyesno);
    	      	gecko_cmd_sm_set_bondable_mode(1);
				gecko_cmd_gatt_set_max_mtu(247);
				gecko_cmd_le_gap_discover(le_gap_discover_generic);
				_main_state = SCANNING;

                break;

      case gecko_evt_sm_confirm_passkey_id:
    	  	  	display_uint32 = evt->data.evt_sm_confirm_passkey.passkey;
            	snprintf(display_number, sizeof(display_number), "%ld?", display_uint32);
            	display(display_number);
            	unsigned int pass_check = 5;
            	while((pass_check = GPIO_PinInGet(gpioPortF,6)) != 0);
            	gecko_cmd_sm_passkey_confirm(BLE_CONNECT, 1);

            	break;

      case gecko_evt_sm_bonded_id:
            	break;

      case gecko_evt_sm_bonding_failed_id:
                break;

      case gecko_evt_le_gap_scan_response_id:
          if(process_scan_response(&(evt->data.evt_le_gap_scan_response)) > 0)
          {
              struct gecko_msg_le_gap_open_rsp_t *pResp;
              gecko_cmd_le_gap_end_procedure();
              pResp = gecko_cmd_le_gap_open(evt->data.evt_le_gap_scan_response.address, evt->data.evt_le_gap_scan_response.address_type);
              _conn_handle = pResp->connection;
          }

          break;

      case gecko_evt_le_connection_opened_id:
			  connect = evt->data.evt_le_connection_opened.connection;
			  gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
			  _main_state = FIND_SERVICE;

          break;

      case gecko_evt_le_connection_closed_id:
    	  connect = 0xFF;
          reset_variables();
          gecko_cmd_le_gap_discover(le_gap_discover_generic);

          break;

      case gecko_evt_gatt_service_id:
          if(evt->data.evt_gatt_service.uuid.len == 16)
          {
              if(memcmp(serviceUUID, evt->data.evt_gatt_service.uuid.data,16) == 0)
              {
                  _service_handle = evt->data.evt_gatt_service.service;
              }
          }

          break;

      case gecko_evt_gatt_procedure_completed_id:

          switch(_main_state)
          {
          case FIND_SERVICE:

              if (_service_handle > 0)
              {
                  gecko_cmd_gatt_discover_characteristics(_conn_handle, _service_handle);
                  _main_state = FIND_CHAR;
              }
              else
              {
                  gecko_cmd_endpoint_close(_conn_handle);
              }

              break;

          case FIND_CHAR:
              if (_char_handle > 0)
              {
                  gecko_cmd_gatt_set_characteristic_notification(_conn_handle, _char_handle, gatt_notification);
                  _main_state = ENABLE_NOTIF;
              }
              else
              {
                  gecko_cmd_endpoint_close(_conn_handle);
              }
              break;

          default:
              break;
          }
          break;
          break;

      case gecko_evt_gatt_characteristic_id:
          if(evt->data.evt_gatt_characteristic.uuid.len == 16)
          {
              if(memcmp(charUUID, evt->data.evt_gatt_characteristic.uuid.data,16) == 0)
              {
                  _char_handle = evt->data.evt_gatt_characteristic.characteristic;
              }
          }

      break;

      case gecko_evt_gatt_characteristic_value_id:
          if(evt->data.evt_gatt_characteristic_value.characteristic == _char_handle)
          {
              memcpy(printbuf, evt->data.evt_gatt_characteristic_value.value.data, evt->data.evt_gatt_characteristic_value.value.len);
              printbuf[evt->data.evt_gatt_characteristic_value.value.len] = 0;
					  if(printbuf[0]=='A')
					  {
						GRAPHICS_AppendString("\n\nCLIENT");
						GRAPHICS_Update();display_lcd("\n\n\n\n   DAYLIGHT!");
					  }else
						{
						GRAPHICS_AppendString("\n\nCLIENT");
						GRAPHICS_Update();
						display_lcd("\n\n\n\n    NIGHT!");
						} GRAPHICS_Clear();
          }
      break;

      default:
        break;
    }
  }
}
