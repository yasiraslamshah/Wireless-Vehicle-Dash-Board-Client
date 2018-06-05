//***********************************************************************************
// Include files
//***********************************************************************************

#include <stdint.h>
#include <stdbool.h>

//***********************************************************************************
// defined files
//***********************************************************************************

//#include "gpio.h"

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************





/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

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

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#if 0
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
#include "gpio.h"

#define DISCONNECTED    0
#define SCANNING        1
#define FIND_SERVICE    2
#define FIND_CHAR       3
#define ENABLE_NOTIF    4
#define DATA_MODE       5
#define DISCONNECTING   6

//Service ID = 0f4f6a92-923a-4c56-aa5f-c9072216f7da
const uint8 serviceUUID[16] = {0xda, 0xf7, 0x16, 0x22, 0x07, 0xc9, 0x5f, 0xaa, 0x56, 0x4c, 0x3a, 0x92, 0x92, 0x6a, 0x4f, 0x0f};

const uint8 charUUID[16] = {0xdb, 0xf7, 0x16, 0x22, 0x07, 0xc9, 0x5f, 0xaa, 0x56, 0x4c, 0x3a, 0x92, 0x92, 0x6a, 0x4f, 0x0f};

//Service ID 2 =6f319e64-a317-45a0-a6c1-4c55367e190c
const uint8 serviceUUID2[16] = {0x0c, 0x19, 0x7e, 0x36, 0x55, 0x4c, 0xc1, 0xa6, 0xa0, 0x45, 0x17, 0xa3, 0x64, 0x9e, 0x31, 0x6f};

const uint8 charUUID2[16] = {0x0d, 0x19, 0x7e, 0x36, 0x55, 0x4c, 0xc1, 0xa6, 0xa0, 0x45, 0x17, 0xa3, 0x64, 0x9e, 0x31, 0x6f};

//Service ID 3 =0f2c47aa-aac8-43b2-99a3-ad6c2de3b649
const uint8 serviceUUID3[16] = {0x49, 0xb6, 0xe3, 0x2d, 0x6c, 0xad, 0xa3, 0x99, 0xb2, 0x43, 0xc8, 0xaa, 0xaa, 0x47, 0x2c, 0x0f};

const uint8 charUUID3[16] = {0x4a, 0xb6, 0xe3, 0x2d, 0x6c, 0xad, 0xa3, 0x99, 0xb2, 0x43, 0xc8, 0xaa, 0xaa, 0x47, 0x2c, 0x0f};


static uint8 _conn_handle = 0xFF;
static int _main_state;
static uint32 _service_handle;
static uint16 _char_handle;
static int state;

static void reset_variables()
{
    _conn_handle = 0xFF;
    _main_state = DISCONNECTED;
    _service_handle = 0;
    _char_handle = 0;
}

void display(char * string)
{
    GRAPHICS_Init();
    GRAPHICS_Clear();
    GRAPHICS_AppendString(string);
    GRAPHICS_Update();
}

static int process_scan_response(struct gecko_msg_le_gap_scan_response_evt_t *pResp)
{
    // decoding advertising packets is done here. The list of AD types can be found
    // at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile

    int i = 0;
    int ad_match_found = 0;
    int ad_len;
    int ad_type;

    char name[32];

    state = 0;

    while (i < (pResp->data.len - 1))
    {

        ad_len  = pResp->data.data[i];
        ad_type = pResp->data.data[i+1];

        if (ad_type == 0x08 || ad_type == 0x09 )
        {
            // type 0x08 = Shortened Local Name
            // type 0x09 = Complete Local Name
            memcpy(name, &(pResp->data.data[i+2]), ad_len-1);
            name[ad_len-1] = 0;
            display(name);

        }

        // 4880c12c-fdcb-4077-8920-a450d7f9b907
        if (ad_type == 0x06 || ad_type == 0x07)
        {
            // type 0x06 = Incomplete List of 128-bit Service Class UUIDs
            // type 0x07 = Complete List of 128-bit Service Class UUIDs
            if(memcmp(serviceUUID, &(pResp->data.data[i+2]),16) == 0)
            {
                display("Found SPP device\r\n");
                ad_match_found = 1;
            }
        }

        //jump to next AD record
        i = i + ad_len + 1;
    }

    return(ad_match_found);
}

void gpio_setup()
{
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(gpioPortF, 7, gpioModeInput, 1);
    GPIO_IntConfig(gpioPortF, 7, true, false, true);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    GPIO_PinModeSet(gpioPortF, 6, gpioModeInput, 1);
    GPIO_IntConfig(gpioPortF, 6, true, false, true);
    GPIO_IntEnable(gpioPortF);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void GPIO_EVEN_IRQHandler(void)
{
    __disable_irq();
    GPIO_IntClear(GPIO_IntGet());
    //GPIO_PinOutClear(LED1_port, LED1_pin);
    //gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_state, 5, "CLEAR");
    //lockState = 0;
    state = 1;
    GPIO_PinOutSet(LED0_port,LED0_pin);
    gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID2);
    _main_state = FIND_SERVICE;
    GPIO_IntEnable(gpioPortF);
    //gecko_external_signal(3);
    __enable_irq();
}

void GPIO_ODD_IRQHandler(void)
{
    __disable_irq();
    GPIO_IntClear(GPIO_IntGet());
    //GPIO_PinOutClear(LED1_port, LED1_pin);
    //gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_door_state, 5, "CLEAR");
    //lockState = 0;
    state = 2;
    GPIO_PinOutSet(LED1_port,LED1_pin);
    gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID3);
    _main_state = FIND_SERVICE;
    GPIO_IntEnable(gpioPortF);
    //gecko_external_signal(3);
    __enable_irq();
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

  gpio_setup();

  char printbuf[128];

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

          gecko_cmd_gatt_set_max_mtu(247);
          gecko_cmd_le_gap_discover(le_gap_discover_generic);
          //gecko_cmd_hardware_set_soft_timer(32768 * 30, 1, 0);
          _main_state = SCANNING;

          break;

      case gecko_evt_le_gap_scan_response_id:

          if(process_scan_response(&(evt->data.evt_le_gap_scan_response)) > 0)
          {
              struct gecko_msg_le_gap_open_rsp_t *pResp;

              // match found -> stop discovery and try to connect
              gecko_cmd_le_gap_end_procedure();

              pResp = gecko_cmd_le_gap_open(evt->data.evt_le_gap_scan_response.address, evt->data.evt_le_gap_scan_response.address_type);

              // make copy of connection handle for later use (for example, to cancel the connection attempt)
              _conn_handle = pResp->connection;

          }

          break;

      case gecko_evt_le_connection_opened_id:

          gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
          _main_state = FIND_SERVICE;

          break;

      case gecko_evt_le_connection_closed_id:

          reset_variables();
          gecko_cmd_le_gap_discover(le_gap_discover_generic);

          break;

      case gecko_evt_gatt_service_id:


          if(evt->data.evt_gatt_service.uuid.len == 16)
          {
              if(memcmp(serviceUUID, evt->data.evt_gatt_service.uuid.data,16) == 0)
              {
                  display("service discovered");
                  _service_handle = evt->data.evt_gatt_service.service;
              }
              else if(memcmp(serviceUUID2, evt->data.evt_gatt_service.uuid.data,16) == 0)
              {
                  display("service 2 discovered");
                  _service_handle = evt->data.evt_gatt_service.service;
              }
              else if(memcmp(serviceUUID3, evt->data.evt_gatt_service.uuid.data,16) == 0)
              {
                  display("service 3 discovered");
                  _service_handle = evt->data.evt_gatt_service.service;
              }
          }

          break;

      case gecko_evt_gatt_procedure_completed_id:

          switch(_main_state)
          {
          case FIND_SERVICE:

              if (_service_handle > 0 && state == 0)
              {
                  // Service found, next search for characteristics
                  gecko_cmd_gatt_discover_characteristics(_conn_handle, _service_handle);
                  _main_state = FIND_CHAR;
              }
              else if (_service_handle > 0 && state == 1)
              {
                  // Service found, next search for characteristics
                  display("service2found");
                  gecko_cmd_gatt_discover_characteristics(_conn_handle, _service_handle);
                  _main_state = FIND_CHAR;
              }
              else if (_service_handle > 0 && state == 2)
              {
                  // Service found, next search for characteristics
                  display("service3found");
                  gecko_cmd_gatt_discover_characteristics(_conn_handle, _service_handle);
                  _main_state = FIND_CHAR;
              }
              else
              {
                  // no service found -> disconnect
                  display("SPP service not found?");
                  gecko_cmd_endpoint_close(_conn_handle);
              }

              break;

          case FIND_CHAR:
              if (_char_handle > 0 && state == 0)
              {
                  // Char found, turn on indications
                  display("IDHAR AYA BE");
                  gecko_cmd_gatt_set_characteristic_notification(_conn_handle, _char_handle, gatt_notification);
                  _main_state = ENABLE_NOTIF;
              }
              else if (_char_handle > 0 && state == 1)
              {
                  // Char found, turn on indications
                  display("IDHAR AYA BE 2");
                  gecko_cmd_gatt_write_characteristic_value_without_response(_conn_handle, _char_handle, 1, 0x00);
                  _main_state = ENABLE_NOTIF;
                  state = 0;
                  gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
                  _main_state = FIND_SERVICE;
              }
              else if (_char_handle > 0 && state == 2)
              {
                  // Char found, turn on indications
                  uint8_t a = 0x77;
                  display("IDHAR AYA BE 3");
                  gecko_cmd_gatt_write_characteristic_value_without_response(_conn_handle, _char_handle, 1, &a);
                  _main_state = ENABLE_NOTIF;
                  state = 0;
                  gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
                  _main_state = FIND_SERVICE;
              }
              else
              {
                  // no characteristic found? -> disconnect
                  display("SPP char not found?");
                  gecko_cmd_endpoint_close(_conn_handle);
              }
              break;

          default:
              break;
          }
          break;

          break;

      case gecko_evt_gatt_characteristic_id:

          if(evt->data.evt_gatt_characteristic.uuid.len == 16 && state == 0)
          {
              if(memcmp(charUUID, evt->data.evt_gatt_characteristic.uuid.data,16) == 0)
              {
                  display("char discovered");
                  _char_handle = evt->data.evt_gatt_characteristic.characteristic;
              }
          }
          if(evt->data.evt_gatt_characteristic.uuid.len == 16 && state == 1)
          {
              if(memcmp(charUUID2, evt->data.evt_gatt_characteristic.uuid.data,16) == 0)
              {
                  display("char 2 discovered");
                  _char_handle = evt->data.evt_gatt_characteristic.characteristic;
              }
          }
          if(evt->data.evt_gatt_characteristic.uuid.len == 16 && state == 2)
          {
              if(memcmp(charUUID3, evt->data.evt_gatt_characteristic.uuid.data,16) == 0)
              {
                  display("char 3 discovered");
                  _char_handle = evt->data.evt_gatt_characteristic.characteristic;
              }
          }

          break;

      case gecko_evt_gatt_characteristic_value_id:

          if(evt->data.evt_gatt_characteristic_value.characteristic == _char_handle)
          {
              // data received from SPP server -> print to UART
              // NOTE: this works only with text (no binary) because printf() expects null-terminated strings as input
              display("INDICATION AYA BE");
              memcpy(printbuf, evt->data.evt_gatt_characteristic_value.value.data, evt->data.evt_gatt_characteristic_value.value.len);
              printbuf[evt->data.evt_gatt_characteristic_value.value.len] = 0;
              if(printbuf[0]==1)
              {
                  //GPIO_PinOutSet(LED1_port, LED1_pin);
                  display("Shock");
              }
              else if(printbuf[0]==2)
              {
                  //GPIO_PinOutSet(LED0_port,LED0_pin);
                  display("Knock");
              }
              else if(printbuf[0]==0)
              {
                  //GPIO_PinOutClear(LED1_port,LED1_pin);
                  display("Clear");
              }
              //gecko_cmd_endpoint_close(_conn_handle);
              gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
              _main_state = FIND_SERVICE;
          }
          break;

      case gecko_evt_hardware_soft_timer_id:

           if(evt->data.evt_hardware_soft_timer.handle == 1)
           {
               state = 1;
               GPIO_PinOutSet(LED0_port,LED0_pin);
               gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID2);
               _main_state = FIND_SERVICE;
           }

           break;

      default:
        break;
    }
  }
}
#endif


#if 0  case gecko_evt_gatt_characteristic_value_id:

          if(evt->data.evt_gatt_characteristic_value.characteristic == _char_handle && state==0)
          { memset(printbuf,0,evt->data.evt_gatt_characteristic_value.value.len);
              // data received from SPP server -> print to UART
              // NOTE: this works only with text (no binary) because printf() expects null-terminated strings as input
              memcpy(printbuf, evt->data.evt_gatt_characteristic_value.value.data, evt->data.evt_gatt_characteristic_value.value.len);
              printbuf[evt->data.evt_gatt_characteristic_value.value.len] = 0;
              display(printbuf);


/*
          char buf1[16] = {0};
         	        	 	  snprintf(buf1,16,"\n LUX1:%d\n",printbuf[0]);
         	        	 	 GRAPHICS_AppendString(buf1);
         	        	 	  GRAPHICS_Update();

         	        	 	    /*if(printbuf[0]>50)
								  {
									 display("\nLIGHT OFF\n");
								  }
								  else {
									 display("\nLIGHT ONN\n");
								  }*/

/*
         	        	 	    GRAPHICS_Update();*/
         	        	 	    state=1;
         	        	 	  gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
         	        	 	                _main_state = FIND_SERVICE;
          }
          else if(evt->data.evt_gatt_characteristic_value.characteristic == _char_handle && state==1)
		{


        	  memset(printbuf,0,evt->data.evt_gatt_characteristic_value.value.len);
        	  memcpy(printbuf, evt->data.evt_gatt_characteristic_value.value.data, evt->data.evt_gatt_characteristic_value.value.len);
        	                printbuf[evt->data.evt_gatt_characteristic_value.value.len] = 0;
        	                display(printbuf);

				/*	char buf2[16] = {0};
				  snprintf(buf2,16,"\n HUM1:%d\n",printbuf[1]);
				  GRAPHICS_AppendString(buf2);
				  GRAPHICS_Update();
					char buf1[16] = {0};
				  snprintf(buf1,16,"\n HUM1:%d\n",printbuf[2]);
				  GRAPHICS_AppendString(buf1);
				  GRAPHICS_Update();
				 uint16_t LSB=(uint16_t)printbuf[0];
				 uint16_t MSB=(uint16_t)printbuf[1];
				 uint16_t HUM=(LSB)|(MSB<<8);
				/* char buf1[16] = {0};
				 snprintf(buf1,16,"\n HUM2:%d\n",HUM);
			     GRAPHICS_AppendString(buf1);
				  /*snprintf(buf1,16,"\n HUM2:%d\n",printbuf[2]);
				  GRAPHICS_AppendString(buf1);
				  snprintf(buf1,16,"\n HUM3:%d\n",printbuf[3]);
					GRAPHICS_AppendString(buf1);*/
					/*if(printbuf[0]>30)
							  {
								 display("\nFAN ONN\n");
							  }
							  else {
								 display("\nFAN OFF\n");
							  }*/


/*GRAPHICS_Update();*/
	state=2;
	gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
	              _main_state = FIND_SERVICE;
}
          else if(evt->data.evt_gatt_characteristic_value.characteristic == _char_handle && state==2)
          {
        	  memcpy(printbuf, evt->data.evt_gatt_characteristic_value.value.data, evt->data.evt_gatt_characteristic_value.value.len);
                  	                printbuf[evt->data.evt_gatt_characteristic_value.value.len] = 0;
                  	              display(printbuf);


				/*	char buf3[16] = {0};
				  snprintf(buf3,16,"\n MOVE1:%d\n",printbuf[3]);
				  GRAPHICS_AppendString(buf3);
				//char buf3[16] = {0};
				  snprintf(buf3,16,"\n MOVE2:%c\n",printbuf[0]);
				  GRAPHICS_AppendString(buf3);
			//	char buf3[16] = {0};
				  snprintf(buf3,16,"\n MOVE3:%s\n",printbuf[0]);
				  GRAPHICS_AppendString(buf3);



          GRAPHICS_Update();*/
          	state=0;
		  /*if(printbuf[0]>0)
		  {
			 display("\nACCELERATING\n");
		  }
		  else {
			 display("\nNON_ACCELERATING\n");}
		  }*/
          	gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, 16, serviceUUID);
          	              _main_state = FIND_SERVICE;}

				break;
#endif



