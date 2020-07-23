
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_log.h"

// SWY
#include "umile_uv.h"

//static nrf_saadc_value_t adc_buf;         //!< Buffer used for storing ADC value.
nrf_saadc_value_t adc_result;

#define SAMPLES_IN_BUFFER 1


static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];

#define TICKS_100_MS APP_TIMER_TICKS(100) //!< Tick count for 100ms.
APP_TIMER_DEF(m_adc_timer_id);

static uint16_t f_uv_frame_rts;
static uint8_t 	m_adc_channel_enabled; 
static nrf_saadc_channel_config_t  channel_0_config;
static nrf_saadc_channel_config_t  channel_1_config;


#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 270  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_12BIT                  4096 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) * ADC_PRE_SCALING_COMPENSATION)

static uint16_t          m_adc_lvl_in_milli_volts; //!< Current battery level.


typedef PACKED_STRUCT
{
    uint16_t          vbatt;
    uint16_t          adc1;
    uint16_t          packet_counter;
} es_my_frame_t;

static es_my_frame_t  m_umile_uv_frame;

/*
*/
static void update_adv_cnt(void)
{
    m_umile_uv_frame.packet_counter++;
}

static void adc_app_timer_handler(void* p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_drv_saadc_sample();
}

/*
typedef PACKED_STRUCT
{
    uint16_t          vbatt;
    uint16_t          adc1;
    uint8_t           packet_counter;
} es_my_frame_t;
*/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        static ret_code_t err_code;
        uint8_t  m_adc_channel_get_data;
        uint16_t    *pVal;

        m_adc_channel_get_data = m_adc_channel_enabled;

            if(m_adc_channel_enabled == 0)
            {
                    err_code = nrf_drv_saadc_channel_uninit(0);
                    APP_ERROR_CHECK(err_code);
                    pVal = &m_umile_uv_frame.vbatt;
                    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
                    APP_ERROR_CHECK(err_code);
                    m_adc_channel_enabled = 1;
            }
            else // if(m_adc_channel_enabled == 1)
            {
                    err_code = nrf_drv_saadc_channel_uninit(1);
                    APP_ERROR_CHECK(err_code);
                    pVal = &m_umile_uv_frame.adc1;
                    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
                    APP_ERROR_CHECK(err_code);
                    m_adc_channel_enabled = 0;
            }
     
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        static char tx_message[128];

        adc_result = p_event->data.done.p_buffer[0];
        m_adc_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        //    ADC_RESULT_IN_MILLI_VOLTS(adc_result) + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        //*pVal = p_event->data.done.p_buffer[0];
        *pVal = m_adc_lvl_in_milli_volts;

		//printf("Channel %d value: %d\r\n", m_adc_channel_get_data, *pVal);
        //if(m_adc_channel_get_data == 4) printf("\r\n");

/* This part is to support UART Debug Message
   The "uart" is initialized by uart_init(); in main()
*/
   //
        if(m_adc_channel_get_data == 0) sprintf(tx_message,"vBatt value: %d\n", *pVal);
        else //if(m_adc_channel_get_data == 1) 
        sprintf(tx_message,"ADC1 value: %d\n", *pVal);
        for (uint32_t i = 0; i < strlen(tx_message); i++)
        {
            do
            {
                err_code = app_uart_put(tx_message[i]);             
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
    //

        if(m_adc_channel_get_data == 1) {
            update_adv_cnt();
            f_uv_frame_rts = 1;
        }
    }
}

void saadc_init(void)
{
    static ret_code_t err_code;
	
	//set configuration for saadc channel 0
    // This is for check battery level
	channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_0_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_0_config.gain       = NRF_SAADC_GAIN1_6;
    channel_0_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
    channel_0_config.acq_time   = NRF_SAADC_ACQTIME_20US;
    channel_0_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_0_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_VDD);
    channel_0_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
	//set configuration for saadc channel 1
    // This is to read ADC1
	channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.gain       = NRF_SAADC_GAIN1_6;
    channel_1_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
    channel_1_config.acq_time   = NRF_SAADC_ACQTIME_20US;
    channel_1_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_1_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0);
    channel_1_config.pin_n      = NRF_SAADC_INPUT_DISABLED;

	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

}
/*
*/
uint16_t get_umile_uv_packet_str(uint8_t *p_str_frame)
{
    sprintf(p_str_frame,"%d %d %d", m_umile_uv_frame.vbatt,
       m_umile_uv_frame.adc1,
       m_umile_uv_frame.packet_counter);
    return(strlen(p_str_frame));
}
/*
*/
uint16_t get_umile_rts()
{
    return(f_uv_frame_rts);
}
/*
*/
void reset_umile_rts()
{
    f_uv_frame_rts = 0; 
}
/*
*/
void   umile_uv_start()
{
    ret_code_t err_code;
    
    nrf_gpio_pin_write(DEMUX_ENA, 1);       // power on sensor
    nrf_gpio_pin_write(LED_PIN, 1);
    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
	m_adc_channel_enabled = 0;
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_adc_timer_id, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);
}
/*
*/
void   umile_uv_stop()
{
    ret_code_t err_code;

    err_code = nrf_drv_saadc_channel_uninit(m_adc_channel_enabled);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(m_adc_timer_id);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_write(DEMUX_ENA, 0);  // Power off sensor
    nrf_gpio_pin_write(LED_PIN, 0);
}

/*
typedef PACKED_STRUCT
{
    uint16_t          vbatt;
    uint16_t          adc1;
    uint16_t          packet_counter;
} es_my_frame_t;

static es_my_frame_t  m_umile_uv_frame;
*/
void umile_uv_init(void)
{
    ret_code_t err_code;

    f_uv_frame_rts = 0;    // uvc frame not ready to send
    // Initialize packet
    memset(&m_umile_uv_frame, 0, sizeof(m_umile_uv_frame));
    m_umile_uv_frame.packet_counter = 0;
    nrf_gpio_cfg_output(DEMUX_ENA);  // sensor enable
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_write(LED_PIN, 1);
    // init. saadc channels and setting
    saadc_init();           
    // timer to read ADC values.
    err_code = app_timer_create(&m_adc_timer_id,APP_TIMER_MODE_REPEATED, adc_app_timer_handler);
}
