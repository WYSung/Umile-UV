

#define UV_ADC_PIN_1                   NRF_GPIO_PIN_MAP(0,2)
#define LED_PIN                        NRF_GPIO_PIN_MAP(0,12)
#define DEMUX_ENA                      NRF_GPIO_PIN_MAP(0,24)
#define BATTERY_LEVEL                  NRF_GPIO_PIN_MAP(0,31)
#define POWER_ON                       NRF_GPIO_PIN_MAP(0,20)

// Defined but not used
#define VCC_OPTION                     NRF_GPIO_PIN_MAP(0,18)

#define BTN_ID_SLEEP                0   /**< ID of the button used to put the application into sleep/system OFF mode. */
#define BTN_ID_WAKEUP               0   /**< ID of the button used to wake up the application. */

void   umile_uv_start();
void   umile_uv_stop();
uint16_t get_umile_uv_packet_str(uint8_t *p_str_frame);
uint16_t get_umile_rts(void);
void reset_umile_rts(void);
void umile_uv_init(void);
