#ifndef BMI_SLOT_H
#define BMI_SLOT_H

//void bmi_slot_resrc_init(void);

void bmi_slot_power_on  (int num);
void bmi_slot_power_off (int num);

void bmi_slot_gpio_configure (int num, int gpio);		
int bmi_slot_gpio_get (int num);

void bmi_slot_gpio_set (int num, int data);
void bmi_slot_uart_enable  (int num);
void bmi_slot_uart_disable (int num);

void bmi_slot_spi_enable (int num);
void bmi_slot_spi_disable (int num);

void bmi_slot_audio_enable  (int num);
void bmi_slot_audio_disable (int num);

void bmi_slot_battery_enable (int num);
void bmi_slot_battery_disable (int num);

int bmi_slot_module_present (int num);
//int bmi_slot_status_irq_state (int num);


#endif
