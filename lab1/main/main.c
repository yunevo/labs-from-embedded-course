#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//Cần include thư viện của RTOS, để sử dụng được vTaskdelay
#include "driver/gpio.h"
//Thao tác trên GPIO

#define LED_PIN 2	//Define chân điều khiển LED là chân 2

void app_main(void) {
	bool Status = 0;	//Biến lưu trạng thái bật tắt LED
	gpio_pad_select_gpio(LED_PIN);	//Cấu hình LED_PIN là 1 chân GPIO
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);		//Cấu hình là output

	while (1) {
		Status = !Status; //Đảo trạng thái
		gpio_set_level(LED_PIN, Status); 	//Bật hoặc tắt LED tùy theo giá trị biến Status
		if (Status) {
			printf("LED status: ON\n -------\n");
		} else {
			printf("LED status: OFF\n -------\n");
		} 	//in trạng thái của LED lên console
		vTaskDelay(1000 / portTICK_PERIOD_MS);		//delay 1s
	}
}
