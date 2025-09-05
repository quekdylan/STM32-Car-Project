#include "userButton.h"

uint8_t user_is_pressed() {
	return HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_SET;
}
