#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include "comon.h"

esp_err_t power_contr_init(void);
esp_err_t power_contr_set(uint16_t value);

#endif