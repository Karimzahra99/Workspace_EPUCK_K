#ifndef READ_IMAGE_H_
#define READ_IMAGE_H_

#include <stdint.h>
#include <main.h>

void read_image_start(void);
void tune_image_start(tuning_config_t arg_tune_settings);

uint8_t get_color(void);
int16_t get_middle_diff(void);
uint16_t get_middle_top(void);
uint16_t get_middle_bot(void);

#endif /* READ_IMAGE_H_ */
