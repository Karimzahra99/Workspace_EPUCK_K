#ifndef READ_IMAGE_H_
#define READ_IMAGE_H_

#include <stdint.h>
#include <main.h>

//Number of pixels of one horizontal line of the camera
#define IMAGE_BUFFER_SIZE			640

void read_image_start(config_t arg_config);
void tune_image_start(tuning_config_t arg_tune_settings);

uint8_t get_color(void);
int16_t get_middle_diff(void);
int16_t get_middle_top(void);
int16_t get_middle_bot(void);

#endif /* READ_IMAGE_H_ */
