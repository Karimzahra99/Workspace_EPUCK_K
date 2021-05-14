#ifndef READ_IMAGE_H_
#define READ_IMAGE_H_

#include <stdint.h>
#include <main.h>

//start the camera threads
void read_image_start(config_t arg_config);
void tune_image_start(tuning_config_t arg_tune_settings);

//get functions
uint8_t get_color(void);
int16_t get_middle_diff(void);
int16_t get_middle_top(void);
int16_t get_middle_bot(void);

//reset both middle position of top and bottom line to 0


//remove if not needed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void reset_middle_positions(void);
void set_middle_top(uint16_t top_middle);
void set_middle_bot(uint16_t bot_middle);

#endif /* READ_IMAGE_H_ */
