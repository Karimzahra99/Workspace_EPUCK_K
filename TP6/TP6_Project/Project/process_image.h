#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void process_image_start(void);

uint16_t get_middle_line(void);
uint8_t get_color(void);

//Enlever du .h ?
void find_color(void);
void set_threshold_color(int selector_pos);
void calc_max_mean(void);
void max_count(void);
void calc_line_middle(void);
void filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value);

#endif /* PROCESS_IMAGE_H */
