#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void process_image_start(void);

uint16_t get_middle_line(void);
uint8_t get_color(void);

//Enlever du .h ?
void set_threshold_color(int selector_pos);
void calc_max_mean(void);
void max_count(void);
void calc_line_middle(uint8_t color_index);

#endif /* PROCESS_IMAGE_H */
