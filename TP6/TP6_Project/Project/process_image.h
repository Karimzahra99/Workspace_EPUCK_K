#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void process_image_start(void);
uint16_t get_line_position(void);

void calc_max(void);
void check_threshold(void);
uint8_t get_color(void);

//Vertical index of line (0 to 480) 0 : highest, 479 :lowest (due to camera library we take two lines)
#define LINE_INDEX					25

//Le nombre minimum de pixel pour valider une detection de ligne pour une certaine couleur
#define MIN_COUNT					5

//Shift pour remettre les bits des couleurs dans l'ordre lors de l'extraction du format RGB565
#define SHIFT_1						3
#define SHIFT_2						5

#endif /* PROCESS_IMAGE_H */
