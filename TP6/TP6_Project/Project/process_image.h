#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void process_image_start(void);
uint16_t get_line_position(void);
void set_threshold_color(int selector_pos);
void calc_max_mean(void);
void max_count(void);
uint8_t get_color(void);
uint16_t get_lineWidth(uint8_t color_index);

//Vertical index of line (0 to 480) 0 : highest, 479 :lowest (due to camera library we take two lines)
#define LINE_INDEX					25

//Le nombre minimum de pixel pour valider une detection de ligne pour une certaine couleur
#define MIN_COUNT					5

//Valeur maximal d'intensite d'un pixel (vert ramener sur [0,31])
#define MAX_VALUE					31

//Shift pour remettre les bits des couleurs dans l'ordre lors de l'extraction du format RGB565
#define SHIFT_3						3
#define SHIFT_5						5

//Index des couleurs / utiliser enum ?
#define RED_IDX						1
#define GREEN_IDX					2
#define BLUE_IDX					3
#endif /* PROCESS_IMAGE_H */
