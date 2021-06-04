#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5

// gets beginning of the line
float get_middle(void);

//starts the process thread to get the beginning of the line to follow
void process_image_start(void);

//void SendUint8ToComputer(uint8_t* data, uint16_t size);

#endif /* PROCESS_IMAGE_H */
