/* Constants and templates used by multiple files in the 16-bit song player */


#define ABUF_SIZE 16000
#define ABUF_BYTE_SIZE (ABUF_SIZE * 2)
#define NUM_ABUF 2
#define DFLT_VOLUME 150			/* 0-255 */  /* This produces a 3.3V output - use a voltage divider to drive a speaker at 1V */
#define VOLUME_INCREMENT 20	// How much to change the volume on each adjustment
