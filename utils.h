#ifndef utils_h
#define utils_h

#define RED_LED 8
#define GREEN_LED 9

#define BLINK_RED 1
#define BLINK_GREEN 2
#define BLINK_ORANGE 3

#define ON 1
#define OFF 0

void dbg_print(const char *string);
void dbg_print_P(const char *string);
void blinkLed(uint8_t color, uint8_t repetition, uint16_t delayMs);
int8_t sim908_read_line(char *line_buffer, uint8_t buff_size, uint16_t tout_ms);
int32_t parsedecimal(char *str);


#endif
