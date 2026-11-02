// Update the EPD, then power down the PMIC(s)
void FrameUpdate(uint8_t *pFramebuffer);

// Initialize the Frame's RGB LED
void FrameLEDInit(void);

// Anode of the LEDs tied together
#define LED_PWM_PIN 23

// Each LED's cathode is tied to a GPIO on the I/O expander #2
#define LED_R_IO 11
#define LED_G_IO 7
#define LED_B_IO 2
 
// Frame LED colors (8 possible values)
enum {
   LED_BLACK = 0,
   LED_RED,
   LED_GREEN,
   LED_YELLOW,
   LED_BLUE,
   LED_MAGENTA,
   LED_CYAN,
   LED_WHITE
};

void FrameLEDSet(uint8_t color, uint8_t brightness);

