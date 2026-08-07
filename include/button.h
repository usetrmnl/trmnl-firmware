enum ButtonPressResult { LongPress, DoubleClick, ShortPress, SoftReset, ShipMode, NoAction };
extern const char *ButtonPressResultNames[5];

ButtonPressResult read_button_presses();

ButtonPressResult read_long_press();