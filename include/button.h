enum ButtonPressResult { LongPress, DoubleClick, ShortPress, SoftReset, NoAction };
extern const char *ButtonPressResultNames[4];

ButtonPressResult read_button_presses();

ButtonPressResult read_long_press();

// What a wake by the button does on boards with a physical button (the X uses its touch bar).
bool button_wake_act(ButtonPressResult button);
void button_wake_run_special_function(void);
