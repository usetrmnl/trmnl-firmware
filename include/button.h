#pragma once

enum ButtonPressResult { LongPress, DoubleClick, ShortPress, SoftReset, NoAction };
constexpr const char *ButtonPressResultNames[] = {"LongPress", "DoubleClick", "ShortPress", "SoftReset", "NoAction"};
static_assert(sizeof(ButtonPressResultNames) / sizeof(ButtonPressResultNames[0]) == NoAction + 1,
              "one name per ButtonPressResult, in enum order");

ButtonPressResult read_button_presses();

ButtonPressResult read_long_press();

// What a wake by the button does on boards with a physical button (the X uses its touch bar).
bool button_wake_act(ButtonPressResult button);
void button_wake_run_special_function(void);
