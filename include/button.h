enum ButtonPressResult
{
  LongPress,
  DoubleClick,
  ShortPress,
  SoftReset,
  NoAction
};
extern const char *ButtonPressResultNames[4];

ButtonPressResult read_button_presses();

ButtonPressResult read_long_press();