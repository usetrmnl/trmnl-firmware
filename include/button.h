enum ButtonPressResult
{
  LongPress,
  DoubleClick,
  NoAction,
  SoftReset
};
extern const char *ButtonPressResultNames[4];

ButtonPressResult read_button_presses();