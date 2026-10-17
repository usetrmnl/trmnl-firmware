#include <battery_curve.h>

// Share of a discharge run's time left at each voltage, from TRMNL X devices in the field.
// Keep in step with the server's copy of this curve.
static const struct {
  float voltage;
  float percent;
} kSocCurve[] = {
    {3.00f, 0},  {3.20f, 1},  {3.40f, 2},  {3.50f, 3},  {3.60f, 5},
    {3.65f, 7},  {3.70f, 14}, {3.75f, 23}, {3.80f, 38}, {3.85f, 52},
    {3.90f, 61}, {4.00f, 74}, {4.10f, 85}, {4.20f, 95}, {4.25f, 100},
};
static const int kSocCurvePoints = sizeof(kSocCurve) / sizeof(kSocCurve[0]);

int estimate_soc_from_voltage(float voltage) {
  if (voltage <= kSocCurve[0].voltage) return 0;
  for (int i = 1; i < kSocCurvePoints; i++) {
    if (voltage < kSocCurve[i].voltage) {
      float fraction = (voltage - kSocCurve[i - 1].voltage) / (kSocCurve[i].voltage - kSocCurve[i - 1].voltage);
      float percent = kSocCurve[i - 1].percent + fraction * (kSocCurve[i].percent - kSocCurve[i - 1].percent);
      return (int)(percent + 0.5f);
    }
  }
  return 100;
}
