#ifndef BUZZER_H
#define BUZZER_H

#include <config.h>

#ifdef PIN_BUZZER

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(unsigned long duration_ms);
void buzzer_beep_pattern(int count, unsigned long on_ms, unsigned long off_ms);

#else

static inline void buzzer_init(void) {}
static inline void buzzer_on(void) {}
static inline void buzzer_off(void) {}
static inline void buzzer_beep(unsigned long duration_ms) { (void)duration_ms; }
static inline void buzzer_beep_pattern(int count, unsigned long on_ms, unsigned long off_ms) { (void)count; (void)on_ms; (void)off_ms; }

#endif // PIN_BUZZER

#endif // BUZZER_H
