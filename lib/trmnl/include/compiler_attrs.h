#pragma once

#if defined(__GNUC__) || defined(__clang__)
#  define PRINTF_LIKE(fmt, args) __attribute__((format(printf, fmt, args)))
#else
#  define PRINTF_LIKE(fmt, args)
#endif
