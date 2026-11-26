/* Freestanding <string.h> for the STM32WB BLE smoke firmware (rsemu, MIT). */
#ifndef RSEMU_FREESTANDING_STRING_H
#define RSEMU_FREESTANDING_STRING_H
#include <stddef.h>
void *memset(void *dst, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);
size_t strlen(const char *s);
#endif
