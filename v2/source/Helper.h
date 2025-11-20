#ifndef HELPER_CPP
#define HELPER_CPP
#include "MicroBit.h"

/* Converts a given float into a string representation, with 4 decimal points.
 * 
 * Parameters:
 * @param f The number to convert.
 * @param s A pointer to the buffer where the resulting string will be stored.
 * 
 * Note: Make sure you allocate enough space for s to hold the sign, integer part, dot, decimal part and null terminator.
 * 
 * Returns:
 * DEVICE_OK, or DEVICE_INVALID_PARAMETER.
 */
int ftoa(float f, char *s, short digits = 3);
ManagedString ftos(float f);
#endif
