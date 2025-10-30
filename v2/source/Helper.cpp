#include "Helper.h"
#include "MicroBit.h"

int ftoa(float f, char *s, short digits) {
    if (s == NULL)
        return DEVICE_INVALID_PARAMETER;

    int i = 0;
    char sign = ' ';
    if (f < 0) {
        f = -f; // Make f positive for processing
        sign = '-';
    }
    int part = (int)((f - (int) f) * pow(10, digits + 1));
    if (part == 0 && ((int) f) == 0) // Both integer and decimal parts are zero
        sign = ' ';
    while (digits--) {
        s[i++] = (part % 10) + '0'; // Get char of digit
        part /= 10; // pop last digit
    }
    s[i++] = '.';
    // TODO: There is a better way to do this. Hint: (int) f * pow(10, digits + 1) will give you the int and digits part of the decimal part
    part = (int) f; // Get the decimal part 
    do {
        s[i++] = (part % 10) + '0';
        part /= 10; // pop last digit
    } while (part);
    s[i++] = sign;
    s[i] = '\0'; // Null-terminate the string

    // Reverse the string to get the correct order
    string_reverse(s);
    return DEVICE_OK;
}

// Wrapper for ftoa to return a ManagedString
ManagedString ftos(float f) {
    /* Gauss values for the LSM303AGR: -+70k normal range and -+1.2M component max range, so:
     * > 1 byte for negative sign,
     * > 8 byte for integer part (up to 10M-1),
     * > 1 byte for dot,
     * > 4 byte for decimal part,
     * > 1 byte for null terminator and
     * > 3 byte for anything else I've missed.
     * For a total of 16 chars/bytes.
     */
    char buf[16];
    ftoa(f, buf);
    return buf;
}