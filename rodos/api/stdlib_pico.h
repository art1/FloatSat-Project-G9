#ifndef __STDLIB_PICO_H_
#define __STDLIB_PICO_H_

//#include <stddef.h>


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

extern int isspace(int c);
extern int isdigit(int c);
extern int isalpha(int c);
extern int isupper(int c);

extern long int strtol (const char * nptr, char ** endptr, int base);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __STDLIB_PICO_H_
