#ifndef __STRING_PICO_H_
#define __STRING_PICO_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

size_t strlen(const char *s);
size_t strnlen(const char *s, size_t n);
char* strcpy(char* dest, const char *s);
char* strncpy(char* dest, const char* src, size_t n);
void* memcpy(void* dest, const void* s, size_t len);
void* memset(void* dest, int val, size_t len);
int memcmp(const void* a, const void* b, size_t len);
int strcmp (char const *a, char const *b) ;
int strncmp (char const *a, char const *b, size_t n);
char* strcpyXXXX(char* dest, const char* src);
extern char * strstr(const char *s, const char *find);
extern char * strchr ( const char * str, int character );

#ifdef __cplusplus
}
#endif // __cplusplus


// bcopy und bzero sind laut POSIX standard nicht mehr erwünscht, wir sollten sie besser ganz weglassen, das ist einfacher.
// Die Argumente source und destinationen sind bei bcopy vertauscht gegenüber
// memcpy, das führt nur zu Irrtümern. Also auch daher sollten wir es vermeiden
#define bcopy(src, dest, n) memcpy(dest, src, n)
#define bzero(dest, len) memset(dest, 0, len)

#endif // __STRING_PICO_H_
