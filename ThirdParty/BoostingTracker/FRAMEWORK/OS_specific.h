#ifndef OS_SPECIFIC_H
#define OS_SPECIFIC_H

//check the operating system types
#ifndef OS_type
#if defined(unix)        || defined(__unix)      || defined(__unix__) \
 || defined(linux)       || defined(__linux)     || defined(__linux__) \
 || defined(sun)         || defined(__sun) \
 || defined(BSD)         || defined(__OpenBSD__) || defined(__NetBSD__) \
 || defined(__FreeBSD__) || defined __DragonFly__ \
 || defined(sgi)         || defined(__sgi) \
 || defined(__MACOSX__)  || defined(__APPLE__) \
 || defined(__CYGWIN__)
#define OS_type 1
#elif defined(_MSC_VER) || defined(WIN32)  || defined(_WIN32) || defined(__WIN32__) \
   || defined(WIN64)    || defined(_WIN64) || defined(__WIN64__)
#define OS_type 2
#else
#define OS_type 0
#endif
#elif !(OS_type==0 || OS_type==1 || OS_type==2)
#error CImg Library : Configuration variable 'OS_type' is badly defined.
#error (valid values are '0=unknown OS', '1=Unix-like OS', '2=Microsoft Windows').
#endif


//include platform specific headers
#if OS_type==1
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <QtCore>

//typedef int32_t __int32;
//typedef int32_t __int64;
//typedef uint32_t  __uint32;
//typedef uint64_t  __uint64;

//MAX_PATH variable used by helmut
#ifdef MAX_PATH /* Work-around for Mingw */
#undef MAX_PATH
#endif /* MAX_PATH */
#define MAX_PATH 260

#elif OS_type==2
#include <windows.h>
#include <shlobj.h>
#include "stdint.h"

//MAX_PATH variable used by helmut
#ifdef MAX_PATH /* Work-around for Mingw */
#undef MAX_PATH
#endif /* MAX_PATH */
#define MAX_PATH        260
#ifndef _WIN32_IE
#define _WIN32_IE 0x0400
#endif
#endif

//general headers
#include <vector>
#include <vector>
#include <string>
#include <map>
#include <cstdio>

//typedef int32_t __int32;
//typedef int32_t __int64;
typedef uint32_t  __uint32;
typedef uint64_t  __uint64;


#endif // OS_SPECIFIC_H
