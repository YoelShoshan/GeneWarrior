
#define DllExport   __declspec( dllexport )

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SFMT_H
#define SFMT_H

#include <stdio.h>

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
  #include <inttypes.h>
#elif defined(_MSC_VER) || defined(__BORLANDC__)
  typedef unsigned int uint32_t;
  typedef unsigned __int64 uint64_t;
  #define inline __inline
#else
  #include <inttypes.h>
  #if defined(__GNUC__)
    #define inline __inline__
  #endif
#endif

#ifndef PRIu64
  #if defined(_MSC_VER) || defined(__BORLANDC__)
    #define PRIu64 "I64u"
    #define PRIx64 "I64x"
  #else
    #define PRIu64 "llu"
    #define PRIx64 "llx"
  #endif
#endif

#if defined(__GNUC__)
#define ALWAYSINLINE __attribute__((always_inline))
#else
#define ALWAYSINLINE
#endif

#if defined(_MSC_VER)
  #if _MSC_VER >= 1200
    #define PRE_ALWAYS __forceinline
  #else
    #define PRE_ALWAYS inline
  #endif
#else
  #define PRE_ALWAYS inline
#endif

DllExport uint32_t gen_rand32(void);
DllExport uint64_t gen_rand64(void);
DllExport int fill_array32(uint32_t *array, int size);
DllExport int fill_array64(uint64_t *array, int size);
DllExport void init_gen_rand(uint32_t seed);

void init_by_array(uint32_t *init_key, int key_length);
const char *get_idstring(void);
int get_min_array_size32(void);
int get_min_array_size64(void);
int get_array32_extended_size(int size);
int get_array64_extended_size(int size);

#endif

#ifdef __cplusplus
}
#endif