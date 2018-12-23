#ifndef _COMPILER_H_
#define _COMPILER_H_

/**
* \def RESTRICT Marks a restricted pointer 
*/
#ifdef __GNUC__
#define RESTRICT __restrict__
#else
#define RESTRICT
#endif

/**
* \def PURE Marks a function as pure, i.e. without global state
*/
#ifdef __GNUC__
#define PURE __attribute__ ((pure))
#else
#define PURE
#endif

/**
* \def HOT Marks a function as a hot spot
*/
#ifdef __GNUC__
#define HOT __attribute__ ((hot))
#else
#define HOT
#endif

/**
* \def COLD Marks a function as a cold spot
*/
#ifdef __GNUC__
#define COLD __attribute__ ((cold))
#else
#define COLD
#endif

/**
* \def INLINE Marks a function as to be inlined
*/
#ifdef _MSC_VER
#define INLINE __inline
#else
#define INLINE inline
#endif

/**
* \def EXTERN_INLINE Marks a function as to be inlined, but also externally defined
*/
#define EXTERN_INLINE extern INLINE

/**
* \def STATIC_INLINE Marks a function as to be inlined, but also statically defined
*/
#define STATIC_INLINE static INLINE

#endif