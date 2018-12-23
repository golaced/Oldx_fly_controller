/*
 * math.h: ANSI 'C' (X3J11 Oct 88) library header, section 4.5
 * Copyright (C) Codemist Ltd., 1988
 * Copyright 1991-1998,2004-2006 ARM Limited. All rights reserved
 */

/*
 * RCS $Revision: 179217 $ Codemist 0.03
 * Checkin $Date: 2013-03-12 14:03:19 +0000 (Tue, 12 Mar 2013) $
 * Revising $Author: statham $
 */

/*
 * Parts of this file are based upon fdlibm:
 *
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#ifndef __math_h
#define __math_h
#define __ARMCLIB_VERSION 5040049

/*
 * Depending on compiler version __int64 or __INT64_TYPE__ should be defined.
 */
#ifndef __int64
  #ifdef __INT64_TYPE__
    #define __int64 __INT64_TYPE__
  #endif
  /* On some architectures neither of these may be defined - if so, fall
     through and error out if used. */
#endif

/*
 * Some of these declarations are new in C99.  To access them in C++
 * you can use -D__USE_C99_MATH (or -D__USE_C99_ALL).
 */
#ifndef __USE_C99_MATH
  #if defined(__USE_C99_ALL) || (defined(__STDC_VERSION__) && 199901L <= __STDC_VERSION__)
    #define __USE_C99_MATH 1
  #endif
#endif

#define _ARMABI __declspec(__nothrow)
#define _ARMABI_SOFTFP __declspec(__nothrow) __attribute__((__pcs__("aapcs")))
#define _ARMABI_PURE __declspec(__nothrow) __attribute__((const))
#ifdef __FP_FENV_EXCEPTIONS
# define _ARMABI_FPEXCEPT _ARMABI
#else
# define _ARMABI_FPEXCEPT _ARMABI __attribute__((const))
#endif

#ifdef __cplusplus
#define _ARMABI_INLINE inline
#define _ARMABI_INLINE_DEF inline
#elif defined __GNUC__ || defined _USE_STATIC_INLINE
#define _ARMABI_INLINE static __inline
#define _ARMABI_INLINE_DEF static __inline
#elif (defined(__STDC_VERSION__) && 199901L <= __STDC_VERSION__)
#define _ARMABI_INLINE inline
#define _ARMABI_INLINE_DEF static inline
#else
#define _ARMABI_INLINE __inline
#define _ARMABI_INLINE_DEF __inline
#endif

   /*
    * If the compiler supports signalling nans as per N965 then it
    * will define __SUPPORT_SNAN__, in which case a user may define
    * _WANT_SNAN in order to obtain the nans function, as well as the
    * FP_NANS and FP_NANQ classification macros.
    */
#if defined(__SUPPORT_SNAN__) && defined(_WANT_SNAN)
#pragma import(__use_snan)
#endif

/*
 * Macros for our inline functions down below.
 * unsigned& __FLT(float x) - returns the bit pattern of x
 * unsigned& __HI(double x) - returns the bit pattern of the high part of x
 *                            (high part has exponent & sign bit in it)
 * unsigned& __LO(double x) - returns the bit pattern of the low part of x
 *
 * We can assign to __FLT, __HI, and __LO and the appropriate bits get set in
 * the floating point variable used.
 *
 * __HI & __LO are affected by the endianness and the target FPU.
 */
#define __FLT(x) (*(unsigned *)&(x))
#ifdef __BIG_ENDIAN
#  define __LO(x) (*(1 + (unsigned *)&(x)))
#  define __HI(x) (*(unsigned *)&(x))
#else /* ndef __BIG_ENDIAN */
#  define __HI(x) (*(1 + (unsigned *)&(x)))
#  define __LO(x) (*(unsigned *)&(x))
#endif /* ndef __BIG_ENDIAN */

#   ifndef __MATH_DECLS
#   define __MATH_DECLS


/*
 * A set of functions that we don't actually want to put in the standard
 * namespace ever.  These are all called by the C99 macros.  As they're
 * not specified by any standard they can't belong in ::std::.  The
 * macro #defines are below amongst the standard function declarations.
 * We only include these if we actually need them later on
 */
#if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)
#   ifdef __cplusplus
      extern "C" {
#   endif /* __cplusplus */

extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double /*x*/, double /*y*/);
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float /*x*/, float /*y*/);
    /*
     * Compare x and y and return the CPSR in r0.  These means we can test for
     * result types with bit pattern matching.
     *
     * These are a copy of the declarations in rt_fp.h keep in sync.
     */

extern _ARMABI_SOFTFP int __ARM_fpclassifyf(float /*x*/);
extern _ARMABI_SOFTFP int __ARM_fpclassify(double /*x*/);
    /* Classify x into NaN, infinite, normal, subnormal, zero */
    /* Used by fpclassify macro */

_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isfinitef(float __x)
{
    return ((__FLT(__x) >> 23) & 0xff) != 0xff;
}
_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isfinite(double __x)
{
    return ((__HI(__x) >> 20) & 0x7ff) != 0x7ff;
}
    /* Return 1 if __x is finite, 0 otherwise */
    /* Used by isfinite macro */

_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isinff(float __x)
{
    return (__FLT(__x) << 1) == 0xff000000;
}
_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isinf(double __x)
{
    return ((__HI(__x) << 1) == 0xffe00000) && (__LO(__x) == 0);
}
    /* Return 1 if __x is infinite, 0 otherwise */
    /* Used by isinf macro */

_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2); /* Just N set or Just Z set */
}
_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2); /* Just N set or Just Z set */
}
    /*
     * Compare __x and __y and return 1 if __x < __y or __x > __y, 0 otherwise
     * Used by islessgreater macro
     */

_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isnanf(float __x)
{
    return (0x7f800000 - (__FLT(__x) & 0x7fffffff)) >> 31;
}
_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isnan(double __x)
{
    unsigned __xf = __HI(__x) | ((__LO(__x) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
    /* Return 1 if __x is a NaN, 0 otherwise */
    /* Used by isnan macro */

_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isnormalf(float __x)
{
    unsigned __xe = (__FLT(__x) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_isnormal(double __x)
{
    unsigned __xe = (__HI(__x) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
    /* Return 1 if __x is a normalised number, 0 otherwise */
    /* used by isnormal macro */

_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_signbitf(float __x)
{
    return __FLT(__x) >> 31;
}
_ARMABI_INLINE_DEF _ARMABI_SOFTFP int __ARM_signbit(double __x)
{
    return __HI(__x) >> 31;
}
    /* Return signbit of __x */
    /* Used by signbit macro */

#   ifdef __cplusplus
      } /* extern "C" */
#   endif /* __cplusplus */
#endif /* Strict ANSI */

#   undef __CLIBNS

#   ifdef __cplusplus
      namespace std {
#       define __CLIBNS ::std::
        extern "C" {
#   else
#       define __CLIBNS
#   endif  /* __cplusplus */


#ifndef __has_builtin
  #define __has_builtin(x) 0
#endif

#if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)
  /* C99 additions */
  typedef float float_t;
  typedef double double_t;
#if __has_builtin(__builtin_inf)
#   define HUGE_VALF __builtin_inff()
#   define HUGE_VALL __builtin_infl()
#   define INFINITY __builtin_inff()
#   define NAN __builtin_nanf("")
# else
#   define HUGE_VALF ((float)__INFINITY__)
#   define HUGE_VALL ((long double)__INFINITY__)
#   define INFINITY ((float)__INFINITY__)
#   define NAN (__ESCAPE__(0f_7FC00000))
#endif

#   define MATH_ERRNO 1
#   define MATH_ERREXCEPT 2
extern const int math_errhandling;
#endif
#if __has_builtin(__builtin_inf)
# define HUGE_VAL __builtin_inf()
#else
# define HUGE_VAL ((double)__INFINITY__)
#endif

extern _ARMABI double acos(double /*x*/);
   /* computes the principal value of the arc cosine of x */
   /* a domain error occurs for arguments not in the range -1 to 1 */
   /* Returns: the arc cosine in the range 0 to Pi. */
extern _ARMABI double asin(double /*x*/);
   /* computes the principal value of the arc sine of x */
   /* a domain error occurs for arguments not in the range -1 to 1 */
   /* and -HUGE_VAL is returned. */
   /* Returns: the arc sine in the range -Pi/2 to Pi/2. */

extern _ARMABI_PURE double atan(double /*x*/);
   /* computes the principal value of the arc tangent of x */
   /* Returns: the arc tangent in the range -Pi/2 to Pi/2. */

extern _ARMABI double atan2(double /*y*/, double /*x*/);
   /* computes the principal value of the arc tangent of y/x, using the */
   /* signs of both arguments to determine the quadrant of the return value */
   /* a domain error occurs if both args are zero, and -HUGE_VAL returned. */
   /* Returns: the arc tangent of y/x, in the range -Pi to Pi. */

extern _ARMABI double cos(double /*x*/);
   /* computes the cosine of x (measured in radians). A large magnitude */
   /* argument may yield a result with little or no significance. */
   /* a domain error occurs for infinite input (C 7.12.1 footnote 196). */
   /* Returns: the cosine value. */
extern _ARMABI double sin(double /*x*/);
   /* computes the sine of x (measured in radians). A large magnitude */
   /* argument may yield a result with little or no significance. */
   /* a domain error occurs for infinite input (C 7.12.1 footnote 196). */
   /* Returns: the sine value. */

extern void __use_accurate_range_reduction(void);
   /* reference this to select the larger, slower, but more accurate */
   /* range reduction in sin, cos and tan */

extern _ARMABI double tan(double /*x*/);
   /* computes the tangent of x (measured in radians). A large magnitude */
   /* argument may yield a result with little or no significance */
   /* Returns: the tangent value. */
   /*          if range error; returns HUGE_VAL. */

extern _ARMABI double cosh(double /*x*/);
   /* computes the hyperbolic cosine of x. A range error occurs if the */
   /* magnitude of x is too large. */
   /* Returns: the hyperbolic cosine value. */
   /*          if range error; returns HUGE_VAL. */
extern _ARMABI double sinh(double /*x*/);
   /* computes the hyperbolic sine of x. A range error occurs if the */
   /* magnitude of x is too large. */
   /* Returns: the hyperbolic sine value. */
   /*          if range error; returns -HUGE_VAL or HUGE_VAL depending */
   /*          on the sign of the argument */

extern _ARMABI_PURE double tanh(double /*x*/);
   /* computes the hyperbolic tangent of x. */
   /* Returns: the hyperbolic tangent value. */

extern _ARMABI double exp(double /*x*/);
   /* computes the exponential function of x. A range error occurs if the */
   /* magnitude of x is too large. */
   /* Returns: the exponential value. */
   /*          if underflow range error; 0 is returned. */
   /*          if overflow range error; HUGE_VAL is returned. */

extern _ARMABI double frexp(double /*value*/, int * /*exp*/) __attribute__((__nonnull__(2)));
   /* breaks a floating-point number into a normalised fraction and an */
   /* integral power of 2. It stores the integer in the int object pointed */
   /* to by exp. */
   /* Returns: the value x, such that x is a double with magnitude in the */
   /* interval 0.5 to 1.0 or zero, and value equals x times 2 raised to the */
   /* power *exp. If value is zero, both parts of the result are zero. */

extern _ARMABI double ldexp(double /*x*/, int /*exp*/);
   /* multiplies a floating-point number by an integral power of 2. */
   /* A range error may occur. */
   /* Returns: the value of x times 2 raised to the power of exp. */
   /*          if range error; HUGE_VAL is returned. */
extern _ARMABI double log(double /*x*/);
   /* computes the natural logarithm of x. A domain error occurs if the */
   /* argument is negative, and -HUGE_VAL is returned. A range error occurs */
   /* if the argument is zero. */
   /* Returns: the natural logarithm. */
   /*          if range error; -HUGE_VAL is returned. */
extern _ARMABI double log10(double /*x*/);
   /* computes the base-ten logarithm of x. A domain error occurs if the */
   /* argument is negative. A range error occurs if the argument is zero. */
   /* Returns: the base-ten logarithm. */
extern _ARMABI double modf(double /*value*/, double * /*iptr*/) __attribute__((__nonnull__(2)));
   /* breaks the argument value into integral and fraction parts, each of */
   /* which has the same sign as the argument. It stores the integral part */
   /* as a double in the object pointed to by iptr. */
   /* Returns: the signed fractional part of value. */

extern _ARMABI double pow(double /*x*/, double /*y*/);
   /* computes x raised to the power of y. A domain error occurs if x is */
   /* zero and y is less than or equal to zero, or if x is negative and y */
   /* is not an integer, and -HUGE_VAL returned. A range error may occur. */
   /* Returns: the value of x raised to the power of y. */
   /*          if underflow range error; 0 is returned. */
   /*          if overflow range error; HUGE_VAL is returned. */
extern _ARMABI double sqrt(double /*x*/);
   /* computes the non-negative square root of x. A domain error occurs */
   /* if the argument is negative, and -HUGE_VAL returned. */
   /* Returns: the value of the square root. */

#if defined(__TARGET_FPU_VFP_DOUBLE) && !defined(__TARGET_FPU_SOFTVFP)
    _ARMABI_INLINE double _sqrt(double __x) { return __sqrt(__x); }
#else
    _ARMABI_INLINE double _sqrt(double __x) { return sqrt(__x); }
#endif
#if defined(__TARGET_FPU_VFP_SINGLE) && !defined(__TARGET_FPU_SOFTVFP)
    _ARMABI_INLINE float _sqrtf(float __x) { return __sqrtf(__x); }
#else
    _ARMABI_INLINE float _sqrtf(float __x) { return (float)sqrt(__x); }
#endif
    /* With VFP, _sqrt and _sqrtf should expand inline as the native VFP square root
     * instructions. They will not behave like the C sqrt() function, because
     * they will report unusual values as IEEE exceptions (in fpmodes which
     * support IEEE exceptions) rather than in errno. These function names
     * are not specified in any standard. */

extern _ARMABI_PURE double ceil(double /*x*/);
   /* computes the smallest integer not less than x. */
   /* Returns: the smallest integer not less than x, expressed as a double. */
extern _ARMABI_PURE double fabs(double /*x*/);
   /* computes the absolute value of the floating-point number x. */
   /* Returns: the absolute value of x. */

extern _ARMABI_PURE double floor(double /*d*/);
   /* computes the largest integer not greater than x. */
   /* Returns: the largest integer not greater than x, expressed as a double */

extern _ARMABI double fmod(double /*x*/, double /*y*/);
   /* computes the floating-point remainder of x/y. */
   /* Returns: the value x - i * y, for some integer i such that, if y is */
   /*          nonzero, the result has the same sign as x and magnitude */
   /*          less than the magnitude of y. If y is zero, a domain error */
   /*          occurs and -HUGE_VAL is returned. */

    /* Additional Mathlib functions not defined by the ANSI standard.
     * Not guaranteed, and not necessarily very well tested.
     * C99 requires the user to include <math.h> to use these functions
     * declaring them "by hand" is not sufficient
     *
     * The above statement is not completely true now.  Some of the above
     * C99 functionality has been added as per the Standard, and (where
     * necessary) old Mathlib functionality withdrawn/changed.  Before
     * including this header #define __ENABLE_MATHLIB_LEGACY if you want to
     * re-enable the legacy functionality.
     */

#if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)

extern _ARMABI double acosh(double /*x*/);
    /*
     * Inverse cosh. EDOM if argument < 1.0
     */
extern _ARMABI double asinh(double /*x*/);
    /*
     * Inverse sinh.
     */
extern _ARMABI double atanh(double /*x*/);
    /*
     * Inverse tanh. EDOM if |argument| > 1.0
     */
extern _ARMABI double cbrt(double /*x*/);
    /*
     * Cube root.
     */
_ARMABI_INLINE _ARMABI_PURE double copysign(double __x, double __y)
    /*
     * Returns x with sign bit replaced by sign of y.
     */
{
    __HI(__x) = (__HI(__x) & 0x7fffffff) | (__HI(__y) & 0x80000000);
    return __x;
}
_ARMABI_INLINE _ARMABI_PURE float copysignf(float __x, float __y)
    /*
     * Returns x with sign bit replaced by sign of y.
     */
{
    __FLT(__x) = (__FLT(__x) & 0x7fffffff) | (__FLT(__y) & 0x80000000);
    return __x;
}
extern _ARMABI double erf(double /*x*/);
    /*
     * Error function. (2/sqrt(pi)) * integral from 0 to x of exp(-t*t) dt.
     */
extern _ARMABI double erfc(double /*x*/);
    /*
     * 1-erf(x). (More accurate than just coding 1-erf(x), for large x.)
     */
extern _ARMABI double expm1(double /*x*/);
    /*
     * exp(x)-1. (More accurate than just coding exp(x)-1, for small x.)
     */
#define fpclassify(x) \
    ((sizeof(x) == sizeof(float)) ? \
        __ARM_fpclassifyf(x) : __ARM_fpclassify(x))
    /*
     * Classify a floating point number into one of the following values:
     */
#define FP_ZERO         (0)
#define FP_SUBNORMAL    (4)
#define FP_NORMAL       (5)
#define FP_INFINITE     (3)
#define FP_NAN          (7)

#if defined(_WANT_SNAN) && defined(__SUPPORT_SNAN__)
/* 
 * Note that we'll never classify a number as FP_NAN, as all NaNs will 
 * be either FP_NANQ or FP_NANS
 */
#  define FP_NANQ       (8)
#  define FP_NANS       (9)
#endif


extern _ARMABI double hypot(double /*x*/, double /*y*/);
    /*
     * sqrt(x*x+y*y), ie the length of the vector (x,y) or the
     * hypotenuse of a right triangle whose other two sides are x
     * and y. Won't overflow unless the _answer_ is too big, even
     * if the intermediate x*x+y*y is too big.
     */
extern _ARMABI int ilogb(double /*x*/);
    /*
     * Exponent of x (returns 0 for 1.0, 1 for 2.0, -1 for 0.5, etc.)
     */
extern _ARMABI int ilogbf(float /*x*/);
    /*
     * Like ilogb but takes a float
     */
extern _ARMABI int ilogbl(long double /*x*/);
    /*
     * Exponent of x (returns 0 for 1.0, 1 for 2.0, -1 for 0.5, etc.)
     */
#define FP_ILOGB0   (-0x7fffffff) /* ilogb(0) == -INT_MAX */
#define FP_ILOGBNAN ( 0x80000000) /* ilogb(NAN) == INT_MIN */

#define isfinite(x) \
    ((sizeof(x) == sizeof(float)) \
        ? __ARM_isfinitef(x) \
        : __ARM_isfinite(x))
    /*
     * Returns true if x is a finite number, size independent.
     */

#define isgreater(x, y) \
    (((sizeof(x) == sizeof(float)) && (sizeof(y) == sizeof(float))) \
        ? ((__ARM_fcmp4((x), (y)) & 0xf0000000) == 0x20000000) \
        : ((__ARM_dcmp4((x), (y)) & 0xf0000000) == 0x20000000))
    /*
     * Returns true if x > y, throws no exceptions except on Signaling NaNs
     *
     * We want the C not set but the Z bit clear, V must be clear
     */

#define isgreaterequal(x, y) \
    (((sizeof(x) == sizeof(float)) && (sizeof(y) == sizeof(float))) \
        ? ((__ARM_fcmp4((x), (y)) & 0x30000000) == 0x20000000) \
        : ((__ARM_dcmp4((x), (y)) & 0x30000000) == 0x20000000))
    /*
     * Returns true if x >= y, throws no exceptions except on Signaling NaNs
     *
     * We just need to see if the C bit is set or not and ensure V clear
     */

#define isinf(x) \
    ((sizeof(x) == sizeof(float)) \
        ? __ARM_isinff(x) \
        : __ARM_isinf(x))
    /*
     * Returns true if x is an infinity, size independent.
     */

#define isless(x, y)  \
    (((sizeof(x) == sizeof(float)) && (sizeof(y) == sizeof(float))) \
        ? ((__ARM_fcmp4((x), (y)) & 0xf0000000) == 0x80000000) \
        : ((__ARM_dcmp4((x), (y)) & 0xf0000000) == 0x80000000))
    /*
     * Returns true if x < y, throws no exceptions except on Signaling NaNs
     *
     * We're less than if N is set, V clear
     */

#define islessequal(x, y) \
    (((sizeof(x) == sizeof(float)) && (sizeof(y) == sizeof(float))) \
        ? ((__ARM_fcmp4((x), (y)) & 0xc0000000) != 0) \
        : ((__ARM_dcmp4((x), (y)) & 0xc0000000) != 0))
    /*
     * Returns true if x <= y, throws no exceptions except on Signaling NaNs
     *
     * We're less than or equal if one of N or Z is set, V clear
     */

#define islessgreater(x, y) \
    (((sizeof(x) == sizeof(float)) && (sizeof(y) == sizeof(float))) \
        ? __ARM_islessgreaterf((x), (y)) \
        : __ARM_islessgreater((x), (y)))
    /*
     * Returns true if x <> y, throws no exceptions except on Signaling NaNs
     * Unfortunately this test is too complicated to do in a macro without
     * evaluating x & y twice.  Shame really...
     */

#define isnan(x) \
    ((sizeof(x) == sizeof(float)) \
        ? __ARM_isnanf(x) \
        : __ARM_isnan(x))
    /*
     * Returns TRUE if x is a NaN.
     */

#define isnormal(x) \
    ((sizeof(x) == sizeof(float)) \
        ? __ARM_isnormalf(x) \
        : __ARM_isnormal(x))
    /*
     * Returns TRUE if x is a NaN.
     */

#define isunordered(x, y) \
    (((sizeof(x) == sizeof(float)) && (sizeof(y) == sizeof(float))) \
        ? ((__ARM_fcmp4((x), (y)) & 0x10000000) == 0x10000000) \
        : ((__ARM_dcmp4((x), (y)) & 0x10000000) == 0x10000000))
    /*
     * Returns true if x ? y, throws no exceptions except on Signaling NaNs
     * Unordered occurs if and only if the V bit is set
     */

extern _ARMABI double lgamma (double /*x*/);
    /*
     * The log of the absolute value of the gamma function of x. The sign
     * of the gamma function of x is returned in the global `signgam'.
     */
extern _ARMABI double log1p(double /*x*/);
    /*
     * log(1+x). (More accurate than just coding log(1+x), for small x.)
     */
extern _ARMABI double logb(double /*x*/);
    /*
     * Like ilogb but returns a double.
     */
extern _ARMABI float logbf(float /*x*/);
    /*
     * Like logb but takes and returns float
     */
extern _ARMABI long double logbl(long double /*x*/);
    /*
     * Like logb but takes and returns long double
     */
extern _ARMABI double nextafter(double /*x*/, double /*y*/);
    /*
     * Returns the next representable number after x, in the
     * direction toward y.
     */
extern _ARMABI float nextafterf(float /*x*/, float /*y*/);
    /*
     * Returns the next representable number after x, in the
     * direction toward y.
     */
extern _ARMABI long double nextafterl(long double /*x*/, long double /*y*/);
    /*
     * Returns the next representable number after x, in the
     * direction toward y.
     */
extern _ARMABI double nexttoward(double /*x*/, long double /*y*/);
    /*
     * Returns the next representable number after x, in the
     * direction toward y.
     */
extern _ARMABI float nexttowardf(float /*x*/, long double /*y*/);
    /*
     * Returns the next representable number after x, in the
     * direction toward y.
     */
extern _ARMABI long double nexttowardl(long double /*x*/, long double /*y*/);
    /*
     * Returns the next representable number after x, in the
     * direction toward y.
     */
extern _ARMABI double remainder(double /*x*/, double /*y*/);
    /*
     * Returns the remainder of x by y, in the IEEE 754 sense.
     */
extern _ARMABI_FPEXCEPT double rint(double /*x*/);
    /*
     * Rounds x to an integer, in the IEEE 754 sense.
     */
extern _ARMABI double scalbln(double /*x*/, long int /*n*/);
    /*
     * Compute x times 2^n quickly.
     */
extern _ARMABI float scalblnf(float /*x*/, long int /*n*/);
    /*
     * Compute x times 2^n quickly.
     */
extern _ARMABI long double scalblnl(long double /*x*/, long int /*n*/);
    /*
     * Compute x times 2^n quickly.
     */
extern _ARMABI double scalbn(double /*x*/, int /*n*/);
    /*
     * Compute x times 2^n quickly.
     */
extern _ARMABI float scalbnf(float /*x*/, int /*n*/);
    /*
     * Compute x times 2^n quickly.
     */
extern _ARMABI long double scalbnl(long double /*x*/, int /*n*/);
    /*
     * Compute x times 2^n quickly.
     */
#define signbit(x) \
    ((sizeof(x) == sizeof(float)) \
        ? __ARM_signbitf(x) \
        : __ARM_signbit(x))
    /*
     * Returns the signbit of x, size independent macro
     */
#endif

/* C99 float versions of functions.  math.h has always reserved these
   identifiers for this purpose (7.13.4). */
extern _ARMABI_PURE float _fabsf(float); /* old ARM name */
_ARMABI_INLINE _ARMABI_PURE float fabsf(float __f) { return _fabsf(__f); }
extern _ARMABI float sinf(float /*x*/);
extern _ARMABI float cosf(float /*x*/);
extern _ARMABI float tanf(float /*x*/);
extern _ARMABI float acosf(float /*x*/);
extern _ARMABI float asinf(float /*x*/);
extern _ARMABI float atanf(float /*x*/);
extern _ARMABI float atan2f(float /*y*/, float /*x*/);
extern _ARMABI float sinhf(float /*x*/);
extern _ARMABI float coshf(float /*x*/);
extern _ARMABI float tanhf(float /*x*/);
extern _ARMABI float expf(float /*x*/);
extern _ARMABI float logf(float /*x*/);
extern _ARMABI float log10f(float /*x*/);
extern _ARMABI float powf(float /*x*/, float /*y*/);
extern _ARMABI float sqrtf(float /*x*/);
extern _ARMABI float ldexpf(float /*x*/, int /*exp*/);
extern _ARMABI float frexpf(float /*value*/, int * /*exp*/) __attribute__((__nonnull__(2)));
extern _ARMABI_PURE float ceilf(float /*x*/);
extern _ARMABI_PURE float floorf(float /*x*/);
extern _ARMABI float fmodf(float /*x*/, float /*y*/);
extern _ARMABI float modff(float /*value*/, float * /*iptr*/) __attribute__((__nonnull__(2)));

/* C99 long double versions of functions. */
/* (also need to have 'using' declarations below) */
#define _ARMDEFLD1(f) \
    _ARMABI long double f##l(long double /*x*/)

#define _ARMDEFLD1P(f, T) \
    _ARMABI long double f##l(long double /*x*/, T /*p*/)

#define _ARMDEFLD2(f) \
    _ARMABI long double f##l(long double /*x*/, long double /*y*/)

/*
 * Long double versions of C89 functions can be defined
 * unconditionally, because C89 reserved these names in "future
 * library directions".
 */
_ARMDEFLD1(acos);
_ARMDEFLD1(asin);
_ARMDEFLD1(atan);
_ARMDEFLD2(atan2);
_ARMDEFLD1(ceil);
_ARMDEFLD1(cos);
_ARMDEFLD1(cosh);
_ARMDEFLD1(exp);
_ARMDEFLD1(fabs);
_ARMDEFLD1(floor);
_ARMDEFLD2(fmod);
_ARMDEFLD1P(frexp, int*) __attribute__((__nonnull__(2)));
_ARMDEFLD1P(ldexp, int);
_ARMDEFLD1(log);
_ARMDEFLD1(log10);
_ARMABI long double modfl(long double /*x*/, long double * /*p*/) __attribute__((__nonnull__(2)));
_ARMDEFLD2(pow);
_ARMDEFLD1(sin);
_ARMDEFLD1(sinh);
_ARMDEFLD1(sqrt);
_ARMDEFLD1(tan);
_ARMDEFLD1(tanh);

#if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)

/*
 * C99 float and long double versions of extra-C89 functions.
 */
extern _ARMABI float acoshf(float /*x*/);
_ARMDEFLD1(acosh);
extern _ARMABI float asinhf(float /*x*/);
_ARMDEFLD1(asinh);
extern _ARMABI float atanhf(float /*x*/);
_ARMDEFLD1(atanh);
_ARMDEFLD2(copysign);
extern _ARMABI float cbrtf(float /*x*/);
_ARMDEFLD1(cbrt);
extern _ARMABI float erff(float /*x*/);
_ARMDEFLD1(erf);
extern _ARMABI float erfcf(float /*x*/);
_ARMDEFLD1(erfc);
extern _ARMABI float expm1f(float /*x*/);
_ARMDEFLD1(expm1);
extern _ARMABI float log1pf(float /*x*/);
_ARMDEFLD1(log1p);
extern _ARMABI float hypotf(float /*x*/, float /*y*/);
_ARMDEFLD2(hypot);
extern _ARMABI float lgammaf(float /*x*/);
_ARMDEFLD1(lgamma);
extern _ARMABI float remainderf(float /*x*/, float /*y*/);
_ARMDEFLD2(remainder);
extern _ARMABI float rintf(float /*x*/);
_ARMDEFLD1(rint);

#endif

#ifdef __USE_C99_MATH
/*
 * Functions new in C99.
 */
extern _ARMABI double exp2(double /*x*/); /* * 2.^x. */
extern _ARMABI float exp2f(float /*x*/);
_ARMDEFLD1(exp2);
extern _ARMABI double fdim(double /*x*/, double /*y*/);
extern _ARMABI float fdimf(float /*x*/, float /*y*/);
_ARMDEFLD2(fdim);
#ifdef __FP_FAST_FMA
#define FP_FAST_FMA
#endif
#ifdef __FP_FAST_FMAF
#define FP_FAST_FMAF
#endif
#ifdef __FP_FAST_FMAL
#define FP_FAST_FMAL
#endif
extern _ARMABI double fma(double /*x*/, double /*y*/, double /*z*/);
extern _ARMABI float fmaf(float /*x*/, float /*y*/, float /*z*/);
_ARMABI_INLINE _ARMABI long double fmal(long double __x, long double __y, long double __z) \
    { return (long double)fma((double)__x, (double)__y, (double)__z); }
extern _ARMABI_FPEXCEPT double fmax(double /*x*/, double /*y*/);
extern _ARMABI_FPEXCEPT float fmaxf(float /*x*/, float /*y*/);
_ARMDEFLD2(fmax);
extern _ARMABI_FPEXCEPT double fmin(double /*x*/, double /*y*/);
extern _ARMABI_FPEXCEPT float fminf(float /*x*/, float /*y*/);
_ARMDEFLD2(fmin);
extern _ARMABI double log2(double /*x*/); /* * log base 2 of x. */
extern _ARMABI float log2f(float /*x*/);
_ARMDEFLD1(log2);
extern _ARMABI long lrint(double /*x*/);
extern _ARMABI long lrintf(float /*x*/);
_ARMABI_INLINE _ARMABI long lrintl(long double __x) \
    { return lrint((double)__x); }
extern _ARMABI __int64 llrint(double /*x*/);
extern _ARMABI __int64 llrintf(float /*x*/);
_ARMABI_INLINE _ARMABI __int64 llrintl(long double __x) \
    { return llrint((double)__x); }
extern _ARMABI long lround(double /*x*/);
extern _ARMABI long lroundf(float /*x*/);
_ARMABI_INLINE _ARMABI long lroundl(long double __x) \
    { return lround((double)__x); }
extern _ARMABI __int64 llround(double /*x*/);
extern _ARMABI __int64 llroundf(float /*x*/);
_ARMABI_INLINE _ARMABI __int64 llroundl(long double __x) \
    { return llround((double)__x); }
extern _ARMABI_PURE double nan(const char */*tagp*/);
extern _ARMABI_PURE float nanf(const char */*tagp*/);
_ARMABI_INLINE _ARMABI_PURE long double nanl(const char *__t) \
    { return (long double)nan(__t); }
#if defined(_WANT_SNAN) && defined(__SUPPORT_SNAN__)
extern _ARMABI_PURE double nans(const char */*tagp*/);
extern _ARMABI_PURE float nansf(const char */*tagp*/);
_ARMABI_INLINE _ARMABI_FPEXCEPT long double nansl(const char *__t) \
    { return (long double)nans(__t); }
#endif 
extern _ARMABI_FPEXCEPT double nearbyint(double /*x*/);
extern _ARMABI_FPEXCEPT float nearbyintf(float /*x*/);
_ARMDEFLD1(nearbyint);
extern  double remquo(double /*x*/, double /*y*/, int */*quo*/);
extern  float remquof(float /*x*/, float /*y*/, int */*quo*/);
_ARMABI_INLINE long double remquol(long double __x, long double __y, int *__q) \
    { return (long double)remquo((double)__x, (double)__y, __q); }
extern _ARMABI_FPEXCEPT double round(double /*x*/);
extern _ARMABI_FPEXCEPT float roundf(float /*x*/);
_ARMDEFLD1(round);
extern _ARMABI double tgamma(double /*x*/); /* * The gamma function of x. */
extern _ARMABI float tgammaf(float /*x*/);
_ARMDEFLD1(tgamma);
extern _ARMABI_FPEXCEPT double trunc(double /*x*/);
extern _ARMABI_FPEXCEPT float truncf(float /*x*/);
_ARMDEFLD1(trunc);
#endif

#undef _ARMDEFLD1
#undef _ARMDEFLD1P
#undef _ARMDEFLD2

#ifdef __cplusplus
  extern "C++" {
    inline float abs(float __x)   { return fabsf(__x); }
    inline float acos(float __x)  { return acosf(__x); }
    inline float asin(float __x)  { return asinf(__x); }
    inline float atan(float __x)  { return atanf(__x); }
    inline float atan2(float __y, float __x)    { return atan2f(__y,__x); }
    inline float ceil(float __x)  { return ceilf(__x); }
    inline float cos(float __x)   { return cosf(__x); }
    inline float cosh(float __x)  { return coshf(__x); }
    inline float exp(float __x)   { return expf(__x); }
    inline float fabs(float __x)  { return fabsf(__x); }
    inline float floor(float __x) { return floorf(__x); }
    inline float fmod(float __x, float __y)     { return fmodf(__x, __y); }
    float frexp(float __x, int* __exp) __attribute__((__nonnull__(2)));
    inline float frexp(float __x, int* __exp)   { return frexpf(__x, __exp); }
    inline float ldexp(float __x, int __exp)    { return ldexpf(__x, __exp);}
    inline float log(float __x)   { return logf(__x); }
    inline float log10(float __x) { return log10f(__x); }
    float modf(float __x, float* __iptr) __attribute__((__nonnull__(2)));
    inline float modf(float __x, float* __iptr) { return modff(__x, __iptr); }
    inline float pow(float __x, float __y)      { return powf(__x,__y); }
    inline float pow(float __x, int __y)     { return powf(__x, (float)__y); }
    inline float sin(float __x)   { return sinf(__x); }
    inline float sinh(float __x)  { return sinhf(__x); }
    inline float sqrt(float __x)  { return sqrtf(__x); }
    inline float _sqrt(float __x) { return _sqrtf(__x); }
    inline float tan(float __x)   { return tanf(__x); }
    inline float tanh(float __x)  { return tanhf(__x); }

    inline double abs(double __x) { return fabs(__x); }
    inline double pow(double __x, int __y)
                { return pow(__x, (double) __y); }

    inline long double abs(long double __x)
                { return (long double)fabsl(__x); }
    inline long double acos(long double __x)
                { return (long double)acosl(__x); }
    inline long double asin(long double __x)
                { return (long double)asinl(__x); }
    inline long double atan(long double __x)
                { return (long double)atanl(__x); }
    inline long double atan2(long double __y, long double __x)
                { return (long double)atan2l(__y, __x); }
    inline long double ceil(long double __x)
                { return (long double)ceill( __x); }
    inline long double cos(long double __x)
                { return (long double)cosl(__x); }
    inline long double cosh(long double __x)
                { return (long double)coshl(__x); }
    inline long double exp(long double __x)
                { return (long double)expl(__x); }
    inline long double fabs(long double __x)
                { return (long double)fabsl(__x); }
    inline long double floor(long double __x)
                { return (long double)floorl(__x); }
    inline long double fmod(long double __x, long double __y)
                { return (long double)fmodl(__x, __y); }
    long double frexp(long double __x, int* __p) __attribute__((__nonnull__(2)));
    inline long double frexp(long double __x, int* __p)
                { return (long double)frexpl(__x, __p); }
    inline long double ldexp(long double __x, int __exp)
                { return (long double)ldexpl(__x, __exp); }
    inline long double log(long double __x)
                { return (long double)logl(__x); }
    inline long double log10(long double __x)
                { return (long double)log10l(__x); }
    long double modf(long double __x, long double* __p) __attribute__((__nonnull__(2)));
    inline long double modf(long double __x, long double* __p)
                { return (long double)modfl(__x, __p); }
    inline long double pow(long double __x, long double __y)
                { return (long double)powl(__x, __y); }
    inline long double pow(long double __x, int __y)
                { return (long double)powl(__x, __y); }
    inline long double sin(long double __x)
                { return (long double)sinl(__x); }
    inline long double sinh(long double __x)
                { return (long double)sinhl(__x); }
    inline long double sqrt(long double __x)
                { return (long double)sqrtl(__x); }
    inline long double _sqrt(long double __x)
                { return (long double)_sqrt((double) __x); }
    inline long double tan(long double __x)
                { return (long double)tanl(__x); }
    inline long double tanh(long double __x)
                { return (long double)tanhl(__x); }

#if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)
    inline float acosh(float __x) { return acoshf(__x); }
    inline float asinh(float __x) { return asinhf(__x); }
    inline float atanh(float __x) { return atanhf(__x); }
    inline float cbrt(float __x) { return cbrtf(__x); }
    inline float erf(float __x) { return erff(__x); }
    inline float erfc(float __x) { return erfcf(__x); }
    inline float expm1(float __x) { return expm1f(__x); }
    inline float log1p(float __x) { return log1pf(__x); }
    inline float hypot(float __x, float __y) { return hypotf(__x, __y); }
    inline float lgamma(float __x) { return lgammaf(__x); }
    inline float remainder(float __x, float __y) { return remainderf(__x, __y); }
    inline float rint(float __x) { return rintf(__x); }
#endif

#ifdef __USE_C99_MATH
    inline float exp2(float __x) { return exp2f(__x); }
    inline float fdim(float __x, float __y) { return fdimf(__x, __y); }
    inline float fma(float __x, float __y, float __z) { return fmaf(__x, __y, __z); }
    inline float fmax(float __x, float __y) { return fmaxf(__x, __y); }
    inline float fmin(float __x, float __y) { return fminf(__x, __y); }
    inline float log2(float __x) { return log2f(__x); }
    inline _ARMABI long lrint(float __x) { return lrintf(__x); }
    inline _ARMABI __int64 llrint(float __x) { return llrintf(__x); }
    inline _ARMABI long lround(float __x) { return lroundf(__x); }
    inline _ARMABI __int64 llround(float __x) { return llroundf(__x); }
    inline _ARMABI_FPEXCEPT float nearbyint(float __x) { return nearbyintf(__x); }
    inline float remquo(float __x, float __y, int *__q) { return remquof(__x, __y, __q); }
    inline _ARMABI_FPEXCEPT float round(float __x) { return roundf(__x); }
    inline float tgamma(float __x) { return tgammaf(__x); }
    inline _ARMABI_FPEXCEPT float trunc(float __x) { return truncf(__x); }

    inline long double acosh(long double __x) { return acoshl(__x); }
    inline long double asinh(long double __x) { return asinhl(__x); }
    inline long double atanh(long double __x) { return atanhl(__x); }
    inline long double cbrt(long double __x) { return cbrtl(__x); }
    inline long double erf(long double __x) { return erfl(__x); }
    inline long double erfc(long double __x) { return erfcl(__x); }
    inline long double expm1(long double __x) { return expm1l(__x); }
    inline long double log1p(long double __x) { return log1pl(__x); }
    inline long double hypot(long double __x, long double __y) { return hypotl(__x, __y); }
    inline long double lgamma(long double __x) { return lgammal(__x); }
    inline long double remainder(long double __x, long double __y) { return remainderl(__x, __y); }
    inline long double rint(long double __x) { return rintl(__x); }
    inline long double exp2(long double __x) { return exp2l(__x); }
    inline long double fdim(long double __x, long double __y) { return fdiml(__x, __y); }
    inline long double fma(long double __x, long double __y, long double __z) { return fmal(__x, __y, __z); }
    inline long double fmax(long double __x, long double __y) { return fmaxl(__x, __y); }
    inline long double fmin(long double __x, long double __y) { return fminl(__x, __y); }
    inline long double log2(long double __x) { return log2l(__x); }
    inline _ARMABI long lrint(long double __x) { return lrintl(__x); }
    inline _ARMABI __int64 llrint(long double __x) { return llrintl(__x); }
    inline _ARMABI long lround(long double __x) { return lroundl(__x); }
    inline _ARMABI __int64 llround(long double __x) { return llroundl(__x); }
    inline _ARMABI_FPEXCEPT long double nearbyint(long double __x) { return nearbyintl(__x); }
    inline long double remquo(long double __x, long double __y, int *__q) { return remquol(__x, __y, __q); }
    inline _ARMABI_FPEXCEPT long double round(long double __x) { return roundl(__x); }
    inline long double tgamma(long double __x) { return tgammal(__x); }
    inline _ARMABI_FPEXCEPT long double trunc(long double __x) { return truncl(__x); }
#endif

  }
#endif

    #ifdef __cplusplus
        }  /* extern "C" */
      }  /* namespace std */
    #endif
  #endif /* __MATH_DECLS */

  #if _AEABI_PORTABILITY_LEVEL != 0 && !defined _AEABI_PORTABLE
    #define _AEABI_PORTABLE
  #endif

  #if defined(__cplusplus) && !defined(__MATH_NO_EXPORTS)
    using ::std::__use_accurate_range_reduction;
    using ::std::abs;
    using ::std::acos;
    using ::std::asin;
    using ::std::atan2;
    using ::std::atan;
    using ::std::ceil;
    using ::std::cos;
    using ::std::cosh;
    using ::std::exp;
    using ::std::fabs;
    using ::std::floor;
    using ::std::fmod;
    using ::std::frexp;
    using ::std::ldexp;
    using ::std::log10;
    using ::std::log;
    using ::std::modf;
    using ::std::pow;
    using ::std::sin;
    using ::std::sinh;
    using ::std::sqrt;
    using ::std::_sqrt;
    using ::std::_sqrtf;
    using ::std::tan;
    using ::std::tanh;
    using ::std::_fabsf;
    /* C99 float and long double versions in already-C89-reserved namespace */
    using ::std::acosf;
    using ::std::acosl;
    using ::std::asinf;
    using ::std::asinl;
    using ::std::atan2f;
    using ::std::atan2l;
    using ::std::atanf;
    using ::std::atanl;
    using ::std::ceilf;
    using ::std::ceill;
    using ::std::cosf;
    using ::std::coshf;
    using ::std::coshl;
    using ::std::cosl;
    using ::std::expf;
    using ::std::expl;
    using ::std::fabsf;
    using ::std::fabsl;
    using ::std::floorf;
    using ::std::floorl;
    using ::std::fmodf;
    using ::std::fmodl;
    using ::std::frexpf;
    using ::std::frexpl;
    using ::std::ldexpf;
    using ::std::ldexpl;
    using ::std::log10f;
    using ::std::log10l;
    using ::std::logf;
    using ::std::logl;
    using ::std::modff;
    using ::std::modfl;
    using ::std::powf;
    using ::std::powl;
    using ::std::sinf;
    using ::std::sinhf;
    using ::std::sinhl;
    using ::std::sinl;
    using ::std::sqrtf;
    using ::std::sqrtl;
    using ::std::tanf;
    using ::std::tanhf;
    using ::std::tanhl;
    using ::std::tanl;
    #if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)
      /* C99 additions which for historical reasons appear in non-strict mode */
      using ::std::acosh;
      using ::std::asinh;
      using ::std::atanh;
      using ::std::cbrt;
      using ::std::copysign;
      using ::std::copysignf;
      using ::std::erf;
      using ::std::erfc;
      using ::std::expm1;
      using ::std::hypot;
      using ::std::ilogb;
      using ::std::ilogbf;
      using ::std::ilogbl;
      using ::std::lgamma;
      using ::std::log1p;
      using ::std::logb;
      using ::std::logbf;
      using ::std::logbl;
      using ::std::nextafter;
      using ::std::nextafterf;
      using ::std::nextafterl;
      using ::std::nexttoward;
      using ::std::nexttowardf;
      using ::std::nexttowardl;
      using ::std::remainder;
      using ::std::rint;
      using ::std::scalbln;
      using ::std::scalblnf;
      using ::std::scalblnl;
      using ::std::scalbn;
      using ::std::scalbnf;
      using ::std::scalbnl;
      using ::std::math_errhandling;
      using ::std::acoshf;
      using ::std::acoshl;
      using ::std::asinhf;
      using ::std::asinhl;
      using ::std::atanhf;
      using ::std::atanhl;
      using ::std::copysignl;
      using ::std::cbrtf;
      using ::std::cbrtl;
      using ::std::erff;
      using ::std::erfl;
      using ::std::erfcf;
      using ::std::erfcl;
      using ::std::expm1f;
      using ::std::expm1l;
      using ::std::log1pf;
      using ::std::log1pl;
      using ::std::hypotf;
      using ::std::hypotl;
      using ::std::lgammaf;
      using ::std::lgammal;
      using ::std::remainderf;
      using ::std::remainderl;
      using ::std::rintf;
      using ::std::rintl;
    #endif
    #if !defined(__STRICT_ANSI__) || defined(__USE_C99_MATH)
      /* C99 additions which appear in C99 or non-strict mode */
      using ::std::float_t;
      using ::std::double_t;
    #endif
    #ifdef __USE_C99_MATH
      /* Functions new in C99. */
      using ::std::exp2;
      using ::std::exp2f;
      using ::std::exp2l;
      using ::std::fdim;
      using ::std::fdimf;
      using ::std::fdiml;
      using ::std::fma;
      using ::std::fmaf;
      using ::std::fmal;
      using ::std::fmax;
      using ::std::fmaxf;
      using ::std::fmaxl;
      using ::std::fmin;
      using ::std::fminf;
      using ::std::fminl;
      using ::std::log2;
      using ::std::log2f;
      using ::std::log2l;
      using ::std::lrint;
      using ::std::lrintf;
      using ::std::lrintl;
      using ::std::llrint;
      using ::std::llrintf;
      using ::std::llrintl;
      using ::std::lround;
      using ::std::lroundf;
      using ::std::lroundl;
      using ::std::llround;
      using ::std::llroundf;
      using ::std::llroundl;
      using ::std::nan;
      using ::std::nanf;
      using ::std::nanl;
      using ::std::nearbyint;
      using ::std::nearbyintf;
      using ::std::nearbyintl;
      using ::std::remquo;
      using ::std::remquof;
      using ::std::remquol;
      using ::std::round;
      using ::std::roundf;
      using ::std::roundl;
      using ::std::tgamma;
      using ::std::tgammaf;
      using ::std::tgammal;
      using ::std::trunc;
      using ::std::truncf;
      using ::std::truncl;
    #endif
  #endif

#endif /* __math_h */

/* end of math.h */

