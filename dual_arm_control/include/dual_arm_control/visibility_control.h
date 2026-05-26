#ifndef DUAL_ARM_CONTROL__VISIBILITY_CONTROL_H_
#define DUAL_ARM_CONTROL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then modified) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUAL_ARM_CONTROL_EXPORT __attribute__ ((dllexport))
    #define DUAL_ARM_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define DUAL_ARM_CONTROL_EXPORT __declspec(dllexport)
    #define DUAL_ARM_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUAL_ARM_CONTROL_BUILDING_DLL
    #define DUAL_ARM_CONTROL_PUBLIC DUAL_ARM_CONTROL_EXPORT
  #else
    #define DUAL_ARM_CONTROL_PUBLIC DUAL_ARM_CONTROL_IMPORT
  #endif
  #define DUAL_ARM_CONTROL_LOCAL
#else
  #define DUAL_ARM_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define DUAL_ARM_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define DUAL_ARM_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define DUAL_ARM_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUAL_ARM_CONTROL_PUBLIC
    #define DUAL_ARM_CONTROL_LOCAL
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // DUAL_ARM_CONTROL__VISIBILITY_CONTROL_H_
