#ifndef BPE_MSGS_CPP__VISIBILITY_CONTROL_H_
#define BPE_MSGS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BPE_MSGS_CPP_EXPORT __attribute__ ((dllexport))
    #define BPE_MSGS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define BPE_MSGS_CPP_EXPORT __declspec(dllexport)
    #define BPE_MSGS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef BPE_MSGS_CPP_BUILDING_DLL
    #define BPE_MSGS_CPP_PUBLIC BPE_MSGS_CPP_EXPORT
  #else
    #define BPE_MSGS_CPP_PUBLIC BPE_MSGS_CPP_IMPORT
  #endif
  #define BPE_MSGS_CPP_PUBLIC_TYPE BPE_MSGS_CPP_PUBLIC
  #define BPE_MSGS_CPP_LOCAL
#else
  #define BPE_MSGS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define BPE_MSGS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define BPE_MSGS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define BPE_MSGS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BPE_MSGS_CPP_PUBLIC
    #define BPE_MSGS_CPP_LOCAL
  #endif
  #define BPE_MSGS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // BPE_MSGS_CPP__VISIBILITY_CONTROL_H_