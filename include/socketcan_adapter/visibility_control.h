#ifndef SOCKETCAN_ADAPTER__VISIBILITY_CONTROL_H_
#define SOCKETCAN_ADAPTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SOCKETCAN_ADAPTER_EXPORT __attribute__ ((dllexport))
    #define SOCKETCAN_ADAPTER_IMPORT __attribute__ ((dllimport))
  #else
    #define SOCKETCAN_ADAPTER_EXPORT __declspec(dllexport)
    #define SOCKETCAN_ADAPTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef SOCKETCAN_ADAPTER_BUILDING_LIBRARY
    #define SOCKETCAN_ADAPTER_PUBLIC SOCKETCAN_ADAPTER_EXPORT
  #else
    #define SOCKETCAN_ADAPTER_PUBLIC SOCKETCAN_ADAPTER_IMPORT
  #endif
  #define SOCKETCAN_ADAPTER_PUBLIC_TYPE SOCKETCAN_ADAPTER_PUBLIC
  #define SOCKETCAN_ADAPTER_LOCAL
#else
  #define SOCKETCAN_ADAPTER_EXPORT __attribute__ ((visibility("default")))
  #define SOCKETCAN_ADAPTER_IMPORT
  #if __GNUC__ >= 4
    #define SOCKETCAN_ADAPTER_PUBLIC __attribute__ ((visibility("default")))
    #define SOCKETCAN_ADAPTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SOCKETCAN_ADAPTER_PUBLIC
    #define SOCKETCAN_ADAPTER_LOCAL
  #endif
  #define SOCKETCAN_ADAPTER_PUBLIC_TYPE
#endif

#endif  // SOCKETCAN_ADAPTER__VISIBILITY_CONTROL_H_
