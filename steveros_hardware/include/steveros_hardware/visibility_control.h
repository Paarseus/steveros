#ifndef STEVEROS_HARDWARE__VISIBILITY_CONTROL_H_
#define STEVEROS_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STEVEROS_HARDWARE_EXPORT __attribute__((dllexport))
    #define STEVEROS_HARDWARE_IMPORT __attribute__((dllimport))
  #else
    #define STEVEROS_HARDWARE_EXPORT __declspec(dllexport)
    #define STEVEROS_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef STEVEROS_HARDWARE_BUILDING_DLL
    #define STEVEROS_HARDWARE_PUBLIC STEVEROS_HARDWARE_EXPORT
  #else
    #define STEVEROS_HARDWARE_PUBLIC STEVEROS_HARDWARE_IMPORT
  #endif
  #define STEVEROS_HARDWARE_LOCAL
#else
  #define STEVEROS_HARDWARE_EXPORT __attribute__((visibility("default")))
  #define STEVEROS_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define STEVEROS_HARDWARE_PUBLIC __attribute__((visibility("default")))
    #define STEVEROS_HARDWARE_LOCAL __attribute__((visibility("hidden")))
  #else
    #define STEVEROS_HARDWARE_PUBLIC
    #define STEVEROS_HARDWARE_LOCAL
  #endif
#endif

#endif  // STEVEROS_HARDWARE__VISIBILITY_CONTROL_H_
