#ifndef XARM_AS_VISIBILITY_CONTROL_H_
#define XARM_AS_VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define XARM_AS_EXPORT __attribute__ ((dllexport))
        #define XARM_AS_IMPORT __attribute__ ((dllimport))
    #else
        #define XARM_AS_EXPORT __declspec(dllexport)
        #define XARM_AS_IMPORT __declspec(dllimport)
    #endif
    #ifdef XARM_AS_BUILDING_DLL
        #define XARM_AS_PUBLIC XARM_AS_EXPORT
    #else
        #define XARM_AS_PUBLIC XARM_AS_IMPORT
    #endif
    #define XARM_AS_PUBLIC_TYPE XARM_AS_PUBLIC
    #define XARM_AS_LOCAL
#else
    #define XARM_AS_EXPORT __attribute__ ((visibility("default")))
    #define XARM_AS_IMPORT
    #if __GNUC__ >= 4
        #define XARM_AS_PUBLIC __attribute__ ((visibility("default")))
        #define XARM_AS_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define XARM_AS_PUBLIC
        #define XARM_AS_LOCAL
    #endif
    #define XARM_AS_PUBLIC_TYPE
#endif

#endif  // XARM_AS_VISIBILITY_CONTROL_H_