#ifndef ACTION_MOVE_TO_GOAL_CPP__VISIBILITY_CONTROL_H_
#define ACTION_MOVE_TO_GOAL_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_MOVE_TO_GOAL_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_MOVE_TO_GOAL_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_MOVE_TO_GOAL_CPP_EXPORT __declspec(dllexport)
    #define ACTION_MOVE_TO_GOAL_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_MOVE_TO_GOAL_CPP_BUILDING_DLL
    #define ACTION_MOVE_TO_GOAL_CPP_PUBLIC ACTION_MOVE_TO_GOAL_CPP_EXPORT
  #else
    #define ACTION_MOVE_TO_GOAL_CPP_PUBLIC ACTION_MOVE_TO_GOAL_CPP_IMPORT
  #endif
  #define ACTION_MOVE_TO_GOAL_CPP_PUBLIC_TYPE ACTION_MOVE_TO_GOAL_CPP_PUBLIC
  #define ACTION_MOVE_TO_GOAL_CPP_LOCAL
#else
  #define ACTION_MOVE_TO_GOAL_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_MOVE_TO_GOAL_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_MOVE_TO_GOAL_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_MOVE_TO_GOAL_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_MOVE_TO_GOAL_CPP_PUBLIC
    #define ACTION_MOVE_TO_GOAL_CPP_LOCAL
  #endif
  #define ACTION_MOVE_TO_GOAL_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_MOVE_TO_GOAL_CPP__VISIBILITY_CONTROL_H_