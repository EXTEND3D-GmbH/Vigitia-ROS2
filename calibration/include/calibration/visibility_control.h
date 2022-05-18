#ifndef PROJECTOR_CALIBRATION_CPP__VISIBILITY_CONTROL_H_
#define PROJECTOR_CALIBRATION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PROJECTOR_CALIBRATION_CPP_EXPORT __attribute__( ( dllexport ) )
#define PROJECTOR_CALIBRATION_CPP_IMPORT __attribute__( ( dllimport ) )
#else
#define PROJECTOR_CALIBRATION_CPP_EXPORT __declspec( dllexport )
#define PROJECTOR_CALIBRATION_CPP_IMPORT __declspec( dllimport )
#endif
#ifdef PROJECTOR_CALIBRATION_CPP_BUILDING_DLL
#define PROJECTOR_CALIBRATION_CPP_PUBLIC PROJECTOR_CALIBRATION_CPP_EXPORT
#else
#define PROJECTOR_CALIBRATION_CPP_PUBLIC PROJECTOR_CALIBRATION_CPP_IMPORT
#endif
#define PROJECTOR_CALIBRATION_CPP_PUBLIC_TYPE PROJECTOR_CALIBRATION_CPP_PUBLIC
#define PROJECTOR_CALIBRATION_CPP_LOCAL
#else
#define PROJECTOR_CALIBRATION_CPP_EXPORT __attribute__( ( visibility( "default" ) ) )
#define PROJECTOR_CALIBRATION_CPP_IMPORT
#if __GNUC__ >= 4
#define PROJECTOR_CALIBRATION_CPP_PUBLIC __attribute__( ( visibility( "default" ) ) )
#define PROJECTOR_CALIBRATION_CPP_LOCAL __attribute__( ( visibility( "hidden" ) ) )
#else
#define PROJECTOR_CALIBRATION_CPP_PUBLIC
#define PROJECTOR_CALIBRATION_CPP_LOCAL
#endif
#define PROJECTOR_CALIBRATION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif	// PROJECTOR_CALIBRATION_CPP__VISIBILITY_CONTROL_H_