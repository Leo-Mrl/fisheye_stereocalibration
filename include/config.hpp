#ifndef __CALIBRATION_CONFIG_HPP__
#define __CALIBRATION_CONFIG_HPP__

#if defined(__GNUC__)
  #define VS_EXPORT __attribute__((visibility("default")))
#elif defined(_MSC_VER)
  #ifdef calibration_EXPORTS
    #define VS_EXPORT __declspec(dllexport)
  #else
    #define VS_EXPORT
  # endif
#else
#define VS_EXPORT
#endif

#endif