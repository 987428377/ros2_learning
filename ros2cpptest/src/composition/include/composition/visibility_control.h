//这段代码定义了一个用于控制库的可见性的头文件，主要用于在跨平台的 C++ 项目中管理库符号的导入和导出。
//它借鉴了 GCC 的可见性控制机制，并扩展了对 Windows 平台的支持。以下是详细的解释：

#ifndef COMPOSITION__VISIBILITY_CONTROL_H_
#define COMPOSITION__VISIBILITY_CONTROL_H_

//C++ 外部链接，这是为了确保在 C++ 编译器中能够正确处理 C 语言的链接方式，使得 C++ 代码可以调用 C 函数，或者被 C 代码调用。
#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COMPOSITION_EXPORT __attribute__ ((dllexport))
    #define COMPOSITION_IMPORT __attribute__ ((dllimport))
  #else
    #define COMPOSITION_EXPORT __declspec(dllexport)
    #define COMPOSITION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COMPOSITION_BUILDING_DLL
    #define COMPOSITION_PUBLIC COMPOSITION_EXPORT
  #else
    #define COMPOSITION_PUBLIC COMPOSITION_IMPORT
  #endif
  #define COMPOSITION_PUBLIC_TYPE COMPOSITION_PUBLIC
  #define COMPOSITION_LOCAL
#else
  #define COMPOSITION_EXPORT __attribute__ ((visibility("default"))) 
  #define COMPOSITION_IMPORT
  #if __GNUC__ >= 4 //：检查 GNU 编译器的版本是否为 4 或更高，版本 4 引入了可见性控制的特性。
    #define COMPOSITION_PUBLIC __attribute__ ((visibility("default"))) //设置符号的可见性为默认，即对外部可见，用于导出符号。
    #define COMPOSITION_LOCAL  __attribute__ ((visibility("hidden")))  //将符号标记为隐藏，仅对内部可见。用于实现细节，不对外部暴露。
  #else
    #define COMPOSITION_PUBLIC
    #define COMPOSITION_LOCAL
  #endif
  #define COMPOSITION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // COMPOSITION__VISIBILITY_CONTROL_H_