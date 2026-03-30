#ifdef _WIN32
  #ifdef DLL_LIBRARY_EXPORTS
    #define DLL_API __declspec(dllexport)
  #else
    #define DLL_API __declspec(dllimport)
  #endif
#else
  #define DLL_API
#endif