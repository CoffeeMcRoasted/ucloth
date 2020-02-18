
#ifndef UCLOTH_DLL_EXPORT_H_
#define UCLOTH_DLL_EXPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _WIN32
#ifdef UCLOTH_API_EXPORTS
#define ucloth_export __declspec(dllexport)
#else
#define ucloth_export __declspec(dllimport)
#endif
#else
#define ucloth_export
#endif

#ifdef __cplusplus
}
#endif

#endif  // !UCLOTH_DLL_EXPORT_H_