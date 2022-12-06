#pragma once

/**
@file GNSSCore_Api.h
@brief defines and constants for GNSSCore dll
*/

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the GNSSCORE_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// GNSSCORE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
/// <summary>import/export macro for GNSSCore dll</summary>
#if defined(GNSSCORE_API)
#elif defined(GNSSCORE_EXPORTS)
#define GNSSCORE_API __declspec(dllexport)
#else
#define GNSSCORE_API __declspec(dllimport)
#endif

/// <summary>constants for GNSSCore dll</summary>
