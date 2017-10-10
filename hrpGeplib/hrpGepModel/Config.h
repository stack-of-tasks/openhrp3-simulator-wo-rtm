
#ifndef HRPGEPMODEL_CONFIG_H_INCLUDED
#define HRPGEPMODEL_CONFIG_H_INCLUDED

#define HRPGEPMODEL_VERSION_MAJOR 
#define HRPGEPMODEL_VERSION_MINOR 
#define HRPGEPMODEL_VERSION_MICRO 

#define OPENHRPGEP_DIR "/home/ostasse/devel/test/install"
#define OPENHRPGEP_SHARE_DIR "/home/ostasse/devel/test/install/"
#define OPENHRPGEP_RELATIVE_SHARE_DIR ""

// for Windows DLL export 
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef HRPGEPMODEL_MAKE_DLL
#   define HRPGEPMODEL_API __declspec(dllexport)
# else 
#   define HRPGEPMODEL_API __declspec(dllimport)
# endif
#else 
# define HRPGEPMODEL_API
#endif /* Windows */

#endif
