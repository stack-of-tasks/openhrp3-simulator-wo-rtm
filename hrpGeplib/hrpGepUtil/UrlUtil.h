/*! @file
  @author Takafumi.Tawara
*/

#ifndef OPENHRPGEP_UTIL_URL_UTIL_H_INCLUDED
#define OPENHRPGEP_UTIL_URL_UTIL_H_INCLUDED

#include "config.h"
#include <string>
using namespace std;

namespace hrpGep
{
    HRP_UTIL_EXPORT string deleteURLScheme(string url);
    HRP_UTIL_EXPORT void getPathFromUrl(string& refUrl, const string& rootDir, string srcUrl);
    HRP_UTIL_EXPORT bool isFileProtocol(const string& ref);
}
#endif
