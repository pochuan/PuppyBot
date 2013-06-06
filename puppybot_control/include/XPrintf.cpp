
#ifdef WIN32
#ifdef USING_XPRINTF
#include "XPrintf.h"

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}

#endif
#endif