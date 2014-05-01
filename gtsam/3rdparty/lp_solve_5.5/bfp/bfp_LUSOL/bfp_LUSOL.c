
#include "bfp_LUSOL.h"
#include "lp_lib.h"
#include "lp_LUSOL.h"

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
    return TRUE;
}

#if defined FORTIFY
int EndOfPgr(int i)
{
    exit(i);
}
#endif
