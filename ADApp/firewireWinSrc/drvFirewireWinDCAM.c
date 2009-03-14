/** \file drvFirewireDCAM.c
 * \brief Exporting the configuration functions to the ioc shell.
 */

#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "drvFirewireWinDCAM.h"

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"ID", iocshArgString};
static const iocshArg configArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg configArg3 = {"maxMemory", iocshArgInt};
static const iocshArg configArg4 = {"priority", iocshArgInt};
static const iocshArg configArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4,
                                              &configArg5};
static const iocshFuncDef configFirewireWinDCAM = {"WinFDC_Config", 6, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    WinFDC_Config(args[0].sval, args[1].sval, args[2].ival, 
                  args[3].ival, args[4].ival, args[5].ival);
}


static void firewireWinDCAMRegister(void)
{

    iocshRegister(&configFirewireWinDCAM, configCallFunc);
}

epicsExportRegistrar(firewireWinDCAMRegister);
