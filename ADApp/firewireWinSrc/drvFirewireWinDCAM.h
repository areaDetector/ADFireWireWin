/** drvFirewireWinDCAM.h
 */

#ifndef DRV_FIREWIREWINDCAM_H
#define DRV_FIREWIREWINDCAM_H

#ifdef __cplusplus
extern "C" {
#endif

int WinFDC_Config(const char *portName, const char* camid, int maxBuffers, size_t maxMemory, int queueSize, int priority);

#ifdef __cplusplus
}
#endif
#endif
