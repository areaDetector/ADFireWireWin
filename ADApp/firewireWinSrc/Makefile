TOP=../..
include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_WIN32 += firewireWinDCAM
# The following are compiled and added to the support library
LIB_SRCS += firewireWinDCAM.cpp
LIB_INSTALLS += ../1394camera.lib

PROD_IOC_WIN32 += firewireWinDCAMApp

DBD += firewireWinDCAMSupport.dbd
#==================================
# <name>.dbd will be created from <name>Include.dbd
DBD += firewireWinDCAMApp.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += firewireWinDCAMApp_registerRecordDeviceDriver.cpp firewireWinDCAMAppMain.cpp

# Add locally compiled object code
PROD_LIBS += firewireWinDCAM
PROD_LIBS += 1394camera

include $(TOP)/ADApp/commonDriverMakefile

# If we don't include nafxcw in the link command then VS links the C runtime before
# MFC and we get errors that operator new and delete are multiply defined.
ifeq ($(T_A), win32-x86)
PROD_SYS_LIBS      += nafxcw
endif
ifeq ($(T_A), win32-x86-debug)
PROD_SYS_LIBS      += nafxcwd
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

