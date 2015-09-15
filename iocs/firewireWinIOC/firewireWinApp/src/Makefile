TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = firewireWinDCAMApp
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  PROD_IOC += $(PROD_NAME)
endif
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
  PROD_IOC += $(PROD_NAME)
endif

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += firewireWinDCAMSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += firewireWinDCAM
PROD_LIBS += 1394camera

include $(ADCORE)/ADApp/commonDriverMakefile

# If we don't include nafxcw in the link command then VS links the C runtime before
# MFC and we get errors that operator new and delete are multiply defined.
ifeq (debug, findstring(debug, $(T_A)))
PROD_SYS_LIBS      += nafxcwd
else
PROD_SYS_LIBS      += nafxcw
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

