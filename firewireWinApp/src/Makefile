TOP=../..
include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
  LIBRARY_IOC += firewireWinDCAM
  # The following are compiled and added to the support library
  LIB_SRCS += firewireWinDCAM.cpp
  LIB_INSTALLS += ../os/win32-x86/1394camera.lib
endif

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  LIBRARY_IOC += firewireWinDCAM
  # The following are compiled and added to the support library
  LIB_SRCS += firewireWinDCAM.cpp
  LIB_INSTALLS += ../os/windows-x64/1394camera.lib
endif

LIB_LIBS += 1394camera

ifeq ($(STATIC_BUILD), NO)
  USR_CXXFLAGS += -D_AFXDLL
endif

DBD += firewireWinDCAMSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

