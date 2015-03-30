errlogInit(20000)

< envPaths

dbLoadDatabase("$(TOP)/dbd/firewireWinDCAMApp.dbd")

firewireWinDCAMApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "13FW1:")
# The port name for the detector
epicsEnvSet("PORT",   "FW1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "1376")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1024")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# This is the Thorlabs camera
#WinFDC_Config("$(PORT)", "116442682213159680", 0, 0)

# This is the SONY camera
#WinFDC_Config("$(PORT)", "163818473825504512", 0, 0)

# This will use the first camera found without needing to know its ID
WinFDC_Config("$(PORT)", "", 0, 0)

asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

dbLoadRecords("$(ADFIREWIREWIN)/db/firewireDCAM.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadTemplate("firewire.substitutions")

# Create a standard arrays plugin, set it to get 8-bit data from the driver.
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)

# Use the following line for an 8-bit camera.  This is enough elements for 1376*1024*3, increase if needed.
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=4227072")

# Use the following line for an 12-bit or 16-bit camera.  This is enough elements for 1500x1000x1, increase if needed.
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=1500000")

# Create a second standard arrays plugin, set it to get 16-bit data from the driver.
NDStdArraysConfigure("Image2", 5, 0, "$(PORT)", 0, 0)

# This is enough elements for 1376*1024*3
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=Image2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=4227072")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot//commonPlugins.cmd
set_requestfile_path("$(ADFIREWIREWIN)/firewireWinApp/Db")

#asynSetTraceMask("$(PORT)",0,255)


iocInit()

#asynSetTraceMask("$(PORT)",0,1)

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")
