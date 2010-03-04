errlogInit(20000)

< envPaths

dbLoadDatabase("$(AREA_DETECTOR)/dbd/firewireWinDCAMApp.dbd")

firewireWinDCAMApp_registerRecordDeviceDriver(pdbbase) 

# This is the Thorlabs camera
#WinFDC_Config("FW1", "116442682213159680", 50, 200000000)
# This is the SONY camera
#WinFDC_Config("FW1", "163818473825504512", 50, 200000000)
# This will use the first camera found without needing to know its ID
WinFDC_Config("FW1", "", 50, 200000000)
asynSetTraceIOMask("FW1",0,2)
#asynSetTraceMask("FW1",0,255)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template",   "P=13FW1:,R=cam1:,PORT=FW1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",   "P=13FW1:,R=cam1:,PORT=FW1,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/firewireDCAM.template",   "P=13FW1:,R=cam1:,PORT=FW1,ADDR=0,TIMEOUT=1")
dbLoadTemplate("firewire.substitutions")

# Create a standard arrays plugin, set it to get 8-bit data from the driver.
NDStdArraysConfigure("FW1Image", 5, 0, "FW1", 0, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13FW1:,R=image1:,PORT=FW1Image,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
# Use the following line for an 8-bit camera.  This is enough elements for 1376*1024*3, increase if needed.
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=13FW1:,R=image1:,PORT=FW1Image,ADDR=0,TIMEOUT=1,TYPE=Int8,FTVL=UCHAR,NELEMENTS=4227072")
# Use the following line for an 12-bit or 16-bit camera.  This is enough elements for 1500x1000x1, increase if needed.
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=13FW1:,R=image1:,PORT=FW1Image,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=1500000")

# Create a second standard arrays plugin, set it to get 16-bit data from the driver.
NDStdArraysConfigure("FW1Image2", 5, 0, "FW1", 0, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13FW1:,R=image2:,PORT=FW1Image2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
# This is enough elements for 1376*1024*3
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=13FW1:,R=image2:,PORT=FW1Image2,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=4227072")

# Load the database to use with Stephen Mudie's IDL code
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/EPICS_AD_Viewer.template", "P=13FW1:, R=image1:")

# Create a netCDF file saving plugin
NDFileNetCDFConfigure("FW1FileNetCDF", 450, 0, "FW1", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13FW1:,R=netCDF1:,PORT=FW1FileNetCDF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13FW1:,R=netCDF1:,PORT=FW1FileNetCDF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=13FW1:,R=netCDF1:,PORT=FW1FileNetCDF,ADDR=0,TIMEOUT=1")

# Create a TIFF file saving plugin
NDFileTIFFConfigure("FW1FileTIFF", 20, 0, "FW1", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13FW1:,R=TIFF1:,PORT=FW1FileTIFF,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13FW1:,R=TIFF1:,PORT=FW1FileTIFF,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileTIFF.template",  "P=13FW1:,R=TIFF1:,PORT=FW1FileTIFF,ADDR=0,TIMEOUT=1")

# Create a JPEG file saving plugin
NDFileJPEGConfigure("FW1FileJPEG", 20, 0, "FW1", 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13FW1:,R=JPEG1:,PORT=FW1FileJPEG,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=13FW1:,R=JPEG1:,PORT=FW1FileJPEG,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileJPEG.template",  "P=13FW1:,R=JPEG1:,PORT=FW1FileJPEG,ADDR=0,TIMEOUT=1")

# Create an ROI plugin
NDROIConfigure("FW1ROI", 5, 0, "FW1", 0, 10, 20, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=13FW1:,R=ROI1:,  PORT=FW1ROI,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROI.template",       "P=13FW1:,R=ROI1:,  PORT=FW1ROI,ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=13FW1:,R=ROI1:0:,PORT=FW1ROI,ADDR=0,TIMEOUT=1,HIST_SIZE=256")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=13FW1:,R=ROI1:1:,PORT=FW1ROI,ADDR=1,TIMEOUT=1,HIST_SIZE=256")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=13FW1:,R=ROI1:2:,PORT=FW1ROI,ADDR=2,TIMEOUT=1,HIST_SIZE=256")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDROIN.template",      "P=13FW1:,R=ROI1:3:,PORT=FW1ROI,ADDR=3,TIMEOUT=1,HIST_SIZE=256")

# Create a color conversion plugin
NDColorConvertConfigure("FW1CC1", 5, 0, "FW1", 0, 20, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","   P=13FW1:,R=CC1:,  PORT=FW1CC1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FW1,NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDColorConvert.template", "P=13FW1:,R=CC1:,  PORT=FW1CC1,ADDR=0,TIMEOUT=1")

#asynSetTraceMask("FW1",0,255)

set_requestfile_path("./")
set_savefile_path("./autosave")
set_requestfile_path("$(AREA_DETECTOR)/ADApp/Db")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("13FW1:")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=13FW1:")

iocInit()

#asynSetTraceMask("FW1",0,1)

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=13FW1:,D=cam1:")
