/*
 * Author: Mark Rivers 
 *         University of Chicago
 *
 * License: This file is part of 'areaDetector'
 * 
 * 'firewireDCAM' is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * 'firewireWinDCAM' is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with 'firewireWinDCAM'.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/** firewireWinDCAM.cpp
 *  This is areaDetector driver support for firewire cameras that comply
 *  with the IIDC DCAM protocol. This implements the FirewireWinDCAM class which
 *  inherits from the areaDetector ADDriver class.
 *
 *  The driver uses the 1394Camera library from Carnegie-Mellon University
 *
 *  Created: March 2009
 *
 */

/* Standard includes... */
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* EPICS includes */
#include <epicsString.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>

/* Dependency support modules includes:
 * asyn, areaDetector, dc1394 and raw1394 */
#include <ADStdDriverParams.h>
#include <NDArray.h>
#include <ADDriver.h>

#include <stdafx.h>

/* 1394Camera includes */
#include <1394Camera.h>

/** Print an errorcode to stderr. Convenience macro to be used when an asynUser is not yet available. */
#define ERR(errCode) if (errCode != 0) fprintf(stderr, "ERROR [%s:%d]: dc1394 code: %d\n", __FILE__, __LINE__, errCode)
/** Convenience macro to be used inside the firewireDCAM class. */
#define PERR(pasynUser, errCode) this->err(pasynUser, errCode, __LINE__)

#define MAX_1394_BUFFERS 6
#define MAX_1394_VIDEO_FORMATS 8
#define MAX_1394_VIDEO_MODES 8
#define MAX_1394_FRAME_RATES 8

/** Only used for debugging/error messages to identify where the message comes from*/
static const char *driverName = "FirewireWinDCAM";

/** Main driver class inherited from areaDetectors ADDriver class.
 * One instance of this class will control one firewire camera on the bus.
 */
class FirewireWinDCAM : public ADDriver
{
public:
    FirewireWinDCAM(const char *portName, const char* camid,
                 int maxBuffers, size_t maxMemory,
                 int priority, int queueSize);

    /* virtual methods to override from ADDriver */
    virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus drvUserCreate( asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
    void report(FILE *fp, int details);

    /* Local methods to this class */
    void imageGrabTask();
    int grabImage();
    asynStatus startCapture(asynUser *pasynUser);
    asynStatus stopCapture(asynUser *pasynUser);

    /* camera feature control functions */
    asynStatus setFeatureValue(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue);
    asynStatus setFeatureAbsValue(asynUser *pasynUser, epicsFloat64 value, epicsFloat64 *rbValue);
    asynStatus setFeatureMode(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue);
    C1394CameraControl* checkFeature(asynUser *pasynUser, char **featureName);
    asynStatus setVideoFormat( asynUser *pasynUser, epicsInt32 format);
    asynStatus setVideoMode( asynUser *pasynUser, epicsInt32 mode);
    asynStatus setFrameRate( asynUser *pasynUser, epicsInt32 rate);
    asynStatus setFormat7Params( asynUser *pasynUser);
    asynStatus formatValidModes();
    int getAllFeatures();
    asynStatus err( asynUser* asynUser, int CAM_err, int errOriginLine);


    /* Data */
    NDArray *pRaw;
    C1394Camera *pCamera;
    C1394CameraControlSize *pCameraControlSize;
    C1394CameraControl **pCameraControl;
    epicsEventId startEventId;
    epicsEventId stopEventId;
};
/* end of FirewireWinDCAM class description */

/** Configuration function to configure one camera.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instanciates one object from the FirewireWinDCAM class.
 * portName Asyn port name to assign to the camera.
 * camid The camera ID or serial number in a hexadecimal string. Lower case and
 *              upper case letters can be used. This is used to identify a specific camera
 *              on the bus. For instance: "0x00b09d01007139d0".
 * maxBuffers Maxiumum number of NDArray objects (image buffers) this driver is allowed to allocate.
 *                   This driver requires 2 buffers, and each queue element in a plugin can require one buffer
 *                   which will all need to be added up in this parameter.
 * maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *                  and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB)
 */
extern "C" int WinFDC_Config(const char *portName, const char* camid, int maxBuffers, size_t maxMemory, int priority, int queueSize)
{
    new FirewireWinDCAM( portName, camid, maxBuffers, maxMemory, priority, queueSize);
    return asynSuccess;
}

static char *errMsg[] = {
    "Success",                  // 0
    "Generic error",            //-1
    "Unknown error",            //-2
    "Unknown error",            //-3
    "Unknown error",            //-4
    "Unknown error",            //-5
    "Unknown error",            //-6
    "Unknown error",            //-7
    "Unknown error",            //-8
    "Unknown error",            //-9
    "Unsupported",              //-10
    "Not initialized",          //-11
    "Invalid video settings",   //-12
    "Busy",                     //-13
    "Insufficient resources",   //-14
    "Param out of range",       //-15
    "Frame timeout",            //-16
};

static char *videoFormatStrings[MAX_1394_VIDEO_FORMATS] = {
    "VGA",
    "Super VGA 1",
    "Super VGA 2",
    "Reserved",
    "Reserved",
    "Reserved",
    "Still image",
    "User-defined"
};

static char *videoModeStrings[MAX_1394_VIDEO_FORMATS][MAX_1394_VIDEO_MODES] = {
    {"160x120 YUV444",  "320x240 YUV422", "640x480 YUV411", "640X480 YUV422",   "640x480 RGB",   "640x480 Mono8",   "640x480 Mono16",  "Reserved"        },
    {"800x600 YUV422",  "800x600 RGB",    "800x600 Mono8",  "1024x768 YUV422",  "1024x768 RGB",  "1024x768 Mono8",  "800x600 Mono16",  "1024x768 Mono16" },
    {"1280x960 YUV422", "1280x960 RGB",   "1280x960 Mono8", "1600x1200 YUV422", "1600x1200 RGB", "1600x1200 Mono8", "1280x960 Mono16", "1600x1200 Mono16"},
    {"Reserved",        "Reserved",       "Reserved",       "Reserved",         "Reserved",      "Reserved",        "Reserved",        "Reserved"        },
    {"Reserved",        "Reserved",       "Reserved",       "Reserved",         "Reserved",      "Reserved",        "Reserved",        "Reserved"        },
    {"Reserved",        "Reserved",       "Reserved",       "Reserved",         "Reserved",      "Reserved",        "Reserved",        "Reserved"        },
    {"Exif",            "Reserved",       "Reserved",       "Reserved",         "Reserved",      "Reserved",        "Reserved",        "Reserved"        },
    {"Format7 Mode0",   "Format7 Mode1",  "Format7 Mode2",  "Format7 Mode3",    "Format7 Mode4", "Format7 Mode5",   "Format7 Mode6",   "Format7 Mode7"   },
};

static char *frameRateStrings[MAX_1394_FRAME_RATES] = {
    "1.875",
    "3.75",
    "7.5",
    "15",
    "30",
    "60",
    "120",
    "240"
};

/* This array converts from the 0-21 index used in the asyn addr field for features to the enum values
 * used by the CMU driver, which are not sequential */
static CAMERA_FEATURE featureIndex[] = {
    FEATURE_BRIGHTNESS,
    FEATURE_AUTO_EXPOSURE,
    FEATURE_SHARPNESS,
    FEATURE_WHITE_BALANCE,
    FEATURE_HUE,
    FEATURE_SATURATION,
    FEATURE_GAMMA,
    FEATURE_SHUTTER,
    FEATURE_GAIN,
    FEATURE_IRIS,
    FEATURE_FOCUS,
    FEATURE_TEMPERATURE,
    FEATURE_TRIGGER_MODE,
    FEATURE_TRIGGER_DELAY,
    FEATURE_WHITE_SHADING,
    FEATURE_FRAME_RATE,
    FEATURE_ZOOM,
    FEATURE_PAN,
    FEATURE_TILT,
    FEATURE_OPTICAL_FILTER,
    FEATURE_CAPTURE_SIZE,
    FEATURE_CAPTURE_QUALITY
};
static int num1394Features = sizeof(featureIndex) / sizeof(featureIndex[0]);

/** Specific asyn commands for this support module. These will be used and
 * managed by the parameter library (part of areaDetector). */
typedef enum FDCParam_t {
    FDC_feat_val = ADFirstDriverParam, /** Feature value (int32 read/write) addr: 0-17 */
    FDC_feat_val_max,                  /** Feature maximum boundry value (int32 read) addr: 0-17 */
    FDC_feat_val_min,                  /** Feature minimum boundry value (int32 read)  addr: 0-17*/
    FDC_feat_val_abs,                  /** Feature absolute value (float64 read/write) addr: 0-17 */
    FDC_feat_val_abs_max,              /** Feature absolute maximum boundry value (float64 read) addr: 0-17 */
    FDC_feat_val_abs_min,              /** Feature absolute minimum boundry value (float64 read) addr: 0-17 */
    FDC_feat_mode,                     /** Feature control mode: 0:manual or 1:automatic (camera controlled) (int32 read/write)*/
    FDC_feat_available,                /** Is a given featurea available in the camera 1=available 0=not available (int32, read) */
    FDC_feat_absolute,                 /** Feature has absolute (floating point) controls available 1=available 0=not available (int32 read) */
    FDC_format,                        /** Set and read back the video format (int32 (enums) read/write)*/
    FDC_mode,                          /** Set and read back the video mode (int32 (enums) read/write)*/
    FDC_framerate,                     /** Set and read back the frame rate (int32 (enums) read/write)*/
    FDC_valid_format,                  /** Read back the valid video formats (octet, read)*/
    FDC_valid_mode,                    /** Read back the valid video modes (octet, read)*/
    FDC_valid_framerate,               /** Read back the valid frame rates (octet, read)*/
    FDC_has_format,                    /** Read back whether video format is supported (int32, read)*/
    FDC_has_mode,                      /** Read back whether video mode is supported (int32, read)*/
    FDC_has_framerate,                 /** Read back whether video framerate is supported (int32, read)*/
    FDC_current_format,                /** Read back the current video format (octet, read)*/
    FDC_current_mode,                  /** Read back the current video mode (octet, read)*/
    FDC_current_framerate,             /** Read back the current frame rate (octet, read)*/
    ADLastDriverParam
} FDCParam_t;

static asynParamString_t FDCParamString[] = {
    {FDC_feat_val,           "FDC_FEAT_VAL"},
    {FDC_feat_val_max,       "FDC_FEAT_VAL_MAX"},
    {FDC_feat_val_min,       "FDC_FEAT_VAL_MIN"},
    {FDC_feat_val_abs,       "FDC_FEAT_VAL_ABS"},
    {FDC_feat_val_abs_max,   "FDC_FEAT_VAL_ABS_MAX"},
    {FDC_feat_val_abs_min,   "FDC_FEAT_VAL_ABS_MIN"},
    {FDC_feat_mode,          "FDC_FEAT_MODE"},
    {FDC_feat_available,     "FDC_FEAT_AVAILABLE"},
    {FDC_feat_absolute,      "FDC_FEAT_ABSOLUTE"},
    {FDC_format,             "FDC_FORMAT"},
    {FDC_mode,               "FDC_MODE"},
    {FDC_framerate,          "FDC_FRAMERATE"},
    {FDC_valid_format,       "FDC_VALID_FORMAT"},
    {FDC_valid_mode,         "FDC_VALID_MODE"},
    {FDC_valid_framerate,    "FDC_VALID_FRAMERATE"},
    {FDC_has_format,         "FDC_HAS_FORMAT"},
    {FDC_has_mode,           "FDC_HAS_MODE"},
    {FDC_has_framerate,      "FDC_HAS_FRAMERATE"},
    {FDC_current_format,     "FDC_CURRENT_FORMAT"},
    {FDC_current_mode,       "FDC_CURRENT_MODE"},
    {FDC_current_framerate,  "FDC_CURRENT_FRAMERATE"},
};

/** Number of asyn parameters (asyn commands) this driver supports. */
#define FDC_N_PARAMS (sizeof( FDCParamString)/ sizeof(FDCParamString[0]))

static void imageGrabTaskC(void *drvPvt)
{
    FirewireWinDCAM *pPvt = (FirewireWinDCAM *)drvPvt;

    pPvt->imageGrabTask();
}

/** Constructor for the FirewireWinDCAM class
 * Initialises the camera object by setting all the default parameters and initializing
 * the camera hardware with it. This function also reads out the current settings of the
 * camera and prints out a selection of parameters to the shell.
 * portName The asyn port name to give the particular instance.
 * camid The unique ID stored in the camera.
 * maxBuffers The largest number of image buffers this driver can create.
 * maxMemory The maximum amount of memory in bytes that the driver can allocate for images.
 * priority The priority for the asyn port driver and the imageGrabTask
 * queueSize The queue size for the asyn port driver and the imageGrabTask
 */
FirewireWinDCAM::FirewireWinDCAM(    const char *portName, const char* camid, 
                            int maxBuffers, size_t maxMemory, int priority, int queueSize )
    : ADDriver(portName, num1394Features, ADLastDriverParam, maxBuffers, maxMemory, 0, 0,
               ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, queueSize),
        pRaw(NULL)
{
    const char *functionName = "FirewireWinDCAM";
    char vendorName[256], cameraName[256];
    LARGE_INTEGER uniqueId;
    unsigned long long int camUID = 0;
    unsigned long version;
    int err;
    int numCameras;
    int selectedCamera = -1;
    unsigned short maxSizeX, maxSizeY;
    int i, status, ret;
    char chMode = 'A';

    /* parse the string of hex-numbers that is the cameras unique ID */
    ret = sscanf(camid, "%lld", &camUID);

    this->pCamera = new C1394Camera();
    numCameras = this->pCamera->RefreshCameraList();
    // Print out information about all the cameras found
    printf("%s::%s: %d cameras found\n", driverName, functionName, numCameras);
    for (i=0; i<numCameras; i++) {
        printf("  camera %d\n", i);
        err = this->pCamera->SelectCamera(i);
        status = PERR(this->pasynUserSelf, err);
        this->pCamera->GetCameraVendor(vendorName, sizeof(vendorName));
        this->pCamera->GetCameraName(cameraName, sizeof(cameraName));
        this->pCamera->GetCameraUniqueID(&uniqueId);
        if (uniqueId.QuadPart == camUID) selectedCamera=i;
        version = this->pCamera->GetVersion();
        printf("Vendor name: %s\n", vendorName);
        printf("Camera name: %s\n", cameraName);
        printf("UniqueId: %lld (0x%llX)\n", uniqueId.QuadPart, uniqueId.QuadPart);
        printf("Version: 0x%lX\n", version);
    }
            
    /* If we didn't find the camera with the specific ID we return... */
    if (selectedCamera < 0)
    {
        fprintf(stderr,"### ERROR ### Did not find camera with GUID: 0x%16.16llX\n", camUID);
        return;
    } else {
        printf("Found requested camera, initializing ...\n ");
        err = this->pCamera->SelectCamera(selectedCamera);
        status = PERR(this->pasynUserSelf, err);
        err = this->pCamera->InitCamera(TRUE);
        status = PERR(this->pasynUserSelf, err);
    }
    this->pCameraControlSize = this->pCamera->GetCameraControlSize();
    this->pCameraControl = (C1394CameraControl **)malloc(num1394Features * sizeof(pCameraControl[0]));
    for (i=0; i<num1394Features; i++) {
        this->pCameraControl[i] = new C1394CameraControl(this->pCamera, featureIndex[i]);
    }

    /* Determine the maximum image size */
    this->pCameraControlSize->GetSizeLimits(&maxSizeX, &maxSizeY);
    
    /* Create the start and stop event that will be used to signal our
     * image grabbing thread when to start/stop     */
    printf("Creating EPICS events...                 ");
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    this->stopEventId = epicsEventCreate(epicsEventFull);
    printf("OK\n");

    /* Set the parameters from the camera in our areaDetector param lib */
    printf("Setting the areaDetector parameters...  ");
    status =  setStringParam (ADManufacturer, vendorName);
    status |= setStringParam (ADModel, cameraName);
    status |= setIntegerParam(ADMaxSizeX, maxSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxSizeY);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeY, maxSizeY);
    status |= setIntegerParam(ADImageSizeX, maxSizeX);
    status |= setIntegerParam(ADImageSizeY, maxSizeY);

    /* We only currently support 8 bit mono and 3x 8 bit RGB...
     * TODO: figure out how to enable support for:
     *       a) 12 bit mono(?)   */
    status |= setIntegerParam(ADImageSize, maxSizeX*maxSizeY*sizeof(char));
    status |= setIntegerParam(ADDataType, NDUInt8);

    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setIntegerParam(ADNumImages, 100);
    status |= this->formatValidModes();
    status |= this->getAllFeatures();
    if (status)
    {
         fprintf(stderr, "ERROR %s: unable to set camera parameters\n", functionName);
         return;
    } else printf("OK\n");

    /* Start up acquisition thread */
    printf("Starting up image grabbing task...     ");
    status = (epicsThreadCreate("imageGrabTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC)imageGrabTaskC,
            this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image task\n",
                driverName, functionName);
        return;
    } else printf("OK\n");
    printf("Configuration complete!\n");
    return;
}


/** Task to grab images off the camera and send them up to areaDetector
 *
 */
void FirewireWinDCAM::imageGrabTask()
{
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire, addr;
    int externalStopCmd = 0;
    const char *functionName = "imageGrabTask";

    printf("FirewireWinDCAM::imageGrabTask: Got the image grabbing thread started!\n");

    epicsEventTryWait(this->stopEventId); /* clear the stop event if it wasn't already */

    epicsMutexLock(this->mutexId);

    while (1) /* ... round and round and round we go ... */
    {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire)
        {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            if (externalStopCmd)
            {
                /* Signal someone that the thread is really stopping to wait for start event */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                                    "%s::%s [%s]: Signalling stop event\n", driverName, functionName, this->portName);
                epicsEventSignal(this->stopEventId);
            } else externalStopCmd = 1;

            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            epicsMutexUnlock(this->mutexId);


            /* Wait for a signal that tells this thread that the transmission
             * has started and we can start asking for image buffers...     */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s [%s]: waiting for acquire to start\n", driverName, functionName, this->portName);
            status = epicsEventWait(this->startEventId);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s [%s]: started!\n", driverName, functionName, this->portName);
            epicsMutexLock(this->mutexId);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
        }

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);
        /* We are now waiting for an image  */
        setIntegerParam(ADStatus, ADStatusWaiting);
        /* Call the callbacks to update any changes */
        callParamCallbacks();

        status = this->grabImage();        /* #### GET THE IMAGE FROM CAMERA HERE! ##### */
        if (status == asynError)         /* check for error */
        {
            /* remember to release the NDArray back to the pool now
             * that we are not using it (we didn't get an image...) */
            if(this->pRaw) this->pRaw->release();
            /* We abort if we had some problem with grabbing an image...
             * This is perhaps not always the desired behaviour but it'll do for now. */
            setIntegerParam(ADStatus, ADStatusAborting);
            this->stopCapture(this->pasynUserSelf);
            continue;
        }

        /* Set a bit of image/frame statistics... */
        getIntegerParam(ADImageCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(ADArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(ADImageCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);
        /* Put the frame number into the buffer */
        this->pRaw->uniqueId = imageCounter;
        /* Set a timestamp in the buffer */
        this->pRaw->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

        /* Call the callbacks to update any changes */
        callParamCallbacks();

        /* Get some information about the current feature settings */
        //this->getAllFeatures();
        /* issue the callbacks for all the updated feature values */
        for (addr=0; addr < this->maxAddr; addr++) callParamCallbacks(addr, addr);

        if (arrayCallbacks)
        {
            /* Call the NDArray callback */
            /* Must release the lock here, or we can get into a deadlock, because we can
             * block on the plugin lock, and the plugin can be calling us */
            epicsMutexUnlock(this->mutexId);
            doCallbacksGenericPointer(this->pRaw, NDArrayData, 0);
            epicsMutexLock(this->mutexId);
        }
        /* Release the NDArray buffer now that we are done with it.
         * After the callback just above we don't need it anymore */
        this->pRaw->release();
        this->pRaw = NULL;

        /* See if acquisition is done if we are in single or multiple mode */
        if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages)))
        {
            /* command the camera to stop acquiring.. */
            setIntegerParam(ADAcquire, 0);
            externalStopCmd = 0;
            this->stopCapture(this->pasynUserSelf);
            if (status == asynError)
            {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s [%s] Stopping transmission failed...\n",
                            driverName, functionName, this->portName);
            }
        }
    }/* back to the top... */
    return;
}

/** Grabs one image off the dc1394 queue, notifies areaDetector about it and
 * finally clears the buffer off the dc1394 queue.
 * This function expects the mutex to be locked already by the caller!
 */
int FirewireWinDCAM::grabImage()
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int dims[3];
    int err;
    unsigned long sizeX, sizeY;
    unsigned long dataLength;
    int bytesPerColor;
    int numColors;
    int ndims;
    int droppedFrames;
    NDColorMode_t colorMode;
    unsigned char * pTmpData;
    const char* functionName = "grabImage";

    /* unlock the mutex while we wait for a new image to be ready */
    epicsMutexUnlock(this->mutexId);
    err = this->pCamera->AcquireImageEx(TRUE, &droppedFrames);
    status = PERR( this->pasynUserSelf, err );
    epicsMutexLock(this->mutexId);
    if (status) return status;   /* if we didn't get an image properly... */

    /* Get the size of the frame */
    //this->pCameraControlSize->GetSize(&sizeX, &sizeY);
    this->pCamera->GetVideoFrameDimensions(&sizeX, &sizeY);
    setIntegerParam(ADImageSizeX, sizeX);
    setIntegerParam(ADImageSizeY, sizeY);
    
    /* Get the data type */
    getIntegerParam(ADDataType, (int *)&dataType);
    switch (dataType) {
        case NDInt8:
        case NDUInt8:
            bytesPerColor = 1;
            break;
        case NDInt16:
        case NDUInt16:
            bytesPerColor = 2;
            break;
        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "Unsupported data type=%d\n", dataType);
            return(asynError);
    }
     /* Get the requested color mode */
    getIntegerParam(ADColorMode, (int *)&colorMode);
    switch (colorMode) {
        case NDColorModeMono:
        case NDColorModeBayer:
            numColors = 1;
            break;
        case NDColorModeRGB3:
            numColors = 3;
            break;
        default:
            break;
    }
    dims[0] = sizeX;
    dims[1] = sizeY;
    if (numColors == 1) {
        ndims = 2;
    } else {
        ndims = 3;
        dims[2] = 3;
    }
   
    this->pRaw = this->pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);
    if (!this->pRaw) {
        /* If we didn't get a valid buffer from the NDArrayPool we must abort
         * the acquisition as we have nowhere to dump the data...       */
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
                    driverName, functionName, this->portName);
        this->stopCapture(this->pasynUserSelf);
        return(asynError);
    }


    /* tell our driver where to find the image buffer with this latest image */
    switch (colorMode) {
        case NDColorModeMono:
        case NDColorModeBayer:
            pTmpData = this->pCamera->GetRawData(&dataLength);
            if ((int)dataLength > this->pRaw->dataSize) dataLength = this->pRaw->dataSize;
            memcpy((unsigned char*)this->pRaw->pData, pTmpData, dataLength);
            break;
        case NDColorModeRGB3:
            this->pCamera->getRGB((unsigned char*)this->pRaw->pData, this->pRaw->dataSize);
            break;
        default:
            break;
    }
    
    /* Change the status to be readout... */
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    this->pRaw->colorMode = colorMode;
    this->pRaw->bayerPattern = NDBayerRGGB;

    return (status);
}


/** Write integer value to the drivers parameter table. */
asynStatus FirewireWinDCAM::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int adstatus;
    int addr, rbValue, tmpVal;

    pasynManager->getAddr(pasynUser, &addr);
    if (addr < 0) addr=0;

    rbValue = value;

    switch(function)
    {
    case ADAcquire:
        getIntegerParam(ADStatus, &adstatus);
        if (value && (adstatus == ADStatusIdle))
        {
            /* start acquisition */
            status = this->startCapture(pasynUser);
        } else if (!value)
        {
            status = this->stopCapture(pasynUser);
        }
        break;

    case ADSizeX:
    case ADSizeY:
    case ADMinX:
    case ADMinY:
        status = this->setFormat7Params(pasynUser);
        break;
    case FDC_feat_val:
        /* First check if the camera is set for manual control... */
        getIntegerParam(addr, FDC_feat_mode, &tmpVal);
        /* if it is not set to 'manual' (0) then we do set it to manual */
        if (tmpVal != 0) status = this->setFeatureMode(pasynUser, 0, NULL);
        if (status == asynError) break;

        /* now send the feature value to the camera */
        status = this->setFeatureValue(pasynUser, value, &rbValue);
        if (status == asynError) break;

        /* update all feature values to check if any settings have changed */
        status = (asynStatus) this->getAllFeatures();
        break;

    case FDC_feat_mode:
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_feat_mode: setting value: %d\n", value);
        status = this->setFeatureMode(pasynUser, value, &rbValue);
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_feat_mode: readback value: %d\n", rbValue);
        break;

    case FDC_format:
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_format: setting value: %d\n", value);
        status = this->setVideoFormat(pasynUser, value);
        break;

    case FDC_mode:
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_mode: setting value: %d\n", value);
        status = this->setVideoMode(pasynUser, value);
        break;

    case FDC_framerate:
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_framerate: setting value: %d\n", value);
        status = this->setFrameRate(pasynUser, value);
        break;

    }

    if (status != asynError) status = setIntegerParam(addr, function, rbValue);
    /* Call the callback for the specific address .. and address ... weird? */
    if (status != asynError) callParamCallbacks(addr, addr);
    return status;
}

/** Write floating point value to the drivers parameter table and possibly to the hardware.
 * \param pasynUser
 * \param value
 * \return asynStatus Either asynError or asynSuccess
 */
asynStatus FirewireWinDCAM::writeFloat64( asynUser *pasynUser, epicsFloat64 value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    epicsFloat64 rbValue;
    int addr, tmpVal;
    
    pasynManager->getAddr(pasynUser, &addr);
    if (addr < 0) addr=0;

    switch(function)
    {
    case FDC_feat_val_abs:
        /* First check if the camera is set for manual control... */
        getIntegerParam(addr, FDC_feat_mode, &tmpVal);
        /* if it is not set to 'manual' (0) then we do set it to manual */
        if (tmpVal != 0) status = this->setFeatureMode(pasynUser, 0, NULL);
        if (status == asynError) break;

        status = this->setFeatureAbsValue(pasynUser, value, &rbValue);
        if (status == asynError) break;
        /* update all feature values to check if any settings have changed */
        status = (asynStatus) this->getAllFeatures();
        break;
    }

    if (status != asynError) status = setDoubleParam(addr, function, rbValue);
    if (status != asynError) callParamCallbacks(addr, addr);
    return status;
}

/** Check if a requested feature is valid
 *
 * Checks for:
 * 
 * Valid range of the asyn request address against feature index.
 * Availability of the requested feature in the current camera.
 * 
 * pasynUser
 * featInfo     Pointer to a feature info structure pointer. The function
 *              will write a valid feature info struct into this pointer or NULL on error.
 * featureName  The function will return a string in this parameter with a
 *              readable name for the given feature.
 * functionName The caller can pass a string which contain the callers function
 *              name. For debugging/printing purposes only.
 * asyn status
 */
C1394CameraControl* FirewireWinDCAM::checkFeature(asynUser *pasynUser, char **featureName)
{
	asynStatus status = asynSuccess;
	int addr;
	const char* functionName = "checkFeature";
    C1394CameraControl *pFeature=NULL;

	pasynManager->getAddr(pasynUser, &addr);

	if (addr < 0 || addr >= num1394Features)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR addr: %d is out of range [0..%d]\n",
					driverName, functionName, addr, num1394Features);
		return NULL;
	}

    /* Read the information from the camera */
    pFeature = this->pCameraControl[addr];
	pFeature->Inquire();

	/* Get a readable name for the feature we are working on */
	if (*featureName != NULL)
	{
		*featureName = (char *)pFeature->GetName();
	}

	/* check if the feature we are working on is available in this camera */
	if (!pFeature->HasPresence())
	{
		return NULL;
	}

	return pFeature;
}

asynStatus FirewireWinDCAM::setFeatureMode(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue)
{
	asynStatus status = asynSuccess;
	const char *functionName = "setFeatureMode";
	char *featureName;
    int err;
    C1394CameraControl *pFeature;
    
	/* First check if the feature is valid for this camera */
	pFeature = this->checkFeature(pasynUser, &featureName);
	if (!pFeature) return asynError;

	/* Check if the desired mode is even supported by the camera on this feature */
    if (value == 0) {
        if (!pFeature->HasManualMode()) return asynError;
    } else {
        if (!pFeature->HasAutoMode()) return asynError;
    }

	/* Send the feature mode to the cam */
	err = pFeature->SetAutoMode(value);
	status = PERR(pasynUser, err);
	if (status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue == NULL) return status;

	/* Finally read back the current value from the cam and set the readback */
	*rbValue = pFeature->StatusAutoMode();
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s value=%d rbValue=%d\n",
				driverName, functionName, value, *rbValue);
	return status;
}


asynStatus FirewireWinDCAM::setFeatureValue(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue)
{
	asynStatus status = asynSuccess;
    int err;
	unsigned short min, max, lo, hi;
	const char *functionName = "setFeatureValue";
    C1394CameraControl *pFeature;
	char *featureName;

	/* First check if the feature is valid for this camera */
	pFeature = this->checkFeature(pasynUser, &featureName);
	if (!pFeature) return status;

	/* Check the value is within the expected boundaries */
	pFeature->GetRange(&min, &max);
	if (value < (epicsInt32)min || value > (epicsInt32)max)
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s ERROR [%s] setting feature %s, value %d is out of range [%d..%d]\n",
					driverName, functionName, this->portName, featureName, value, min, max);
		return asynError;
	}

    /* Disable absolute mode control for this feature */
    err = pFeature->SetAbsControl(FALSE);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;

	/* Set the feature value in the camera */
    lo = value & 0xFFF;
    hi = (value >> 12) & 0xFFF;
	err = pFeature->SetValue(lo, hi);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue != NULL)
	{
		/* Finally read back the value from the camera and set that as the new value */
        pFeature->Inquire();
		pFeature->GetValue(&lo, &hi);
        *rbValue = lo + (hi << 12);
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %d readback value from cam: %d\n",
				driverName, functionName, value, *rbValue);
	} else
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %d\n",
				driverName, functionName, value);
	}
	return status;
}


asynStatus FirewireWinDCAM::setFeatureAbsValue(asynUser *pasynUser, epicsFloat64 value, epicsFloat64 *rbValue)
{
	asynStatus status = asynSuccess;
    int err;
	float min, max, fvalue;
	const char *functionName = "setFeatureAbsValue";
    C1394CameraControl *pFeature;
	char *featureName;

	/* First check if the feature is valid for this camera */
	pFeature = this->checkFeature(pasynUser, &featureName);
	if (!pFeature) return status;

	/* Check if the specific feature supports absolute values */
	if (!pFeature->HasAbsControl()) { 
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s] setting feature \'%s\': No absolute control for this feature\n",
					driverName, functionName, this->portName, featureName);
		return asynError;
	}

	/* Check the value is within the expected boundaries */
	pFeature->GetRangeAbsolute(&min, &max);
	if (value < min || value > max)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s] setting feature %s, value %.5f is out of range [%.3f..%.3f]\n",
					driverName, functionName, this->portName, featureName, value, min, max);
		return asynError;
	}

    /* Enable absolute mode control for this feature */
    err = pFeature->SetAbsControl(TRUE);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;

	/* Finally set the feature value in the camera */
	err = pFeature->SetValueAbsolute((float)value);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue != NULL)
	{
		pFeature->GetValueAbsolute(&fvalue);
        *rbValue = fvalue;
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %.3f readback value from cam: %.3f\n",
				driverName, functionName, value, *rbValue);

	} else
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %.3f\n",
				driverName, functionName, value);
	}

	return status;
}

/** Set the framerate in the camera. */
asynStatus FirewireWinDCAM::setVideoFormat( asynUser *pasynUser, epicsInt32 format)
{
	asynStatus status = asynSuccess;
	int err;
	int wasAcquiring;
	const char* functionName = "setVideoFormat";

	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCapture(pasynUser);
		if (status == asynError) goto done;
	}

    if (!this->pCamera->HasVideoFormat(format)) {
 		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s]: camera does not support format %d\n",
					driverName, functionName, this->portName, format);
		goto done;
	}

	/* attempt to write the format to camera */
	asynPrint( 	pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]: setting video format:%d\n",
				driverName, functionName, this->portName,format);
	err = this->pCamera->SetVideoFormat(format);
	status = PERR( pasynUser, err );
    if (status == asynError) goto done;
    
    /* If the new format is format 7 then set the parameters */
    if (format == 7) this->setFormat7Params(pasynUser);

    /* When the format changes the supported values of video mode and frame rate change */
    this->formatValidModes();
    /* When the format changes the available features can also change */
    this->getAllFeatures();

    done:
    if (wasAcquiring) this->startCapture(pasynUser);
	return status;
}

asynStatus FirewireWinDCAM::setVideoMode( asynUser *pasynUser, epicsInt32 mode)
{
	asynStatus status = asynSuccess;
	int err;
	int wasAcquiring;
    int format;
	const char* functionName = "setVideoMode";

	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCapture(pasynUser);
		if (status == asynError) goto done;
	}

    /* Get the current video format */
    format = this->pCamera->GetVideoFormat();
    if (!this->pCamera->HasVideoMode(format, mode)) {
 		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s]: camera does not support mode %d\n",
					driverName, functionName, this->portName, mode);
		status = asynError;
        goto done;
	}

	/* attempt to write the mode to camera */
	asynPrint( 	pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]: setting video mode:%d\n",
				driverName, functionName, this->portName, mode);
	err = this->pCamera->SetVideoMode(mode);
	status = PERR( pasynUser, err );
    if (status == asynError) goto done;

    done:
    /* When the mode changes the supported values of frame rate change */
    this->formatValidModes();
    /* When the mode changes the available features can also change */
    this->getAllFeatures();

    if (wasAcquiring) this->startCapture(pasynUser);
	return status;
}

asynStatus FirewireWinDCAM::setFrameRate( asynUser *pasynUser, epicsInt32 rate)
{
	asynStatus status = asynSuccess;
	unsigned int newframerate = 0;
	int err;
	int wasAcquiring;
    int format, mode;
	const char* functionName = "setFrameRate";

	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCapture(pasynUser);
		if (status == asynError) goto done;
	}

    /* Get the current video format and mode */
    format = this->pCamera->GetVideoFormat();
    mode = this->pCamera->GetVideoMode();
    if (!this->pCamera->HasVideoFrameRate(format, mode, rate)) {
 		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s]: camera does not support framerate %d\n",
					driverName, functionName, this->portName, rate);
		goto done;
	}

	/* attempt to write the framerate to camera */
	asynPrint( 	pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]: setting framerate:%d\n",
				driverName, functionName, this->portName, rate);
	err = this->pCamera->SetVideoFrameRate(rate);
	status = PERR( pasynUser, err );
    if (status == asynError) goto done;

    done:
    /* When the mode changes the supported values of frame rate change */
    this->formatValidModes();
    /* When the mode changes the available features can also change */
    this->getAllFeatures();

    if (wasAcquiring) this->startCapture(pasynUser);
	return status;
}

asynStatus FirewireWinDCAM::setFormat7Params( asynUser *pasynUser)
{
	asynStatus status = asynSuccess;
	int err;
	int wasAcquiring;
    int format;
    int sizeX, sizeY, minX, minY;
    unsigned short width, height, left, top;
	const char* functionName = "setFormat7Params";

	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCapture(pasynUser);
		if (status == asynError) goto done;
	}

    /* Get the current video format */
    format = this->pCamera->GetVideoFormat();
    /* If not format 7 then silently exit */
    if (format != 7) goto done;
    
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
	/* attempt to write the parameters to camera */
	asynPrint( 	pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]: setting format 7 parameters sizeX=%d, sizeY=%d, minX=%d, minY=%d\n",
				driverName, functionName, this->portName, sizeX, sizeY, minX, minY);
	err = this->pCameraControlSize->SetPos(minX, minY);
  	status = PERR( pasynUser, err );
    if (status == asynError) goto done;
	err = this->pCameraControlSize->SetSize(sizeX, sizeY);
  	status = PERR( pasynUser, err );
    if (status == asynError) goto done;

    done:
    /* Read back the actual values */
	this->pCameraControlSize->GetPos(&left, &top);
    setIntegerParam(ADMinX, left);
    setIntegerParam(ADMinY, top);
	this->pCameraControlSize->GetSize(&width, &height);
    setIntegerParam(ADSizeX, width);
    setIntegerParam(ADSizeY, height);
    callParamCallbacks();

    if (wasAcquiring) this->startCapture(pasynUser);
	return status;
}



asynStatus FirewireWinDCAM::formatValidModes()
{
    int format, mode, rate;
    char str[40];
 
    /* This function writes strings for all valid video formats,
     * the valid video modes for the current video format, and the
     * valid frame rates for the current video format and video mode. */

    /* Format all valid video formats */
    for (format=0; format<=MAX_1394_VIDEO_FORMATS; format++) {
        if (this->pCamera->HasVideoFormat(format)) {
            sprintf(str, "%d %s", format, videoFormatStrings[format]);
            setIntegerParam(format, FCD_has_format, 1);
        } else {
            sprintf(str, "%d N.A.", format);
            setIntegerParam(format, FCD_has_format, 0);
        }
        setStringParam(format, FDC_valid_format, str);
    }

    /* Get the current video format */
    format = this->pCamera->GetVideoFormat();
    setIntegerParam(FDC_format, format);
    sprintf(str, "%s", videoFormatStrings[format]);
    setStringParam(FDC_current_format, str);

    /* Format all valid video modes for this format */
    for (mode=0; mode<=MAX_1394_VIDEO_MODES; mode++) {
        if (this->pCamera->HasVideoMode(format, mode)) {
            sprintf(str, "%d %s", mode, videoModeStrings[format][mode]);
            setIntegerParam(mode, FCD_has_mode, 1);
        } else {
            sprintf(str, "%d N.A.", mode);
            setIntegerParam(mode, FCD_has_mode, 0);
        }
        setStringParam(mode, FDC_valid_mode, str);
    }

    /* Get the current video mode */
    mode = this->pCamera->GetVideoMode();
    setIntegerParam(FDC_mode, mode);
    sprintf(str, "%s", videoModeStrings[format][mode]);
    setStringParam(FDC_current_mode, str);

    /* Format all valid video rates for this format and mode */
    for (rate=0; rate<=MAX_1394_FRAME_RATES; rate++) {
        if (this->pCamera->HasVideoFrameRate(format, mode, rate)) {
            sprintf(str, "%d %s", rate, frameRateStrings[rate]);
            setIntegerParam(rate, FCD_has_framerate, 1);
        } else {
            sprintf(str, "%d N.A.", rate);
            setIntegerParam(rate, FCD_has_framerate, 0);
        }
        setStringParam(rate, FDC_valid_framerate, str);
    }

    /* Get the current video rate */
    rate = this->pCamera->GetVideoFrameRate();
    setIntegerParam(FDC_framerate, rate);
    sprintf(str, "%s", frameRateStrings[rate]);
    setStringParam(FDC_current_framerate, str);
    
    /* This assumes that the number of formats, modes and rates are all the same */
    for (format=0; format<MAX_1394_VIDEO_FORMATS; format++) callParamCallbacks(format, format);

    return asynSuccess;
}

/** Read all the feature settings and values from the camera.
 * This function will collect all the current values and settings from the camera,
 * and set the appropriate integer/double parameters in the param lib. If a certain feature
 * is not available in the given camera, this function will set all the parameters relating to that
 * feature to -1 or -1.0 to indicate it is not available.
 * Note the caller is responsible for calling any update callbacks if I/O interrupts
 * are to be processed after calling this function.
 * Returns asynStatus asynError or asynSuccess as an int.
 */
int FirewireWinDCAM::getAllFeatures()
{
	int status = (int)asynSuccess;
	C1394CameraControl *pFeature;
	int tmp, addr, value;
    unsigned short min, max, lo, hi;
    float fmin, fmax, fvalue;
	double dtmp;
    const char* functionName="getAllFeatures";

	/* Iterate through all of the available features and update their values and settings  */
	for (addr = 0; addr < num1394Features; addr++) {

        pFeature = this->pCameraControl[addr];
	    pFeature->Inquire();

		/* If the feature is not available in the camera, we just set
		 * all the parameters to -1 to indicate this is not available to the user. */
		if (!pFeature->HasPresence()) {
			status |= setIntegerParam(addr, FDC_feat_available, 0);
			tmp = -1;
			dtmp = -1.0;
			setIntegerParam(addr, FDC_feat_val, tmp);
			setIntegerParam(addr, FDC_feat_val_min, tmp);
			setIntegerParam(addr, FDC_feat_val_max, tmp);
			//setIntegerParam(addr, FDC_feat_mode, tmp);
			setDoubleParam(addr, FDC_feat_val_abs, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_max, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_min, dtmp);
			continue;
		}
	    pFeature->GetRange(&min, &max);
		pFeature->GetValue(&lo, &hi);
        value = lo + (hi << 12);
		status |= setIntegerParam(addr, FDC_feat_available, 1);
		status |= setIntegerParam(addr, FDC_feat_val, value);
		status |= setIntegerParam(addr, FDC_feat_val_min, min);
		status |= setIntegerParam(addr, FDC_feat_val_max, max);

	    tmp = pFeature->StatusAutoMode();
		status |= setIntegerParam(addr, FDC_feat_mode, tmp);

		/* If the feature does not support 'absolute' control then we just
		 * set all the absolute values to -1.0 to indicate it is not available to the user */
	    if (!pFeature->HasAbsControl()) { 
			dtmp = -1.0;
			status |= setIntegerParam(addr, FDC_feat_absolute, 0);
			setDoubleParam(addr, FDC_feat_val_abs, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_max, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_min, dtmp);
			continue;
		}
	    pFeature->GetRangeAbsolute(&fmin, &fmax);
		pFeature->GetValueAbsolute(&fvalue);
		status |= setIntegerParam(addr, FDC_feat_absolute, 1);
		status |= setDoubleParam(addr, FDC_feat_val_abs, fvalue);
		status |= setDoubleParam(addr, FDC_feat_val_abs_max, fmax);
		status |= setDoubleParam(addr, FDC_feat_val_abs_min, fmin);
	}

	/* Finally map a few of the AreaDetector parameters on to the camera 'features' */
    for (addr=0; addr<num1394Features; addr++) {
        if (featureIndex[addr] == FEATURE_SHUTTER) break;
    }
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	status |= setDoubleParam(ADAcquireTime, dtmp);
	getDoubleParam(addr, FDC_feat_val_abs_max, &dtmp);
	status |= setDoubleParam(ADAcquirePeriod, dtmp);

    for (addr=0; addr<num1394Features; addr++) {
        if (featureIndex[addr] == FEATURE_GAIN) break;
    }
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	status |= setDoubleParam(ADGain, dtmp);

	return status;
}



asynStatus FirewireWinDCAM::startCapture(asynUser *pasynUser)
{
    asynStatus status = asynSuccess;
    int err;
    int acquiring;
    double timeout;
    const char* functionName = "startCapture";

    /* return error if already acquiring */
    getIntegerParam(ADAcquire, &acquiring);
    if (acquiring)
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Camera already acquiring.\n",
                    driverName, functionName, this->portName);
        return asynError;
    }

    getDoubleParam(ADAcquireTime, &timeout);
    if (timeout < 1.0) timeout = 1.0;
    
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] Starting firewire transmission\n",
                driverName, functionName, this->portName);
    /* Start the camera transmission... */
    err = this->pCamera->StartImageAcquisitionEx(MAX_1394_BUFFERS, (int)(1000*timeout), ACQ_START_VIDEO_STREAM);
    status = PERR( this->pasynUserSelf, err );
    if (status == asynError)
    {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] starting transmission failed... Staying in idle state.\n",
                    driverName, functionName, this->portName);
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
        return status;
    }

    /* Signal the image grabbing thread that the acquisition/transmission has
     * started and it can start dequeueing images from the driver buffer */
    epicsEventSignal(this->startEventId);
    return status;
}


asynStatus FirewireWinDCAM::stopCapture(asynUser *pasynUser)
{
    asynStatus status = asynSuccess;
    int err;
    epicsEventWaitStatus eventStatus;
    int acquiring;
    const char * functionName = "stopCapture";

    /* return error if already stopped */
    getIntegerParam(ADAcquire, &acquiring);
    if (!acquiring)
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Camera already stopped.\n",
                    driverName, functionName, this->portName);
        return asynError;
    }

    setIntegerParam(ADAcquire, 0);        /* set acquiring state to stopped */
    callParamCallbacks();                 /* update whoever is interested in the acquire-state */

    /* Now wait for the capture thread to actually stop */
    asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] waiting for stopped event...\n",
                    driverName, functionName, this->portName);
    /* unlock the mutex while we're waiting for the capture thread to stop acquiring */
    epicsMutexUnlock(this->mutexId);
    eventStatus = epicsEventWaitWithTimeout(this->stopEventId, 3.0);
    epicsMutexLock(this->mutexId);
    if (eventStatus != epicsEventWaitOK)
    {
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] ERROR: Timeout when trying to stop image grabbing thread.\n",
                    driverName, functionName, this->portName);
    }

    asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] Stopping firewire transmission\n",
                driverName, functionName, this->portName);

    /* Stop the actual transmission! */
    err=this->pCamera->StopImageAcquisition();
    status = PERR( pasynUser, err );
    if (status == asynError)
    {
        /* if stopping transmission results in an error (weird situation!) we print a message
         * but does not abort the function because we still want to set status to stopped...  */
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Stopping transmission failed...\n",
                    driverName, functionName, this->portName);
    }

    return status;
}


/** Parse a dc1394 error code into a user readable string
 * Defaults to printing out using the pasynUser.
 * \param asynUser The asyn user to print out with on ASYN_TRACE_ERR. If pasynUser == NULL just print to stderr.
 * \param dc1394_err The error code, returned from the dc1394 function call. If the error code is OK we just ignore it.
 * \param errOriginLine Line number where the error came from.
 */
asynStatus FirewireWinDCAM::err( asynUser* asynUser, int CAM_err, int errOriginLine)
{
    if (CAM_err == CAM_SUCCESS) return asynSuccess; /* if everything is OK we just ignore it */

    if (this->pasynUserSelf == NULL) fprintf(stderr, "### ERROR port=%s line=%d, error=%d (%s)\n", 
        this->portName, errOriginLine, CAM_err, errMsg[-CAM_err]);
    else asynPrint( this->pasynUserSelf, ASYN_TRACE_ERROR, "### ERROR port=%s line=%d, error=%d (%s)\n", 
        this->portName, errOriginLine, CAM_err, errMsg[-CAM_err]);
    return asynError;
}

/** Create an asyn user for the driver.
 * Maps the integer/enum asyn commands on to a string representation that
 * can be used to indicate a certain command in in the INP/OUT field of a record.
 * \param pasynUser
 * \param drvInfo
 * \param pptypeName
 * \param psize
 * \return asynStatus Either asynError or asynSuccess
 */
asynStatus FirewireWinDCAM::drvUserCreate( asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
    asynStatus status;
    int param;
    const char *functionName = "drvUserCreate";

    /* See if this is one of the drivers local parameters */
    status = findParam(FDCParamString, FDC_N_PARAMS, drvInfo, &param);

    if (status == asynSuccess)
    {
        pasynUser->reason = param;
        if (pptypeName) { *pptypeName = epicsStrDup(drvInfo); }
        if (psize) { *psize = sizeof(param); }
        asynPrint(    pasynUser, ASYN_TRACE_FLOW, "%s:%s: drvInfo=%s, param=%d\n",
                    driverName, functionName, drvInfo, param);
        return asynSuccess;
    }

    /* If not a local driver parameter, then see if it is a base class parameter */
    status = ADDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
    return status;
}

/** Print out a report.
 */
void FirewireWinDCAM::report(FILE *fp, int details)
{
    char vendorName[256], cameraName[256];
    LARGE_INTEGER uniqueId;
    int version;
    int format, mode, rate;
    unsigned long sizeX, sizeY;
    unsigned short maxSizeX, maxSizeY;
    
    this->pCamera->GetCameraVendor(vendorName, sizeof(vendorName));
    this->pCamera->GetCameraName(cameraName, sizeof(cameraName));
    this->pCamera->GetCameraUniqueID(&uniqueId);
    this->pCameraControlSize->GetSizeLimits(&maxSizeX, &maxSizeY);
    version = this->pCamera->GetVersion();
    fprintf(fp, "Vendor name: %s\n", vendorName);
    fprintf(fp, "Camera name: %s\n", cameraName);
    fprintf(fp, "UniqueId: %lld (0x%llX)\n", uniqueId.QuadPart, uniqueId.QuadPart);
    fprintf(fp, "Version: 0x%lX\n", version);
    fprintf(fp, "Max size: X=%d, Y=%d\n", maxSizeX, maxSizeY);
    if (details > 1) {
        fprintf(fp, "Supported formats, modes and rates:\n");
        for (format=0; format<=7; format++) {
            if (this->pCamera->HasVideoFormat(format)) {
                fprintf(fp, "Format %d supported (%s)\n", format, videoFormatStrings[format]);
                for (mode=0; mode<=7; mode++) {
                    if (this->pCamera->HasVideoMode(format, mode)) {
                        fprintf(fp, "  Mode %d supported (%s)\n", mode, videoModeStrings[format][mode]);
                        for (rate=0; rate<=7; rate++) {
                            if (this->pCamera->HasVideoFrameRate(format, mode, rate)) {
                                fprintf(fp, "    Rate %d supported (%s)\n", rate, frameRateStrings[rate]);
                            }
                        }
                    }
                }
            }
        }
        fprintf(fp, "Has one-shot: %s\n", this->pCamera->HasOneShot() ? "Yes":"No");
        fprintf(fp, "Has multi-shot: %s\n", this->pCamera->HasMultiShot() ? "Yes":"No");
        format = this->pCamera->GetVideoFormat();
        mode = this->pCamera->GetVideoMode();
        rate = this->pCamera->GetVideoFrameRate();
        this->pCamera->GetVideoFrameDimensions(&sizeX, &sizeY);
        fprintf(fp, "Current settings\n");
        fprintf(fp, "  Format: %d (%s)\n", format, videoFormatStrings[format]);
        fprintf(fp, "  Mode: %d (%s)\n", mode, videoModeStrings[format][mode]);
        fprintf(fp, "  Rate: %d (%s)\n", rate, frameRateStrings[rate]);
        fprintf(fp, "  Size: %d %d\n", sizeX, sizeY);
    }
                    
    ADDriver::report(fp, details);
    return;
}

