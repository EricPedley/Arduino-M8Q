#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

SFE_UBLOX_GNSS::SFE_UBLOX_GNSS(void)
{
  // Constructor
  if (debugPin >= 0)
  {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0;                             // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA messages to processNMEA

  // Support for platforms like ESP32 which do not support multiple I2C restarts
  // If _i2cStopRestart is true, endTransmission will always use a stop. If false, a restart will be used where needed.
#if defined(ARDUINO_ARCH_ESP32)
  _i2cStopRestart = true; // Always use a stop
#else
  _i2cStopRestart = false; // Use a restart where needed
#endif
}

SFE_UBLOX_GNSS::~SFE_UBLOX_GNSS(void)
{
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and spiBuffer

  if (payloadCfg != NULL)
  {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = NULL;   // Redundant?
  }

  if (payloadAuto != NULL)
  {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = NULL;   // Redundant?
  }

  if (spiBuffer != NULL)
  {
    delete[] spiBuffer; // Created with new[]
    spiBuffer = NULL;   // Redundant?
  }
}

// Stop all automatic message processing. Free all used RAM
void SFE_UBLOX_GNSS::end(void)
{
  // Note: payloadCfg is not deleted

  // Note: payloadAuto is not deleted

  // Note: spiBuffer is not deleted

  if (ubxFileBuffer != NULL) // Check if RAM has been allocated for the file buffer
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("end: the file buffer has been deleted. You will need to call setFileBufferSize before .begin to create a new one."));
    }
#endif
    delete[] ubxFileBuffer; // Created with new[]
    ubxFileBuffer = NULL;   // Redundant?
    fileBufferSize = 0;     // Reset file buffer size. User will have to call setFileBufferSize again
    fileBufferMaxAvail = 0;
  }

  if (moduleSWVersion != NULL)
  {
    delete moduleSWVersion; // Created with new moduleSWVersion_t
    moduleSWVersion = NULL; // Redundant?
  }

  if (currentGeofenceParams != NULL)
  {
    delete currentGeofenceParams; // Created with new geofenceParams_t
    currentGeofenceParams = NULL; // Redundant?
  }

  if (packetUBXNAVTIMELS != NULL)
  {
    delete packetUBXNAVTIMELS; // Created with new UBX_NAV_TIMELS_t
    packetUBXNAVTIMELS = NULL; // Redundant?
  }

  if (packetUBXNAVPOSECEF != NULL)
  {
    if (packetUBXNAVPOSECEF->callbackData != NULL)
    {
      delete packetUBXNAVPOSECEF->callbackData; // Created with new UBX_NAV_POSECEF_data_t
    }
    delete packetUBXNAVPOSECEF; // Created with new UBX_NAV_POSECEF_t
    packetUBXNAVPOSECEF = NULL; // Redundant?
  }

  if (packetUBXNAVSTATUS != NULL)
  {
    if (packetUBXNAVSTATUS->callbackData != NULL)
    {
      delete packetUBXNAVSTATUS->callbackData;
    }
    delete packetUBXNAVSTATUS;
    packetUBXNAVSTATUS = NULL; // Redundant?
  }

  if (packetUBXNAVDOP != NULL)
  {
    if (packetUBXNAVDOP->callbackData != NULL)
    {
      delete packetUBXNAVDOP->callbackData;
    }
    delete packetUBXNAVDOP;
    packetUBXNAVDOP = NULL; // Redundant?
  }

  if (packetUBXNAVATT != NULL)
  {
    if (packetUBXNAVATT->callbackData != NULL)
    {
      delete packetUBXNAVATT->callbackData;
    }
    delete packetUBXNAVATT;
    packetUBXNAVATT = NULL; // Redundant?
  }

  if (packetUBXNAVPVT != NULL)
  {
    if (packetUBXNAVPVT->callbackData != NULL)
    {
      delete packetUBXNAVPVT->callbackData;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("end: packetUBXNAVPVT->callbackData has been deleted"));
      }
#endif
    }
    delete packetUBXNAVPVT;
    packetUBXNAVPVT = NULL; // Redundant?
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("end: packetUBXNAVPVT has been deleted"));
    }
#endif
  }

  if (packetUBXNAVODO != NULL)
  {
    if (packetUBXNAVODO->callbackData != NULL)
    {
      delete packetUBXNAVODO->callbackData;
    }
    delete packetUBXNAVODO;
    packetUBXNAVODO = NULL; // Redundant?
  }

  if (packetUBXNAVVELECEF != NULL)
  {
    if (packetUBXNAVVELECEF->callbackData != NULL)
    {
      delete packetUBXNAVVELECEF->callbackData;
    }
    delete packetUBXNAVVELECEF;
    packetUBXNAVVELECEF = NULL; // Redundant?
  }

  if (packetUBXNAVVELNED != NULL)
  {
    if (packetUBXNAVVELNED->callbackData != NULL)
    {
      delete packetUBXNAVVELNED->callbackData;
    }
    delete packetUBXNAVVELNED;
    packetUBXNAVVELNED = NULL; // Redundant?
  }

  if (packetUBXNAVHPPOSECEF != NULL)
  {
    if (packetUBXNAVHPPOSECEF->callbackData != NULL)
    {
      delete packetUBXNAVHPPOSECEF->callbackData;
    }
    delete packetUBXNAVHPPOSECEF;
    packetUBXNAVHPPOSECEF = NULL; // Redundant?
  }

  if (packetUBXNAVHPPOSLLH != NULL)
  {
    if (packetUBXNAVHPPOSLLH->callbackData != NULL)
    {
      delete packetUBXNAVHPPOSLLH->callbackData;
    }
    delete packetUBXNAVHPPOSLLH;
    packetUBXNAVHPPOSLLH = NULL; // Redundant?
  }

  if (packetUBXNAVPVAT != NULL)
  {
    if (packetUBXNAVPVAT->callbackData != NULL)
    {
      delete packetUBXNAVPVAT->callbackData;
    }
    delete packetUBXNAVPVAT;
    packetUBXNAVPVAT = NULL; // Redundant?
  }

  if (packetUBXNAVTIMEUTC != NULL)
  {
    if (packetUBXNAVTIMEUTC->callbackData != NULL)
    {
      delete packetUBXNAVTIMEUTC->callbackData;
    }
    delete packetUBXNAVTIMEUTC;
    packetUBXNAVTIMEUTC = NULL; // Redundant?
  }

  if (packetUBXNAVCLOCK != NULL)
  {
    if (packetUBXNAVCLOCK->callbackData != NULL)
    {
      delete packetUBXNAVCLOCK->callbackData;
    }
    delete packetUBXNAVCLOCK;
    packetUBXNAVCLOCK = NULL; // Redundant?
  }

  if (packetUBXNAVSVIN != NULL)
  {
    if (packetUBXNAVSVIN->callbackData != NULL)
    {
      delete packetUBXNAVSVIN->callbackData;
    }
    delete packetUBXNAVSVIN;
    packetUBXNAVSVIN = NULL; // Redundant?
  }

  if (packetUBXNAVSAT != NULL)
  {
    if (packetUBXNAVSAT->callbackData != NULL)
    {
      delete packetUBXNAVSAT->callbackData;
    }
    delete packetUBXNAVSAT;
    packetUBXNAVSAT = NULL; // Redundant?
  }

  if (packetUBXNAVRELPOSNED != NULL)
  {
    if (packetUBXNAVRELPOSNED->callbackData != NULL)
    {
      delete packetUBXNAVRELPOSNED->callbackData;
    }
    delete packetUBXNAVRELPOSNED;
    packetUBXNAVRELPOSNED = NULL; // Redundant?
  }

  if (packetUBXNAVAOPSTATUS != NULL)
  {
    if (packetUBXNAVAOPSTATUS->callbackData != NULL)
    {
      delete packetUBXNAVAOPSTATUS->callbackData;
    }
    delete packetUBXNAVAOPSTATUS;
    packetUBXNAVAOPSTATUS = NULL; // Redundant?
  }

  if (packetUBXNAVEOE != NULL)
  {
    if (packetUBXNAVEOE->callbackData != NULL)
    {
      delete packetUBXNAVEOE->callbackData;
    }
    delete packetUBXNAVEOE;
    packetUBXNAVEOE = NULL; // Redundant?
  }

  if (packetUBXRXMPMP != NULL)
  {
    if (packetUBXRXMPMP->callbackData != NULL)
    {
      delete packetUBXRXMPMP->callbackData;
    }
    delete packetUBXRXMPMP;
    packetUBXRXMPMP = NULL; // Redundant?
  }

  if (packetUBXRXMPMPmessage != NULL)
  {
    if (packetUBXRXMPMPmessage->callbackData != NULL)
    {
      delete packetUBXRXMPMPmessage->callbackData;
    }
    delete packetUBXRXMPMPmessage;
    packetUBXRXMPMPmessage = NULL; // Redundant?
  }

  if (packetUBXRXMQZSSL6message != NULL)
  {
    if (packetUBXRXMQZSSL6message->callbackData != NULL)
    {
      delete[] packetUBXRXMQZSSL6message->callbackData;
    }
    delete packetUBXRXMQZSSL6message;
    packetUBXRXMQZSSL6message = NULL; // Redundant?
  }

  if (packetUBXRXMCOR != NULL)
  {
    if (packetUBXRXMCOR->callbackData != NULL)
    {
      delete packetUBXRXMCOR->callbackData;
    }
    delete packetUBXRXMCOR;
    packetUBXRXMCOR = NULL; // Redundant?
  }

  if (packetUBXRXMSFRBX != NULL)
  {
    if (packetUBXRXMSFRBX->callbackData != NULL)
    {
      delete packetUBXRXMSFRBX->callbackData;
    }
    delete packetUBXRXMSFRBX;
    packetUBXRXMSFRBX = NULL; // Redundant?
  }

  if (packetUBXRXMRAWX != NULL)
  {
    if (packetUBXRXMRAWX->callbackData != NULL)
    {
      delete packetUBXRXMRAWX->callbackData;
    }
    delete packetUBXRXMRAWX;
    packetUBXRXMRAWX = NULL; // Redundant?
  }

  if (packetUBXCFGRATE != NULL)
  {
    delete packetUBXCFGRATE;
    packetUBXCFGRATE = NULL; // Redundant?
  }

  if (packetUBXTIMTM2 != NULL)
  {
    if (packetUBXTIMTM2->callbackData != NULL)
    {
      delete packetUBXTIMTM2->callbackData;
    }
    delete packetUBXTIMTM2;
    packetUBXTIMTM2 = NULL; // Redundant?
  }

  if (packetUBXESFALG != NULL)
  {
    if (packetUBXESFALG->callbackData != NULL)
    {
      delete packetUBXESFALG->callbackData;
    }
    delete packetUBXESFALG;
    packetUBXESFALG = NULL; // Redundant?
  }

  if (packetUBXESFSTATUS != NULL)
  {
    if (packetUBXESFSTATUS->callbackData != NULL)
    {
      delete packetUBXESFSTATUS->callbackData;
    }
    delete packetUBXESFSTATUS;
    packetUBXESFSTATUS = NULL; // Redundant?
  }

  if (packetUBXESFINS != NULL)
  {
    if (packetUBXESFINS->callbackData != NULL)
    {
      delete packetUBXESFINS->callbackData;
    }
    delete packetUBXESFINS;
    packetUBXESFINS = NULL; // Redundant?
  }

  if (packetUBXESFMEAS != NULL)
  {
    if (packetUBXESFMEAS->callbackData != NULL)
    {
      delete packetUBXESFMEAS->callbackData;
    }
    delete packetUBXESFMEAS;
    packetUBXESFMEAS = NULL; // Redundant?
  }

  if (packetUBXESFRAW != NULL)
  {
    if (packetUBXESFRAW->callbackData != NULL)
    {
      delete packetUBXESFRAW->callbackData;
    }
    delete packetUBXESFRAW;
    packetUBXESFRAW = NULL; // Redundant?
  }

  if (packetUBXMGAACK != NULL)
  {
    delete packetUBXMGAACK;
    packetUBXMGAACK = NULL; // Redundant?
  }

  if (packetUBXMGADBD != NULL)
  {
    delete packetUBXMGADBD;
    packetUBXMGADBD = NULL; // Redundant?
  }

  if (packetUBXHNRATT != NULL)
  {
    if (packetUBXHNRATT->callbackData != NULL)
    {
      delete packetUBXHNRATT->callbackData;
    }
    delete packetUBXHNRATT;
    packetUBXHNRATT = NULL; // Redundant?
  }

  if (packetUBXHNRINS != NULL)
  {
    if (packetUBXHNRINS->callbackData != NULL)
    {
      delete packetUBXHNRINS->callbackData;
    }
    delete packetUBXHNRINS;
    packetUBXHNRINS = NULL; // Redundant?
  }

  if (packetUBXHNRPVT != NULL)
  {
    if (packetUBXHNRPVT->callbackData != NULL)
    {
      delete packetUBXHNRPVT->callbackData;
    }
    delete packetUBXHNRPVT;
    packetUBXHNRPVT = NULL; // Redundant?
  }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  if (storageNMEAGPGGA != NULL)
  {
    if (storageNMEAGPGGA->callbackCopy != NULL)
    {
      delete storageNMEAGPGGA->callbackCopy;
    }
    delete storageNMEAGPGGA;
    storageNMEAGPGGA = NULL; // Redundant?
  }

  if (storageNMEAGNGGA != NULL)
  {
    if (storageNMEAGNGGA->callbackCopy != NULL)
    {
      delete storageNMEAGNGGA->callbackCopy;
    }
    delete storageNMEAGNGGA;
    storageNMEAGNGGA = NULL; // Redundant?
  }

  if (storageNMEAGPVTG != NULL)
  {
    if (storageNMEAGPVTG->callbackCopy != NULL)
    {
      delete storageNMEAGPVTG->callbackCopy;
    }
    delete storageNMEAGPVTG;
    storageNMEAGPVTG = NULL; // Redundant?
  }

  if (storageNMEAGNVTG != NULL)
  {
    if (storageNMEAGNVTG->callbackCopy != NULL)
    {
      delete storageNMEAGNVTG->callbackCopy;
    }
    delete storageNMEAGNVTG;
    storageNMEAGNVTG = NULL; // Redundant?
  }

  if (storageNMEAGPRMC != NULL)
  {
    if (storageNMEAGPRMC->callbackCopy != NULL)
    {
      delete storageNMEAGPRMC->callbackCopy;
    }
    delete storageNMEAGPRMC;
    storageNMEAGPRMC = NULL; // Redundant?
  }

  if (storageNMEAGNRMC != NULL)
  {
    if (storageNMEAGNRMC->callbackCopy != NULL)
    {
      delete storageNMEAGNRMC->callbackCopy;
    }
    delete storageNMEAGNRMC;
    storageNMEAGNRMC = NULL; // Redundant?
  }

  if (storageNMEAGPZDA != NULL)
  {
    if (storageNMEAGPZDA->callbackCopy != NULL)
    {
      delete storageNMEAGPZDA->callbackCopy;
    }
    delete storageNMEAGPZDA;
    storageNMEAGPZDA = NULL; // Redundant?
  }

  if (storageNMEAGNZDA != NULL)
  {
    if (storageNMEAGNZDA->callbackCopy != NULL)
    {
      delete storageNMEAGNZDA->callbackCopy;
    }
    delete storageNMEAGNZDA;
    storageNMEAGNZDA = NULL; // Redundant?
  }
#endif
}

// Allow the user to change packetCfgPayloadSize. Handy if you want to process big messages like RAWX
// This can be called before .begin if required / desired
bool SFE_UBLOX_GNSS::setPacketCfgPayloadSize(size_t payloadSize)
{
  bool success = true;

  if ((payloadSize == 0) && (payloadCfg != NULL))
  {
    // Zero payloadSize? Dangerous! But we'll free the memory anyway...
    delete[] payloadCfg; // Created with new[]
    payloadCfg = NULL;   // Redundant?
    packetCfg.payload = payloadCfg;
    packetCfgPayloadSize = payloadSize;
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setPacketCfgPayloadSize: Zero payloadSize!"));
  }

  else if (payloadCfg == NULL) // Memory has not yet been allocated - so use new
  {
    payloadCfg = new uint8_t[payloadSize];
    packetCfg.payload = payloadCfg;
    if (payloadCfg == NULL)
    {
      success = false;
      packetCfgPayloadSize = 0;
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial->println(F("setPacketCfgPayloadSize: RAM alloc failed!"));
    }
    else
      packetCfgPayloadSize = payloadSize;
  }

  else // Memory has already been allocated - so resize
  {
    uint8_t *newPayload = new uint8_t[payloadSize];

    if (newPayload == NULL) // Check if the alloc was successful
    {
      success = false;                                           // Report failure. Don't change payloadCfg, packetCfg.payload or packetCfgPayloadSize
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial->println(F("setPacketCfgPayloadSize: RAM resize failed!"));
    }
    else
    {
      memcpy(newPayload, payloadCfg, payloadSize <= packetCfgPayloadSize ? payloadSize : packetCfgPayloadSize); // Copy as much existing data as we can
      delete[] payloadCfg;                                                                                      // Free payloadCfg. Created with new[]
      payloadCfg = newPayload;                                                                                  // Point to the newPayload
      packetCfg.payload = payloadCfg;                                                                           // Update the packet pointer
      packetCfgPayloadSize = payloadSize;                                                                       // Update the packet payload size
    }
  }

  return (success);
}

// Return the number of free bytes remaining in packetCfgPayload
size_t SFE_UBLOX_GNSS::getPacketCfgSpaceRemaining()
{
  return (packetCfgPayloadSize - packetCfg.len);
}

// Initialize the I2C port
bool SFE_UBLOX_GNSS::begin(TwoWire &wirePort, uint8_t deviceAddress, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_I2C;
  _i2cPort = &wirePort; // Grab which port the user wants us to use
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // We expect caller to begin their I2C port, with the speed of their choice external to the library
  // But if they forget, we start the hardware here.

  // We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
  // ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
  // hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
  //_i2cPort->begin();

  _gpsI2Caddress = deviceAddress; // Store the I2C address from user

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// PRIVATE: Create the file buffer. Called by .begin
bool SFE_UBLOX_GNSS::createFileBuffer(void)
{
  if (fileBufferSize == 0) // Bail if the user has not called setFileBufferSize
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("createFileBuffer: Warning. fileBufferSize is zero. Data logging is not possible."));
    }
#endif
    return (false);
  }

  if (ubxFileBuffer != NULL) // Bail if RAM has already been allocated for the file buffer
  {                          // This will happen if you call .begin more than once - without calling .end first
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("createFileBuffer: Warning. File buffer already exists. Skipping..."));
    }
#endif
    return (false);
  }

  ubxFileBuffer = new uint8_t[fileBufferSize]; // Allocate RAM for the buffer

  if (ubxFileBuffer == NULL) // Check if the new (alloc) was successful
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("createFileBuffer: RAM alloc failed!"));
    }
    return (false);
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("createFileBuffer: fileBufferSize is: "));
    _debugSerial->println(fileBufferSize);
  }
#endif

  fileBufferHead = 0; // Initialize head and tail
  fileBufferTail = 0;

  return (true);
}

// Want to see the NMEA messages on the Serial port? Here's how
void SFE_UBLOX_GNSS::setNMEAOutputPort(Stream &nmeaOutputPort)
{
  _nmeaOutputPort = &nmeaOutputPort; // Store the port from user
}

// Returns true if I2C device ack's
bool SFE_UBLOX_GNSS::isConnected(uint16_t maxWait)
{
  if (commType == COMM_TYPE_I2C)
  {
    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    if (_i2cPort->endTransmission() != 0)
      return false; // Sensor did not ack
  }

  // Query port configuration to see whether we get a meaningful response
  // We could simply request the config for any port but, just for giggles, let's request the config for most appropriate port
  if (commType == COMM_TYPE_I2C)
    return (getPortSettingsInternal(COM_PORT_I2C, maxWait));
  else if (commType == COMM_TYPE_SERIAL)
    return (getPortSettingsInternal(COM_PORT_UART1, maxWait)); // Could be UART2 - but this is just a response check
  else                                                         // if (commType == COMM_TYPE_SPI)
    return (getPortSettingsInternal(COM_PORT_SPI, maxWait));
}

// Get the latest CFG PRT - as used by isConnected
//  Here's the dilemma:
//  The NEO-D9S doesn't support NAV-RATE so, if we want to include the D9 without creating a special class for it,
//  we need to use something else as the 'isConnected' test. The D9 does support CFG-PRT so we'll use that.
//  BUT many users could already be using getPortSettings and expecting the settings to be returned in packetCfg.
//  So, for isConnected ONLY, we need to enable auto support for CFG-PRT and then disable it afterwards so the settings
//  go back to being returned in packetCfg... What a tangled web we weave...!
bool SFE_UBLOX_GNSS::getPortSettingsInternal(uint8_t portID, uint16_t maxWait)
{
  if (packetUBXCFGPRT == NULL)
    initPacketUBXCFGPRT();     // Check that RAM has been allocated for the data
  if (packetUBXCFGPRT == NULL) // Bail if the RAM allocation failed
    return (false);

  // The CFG PRT message will never be produced automatically - that would be pointless.
  // There is no setAutoCFGPRT function. We always need to poll explicitly.
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 1;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = portID;

  // The data is parsed as part of processing the response
  sfe_ublox_status_e result = sendCommand(&packetCfg, maxWait);
  bool retVal = false;

  if (result == SFE_UBLOX_STATUS_DATA_RECEIVED)
    retVal = true;

  if (result == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    retVal = true;

  // Now disable automatic support for CFG-PRT (see above)
  delete packetUBXCFGPRT;
  packetUBXCFGPRT = NULL;

  return (retVal);
}

// This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
// User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
// Or user could pipe each character to a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA_v(char incoming)
{
  // If user has assigned an output port then pipe the characters there
  if (_nmeaOutputPort != NULL)
    _nmeaOutputPort->write(incoming); // Echo this byte to the serial port
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
bool SFE_UBLOX_GNSS::getProtocolVersion(uint16_t maxWait)
{
  Serial.println("Getting proto version");
  // return false;
  if (moduleSWVersion == NULL)
    initModuleSWVersion();     // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == NULL) // Bail if the RAM allocation failed
    return (false);

  // Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_VER;

  packetCfg.len = 0;
  packetCfg.startingSpot = 40; // Start at first "extended software information" string

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) {// We are only expecting data (no ACK)
    Serial.println(":(");
    return (false);                                                       // If command send fails then bail
  }

  // Payload should now contain ~220 characters (depends on module type)

  // if (_printDebug == true)
  // {
  //   _debugSerial->print(F("MON VER Payload:"));
  //   for (int location = 0; location < packetCfg.len; location++)
  //   {
  //     if (location % 30 == 0)
  //       _debugSerial->println();
  //     _debugSerial->write(payloadCfg[location]);
  //   }
  //   _debugSerial->println();
  // }

  // We will step through the payload looking at each extension field of 30 bytes
  for (uint8_t extensionNumber = 0; extensionNumber < 10; extensionNumber++)
  {
    // Now we need to find "PROTVER=18.00" in the incoming byte stream
    if ((payloadCfg[(30 * extensionNumber) + 0] == 'P') && (payloadCfg[(30 * extensionNumber) + 6] == 'R'))
    {
      moduleSWVersion->versionHigh = (payloadCfg[(30 * extensionNumber) + 8] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 9] - '0');  // Convert '18' to 18
      moduleSWVersion->versionLow = (payloadCfg[(30 * extensionNumber) + 11] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 12] - '0'); // Convert '00' to 00
      moduleSWVersion->moduleQueried = true;                                                                                                // Mark this data as new

      if (true)
      {
        Serial.println(F("Protocol version: "));
        Serial.println(moduleSWVersion->versionHigh);
        Serial.println(F("."));
        Serial.println(moduleSWVersion->versionLow);
      }
      return (true); // Success!
    }
  }

  return (false); // We failed
}

// This function is called for each byte of an RTCM frame
// Ths user can overwrite this function and process the RTCM frame as they please
// Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
void SFE_UBLOX_GNSS::processRTCM_v(uint8_t incoming)
{
  // Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

  //_debugSerial->write(incoming); //An example of passing this byte out the serial port

  // Debug printing
  //   _debugSerial->print(F(" "));
  //   if(incoming < 0x10) _debugSerial->print(F("0"));
  //   _debugSerial->print(incoming, HEX);
  //   if(rtcmFrameCounter % 16 == 0) _debugSerial->println();

  (void)incoming; // Do something with incoming just to get rid of the pesky compiler warning!
}

// We need to be able to identify an RTCM packet and then the length
// so that we know when the RTCM message is completely received and we then start
// listening for other sentences (like NMEA or UBX)
// RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
// http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
// https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
// Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
// RTCM 3.2 bytes look like this:
// Byte 0: Always 0xD3
// Byte 1: 6-bits of zero
// Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
// byte 3 + 4 bits: Msg type 12 bits
// Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
SFE_UBLOX_GNSS::sfe_ublox_sentence_types_e SFE_UBLOX_GNSS::processRTCMframe_v(uint8_t incoming, uint16_t *rtcmFrameCounter)
{
  static uint16_t rtcmLen = 0;

  if (*rtcmFrameCounter == 1)
  {
    rtcmLen = (incoming & 0x03) << 8; // Get the last two bits of this byte. Bits 8&9 of 10-bit length
  }
  else if (*rtcmFrameCounter == 2)
  {
    rtcmLen |= incoming; // Bits 0-7 of packet length
    rtcmLen += 6;        // There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
  }
  /*else if (rtcmFrameCounter == 3)
  {
    rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
  }
  else if (rtcmFrameCounter == 4)
  {
    rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }*/

  *rtcmFrameCounter = *rtcmFrameCounter + 1;

  processRTCM(incoming); // Here is where we expose this byte to the user

  // Reset and start looking for next sentence type when done
  return (*rtcmFrameCounter == rtcmLen) ? SFE_UBLOX_SENTENCE_TYPE_NONE : SFE_UBLOX_SENTENCE_TYPE_RTCM;
}

// PRIVATE: Allocate RAM for moduleSWVersion and initialize it
bool SFE_UBLOX_GNSS::initModuleSWVersion()
{
  moduleSWVersion = new moduleSWVersion_t; // Allocate RAM for the main struct
  if (moduleSWVersion == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initModuleSWVersion: RAM alloc failed!"));
#endif
    return (false);
  }
  moduleSWVersion->versionHigh = 0;
  moduleSWVersion->versionLow = 0;
  moduleSWVersion->moduleQueried = false;
  return (true);
}

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e SFE_UBLOX_GNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{
  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("\nSending: "));
    printPacket(outgoingUBX, true); // Always print payload
  }
#endif

  if (commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX, maxWait);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("Send I2C Command failed"));
      }
#endif
      return retVal;
    }
  }
  else {
    if (_printDebug == true) {
      _debugSerial->println("Code for other command formats has been ripped out");
    }
  }

  if (maxWait > 0)
  {
    // Depending on what we just sent, either we need to look for an ACK or not
    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("sendCommand: Waiting for ACK response"));
      }
#endif
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("sendCommand: Waiting for No ACK response"));
      }
#endif
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }
  return retVal;
}

// PRIVATE: Allocate RAM for packetUBXCFGPRT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXCFGPRT()
{
  packetUBXCFGPRT = new UBX_CFG_PRT_t; // Allocate RAM for the main struct
  if (packetUBXCFGPRT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXCFGPRT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXCFGPRT->dataValid = false;
  return (true);
}

// Returns false if sensor fails to respond to I2C traffic
sfe_ublox_status_e SFE_UBLOX_GNSS::sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
  // From the integration guide:
  // "The receiver does not provide any write access except for writing UBX and NMEA messages to the
  //  receiver, such as configuration or aiding data. Therefore, the register set mentioned in section Read
  //  Access is not writeable. Following the start condition from the master, the 7-bit device address and
  //  the RW bit (which is a logic low for write access) are clocked onto the bus by the master transmitter.
  //  The receiver answers with an acknowledge (logic low) to indicate that it is responsible for the given
  //  address. Now, the master can write 2 to N bytes to the receiver, generating a stop condition after the
  //  last byte being written. The number of data bytes must be at least 2 to properly distinguish from
  //  the write access to set the address counter in random read accesses."
  // I take two things from this:
  // 1) We do not need to write 0xFF to point at register 0xFF. We're already pointing at it.
  // 2) We must always write at least 2 bytes, otherwise it looks like we are starting to do a read.
  // Point 2 is important. It means:
  // * In this function:
  //     if we do multiple writes (because we're trying to write more than i2cTransactionSize),
  //     we may need to write one byte less in the penultimate write to ensure we always have two bytes left for the final write.
  // * In pushRawData:
  //     if there is one byte to write, or one byte left to write, we need to do the same thing and may need to store a single
  //     byte until pushRawData is called again.
  // The next four lines can be commented. We do not need to point at the 0xFF data register
  //_i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  //_i2cPort->write(0xFF);
  // if (_i2cPort->endTransmission(false) != 0)         //Don't release bus
  //  return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

  // The total number of bytes to be written is: payload len + 8
  // UBX_SYNCH_1
  // UBX_SYNCH_2
  // cls
  // id
  // len (MSB)
  // len (LSB)
  // < payload >
  // checksumA
  // checksumB

  // i2cTransactionSize will be at least 8. We don't need to check for smaller values than that.

  uint16_t bytesToSend = outgoingUBX->len + 8; // How many bytes need to be sent
  uint16_t bytesSent = 0;                      // How many bytes have been sent
  uint16_t bytesLeftToSend = bytesToSend;      // How many bytes remain to be sent
  uint16_t startSpot = 0;                      // Payload pointer

  while (bytesLeftToSend > 0)
  {
    uint16_t len = bytesLeftToSend; // How many bytes should we actually write?
    if (len > i2cTransactionSize)   // Limit len to i2cTransactionSize
      len = i2cTransactionSize;

    bytesLeftToSend -= len; // Calculate how many bytes will be left after we do this write

    // If bytesLeftToSend is zero, that's OK.
    // If bytesLeftToSend is >= 2, that's OK.
    // But if bytesLeftToSend is 1, we need to adjust len to make sure we write at least 2 bytes in the final write
    if (bytesLeftToSend == 1)
    {
      len -= 1;             // Decrement len by 1
      bytesLeftToSend += 1; // Increment bytesLeftToSend by 1
    }

    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); // Start the transmission

    if (bytesSent == 0) // Is this the first write? If it is, write the header bytes
    {
      _i2cPort->write(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
      _i2cPort->write(UBX_SYNCH_2); // b
      _i2cPort->write(outgoingUBX->cls);
      _i2cPort->write(outgoingUBX->id);
      _i2cPort->write(outgoingUBX->len & 0xFF); // LSB
      _i2cPort->write(outgoingUBX->len >> 8);   // MSB

      bytesSent += 6;

      uint16_t x = 0;
      // Write a portion of the payload to the bus.
      // Keep going until we reach the end of the payload (x == outgoingUBX->len)
      // or we've sent as many bytes as we can in this transmission (bytesSent == len).
      for (; (x < outgoingUBX->len) && (bytesSent < len); x++)
      {
        _i2cPort->write(outgoingUBX->payload[startSpot + x]);
        bytesSent++;
      }
      startSpot += x;

      // Can we write both checksum bytes?
      // We can send both bytes now if we have exactly 2 bytes left
      // to be sent in this transmission (bytesSent == (len - 2)).
      if (bytesSent == (len - 2))
      {
        // Write checksum
        _i2cPort->write(outgoingUBX->checksumA);
        _i2cPort->write(outgoingUBX->checksumB);
        bytesSent += 2;
      }
    }
    else // Keep writing payload bytes. Write the checksum at the right time.
    {
      uint16_t x = 0;
      // Write a portion of the payload to the bus.
      // Keep going until we've sent as many bytes as we can in this transmission (x == len)
      // or until we reach the end of the payload ((startSpot + x) == (outgoingUBX->len))
      for (; (x < len) && ((startSpot + x) < (outgoingUBX->len)); x++)
      {
        _i2cPort->write(outgoingUBX->payload[startSpot + x]);
        bytesSent++;
      }
      startSpot += x;

      // Can we write both checksum bytes?
      // We can send both bytes if we have exactly 2 bytes left to be sent (bytesSent == (bytesToSend - 2))
      // and if there is room for 2 bytes in this transmission
      if ((bytesSent == (bytesToSend - 2)) && (x == (len - 2)))
      {
        // Write checksum
        _i2cPort->write(outgoingUBX->checksumA);
        _i2cPort->write(outgoingUBX->checksumB);
        bytesSent += 2;
      }
    }

    if (bytesSent < bytesToSend) // Do we need to go round the loop again?
    {
      if (_i2cPort->endTransmission(_i2cStopRestart) != 0) // Don't release bus unless we have to
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE);        // Sensor did not ACK
    }
  }

  // All done transmitting bytes. Release bus.
  if (_i2cPort->endTransmission() != 0)
    return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

  (void)maxWait; // Do something with maxWait just to avoid the pesky compiler warnings!

  return (SFE_UBLOX_STATUS_SUCCESS);
}

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
void SFE_UBLOX_GNSS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

// When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge"(UBX - ACK - ACK) or a
//"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.
// Some messages from other classes also use the same acknowledgement mechanism.

// When we poll or get a setting, we will receive _both_ a config packet and an ACK
// If the poll or get request is not valid, we will receive _only_ a NACK

// If we are trying to get or poll a setting, then packetCfg.len will be 0 or 1 when the packetCfg is _sent_.
// If we poll the setting for a particular port using UBX-CFG-PRT then .len will be 1 initially
// For all other gets or polls, .len will be 0 initially
//(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to poll the settings for a particular message - but we don't use that (currently))

// If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still be 0 or 1 after the NACK is received
// But if the get or poll is ACK'd, then packetCfg.len will have been updated by the incoming data and will always be at least 2

// If we are going to set the value for a setting, then packetCfg.len will be at least 3 when the packetCfg is _sent_.
//(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

// We need to think carefully about how interleaved PVT packets affect things.
// It is entirely possible that our packetCfg and packetAck were received successfully
// but while we are still in the "if (checkUblox() == true)" loop a PVT packet is processed
// or _starts_ to arrive (remember that Serial data can arrive very slowly).

// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
// Returns SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
// Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
// Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() < (startTime + (unsigned long)maxTime))
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {
      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: valid data and valid ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by an automatic packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: no data and valid ACK after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
      }

      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: data being OVERWRITTEN after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: CRC failed after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
      }

      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
      // but outgoingUBX->cls and outgoingUBX->id would not match...
      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
      // the packet was definitely NACK'd otherwise we are possibly just guessing...
      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: valid data after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec. Waiting for ACK."));
        // }
      }

    } // checkUbloxInternal == true

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }           // while (millis() < (startTime + (unsigned long)maxTime))

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->print(F("waitForACKResponse: TIMEOUT with valid data after "));
      _debugSerial->print(millis() - startTime);
      _debugSerial->println(F(" msec. "));
    }
#endif
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("waitForACKResponse: TIMEOUT after "));
    _debugSerial->print(millis() - startTime);
    _debugSerial->println(F(" msec."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// For non-CFG queries no ACK is sent so we use this function
// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {

      // If outgoingUBX->classAndIDmatch is VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
      }

      // If the outgoingUBX->classAndIDmatch is VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If outgoingUBX->classAndIDmatch is NOT_DEFINED
      // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForNoACKResponse: valid but UNWANTED data after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->print(F(" msec. Class: "));
        //   _debugSerial->print(outgoingUBX->cls);
        //   _debugSerial->print(F(" ID: "));
        //   _debugSerial->print(outgoingUBX->id);
        // }
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
      else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
      }
    }

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("waitForNoACKResponse: TIMEOUT after "));
    _debugSerial->print(millis() - startTime);
    _debugSerial->println(F(" msec. No packet received."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}


// PRIVATE: Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GNSS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (commType == COMM_TYPE_I2C)
    return (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  return false;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool SFE_UBLOX_GNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_gpsI2Caddress);
    _i2cPort->write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = _i2cPort->endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: I2C error: endTransmission returned "));
        _debugSerial->println(i2cError);
      }
#endif
      return (false); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint8_t bytesReturned = _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: I2C error: requestFrom 0xFD returned "));
        _debugSerial->println(bytesReturned);
      }
#endif
      return (false); // Sensor did not return 2 bytes
    }
    else // if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      // if (lsb == 0xFF)
      // {
      //   //I believe this is a u-blox bug. Device should never present an 0xFF.
      //   if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      //   {
      //     _debugSerial->print(F("checkUbloxI2C: u-blox bug? Length lsb is 0xFF. i2cPollingWait is "));
      //     _debugSerial->println(i2cPollingWait);
      //   }
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      //   lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      //   return (false);
      // }
      // if (msb == 0xFF)
      // {
      //   //I believe this is a u-blox bug. Device should never present an 0xFF.
      //   if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      //   {
      //     _debugSerial->print(F("checkUbloxI2C: u-blox bug? Length msb is 0xFF. i2cPollingWait is "));
      //     _debugSerial->println(i2cPollingWait);
      //   }
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      //   lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      //   return (false);
      // }
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("checkUbloxI2C: OK, zero bytes available"));
      }
#endif
      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);

      // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      // {
      //   _debugSerial->print(F("checkUbloxI2C: Bytes available error: "));
      //   _debugSerial->println(bytesAvailable);
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      // }
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (bytesAvailable > 100)
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Large packet of "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes received"));
      }
    }
    else
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Reading "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes"));
      }
    }
#endif

    while (bytesAvailable)
    {
      // From the u-blox integration manual:
      // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
      //  address and thus allows any register to be read. The second "current address" form omits the
      //  register address. If this second form is used, then an address pointer in the receiver is used to
      //  determine which register to read. This address pointer will increment after each read unless it
      //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
      //  unaltered."
      // This means that after reading bytesAvailable from 0xFD and 0xFE, the address pointer will already be
      // pointing at 0xFF, so we do not need to write it here. The next four lines can be commented.
      //_i2cPort->beginTransmission(_gpsI2Caddress);
      //_i2cPort->write(0xFF);                     //0xFF is the register to read data from
      // if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      //  return (false);                          //Sensor did not ACK

      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead > i2cTransactionSize)  // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // TRY_AGAIN:

      // Here it would be desireable to use a restart where possible / supported, but only if there will be multiple reads.
      // However, if an individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t bytesReturned = _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead)
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          uint8_t incoming = _i2cPort->read(); // Grab the actual character

          // Check to see if the first read is 0x7F. If it is, the module is not ready to respond. Stop, wait, and try again.
          // Note: the integration manual says:
          //"If there is no data awaiting transmission from the receiver, then this register will deliver the value 0xFF,
          //  which cannot be the first byte of a valid message."
          // But it can be the first byte waiting to be read from the buffer if we have already read part of the message.
          // Therefore I think this check needs to be commented.
          //  if (x == 0)
          //  {
          //    if ((incoming == 0x7F) && (ubx7FcheckDisabled == false))
          //    {
          //      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          //      {
          //        _debugSerial->println(F("checkUbloxU2C: u-blox error, module not ready with data (7F error)"));
          //      }
          //      delay(5); //In logic analyzation, the module starting responding after 1.48ms
          //      if (debugPin >= 0)
          //      {
          //        digitalWrite((uint8_t)debugPin, LOW);
          //        delay(10);
          //        digitalWrite((uint8_t)debugPin, HIGH);
          //      }
          //      goto TRY_AGAIN;
          //    }
          //  }

          process(incoming, incomingUBX, requestedClass, requestedID); // Process this valid character
        }
      }
      else
        return (false); // Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()


// Processes NMEA and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void SFE_UBLOX_GNSS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (_outputPort != NULL)
    _outputPort->write(incoming); // Echo this byte to the serial port
  if ((currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE) || (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA))
  {
    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
    {
      // This is the start of a binary sentence. Reset flags.
      // We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_UBX;
      // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; // We should not ignore this payload - yet
      // Store data in packetBuf until we know if we have a requested class and ID match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    }
    else if (incoming == '$')
    {
      nmeaByteCounter = 0; // Reset the NMEA byte counter
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NMEA;
    }
    else if (incoming == 0xD3) // RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_RTCM;
    }
    else
    {
      // This character is unknown or we missed the previous start of a sentence
    }
  }

  // Depending on the sentence, pass the character to the individual processor
  if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_UBX)
  {
    // Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) // Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) // ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      // We can now identify the type of response
      // If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        // This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
        {
          // This is not an ACK and we have a class and ID match
          // So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
        }
        // This is not an ACK and we do not have a complete class and ID match
        // So let's check if this is an "automatic" message which has its own storage defined
        else if (checkAutomatic(packetBuf.cls, packetBuf.id))
        {
          // This is not the message we were expecting but it has its own storage and so we should process it anyway.
          // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
          // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
          // reception is complete.
          uint16_t maxPayload = getMaxPayloadSize(packetBuf.cls, packetBuf.id); // Calculate how much RAM we need
          if (maxPayload == 0)
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: getMaxPayloadSize returned ZERO!! Class: 0x"));
              _debugSerial->print(packetBuf.cls);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id);
            }
#endif
          }
          if (payloadAuto != NULL) // Check if memory is already allocated - this should be impossible!
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->println(F("process: memory is already allocated for payloadAuto! Deleting..."));
            }
#endif
            delete[] payloadAuto; // Created with new[]
            payloadAuto = NULL;   // Redundant?
            packetAuto.payload = payloadAuto;
          }
          payloadAuto = new uint8_t[maxPayload]; // Allocate RAM for payloadAuto
          packetAuto.payload = payloadAuto;
          if (payloadAuto == NULL) // Check if the alloc failed
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: memory allocation failed for \"automatic\" message: Class: 0x"));
              _debugSerial->print(packetBuf.cls, HEX);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id, HEX);
              _debugSerial->println(F("process: \"automatic\" message could overwrite data"));
            }
#endif
            // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
            incomingUBX->id = packetBuf.id;
            incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
          }
          else
          {
            // The RAM allocation was successful so we start diverting data into packetAuto and process it
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
            packetAuto.cls = packetBuf.cls; // Copy the class and ID into packetAuto
            packetAuto.id = packetBuf.id;
            packetAuto.counter = packetBuf.counter;           // Copy over the .counter too
            packetAuto.startingSpot = packetBuf.startingSpot; // And the starting spot? (Probably redundant)
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if (_printDebug == true)
            {
              _debugSerial->print(F("process: incoming \"automatic\" message: Class: 0x"));
              _debugSerial->print(packetBuf.cls, HEX);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id, HEX);
            }
#endif
          }
        }
        else
        {
          // This is not an ACK and we do not have a class and ID match
          // so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = true;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) // Length LSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) // Length MSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("process: ZERO LENGTH packet received: Class: 0x"));
          _debugSerial->print(packetBuf.cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(packetBuf.id, HEX);
        }
#endif
        // If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        // The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        // The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == requestedClass)        // and if the class matches
          && (packetBuf.payload[1] == requestedID))          // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->print(F("process: ACK received with .len != 2: Class: 0x"));
            _debugSerial->print(packetBuf.payload[0], HEX);
            _debugSerial->print(F(" ID: 0x"));
            _debugSerial->print(packetBuf.payload[1], HEX);
            _debugSerial->print(F(" len: "));
            _debugSerial->println(packetBuf.len);
          }
#endif
        }
      }
    }

    // Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, requestedClass, requestedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
      processUBX(incoming, &packetAuto, requestedClass, requestedID);

    // Finally, increment the frame counter
    ubxFrameCounter++;
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA) // Process incoming NMEA mesages. Selectively log if desired.
  {
    if ((nmeaByteCounter == 0) && (incoming != '$'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset. (Almost certainly redundant!)
    }
    else if ((nmeaByteCounter == 1) && (incoming != 'G'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.
    }
    else if ((nmeaByteCounter >= 0) && (nmeaByteCounter <= 5))
    {
      nmeaAddressField[nmeaByteCounter] = incoming; // Store the start character and NMEA address field
    }

    if (nmeaByteCounter == 5)
    {
      if (!_signsOfLife) // If _signsOfLife is not already true, set _signsOfLife to true if the NMEA header is valid
      {
        _signsOfLife = isNMEAHeaderValid();
      }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      // Check if we have automatic storage for this message
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        *lengthPtr = 6;                           // Set the working copy length
        memset(nmeaPtr, 0, nmeaMaxLength);        // Clear the working copy
        memcpy(nmeaPtr, &nmeaAddressField[0], 6); // Copy the start character and address field into the working copy
      }
      else
#endif
      {
        // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        // {
        //   _debugSerial->println(F("process: non-auto NMEA message"));
        // }
      }

      // We've just received the end of the address field. Check if it is selected for logging
      if (logThisNMEA())
      {
        storeFileBytes(&nmeaAddressField[0], 6); // Add start character and address field to the file buffer
      }
      // Check if it should be passed to processNMEA
      if (processThisNMEA())
      {
        processNMEA(nmeaAddressField[0]); // Process the start character and address field
        processNMEA(nmeaAddressField[1]);
        processNMEA(nmeaAddressField[2]);
        processNMEA(nmeaAddressField[3]);
        processNMEA(nmeaAddressField[4]);
        processNMEA(nmeaAddressField[5]);
      }
    }

    if ((nmeaByteCounter > 5) || (nmeaByteCounter < 0)) // Should we add incoming to the file buffer and/or pass it to processNMEA?
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        if (*lengthPtr < nmeaMaxLength)
        {
          *(nmeaPtr + *lengthPtr) = incoming; // Store the character
          *lengthPtr = *lengthPtr + 1;        // Increment the length
          if (*lengthPtr == nmeaMaxLength)
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->println(F("process: NMEA buffer is full!"));
            }
          }
        }
      }
#endif
      if (logThisNMEA())
        storeFileBytes(&incoming, 1); // Add incoming to the file buffer
      if (processThisNMEA())
        processNMEA(incoming); // Pass incoming to processNMEA
    }

    if (incoming == '*')
      nmeaByteCounter = -5; // We are expecting * plus two checksum bytes plus CR and LF

    nmeaByteCounter++; // Increment the byte counter

    if (nmeaByteCounter == maxNMEAByteCount)          // Check if we have processed too many bytes
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.

    if (nmeaByteCounter == 0) // Check if we are done
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *workingLengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *workingNMEAPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();

        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
        uint8_t nmeaChecksum = 0;
        uint8_t charsChecked = 1; // Start after the $
        uint8_t thisChar = '\0';
        while ((charsChecked < (nmeaMaxLength - 1)) && (charsChecked < ((*workingLengthPtr) - 4)) && (thisChar != '*'))
        {
          thisChar = *(workingNMEAPtr + charsChecked); // Get a char from the working copy
          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
            nmeaChecksum ^= thisChar;
          charsChecked++; // Increment the counter
        }
        if (thisChar == '*') // Make sure we found the *
        {
          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
          if (expectedChecksum1 >= ':') // Handle Hex correctly
            expectedChecksum1 += 'A' - ':';
          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
          if (expectedChecksum2 >= ':') // Handle Hex correctly
            expectedChecksum2 += 'A' - ':';
          if ((expectedChecksum1 == *(workingNMEAPtr + charsChecked)) && (expectedChecksum2 == *(workingNMEAPtr + charsChecked + 1)))
          {
            uint8_t *completeLengthPtr = getNMEACompleteLengthPtr();    // Get a pointer to the complete copy length
            uint8_t *completeNMEAPtr = getNMEACompleteNMEAPtr();        // Get a pointer to the complete copy NMEA data
            memset(completeNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous complete copy
            memcpy(completeNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the complete copy
            *completeLengthPtr = *workingLengthPtr;                     // Update the length
            nmeaAutomaticFlags *flagsPtr = getNMEAFlagsPtr();           // Get a pointer to the flags
            nmeaAutomaticFlags flagsCopy = *flagsPtr;
            flagsCopy.flags.bits.completeCopyValid = 1; // Set the complete copy valid flag
            flagsCopy.flags.bits.completeCopyRead = 0;  // Clear the complete copy read flag
            *flagsPtr = flagsCopy;                      // Update the flags
            // Callback
            if (doesThisNMEAHaveCallback()) // Do we need to copy the data into the callback copy?
            {
              if (flagsCopy.flags.bits.callbackCopyValid == 0) // Has the callback copy valid flag been cleared (by checkCallbacks)
              {
                uint8_t *callbackLengthPtr = getNMEACallbackLengthPtr();    // Get a pointer to the callback copy length
                uint8_t *callbackNMEAPtr = getNMEACallbackNMEAPtr();        // Get a pointer to the callback copy NMEA data
                memset(callbackNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous callback copy
                memcpy(callbackNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the callback copy
                *callbackLengthPtr = *workingLengthPtr;                     // Update the length
                flagsCopy.flags.bits.callbackCopyValid = 1;                 // Set the callback copy valid flag
                *flagsPtr = flagsCopy;                                      // Update the flags
              }
            }
          }
          else
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: NMEA checksum fail (2)! Expected "));
              _debugSerial->write(expectedChecksum1);
              _debugSerial->write(expectedChecksum2);
              _debugSerial->print(F(" Got "));
              _debugSerial->write(*(workingNMEAPtr + charsChecked));
              _debugSerial->write(*(workingNMEAPtr + charsChecked + 1));
              _debugSerial->println();
            }
          }
        }
        else
        {
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->println(F("process: NMEA checksum fail (1)!"));
          }
        }
      }
#endif
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // All done!
    }
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_RTCM)
  {
    currentSentence = processRTCMframe(incoming, &rtcmFrameCounter); // Deal with RTCM bytes
  }
}

// PRIVATE: Calculate how much RAM is needed to store the payload for a given automatic message
uint16_t SFE_UBLOX_GNSS::getMaxPayloadSize(uint8_t Class, uint8_t ID)
{
  uint16_t maxSize = 0;
  switch (Class)
  {
  case UBX_CLASS_NAV:
  {
    switch (ID)
    {
    case UBX_NAV_POSECEF:
      maxSize = UBX_NAV_POSECEF_LEN;
      break;
    case UBX_NAV_STATUS:
      maxSize = UBX_NAV_STATUS_LEN;
      break;
    case UBX_NAV_DOP:
      maxSize = UBX_NAV_DOP_LEN;
      break;
    case UBX_NAV_ATT:
      maxSize = UBX_NAV_ATT_LEN;
      break;
    case UBX_NAV_PVT:
      maxSize = UBX_NAV_PVT_LEN;
      break;
    case UBX_NAV_ODO:
      maxSize = UBX_NAV_ODO_LEN;
      break;
    case UBX_NAV_VELECEF:
      maxSize = UBX_NAV_VELECEF_LEN;
      break;
    case UBX_NAV_VELNED:
      maxSize = UBX_NAV_VELNED_LEN;
      break;
    case UBX_NAV_HPPOSECEF:
      maxSize = UBX_NAV_HPPOSECEF_LEN;
      break;
    case UBX_NAV_HPPOSLLH:
      maxSize = UBX_NAV_HPPOSLLH_LEN;
      break;
    case UBX_NAV_PVAT:
      maxSize = UBX_NAV_PVAT_LEN;
      break;
    case UBX_NAV_TIMEUTC:
      maxSize = UBX_NAV_TIMEUTC;
      break;
    case UBX_NAV_CLOCK:
      maxSize = UBX_NAV_CLOCK_LEN;
      break;
    case UBX_NAV_TIMELS:
      maxSize = UBX_NAV_TIMELS_LEN;
      break;
    case UBX_NAV_SVIN:
      maxSize = UBX_NAV_SVIN_LEN;
      break;
    case UBX_NAV_SAT:
      maxSize = UBX_NAV_SAT_MAX_LEN;
      break;
    case UBX_NAV_RELPOSNED:
      maxSize = UBX_NAV_RELPOSNED_LEN_F9;
      break;
    case UBX_NAV_AOPSTATUS:
      maxSize = UBX_NAV_AOPSTATUS_LEN;
      break;
    case UBX_NAV_EOE:
      maxSize = UBX_NAV_EOE_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_RXM:
  {
    switch (ID)
    {
    case UBX_RXM_SFRBX:
      maxSize = UBX_RXM_SFRBX_MAX_LEN;
      break;
    case UBX_RXM_RAWX:
      maxSize = UBX_RXM_RAWX_MAX_LEN;
      break;
    case UBX_RXM_PMP:
      maxSize = UBX_RXM_PMP_MAX_LEN;
      break;
    case UBX_RXM_QZSSL6:
      maxSize = UBX_RXM_QZSSL6_MAX_LEN;
      break;
    case UBX_RXM_COR:
      maxSize = UBX_RXM_COR_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_CFG:
  {
    switch (ID)
    {
    case UBX_CFG_PRT:
      maxSize = UBX_CFG_PRT_LEN;
      break;
    case UBX_CFG_RATE:
      maxSize = UBX_CFG_RATE_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_TIM:
  {
    switch (ID)
    {
    case UBX_TIM_TM2:
      maxSize = UBX_TIM_TM2_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_ESF:
  {
    switch (ID)
    {
    case UBX_ESF_ALG:
      maxSize = UBX_ESF_ALG_LEN;
      break;
    case UBX_ESF_INS:
      maxSize = UBX_ESF_INS_LEN;
      break;
    case UBX_ESF_MEAS:
      maxSize = UBX_ESF_MEAS_MAX_LEN;
      break;
    case UBX_ESF_RAW:
      maxSize = UBX_ESF_RAW_MAX_LEN;
      break;
    case UBX_ESF_STATUS:
      maxSize = UBX_ESF_STATUS_MAX_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_MGA:
  {
    switch (ID)
    {
    case UBX_MGA_ACK_DATA0:
      maxSize = UBX_MGA_ACK_DATA0_LEN;
      break;
    case UBX_MGA_DBD:
      maxSize = UBX_MGA_DBD_LEN; // UBX_MGA_DBD_LEN is actually a maximum length. The packets could be shorter than this.
      break;
    }
  }
  break;
  case UBX_CLASS_HNR:
  {
    switch (ID)
    {
    case UBX_HNR_PVT:
      maxSize = UBX_HNR_PVT_LEN;
      break;
    case UBX_HNR_ATT:
      maxSize = UBX_HNR_ATT_LEN;
      break;
    case UBX_HNR_INS:
      maxSize = UBX_HNR_INS_LEN;
      break;
    }
  }
  break;
  }
  return (maxSize);
}

// PRIVATE: Check if we have storage allocated for an incoming "automatic" message
bool SFE_UBLOX_GNSS::checkAutomatic(uint8_t Class, uint8_t ID)
{
  bool result = false;
  switch (Class)
  {
  case UBX_CLASS_NAV:
  {
    switch (ID)
    {
    case UBX_NAV_POSECEF:
      if (packetUBXNAVPOSECEF != NULL)
        result = true;
      break;
    case UBX_NAV_STATUS:
      if (packetUBXNAVSTATUS != NULL)
        result = true;
      break;
    case UBX_NAV_DOP:
      if (packetUBXNAVDOP != NULL)
        result = true;
      break;
    case UBX_NAV_ATT:
      if (packetUBXNAVATT != NULL)
        result = true;
      break;
    case UBX_NAV_PVT:
      if (packetUBXNAVPVT != NULL)
        result = true;
      break;
    case UBX_NAV_ODO:
      if (packetUBXNAVODO != NULL)
        result = true;
      break;
    case UBX_NAV_VELECEF:
      if (packetUBXNAVVELECEF != NULL)
        result = true;
      break;
    case UBX_NAV_VELNED:
      if (packetUBXNAVVELNED != NULL)
        result = true;
      break;
    case UBX_NAV_HPPOSECEF:
      if (packetUBXNAVHPPOSECEF != NULL)
        result = true;
      break;
    case UBX_NAV_HPPOSLLH:
      if (packetUBXNAVHPPOSLLH != NULL)
        result = true;
      break;
    case UBX_NAV_PVAT:
      if (packetUBXNAVPVAT != NULL)
        result = true;
      break;
    case UBX_NAV_TIMEUTC:
      if (packetUBXNAVTIMEUTC != NULL)
        result = true;
      break;
    case UBX_NAV_CLOCK:
      if (packetUBXNAVCLOCK != NULL)
        result = true;
      break;
    case UBX_NAV_TIMELS:
      if (packetUBXNAVTIMELS != NULL)
        result = true;
      break;
    case UBX_NAV_SVIN:
      if (packetUBXNAVSVIN != NULL)
        result = true;
      break;
    case UBX_NAV_SAT:
      if (packetUBXNAVSAT != NULL)
        result = true;
      break;
    case UBX_NAV_RELPOSNED:
      if (packetUBXNAVRELPOSNED != NULL)
        result = true;
      break;
    case UBX_NAV_AOPSTATUS:
      if (packetUBXNAVAOPSTATUS != NULL)
        result = true;
      break;
    case UBX_NAV_EOE:
      if (packetUBXNAVEOE != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_RXM:
  {
    switch (ID)
    {
    case UBX_RXM_SFRBX:
      if (packetUBXRXMSFRBX != NULL)
        result = true;
      break;
    case UBX_RXM_RAWX:
      if (packetUBXRXMRAWX != NULL)
        result = true;
      break;
    case UBX_RXM_PMP:
      if ((packetUBXRXMPMP != NULL) || (packetUBXRXMPMPmessage != NULL))
        result = true;
      break;
    case UBX_RXM_QZSSL6:
      if (packetUBXRXMQZSSL6message != NULL)
        result = true;
      break;
    case UBX_RXM_COR:
      if (packetUBXRXMCOR != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_CFG:
  {
    switch (ID)
    {
    case UBX_CFG_PRT:
      if (packetUBXCFGPRT != NULL)
        result = true;
      break;
    case UBX_CFG_RATE:
      if (packetUBXCFGRATE != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_TIM:
  {
    switch (ID)
    {
    case UBX_TIM_TM2:
      if (packetUBXTIMTM2 != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_ESF:
  {
    switch (ID)
    {
    case UBX_ESF_ALG:
      if (packetUBXESFALG != NULL)
        result = true;
      break;
    case UBX_ESF_INS:
      if (packetUBXESFINS != NULL)
        result = true;
      break;
    case UBX_ESF_MEAS:
      if (packetUBXESFMEAS != NULL)
        result = true;
      break;
    case UBX_ESF_RAW:
      if (packetUBXESFRAW != NULL)
        result = true;
      break;
    case UBX_ESF_STATUS:
      if (packetUBXESFSTATUS != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_MGA:
  {
    switch (ID)
    {
    case UBX_MGA_ACK_DATA0:
      if (packetUBXMGAACK != NULL)
        result = true;
      break;
    case UBX_MGA_DBD:
      if (packetUBXMGADBD != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_HNR:
  {
    switch (ID)
    {
    case UBX_HNR_PVT:
      if (packetUBXHNRPVT != NULL)
        result = true;
      break;
    case UBX_HNR_ATT:
      if (packetUBXHNRATT != NULL)
        result = true;
      break;
    case UBX_HNR_INS:
      if (packetUBXHNRINS != NULL)
        result = true;
      break;
    }
  }
  break;
  }
  return (result);
}



// PRIVATE: Return true if the NMEA header is valid
bool SFE_UBLOX_GNSS::isNMEAHeaderValid()
{
  if (nmeaAddressField[0] != '*')
    return (false);
  if (nmeaAddressField[1] != 'G')
    return (false);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M'))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V'))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M'))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C'))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G'))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A'))
    return (true);
  return (false);
}

// PRIVATE: Return true if we should add this NMEA message to the file buffer for logging
bool SFE_UBLOX_GNSS::logThisNMEA()
{
  if (_logNMEA.bits.all == 1)
    return (true);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_DTM == 1))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GAQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GBQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GBS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GGA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_logNMEA.bits.UBX_NMEA_GLL == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GLQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GNQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GNS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GPQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GQQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GRS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GSA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_GST == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_logNMEA.bits.UBX_NMEA_GSV == 1))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_RLM == 1))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_logNMEA.bits.UBX_NMEA_RMC == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_TXT == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_logNMEA.bits.UBX_NMEA_VLW == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_logNMEA.bits.UBX_NMEA_VTG == 1))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_ZDA == 1))
    return (true);
  return (false);
}



// PRIVATE: Return true if we should pass this NMEA message to processNMEA
bool SFE_UBLOX_GNSS::processThisNMEA()
{
  if (_processNMEA.bits.all == 1)
    return (true);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_DTM == 1))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GAQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GBQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GBS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GGA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_processNMEA.bits.UBX_NMEA_GLL == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GLQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GNQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GNS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GPQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GQQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GRS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GSA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_GST == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_processNMEA.bits.UBX_NMEA_GSV == 1))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_RLM == 1))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_processNMEA.bits.UBX_NMEA_RMC == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_TXT == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_processNMEA.bits.UBX_NMEA_VLW == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_processNMEA.bits.UBX_NMEA_VTG == 1))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_ZDA == 1))
    return (true);
  return (false);
}


// PRIVATE: Add theBytes to the file buffer
bool SFE_UBLOX_GNSS::storeFileBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == NULL) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("storeFileBytes: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("storeFileBytes: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToFileBuffer(theBytes, numBytes);

  return (true);
}


// PRIVATE: Write theBytes to the file buffer
void SFE_UBLOX_GNSS::writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&ubxFileBuffer[fileBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&ubxFileBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    fileBufferHead = bytesLeftToCopy;                                             // Update Head. The next byte written will be written here.
  }
  else
  {
    fileBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }

  // Update fileBufferMaxAvail if required
  uint16_t bytesInBuffer = fileBufferSpaceUsed();
  if (bytesInBuffer > fileBufferMaxAvail)
    fileBufferMaxAvail = bytesInBuffer;
}


// PRIVATE: Check how much space is available in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceAvailable(void)
{
  return (fileBufferSize - fileBufferSpaceUsed());
}

// Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
// This is used when receiving messages from module
void SFE_UBLOX_GNSS::addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}

// Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
// From v2.0: These are public. The user can call these to extract data from custom packets

// Given a spot in the payload array, extract eight bytes and build a uint64_t
uint64_t SFE_UBLOX_GNSS::extractLongLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint64_t val = 0;
  val |= (uint64_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint64_t)msg->payload[spotToStart + 1] << 8 * 1;
  val |= (uint64_t)msg->payload[spotToStart + 2] << 8 * 2;
  val |= (uint64_t)msg->payload[spotToStart + 3] << 8 * 3;
  val |= (uint64_t)msg->payload[spotToStart + 4] << 8 * 4;
  val |= (uint64_t)msg->payload[spotToStart + 5] << 8 * 5;
  val |= (uint64_t)msg->payload[spotToStart + 6] << 8 * 6;
  val |= (uint64_t)msg->payload[spotToStart + 7] << 8 * 7;
  return (val);
}

// Given a spot in the payload array, extract four bytes and build a long
uint32_t SFE_UBLOX_GNSS::extractLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint32_t val = 0;
  val |= (uint32_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint32_t)msg->payload[spotToStart + 1] << 8 * 1;
  val |= (uint32_t)msg->payload[spotToStart + 2] << 8 * 2;
  val |= (uint32_t)msg->payload[spotToStart + 3] << 8 * 3;
  return (val);
}

// Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t SFE_UBLOX_GNSS::extractSignedLong(ubxPacket *msg, uint16_t spotToStart)
{
  union // Use a union to convert from uint32_t to int32_t
  {
    uint32_t unsignedLong;
    int32_t signedLong;
  } unsignedSigned;

  unsignedSigned.unsignedLong = extractLong(msg, spotToStart);
  return (unsignedSigned.signedLong);
}

// Given a spot in the payload array, extract two bytes and build an int
uint16_t SFE_UBLOX_GNSS::extractInt(ubxPacket *msg, uint16_t spotToStart)
{
  uint16_t val = 0;
  val |= (uint16_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)msg->payload[spotToStart + 1] << 8 * 1;
  return (val);
}

// Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t SFE_UBLOX_GNSS::extractSignedInt(ubxPacket *msg, uint16_t spotToStart)
{
  union // Use a union to convert from uint16_t to int16_t
  {
    uint16_t unsignedInt;
    int16_t signedInt;
  } stSignedInt;

  stSignedInt.unsignedInt = extractInt(msg, spotToStart);
  return (stSignedInt.signedInt);
}

// Given a spot, extract a byte from the payload
uint8_t SFE_UBLOX_GNSS::extractByte(ubxPacket *msg, uint16_t spotToStart)
{
  return (msg->payload[spotToStart]);
}

// Given a spot, extract a signed 8-bit value from the payload
int8_t SFE_UBLOX_GNSS::extractSignedChar(ubxPacket *msg, uint16_t spotToStart)
{
  union // Use a union to convert from uint8_t to int8_t
  {
    uint8_t unsignedByte;
    int8_t signedByte;
  } stSignedByte;

  stSignedByte.unsignedByte = extractByte(msg, spotToStart);
  return (stSignedByte.signedByte);
}



// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
// The payload portion of the packet can be 100s of bytes but the max array size is packetCfgPayloadSize bytes.
// startingSpot can be set so we only record a subset of bytes within a larger packet.
void SFE_UBLOX_GNSS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // If incomingUBX is a user-defined custom packet, then the payload size could be different to packetCfgPayloadSize.
  // TO DO: update this to prevent an overrun when receiving an automatic message
  //        and the incomingUBX payload size is smaller than packetCfgPayloadSize.
  uint16_t maximum_payload_size;
  if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
    maximum_payload_size = packetCfgPayloadSize;
  else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
  {
    // Calculate maximum payload size once Class and ID have been received
    // (This check is probably redundant as activePacketBuffer can only be SFE_UBLOX_PACKET_PACKETAUTO
    //  when ubxFrameCounter >= 3)
    // if (incomingUBX->counter >= 2)
    //{
    maximum_payload_size = getMaxPayloadSize(incomingUBX->cls, incomingUBX->id);
    if (maximum_payload_size == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("processUBX: getMaxPayloadSize returned ZERO!! Class: 0x"));
        _debugSerial->print(incomingUBX->cls);
        _debugSerial->print(F(" ID: 0x"));
        _debugSerial->println(incomingUBX->id);
      }
#endif
    }
    //}
    // else
    //  maximum_payload_size = 2;
  }
  else
    maximum_payload_size = 2;

  bool overrun = false;

  // Add all incoming bytes to the rolling checksum
  // Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < incomingUBX->len + 4)
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) // Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) // Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // We're done! Reset the sentence to being looking for a new start char

    // Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid
      _signsOfLife = true;                                  // The checksum is valid, so set the _signsOfLife flag

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: NACK received: Requested Class: 0x"));
          _debugSerial->print(incomingUBX->payload[0], HEX);
          _debugSerial->print(F(" Requested ID: 0x"));
          _debugSerial->println(incomingUBX->payload[1], HEX);
        }
#endif
      }

      // This is not an ACK and we do not have a complete class and ID match
      // So let's check for an "automatic" message arriving
      else if (checkAutomatic(incomingUBX->cls, incomingUBX->id))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: incoming \"automatic\" message: Class: 0x"));
          _debugSerial->print(incomingUBX->cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(incomingUBX->id, HEX);
        }
#endif
      }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->print(F("Incoming: Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck classAndIDmatch"));
        }
      }
#endif

      // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == false)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        // Drive an external pin to allow for easier logic analyzation
        if (debugPin >= 0)
        {
          digitalWrite((uint8_t)debugPin, LOW);
          delay(10);
          digitalWrite((uint8_t)debugPin, HIGH);
        }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        _debugSerial->print(F("Checksum failed:"));
        _debugSerial->print(F(" checksumA: "));
        _debugSerial->print(incomingUBX->checksumA);
        _debugSerial->print(F(" checksumB: "));
        _debugSerial->print(incomingUBX->checksumB);

        _debugSerial->print(F(" rollingChecksumA: "));
        _debugSerial->print(rollingChecksumA);
        _debugSerial->print(F(" rollingChecksumB: "));
        _debugSerial->print(rollingChecksumB);
        _debugSerial->println();
#endif
      }
    }

    // Now that the packet is complete and has been processed, we need to delete the memory
    // allocated for packetAuto
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
    {
      delete[] payloadAuto; // Created with new[]
      payloadAuto = NULL;   // Redundant?
      packetAuto.payload = payloadAuto;
    }
  }
  else // Load this byte into the payload array
  {
    // If an automatic packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (checkAutomatic(incomingUBX->cls, incomingUBX->id))
      startingSpot = 0;
    // Check if this is payload data which should be ignored
    if (ignoreThisPayload == false)
    {
      // Begin recording if counter goes past startingSpot
      if ((incomingUBX->counter - 4) >= startingSpot)
      {
        // Check to see if we have room for this byte
        if (((incomingUBX->counter - 4) - startingSpot) < maximum_payload_size) // If counter = 208, starting spot = 200, we're good to record.
        {
          incomingUBX->payload[(incomingUBX->counter - 4) - startingSpot] = incoming; // Store this byte into payload array
        }
        else
        {
          overrun = true;
        }
      }
    }
  }

  // incomingUBX->counter should never reach maximum_payload_size + class + id + len[2] + checksum[2]
  if (overrun || ((incomingUBX->counter == maximum_payload_size + 6) && (ignoreThisPayload == false)))
  {
    // Something has gone very wrong
    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      if (overrun)
        _debugSerial->print(F("processUBX: buffer overrun detected!"));
      else
        _debugSerial->print(F("processUBX: counter hit maximum_payload_size + 6!"));
      _debugSerial->print(F(" activePacketBuffer: "));
      _debugSerial->print(activePacketBuffer);
      _debugSerial->print(F(" maximum_payload_size: "));
      _debugSerial->println(maximum_payload_size);
    }
#endif
  }

  // Increment the counter
  incomingUBX->counter++;
}

// Once a packet has been received and validated, identify this packet's class/id and update internal flags
void SFE_UBLOX_GNSS::processUBXpacket(ubxPacket *msg)
{
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_POSECEF && msg->len == UBX_NAV_POSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPOSECEF != NULL)
      {
        packetUBXNAVPOSECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPOSECEF->data.ecefX = extractSignedLong(msg, 4);
        packetUBXNAVPOSECEF->data.ecefY = extractSignedLong(msg, 8);
        packetUBXNAVPOSECEF->data.ecefZ = extractSignedLong(msg, 12);
        packetUBXNAVPOSECEF->data.pAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPOSECEF->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPOSECEF->callbackData->iTOW, &packetUBXNAVPOSECEF->data.iTOW, sizeof(UBX_NAV_POSECEF_data_t));
          packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_STATUS && msg->len == UBX_NAV_STATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSTATUS != NULL)
      {
        packetUBXNAVSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVSTATUS->data.gpsFix = extractByte(msg, 4);
        packetUBXNAVSTATUS->data.flags.all = extractByte(msg, 5);
        packetUBXNAVSTATUS->data.fixStat.all = extractByte(msg, 6);
        packetUBXNAVSTATUS->data.flags2.all = extractByte(msg, 7);
        packetUBXNAVSTATUS->data.ttff = extractLong(msg, 8);
        packetUBXNAVSTATUS->data.msss = extractLong(msg, 12);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSTATUS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSTATUS->callbackData->iTOW, &packetUBXNAVSTATUS->data.iTOW, sizeof(UBX_NAV_STATUS_data_t));
          packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_DOP && msg->len == UBX_NAV_DOP_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVDOP != NULL)
      {
        packetUBXNAVDOP->data.iTOW = extractLong(msg, 0);
        packetUBXNAVDOP->data.gDOP = extractInt(msg, 4);
        packetUBXNAVDOP->data.pDOP = extractInt(msg, 6);
        packetUBXNAVDOP->data.tDOP = extractInt(msg, 8);
        packetUBXNAVDOP->data.vDOP = extractInt(msg, 10);
        packetUBXNAVDOP->data.hDOP = extractInt(msg, 12);
        packetUBXNAVDOP->data.nDOP = extractInt(msg, 14);
        packetUBXNAVDOP->data.eDOP = extractInt(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVDOP->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVDOP->callbackData->iTOW, &packetUBXNAVDOP->data.iTOW, sizeof(UBX_NAV_DOP_data_t));
          packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVDOP->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ATT && msg->len == UBX_NAV_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVATT != NULL)
      {
        packetUBXNAVATT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVATT->data.version = extractByte(msg, 4);
        packetUBXNAVATT->data.roll = extractSignedLong(msg, 8);
        packetUBXNAVATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXNAVATT->data.heading = extractSignedLong(msg, 16);
        packetUBXNAVATT->data.accRoll = extractLong(msg, 20);
        packetUBXNAVATT->data.accPitch = extractLong(msg, 24);
        packetUBXNAVATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXNAVATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVATT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVATT->callbackData->iTOW, &packetUBXNAVATT->data.iTOW, sizeof(UBX_NAV_ATT_data_t));
          packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVT && msg->len == UBX_NAV_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVT != NULL)
      {
        packetUBXNAVPVT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVT->data.year = extractInt(msg, 4);
        packetUBXNAVPVT->data.month = extractByte(msg, 6);
        packetUBXNAVPVT->data.day = extractByte(msg, 7);
        packetUBXNAVPVT->data.hour = extractByte(msg, 8);
        packetUBXNAVPVT->data.min = extractByte(msg, 9);
        packetUBXNAVPVT->data.sec = extractByte(msg, 10);
        packetUBXNAVPVT->data.valid.all = extractByte(msg, 11);
        packetUBXNAVPVT->data.tAcc = extractLong(msg, 12);
        packetUBXNAVPVT->data.nano = extractSignedLong(msg, 16); // Includes milliseconds
        packetUBXNAVPVT->data.fixType = extractByte(msg, 20);
        packetUBXNAVPVT->data.flags.all = extractByte(msg, 21);
        packetUBXNAVPVT->data.flags2.all = extractByte(msg, 22);
        packetUBXNAVPVT->data.numSV = extractByte(msg, 23);
        packetUBXNAVPVT->data.lon = extractSignedLong(msg, 24);
        packetUBXNAVPVT->data.lat = extractSignedLong(msg, 28);
        packetUBXNAVPVT->data.height = extractSignedLong(msg, 32);
        packetUBXNAVPVT->data.hMSL = extractSignedLong(msg, 36);
        packetUBXNAVPVT->data.hAcc = extractLong(msg, 40);
        packetUBXNAVPVT->data.vAcc = extractLong(msg, 44);
        packetUBXNAVPVT->data.velN = extractSignedLong(msg, 48);
        packetUBXNAVPVT->data.velE = extractSignedLong(msg, 52);
        packetUBXNAVPVT->data.velD = extractSignedLong(msg, 56);
        packetUBXNAVPVT->data.gSpeed = extractSignedLong(msg, 60);
        packetUBXNAVPVT->data.headMot = extractSignedLong(msg, 64);
        packetUBXNAVPVT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVT->data.headAcc = extractLong(msg, 72);
        packetUBXNAVPVT->data.pDOP = extractInt(msg, 76);
        packetUBXNAVPVT->data.flags3.all = extractByte(msg, 78);
        packetUBXNAVPVT->data.headVeh = extractSignedLong(msg, 84);
        packetUBXNAVPVT->data.magDec = extractSignedInt(msg, 88);
        packetUBXNAVPVT->data.magAcc = extractInt(msg, 90);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVT->callbackData->iTOW, &packetUBXNAVPVT->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
          packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ODO && msg->len == UBX_NAV_ODO_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVODO != NULL)
      {
        packetUBXNAVODO->data.version = extractByte(msg, 0);
        packetUBXNAVODO->data.iTOW = extractLong(msg, 4);
        packetUBXNAVODO->data.distance = extractLong(msg, 8);
        packetUBXNAVODO->data.totalDistance = extractLong(msg, 12);
        packetUBXNAVODO->data.distanceStd = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVODO->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVODO->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVODO->callbackData->version, &packetUBXNAVODO->data.version, sizeof(UBX_NAV_ODO_data_t));
          packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVODO->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELECEF && msg->len == UBX_NAV_VELECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELECEF != NULL)
      {
        packetUBXNAVVELECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELECEF->data.ecefVX = extractSignedLong(msg, 4);
        packetUBXNAVVELECEF->data.ecefVY = extractSignedLong(msg, 8);
        packetUBXNAVVELECEF->data.ecefVZ = extractSignedLong(msg, 12);
        packetUBXNAVVELECEF->data.sAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELECEF->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELECEF->callbackData->iTOW, &packetUBXNAVVELECEF->data.iTOW, sizeof(UBX_NAV_VELECEF_data_t));
          packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELNED && msg->len == UBX_NAV_VELNED_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELNED != NULL)
      {
        packetUBXNAVVELNED->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELNED->data.velN = extractSignedLong(msg, 4);
        packetUBXNAVVELNED->data.velE = extractSignedLong(msg, 8);
        packetUBXNAVVELNED->data.velD = extractSignedLong(msg, 12);
        packetUBXNAVVELNED->data.speed = extractLong(msg, 16);
        packetUBXNAVVELNED->data.gSpeed = extractLong(msg, 20);
        packetUBXNAVVELNED->data.heading = extractSignedLong(msg, 24);
        packetUBXNAVVELNED->data.sAcc = extractLong(msg, 28);
        packetUBXNAVVELNED->data.cAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELNED->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELNED->callbackData->iTOW, &packetUBXNAVVELNED->data.iTOW, sizeof(UBX_NAV_VELNED_data_t));
          packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSECEF && msg->len == UBX_NAV_HPPOSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSECEF != NULL)
      {
        packetUBXNAVHPPOSECEF->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSECEF->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSECEF->data.ecefX = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSECEF->data.ecefY = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSECEF->data.ecefZ = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSECEF->data.ecefXHp = extractSignedChar(msg, 20);
        packetUBXNAVHPPOSECEF->data.ecefYHp = extractSignedChar(msg, 21);
        packetUBXNAVHPPOSECEF->data.ecefZHp = extractSignedChar(msg, 22);
        packetUBXNAVHPPOSECEF->data.flags.all = extractByte(msg, 23);
        packetUBXNAVHPPOSECEF->data.pAcc = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSECEF->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSECEF->callbackData->version, &packetUBXNAVHPPOSECEF->data.version, sizeof(UBX_NAV_HPPOSECEF_data_t));
          packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == UBX_NAV_HPPOSLLH_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSLLH != NULL)
      {
        packetUBXNAVHPPOSLLH->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSLLH->data.flags.all = extractByte(msg, 3);
        packetUBXNAVHPPOSLLH->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSLLH->data.lon = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSLLH->data.lat = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSLLH->data.height = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSLLH->data.hMSL = extractSignedLong(msg, 20);
        packetUBXNAVHPPOSLLH->data.lonHp = extractSignedChar(msg, 24);
        packetUBXNAVHPPOSLLH->data.latHp = extractSignedChar(msg, 25);
        packetUBXNAVHPPOSLLH->data.heightHp = extractSignedChar(msg, 26);
        packetUBXNAVHPPOSLLH->data.hMSLHp = extractSignedChar(msg, 27);
        packetUBXNAVHPPOSLLH->data.hAcc = extractLong(msg, 28);
        packetUBXNAVHPPOSLLH->data.vAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSLLH->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSLLH->callbackData->version, &packetUBXNAVHPPOSLLH->data.version, sizeof(UBX_NAV_HPPOSLLH_data_t));
          packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVAT && msg->len == UBX_NAV_PVAT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVAT != NULL)
      {
        packetUBXNAVPVAT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVAT->data.version = extractByte(msg, 4);
        packetUBXNAVPVAT->data.valid.all = extractByte(msg, 5);
        packetUBXNAVPVAT->data.year = extractInt(msg, 6);
        packetUBXNAVPVAT->data.month = extractByte(msg, 8);
        packetUBXNAVPVAT->data.day = extractByte(msg, 9);
        packetUBXNAVPVAT->data.hour = extractByte(msg, 10);
        packetUBXNAVPVAT->data.min = extractByte(msg, 11);
        packetUBXNAVPVAT->data.sec = extractByte(msg, 12);
        packetUBXNAVPVAT->data.tAcc = extractLong(msg, 16);
        packetUBXNAVPVAT->data.nano = extractSignedLong(msg, 20); // Includes milliseconds
        packetUBXNAVPVAT->data.fixType = extractByte(msg, 24);
        packetUBXNAVPVAT->data.flags.all = extractByte(msg, 25);
        packetUBXNAVPVAT->data.flags2.all = extractByte(msg, 26);
        packetUBXNAVPVAT->data.numSV = extractByte(msg, 27);
        packetUBXNAVPVAT->data.lon = extractSignedLong(msg, 28);
        packetUBXNAVPVAT->data.lat = extractSignedLong(msg, 32);
        packetUBXNAVPVAT->data.height = extractSignedLong(msg, 36);
        packetUBXNAVPVAT->data.hMSL = extractSignedLong(msg, 40);
        packetUBXNAVPVAT->data.hAcc = extractLong(msg, 44);
        packetUBXNAVPVAT->data.vAcc = extractLong(msg, 48);
        packetUBXNAVPVAT->data.velN = extractSignedLong(msg, 52);
        packetUBXNAVPVAT->data.velE = extractSignedLong(msg, 56);
        packetUBXNAVPVAT->data.velD = extractSignedLong(msg, 60);
        packetUBXNAVPVAT->data.gSpeed = extractSignedLong(msg, 64);
        packetUBXNAVPVAT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVAT->data.vehRoll = extractSignedLong(msg, 72);
        packetUBXNAVPVAT->data.vehPitch = extractSignedLong(msg, 76);
        packetUBXNAVPVAT->data.vehHeading = extractSignedLong(msg, 80);
        packetUBXNAVPVAT->data.motHeading = extractSignedLong(msg, 84);
        packetUBXNAVPVAT->data.accRoll = extractInt(msg, 88);
        packetUBXNAVPVAT->data.accPitch = extractInt(msg, 90);
        packetUBXNAVPVAT->data.accHeading = extractInt(msg, 92);
        packetUBXNAVPVAT->data.magDec = extractSignedInt(msg, 94);
        packetUBXNAVPVAT->data.magAcc = extractInt(msg, 96);
        packetUBXNAVPVAT->data.errEllipseOrient = extractInt(msg, 98);
        packetUBXNAVPVAT->data.errEllipseMajor = extractLong(msg, 100);
        packetUBXNAVPVAT->data.errEllipseMinor = extractLong(msg, 104);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVAT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVAT->callbackData->iTOW, &packetUBXNAVPVAT->data.iTOW, sizeof(UBX_NAV_PVAT_data_t));
          packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMEUTC && msg->len == UBX_NAV_TIMEUTC_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMEUTC != NULL)
      {
        packetUBXNAVTIMEUTC->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMEUTC->data.tAcc = extractLong(msg, 4);
        packetUBXNAVTIMEUTC->data.nano = extractSignedLong(msg, 8);
        packetUBXNAVTIMEUTC->data.year = extractInt(msg, 12);
        packetUBXNAVTIMEUTC->data.month = extractByte(msg, 14);
        packetUBXNAVTIMEUTC->data.day = extractByte(msg, 15);
        packetUBXNAVTIMEUTC->data.hour = extractByte(msg, 16);
        packetUBXNAVTIMEUTC->data.min = extractByte(msg, 17);
        packetUBXNAVTIMEUTC->data.sec = extractByte(msg, 18);
        packetUBXNAVTIMEUTC->data.valid.all = extractByte(msg, 19);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVTIMEUTC->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVTIMEUTC->callbackData->iTOW, &packetUBXNAVTIMEUTC->data.iTOW, sizeof(UBX_NAV_TIMEUTC_data_t));
          packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_CLOCK && msg->len == UBX_NAV_CLOCK_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVCLOCK != NULL)
      {
        packetUBXNAVCLOCK->data.iTOW = extractLong(msg, 0);
        packetUBXNAVCLOCK->data.clkB = extractSignedLong(msg, 4);
        packetUBXNAVCLOCK->data.clkD = extractSignedLong(msg, 8);
        packetUBXNAVCLOCK->data.tAcc = extractLong(msg, 12);
        packetUBXNAVCLOCK->data.fAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVCLOCK->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVCLOCK->callbackData->iTOW, &packetUBXNAVCLOCK->data.iTOW, sizeof(UBX_NAV_CLOCK_data_t));
          packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMELS && msg->len == UBX_NAV_TIMELS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMELS != NULL)
      {
        packetUBXNAVTIMELS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMELS->data.version = extractByte(msg, 4);
        packetUBXNAVTIMELS->data.srcOfCurrLs = extractByte(msg, 8);
        packetUBXNAVTIMELS->data.currLs = extractSignedChar(msg, 9);
        packetUBXNAVTIMELS->data.srcOfLsChange = extractByte(msg, 10);
        packetUBXNAVTIMELS->data.lsChange = extractSignedChar(msg, 11);
        packetUBXNAVTIMELS->data.timeToLsEvent = extractSignedLong(msg, 12);
        packetUBXNAVTIMELS->data.dateOfLsGpsWn = extractInt(msg, 16);
        packetUBXNAVTIMELS->data.dateOfLsGpsDn = extractInt(msg, 18);
        packetUBXNAVTIMELS->data.valid.all = extractSignedChar(msg, 23);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMELS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
      }
    }
    else if (msg->id == UBX_NAV_SVIN && msg->len == UBX_NAV_SVIN_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSVIN != NULL)
      {
        packetUBXNAVSVIN->data.version = extractByte(msg, 0);
        packetUBXNAVSVIN->data.iTOW = extractLong(msg, 4);
        packetUBXNAVSVIN->data.dur = extractLong(msg, 8);
        packetUBXNAVSVIN->data.meanX = extractSignedLong(msg, 12);
        packetUBXNAVSVIN->data.meanY = extractSignedLong(msg, 16);
        packetUBXNAVSVIN->data.meanZ = extractSignedLong(msg, 20);
        packetUBXNAVSVIN->data.meanXHP = extractSignedChar(msg, 24);
        packetUBXNAVSVIN->data.meanYHP = extractSignedChar(msg, 25);
        packetUBXNAVSVIN->data.meanZHP = extractSignedChar(msg, 26);
        packetUBXNAVSVIN->data.meanAcc = extractLong(msg, 28);
        packetUBXNAVSVIN->data.obs = extractLong(msg, 32);
        packetUBXNAVSVIN->data.valid = extractSignedChar(msg, 36);
        packetUBXNAVSVIN->data.active = extractSignedChar(msg, 37);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSVIN->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSVIN->callbackData->version, &packetUBXNAVSVIN->data.version, sizeof(UBX_NAV_SVIN_data_t));
          packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSVIN->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_SAT) // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSAT != NULL)
      {
        packetUBXNAVSAT->data.header.iTOW = extractLong(msg, 0);
        packetUBXNAVSAT->data.header.version = extractByte(msg, 4);
        packetUBXNAVSAT->data.header.numSvs = extractByte(msg, 5);

        // The NAV SAT message could contain data for 255 SVs max. (numSvs is uint8_t. UBX_NAV_SAT_MAX_BLOCKS is 255)
        for (uint16_t i = 0; (i < UBX_NAV_SAT_MAX_BLOCKS) && (i < ((uint16_t)packetUBXNAVSAT->data.header.numSvs)) && ((i * 12) < (msg->len - 8)); i++)
        {
          uint16_t offset = (i * 12) + 8;
          packetUBXNAVSAT->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXNAVSAT->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXNAVSAT->data.blocks[i].cno = extractByte(msg, offset + 2);
          packetUBXNAVSAT->data.blocks[i].elev = extractSignedChar(msg, offset + 3);
          packetUBXNAVSAT->data.blocks[i].azim = extractSignedInt(msg, offset + 4);
          packetUBXNAVSAT->data.blocks[i].prRes = extractSignedInt(msg, offset + 6);
          packetUBXNAVSAT->data.blocks[i].flags.all = extractLong(msg, offset + 8);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVSAT->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSAT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSAT->callbackData->header.iTOW, &packetUBXNAVSAT->data.header.iTOW, sizeof(UBX_NAV_SAT_data_t));
          packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_RELPOSNED && ((msg->len == UBX_NAV_RELPOSNED_LEN) || (msg->len == UBX_NAV_RELPOSNED_LEN_F9)))
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVRELPOSNED != NULL)
      {
        // Note:
        //   RELPOSNED on the M8 is only 40 bytes long
        //   RELPOSNED on the F9 is 64 bytes long and contains much more information

        packetUBXNAVRELPOSNED->data.version = extractByte(msg, 0);
        packetUBXNAVRELPOSNED->data.refStationId = extractInt(msg, 2);
        packetUBXNAVRELPOSNED->data.iTOW = extractLong(msg, 4);
        packetUBXNAVRELPOSNED->data.relPosN = extractSignedLong(msg, 8);
        packetUBXNAVRELPOSNED->data.relPosE = extractSignedLong(msg, 12);
        packetUBXNAVRELPOSNED->data.relPosD = extractSignedLong(msg, 16);

        if (msg->len == UBX_NAV_RELPOSNED_LEN)
        {
          // The M8 version does not contain relPosLength or relPosHeading
          packetUBXNAVRELPOSNED->data.relPosLength = 0;
          packetUBXNAVRELPOSNED->data.relPosHeading = 0;
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 21);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 22);
          packetUBXNAVRELPOSNED->data.relPosHPLength = 0; // The M8 version does not contain relPosHPLength
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 24);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 28);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 32);
          // The M8 version does not contain accLength or accHeading
          packetUBXNAVRELPOSNED->data.accLength = 0;
          packetUBXNAVRELPOSNED->data.accHeading = 0;
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 36);
        }
        else
        {
          packetUBXNAVRELPOSNED->data.relPosLength = extractSignedLong(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHeading = extractSignedLong(msg, 24);
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 32);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 33);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 34);
          packetUBXNAVRELPOSNED->data.relPosHPLength = extractSignedChar(msg, 35);
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 36);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 40);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 44);
          packetUBXNAVRELPOSNED->data.accLength = extractLong(msg, 48);
          packetUBXNAVRELPOSNED->data.accHeading = extractLong(msg, 52);
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 60);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVRELPOSNED->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVRELPOSNED->callbackData->version, &packetUBXNAVRELPOSNED->data.version, sizeof(UBX_NAV_RELPOSNED_data_t));
          packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_AOPSTATUS && msg->len == UBX_NAV_AOPSTATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVAOPSTATUS != NULL)
      {
        packetUBXNAVAOPSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVAOPSTATUS->data.aopCfg.all = extractByte(msg, 4);
        packetUBXNAVAOPSTATUS->data.status = extractByte(msg, 5);

        // Mark all datums as fresh (not read before)
        packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVAOPSTATUS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVAOPSTATUS->callbackData->iTOW, &packetUBXNAVAOPSTATUS->data.iTOW, sizeof(UBX_NAV_AOPSTATUS_data_t));
          packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_EOE && msg->len == UBX_NAV_EOE_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVEOE != NULL)
      {
        packetUBXNAVEOE->data.iTOW = extractLong(msg, 0);

        // Mark all datums as fresh (not read before)
        packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVEOE->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVEOE->callbackData->iTOW, &packetUBXNAVEOE->data.iTOW, sizeof(UBX_NAV_EOE_data_t));
          packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVEOE->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_RXM:
    if (msg->id == UBX_RXM_PMP)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it.
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMP != NULL) && (packetUBXRXMPMP->callbackData != NULL)
          //&& (packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMP->callbackData->version = extractByte(msg, 0);
        packetUBXRXMPMP->callbackData->numBytesUserData = extractInt(msg, 2);
        packetUBXRXMPMP->callbackData->timeTag = extractLong(msg, 4);
        packetUBXRXMPMP->callbackData->uniqueWord[0] = extractLong(msg, 8);
        packetUBXRXMPMP->callbackData->uniqueWord[1] = extractLong(msg, 12);
        packetUBXRXMPMP->callbackData->serviceIdentifier = extractInt(msg, 16);
        packetUBXRXMPMP->callbackData->spare = extractByte(msg, 18);
        packetUBXRXMPMP->callbackData->uniqueWordBitErrors = extractByte(msg, 19);

        if (packetUBXRXMPMP->callbackData->version == 0x00)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 524);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 526);
        }
        else // if (packetUBXRXMPMP->data.version == 0x01)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 20);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 22);
        }

        uint16_t userDataStart = (packetUBXRXMPMP->callbackData->version == 0x00) ? 20 : 24;
        uint16_t userDataLength = (packetUBXRXMPMP->callbackData->version == 0x00) ? 504 : (packetUBXRXMPMP->callbackData->numBytesUserData);
        for (uint16_t i = 0; (i < userDataLength) && (i < 504); i++)
        {
          packetUBXRXMPMP->callbackData->userData[i] = extractByte(msg, i + userDataStart);
        }

        packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }

      // Full PMP message, including Class, ID and checksum
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMPmessage != NULL) && (packetUBXRXMPMPmessage->callbackData != NULL)
          //&& (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMPmessage->callbackData->sync1 = UBX_SYNCH_1;
        packetUBXRXMPMPmessage->callbackData->sync2 = UBX_SYNCH_2;
        packetUBXRXMPMPmessage->callbackData->cls = UBX_CLASS_RXM;
        packetUBXRXMPMPmessage->callbackData->ID = UBX_RXM_PMP;
        packetUBXRXMPMPmessage->callbackData->lengthLSB = msg->len & 0xFF;
        packetUBXRXMPMPmessage->callbackData->lengthMSB = msg->len >> 8;

        memcpy(packetUBXRXMPMPmessage->callbackData->payload, msg->payload, msg->len);

        packetUBXRXMPMPmessage->callbackData->checksumA = msg->checksumA;
        packetUBXRXMPMPmessage->callbackData->checksumB = msg->checksumB;

        packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_QZSSL6)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Full QZSSL6 message, including Class, ID and checksum
      for (int ch = 0; ch < UBX_RXM_QZSSL6_NUM_CHANNELS; ch++)
      {
        if (0 == (packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid & (1 << ch)))
        {

          packetUBXRXMQZSSL6message->callbackData[ch].sync1 = UBX_SYNCH_1;
          packetUBXRXMQZSSL6message->callbackData[ch].sync2 = UBX_SYNCH_2;
          packetUBXRXMQZSSL6message->callbackData[ch].cls = UBX_CLASS_RXM;
          packetUBXRXMQZSSL6message->callbackData[ch].ID = UBX_RXM_QZSSL6;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthLSB = msg->len & 0xFF;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthMSB = msg->len >> 8;

          memcpy(packetUBXRXMQZSSL6message->callbackData[ch].payload, msg->payload, msg->len);

          packetUBXRXMQZSSL6message->callbackData[ch].checksumA = msg->checksumA;
          packetUBXRXMQZSSL6message->callbackData[ch].checksumB = msg->checksumB;

          packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid |= (1 << ch);
          break; // abort when added
        }
      }
    }
    else if (msg->id == UBX_RXM_COR)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if ((packetUBXRXMCOR != NULL) && (packetUBXRXMCOR->callbackData != NULL)
          //&& (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMCOR->callbackData->version = extractByte(msg, 0);
        packetUBXRXMCOR->callbackData->ebno = extractByte(msg, 1);
        packetUBXRXMCOR->callbackData->statusInfo.all = extractLong(msg, 4);
        packetUBXRXMCOR->callbackData->msgType = extractInt(msg, 8);
        packetUBXRXMCOR->callbackData->msgSubType = extractInt(msg, 10);

        packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_SFRBX)
    // Note: length is variable
    // Note: on protocol version 17: numWords is (0..16)
    //       on protocol version 18+: numWords is (0..10)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMSFRBX != NULL)
      {
        packetUBXRXMSFRBX->data.gnssId = extractByte(msg, 0);
        packetUBXRXMSFRBX->data.svId = extractByte(msg, 1);
        packetUBXRXMSFRBX->data.freqId = extractByte(msg, 3);
        packetUBXRXMSFRBX->data.numWords = extractByte(msg, 4);
        packetUBXRXMSFRBX->data.chn = extractByte(msg, 5);
        packetUBXRXMSFRBX->data.version = extractByte(msg, 6);

        for (uint8_t i = 0; (i < UBX_RXM_SFRBX_MAX_WORDS) && (i < packetUBXRXMSFRBX->data.numWords) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXRXMSFRBX->data.dwrd[i] = extractLong(msg, 8 + (i * 4));
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMSFRBX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMSFRBX->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMSFRBX->callbackData->gnssId, &packetUBXRXMSFRBX->data.gnssId, sizeof(UBX_RXM_SFRBX_data_t));
          packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMSFRBX->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_RXM_RAWX)
    // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMRAWX != NULL)
      {
        for (uint8_t i = 0; i < 8; i++)
        {
          packetUBXRXMRAWX->data.header.rcvTow[i] = extractByte(msg, i);
        }
        packetUBXRXMRAWX->data.header.week = extractInt(msg, 8);
        packetUBXRXMRAWX->data.header.leapS = extractSignedChar(msg, 10);
        packetUBXRXMRAWX->data.header.numMeas = extractByte(msg, 11);
        packetUBXRXMRAWX->data.header.recStat.all = extractByte(msg, 12);
        packetUBXRXMRAWX->data.header.version = extractByte(msg, 13);

        for (uint8_t i = 0; (i < UBX_RXM_RAWX_MAX_BLOCKS) && (i < packetUBXRXMRAWX->data.header.numMeas) && ((((uint16_t)i) * 32) < (msg->len - 16)); i++)
        {
          uint16_t offset = (((uint16_t)i) * 32) + 16;
          for (uint8_t j = 0; j < 8; j++)
          {
            packetUBXRXMRAWX->data.blocks[i].prMes[j] = extractByte(msg, offset + j);
            packetUBXRXMRAWX->data.blocks[i].cpMes[j] = extractByte(msg, offset + 8 + j);
            if (j < 4)
              packetUBXRXMRAWX->data.blocks[i].doMes[j] = extractByte(msg, offset + 16 + j);
          }
          packetUBXRXMRAWX->data.blocks[i].gnssId = extractByte(msg, offset + 20);
          packetUBXRXMRAWX->data.blocks[i].svId = extractByte(msg, offset + 21);
          packetUBXRXMRAWX->data.blocks[i].sigId = extractByte(msg, offset + 22);
          packetUBXRXMRAWX->data.blocks[i].freqId = extractByte(msg, offset + 23);
          packetUBXRXMRAWX->data.blocks[i].lockTime = extractInt(msg, offset + 24);
          packetUBXRXMRAWX->data.blocks[i].cno = extractByte(msg, offset + 26);
          packetUBXRXMRAWX->data.blocks[i].prStdev = extractByte(msg, offset + 27);
          packetUBXRXMRAWX->data.blocks[i].cpStdev = extractByte(msg, offset + 28);
          packetUBXRXMRAWX->data.blocks[i].doStdev = extractByte(msg, offset + 29);
          packetUBXRXMRAWX->data.blocks[i].trkStat.all = extractByte(msg, offset + 30);
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMRAWX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMRAWX->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMRAWX->callbackData->header.rcvTow[0], &packetUBXRXMRAWX->data.header.rcvTow[0], sizeof(UBX_RXM_RAWX_data_t));
          packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMRAWX->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_CFG:
    if (msg->id == UBX_CFG_PRT && msg->len == UBX_CFG_PRT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXCFGPRT != NULL)
      {
        packetUBXCFGPRT->data.portID = extractByte(msg, 0);
        packetUBXCFGPRT->data.txReady.all = extractInt(msg, 2);
        packetUBXCFGPRT->data.mode = extractLong(msg, 4);
        packetUBXCFGPRT->data.baudRate = extractLong(msg, 8);
        packetUBXCFGPRT->data.inProtoMask.all = extractInt(msg, 12);
        packetUBXCFGPRT->data.outProtoMask.all = extractInt(msg, 14);
        packetUBXCFGPRT->data.flags = extractInt(msg, 16);

        // Mark data as valid
        packetUBXCFGPRT->dataValid = true;
      }
    }
    else if (msg->id == UBX_CFG_RATE && msg->len == UBX_CFG_RATE_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXCFGRATE != NULL)
      {
        packetUBXCFGRATE->data.measRate = extractInt(msg, 0);
        packetUBXCFGRATE->data.navRate = extractInt(msg, 2);
        packetUBXCFGRATE->data.timeRef = extractInt(msg, 4);

        // Mark all datums as fresh (not read before)
        packetUBXCFGRATE->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
      }
    }
    break;
  case UBX_CLASS_TIM:
    if (msg->id == UBX_TIM_TM2 && msg->len == UBX_TIM_TM2_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXTIMTM2 != NULL)
      {
        packetUBXTIMTM2->data.ch = extractByte(msg, 0);
        packetUBXTIMTM2->data.flags.all = extractByte(msg, 1);
        packetUBXTIMTM2->data.count = extractInt(msg, 2);
        packetUBXTIMTM2->data.wnR = extractInt(msg, 4);
        packetUBXTIMTM2->data.wnF = extractInt(msg, 6);
        packetUBXTIMTM2->data.towMsR = extractLong(msg, 8);
        packetUBXTIMTM2->data.towSubMsR = extractLong(msg, 12);
        packetUBXTIMTM2->data.towMsF = extractLong(msg, 16);
        packetUBXTIMTM2->data.towSubMsF = extractLong(msg, 20);
        packetUBXTIMTM2->data.accEst = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXTIMTM2->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXTIMTM2->callbackData->ch, &packetUBXTIMTM2->data.ch, sizeof(UBX_TIM_TM2_data_t));
          packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXTIMTM2->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_ESF:
    if (msg->id == UBX_ESF_ALG && msg->len == UBX_ESF_ALG_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFALG != NULL)
      {
        packetUBXESFALG->data.iTOW = extractLong(msg, 0);
        packetUBXESFALG->data.version = extractByte(msg, 4);
        packetUBXESFALG->data.flags.all = extractByte(msg, 5);
        packetUBXESFALG->data.error.all = extractByte(msg, 6);
        packetUBXESFALG->data.yaw = extractLong(msg, 8);
        packetUBXESFALG->data.pitch = extractSignedInt(msg, 12);
        packetUBXESFALG->data.roll = extractSignedInt(msg, 14);

        // Mark all datums as fresh (not read before)
        packetUBXESFALG->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFALG->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFALG->callbackData->iTOW, &packetUBXESFALG->data.iTOW, sizeof(UBX_ESF_ALG_data_t));
          packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFALG->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_INS && msg->len == UBX_ESF_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFINS != NULL)
      {
        packetUBXESFINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXESFINS->data.iTOW = extractLong(msg, 8);
        packetUBXESFINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXESFINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXESFINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXESFINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXESFINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXESFINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXESFINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFINS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFINS->callbackData->bitfield0.all, &packetUBXESFINS->data.bitfield0.all, sizeof(UBX_ESF_INS_data_t));
          packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_MEAS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFMEAS != NULL)
      {
        packetUBXESFMEAS->data.timeTag = extractLong(msg, 0);
        packetUBXESFMEAS->data.flags.all = extractInt(msg, 4);
        packetUBXESFMEAS->data.id = extractInt(msg, 6);
        for (uint16_t i = 0; (i < DEF_MAX_NUM_ESF_MEAS) && (i < packetUBXESFMEAS->data.flags.bits.numMeas) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXESFMEAS->data.data[i].data.all = extractLong(msg, 8 + (i * 4));
        }
        if ((uint16_t)msg->len > (uint16_t)(8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4)))
          packetUBXESFMEAS->data.calibTtag = extractLong(msg, 8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4));

        // Check if we need to copy the data for the callback
        if ((packetUBXESFMEAS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFMEAS->callbackData->timeTag, &packetUBXESFMEAS->data.timeTag, sizeof(UBX_ESF_MEAS_data_t));
          packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFMEAS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_RAW)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFRAW != NULL)
      {
        packetUBXESFRAW->data.numEsfRawBlocks = (msg->len - 4) / 8; // Record how many blocks were received. Could be 7 or 70 (ZED-F9R vs. NEO-M8U)
        for (uint16_t i = 0; (i < (DEF_NUM_SENS * DEF_MAX_NUM_ESF_RAW_REPEATS)) && ((i * 8) < (msg->len - 4)); i++)
        {
          packetUBXESFRAW->data.data[i].data.all = extractLong(msg, 4 + (i * 8));
          packetUBXESFRAW->data.data[i].sTag = extractLong(msg, 8 + (i * 8));
        }

        // Check if we need to copy the data for the callback
        if ((packetUBXESFRAW->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFRAW->callbackData->data[0].data.all, &packetUBXESFRAW->data.data[0].data.all, sizeof(UBX_ESF_RAW_data_t));
          packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFRAW->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_STATUS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFSTATUS != NULL)
      {
        packetUBXESFSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXESFSTATUS->data.version = extractByte(msg, 4);
        packetUBXESFSTATUS->data.fusionMode = extractByte(msg, 12);
        packetUBXESFSTATUS->data.numSens = extractByte(msg, 15);
        for (uint16_t i = 0; (i < DEF_NUM_SENS) && (i < packetUBXESFSTATUS->data.numSens) && ((i * 4) < (msg->len - 16)); i++)
        {
          packetUBXESFSTATUS->data.status[i].sensStatus1.all = extractByte(msg, 16 + (i * 4) + 0);
          packetUBXESFSTATUS->data.status[i].sensStatus2.all = extractByte(msg, 16 + (i * 4) + 1);
          packetUBXESFSTATUS->data.status[i].freq = extractByte(msg, 16 + (i * 4) + 2);
          packetUBXESFSTATUS->data.status[i].faults.all = extractByte(msg, 16 + (i * 4) + 3);
        }

        // Mark all datums as fresh (not read before)
        packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFSTATUS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFSTATUS->callbackData->iTOW, &packetUBXESFSTATUS->data.iTOW, sizeof(UBX_ESF_STATUS_data_t));
          packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_MGA:
    if (msg->id == UBX_MGA_ACK_DATA0 && msg->len == UBX_MGA_ACK_DATA0_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGAACK != NULL)
      {
        // Calculate how many ACKs are already stored in the ring buffer
        uint8_t ackBufferContains;
        if (packetUBXMGAACK->head >= packetUBXMGAACK->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          ackBufferContains = packetUBXMGAACK->head - packetUBXMGAACK->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_ACK_RINGBUFFER_LEN)
          ackBufferContains = ((uint8_t)(((uint16_t)packetUBXMGAACK->head + (uint16_t)UBX_MGA_ACK_DATA0_RINGBUFFER_LEN) - (uint16_t)packetUBXMGAACK->tail));
        }
        // Have we got space to store this ACK?
        if (ackBufferContains < (UBX_MGA_ACK_DATA0_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          packetUBXMGAACK->data[packetUBXMGAACK->head].type = extractByte(msg, 0);
          packetUBXMGAACK->data[packetUBXMGAACK->head].version = extractByte(msg, 1);
          packetUBXMGAACK->data[packetUBXMGAACK->head].infoCode = extractByte(msg, 2);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgId = extractByte(msg, 3);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[0] = extractByte(msg, 4);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[1] = extractByte(msg, 5);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[2] = extractByte(msg, 6);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[3] = extractByte(msg, 7);
          // Increment the head
          packetUBXMGAACK->head++;
          if (packetUBXMGAACK->head == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
            packetUBXMGAACK->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->println(F("processUBXpacket: packetUBXMGAACK is full. ACK will be lost!"));
          }
#endif
        }
      }
    }
    else if (msg->id == UBX_MGA_DBD && msg->len <= UBX_MGA_DBD_LEN) // Message length may be less than UBX_MGA_DBD_LEN. UBX_MGA_DBD_LEN is the maximum it will be.
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGADBD != NULL)
      {
        // Calculate how many DBDs are already stored in the ring buffer
        uint8_t dbdBufferContains;
        if (packetUBXMGADBD->head >= packetUBXMGADBD->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          dbdBufferContains = packetUBXMGADBD->head - packetUBXMGADBD->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_DBD_RINGBUFFER_LEN)
          dbdBufferContains = ((uint8_t)(((uint16_t)packetUBXMGADBD->head + (uint16_t)UBX_MGA_DBD_RINGBUFFER_LEN) - (uint16_t)packetUBXMGADBD->tail));
        }
        // Have we got space to store this DBD?
        if (dbdBufferContains < (UBX_MGA_DBD_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          // We need to save the entire message - header, payload and checksum
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader1 = UBX_SYNCH_1;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader2 = UBX_SYNCH_2;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryClass = UBX_CLASS_MGA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryID = UBX_MGA_DBD;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenLSB = (uint8_t)(msg->len & 0xFF); // We need to store the length of the DBD entry. The entry itself does not contain a length...
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenMSB = (uint8_t)((msg->len >> 8) & 0xFF);
          for (uint16_t i = 0; i < msg->len; i++)
          {
            packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntry[i] = extractByte(msg, i);
          }
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumA = msg->checksumA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumB = msg->checksumB;
          // Increment the head
          packetUBXMGADBD->head++;
          if (packetUBXMGADBD->head == UBX_MGA_DBD_RINGBUFFER_LEN)
            packetUBXMGADBD->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->println(F("processUBXpacket: packetUBXMGADBD is full. DBD data will be lost!"));
          }
#endif
        }
      }
    }
    break;
  case UBX_CLASS_HNR:
    if (msg->id == UBX_HNR_PVT && msg->len == UBX_HNR_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRPVT != NULL)
      {
        packetUBXHNRPVT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRPVT->data.year = extractInt(msg, 4);
        packetUBXHNRPVT->data.month = extractByte(msg, 6);
        packetUBXHNRPVT->data.day = extractByte(msg, 7);
        packetUBXHNRPVT->data.hour = extractByte(msg, 8);
        packetUBXHNRPVT->data.min = extractByte(msg, 9);
        packetUBXHNRPVT->data.sec = extractByte(msg, 10);
        packetUBXHNRPVT->data.valid.all = extractByte(msg, 11);
        packetUBXHNRPVT->data.nano = extractSignedLong(msg, 12);
        packetUBXHNRPVT->data.gpsFix = extractByte(msg, 16);
        packetUBXHNRPVT->data.flags.all = extractByte(msg, 17);
        packetUBXHNRPVT->data.lon = extractSignedLong(msg, 20);
        packetUBXHNRPVT->data.lat = extractSignedLong(msg, 24);
        packetUBXHNRPVT->data.height = extractSignedLong(msg, 28);
        packetUBXHNRPVT->data.hMSL = extractSignedLong(msg, 32);
        packetUBXHNRPVT->data.gSpeed = extractSignedLong(msg, 36);
        packetUBXHNRPVT->data.speed = extractSignedLong(msg, 40);
        packetUBXHNRPVT->data.headMot = extractSignedLong(msg, 44);
        packetUBXHNRPVT->data.headVeh = extractSignedLong(msg, 48);
        packetUBXHNRPVT->data.hAcc = extractLong(msg, 52);
        packetUBXHNRPVT->data.vAcc = extractLong(msg, 56);
        packetUBXHNRPVT->data.sAcc = extractLong(msg, 60);
        packetUBXHNRPVT->data.headAcc = extractLong(msg, 64);

        // Mark all datums as fresh (not read before)
        packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRPVT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRPVT->callbackData->iTOW, &packetUBXHNRPVT->data.iTOW, sizeof(UBX_HNR_PVT_data_t));
          packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_ATT && msg->len == UBX_HNR_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRATT != NULL)
      {
        packetUBXHNRATT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRATT->data.version = extractByte(msg, 4);
        packetUBXHNRATT->data.roll = extractSignedLong(msg, 8);
        packetUBXHNRATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXHNRATT->data.heading = extractSignedLong(msg, 16);
        packetUBXHNRATT->data.accRoll = extractLong(msg, 20);
        packetUBXHNRATT->data.accPitch = extractLong(msg, 24);
        packetUBXHNRATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXHNRATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRATT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRATT->callbackData->iTOW, &packetUBXHNRATT->data.iTOW, sizeof(UBX_HNR_ATT_data_t));
          packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_INS && msg->len == UBX_HNR_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRINS != NULL)
      {
        packetUBXHNRINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXHNRINS->data.iTOW = extractLong(msg, 8);
        packetUBXHNRINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXHNRINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXHNRINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXHNRINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXHNRINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXHNRINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXHNRINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRINS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRINS->callbackData->bitfield0.all, &packetUBXHNRINS->data.bitfield0.all, sizeof(UBX_HNR_INS_data_t));
          packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  }
}

// PRIVATE: Check how much space is used in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceUsed(void)
{
  if (fileBufferHead >= fileBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (fileBufferHead - fileBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the fileBufferSize
    return ((uint16_t)(((uint32_t)fileBufferHead + (uint32_t)fileBufferSize) - (uint32_t)fileBufferTail));
  }
}



// PRIVATE: Add a UBX packet to the file buffer
bool SFE_UBLOX_GNSS::storePacket(ubxPacket *msg)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == NULL) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("storePacket: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  uint16_t totalLength = msg->len + 8; // Total length. Include sync chars, class, id, length and checksum bytes
  if (totalLength > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("storePacket: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // Store the two sync chars
  uint8_t sync_chars[] = {UBX_SYNCH_1, UBX_SYNCH_2};
  writeToFileBuffer(sync_chars, 2);

  // Store the Class & ID
  writeToFileBuffer(&msg->cls, 1);
  writeToFileBuffer(&msg->id, 1);

  // Store the length. Ensure length is little-endian
  uint8_t msg_length[2];
  msg_length[0] = msg->len & 0xFF;
  msg_length[1] = msg->len >> 8;
  writeToFileBuffer(msg_length, 2);

  // Store the payload
  writeToFileBuffer(msg->payload, msg->len);

  // Store the checksum
  writeToFileBuffer(&msg->checksumA, 1);
  writeToFileBuffer(&msg->checksumB, 1);

  return (true);
}
