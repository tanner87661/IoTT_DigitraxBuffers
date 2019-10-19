//Version 1.0.0
#ifndef IoTT_DigitraxBuffers_h
#define IoTT_DigitraxBuffers_h

#include <Arduino.h>
#include <IoTTCommDef.h>

#define numSigs 2048
#define numSwis 512 //=2048/4
#define numBDs 512 //=4096/8
#define numAnalogVals 4096
#define numButtons 4096
#define numSlots 128

#define bufferUpdateInterval 1000

typedef  uint8_t blockDetBuffer[numBDs]; //4096 input bits, 8 per yte, lsb is lowest number
typedef uint8_t switchBuffer[numSwis]; //current status of switches, 4 per byte. First bit indicates correct state, second is position
typedef uint8_t signalBuffer[numSigs]; //current status of aspects, 1 per byte values 0..31, 3 MSB reserved
typedef uint16_t analogValBuffer[numAnalogVals]; //current status of analog values, 1 per word, running from 0 to 4096
typedef uint8_t buttonValBuffer[numButtons]; //current status of buttons, 2 per byte, statuses down, up, click, hold, dblclick
typedef uint8_t powerStatusBuffer;
typedef uint8_t slotData[10]; //slot data 0 is slot number, this is given by position in array, so we only need 10 bytes
typedef slotData slotDataBuffer[numSlots];

typedef uint16_t (*txFct) (lnTransmitMsg);

void setTxFunction(txFct newFct);

void processLocoNetMsg(lnReceiveBuffer * newData);

uint8_t getBDStatus(uint16_t bdNum);

uint8_t getSwiPosition(uint16_t swiNum);
uint8_t getSwiCoilStatus(uint16_t swiNum);
uint8_t getSwiStatus(uint16_t swiNum);

uint8_t getSignalAspect(uint16_t sigNum);

void setAnalogValue(uint16_t analogNum, uint16_t analogValue);
uint16_t getAnalogValue(uint16_t analogNum);

uint8_t getButtonValue(uint16_t buttonNum);

slotData * getSlotData(uint8_t slotNum);
uint8_t getBushbyStatus();

uint8_t getPowerStatus();

void processBufferUpdates();

#endif

extern void handlePowerStatus() __attribute__ ((weak)); //power status change
extern void handleSwiEvent(uint16_t swiAddr, uint8_t swiPos, uint8_t coilStat) __attribute__ ((weak));
extern void handleInputEvent(uint16_t inpAddr, uint8_t inpStatus) __attribute__ ((weak));
extern void handleSignalEvent(uint16_t sigAddr, uint8_t sigAspect) __attribute__ ((weak));
extern void handleAnalogValue(uint16_t analogAddr, uint16_t inputValue) __attribute__ ((weak));
extern void handleButtonValue(uint16_t btnAddr, uint8_t inputValue) __attribute__ ((weak));
