#include <IoTT_DigitraxBuffers.h>

blockDetBuffer blockDetectorBuffer;
switchBuffer switchPositionBuffer;
signalBuffer signalAspectBuffer;
analogValBuffer analogValueBuffer;
buttonValBuffer buttonValueBuffer;
powerStatusBuffer sysPowerStatus = 0; //OPC_IDLE
slotDataBuffer slotBuffer;

txFct sendMessage = NULL;

uint32_t nextBufferUpdate = millis();

typedef struct
{
	uint32_t lastActivity;
	uint16_t devAddr;
}protocolEntry;

const uint8_t switchProtLen = 20;
protocolEntry switchProtocol[switchProtLen];
uint8_t swWrPtr = 0;
bool bushbyWatch = false;

void setTxFunction(txFct newFct)
{
	sendMessage = newFct;
}

blockDetBuffer * getBDList()
{
	return &blockDetectorBuffer;
}

void setBDStatus(uint16_t bdNum, bool bdStatus)
{
	uint16_t byteNr = bdNum>>3;  //	uint16_t byteNr = trunc(bdNum/8);
    uint8_t bitMask = 0x01<<(bdNum % 8);
    if (bdStatus)
		blockDetectorBuffer[byteNr] |= bitMask;
	else
        blockDetectorBuffer[byteNr] &= ~bitMask;
}
	
uint8_t getBDStatus(uint16_t bdNum)
{
	uint16_t byteNr = bdNum>>3;  //	uint16_t byteNr = trunc(bdNum/8);
	return ((blockDetectorBuffer[byteNr] >> (bdNum % 8)) & 0x01); //0=free, 1=occ
}

switchBuffer * getSwitchList()
{
	return &switchPositionBuffer;
}

void setSwitchStatus(uint16_t swiNum, bool swiPos, bool coilStatus)
{
	uint16_t byteNr = trunc(swiNum/4);
	uint8_t inpPosStat = 0;
	swWrPtr = (swWrPtr + 1) % switchProtLen;
	switchProtocol[swWrPtr].devAddr = swiNum;
	switchProtocol[swWrPtr].lastActivity = millis();
	if (swiPos)
		inpPosStat |= 0x02;
	if (coilStatus)
		inpPosStat |= 0x01;
    switchPositionBuffer[byteNr] &= ~(0x03<<(2*(swiNum % 4)));
    switchPositionBuffer[byteNr] |= inpPosStat<<(2*(swiNum % 4));
}

uint32_t getLastSwitchActivity(uint16_t swiNum)
{
	uint8_t thisOfs = swWrPtr + switchProtLen;
	for (uint8_t i = 0; i < switchProtLen; i++)
	{
		uint8_t thisEntry = (thisOfs - i) % switchProtLen;
		if (switchProtocol[thisEntry].devAddr == swiNum)
			return switchProtocol[thisEntry].lastActivity;
	}
	return 0;
}

uint8_t getSwiPosition(uint16_t swiNum)
{
	return ((switchPositionBuffer[swiNum >> 2] >> (2 * (swiNum % 4))) & 0x02) << 4;
}

uint8_t getSwiCoilStatus(uint16_t swiNum)
{
	return ((switchPositionBuffer[swiNum >> 2] >> (2 * (swiNum % 4))) & 0x01) << 4;
}

uint8_t getSwiStatus(uint16_t swiNum)
{
	return ((switchPositionBuffer[swiNum >> 2] >> (2 * (swiNum % 4))) & 0x03) << 4;
}

signalBuffer * getSignalList()
{
	return &signalAspectBuffer;
}

void setSignalAspect(uint16_t sigNum, uint8_t sigAspect)
{
	signalAspectBuffer[sigNum] = (sigAspect & 0x1F);
}

uint8_t getSignalAspect(uint16_t sigNum)
{
	return signalAspectBuffer[sigNum];
}

analogValBuffer * getAnalogValBuffer()
{
	return &analogValueBuffer;
}

void setAnalogValue(uint16_t analogNum, uint16_t analogValue)
{
	analogValueBuffer[analogNum] = analogValue;
}

uint16_t getAnalogValue(uint16_t analogNum)
{
	return analogValueBuffer[analogNum];
}


buttonValBuffer * getButtonValBuffer()
{
	return &buttonValueBuffer;
}

void setButtonValue(uint16_t buttonNum, uint8_t buttonValue)
{
	buttonValueBuffer[buttonNum] = buttonValue;
}

uint8_t getButtonValue(uint16_t buttonNum)
{
	return buttonValueBuffer[buttonNum];
}

void setPowerStatus(uint8_t newStatus)
{
	sysPowerStatus = newStatus;
}

uint8_t getPowerStatus()
{
	return sysPowerStatus;
}

slotDataBuffer * getSlotDataBuffer()
{
	return &slotBuffer;
}

void setSlotData(uint8_t slotNum, slotData thisSlot)
{
	memcpy(slotBuffer[slotNum], thisSlot, 10);
}

slotData * getSlotData(uint8_t slotNum)
{
	return &slotBuffer[slotNum];
}

void enableBushbyWatch(bool enableBushby)
{
	bushbyWatch = enableBushby;
}

uint8_t getBushbyStatus()
{
	slotData * ctrlSlot = getSlotData(0x7F);
	return (((*ctrlSlot)[3] & 0x04) >> 2);
}

void processBufferUpdates()
{
	if (millis() > nextBufferUpdate)
	{
		nextBufferUpdate += bufferUpdateInterval;
		//check if system slot need update
	}
}

void processLocoNetMsg(lnReceiveBuffer * newData)
{
  {
      switch (newData->lnData[0])
      {
        case 0xB0: 
			if (bushbyWatch) 
				if (getBushbyStatus() > 0) 
					break; //OPC_SW_REQ
        case 0xB1:; //OPC_SW_REP
        case 0xBD:  //OPC_SW_ACK
        {
          uint16_t swiAddr = ((newData->lnData[1] & 0x7F)) + ((newData->lnData[2] & 0x0F)<<7);
          uint8_t inpPosStat = (newData->lnData[2] & 0x30)>>4; //Direction and ON Status
//          Serial.printf("Set Switch %i to %i\n", swiAddr, inpPosStat);
          setSwitchStatus(swiAddr, (inpPosStat & 0x02)>>1, (inpPosStat & 0x01));
          if (handleSwiEvent)
			handleSwiEvent(swiAddr, (inpPosStat & 0x02)>>1, (inpPosStat & 0x01));
          break;
        }
        case 0xB2: //OPC_INPUT_REP
        {
          uint16_t inpAddr = ((newData->lnData[1] & 0x7F)<<1) + ((newData->lnData[2] & 0x0F)<<8) + ((newData->lnData[2] & 0x20)>>5);
          uint8_t inpStatus = (newData->lnData[2] & 0x10)>>4;
          setBDStatus(inpAddr, inpStatus > 0);
          if (handleInputEvent)
            handleInputEvent(inpAddr, inpStatus);
          break;
        }
        case 0xED: //OPC_IMM_PACKET
        {
          if ((newData->lnData[1] == 0x0B) && (newData->lnData[2] == 0x7F))
          {
            byte recData[6];
            for (int i=0; i < 5; i++)
            {
              recData[i] = newData->lnData[i+4]; //get all the IMM bytes
              if (i > 0)
                recData[i] |= ((recData[0] & (0x01<<(i-1)))<<(8-i)); //distribute 8th  its to data bytes
            }
            word boardAddress = (((~recData[2]) & 0x70) << 2) | (recData[1] & 0x3F) ;
            byte turnoutPairIndex = (recData[2] & 0x06) >> 1;
            word sigAddress = (((boardAddress-1)<<2) | turnoutPairIndex) + 1;
            word sigAspect = recData[3] & 0x1F;
            setSignalAspect(sigAddress, sigAspect);
            if (handleSignalEvent)
              handleSignalEvent(sigAddress, sigAspect);
          }
          break;
        }
        case 0xE5: //OPC_PEER_XFER (button input, sv subgroup)
        {
          //add code to interpret button and analog commands
          uint16_t btnAddr = (newData->lnData[8] & 0x3F) + ((newData->lnData[9] & 0x3F) << 6);
          uint8_t  svCmd = newData->lnData[3];
          uint8_t  svType = newData->lnData[4];
          uint8_t  inputType = newData->lnData[11];
          uint16_t inputValue = (newData->lnData[12] & 0x3F) + ((newData->lnData[13] & 0x3F) << 6);
          if ((svCmd == 0x71) && (svType == 0x02))
          {
            switch (inputType)
            {
              case 0: setAnalogValue(btnAddr, inputValue); 
					  if (handleAnalogValue)	
						handleAnalogValue(btnAddr, inputValue); 
                      break;
              case 1: setButtonValue(btnAddr, inputValue & 0xFF); 
					  if (handleButtonValue)	
						handleButtonValue(btnAddr, inputValue); 
                      break;
            }
          }
          break;
        }
        case 0x82: setPowerStatus(0); if (handlePowerStatus) handlePowerStatus(); break;//OPC_GPOFF
        case 0x83: setPowerStatus(1); if (handlePowerStatus) handlePowerStatus(); break;//OPC_GPON
        case 0x85: setPowerStatus(2); if (handlePowerStatus) handlePowerStatus(); break;//OPC_IDLE
        case 0xEF: //OPC_WR_SL
        case 0xE7: //OPC_SL_RD
          switch (newData->lnData[1])
          {
            case 0x0E: //SL_RD or WR_SL
            {
              uint8_t slotNr = newData->lnData[2];
              slotData newSlot;
              memcpy(&newSlot[0], &newData->lnData[3], 10);
              setSlotData(slotNr, newSlot);
              if (slotNr == 0x7F) //Status slot with OpSw
              {
//                isBushby = ((newData->lnData[6] & 0x04) > 0);
//                Serial.printf("Bushby Bit is %i \n", isBushby);
              }
              break;
            }
            default:
              break;
          }
          break;
      }
  }
}

