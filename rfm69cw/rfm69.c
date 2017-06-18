/*
 * rfm69.c
 *
 *  Created on: 19.05.2017
 *      Author: marcio
 */

#include "rfm69.h"

// Local methods
static void rfm69SetHighPowerRegs(bool onOff);
static void rfm69SetupRxInterrupt(uint8_t port, uint16_t pin);
static void rfm69ReceiveBegin(void);
static void rfm69SendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK);

// Global variables
volatile uint8_t dataReceived = false;


// Local shared variables
rfm69Status_t rfm69Stats;


uint8_t rfm69Initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID){
    uint32_t start;
    volatile uint8_t i;
    {
      const uint8_t CONFIG[][2] =
      {
        /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
        /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
        /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
        /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
        /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
        /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

        /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
        /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
        /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

        // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
        // +17dBm and +20dBm are possible on RFM69HW
        // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
        // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
        // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
        ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
        ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

        // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
        /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
        //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
        /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
        /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
        /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
        /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
        ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
        /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
        /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
        /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
        /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
        /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
        ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
        /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
        /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
        {255, 0}
      };

      rfm69Stats.dataAvailable = false;
      rfm69Stats.promiscuousMode = false;
      rfm69Stats.mode = RF69_MODE_INIT;
      rfm69Stats.powerLevel = POWER_LEVEL;

      //digitalWrite(_slaveSelectPin, HIGH);
      GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

      //pinMode(_slaveSelectPin, OUTPUT);
      GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2);

      spiInit();

      start = sysGetGTime();
      do{
          spiWriteReg(REG_SYNCVALUE1, 0xaa);
      }while((spiReadReg(REG_SYNCVALUE1) != 0xaa) && ((sysGetGTime()-start) < RFM69_TIMEOUT));

      start = sysGetGTime();
      do{
          spiWriteReg(REG_SYNCVALUE1, 0x55);
      }while((spiReadReg(REG_SYNCVALUE1) != 0x55) && ((sysGetGTime()-start) < RFM69_TIMEOUT));

      for(i=0; CONFIG[i][0] != 255; i++){
          spiWriteReg(CONFIG[i][0], CONFIG[i][1]);
      }

      // Encryption is persistent between resets and can trip you up during debugging.
      // Disable it during initialization so we always start from a known state.
      rfm69Encrypt(0);

      rfm69SetHighPower();                          // called regardless if it's a RFM69W or RFM69HW
      rfm69Sleep();
      start = sysGetGTime();
      while(((spiReadReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && sysGetGTime()-start < RFM69_TIMEOUT); // wait for ModeReady
      if(sysGetGTime()-start >= RFM69_TIMEOUT){
          return false;
      }
      //_inISR = false;
      rfm69SetupRxInterrupt(RF69_IRQ_PORT, RF69_IRQ_PIN);

      rfm69Stats.devId = nodeID;
      return true;
    }
} // Tested OK! (interrupt handling missing)

void rfm69Encrypt(const uint8_t* key){
    uint8_t i;
    //radioSetMode(RF69_MODE_STANDBY);
    if(key != 0){
        spiSlaveEnable();
        spiTransmitByte(REG_AESKEY1 | 0x80);
        for(i=0; i<16; i++){
            spiTransmitByte(key[i]);
        }
        spiSlaveDisable();
    }
    spiWriteReg(REG_PACKETCONFIG2, (spiReadReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
} // Not tested

// return the frequency (in Hz)
uint32_t rfm69GetFrequency(void){
    return RF69_FSTEP * (((uint32_t) spiReadReg(REG_FRFMSB) << 16) + ((uint16_t) spiReadReg(REG_FRFMID) << 8) + spiReadReg(REG_FRFLSB));
} // Tested OK!

// set the frequency (in Hz)
void rfm69SetFrequency(uint32_t freqHz){
    uint8_t oldMode = rfm69Stats.mode;
    if(oldMode == RF69_MODE_TX){
        rfm69SetMode(RF69_MODE_RX);
    }
    freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
    spiWriteReg(REG_FRFMSB, freqHz >> 16);
    spiWriteReg(REG_FRFMID, freqHz >> 8);
    spiWriteReg(REG_FRFLSB, freqHz);
    if(oldMode == RF69_MODE_RX){
        rfm69SetMode(RF69_MODE_SYNTH);
    }
    rfm69SetMode(oldMode);
} // Tested OK!

void rfm69SetHighPower(void){
    spiWriteReg(REG_OCP, ISRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
    if (ISRFM69HW){ // turning ON
        spiWriteReg(REG_PALEVEL, (spiReadReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    }
    else{
        spiWriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | POWER_LEVEL); // enable P0 only
    }
} // Not tested


//put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void rfm69Sleep(void) {
    rfm69SetMode(RF69_MODE_SLEEP);
}

//set this node's address
void rfm69SetAddress(uint8_t addr){
    rfm69Stats.devId = addr;
    spiWriteReg(REG_NODEADRS, addr);
}

//set this node's network id
void rfm69SetNetwork(uint8_t networkID){
    spiWriteReg(REG_SYNCVALUE2, networkID);
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void rfm69SetPowerLevel(uint8_t powerLevel){
    rfm69Stats.powerLevel = (powerLevel > 31 ? 31 : powerLevel);
    if(ISRFM69HW){
        rfm69Stats.powerLevel /= 2;
    }
    spiWriteReg(REG_PALEVEL, (spiReadReg(REG_PALEVEL) & 0xE0) | rfm69Stats.powerLevel);
}

bool rfm69CanSend(void){
    if((rfm69Stats.mode == RF69_MODE_RX) && (rfm69Stats.payloadLen == 0) && (rfm69ReadRSSI(false) < CSMA_LIMIT)){ // if signal stronger than -100dBm is detected assume channel activity
        rfm69SetMode(RF69_MODE_STANDBY);
        return true;
    }
    return false;
}

void rfm69Send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK){
    spiWriteReg(REG_PACKETCONFIG2, (spiReadReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    uint32_t now = sysGetGTime();
    while((!rfm69CanSend() && sysGetGTime()) - (now < RF69_CSMA_LIMIT_MS)){
        rfm69ReceiveDone();
    }
    rfm69SendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

//ATOMIC_BLOCK(ATOMIC_FORCEON)
bool rfm69ReceiveDone(void){
    __disable_interrupt();
    if(rfm69Stats.mode == RF69_MODE_RX && rfm69Stats.payloadLen > 0){
        rfm69Sleep();
        __enable_interrupt();
        return true;
    }
    else if(rfm69Stats.mode == RF69_MODE_RX){                                   // already in RX no payload yet
        __enable_interrupt();                                                   // explicitly re-enable interrupts
        return false;
    }
    rfm69ReceiveBegin();
    __enable_interrupt();
    return false;
}

// get the received signal strength indicator (RSSI)
int16_t rfm69ReadRSSI(bool forceTrigger){
    int16_t rssi = 0;
    if(forceTrigger){
        // RSSI trigger not needed if DAGC is in continuous mode
        spiWriteReg(REG_RSSICONFIG, RF_RSSI_START);
        while((spiReadReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
    }
    rssi = -spiReadReg(REG_RSSIVALUE);
    rssi >>= 1;
    return rssi;
}

void rfm69InterruptHandler(void){
    uint8_t i = 0;
    uint8_t CTLbyte;
    if(rfm69Stats.mode == RF69_MODE_RX && (spiReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)){
        rfm69Sleep();
        spiSlaveEnable();
        spiTransmitByte(REG_FIFO & 0x7F);
        rfm69Stats.payloadLen = spiReceiveByte(0);
        rfm69Stats.payloadLen = rfm69Stats.payloadLen > 66 ? 66 : rfm69Stats.payloadLen; // precaution
        rfm69Stats.targetId = spiReceiveByte(0);         // TODO: is this really receiving anything?

        // match this node's address, or broadcast address or anything in promiscuous mode
        // address situation could receive packets that are malformed and don't fit this libraries extra fields
        if(!(rfm69Stats.promiscuousMode || (rfm69Stats.targetId == rfm69Stats.devId) || (rfm69Stats.targetId == RF69_BROADCAST_ADDR)) || (rfm69Stats.payloadLen < 3)){
            rfm69Stats.payloadLen = 0;
            spiSlaveDisable();
            rfm69ReceiveBegin();
          return;
        }

        rfm69Stats.dataLen = rfm69Stats.payloadLen - 3;
        rfm69Stats.senderId = spiReceiveByte(0);         // TODO: is this really receiving anything?
        CTLbyte = spiReceiveByte(0);                     // TODO: is this really receiving anything?

        rfm69Stats.ackReceived = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
        rfm69Stats.ackRequested = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag

        //interruptHook(CTLbyte);     // TWS: hook to derived class interrupt function      TODO: what is the impact of the absence of this interrupt hook

        for(i=0; i<rfm69Stats.dataLen; i++){                                                // TODO: transfer information using DMA
            rfm69Stats.data[i] = spiReceiveByte(0);
        }

        if(rfm69Stats.dataLen < RF69_MAX_DATA_LEN){
            rfm69Stats.data[rfm69Stats.dataLen] = '\n';                                        // add null at end of string
        }
        spiSlaveDisable();
        rfm69SetMode(RF69_MODE_RX);
        rfm69Stats.dataAvailable = true;
    }
    rfm69Stats.rssi = rfm69ReadRSSI(false);
    pinInterrupt = false;                                                                   // clear the pending iterrupt flag
}

void rfm69Promiscuous(bool state){
    rfm69Stats.promiscuousMode = state;
    // TODO: Evaluate the line below
    //writeReg(REG_PACKETCONFIG1, (readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

static void rfm69ReceiveBegin(void){
    rfm69Stats.dataLen = 0;
    rfm69Stats.senderId = 0;
    rfm69Stats.targetId = 0;
    rfm69Stats.payloadLen = 0;
    rfm69Stats.ackRequested = 0;
    rfm69Stats.ackReceived = 0;
    rfm69Stats.rssi = 0;
    rfm69Stats.dataAvailable = false;
    if(spiReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY){
        spiWriteReg(REG_PACKETCONFIG2, (spiReadReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);  // avoid RX deadlocks
    }
    spiWriteReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);                                               // set DIO0 to "PAYLOADREADY" in receive mode
    rfm69SetMode(RF69_MODE_RX);
}

static void rfm69SendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK){
    uint8_t CTLbyte = 0x00;                                                     // control byte
    uint8_t i;
    rfm69SetMode(RF69_MODE_STANDBY);                                            // turn off receiver to prevent reception while filling fifo
    while((spiReadReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);        // wait for ModeReady
    spiWriteReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00);                       // DIO0 is "Packet Sent"
    if(bufferSize > RF69_MAX_DATA_LEN){                                         // force buffer size to maximum allowed buffer length
        bufferSize = RF69_MAX_DATA_LEN;
    }

    if(sendACK){
        CTLbyte = RFM69_CTL_SENDACK;
    }
    else if(requestACK){
        CTLbyte = RFM69_CTL_REQACK;
    }

    // write to FIFO
    spiSlaveEnable();
    spiTransmitByte(REG_FIFO | 0x80);
    spiTransmitByte(bufferSize + 3);
    spiTransmitByte(toAddress);
    spiTransmitByte(rfm69Stats.devId);
    spiTransmitByte(CTLbyte);

    for(i=0; i<bufferSize; i++){
        spiTransmitByte(((uint8_t*) buffer)[i]);
    }
    spiSlaveDisable();

    // no need to wait for transmit mode to be ready since its handled by the radio
    rfm69SetMode(RF69_MODE_TX);
    uint32_t txStart = sysGetGTime();
    while((gpioReadPinStatus(RF69_IRQ_PORT, RF69_IRQ_PIN) == 0) && (sysGetGTime() - txStart < RF69_TX_LIMIT_MS));      // wait for DIO0 to turn HIGH signalling transmission finish
    //while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady       // TODO: check this command and then remove this
    rfm69SetMode(RF69_MODE_STANDBY);
}

// Temporarily made global, evaluate and move back to static if necessary
void rfm69SetMode(uint8_t newMode){
    if(newMode == rfm69Stats.mode){
        return;
    }
    switch (newMode) {
        case RF69_MODE_TX:
            spiWriteReg(REG_OPMODE, (spiReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            if(ISRFM69HW){
                rfm69SetHighPowerRegs(true);
            }
            break;
        case RF69_MODE_RX:
            spiWriteReg(REG_OPMODE, (spiReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            if(ISRFM69HW){
                rfm69SetHighPowerRegs(false);
            }
            break;
        case RF69_MODE_SYNTH:
            spiWriteReg(REG_OPMODE, (spiReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69_MODE_STANDBY:
            spiWriteReg(REG_OPMODE, (spiReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69_MODE_SLEEP:
            spiWriteReg(REG_OPMODE, (spiReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default:
            return;
        }
    while((rfm69Stats.mode == RF69_MODE_SLEEP) && (((spiReadReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY)) == 0x00));
    rfm69Stats.mode = newMode;
} // Not tested

static void rfm69SetupRxInterrupt(uint8_t port, uint16_t pin){
    gpioSetAsInputPin(port, pin);
    gpioSelectPinInterruptEdge(port, pin, GPIO_LOW_TO_HIGH_TRANSITION);
    gpioClearPinInterrupt(port, pin);
    gpioEnablePinInterrupt(port, pin);
}

// internal function
static void rfm69SetHighPowerRegs(bool onOff){
    spiWriteReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
    spiWriteReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
} // Not tested

