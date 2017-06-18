/*
 * rfm69.h
 *
 *  Created on: 19.05.2017
 *      Author: marcio
 */

#ifndef RFM69CW_RFM69_H_
#define RFM69CW_RFM69_H_

// Includes
#include "peripheralAccess.h"
#include "rfm69registers.h"

// Defines
#define CSMA_LIMIT              -90     // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0       // XTAL OFF
#define RF69_MODE_STANDBY       1       // XTAL ON
#define RF69_MODE_SYNTH         2       // PLL ON
#define RF69_MODE_RX            3       // RX MODE
#define RF69_MODE_TX            4       // TX MODE
#define RF69_MODE_INIT          255

// available frequency bands
#define RF69_315MHZ            31       // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90         // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625         // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40


// Radio Timing
#define RFM69_TIMEOUT       500
#define POWER_LEVEL         31
#define RF69_MAX_DATA_LEN   61
#define RF69_IRQ_PIN        GPIO_PIN6
#define RF69_IRQ_PORT       GPIO_PORT_P2

// Radio model
#define ISRFM69HW           false

// Custom data types
typedef struct rfm69Status{
    uint8_t data[RF69_MAX_DATA_LEN];
    uint8_t dataLen;
    uint8_t senderId;
    uint8_t targetId;
    uint8_t payloadLen;
    uint8_t ackRequested;
    uint8_t ackReceived;
    int16_t rssi;
    uint8_t mode;
    uint8_t devId;
    uint8_t powerLevel;
    uint8_t promiscuousMode;
    bool dataAvailable;
} rfm69Status_t;

// Function prototypes
extern void rfm69Sleep(void);
extern void rfm69SetMode(uint8_t newMode);
extern uint8_t rfm69Initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
extern void rfm69Encrypt(const uint8_t* key);
extern uint32_t rfm69GetFrequency(void);
extern void rfm69SetFrequency(uint32_t freqHz);
extern void rfm69SetHighPower(void);
extern void rfm69SetPowerLevel(uint8_t powerLevel);
extern int16_t rfm69ReadRSSI(bool forceTrigger);
extern void rfm69InterruptHandler(void);
extern bool rfm69ReceiveDone(void);
extern void rfm69Promiscuous(bool state);

// Global shared variables
extern volatile uint8_t dataReceived;
extern rfm69Status_t rfm69Stats;

#endif /* RFM69CW_RFM69_H_ */
