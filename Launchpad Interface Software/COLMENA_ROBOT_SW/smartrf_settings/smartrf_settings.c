//*********************************************************************************
// Generated by SmartRF Studio version 2.10.0 (build #88)
// Compatible with SimpleLink SDK version: CC2640R2 SDK 2.20.xx.xx
// Device: CC2640R2F Rev. 1.0
// 
//*********************************************************************************


//*********************************************************************************
// Parameter summary
// Address: off
// Address0: 0xAA
// Address1: 0xBB
// Frequency: 2440.00000 MHz
// Data Format: Serial mode disable
// Deviation: 125.000 kHz
// Packet Length Config: Variable
// Max Packet Length: 255
// Packet Length: 20
// Packet Data: 255 
// RX Filter BW: 530 kHz
// Symbol Rate: 250.00000 kBaud
// Sync Word Length: 32 Bits
// TX Power: 5 dBm
// Whitening: No whitening

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_prop.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_genfsk.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_genfsk.h)
#include "smartrf_settings.h"


// TI-RTOS RF Mode Object
RF_Mode RF_prop =
{
    .rfMode = RF_MODE_PROPRIETARY_2_4,
    .cpePatchFxn = &rf_patch_cpe_prop,
    .mcePatchFxn = &rf_patch_mce_genfsk,
    .rfePatchFxn = &rf_patch_rfe_genfsk,
};

// Overrides for CMD_PROP_RADIO_SETUP
uint32_t pOverrides[] =
{
    // override_use_patch_prop_genfsk.xml
    // PHY: Use MCE RAM patch, RFE RAM patch
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    // override_synth_prop_2440_div2.xml
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure faster calibration
    HW32_ARRAY_OVERRIDE(0x4004, 1),
    // Synth: Configure faster calibration
    (uint32_t)0x1C0C0618,
    // Synth: Configure faster calibration
    (uint32_t)0xC00401A1,
    // Synth: Configure faster calibration
    (uint32_t)0x21010101,
    // Synth: Configure faster calibration
    (uint32_t)0xC0040141,
    // Synth: Configure faster calibration
    (uint32_t)0x00214AD3,
    // Synth: Decrease synth programming time-out by 90 us (0x0298 RAT ticks = 166 us)
    (uint32_t)0x02980243,
    // Rx: Set LNA IB offset used for automatic software compensation to 0
    (uint32_t)0x00008883,
    // Rx: Set LNA IB trim value based on the selected defaultPhy.mainMode setting.
    ADI_HALFREG_OVERRIDE(0,4,0xF,0xF),
    // Rx: Set RSSI offset to adjust reported RSSI by -1 dB (default: 0)
    (uint32_t)0x000188A3,
    // override_phy_tx_pa_ramp_gfsk_pa_ramp_12us_agc_reflevel_0x22.xml
    // Tx: Configure PA ramping setting (0x3F). Rx: Set AGC reference level to 0x22.
    HW_REG_OVERRIDE(0x6088,0x3F22),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x0413),
    (uint32_t)0xFFFFFFFF,
};

// CMD_PROP_RADIO_SETUP
// Proprietary Mode Radio Setup Command
rfc_CMD_PROP_RADIO_SETUP_t RF_cmdPropRadioDivSetup =
{
    .commandNo = 0x3806,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x1,
    .modulation.deviation = 0xC8,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x8000,
    .symbolRate.decimMode = 0x0,
    .rxBw = 0x05,
    .preamConf.nPreamBytes = 0x4,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x20,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x1,
    .formatConf.fecMode = 0x0,
    .formatConf.whitenMode = 0x0,
    .config.frontEndMode = 0x0,
    .config.biasMode = 0x0,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0x9330,
    //.txPower = 0x0CC7,
    .pRegOverride = pOverrides,
};

// CMD_FS
// Frequency Synthesizer Programming Command
rfc_CMD_FS_t RF_cmdFs =
{
    .commandNo = 0x0803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .frequency = 0x0988,
    .fractFreq = 0x0000,
    .synthConf.bTxMode = 0x0,
    .synthConf.refFreq = 0x0,
    .__dummy0 = 0x00,
    .__dummy1 = 0x00,
    .__dummy2 = 0x00,
    .__dummy3 = 0x0000,
};

// CMD_PROP_TX
// Proprietary Mode Transmit Command
rfc_CMD_PROP_TX_t RF_cmdPropTx =
{
    .commandNo = 0x3801,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bUseCrc = 0x1,
    .pktConf.bVarLen = 0x1,
    .pktLen = 0x14, // SET APPLICATION PAYLOAD LENGTH
    .syncWord = 0x930B51DE,
    .pPkt = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

// CMD_PROP_RX
// Proprietary Mode Receive Command
rfc_CMD_PROP_RX_t RF_cmdPropRx =
{
    .commandNo = 0x3802,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bRepeatOk = 0x0,
    .pktConf.bRepeatNok = 0x0,
    .pktConf.bUseCrc = 0x1,
    .pktConf.bVarLen = 0x1,
    .pktConf.bChkAddress = 0x0,
    .pktConf.endType = 0x0,
    .pktConf.filterOp = 0x0,
    .rxConf.bAutoFlushIgnored = 0x0,
    .rxConf.bAutoFlushCrcErr = 0x0,
    .rxConf.bIncludeHdr = 0x1,
    .rxConf.bIncludeCrc = 0x0,
    .rxConf.bAppendRssi = 0x0,
    .rxConf.bAppendTimestamp = 0x0,
    .rxConf.bAppendStatus = 0x1,
    .syncWord = 0x930B51DE,
    .maxPktLen = 0xFF, // MAKE SURE DATA ENTRY IS LARGE ENOUGH
    .address0 = 0xAA,
    .address1 = 0xBB,
    .endTrigger.triggerType = 0x1,
    .endTrigger.bEnaCmd = 0x0,
    .endTrigger.triggerNo = 0x0,
    .endTrigger.pastTrig = 0x0,
    .endTime = 0x00000000,
    .pQueue = 0, // INSERT APPLICABLE POINTER: (dataQueue_t*)&xxx
    .pOutput = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

// CMD_TX_TEST
// Proprietary Mode Transmit Test Command 
rfc_CMD_TX_TEST_t RF_cmdTxTest =
{
    .commandNo = 0x0808,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .config.bUseCw = 0x0,
    .config.bFsOff = 0x1,
    .config.whitenMode = 0x2,
    .__dummy0 = 0x00,
    .txWord = 0xABCD,
    .__dummy1 = 0x00,
    .endTrigger.triggerType = 0x1,
    .endTrigger.bEnaCmd = 0x0,
    .endTrigger.triggerNo = 0x0,
    .endTrigger.pastTrig = 0x0,
    .syncWord = 0x930B51DE,
    .endTime = 0x00000000,
};

rfc_CMD_FS_t *RF_pcmdFs = &RF_cmdFs;
