#pragma once

#define CHG_STAT_NOT_CHG    0b00
#define CHG_STAT_PRE_CHG    0b01
#define CHG_STAT_FAST_CHG   0b10
#define CHG_STAT_DONE       0b11

namespace PMS2 {
    bool isConnected();
    float getVer();
    u16 getChargeCurrent();
    u16 getTermCurrent();
    u16 getPreChargeCurrent();
    u16 getChargeVoltage();
    u8 getChargeStatus();
    float getSOC();
    u8 getSOCRaw();
    float getVCell();
    u16 getVCellRaw();
    void setChargeCurrent(u16 v);
    void setTermCurrent(u16 v);
    void setPreChargeCurrent(u16 v);
    void setChargeVoltage(u16 v);
    void enableShippingMode();
    void flashConfig();
    float getNTC();
    u8 getFanSpeed();
    void setFanSpeed(u8 speed);
};
