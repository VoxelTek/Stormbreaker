#include <string>
#include <string.h>
#include <malloc.h>
#include <gccore.h>
#include <math.h>
#include <ogc/lwp_watchdog.h>
#include <ogc/machine/processor.h>
#include <ogc/lwp_threads.h>
#include "i2c.h"
#include "stormbreaker.h"
#include "main.h"

#define STORM_ADDR (0x20 << 1)

#define TMP1075_ADDR (0x49 << 1)
#define CMD_NTC                 0x00

#define PGM_BLOCK_SIZE  0x80UL      //bytes
#define PMS2_PAYLOAD_ADDR 0x2000UL

#define CMD_GETVER              0x00
//#define CMD_BOOTBL              0x01
#define CMD_FLASHCONFIG         0x02

#define CMD_FAN                 0x04
//#define CMD_PID_KP              0x05
//#define CMD_PID_KI              0x06
//#define CMD_PID_KD              0x07
//#define CMD_PID_TARGET          0x08
//#define CMD_PD_PROFILES         0x09
//#define CMD_CONF0               0x0A
#define CMD_SHIPPINGMODE        0x0B
//#define CMD_FANRANGE            0x0C
//#define CMD_LEDINTENSITY        0x0D
//#define CMD_POT                 0x0E

#define CMD_CHARGECURRENT       0x10
#define CMD_TERMCURRENT         0x12
#define CMD_PRECHARGECURRENT    0x14
#define CMD_CHARGEVOLTAGE       0x16
//#define CMD_TREG                0x14
#define CMD_CHARGESTATUS        0x18

//#define CMD_MAXRECONFIGURE      0x20
//#define CMD_BATDESIGNCAP        0x21
//#define CMD_MAXVEMPTY           0x22
//#define CMD_MAXCYCLES           0x23
#define CMD_BATSOC              0x24
//#define CMD_BATCURRENT          0x25
#define CMD_BATVOLTAGE          0x26
//#define CMD_TTE                 0x27
//#define CMD_TTF                 0x28

//#define CMD_WRITEFLASH          0xF0
//#define CMD_READFLASH           0xF1
//#define CMD_BOOTPAYLOAD         0xF2

#define ERR_NONE                0x00
#define ERR_LOCKED              0xFF
#define ERR_WRONGARG            0xFE
#define ERR_WRONGMODE           0xFD
#define ERR_WRONGCRC            0xFC
#define ERR_WRONGADDR           0xFB

#define PMS_FLASH_RETRIES       20

#define PMS_POLLUPDATE_TIMEOUT  200

#define STACKSIZE 8192

extern "C" {
    extern void udelay(int us);
}

namespace STORMBREAKER {
    static u8 curPMSAddress = STORM_ADDR;

    static inline void swap32(u8* data) {
        u8 temp;
        temp = data[0];
        data[0] = data[3];
        data[3] = temp;
        temp = data[1];
        data[1] = data[2];
        data[2] = temp;
    }

    bool isConnected() {
        if (isRunningOnDolphin()) {
            return false;
        }

        u32 level;
        u8 i2cRet;
        static bool firstTime = true;
        static bool ret = false;

        if (firstTime) {
            firstTime = false;
            _CPU_ISR_Disable(level);
            ret = true;
            i2c_start();
            i2cRet = i2c_sendByte(STORM_ADDR);
            i2c_stop();
            if (!i2cRet) {
                ret = false;
            }
            _CPU_ISR_Restore(level);
        }
        return ret;
    }

    float getVer() {
        u8 error;
        float ver = i2c_read8(curPMSAddress, CMD_GETVER, &error) / 10;
        return ver;
    }
    /*
    void getPDProfiles(u32* profiles) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        i2c_readBuffer(curPMSAddress, CMD_PD_PROFILES, &error, (u8*)profiles, 11 * sizeof(u32));
        for (int i = 0; i < 11; i++)
            swap32((u8*)&profiles[i]);
    }

    u8 getConf0() {
        u8 error;
        static u64 lastTime = 0;
        static u8 ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        ret = i2c_read8(curPMSAddress, CMD_CONF0, &error);
        return ret;
    }

    u16 getPot() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        ret = i2c_read16(curPMSAddress, CMD_POT, &error);
        return ret;
    }
    */
    u16 getChargeCurrent() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_CHARGECURRENT, &error);
        ret += i2c_read8(curPMSAddress, CMD_CHARGECURRENT + 1, &error) << 8;
        return ret;
    }

    u16 getTermCurrent() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_TERMCURRENT, &error);
        ret += i2c_read8(curPMSAddress, CMD_TERMCURRENT + 1, &error) << 8;
        return ret;
    }

    u16 getPreChargeCurrent() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_PRECHARGECURRENT, &error);
        ret += i2c_read8(curPMSAddress, CMD_PRECHARGECURRENT + 1, &error) << 8;
        return ret;
    }

    u16 getChargeVoltage() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_CHARGEVOLTAGE, &error);
        ret += i2c_read8(curPMSAddress, CMD_CHARGEVOLTAGE + 1, &error) << 8;
        return ret;
    }
    /*
    u8 getTREG() {
        u8 error;
        static u64 lastTime = 0;
        static u8 ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        ret = i2c_read8(curPMSAddress, CMD_TREG, &error);
        return ret;
    }
    */

    u8 getChargeStatus() {
        u8 error;
        static u64 lastTime = 0;
        static u8 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_CHARGESTATUS, &error);
        return ret;
    }
    /*
    u16 getBatDesignCapacity() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return ret;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        ret = i2c_read16(curPMSAddress, CMD_BATDESIGNCAP, &error);
        return ret;
    }
    */
    float getSOC() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0.0f;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        u8 soc = i2c_read8(curPMSAddress, CMD_BATSOC, &error);
        ret = (float)soc / 0xff;
        return ret;
    }

    u8 getSOCRaw() {
        u8 error;
        static u64 lastTime = 0;
        static u8 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        ret = i2c_read8(curPMSAddress, CMD_BATSOC, &error);
        return ret;
    }


    float getVCell() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0.0f;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }

        lastTime = gettime();

        u16 vcell = i2c_read8(curPMSAddress, CMD_BATVOLTAGE, &error);
        vcell += i2c_read8(curPMSAddress, CMD_BATVOLTAGE + 1, &error) << 8;
        ret = (float)(vcell / 1000.0f);
        return ret;
    }
    /*
    float getCurrent() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0.0f;

        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return ret;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        s16 current = (s16)i2c_read16(curPMSAddress, CMD_BATCURRENT, &error);
        ret = 312.5f * (float)current / 1000.0f;
        return ret;
    }
    */

    u16 getVCellRaw() {
        u8 error;
        static u64 lastTime = 0;
        static u16 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }

        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_BATVOLTAGE, &error);
        ret += i2c_read8(curPMSAddress, CMD_BATVOLTAGE + 1, &error) << 8;
        return ret;
    }
    /*
    s16 getCurrentRaw() {
        u8 error;
        static u64 lastTime = 0;
        static s16 ret = 0;

        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return ret;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        ret = (s16)i2c_read16(curPMSAddress, CMD_BATCURRENT, &error);
        return ret;
    }

    u32 getTTE() {
        u8 error;
        static u64 lastTime = 0;
        static u32 ret = 0;

        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return ret;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        u16 tte  = i2c_read16(curPMSAddress, CMD_TTE, &error);
        ret = (u32)round(5.625f * (float)tte / 60.0f);
        return ret;
    }

    u32 getTTF() {
        u8 error;
        static u64 lastTime = 0;
        static u32 ret = 0;

        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return ret;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        u16 ttf  = i2c_read16(curPMSAddress, CMD_TTF, &error);
        ret = (u32)round(5.625f * (float)ttf / 60.0f);
        return ret;
    }

    void setConf0(u8 v) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        i2c_write8(curPMSAddress, CMD_CONF0, v, &error);
    }
    */
    void setChargeCurrent(u16 v) {
        u8 error;

        i2c_write8(curPMSAddress, CMD_CHARGECURRENT, (v & 0x00FF), &error);
        i2c_write8(curPMSAddress, CMD_CHARGECURRENT + 1, (v & 0xFF00) >> 8, &error);
    }

    void setTermCurrent(u16 v) {
        u8 error;

        i2c_write8(curPMSAddress, CMD_TERMCURRENT, (v & 0x00FF), &error);
        i2c_write8(curPMSAddress, CMD_TERMCURRENT + 1, (v & 0xFF00) >> 8, &error);
    }

    void setPreChargeCurrent(u16 v) {
        u8 error;

        i2c_write8(curPMSAddress, CMD_PRECHARGECURRENT, (v & 0x00FF), &error);
        i2c_write8(curPMSAddress, CMD_PRECHARGECURRENT + 1, (v & 0xFF00) >> 8, &error);
    }

    void setChargeVoltage(u16 v) {
        u8 error;

        i2c_write8(curPMSAddress, CMD_CHARGEVOLTAGE, (v & 0x00FF), &error);
        i2c_write8(curPMSAddress, CMD_CHARGEVOLTAGE + 1, (v & 0xFF00) >> 8, &error);
    }
    /*
    void setTREG(u8 v) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        i2c_write8(curPMSAddress, CMD_TREG, v, &error);
    }

    void setBatDesignCapacity(u16 v) {
        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        i2c_write16(curPMSAddress, CMD_BATDESIGNCAP, v, &error);
    }
    */
    void enableShippingMode() {
        u8 error;

        i2c_write8(curPMSAddress, CMD_SHIPPINGMODE, 0x00, &error);
    }

    void flashConfig() {
        u8 error;

        i2c_write8(curPMSAddress, CMD_FLASHCONFIG, 0x00, &error);
    }
    /*

    void reconfigureMAX() {
        //This command is not available on PMS2 lite
        if (curPMSAddress == PMS2L_ADDR)
            return;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        u8 tempBuffer[4] = {0x50, 0x4D , 0x53, 0x32};
        i2c_writeBuffer(curPMSAddress, CMD_MAXRECONFIGURE, tempBuffer, 4, &error);
    }

    #define NTC_RBOT 2200.0f
    #define NTC_R25 10000.0f
    #define NTC_B 3988.0f
    #define NTC_T0 298.15f

    float NTCToCelsius(u16 ntc) {
        float VNTC = 2.048f * (float)ntc / 4095.0f;
        //VNTC = 1.8 * NTC_RBOT / (RNTC + NTC_RBOT)
        float RNTC = 1.8f * NTC_RBOT / VNTC - NTC_RBOT;
        //float RNTC  = NTC_RBOT * (4095.0f / (float)ntc - 1.0f);
        float Rinf = NTC_R25 * exp(-NTC_B / NTC_T0);
        float ret = NTC_B / log(RNTC / Rinf) - 273.15;
        return ret;
    }

    u16 CelsiusToNTC(float temperature) {
        float RNTC = NTC_R25 * exp(NTC_B * (1 / (temperature + 273.15) - 1 / NTC_T0));
        float VNTC = 1.8f * NTC_RBOT / (RNTC + NTC_RBOT);
        u16 ntc = (u16)(VNTC * 4095.0f / 2.048);
        return ntc;
    }
    */
    float getNTC() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0.0f;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        u16 tmp = i2c_read16(TMP1075_ADDR, CMD_NTC, &error) >> 4;
        ret = (tmp & 0b011111111111) / 16;
        if (tmp & 0b100000000000) {
            ret -= 128.0f;
        }
        return ret;
    }

    u8 getFanSpeed() {
        u8 error;
        static u64 lastTime = 0;
        static u8 ret = 0;

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();

        ret = i2c_read8(curPMSAddress, CMD_FAN, &error);

        return ret;
    }

    void setFanSpeed(u8 speed) {
        u8 error;
        i2c_write8(curPMSAddress, CMD_FAN, speed, &error);
    }
    /*

    void freeFan() {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        u8 tempBuffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
        i2c_writeBuffer(curPMSAddress, CMD_FAN, tempBuffer, 5, &error);
    }

    float getFanPIDkP() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        i2c_readBuffer(curPMSAddress, CMD_PID_KP, &error, (u8*)&ret, 4);
        swap32((u8*)&ret);

        return ret;
    }

    void setFanPIDkP(float kP) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        swap32((u8*)&kP);
        i2c_writeBuffer(curPMSAddress, CMD_PID_KP, (u8*)&kP, 4, &error);
    }

    float getFanPIDkI() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        i2c_readBuffer(curPMSAddress, CMD_PID_KI, &error, (u8*)&ret, 4);
        swap32((u8*)&ret);

        return ret;
    }

    void setFanPIDkI(float kI) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        swap32((u8*)&kI);
        i2c_writeBuffer(curPMSAddress, CMD_PID_KI, (u8*)&kI, 4, &error);
    }

    float getFanPIDkD() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        i2c_readBuffer(curPMSAddress, CMD_PID_KD, &error, (u8*)&ret, 4);
        swap32((u8*)&ret);

        return ret;
    }

    void setFanPIDkD(float kD) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        swap32((u8*)&kD);
        i2c_writeBuffer(curPMSAddress, CMD_PID_KD, (u8*)&kD, 4, &error);
    }

    float getFanPIDTarget() {
        u8 error;
        static u64 lastTime = 0;
        static float ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        i2c_readBuffer(curPMSAddress, CMD_PID_TARGET, &error, (u8*)&ret, 4);
        swap32((u8*)&ret);

        return ret;
    }

    void setFanPIDTarget(float target) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        swap32((u8*)&target);
        i2c_writeBuffer(curPMSAddress, CMD_PID_TARGET, (u8*)&target, 4, &error);
    }

    fanRange_t getFanRange() {
        u8 error;
        static u64 lastTime = 0;
        static fanRange_t ret = {0, 255};

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        i2c_readBuffer(curPMSAddress, CMD_FANRANGE, &error, (u8*)&ret, sizeof(fanRange_t));

        return ret;
    }

    void setFanRange(fanRange_t range) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        i2c_writeBuffer(curPMSAddress, CMD_FANRANGE, (u8*)&range, sizeof(fanRange_t), &error);
    }
    /*

    uint8_t getLEDIntensity() {
        u8 error;
        static u64 lastTime = 0;
        static uint8_t ret = 0;

        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return ret;
        }
        LWP_MutexUnlock(updateMutex);

        if ((diff_msec(gettime(), lastTime) < PMS_POLLUPDATE_TIMEOUT) && lastTime) {
            return ret;
        }
        lastTime = gettime();
        i2c_readBuffer(curPMSAddress, CMD_LEDINTENSITY, &error, (u8*)&ret, 1);

        return ret;
    }

    void setLEDIntensity(uint8_t intensity) {
        if (!updateMutex)
            LWP_MutexInit(&updateMutex, false);
        LWP_MutexLock(updateMutex);
        if (updating) {
            LWP_MutexUnlock(updateMutex);
            return;
        }
        LWP_MutexUnlock(updateMutex);

        u8 error;
        i2c_writeBuffer(curPMSAddress, CMD_LEDINTENSITY, &intensity, 1, &error);
    }
    */
};
