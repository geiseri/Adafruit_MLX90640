/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
 //#include <MLX90640_I2C_Driver.h>
#include "../headers/MLX90640_API.h"
#include "../Adafruit_MLX90640.h"
#include <math.h>
#include <cmath>
#include <limits>

#define MLX90640_NO_ERROR 0
#define MLX90640_I2C_NACK_ERROR 1
#define MLX90640_I2C_WRITE_ERROR 2
#define MLX90640_BROKEN_PIXELS_NUM_ERROR 3
#define MLX90640_OUTLIER_PIXELS_NUM_ERROR 4
#define MLX90640_BAD_PIXELS_NUM_ERROR 5
#define MLX90640_ADJACENT_BAD_PIXELS_ERROR 6
#define MLX90640_EEPROM_DATA_ERROR 7
#define MLX90640_FRAME_DATA_ERROR 8
#define MLX90640_MEAS_TRIGGER_ERROR 9

#define BIT_MASK(x) (1UL << (x))
#define REG_MASK(sbit,nbits) ~((~(~0UL << (nbits))) << (sbit))

#define MLX90640_EEPROM_START_ADDRESS 0x2400
#define MLX90640_EEPROM_DUMP_NUM 832
#define MLX90640_PIXEL_DATA_START_ADDRESS 0x0400
#define MLX90640_PIXEL_NUM 768
#define MLX90640_LINE_NUM 24
#define MLX90640_COLUMN_NUM 32
#define MLX90640_LINE_SIZE 32
#define MLX90640_COLUMN_SIZE 24
#define MLX90640_AUX_DATA_START_ADDRESS 0x0700
#define MLX90640_AUX_NUM 64
#define MLX90640_STATUS_REG 0x8000
#define MLX90640_INIT_STATUS_VALUE 0x0030
#define MLX90640_STAT_FRAME_MASK BIT_MASK(0) 
#define MLX90640_GET_FRAME(reg_value) (reg_value & MLX90640_STAT_FRAME_MASK)
#define MLX90640_STAT_DATA_READY_MASK BIT_MASK(3) 
#define MLX90640_GET_DATA_READY(reg_value) (reg_value & MLX90640_STAT_DATA_READY_MASK)

#define MLX90640_CTRL_REG 0x800D
#define MLX90640_CTRL_TRIG_READY_MASK BIT_MASK(15) 
#define MLX90640_CTRL_REFRESH_SHIFT 7
#define MLX90640_CTRL_REFRESH_MASK REG_MASK(MLX90640_CTRL_REFRESH_SHIFT,3)
#define MLX90640_CTRL_RESOLUTION_SHIFT 10
#define MLX90640_CTRL_RESOLUTION_MASK REG_MASK(MLX90640_CTRL_RESOLUTION_SHIFT,2)
#define MLX90640_CTRL_MEAS_MODE_SHIFT 12
#define MLX90640_CTRL_MEAS_MODE_MASK BIT_MASK(12)

#define MLX90640_MS_BYTE_SHIFT 8
#define MLX90640_MS_BYTE_MASK 0xFF00
#define MLX90640_LS_BYTE_MASK 0x00FF
#define MLX90640_MS_BYTE(reg16) ((reg16 & MLX90640_MS_BYTE_MASK) >> MLX90640_MS_BYTE_SHIFT)
#define MLX90640_LS_BYTE(reg16) (reg16 & MLX90640_LS_BYTE_MASK)
#define MLX90640_MSBITS_6_MASK 0xFC00
#define MLX90640_LSBITS_10_MASK 0x03FF
#define MLX90640_NIBBLE1_MASK 0x000F
#define MLX90640_NIBBLE2_MASK 0x00F0
#define MLX90640_NIBBLE3_MASK 0x0F00
#define MLX90640_NIBBLE4_MASK 0xF000
#define MLX90640_NIBBLE1(reg16) ((reg16 & MLX90640_NIBBLE1_MASK))
#define MLX90640_NIBBLE2(reg16) ((reg16 & MLX90640_NIBBLE2_MASK) >> 4)
#define MLX90640_NIBBLE3(reg16) ((reg16 & MLX90640_NIBBLE3_MASK) >> 8)
#define MLX90640_NIBBLE4(reg16) ((reg16 & MLX90640_NIBBLE4_MASK) >> 12)



//static float scratchData[768];

static void ExtractVDDParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractPTATParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractGainParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractTgcParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractResolutionParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractKsTaParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractKsToParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractAlphaParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractOffsetParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractKtaPixelParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractKvPixelParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractCPParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static void ExtractCILCParameters(uint16_t* eeData, paramsMLX90640* mlx90640);
static int ExtractDeviatingPixels(uint16_t* eeData, paramsMLX90640* mlx90640);
static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);
static float GetMedian(float* values, int n);
static int IsPixelBad(uint16_t pixel, paramsMLX90640* params);
static int ValidateFrameData(uint16_t* frameData);
static int ValidateAuxData(uint16_t* auxData);





int Adafruit_MLX90640::MLX90640_DumpEE(uint8_t slaveAddr, uint16_t* eeData) {
    return MLX90640_I2CRead(slaveAddr, MLX90640_EEPROM_START_ADDRESS, MLX90640_EEPROM_DUMP_NUM, eeData);
}

int Adafruit_MLX90640::MLX90640_SynchFrame(uint8_t slaveAddr) {
    uint16_t dataReady = 0;
    uint16_t statusRegister;
    int error = 1;

    error = MLX90640_I2CWrite(slaveAddr, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
    if (error == -MLX90640_I2C_NACK_ERROR) {
        return error;
    }

    while (dataReady == 0) {
        error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister);
        if (error != MLX90640_NO_ERROR) {
            return error;
        }
        //dataReady = statusRegister & 0x0008;
        dataReady = MLX90640_GET_DATA_READY(statusRegister);
    }

    return MLX90640_NO_ERROR;
}

int Adafruit_MLX90640::MLX90640_TriggerMeasurement(uint8_t slaveAddr) {
    int error = 1;
    uint16_t ctrlReg;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &ctrlReg);

    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    ctrlReg |= MLX90640_CTRL_TRIG_READY_MASK;
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, ctrlReg);

    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = MLX90640_I2CGeneralReset();

    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &ctrlReg);

    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    if ((ctrlReg & MLX90640_CTRL_TRIG_READY_MASK) != 0) {
        return -MLX90640_MEAS_TRIGGER_ERROR;
    }

    return MLX90640_NO_ERROR;
}


int Adafruit_MLX90640::MLX90640_GetFramePage(uint8_t slaveAddr, uint16_t& statusRegister) {
    uint16_t dataReady = 0;
    uint16_t controlRegister1;
    int error = 1;
    uint16_t data[64];
    uint8_t cnt = 0;


    error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }
    dataReady = MLX90640_GET_DATA_READY(statusRegister);
    if (dataReady == 0) {
        return MLX90640_NO_ERROR;
    }

    error = MLX90640_I2CWrite(slaveAddr, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
    if (error == -MLX90640_I2C_NACK_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM, mlx90640Frame_.data());
    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM, data);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    mlx90640Frame_[832] = controlRegister1;
    mlx90640Frame_[833] = MLX90640_GET_FRAME(statusRegister);

    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = ValidateAuxData(data);
    if (error == MLX90640_NO_ERROR) {
        for (cnt = 0; cnt < MLX90640_AUX_NUM; cnt++) {
            mlx90640Frame_[cnt + MLX90640_PIXEL_NUM] = data[cnt];
        }
    }

    error = ValidateFrameData(mlx90640Frame_.data());
    if (error != MLX90640_NO_ERROR) {
        return error;
    }
    return MLX90640_NO_ERROR;
}

int Adafruit_MLX90640::MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t* frameData) {
    uint16_t dataReady = 0;
    uint16_t controlRegister1;
    uint16_t statusRegister;
    int error = 1;
    uint16_t data[64];
    uint8_t cnt = 0;

    while (dataReady == 0) {
        error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister);
        if (error != MLX90640_NO_ERROR) {
            return error;
        }
        //dataReady = statusRegister & 0x0008;
        dataReady = MLX90640_GET_DATA_READY(statusRegister);
    }

    error = MLX90640_I2CWrite(slaveAddr, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
    if (error == -MLX90640_I2C_NACK_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM, frameData);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM, data);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    frameData[832] = controlRegister1;
    //frameData[833] = statusRegister & 0x0001;
    frameData[833] = MLX90640_GET_FRAME(statusRegister);

    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    error = ValidateAuxData(data);
    if (error == MLX90640_NO_ERROR) {
        for (cnt = 0; cnt < MLX90640_AUX_NUM; cnt++) {
            frameData[cnt + MLX90640_PIXEL_NUM] = data[cnt];
        }
    }

    error = ValidateFrameData(frameData);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }

    return frameData[833];
}

static int ValidateFrameData(uint16_t* frameData) {
    uint8_t line = 0;

    for (int i = 0; i < MLX90640_PIXEL_NUM; i += MLX90640_LINE_SIZE) {
        if ((frameData[i] == 0x7FFF) && (line % 2 == frameData[833])) return -MLX90640_FRAME_DATA_ERROR;
        line = line + 1;
    }

    return MLX90640_NO_ERROR;
}

static int ValidateAuxData(uint16_t* auxData) {

    if (auxData[0] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;

    for (int i = 8; i < 19; i++) {
        if (auxData[i] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;
    }

    for (int i = 20; i < 23; i++) {
        if (auxData[i] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;
    }

    for (int i = 24; i < 33; i++) {
        if (auxData[i] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;
    }

    for (int i = 40; i < 51; i++) {
        if (auxData[i] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;
    }

    for (int i = 52; i < 55; i++) {
        if (auxData[i] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;
    }

    for (int i = 56; i < 64; i++) {
        if (auxData[i] == 0x7FFF) return -MLX90640_FRAME_DATA_ERROR;
    }

    return MLX90640_NO_ERROR;

}

int Adafruit_MLX90640::MLX90640_ExtractParameters(uint16_t* eeData) {
    int error = 0;

    ExtractVDDParameters(eeData, &_params);
    ExtractPTATParameters(eeData, &_params);
    ExtractGainParameters(eeData, &_params);
    ExtractTgcParameters(eeData, &_params);
    ExtractResolutionParameters(eeData, &_params);
    ExtractKsTaParameters(eeData, &_params);
    ExtractKsToParameters(eeData, &_params);
    ExtractCPParameters(eeData, &_params);
    ExtractAlphaParameters(eeData, &_params);
    ExtractOffsetParameters(eeData, &_params);
    ExtractKtaPixelParameters(eeData, &_params);
    ExtractKvPixelParameters(eeData, &_params);
    ExtractCILCParameters(eeData, &_params);
    error = ExtractDeviatingPixels(eeData, &_params);

    return error;

}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution) {
    uint16_t controlRegister1;
    uint16_t value;
    int error;

    //value = (resolution & 0x03) << 10;
    value = ((uint16_t) resolution << MLX90640_CTRL_RESOLUTION_SHIFT);
    value &= ~MLX90640_CTRL_RESOLUTION_MASK;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);

    if (error == MLX90640_NO_ERROR) {
        value = (controlRegister1 & MLX90640_CTRL_RESOLUTION_MASK) | value;
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }

    return error;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_GetCurResolution(uint8_t slaveAddr) {
    uint16_t controlRegister1;
    int resolutionRAM;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }
    resolutionRAM = (controlRegister1 & ~MLX90640_CTRL_RESOLUTION_MASK) >> MLX90640_CTRL_RESOLUTION_SHIFT;

    return resolutionRAM;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate) {
    uint16_t controlRegister1;
    uint16_t value;
    int error;

    //value = (refreshRate & 0x07)<<7;
    value = ((uint16_t) refreshRate << MLX90640_CTRL_REFRESH_SHIFT);
    value &= ~MLX90640_CTRL_REFRESH_MASK;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error == MLX90640_NO_ERROR) {
        value = (controlRegister1 & MLX90640_CTRL_REFRESH_MASK) | value;
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }

    return error;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_GetRefreshRate(uint8_t slaveAddr) {
    uint16_t controlRegister1;
    int refreshRate;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error != MLX90640_NO_ERROR) {
        return error;
    }
    refreshRate = (controlRegister1 & ~MLX90640_CTRL_REFRESH_MASK) >> MLX90640_CTRL_REFRESH_SHIFT;

    return refreshRate;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_SetInterleavedMode(uint8_t slaveAddr) {
    uint16_t controlRegister1;
    uint16_t value;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);

    if (error == 0) {
        value = (controlRegister1 & ~MLX90640_CTRL_MEAS_MODE_MASK);
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }

    return error;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_SetChessMode(uint8_t slaveAddr) {
    uint16_t controlRegister1;
    uint16_t value;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);

    if (error == 0) {
        value = (controlRegister1 | MLX90640_CTRL_MEAS_MODE_MASK);
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }

    return error;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_GetCurMode(uint8_t slaveAddr) {
    uint16_t controlRegister1;
    int modeRAM;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error != 0) {
        return error;
    }
    modeRAM = (controlRegister1 & MLX90640_CTRL_MEAS_MODE_MASK) >> MLX90640_CTRL_MEAS_MODE_SHIFT;

    return modeRAM;
}

//------------------------------------------------------------------------------

void Adafruit_MLX90640::MLX90640_CalculateTo(float emissivity, float tr, float* result) {
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorrR[4];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;

    subPage = mlx90640Frame_[833];
    vdd = MLX90640_GetVdd();
    ta = MLX90640_GetTa();
    
    // Validate ta early since many calculations depend on it
    // If ta is invalid, return early without modifying result buffer
    // (result may contain valid data from previous frames)
    if (!std::isfinite(ta)) {
        ESP_LOGE("MLX90640", "ta is invalid: %f", ta);
        return;
    }

    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    taTr = tr4 - (tr4 - ta4) / emissivity;

    ktaScale = POW2(_params.ktaScale);
    kvScale = POW2(_params.kvScale);
    alphaScale = POW2(_params.alphaScale);

    alphaCorrR[0] = 1 / (1 + _params.ksTo[0] * 40);
    alphaCorrR[1] = 1;
    alphaCorrR[2] = (1 + _params.ksTo[1] * _params.ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + _params.ksTo[2] * (_params.ct[3] - _params.ct[2]));

    //------------------------- Gain calculation -----------------------------------    

    // Guard against division by zero in gain calculation
    // Both gainEE and mlx90640Frame_[778] are int16_t, so result is always finite when gainRaw != 0
    int16_t gainRaw = (int16_t) mlx90640Frame_[778];
    if (gainRaw == 0) {
        gain = 1.0f; // Safe default to prevent inf
    } else {
        gain = (float) _params.gainEE / gainRaw;
    }

    //------------------------- To calculation -------------------------------------    
    mode = (mlx90640Frame_[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5;

    irDataCP[0] = (int16_t) mlx90640Frame_[776] * gain;
    irDataCP[1] = (int16_t) mlx90640Frame_[808] * gain;

    irDataCP[0] = irDataCP[0] - _params.cpOffset[0] * (1 + _params.cpKta * (ta - 25)) * (1 + _params.cpKv * (vdd - 3.3));
    if (mode == _params.calibrationModeEE) {
        irDataCP[1] = irDataCP[1] - _params.cpOffset[1] * (1 + _params.cpKta * (ta - 25)) * (1 + _params.cpKv * (vdd - 3.3));
    }
    else {
        irDataCP[1] = irDataCP[1] - (_params.cpOffset[1] + _params.ilChessC[0]) * (1 + _params.cpKta * (ta - 25)) * (1 + _params.cpKv * (vdd - 3.3));
    }

    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

        if (mode == 0) {
            pattern = ilPattern;
        }
        else {
            pattern = chessPattern;
        }

        if (pattern == mlx90640Frame_[833]) {
            irData = (int16_t) mlx90640Frame_[pixelNumber] * gain;

            kta = _params.kta[pixelNumber] / ktaScale;
            kv = _params.kv[pixelNumber] / kvScale;
            irData = irData - _params.offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3));

            if (mode != _params.calibrationModeEE) {
                irData = irData + _params.ilChessC[2] * (2 * ilPattern - 1) - _params.ilChessC[1] * conversionPattern;
            }

            irData = irData - _params.tgc * irDataCP[subPage];
            irData = irData / emissivity;

            alphaCompensated = SCALEALPHA * alphaScale / _params.alpha[pixelNumber];
            alphaCompensated = alphaCompensated * (1 + _params.KsTa * (ta - 25));
            
            // Validate alphaCompensated is finite and positive
            // Edge cases that could still cause issues despite ta/KsTa/alphaScale validation:
            // 1. Floating point overflow/underflow in SCALEALPHA * alphaScale multiplication
            // 2. Negative result: if ta >> 25 and KsTa < 0, (1 + KsTa*(ta-25)) could be < 0
            // 3. Very small _params.alpha[pixelNumber] could cause inf from division
            // 4. Floating point precision issues producing NaN
            // Skip updating this pixel if invalid (preserve existing value in result buffer)
            if (!std::isfinite(alphaCompensated) || alphaCompensated <= 0.0f) {
                ESP_LOGE("MLX90640", "alphaCompensated is invalid: %f", alphaCompensated);
                continue; // Skip invalid pixel
            }

            // Validate taTr is finite before calculations
            // Skip updating this pixel if invalid (preserve existing value in result buffer)
            if (!std::isfinite(taTr)) {
                ESP_LOGE("MLX90640", "taTr is invalid: %f", taTr);
                continue; // Skip invalid pixel
            }
            
            Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
            
            // Guard against negative values before sqrt
            if (Sx < 0.0f) {
                ESP_LOGE("MLX90640", "Sx is negative: %f", Sx);
                Sx = 0.0f; // Clamp to zero to prevent NaN from sqrt
            }
            Sx = sqrt(sqrt(Sx)) * _params.ksTo[1];
            
            // Guard against division by zero and negative values in To calculation
            // Skip updating this pixel if invalid (preserve existing value in result buffer)
            float denominator = alphaCompensated * (1 - _params.ksTo[1] * 273.15) + Sx;
            if (denominator == 0.0f || !std::isfinite(denominator)) {
                ESP_LOGE("MLX90640", "denominator is invalid: %f", denominator);
                continue; // Skip invalid pixel
            }
            
            float innerValue = irData / denominator + taTr;
            // Guard against overflow to infinity: if irData/denominator would overflow, skip
            if (!std::isfinite(innerValue)) {
                ESP_LOGE("MLX90640", "innerValue is invalid: %f", innerValue);
                continue; // Skip invalid pixel
            }
            if (innerValue < 0.0f) {
                innerValue = 0.0f; // Clamp to prevent NaN from sqrt
            }
            To = sqrt(sqrt(innerValue)) - 273.15;

            if (To < _params.ct[1]) {
                range = 0;
            }
            else if (To < _params.ct[2]) {
                range = 1;
            }
            else if (To < _params.ct[3]) {
                range = 2;
            }
            else {
                range = 3;
            }

            // Guard against division by zero and negative values in final To calculation
            // Skip updating this pixel if invalid (preserve existing value in result buffer)
            float rangeDenominator = alphaCompensated * alphaCorrR[range] * (1 + _params.ksTo[range] * (To - _params.ct[range]));
            if (rangeDenominator == 0.0f || !std::isfinite(rangeDenominator)) {
                ESP_LOGE("MLX90640", "rangeDenominator is invalid: %f", rangeDenominator);
                continue; // Skip invalid pixel
            }
            
            float finalInnerValue = irData / rangeDenominator + taTr;
            // Guard against overflow to infinity: if irData/rangeDenominator would overflow, skip
            if (!std::isfinite(finalInnerValue)) {
                ESP_LOGE("MLX90640", "finalInnerValue is invalid: %f", finalInnerValue);
                continue; // Skip invalid pixel
            }
            if (finalInnerValue < 0.0f) {
                ESP_LOGE("MLX90640", "finalInnerValue is negative: %f", finalInnerValue);
                finalInnerValue = 0.0f; // Clamp to prevent NaN from sqrt
            }
            To = sqrt(sqrt(finalInnerValue)) - 273.15;
            
            // Final validation - ensure result is finite before assigning
            // Skip updating this pixel if invalid (preserve existing value in result buffer)
            if (std::isfinite(To)) {
                result[pixelNumber] = To;
            } else {
                ESP_LOGE("MLX90640", "To is invalid: %f", To);
            }
        }
    }
}

//------------------------------------------------------------------------------

void Adafruit_MLX90640::MLX90640_GetImage(float* result) {
    float vdd;
    float ta;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float image;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float kta;
    float kv;

    subPage = mlx90640Frame_[833];
    vdd = MLX90640_GetVdd();
    ta = MLX90640_GetTa();

    ktaScale = POW2(_params.ktaScale);
    kvScale = POW2(_params.kvScale);

    //------------------------- Gain calculation -----------------------------------    

    gain = (float) _params.gainEE / (int16_t) mlx90640Frame_[778];

    //------------------------- Image calculation -------------------------------------    

    mode = (mlx90640Frame_[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5;

    irDataCP[0] = (int16_t) mlx90640Frame_[776] * gain;
    irDataCP[1] = (int16_t) mlx90640Frame_[808] * gain;

    irDataCP[0] = irDataCP[0] - _params.cpOffset[0] * (1 + _params.cpKta * (ta - 25)) * (1 + _params.cpKv * (vdd - 3.3));
    if (mode == _params.calibrationModeEE) {
        irDataCP[1] = irDataCP[1] - _params.cpOffset[1] * (1 + _params.cpKta * (ta - 25)) * (1 + _params.cpKv * (vdd - 3.3));
    }
    else {
        irDataCP[1] = irDataCP[1] - (_params.cpOffset[1] + _params.ilChessC[0]) * (1 + _params.cpKta * (ta - 25)) * (1 + _params.cpKv * (vdd - 3.3));
    }

    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

        if (mode == 0) {
            pattern = ilPattern;
        }
        else {
            pattern = chessPattern;
        }

        if (pattern == mlx90640Frame_[833]) {
            irData = (int16_t) mlx90640Frame_[pixelNumber] * gain;

            kta = _params.kta[pixelNumber] / ktaScale;
            kv = _params.kv[pixelNumber] / kvScale;
            irData = irData - _params.offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3));

            if (mode != _params.calibrationModeEE) {
                irData = irData + _params.ilChessC[2] * (2 * ilPattern - 1) - _params.ilChessC[1] * conversionPattern;
            }

            irData = irData - _params.tgc * irDataCP[subPage];

            alphaCompensated = _params.alpha[pixelNumber];

            image = irData * alphaCompensated;

            result[pixelNumber] = image;
        }
    }
}

//------------------------------------------------------------------------------

float Adafruit_MLX90640::MLX90640_GetVdd() {
    float vdd;
    float resolutionCorrection;

    uint16_t resolutionRAM;

    resolutionRAM = (mlx90640Frame_[832] & ~MLX90640_CTRL_RESOLUTION_MASK) >> MLX90640_CTRL_RESOLUTION_SHIFT;
    resolutionCorrection = POW2(_params.resolutionEE) / POW2(resolutionRAM);
    vdd = (resolutionCorrection * (int16_t) mlx90640Frame_[810] - _params.vdd25) / _params.kVdd + 3.3;

    return vdd;
}

//------------------------------------------------------------------------------

float Adafruit_MLX90640::MLX90640_GetTa() {
    int16_t ptat;
    float ptatArt;
    float vdd;
    float ta;

    vdd = MLX90640_GetVdd();

    ptat = (int16_t) mlx90640Frame_[800];

    // Guard against division by zero in ptatArt calculation
    float ptatDenominator = ptat * _params.alphaPTAT + (int16_t) mlx90640Frame_[768];
    if (ptatDenominator == 0.0f) {
        ESP_LOGE("MLX90640", "ptatDenominator is invalid: %f", ptatDenominator);
        return std::numeric_limits<float>::quiet_NaN();
    }
    ptatArt = (ptat / ptatDenominator) * POW2(18);

    // Guard against division by zero in ta calculation
    // kvDenominator = 1 + KvPTAT * (vdd - 3.3)
    // KvPTAT ranges from -0.0078125 to 0.007568359375
    // For kvDenominator to be 0, (vdd - 3.3) would need to be ~±128V, which is unrealistic
    // but could occur with corrupted EEPROM/sensor data
    float kvDenominator = 1 + _params.KvPTAT * (vdd - 3.3);
    ta = (ptatArt / kvDenominator - _params.vPTAT25);
    
    ta = ta / _params.KtPTAT + 25;

    return ta;
}

//------------------------------------------------------------------------------

int Adafruit_MLX90640::MLX90640_GetSubPageNumber() {
    return mlx90640Frame_[833];
}

//------------------------------------------------------------------------------
void Adafruit_MLX90640::MLX90640_BadPixelsCorrection(uint16_t* pixels, float* to, int mode) {
    float ap[4];
    uint8_t pix;
    uint8_t line;
    uint8_t column;

    pix = 0;
    while (pixels[pix] != 0xFFFF) {
        line = pixels[pix] >> 5;
        column = pixels[pix] - (line << 5);

        if (mode == 1) {
            if (line == 0) {
                if (column == 0) {
                    to[pixels[pix]] = to[33];
                }
                else if (column == 31) {
                    to[pixels[pix]] = to[62];
                }
                else {
                    to[pixels[pix]] = (to[pixels[pix] + 31] + to[pixels[pix] + 33]) / 2.0;
                }
            }
            else if (line == 23) {
                if (column == 0) {
                    to[pixels[pix]] = to[705];
                }
                else if (column == 31) {
                    to[pixels[pix]] = to[734];
                }
                else {
                    to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] - 31]) / 2.0;
                }
            }
            else if (column == 0) {
                to[pixels[pix]] = (to[pixels[pix] - 31] + to[pixels[pix] + 33]) / 2.0;
            }
            else if (column == 31) {
                to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] + 31]) / 2.0;
            }
            else {
                ap[0] = to[pixels[pix] - 33];
                ap[1] = to[pixels[pix] - 31];
                ap[2] = to[pixels[pix] + 31];
                ap[3] = to[pixels[pix] + 33];
                to[pixels[pix]] = GetMedian(ap, 4);
            }
        }
        else {
            if (column == 0) {
                to[pixels[pix]] = to[pixels[pix] + 1];
            }
            else if (column == 1 || column == 30) {
                to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0;
            }
            else if (column == 31) {
                to[pixels[pix]] = to[pixels[pix] - 1];
            }
            else {
                if (IsPixelBad(pixels[pix] - 2, &_params) == 0 && IsPixelBad(pixels[pix] + 2, &_params) == 0) {
                    ap[0] = to[pixels[pix] + 1] - to[pixels[pix] + 2];
                    ap[1] = to[pixels[pix] - 1] - to[pixels[pix] - 2];
                    if (fabs(ap[0]) > fabs(ap[1])) {
                        to[pixels[pix]] = to[pixels[pix] - 1] + ap[1];
                    }
                    else {
                        to[pixels[pix]] = to[pixels[pix] + 1] + ap[0];
                    }
                }
                else {
                    to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0;
                }
            }
        }
        pix = pix + 1;
    }
}

//------------------------------------------------------------------------------

static void ExtractVDDParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    int8_t kVdd;
    int16_t vdd25;

    kVdd = MLX90640_MS_BYTE(eeData[51]);

    vdd25 = MLX90640_LS_BYTE(eeData[51]);
    vdd25 = ((vdd25 - 256) << 5) - 8192;

    mlx90640->kVdd = 32 * kVdd;
    mlx90640->vdd25 = vdd25;
}

//------------------------------------------------------------------------------

static void ExtractPTATParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;

    KvPTAT = (eeData[50] & MLX90640_MSBITS_6_MASK) >> 10;
    if (KvPTAT > 31) {
        KvPTAT = KvPTAT - 64;
    }
    KvPTAT = KvPTAT / 4096;

    KtPTAT = eeData[50] & MLX90640_LSBITS_10_MASK;
    if (KtPTAT > 511) {
        KtPTAT = KtPTAT - 1024;
    }
    KtPTAT = KtPTAT / 8;

    vPTAT25 = eeData[49];

    alphaPTAT = (eeData[16] & MLX90640_NIBBLE4_MASK) / POW2(14) + 8.0f;

    mlx90640->KvPTAT = KvPTAT;
    mlx90640->KtPTAT = KtPTAT;
    mlx90640->vPTAT25 = vPTAT25;
    mlx90640->alphaPTAT = alphaPTAT;
}

//------------------------------------------------------------------------------

static void ExtractGainParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    mlx90640->gainEE = (int16_t) eeData[48];;
}

//------------------------------------------------------------------------------

static void ExtractTgcParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    mlx90640->tgc = (int8_t) MLX90640_LS_BYTE(eeData[60]) / 32.0f;
}

//------------------------------------------------------------------------------

static void ExtractResolutionParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    uint8_t resolutionEE;
    resolutionEE = (eeData[56] & 0x3000) >> 12;

    mlx90640->resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

static void ExtractKsTaParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    mlx90640->KsTa = (int8_t) MLX90640_MS_BYTE(eeData[60]) / 8192.0f;
}

//------------------------------------------------------------------------------

static void ExtractKsToParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    int32_t KsToScale;
    int8_t step;

    step = ((eeData[63] & 0x3000) >> 12) * 10;

    mlx90640->ct[0] = -40;
    mlx90640->ct[1] = 0;
    mlx90640->ct[2] = MLX90640_NIBBLE2(eeData[63]);
    mlx90640->ct[3] = MLX90640_NIBBLE3(eeData[63]);

    mlx90640->ct[2] = mlx90640->ct[2] * step;
    mlx90640->ct[3] = mlx90640->ct[2] + mlx90640->ct[3] * step;
    mlx90640->ct[4] = 400;

    KsToScale = MLX90640_NIBBLE1(eeData[63]) + 8;
    KsToScale = 1UL << KsToScale;

    mlx90640->ksTo[0] = (int8_t) MLX90640_LS_BYTE(eeData[61]) / (float) KsToScale;
    mlx90640->ksTo[1] = (int8_t) MLX90640_MS_BYTE(eeData[61]) / (float) KsToScale;
    mlx90640->ksTo[2] = (int8_t) MLX90640_LS_BYTE(eeData[62]) / (float) KsToScale;
    mlx90640->ksTo[3] = (int8_t) MLX90640_MS_BYTE(eeData[62]) / (float) KsToScale;
    mlx90640->ksTo[4] = -0.0002;
}

//------------------------------------------------------------------------------

static void ExtractAlphaParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;
    float alphaTemp[768];
    float temp;


    accRemScale = MLX90640_NIBBLE1(eeData[32]);
    accColumnScale = MLX90640_NIBBLE2(eeData[32]);
    accRowScale = MLX90640_NIBBLE3(eeData[32]);
    alphaScale = MLX90640_NIBBLE4(eeData[32]) + 30;
    alphaRef = eeData[33];

    for (int i = 0; i < 6; i++) {
        p = i * 4;
        accRow[p + 0] = MLX90640_NIBBLE1(eeData[34 + i]);
        accRow[p + 1] = MLX90640_NIBBLE2(eeData[34 + i]);
        accRow[p + 2] = MLX90640_NIBBLE3(eeData[34 + i]);
        accRow[p + 3] = MLX90640_NIBBLE4(eeData[34 + i]);
    }

    for (int i = 0; i < MLX90640_LINE_NUM; i++) {
        if (accRow[i] > 7) {
            accRow[i] = accRow[i] - 16;
        }
    }

    for (int i = 0; i < 8; i++) {
        p = i * 4;
        accColumn[p + 0] = MLX90640_NIBBLE1(eeData[40 + i]);
        accColumn[p + 1] = MLX90640_NIBBLE2(eeData[40 + i]);
        accColumn[p + 2] = MLX90640_NIBBLE3(eeData[40 + i]);
        accColumn[p + 3] = MLX90640_NIBBLE4(eeData[40 + i]);
    }

    for (int i = 0; i < MLX90640_COLUMN_NUM; i++) {
        if (accColumn[i] > 7) {
            accColumn[i] = accColumn[i] - 16;
        }
    }

    for (int i = 0; i < MLX90640_LINE_NUM; i++) {
        for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
            p = 32 * i + j;
            alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
            if (alphaTemp[p] > 31) {
                alphaTemp[p] = alphaTemp[p] - 64;
            }
            alphaTemp[p] = alphaTemp[p] * (1 << accRemScale);
            alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + alphaTemp[p]);
            alphaTemp[p] = alphaTemp[p] / POW2(alphaScale);
            alphaTemp[p] = alphaTemp[p] - mlx90640->tgc * (mlx90640->cpAlpha[0] + mlx90640->cpAlpha[1]) / 2;
            // Guard against division by zero - if alphaTemp[p] is 0, set to a safe default
            if (alphaTemp[p] == 0.0f || !std::isfinite(alphaTemp[p])) {
                ESP_LOGE("MLX90640", "alphaTemp[p] is invalid: %f", alphaTemp[p]);
                alphaTemp[p] = 1.0f; // Safe default to prevent inf/NaN
            }
            alphaTemp[p] = SCALEALPHA / alphaTemp[p];
        }
    }

    temp = alphaTemp[0];
    for (int i = 1; i < MLX90640_PIXEL_NUM; i++) {
        if (alphaTemp[i] > temp) {
            temp = alphaTemp[i];
        }
    }

    alphaScale = 0;
    // Guard against infinite loop if temp is NaN/inf (comparison will be false, loop won't run)
    // Also guard against overflow by limiting iterations
    if (std::isfinite(temp) && temp > 0.0f) {
        while (temp < 32767.4 && alphaScale < 31) { // Limit to prevent overflow (2^31 would overflow)
            temp = temp * 2;
            alphaScale = alphaScale + 1;
        }
    }
    // If temp was invalid, alphaScale stays at 0 (safe default)

    for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
        temp = alphaTemp[i] * POW2(alphaScale);
        mlx90640->alpha[i] = (temp + 0.5);

    }

    mlx90640->alphaScale = alphaScale;

}

//------------------------------------------------------------------------------

static void ExtractOffsetParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;


    occRemScale = MLX90640_NIBBLE1(eeData[16]);
    occColumnScale = MLX90640_NIBBLE2(eeData[16]);
    occRowScale = MLX90640_NIBBLE3(eeData[16]);
    offsetRef = (int16_t) eeData[17];

    for (int i = 0; i < 6; i++) {
        p = i * 4;
        occRow[p + 0] = MLX90640_NIBBLE1(eeData[18 + i]);
        occRow[p + 1] = MLX90640_NIBBLE2(eeData[18 + i]);
        occRow[p + 2] = MLX90640_NIBBLE3(eeData[18 + i]);
        occRow[p + 3] = MLX90640_NIBBLE4(eeData[18 + i]);
    }

    for (int i = 0; i < MLX90640_LINE_NUM; i++) {
        if (occRow[i] > 7) {
            occRow[i] = occRow[i] - 16;
        }
    }

    for (int i = 0; i < 8; i++) {
        p = i * 4;
        occColumn[p + 0] = MLX90640_NIBBLE1(eeData[24 + i]);
        occColumn[p + 1] = MLX90640_NIBBLE2(eeData[24 + i]);
        occColumn[p + 2] = MLX90640_NIBBLE3(eeData[24 + i]);
        occColumn[p + 3] = MLX90640_NIBBLE4(eeData[24 + i]);
    }

    for (int i = 0; i < MLX90640_COLUMN_NUM; i++) {
        if (occColumn[i] > 7) {
            occColumn[i] = occColumn[i] - 16;
        }
    }

    for (int i = 0; i < MLX90640_LINE_NUM; i++) {
        for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
            p = 32 * i + j;
            mlx90640->offset[p] = (eeData[64 + p] & MLX90640_MSBITS_6_MASK) >> 10;
            if (mlx90640->offset[p] > 31) {
                mlx90640->offset[p] = mlx90640->offset[p] - 64;
            }
            mlx90640->offset[p] = mlx90640->offset[p] * (1 << occRemScale);
            mlx90640->offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + mlx90640->offset[p]);
        }
    }
}

//------------------------------------------------------------------------------

static void ExtractKtaPixelParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    int p = 0;
    int8_t KtaRC[4];
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
    float ktaTemp[768];
    float temp;

    KtaRC[0] = (int8_t) MLX90640_MS_BYTE(eeData[54]);;
    KtaRC[2] = (int8_t) MLX90640_LS_BYTE(eeData[54]);;
    KtaRC[1] = (int8_t) MLX90640_MS_BYTE(eeData[55]);;
    KtaRC[3] = (int8_t) MLX90640_LS_BYTE(eeData[55]);;

    ktaScale1 = MLX90640_NIBBLE2(eeData[56]) + 8;
    ktaScale2 = MLX90640_NIBBLE1(eeData[56]);

    for (int i = 0; i < MLX90640_LINE_NUM; i++) {
        for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
            p = 32 * i + j;
            split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
            ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
            if (ktaTemp[p] > 3) {
                ktaTemp[p] = ktaTemp[p] - 8;
            }
            ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2);
            ktaTemp[p] = KtaRC[split] + ktaTemp[p];
            ktaTemp[p] = ktaTemp[p] / POW2(ktaScale1);

        }
    }

    temp = fabs(ktaTemp[0]);
    for (int i = 1; i < MLX90640_PIXEL_NUM; i++) {
        if (fabs(ktaTemp[i]) > temp) {
            temp = fabs(ktaTemp[i]);
        }
    }

    ktaScale1 = 0;
    while (temp < 63.4) {
        temp = temp * 2;
        ktaScale1 = ktaScale1 + 1;
    }

    for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
        temp = ktaTemp[i] * POW2(ktaScale1);
        if (temp < 0) {
            mlx90640->kta[i] = (temp - 0.5);
        }
        else {
            mlx90640->kta[i] = (temp + 0.5);
        }

    }

    mlx90640->ktaScale = ktaScale1;
}


//------------------------------------------------------------------------------

static void ExtractKvPixelParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    int p = 0;
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvRoCe;
    int8_t KvReCo;
    int8_t KvReCe;
    uint8_t kvScale;
    uint8_t split;
    float kvTemp[768];
    float temp;

    KvRoCo = MLX90640_NIBBLE4(eeData[52]);
    if (KvRoCo > 7) {
        KvRoCo = KvRoCo - 16;
    }
    KvT[0] = KvRoCo;

    KvReCo = MLX90640_NIBBLE3(eeData[52]);
    if (KvReCo > 7) {
        KvReCo = KvReCo - 16;
    }
    KvT[2] = KvReCo;

    KvRoCe = MLX90640_NIBBLE2(eeData[52]);
    if (KvRoCe > 7) {
        KvRoCe = KvRoCe - 16;
    }
    KvT[1] = KvRoCe;

    KvReCe = MLX90640_NIBBLE1(eeData[52]);
    if (KvReCe > 7) {
        KvReCe = KvReCe - 16;
    }
    KvT[3] = KvReCe;

    kvScale = MLX90640_NIBBLE3(eeData[56]);


    for (int i = 0; i < MLX90640_LINE_NUM; i++) {
        for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
            p = 32 * i + j;
            split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
            kvTemp[p] = KvT[split];
            kvTemp[p] = kvTemp[p] / POW2(kvScale);
        }
    }

    temp = fabs(kvTemp[0]);
    for (int i = 1; i < MLX90640_PIXEL_NUM; i++) {
        if (fabs(kvTemp[i]) > temp) {
            temp = fabs(kvTemp[i]);
        }
    }

    kvScale = 0;
    while (temp < 63.4) {
        temp = temp * 2;
        kvScale = kvScale + 1;
    }

    for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
        temp = kvTemp[i] * POW2(kvScale);
        if (temp < 0) {
            mlx90640->kv[i] = (temp - 0.5);
        }
        else {
            mlx90640->kv[i] = (temp + 0.5);
        }

    }

    mlx90640->kvScale = kvScale;
}

//------------------------------------------------------------------------------

static void ExtractCPParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = MLX90640_NIBBLE4(eeData[32]) + 27;

    offsetSP[0] = (eeData[58] & MLX90640_LSBITS_10_MASK);
    if (offsetSP[0] > 511) {
        offsetSP[0] = offsetSP[0] - 1024;
    }

    offsetSP[1] = (eeData[58] & MLX90640_MSBITS_6_MASK) >> 10;
    if (offsetSP[1] > 31) {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0];

    alphaSP[0] = (eeData[57] & MLX90640_LSBITS_10_MASK);
    if (alphaSP[0] > 511) {
        alphaSP[0] = alphaSP[0] - 1024;
    }
    alphaSP[0] = alphaSP[0] / POW2(alphaScale);

    alphaSP[1] = (eeData[57] & MLX90640_MSBITS_6_MASK) >> 10;
    if (alphaSP[1] > 31) {
        alphaSP[1] = alphaSP[1] - 64;
    }
    alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0];

    cpKta = (int8_t) MLX90640_LS_BYTE(eeData[59]);

    ktaScale1 = MLX90640_NIBBLE2(eeData[56]) + 8;
    mlx90640->cpKta = cpKta / POW2(ktaScale1);

    cpKv = (int8_t) MLX90640_MS_BYTE(eeData[59]);

    kvScale = MLX90640_NIBBLE3(eeData[56]);
    mlx90640->cpKv = cpKv / POW2(kvScale);

    mlx90640->cpAlpha[0] = alphaSP[0];
    mlx90640->cpAlpha[1] = alphaSP[1];
    mlx90640->cpOffset[0] = offsetSP[0];
    mlx90640->cpOffset[1] = offsetSP[1];
}

//------------------------------------------------------------------------------

static void ExtractCILCParameters(uint16_t* eeData, paramsMLX90640* mlx90640) {
    float ilChessC[3];
    uint8_t calibrationModeEE;

    calibrationModeEE = (eeData[10] & 0x0800) >> 4;
    calibrationModeEE = calibrationModeEE ^ 0x80;

    ilChessC[0] = (eeData[53] & 0x003F);
    if (ilChessC[0] > 31) {
        ilChessC[0] = ilChessC[0] - 64;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;

    ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
    if (ilChessC[1] > 15) {
        ilChessC[1] = ilChessC[1] - 32;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;

    ilChessC[2] = (eeData[53] & 0xF800) >> 11;
    if (ilChessC[2] > 15) {
        ilChessC[2] = ilChessC[2] - 32;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;

    mlx90640->calibrationModeEE = calibrationModeEE;
    mlx90640->ilChessC[0] = ilChessC[0];
    mlx90640->ilChessC[1] = ilChessC[1];
    mlx90640->ilChessC[2] = ilChessC[2];
}

//------------------------------------------------------------------------------

static int ExtractDeviatingPixels(uint16_t* eeData, paramsMLX90640* mlx90640) {
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    int warn = 0;
    int i;

    for (pixCnt = 0; pixCnt < 5; pixCnt++) {
        mlx90640->brokenPixels[pixCnt] = 0xFFFF;
        mlx90640->outlierPixels[pixCnt] = 0xFFFF;
    }

    pixCnt = 0;
    while (pixCnt < MLX90640_PIXEL_NUM && brokenPixCnt < 5 && outlierPixCnt < 5) {
        if (eeData[pixCnt + 64] == 0) {
            mlx90640->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }
        else if ((eeData[pixCnt + 64] & 0x0001) != 0) {
            mlx90640->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt = outlierPixCnt + 1;
        }

        pixCnt = pixCnt + 1;

    }

    if (brokenPixCnt > 4) {
        warn = -MLX90640_BROKEN_PIXELS_NUM_ERROR;
    }
    else if (outlierPixCnt > 4) {
        warn = -MLX90640_OUTLIER_PIXELS_NUM_ERROR;
    }
    else if ((brokenPixCnt + outlierPixCnt) > 4) {
        warn = -MLX90640_BAD_PIXELS_NUM_ERROR;
    }
    else {
        for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
            for (i = pixCnt + 1; i < brokenPixCnt; i++) {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt], mlx90640->brokenPixels[i]);
                if (warn != 0) {
                    return warn;
                }
            }
        }

        for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++) {
            for (i = pixCnt + 1; i < outlierPixCnt; i++) {
                warn = CheckAdjacentPixels(mlx90640->outlierPixels[pixCnt], mlx90640->outlierPixels[i]);
                if (warn != 0) {
                    return warn;
                }
            }
        }

        for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
            for (i = 0; i < outlierPixCnt; i++) {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt], mlx90640->outlierPixels[i]);
                if (warn != 0) {
                    return warn;
                }
            }
        }

    }


    return warn;

}

//------------------------------------------------------------------------------

static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2) {

    int pixPosDif;
    uint16_t lp1 = pix1 >> 5;
    uint16_t lp2 = pix2 >> 5;
    uint16_t cp1 = pix1 - (lp1 << 5);
    uint16_t cp2 = pix2 - (lp2 << 5);

    pixPosDif = lp1 - lp2;
    if (pixPosDif > -2 && pixPosDif < 2) {
        pixPosDif = cp1 - cp2;
        if (pixPosDif > -2 && pixPosDif < 2) {
            return -6;
        }

    }

    return 0;
}

//------------------------------------------------------------------------------

static float GetMedian(float* values, int n) {
    float temp;

    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (values[j] < values[i]) {
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }

    if (n % 2 == 0) {
        return ((values[n / 2] + values[n / 2 - 1]) / 2.0);

    }
    else {
        return values[n / 2];
    }

}

//------------------------------------------------------------------------------

static int IsPixelBad(uint16_t pixel, paramsMLX90640* params) {
    for (int i = 0; i < 5; i++) {
        if (pixel == params->outlierPixels[i] || pixel == params->brokenPixels[i]) {
            return 1;
        }
    }

    return 0;
}

//------------------------------------------------------------------------------