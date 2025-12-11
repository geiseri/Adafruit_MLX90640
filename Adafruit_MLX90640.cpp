#include <Adafruit_MLX90640.h>

Adafruit_MLX90640::Adafruit_MLX90640() {}

boolean Adafruit_MLX90640::begin(uint8_t i2c_addr, TwoWire* wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

  if (!i2c_dev->begin()) {
    return false;
  }
  i2c_dev->setSpeed(1000000); // Speed it up, lots to read :)
  MLX90640_I2CRead(0, Adafruit_MLX90640::MLX90640_DEVICEID1, 3, serialNumber);

  uint16_t eeMLX90640[832];
  if (MLX90640_DumpEE(0, eeMLX90640) != 0) {
    return false;
  }
#ifdef MLX90640_DEBUG
  for (int i = 0; i < 832; i++) {
    Serial.printf("0x%x, ", eeMLX90640[i]);
  }
  Serial.println();
#endif

  MLX90640_ExtractParameters(eeMLX90640);
  // whew!
  return true;
}

int Adafruit_MLX90640::MLX90640_I2CRead(uint8_t slaveAddr,
  uint16_t startAddress,
  uint16_t nMemAddressRead,
  uint16_t* data) {
  uint8_t cmd[2];

  while (nMemAddressRead > 0) {
    uint16_t toRead16 =
      min(nMemAddressRead, (uint16_t) (i2c_dev->maxBufferSize() / 2));

    cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0x00FF;
    // Serial.printf("Reading %d words\n", toRead16);
    if (!i2c_dev->write_then_read(cmd, 2, (uint8_t*) data, toRead16 * 2,
      false)) {
      return -1;
    }
    // we now have to swap every two bytes
    for (int i = 0; i < toRead16; i++) {
      data[i] = __builtin_bswap16(data[i]);
    }
    // advance buffer
    data += toRead16;
    // advance address
    startAddress += toRead16;
    // reduce remaining to read
    nMemAddressRead -= toRead16;
  }
  // success!
  return 0;
}

int Adafruit_MLX90640::MLX90640_I2CWrite(uint8_t slaveAddr,
  uint16_t writeAddress, uint16_t data) {
  uint8_t cmd[4];
  uint16_t dataCheck = 0;

  cmd[0] = writeAddress >> 8;
  cmd[1] = writeAddress & 0x00FF;
  cmd[2] = data >> 8;
  cmd[3] = data & 0x00FF;

  if (!i2c_dev->write(cmd, 4, true)) {
    return -1;
  }
  delay(1);

  if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &dataCheck) != 0) {
    return -1;
  }

  // check echo
  if (dataCheck != data) {
    return -2;
  }
  // OK!
  return 0;
}

int Adafruit_MLX90640::MLX90640_I2CGeneralReset() {
  return 0;
}

mlx90640_mode_t Adafruit_MLX90640::getMode() {
  return static_cast<mlx90640_mode_t>(MLX90640_GetCurMode(0));
}

void Adafruit_MLX90640::setMode(mlx90640_mode_t mode) {
  if (mode == mlx90640_mode_t::CHESS) {
    MLX90640_SetChessMode(0);
  }
  else {
    MLX90640_SetInterleavedMode(0);
  }
}

mlx90640_resolution_t Adafruit_MLX90640::getResolution() {
  return static_cast<mlx90640_resolution_t>(MLX90640_GetCurResolution(0));
}

void Adafruit_MLX90640::setResolution(mlx90640_resolution_t res) {
  MLX90640_SetResolution(0, (int) res);
}

mlx90640_refreshrate_t Adafruit_MLX90640::getRefreshRate() {
  return static_cast<mlx90640_refreshrate_t>(MLX90640_GetRefreshRate(0));
}

void Adafruit_MLX90640::setRefreshRate(mlx90640_refreshrate_t rate) {
  MLX90640_SetRefreshRate(0, (int) rate);
}

int Adafruit_MLX90640::getFrame(float* framebuf) {
  float emissivity = Adafruit_MLX90640::MLX90640_DEFAULT_EMISSIVITY;
  float tr = 23.15;
  int status;


  for (uint8_t page = 0; page < 2; page++) {
    grab_start_time = millis();
    status = MLX90640_GetFrameData(0, mlx90640Frame_.data());
    grab_end_time = millis();

    if (status < 0) {
      return status;
    }

    tr = MLX90640_GetTa() - Adafruit_MLX90640::OPENAIR_TA_SHIFT; // For a MLX90640 in the open air the shift is -8  degC.

    calculate_start_time = millis();
    MLX90640_CalculateTo(emissivity, tr, framebuf);
    calculate_end_time = millis();
    ta = tr;
  }
  return 0;
}

int Adafruit_MLX90640::getImage(float* framebuf) {
  int status;


  for (uint8_t page = 0; page < 2; page++) {
    grab_start_time = millis();
    status = MLX90640_GetFrameData(0, mlx90640Frame_.data());
    grab_end_time = millis();

    if (status < 0) {
      return status;
    }


    calculate_start_time = millis();
    MLX90640_GetImage(framebuf);
    calculate_end_time = millis();
  }
  return 0;
}

float Adafruit_MLX90640::getTa(bool newFrame) {
  if (!newFrame) {
    return ta;
  }
  MLX90640_GetFrameData(0, mlx90640Frame_.data());
  return MLX90640_GetTa();
}

bool Adafruit_MLX90640::updatePartialFrame(float* framebuf) {
  float emissivity = Adafruit_MLX90640::MLX90640_DEFAULT_EMISSIVITY;
  float tr = 23.15;
  uint16_t status = 0;

  auto code = MLX90640_GetFramePage(0, status);
  if (code != MLX90640_NO_ERROR){
    return false;
  }

  if (status < 0) {
    return false;
  }

  tr = MLX90640_GetTa() - Adafruit_MLX90640::OPENAIR_TA_SHIFT; // For a MLX90640 in the open air the shift is -8  degC.

  calculate_start_time = millis();
  MLX90640_CalculateTo(emissivity, tr, framebuf);
  calculate_end_time = millis();
  ta = tr;
  return true;
}