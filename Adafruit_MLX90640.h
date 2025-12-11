/*!
 *  @file Adafruit_MLX90640.h
 *
 * 	I2C Driver for MLX90640 24x32 IR Thermal Camera
 *
 * 	This is a library for the Adafruit MLX90640 breakout:
 * 	https://www.adafruit.com/products/4407
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#pragma once

#include "Arduino.h"
#include "headers/MLX90640_API.h"
#include <Adafruit_I2CDevice.h>
#include <Wire.h>
#include <array>

 /** Mode to read pixel frames (two per image) */
enum class mlx90640_mode_t {
  INTERLEAVED, ///< Read data from camera by interleaved lines
  CHESS,       ///< Read data from camera in alternating pixels
};

/** Internal ADC resolution for pixel calculation */
enum class mlx90640_resolution_t {
  ADC_16BIT,
  ADC_17BIT,
  ADC_18BIT,
  ADC_19BIT,
};

/** How many PAGES we will read per second (2 pages per frame) */
enum class mlx90640_refreshrate_t {
  HZ_0_5,
  HZ_1,
  HZ_2,
  HZ_4,
  HZ_8,
  HZ_16,
  HZ_32,
  HZ_64,
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MLX90640 sensor
 */
class Adafruit_MLX90640 {
public:
  static constexpr uint8_t MLX90640_I2CADDR_DEFAULT = 0x33; ///< I2C address by default
  static constexpr uint16_t MLX90640_DEVICEID1 = 0x2407; ///< I2C identification register
  static constexpr int OPENAIR_TA_SHIFT = 8; ///< Default 8 degree offset from ambient air
  static constexpr float MLX90640_DEFAULT_EMISSIVITY = 0.95f; ///< Default emissivity value for temperature calculations

  int grab_start_time = 0;
  int grab_end_time = 0;
  int calculate_start_time = 0;
  int calculate_end_time = 0;

  /*!
   *    @brief  Instantiates a new MLX90640 class
   */
  Adafruit_MLX90640();
  /*!
   *    @brief  Sets up the hardware and initializes I2C
   *    @param  i2c_addr
   *            The I2C address to be used.
   *    @param  wire
   *            The Wire object to be used for I2C connections.
   *    @return True if initialization was successful, otherwise false.
   */
  boolean begin(uint8_t i2c_addr = MLX90640_I2CADDR_DEFAULT,
    TwoWire* wire = &Wire);

  /*!
   *    @brief Get the frame-read mode
   *    @return Chess or interleaved mode
   */
  mlx90640_mode_t getMode();
  /*!
   *    @brief Set the frame-read mode
   *    @param mode Chess or interleaved mode
   */
  void setMode(mlx90640_mode_t mode);
  /*!
   *    @brief  Get resolution for temperature precision
   *    @returns The desired resolution (bits)
   */
  mlx90640_resolution_t getResolution();
  /*!
   *    @brief  Set resolution for temperature precision
   *    @param res The desired resolution (bits)
   */
  void setResolution(mlx90640_resolution_t res);
  /*!
   *    @brief  Get max refresh rate
   *    @returns How many pages per second to read (2 pages per frame)
   */
  mlx90640_refreshrate_t getRefreshRate();
  /*!
   *    @brief  Set max refresh rate - too fast and we can't read the
   *    the pages in time, start low and then increment while speeding
   *    up I2C!
   *    @param rate How many pages per second to read (2 pages per frame)
   */
  void setRefreshRate(mlx90640_refreshrate_t rate);

  /*!
   *    @brief  Read 2 pages, calculate temperatures and place into framebuf
   *    @param  framebuf 24*32 floating point memory buffer
   *    @return 0 on success
   */
  int getFrame(float* framebuf);
  /*!
   *    @brief  Read 2 pages, generate image and place into framebuf
   *    @param  framebuf 24*32 floating point memory buffer
   *    @return 0 on success
   */
  int getImage(float* framebuf);
  /*!
   *    @brief  Return ambient temperature of the TO39 package.
   *    @param  newFrame If true, will also capture a new data frame. If false,
   * return the value from the last data frame read.
   *    @return Ambient temperature as a float in degrees Celsius.
   */
  float getTa(bool newFrame = true);
  /*!
   *    @brief  Read a single page of frame data, calculate temperatures and place into framebuf
   *    @param  framebuf 24*32 floating point memory buffer
   *    @return True on success, false if data was not ready or an error occurred
   *    @note   This method reads only one page. A complete frame requires 2 pages.
   *            Call this method twice to get a complete frame, or use getFrame() for a full frame.
   */
  bool updatePartialFrame(float* framebuf);
  uint16_t serialNumber[3]; ///< Unique serial number read from device

private:
  float ta = -999.0;
  /*!
   *    @brief  Read nMemAddressRead words from I2C startAddress into data
   *    @param  slaveAddr Not used - kept to maintain backcompatible API
   *    @param  startAddress I2C memory address to start reading
   *    @param  nMemAddressRead 16-bit words to read
   *    @param  data Location to place data read
   *    @return 0 on success
   */
  int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
    uint16_t nMemAddressRead, uint16_t* data);
  int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress,
    uint16_t data);
  int MLX90640_I2CGeneralReset();
  Adafruit_I2CDevice* i2c_dev;
  paramsMLX90640 _params;
  std::array<uint16_t, 834> mlx90640Frame_;

  int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t* eeData);
  int MLX90640_SynchFrame(uint8_t slaveAddr);
  int MLX90640_TriggerMeasurement(uint8_t slaveAddr);
  int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t* frameData);
  int MLX90640_ExtractParameters(uint16_t* eeData);
  float MLX90640_GetVdd();
  float MLX90640_GetTa();
  void MLX90640_GetImage(float* result);
  void MLX90640_CalculateTo(float emissivity, float tr, float* result);
  int MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution);
  int MLX90640_GetCurResolution(uint8_t slaveAddr);
  int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate);
  int MLX90640_GetRefreshRate(uint8_t slaveAddr);
  int MLX90640_GetSubPageNumber();
  int MLX90640_GetCurMode(uint8_t slaveAddr);
  int MLX90640_SetInterleavedMode(uint8_t slaveAddr);
  int MLX90640_SetChessMode(uint8_t slaveAddr);
  void MLX90640_BadPixelsCorrection(uint16_t* pixels, float* to, int mode);
  /*
  Try to read a page of data.  If data was not ready `0` is returned.
  This must be called 2x to completely get the frameData.
  */
  int MLX90640_GetFramePage(uint8_t slaveAddr, uint16_t& statusRegister);
};
