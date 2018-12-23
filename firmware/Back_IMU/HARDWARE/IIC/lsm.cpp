#include <lsm.h>
#include <math.h>
#include "iic_imu1.h"
#include "iic_soft.h"
#include "iic.h"
// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define D_SA0_HIGH_ADDRESS              0b0011101 // D with SA0 high
#define D_SA0_LOW_ADDRESS               0b0011110 // D with SA0 low or non-D magnetometer
#define NON_D_MAG_ADDRESS               0b0011110 // D with SA0 low or non-D magnetometer
#define NON_D_ACC_SA0_LOW_ADDRESS       0b0011000 // non-D accelerometer with SA0 low
#define NON_D_ACC_SA0_HIGH_ADDRESS      0b0011001 // non-D accelerometer with SA0 high

#define TEST_REG_NACK -1

#define D_WHO_ID    0x49
#define DLM_WHO_ID  0x3C

// Constructors ////////////////////////////////////////////////////////////////

LSM303::LSM303(void)
{
  /*
  These values lead to an assumed magnetometer bias of 0.
  Use the Calibrate example program to determine appropriate values
  for your particular unit. The Heading example demonstrates how to
  adjust these values in your own sketch.
  */
  m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readMag(), or read() since the last call to timeoutOccurred()?
bool LSM303::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;

}

void LSM303::setTimeout(unsigned int timeout)
{
  io_timeout = timeout;
}

unsigned int LSM303::getTimeout()
{
  return io_timeout;
}

bool LSM303::init(deviceType device, sa0State sa0)
{
  // determine device type if necessary
  if (device == device_auto)
  {
    if (testReg(D_SA0_HIGH_ADDRESS, WHO_AM_I) == D_WHO_ID)
    {
      // device responds to address 0011101 with D ID; it's a D with SA0 high
      device = device_D;
      sa0 = sa0_high;
    }
    else if (testReg(D_SA0_LOW_ADDRESS, WHO_AM_I) == D_WHO_ID)
    {
      // device responds to address 0011110 with D ID; it's a D with SA0 low
      device = device_D;
      sa0 = sa0_low;
    }
    // Remaining possibilities: DLHC, DLM, or DLH. DLHC seems to respond to WHO_AM_I request the
    // same way as DLM, even though this register isn't documented in its datasheet, so instead,
    // guess if it's a DLHC based on acc address (Pololu boards pull SA0 low on DLM and DLH;
    // DLHC doesn't have SA0 but uses same acc address as DLH/DLM with SA0 high).
    else if (testReg(NON_D_ACC_SA0_HIGH_ADDRESS, CTRL_REG1_A) != TEST_REG_NACK)
    {
      // device responds to address 0011001; guess that it's a DLHC
      device = device_DLHC;
      sa0 = sa0_high;
    }
    // Remaining possibilities: DLM or DLH. Check acc with SA0 low address to make sure it's responsive
    else if (testReg(NON_D_ACC_SA0_LOW_ADDRESS, CTRL_REG1_A) != TEST_REG_NACK)
    {
      // device responds to address 0011000 with DLM ID; guess that it's a DLM
      sa0 = sa0_low;

      // Now check WHO_AM_I_M
      if (testReg(NON_D_MAG_ADDRESS, WHO_AM_I_M) == DLM_WHO_ID)
      {
        device = device_DLM;
      }
      else
      {
        device = device_DLH;
      }
    }
    else
    {
      // device hasn't responded meaningfully, so give up
      return false;
    }
  }

  // determine SA0 if necessary
  if (sa0 == sa0_auto)
  {
    if (device == device_D)
    {
      if (testReg(D_SA0_HIGH_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        sa0 = sa0_high;
      }
      else if (testReg(D_SA0_LOW_ADDRESS, WHO_AM_I) == D_WHO_ID)
      {
        sa0 = sa0_low;
      }
      else
      {
        // no response on either possible address; give up
        return false;
      }
    }
    else if (device == device_DLM || device == device_DLH)
    {
      if (testReg(NON_D_ACC_SA0_HIGH_ADDRESS, CTRL_REG1_A) != TEST_REG_NACK)
      {
        sa0 = sa0_high;
      }
      else if (testReg(NON_D_ACC_SA0_LOW_ADDRESS, CTRL_REG1_A) != TEST_REG_NACK)
      {
        sa0 = sa0_low;
      }
      else
      {
        // no response on either possible address; give up
        return false;
      }
    }
  }

  _device = device;

  // set device addresses and translated register addresses
  switch (device)
  {
    case device_D:
      acc_address = mag_address = (sa0 == sa0_high) ? D_SA0_HIGH_ADDRESS : D_SA0_LOW_ADDRESS;
      translated_regs[-OUT_X_L_M] = D_OUT_X_L_M;
      translated_regs[-OUT_X_H_M] = D_OUT_X_H_M;
      translated_regs[-OUT_Y_L_M] = D_OUT_Y_L_M;
      translated_regs[-OUT_Y_H_M] = D_OUT_Y_H_M;
      translated_regs[-OUT_Z_L_M] = D_OUT_Z_L_M;
      translated_regs[-OUT_Z_H_M] = D_OUT_Z_H_M;
      break;

    case device_DLHC:
      acc_address = NON_D_ACC_SA0_HIGH_ADDRESS; // DLHC doesn't have SA0 but uses same acc address as DLH/DLM with SA0 high
      mag_address = NON_D_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLHC_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLHC_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLHC_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLHC_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLHC_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLHC_OUT_Z_L_M;
      break;

    case device_DLM:
      acc_address = (sa0 == sa0_high) ? NON_D_ACC_SA0_HIGH_ADDRESS : NON_D_ACC_SA0_LOW_ADDRESS;
      mag_address = NON_D_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLM_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLM_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLM_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLM_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLM_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLM_OUT_Z_L_M;
      break;

    case device_DLH:
      acc_address = (sa0 == sa0_high) ? NON_D_ACC_SA0_HIGH_ADDRESS : NON_D_ACC_SA0_LOW_ADDRESS;
      mag_address = NON_D_MAG_ADDRESS;
      translated_regs[-OUT_X_H_M] = DLH_OUT_X_H_M;
      translated_regs[-OUT_X_L_M] = DLH_OUT_X_L_M;
      translated_regs[-OUT_Y_H_M] = DLH_OUT_Y_H_M;
      translated_regs[-OUT_Y_L_M] = DLH_OUT_Y_L_M;
      translated_regs[-OUT_Z_H_M] = DLH_OUT_Z_H_M;
      translated_regs[-OUT_Z_L_M] = DLH_OUT_Z_L_M;
      break;
  }
  return true;
}

/*
Enables the LSM303's accelerometer and magnetometer. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and +/- 1.3 gauss for magnetometer
  (+/- 4 gauss on LSM303D).
- Selects 50 Hz ODR (output data rate) for accelerometer and 7.5 Hz
  ODR for magnetometer (6.25 Hz on LSM303D). (These are the ODR
  settings for which the electrical characteristics are specified in
  the datasheets.)
- Enables high resolution modes (if available).
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM303::enableDefault(void)
{

  if (_device == device_D)
  {
    // Accelerometer

    // 0x57 = 0b01010111
    // AFS = 0 (+/- 2 g full scale)
    writeReg(CTRL2, 0x00);

    // 0x57 = 0b01010111
    // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    writeReg(CTRL1, 0x57);

    // Magnetometer

    // 0x64 = 0b01100100
    // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
    writeReg(CTRL5, 0x64);

    // 0x20 = 0b00100000
    // MFS = 01 (+/- 4 gauss full scale)
    writeReg(CTRL6, 0x00);

    // 0x00 = 0b00000000
    // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
    writeReg(CTRL7, 0x00);
  }
  else if (_device == device_DLHC)
  {
    // Accelerometer

    // 0x08 = 0b00001000
    // FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
    writeAccReg(CTRL_REG4_A, 0x08);

    // 0x47 = 0b01000111
    // ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
    writeAccReg(CTRL_REG1_A, 0x47);

    // Magnetometer

    // 0x0C = 0b00001100
    // DO = 011 (7.5 Hz ODR)
    writeMagReg(CRA_REG_M, 0x0C);

    // 0x20 = 0b00100000
    // GN = 001 (+/- 1.3 gauss full scale)
    writeMagReg(CRB_REG_M, 0x20);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeMagReg(MR_REG_M, 0x00);
  }
  else // DLM, DLH
  {
    // Accelerometer

    // 0x00 = 0b00000000
    // FS = 00 (+/- 2 g full scale)
    writeAccReg(CTRL_REG4_A, 0x00);

    // 0x27 = 0b00100111
    // PM = 001 (normal mode); DR = 00 (50 Hz ODR); Zen = Yen = Xen = 1 (all axes enabled)
    writeAccReg(CTRL_REG1_A, 0x27);

    // Magnetometer

    // 0x0C = 0b00001100
    // DO = 011 (7.5 Hz ODR)
    writeMagReg(CRA_REG_M, 0x0C);

    // 0x20 = 0b00100000
    // GN = 001 (+/- 1.3 gauss full scale)
    writeMagReg(CRB_REG_M, 0x60);//111=0xe0

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeMagReg(MR_REG_M, 0x00);
  }
}

// Writes an accelerometer register
void LSM303::writeAccReg(regAddr reg, byte value)
{
  Wire.beginTransmission(acc_address);
  Wire.write((byte)reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads an accelerometer register
byte LSM303::readAccReg(regAddr reg)
{
  byte value;

  Wire.beginTransmission(acc_address);
  Wire.write((byte)reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Writes a magnetometer register
void LSM303::writeMagReg(regAddr reg, byte value)
{
  Wire.beginTransmission(mag_address);
  Wire.write((byte)reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads a magnetometer register
byte LSM303::readMagReg(regAddr reg)
{
  byte value;

  // if dummy register address (magnetometer Y/Z), look up actual translated address (based on device type)
  if (reg < 0)
  {
    reg = translated_regs[-reg];
  }

  Wire.beginTransmission(mag_address);
  Wire.write((byte)reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void LSM303::writeReg(regAddr reg, byte value)
{
  // mag address == acc_address for LSM303D, so it doesn't really matter which one we use.
  // Use writeMagReg so it can translate OUT_[XYZ]_[HL]_M
  if (_device == device_D || reg < CTRL_REG1_A)
  {
    writeMagReg(reg, value);
  }
  else
  {
    writeAccReg(reg, value);
  }
}

// Note that this function will not work for reading TEMP_OUT_H_M and TEMP_OUT_L_M on the DLHC.
// To read those two registers, use readMagReg() instead.
byte LSM303::readReg(regAddr reg)
{
  // mag address == acc_address for LSM303D, so it doesn't really matter which one we use.
  // Use writeMagReg so it can translate OUT_[XYZ]_[HL]_M
  if (_device == device_D || reg < CTRL_REG1_A)
  {
    return readMagReg(reg);
  }
  else
  {
    return readAccReg(reg);
  }
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303::readAcc(void)
{
  Wire.beginTransmission(acc_address);
  // assert the MSB of the address to get the accelerometer
  // to do slave-transmit subaddress updating.
  Wire.write(OUT_X_L_A | (1 << 7));
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes
  // This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
  // (12-bit resolution, left-aligned). The D has 16-bit resolution
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303::readMag(void)
{
  Wire.beginTransmission(mag_address);
  // If LSM303D, assert MSB to enable subaddress updating
  // OUT_X_L_M comes first on D, OUT_X_H_M on others
  Wire.write((_device == device_D) ? translated_regs[-OUT_X_L_M] | (1 << 7) : translated_regs[-OUT_X_H_M]);
  last_status = Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  byte xlm, xhm, ylm, yhm, zlm, zhm;

  if (_device == device_D)
  {
    /// D: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    xlm = Wire.read();
    xhm = Wire.read();
    ylm = Wire.read();
    yhm = Wire.read();
    zlm = Wire.read();
    zhm = Wire.read();
  }
  else
  {
    // DLHC, DLM, DLH: X_H, X_L...
    xhm = Wire.read();
    xlm = Wire.read();

    if (_device == device_DLH)
    {
      // DLH: ...Y_H, Y_L, Z_H, Z_L
      yhm = Wire.read();
      ylm = Wire.read();
      zhm = Wire.read();
      zlm = Wire.read();
    }
    else
    {
      // DLM, DLHC: ...Z_H, Z_L, Y_H, Y_L
      zhm = Wire.read();
      zlm = Wire.read();
      yhm = Wire.read();
      ylm = Wire.read();
    }
  }

  // combine high and low bytes
  m.x = (int16_t)(xhm << 8 | xlm);
  m.y = (int16_t)(yhm << 8 | ylm);
  m.z = (int16_t)(zhm << 8 | zlm);
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void)
{
  readAcc();
  readMag();
}

/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silkscreen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
*/
float LSM303::heading(void)
{
  if (_device == device_D)
  {
    return heading((vector<int>){1, 0, 0});
  }
  else
  {
    return heading((vector<int>){0, -1, 0});
  }
}

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
template <typename T> float LSM303::heading(vector<T> from)
{
    vector<int32_t> temp_m = {m.x, m.y, m.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

    // compute E and N
    vector<float> E;
    vector<float> N;
    vector_cross(&temp_m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
    if (heading < 0) heading += 360;
    return heading;
}

template <typename Ta, typename Tb, typename To> void LSM303::vector_cross(const vector<Ta> *a,const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LSM303::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void LSM303::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int LSM303::testReg(byte address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  last_status = Wire.endTransmission();

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
    return Wire.read();
  else
    return TEST_REG_NACK;
}