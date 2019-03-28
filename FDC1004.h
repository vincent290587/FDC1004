#ifndef _FDC1004_H
#define _FDC1004_H

#include <stdint.h>

// Lower and higher range limits (if lower/higher the capdac will be changed)
#define FDC1004_LOWER_LIMIT        -8388500
#define FDC1004_UPPER_LIMIT         8388500

// Values for enabling and disabling CAPDAC in configuration.
#define FDC1004_CAPDAC              0b100UL
#define FDC1004_DISABLED            0b111UL

// Measuring rate (0b01 = 100S/s, 0b010 = 200S/s, 0b11 = 400S/s)
#define FDC1004_RATE                0b01UL

// Communication rate for I2C-Bus (in Hz)
#define SCL_CLOCK                   1000000L

// Registers
#define FDC1004_REG_FDC              0x0C
#define FDC1004_REG_MANUFACTURER_ID  0xFE
#define FDC1004_REG_DEVICE_ID        0xFF

// Device properties
#define FDC1004_MANUFACTURER_ID      0x5449
#define FDC1004_DEVICE_ID            0x1004

typedef enum {
	eConfRegRate100SPS   = 0b01,
	eConfRegRate200SPS   = 0b10,
	eConfRegRate400SPS   = 0b11,
} eConfRegRate;

/* FDC configuration register:
	 [ 15  ] = Reset
	 [14:12] = RESERVED, always 0 (RO)
	 [11:10] = Measurement rate
	 [  9  ] = RESERVED, always 0 (RO)
	 [  8  ] = Repeat
	 [ 7:4 ] = Enable measurement 1-4
	 [ 3:0 ] = Measurement done
 */
typedef struct {
	uint8_t meas_done  : 4;
	uint8_t meas_en    : 4;
	uint8_t repeat     : 1;
	uint8_t rsv1       : 1;
	uint8_t meas_rate  : 2;
	uint8_t rsv2       : 3;
	uint8_t rst        : 1;
} sChannelTriggerBitField;

typedef union {
	uint16_t val;
	sChannelTriggerBitField bitfield;
} sChannelTrigger;

typedef enum {
  eCHACIN1   = 0b000,
  eCHACIN2   = 0b001,
  eCHACIN3   = 0b010,
  eCHACIN4   = 0b011,
} eCHA;

typedef enum {
  eCHBCIN1   = eCHACIN1,
  eCHBCIN2   = eCHACIN2,
  eCHBCIN3   = eCHACIN3,
  eCHBCIN4   = eCHACIN4,
  eCHBCDAC   = 0b100,
  eCHBDis    = 0b111,
} eCHB;

/* Measurement configuration register:
	 [15:13] = Positive channel number
	 [12:10] = Negative channel number (or CAPDAC/DISABLE)
	 [ 9:5 ] = CAPDAC 0b00000 - 0b11111 (C_offset = CAPDAC * 3.125 pF)
	 [ 4:0 ] = RESERVED, always 0
 */
typedef struct {
	uint8_t reserved  : 5;
	uint8_t capdac    : 5;
	uint8_t n_channel : 3;
	uint8_t p_channel : 3;
} sChannelMeasurementBitField;

typedef union {
	uint16_t val;
	sChannelMeasurementBitField bitfield;
} sChannelMeasurement;


#ifdef	__cplusplus
extern "C" {
#endif


uint8_t FDC1004_init();

uint8_t FDC1004_configure_measurement(uint8_t meas, sChannelMeasurement *ch_meas);

uint8_t FDC1004_configure_differential_measurement(uint8_t meas, sChannelMeasurement *ch_meas);

uint8_t FDC1004_trigger_measurement(sChannelTrigger *trigger);

uint8_t FDC1004_read_raw_measurement(uint8_t measurement, int32_t *result);

uint8_t FDC1004_read_measurement(uint8_t measurement, double *result);

uint8_t FDC1004_measure_channel(uint8_t channel, double *result);


#ifdef	__cplusplus
}
#endif

#endif
