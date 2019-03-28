
#include <stdlib.h>
#include <stdio.h>
#include "i2c.h"
#include "FDC1004.h"
#include "nrf_twi_mngr.h"
#include "assert_wrapper.h"
#include "segger_wrapper.h"



#define I2C_READ_REG(addr, p_reg_addr, p_buffer, byte_cnt) \
		NRF_TWI_MNGR_WRITE(addr, p_reg_addr, 1, NRF_TWI_MNGR_NO_STOP), \
		NRF_TWI_MNGR_READ (addr, p_buffer, byte_cnt, 0)

#define I2C_READ_REG_REP_STOP(addr, p_reg_addr, p_buffer, byte_cnt) \
		NRF_TWI_MNGR_WRITE(addr, p_reg_addr, 1, 0), \
		NRF_TWI_MNGR_READ (addr, p_buffer, byte_cnt, 0)

#define I2C_WRITE(addr, p_data, byte_cnt) \
		NRF_TWI_MNGR_WRITE(addr, p_data, byte_cnt, 0)

static uint8_t FDC1004_MEAS_CONFIG[] = { 0x08, 0x09, 0x0A, 0x0B };
static uint8_t FDC1004_MEAS_MSB[] = { 0x00, 0x02, 0x04, 0x06 };
static uint8_t FDC1004_MEAS_LSB[] = { 0x01, 0x03, 0x05, 0x07 };
static uint8_t FDC1004_capdac_values[] = { 0, 0, 0, 0 };

static int32_t m_raw_measurement;
static bool m_is_updated;

/**
 * @brief Write data directly to a register in the FDC.
 *
 * @param reg Register to write to.
 * @param data 16-bit data to write.
 */
uint8_t FDC1004_write(uint8_t reg_addr, uint16_t data) {

	uint8_t p_data[3];

	p_data[0] = reg_addr;
	p_data[1] = (data >> 8) & 0xFF;
	p_data[2] = data & 0xFF;

	nrf_twi_mngr_transfer_t const xfer[] =
	{
			I2C_WRITE(FDC1004_ADDRESS, p_data, 3)
	};

	i2c_perform(NULL, xfer, sizeof(xfer) / sizeof(xfer[0]), NULL);

	return 0;
}

/**
 * @brief Read a 16-bit register from the FDC.
 *
 * @param reg Register to read from
 * @return Data in the register
 */
uint16_t FDC1004_read(uint8_t reg_addr) {

	// Actually read the data back
	uint16_t data = 0;

	nrf_twi_mngr_transfer_t const xfer[] =
	{
			I2C_READ_REG(FDC1004_ADDRESS, &reg_addr, &data, 2)
	};

	i2c_perform(NULL, xfer, sizeof(xfer) / sizeof(xfer[0]), NULL);

	return data;
}

static void _fdc_readout_cb(ret_code_t err_code, void * p_user_data) {

	uint16_t idx = 0;
	uint16_t lsb;
	uint16_t msb;
	uint8_t *data_array = (uint8_t*)p_user_data;

	ASSERT(p_user_data);

	if (err_code) {
		LOG_WARNING("FDC1004 read error");
		return;
	}

	sChannelTrigger res;
	res.val = 0;
	res.bytes[0] =data_array[idx++] << 8;
	res.bytes[1] =data_array[idx++] & 0xFF;

	LOG_WARNING("FDC1004 REG: 0x%02X", res.val);

	msb = data_array[idx++] << 8;
	msb |= data_array[idx++] & 0xFF;

	lsb = data_array[idx++] << 8;
	lsb |= data_array[idx++] & 0xFF;

	/* Data format:
	 *    MSB[15:0] - 16 MSB of measurement
	 *    LSB[15:8] - 8 LSB of measurement
	 *    LSB[ 7:0] - Reserved, always 0
	 */
	m_raw_measurement = (msb << 8) | (lsb >> 8);

	// Convert Two's complement to signed integer (necessary since the data is shifted by 8)
	uint32_t raw_result = (msb << 16) | lsb;
	if (raw_result & ((uint32_t)1 << 31)) {
		m_raw_measurement |= ((uint32_t)0xFF << 24);
	}

	m_is_updated = true;

	LOG_DEBUG("FDC1004 read: res=%d", m_raw_measurement);

}

bool FDC1004_is_updated(void) {
	return m_is_updated;
}

void FDC1004_clear_updated(void) {
	m_is_updated = false;
}

/**
 * @brief Initialize the connection to the FDC1004.
 * @details Initialize the connection to the FDC1004 via hardware TWI. To check
 * if the connection is working properly the manufacturer and device id are
 * checked against the value given in the datasheet.
 *
 * @return 0 if successful, 1 otherwise
 */
uint8_t FDC1004_init() {

	// Check manufacturer id
	if (FDC1004_read(FDC1004_REG_MANUFACTURER_ID) != FDC1004_MANUFACTURER_ID) {
		return 1;
	}

	// Check device id
	if (FDC1004_read(FDC1004_REG_DEVICE_ID) != FDC1004_DEVICE_ID) {
		return 1;
	}

	return 0;
}

/**
 * @brief Configure a single measurement.
 * @details Write a measurement configuration to the FDC. A measurement
 * configuration consists out of the channel to use, the measurement mode and
 * the value for the CAPDAC.
 *
 * @param measurement Measurement ID (0 - 3)
 * @param channel Channel ID (0 - 3)
 * @param capdac Capacity offset (C_offset = capdac * 3.125 pF)
 * @return 0 if successful, 1 otherwise
 */
uint8_t FDC1004_configure_single_measurement(uint8_t meas, sChannelMeasurement *ch_meas) {

	ch_meas->bitfield.n_channel = 0x00u;
	FDC1004_capdac_values[meas] = ch_meas->bitfield.capdac;

	if (FDC1004_write(FDC1004_MEAS_CONFIG[meas], ch_meas->val)) {
		// Writing the configuration failed.
		return 1;
	}
	return 0;
}

/**
 * @brief Configure a differential measurement.
 * @details Write a measurement configuration to the FDC. A measurement
 * configuration consists on the channel to use, the measurement mode and the
 * value for the CAPDAC.
 *
 * @param measurement Measurement ID (0 - 3)
 * @param channel Channel ID (0 - 3)
 * @param capdac Capacity offset (C_offset = capdac * 3.125 pF)
 * @return 0 if successful, 1 otherwise
 */
uint8_t FDC1004_configure_differential_measurement(uint8_t meas, sChannelMeasurement *ch_meas) {

	ch_meas->bitfield.capdac = 0x00u;

	if (FDC1004_write(FDC1004_MEAS_CONFIG[meas], ch_meas->val)) {
		// Writing the configuration failed.
		return 1;
	}
	return 0;
}

/**
 * @brief Trigger a measurement.
 * @details Trigger a measurement. Note that the measurement is only triggered
 * here, it is not read out nor waits this function on the completion of the
 * measurement. For reading the measurement directly please use
 * FDC1004_measure_channel.
 *
 * @param measurement Measurement ID (0 - 3)
 * @return Success code
 */
uint8_t FDC1004_trigger_measurement(sChannelTrigger *trigger) {

	static uint8_t p_buffer[3] = {0};

	// MSB first
	p_buffer[0] = FDC1004_REG_FDC;
	p_buffer[1] = trigger->bytes[1];
	p_buffer[2] = trigger->bytes[0];

	static nrf_twi_mngr_transfer_t const _trigger_transfer[] =
	{
			I2C_WRITE(FDC1004_ADDRESS, p_buffer , sizeof(p_buffer))
	};

	static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
	{
			.callback            = NULL,
			.p_user_data         = NULL,
			.p_transfers         = _trigger_transfer,
			.number_of_transfers = sizeof(_trigger_transfer) / sizeof(_trigger_transfer[0])
	};

	i2c_schedule(&transaction);

	return 0;
}

/**
 * @brief Read raw measurement result.
 */
uint8_t FDC1004_read_raw_measurement(uint8_t measurement, int32_t *result) {

	static uint8_t p_ans_buffer[8] = {0};

	static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND readout_reg[3];

	readout_reg[0] = FDC1004_REG_FDC;
	readout_reg[1] = FDC1004_MEAS_MSB[measurement];
	readout_reg[2] = FDC1004_MEAS_LSB[measurement];

	static nrf_twi_mngr_transfer_t const _readout_transfer[] =
	{
			I2C_READ_REG(FDC1004_ADDRESS, readout_reg  , p_ans_buffer+0 , 2),
			I2C_READ_REG(FDC1004_ADDRESS, readout_reg+1, p_ans_buffer+2 , 2),
			I2C_READ_REG(FDC1004_ADDRESS, readout_reg+2, p_ans_buffer+4 , 2)
	};

	static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
	{
			.callback            = _fdc_readout_cb,
			.p_user_data         = p_ans_buffer,
			.p_transfers         = _readout_transfer,
			.number_of_transfers = sizeof(_readout_transfer) / sizeof(_readout_transfer[0])
	};

	i2c_schedule(&transaction);

	return 0;
}

/**
 * @brief Read a measurement result.
 * @details Read the result of a previously triggered measurement. This
 * will not wait for the measurement to be ready. If it is not ready yet,
 * it will exit with an error code.
 *
 * @param measurement Measurement ID (0 - 3)
 * @param result Result in pF
 *
 * @return Success code
 */
uint8_t FDC1004_read_measurement(uint8_t measurement, float *result) {

	// Value near lower end of the range
	if (m_raw_measurement <= FDC1004_LOWER_LIMIT) {
		return 2;
	}
	// Value near upper end of the range
	if (m_raw_measurement >= FDC1004_UPPER_LIMIT) {
		return 3;
	}

	(*result) = (float) m_raw_measurement / ((int32_t) 1 << 19)
			+ FDC1004_capdac_values[measurement] * 3.125;
	return 0;
}

/*
 * @brief Measure the capacity on a channel.
 * @details Measure the capacity on a specified channel. This will
 * override the configuration of the measurement with the same id
 * as the channel to measure.
 *
 * @param channel Channel ID
 * @param capdac Capacity offset
 * @param result Result from measurement.
 * @return Status code
 */
//uint8_t FDC1004_measure_channel(uint8_t channel, float *result) {
//	uint8_t error_code = 0;
//	while (1) {
//		FDC1004_configure_single_measurement(channel, channel,
//				FDC1004_capdac_values[channel]);
//		FDC1004_trigger_measurement(channel);
//
//		// Wait until result is ready
//		while (1) {
//			error_code = FDC1004_read_measurement(channel, result);
//			if (error_code != 1) {
//				break;
//			}
//		}
//
//		// Measurement done
//		if (error_code == 0) {
//			return 0;
//		}
//
//		// Capdac value is to high (capacity is below range)
//		if (error_code == 2) {
//			if (FDC1004_capdac_values[channel] > 0) {
//				FDC1004_capdac_values[channel] -= 1;
//			} else {
//				return 1;
//			}
//		}
//
//		// Capdac value is to low (capacity is over range)
//		if (error_code == 3) {
//			if (FDC1004_capdac_values[channel] < 31) {
//				FDC1004_capdac_values[channel] += 1;
//			} else {
//				return 1;
//			}
//		}
//	}
//	return 0;
//}
