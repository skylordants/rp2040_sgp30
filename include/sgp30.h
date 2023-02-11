#ifndef __SGP30_H__
#define __SGP30_H__

#include "pico/stdlib.h"
#include "at93c46.h"
#include "rp2040_i2c.h"


class SGP30 {
public:
	SGP30(AT93C46 *eeprom, RP2040_I2C *i2c, bool baseline_valid);
	SGP30();

	/// @brief Measure air quality. Call at 1 Hz frequency for best result. The results can be read from co2eq and tvoc variables.
	bool measure_air_quality();
	
	bool get_baseline(uint8_t baseline[]);

	bool set_baseline(uint8_t baseline[]);
	bool set_humidity(uint16_t humidity);

	uint16_t co2eq;
	uint16_t tvoc;
private:
	int read(uint16_t cmd, uint8_t *buf, const uint8_t nbytes, int delay);
	int write(uint16_t cmd, uint8_t *buf, const uint8_t nbytes);
	bool init_air_quality();
	bool update_eeprom_baseline();
	bool update_baseline_from_eeprom();

	// cal_time in seconds, it is <= 12*60*60
	uint16_t eeprom_get_cal_time();
	bool eeprom_set_cal_time(uint64_t time);

	int64_t _zerovalue_first_time; // First time a default value indicating reset was received, if -1, then the last value got was not default
	uint64_t _baseline_last_update;
	uint64_t _baseline_start_time;
	bool _baseline_valid;

	AT93C46 *_eeprom;
	RP2040_I2C *_i2c;

	static const uint8_t crc_pol;
	static uint8_t calculate_crc(uint8_t data_1, uint8_t data_2);
};

#endif