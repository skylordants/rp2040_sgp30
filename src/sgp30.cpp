#include <stdio.h>

#include "pico/time.h"

#include "sgp30.h"

#define SGP30_ADDRESS 0x58

#define SGP30_INIT_AIR_QUALITY 0x2003
#define SGP30_MEASURE_AIR_QUALITY 0x2008
#define SGP30_GET_BASELINE 0x2015
#define SGP30_SET_BASELINE 0x201e
#define SGP30_SET_HUMIDITY 0x2061
#define SGP30_MEASURE_TEST 0x2032
#define SGP30_GET_FEATURE_SET 0x202f
#define SGP30_GET_SERIAL_ID 0x3682

#define SGP30_RESTART_DELAY 10*60*1000000
#define SGP30_BASELINE_SAVE_DELAY 1*60*60*1000000ul // For testing it should be like 5*1*1000000 to not wait an hour for the first update
#define SGP30_BASELINE_VALID_DELAY 12*60*60

#define EEPROM_BASELINE_ADDRESS 0
#define EEPROM_BASELINE_LENGTH 4
#define EEPROM_TIME_ADDRESS (EEPROM_BASELINE_ADDRESS + EEPROM_BASELINE_LENGTH)

SGP30::SGP30(AT93C46 *eeprom, RP2040_I2C *i2c, bool baseline_valid)
	: co2eq(0)
	, tvoc(0)
	, _zerovalue_first_time(0)
	, _baseline_valid(baseline_valid)
	, _eeprom(eeprom)
	, _i2c(i2c)
{
	init_air_quality();
}

SGP30::SGP30() {

}

const uint8_t SGP30::crc_pol = 0x31;

// Inspiration: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
uint8_t SGP30::calculate_crc(uint8_t data_1, uint8_t data_2) {
	
	uint8_t crc = data_1 ^ 0xff;

	uint8_t data_buffer[2] = {data_2, 0x00};

	for (int b = 0; b < 2; b++) {
		for (int i = 7; i >= 0; i--) {
			bool x = crc & 0x80;

			crc <<= 1;
			crc = (data_buffer[b]&(1<<i)) ? crc|0x01 : crc&0xfe;

			if (x) {
				crc ^= crc_pol;
			}
		}
	}
	return crc;
}

int SGP30::read(uint16_t cmd, uint8_t *buf, const uint8_t nbytes, int delay) {

	uint8_t cmdbuf[2] = { (uint8_t)(cmd>>8), (uint8_t)(cmd&0x00ff) };

	_i2c->write(SGP30_ADDRESS, cmdbuf, 2);
	sleep_ms(delay);
	int num_bytes_read = _i2c->read(SGP30_ADDRESS, buf, nbytes);

	return num_bytes_read;
}

int SGP30::write(uint16_t cmd, uint8_t *buf, const uint8_t nbytes) {
	int num_bytes_written = 0;
	uint8_t msg[24] = { (uint8_t)(cmd>>8), (uint8_t)(cmd&0x00ff) };

	if (nbytes%2 != 0) {
		printf("Wrong length data");
		return 0;
	}


	for (int i = 0; i < nbytes/2; i++) {
		msg[2+3*i] = buf[2*i];
		msg[3+3*i] = buf[2*i+1];
		msg[4+3*i] = calculate_crc(buf[2*i], buf[2*i+1]);
	}

	// Write data to register(s) over I2C
	num_bytes_written = _i2c->write(SGP30_ADDRESS, msg, nbytes/2*3+2);

	return num_bytes_written;
}

bool SGP30::init_air_quality() {
	write(SGP30_INIT_AIR_QUALITY, NULL, 0);
	sleep_ms(10);

	if (_eeprom) {
		update_baseline_from_eeprom();
		_baseline_start_time = time_us_64();
		_baseline_last_update = _baseline_start_time;
	}

	_zerovalue_first_time = time_us_64();
	return true;
}


bool SGP30::measure_air_quality() {
	uint8_t data[6];

	int num_bytes_read = read(SGP30_MEASURE_AIR_QUALITY, data, 6, 15);

	if (num_bytes_read != 6) {
		_zerovalue_first_time = 0;
		return false;
	}

	uint16_t t_co2eq = ((uint16_t)data[0]<<8)|data[1];
	uint16_t t_tvoc = ((uint16_t)data[3]<<8)|data[4];


	if (t_co2eq == 400 && t_tvoc == 0){
		if (_zerovalue_first_time == -1) {
			_zerovalue_first_time = time_us_64();
		}

		if (time_us_64() - _zerovalue_first_time > SGP30_RESTART_DELAY) {
			init_air_quality();
		}
		return false;
	}
	else {
		co2eq = t_co2eq;
		tvoc = t_tvoc;

		_zerovalue_first_time = -1;

		if (time_us_64()-_baseline_last_update > SGP30_BASELINE_SAVE_DELAY) {
			update_eeprom_baseline();
			_baseline_last_update = time_us_64();
		}
	}

	return true;
}


bool SGP30::set_humidity(uint16_t humidity) {
	uint8_t data[2] = {(uint8_t)(humidity>>8), (uint8_t)humidity};
	write(SGP30_SET_HUMIDITY, data, 2);
	sleep_ms(10);
	return true;
}


// Order: CO2eq, TVOC
bool SGP30::get_baseline(uint8_t baseline[]) {
	return read(SGP30_GET_BASELINE, baseline, 6, 15) == 6;
}

// Order: TVOC, CO2eq
bool SGP30::set_baseline(uint8_t baseline[]) {
	uint8_t written_bytes = write(SGP30_SET_BASELINE, baseline, 10);
	return written_bytes == 6;
}

bool SGP30::update_eeprom_baseline() {
	if (_eeprom) {
		uint8_t buf[6];
		bool ret;
		uint8_t tries = 0;
		
		while(!(ret = get_baseline(buf)) || buf[2] != calculate_crc(buf[0], buf[1]) || buf[5] != calculate_crc(buf[3], buf[4]))
		{
			if (!ret) {
				printf("ERROR: Unable to get baseline\n");
				return false;
			}

			if (tries > 5) {
				printf("ERROR: Wrong baseline CRC, too many tries, abort\n");
				return false;
			}

			printf("Wrong baseline CRC, trying again\n");
			tries++;
		}

		uint16_t baseline[4];
		for (int i = 0; i < 2; i++) {
			baseline[i*2] = buf[i*3];
			baseline[i*2+1] = buf[i*3+1];
		}

		_eeprom->write_words(EEPROM_BASELINE_ADDRESS, baseline, 4);

		// Check if it was written correctly
		uint16_t baseline_control[4];
		_eeprom->read_words(EEPROM_BASELINE_ADDRESS, baseline_control, 4);

		for (int i = 0; i < 4; i++) {
			if (baseline[i] != baseline_control[i]) {
				eeprom_set_cal_time(0);
				_baseline_start_time = time_us_64();
				printf("ERROR: Failed writing the baseline to the EEPROM\n");
				return false;
			}
		}
		eeprom_set_cal_time((time_us_64() - _baseline_start_time)/1000000);
	}
	else {
		return false;
	}
	return true;
}

bool SGP30::update_baseline_from_eeprom() {
	if (_eeprom && _baseline_valid && eeprom_get_cal_time() >= SGP30_BASELINE_VALID_DELAY) {
		uint16_t baseline[4];
		uint8_t buf[6];
		
		_eeprom->read_words(EEPROM_BASELINE_ADDRESS, baseline, 4);

		for (int i = 0; i < 2; i++) {
			buf[(i*3+3)%6] = baseline[i*2];
			buf[(i*3+4)%6] = baseline[i*2+1];
			buf[(i*3+4)%6] = calculate_crc(baseline[i*2], baseline[i*2+1]);
		}

		return set_baseline(buf);
	}
	else {
		_baseline_valid = false;
		return false;
	}

	return true;
}

uint16_t SGP30::eeprom_get_cal_time() {
	if (_eeprom) {
		uint16_t time_buf[2];

		_eeprom->read_words(EEPROM_TIME_ADDRESS, time_buf, 2);

		uint16_t time = (time_buf[0]<<8)|(time_buf[1]&0xff);
		return time;
	}

	return 0;
}

bool SGP30::eeprom_set_cal_time(uint64_t time) {
	if (_eeprom) {
		if (time > SGP30_BASELINE_VALID_DELAY) {
			_baseline_valid = true;
			time = SGP30_BASELINE_VALID_DELAY;
		}

		uint16_t time_buf[2] = {(uint16_t)((time>>8)&0xff), (uint16_t)(time&0xff)};

		_eeprom->write_words(EEPROM_TIME_ADDRESS, time_buf, 2);
	}
	else {
		return false;
	}

	return true;
}
