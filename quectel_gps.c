#include "quectel_gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "quectel_define.h"

#include "nrf_gpio.h"
#include "app_uart.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define NEMA_DATA_RX_BUFFER_SIZE 255
#define NMEA_WRITE_READ_DATA_SIZE 255

static quectel_gps_data_t quectel_gps_data;
static quectel_gps_callback quectel_callback;

static uint8_t rxSize = 0;
static char rxBuffer[NMEA_WRITE_READ_DATA_SIZE] = { 0x00 };

static uint8_t nmeaDataRxSize = 0;
static char nemaDataRxBuffer[NEMA_DATA_RX_BUFFER_SIZE] = { 0x00 };

static uint32_t POWER_GPIO_PIN = 0;

void set_quectel_gps_callback(quectel_gps_callback callback) {

	quectel_callback = callback;

}

void set_quectel_gps_gpio_instance(uint32_t POWER_PIN) {

	POWER_GPIO_PIN = POWER_PIN;

}

static void wait_100_ms() {

	quectel_gps_timer = 0x00;

	while (!quectel_gps_timer) {

		if (NRF_LOG_PROCESS() == false) {

			nrf_pwr_mgmt_run();

		}

	}

}

static void wait_100_ms_timer(uint16_t wait) {

	for (uint16_t i = 0; i < wait; i++) {

		wait_100_ms();

	}

}

static uint8_t quectel_uart_data_send(char *data, uint16_t size, uint16_t wait) {

	quectel_gps_data.ack = 0x00;

	for (uint16_t i = 0; i < size; i++) {

		while (app_uart_put(data[i]) != NRF_SUCCESS) {

		}

	}

	for (uint16_t i = 0; i < wait; i++) {

		if (quectel_gps_data.ack) {

			return 0x01;

		}

		wait_100_ms();

	}

	return 0x00;

}

uint8_t set_quectel_gps_power_on(uint16_t wait) {

	memset(&quectel_gps_data, 0x00, sizeof(quectel_gps_data_t));

	nrf_gpio_pin_set(POWER_GPIO_PIN);

	for (uint16_t i = 0; i < wait; i++) {

		if (quectel_gps_data.gps_start_state) {

			return 0x01;

		}

		wait_100_ms();

	}

	return 0x00;

}

void set_quectel_gps_power_off() {

	quectel_gps_data.gps_start_state = 0x00;

	nrf_gpio_pin_clear(POWER_GPIO_PIN);

	wait_100_ms_timer(10);

}

uint8_t set_quectel_gps_sbas_disable(uint16_t wait) {

	return quectel_uart_data_send(PMTK_SBAS_DISABLE, sizeof(PMTK_SBAS_DISABLE),
			wait);

}

uint8_t set_quectel_gps_nmea_off_gga_only(uint16_t wait) {

	return quectel_uart_data_send(PMTK_NEMA_OUTPUT_DISABLE_GGA_ONLY,
			sizeof(PMTK_NEMA_OUTPUT_DISABLE_GGA_ONLY), wait);

}

static uint8_t nmea_crc_update(char *data, uint16_t size) {

	uint8_t crc = 0;
	char convert_crc[4];

	for (uint16_t i = 1; i < size - 3; i++) {

		crc ^= data[i];

	}

	sprintf(convert_crc, "%02X", crc);

	if (strncmp(convert_crc, data + (size - 2), 2) == 0) {

		return 0x01;

	}

	return 0x00;

}

void quectel_gps_nmea_input(char data) {

	rxBuffer[rxSize++] = data;

	if (rxSize > 2) {

		if ((rxBuffer[rxSize - 1] == '\n') && (rxBuffer[rxSize - 2] == '\r')) {

			if (nmea_crc_update(rxBuffer, rxSize - 2)) {

				if (strncmp("$GPGGA", (char*) rxBuffer, 5) == 0) {

					uint8_t step = 0;

					nmeaDataRxSize = 0;
					memset(nemaDataRxBuffer, 0x00, sizeof(nemaDataRxBuffer));

					for (uint16_t i = 0; i < rxSize; i++) {

						if (rxBuffer[i] == ',') {

							if (step == 2) {

								float lat = atof(nemaDataRxBuffer);
								quectel_gps_data.latitude = lat * 100000;

							} else if (step == 3) {

								if (nemaDataRxBuffer[0] == 'S') {

									quectel_gps_data.latitude *= -1;

								}

								NRF_LOG_INFO("Quectel_GPS LAT %d",
										quectel_gps_data.latitude);

							} else if (step == 4) {

								float lon = atof(nemaDataRxBuffer);
								quectel_gps_data.longitude = lon * 100000;

							} else if (step == 5) {

								if (nemaDataRxBuffer[0] == 'W') {

									quectel_gps_data.longitude *= -1;

								}

								NRF_LOG_INFO("Quectel_GPS LON %d",
										quectel_gps_data.longitude);

							} else if (step == 6) {

								quectel_gps_data.navigation_statue = atoi(
										nemaDataRxBuffer);

								NRF_LOG_INFO("Quectel_GPS Navigaton state %d",
										quectel_gps_data.navigation_statue);

							} else if (step == 7) {

								quectel_gps_data.satellites_number = atoi(
										nemaDataRxBuffer);

								NRF_LOG_INFO("Quectel_GPS Navigaton state %d",
										quectel_gps_data.navigation_statue);

							} else if (step == 8) {

								float HDOP = atof(nemaDataRxBuffer);
								HDOP *= 10;
								quectel_gps_data.HDOP = (uint8_t) HDOP;

								NRF_LOG_INFO("Quectel_GPS HDOP %d",
										quectel_gps_data.HDOP);

							} else if (step == 9) {

								quectel_gps_data.height = atof(
										nemaDataRxBuffer);

								NRF_LOG_INFO("Quectel_GPS Height %d",
										quectel_gps_data.height);

							} else if (step >= 10) {

								if (quectel_callback) {

									NRF_LOG_INFO("Quectel_GPS Get %s", rxBuffer);
									NRF_LOG_FLUSH();

									quectel_gps_data.gps_start_state = 0x01;

									quectel_callback(quectel_gps_data);

								}

								break;

							}

							nmeaDataRxSize = 0;
							memset(nemaDataRxBuffer, 0x00,
									sizeof(nemaDataRxBuffer));

							step++;

						} else {

							nemaDataRxBuffer[nmeaDataRxSize++] = rxBuffer[i];

							if (nmeaDataRxSize >= sizeof(nemaDataRxBuffer)) {

								nmeaDataRxSize = 0;

							}

						}

					}

				} else if (strncmp("$PMTK001,314,3", (char*) rxBuffer, 14)
						== 0) {

					quectel_gps_data.ack = 0x01;

					if (quectel_callback) {

						quectel_callback(quectel_gps_data);

					}

				}

			}

			rxSize = 0;

		}

	}

	if (rxSize >= sizeof(rxBuffer)) {

		rxSize = 0;

	}

}
