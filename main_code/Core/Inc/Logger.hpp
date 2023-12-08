/*
 * Logger.hpp
 *
 *  Created on: Jun 22, 2023
 *      Author: under
 */

#ifndef INC_LOGGER_HPP_
#define INC_LOGGER_HPP_

#include "sdCard.hpp"

class Logger{

private:
	sdCard *sd_card_;
	uint16_t log_idx_;
	float *logs_;
	bool recording_flag_;
	uint16_t max_log_size_;

public:
	Logger(sdCard *, uint16_t);

	void storeLogs(float);
	void saveLogs(const char *, const char *);
	void clearLogs();
	void start();
	void stop();
	uint16_t getLoggedSize();

	const float *getLogsPointer();
	void  importLatestLogs(const char *, const char * );
	uint16_t getLogsSize();
	float getLogData(uint16_t);

};





#endif /* INC_LOGGER_HPP_ */
