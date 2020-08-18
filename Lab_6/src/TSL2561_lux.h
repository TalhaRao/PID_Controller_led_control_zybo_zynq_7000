/*
 * TSL2561_lux.h
 *
 *  Created on: Dec 18, 2017
 *      Author: Talha
 */

#ifndef SRC_TSL2561_LUX_H_
#define SRC_TSL2561_LUX_H_


int i2c_init(u16 DeviceId);
int powerOnSensor();
int setGain(int gain);
float getLux();
void printLux(float lux);

#endif /* SRC_TSL2561_LUX_H_ */
