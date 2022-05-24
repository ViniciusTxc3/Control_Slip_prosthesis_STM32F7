/*
 *  -*- coding: utf-8 -*-
 *-------------------------------------------------------------------------------
 * FEDERAL UNIVERSITY OF UBERLANDIA - UFU
 * Faculty of Electrical Engineering - FEELT
 * Biomedical Engineering Lab - Biolab
 * -------------------------------------------------------------------------------
 * Author: Vinicius Teixeira da Costa
 * Contact: viniciustxc3@gmail.com
 * -------------------------------------------------------------------------------
 * Description: Izhikevich Neural Model - Header File.
 * -------------------------------------------------------------------------------
 */

#ifndef INC_IZHIKEVICH_H_
#define INC_IZHIKEVICH_H_

typedef struct
{
	double a;
	double b;
	double c;
	double d;
	double dt;
	double Vm;
	double Un;

} Izhikevich;

void set_model_parameters(Izhikevich *SP, double a1, double b1, double c1, double d1, double dt1, double V, double U);
uint8_t integrate(Izhikevich *P, double input);


#endif /* INC_IZHIKEVICH_H_ */
