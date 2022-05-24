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
 * Description: Izhikevich Neural Model - Source File.
 * -------------------------------------------------------------------------------
 */
#include "stdint.h"
#include "izhikevich.h"
#include "math.h"

void set_model_parameters(Izhikevich *SP, double a1, double b1, double c1, double d1, double dt1, double V, double U)
{
	(*SP).a = a1;
	(*SP).b = b1;
	(*SP).c = c1;
	(*SP).d = d1;
	(*SP).dt = dt1;
	(*SP).Vm = V;
	(*SP).Un = U;
}

uint8_t integrate(Izhikevich *P, double input)
{
	(*P).Vm = (*P).Vm + (*P).dt*((0.04*(pow((*P).Vm, 2))) + (5*(*P).Vm) + 140 - (*P).Un + input);
	(*P).Un = (*P).Un + (*P).dt*((*P).a * ( ((*P).b * (*P).Vm)) - (*P).Un);

	if ((*P).Vm >= 30)
	{
		(*P).Vm = (*P).c;
		(*P).Un = (*P).Un + (*P).d;
		return 1;
	}
	else
	{
		return 0;
	}
}
