/*
 * lux.c
 *
 *  Created on: Sep 25, 2024
 *      Author: Krit0
 */
#include <math.h>
#include "lux.h"

int to_lux(int raw) {
	double ldr_val = raw + f(raw);
	double v_out = ldr_val * (3.3 / 4096.0);
	double r_ldr = (10000 * (3.3 - v_out)) / v_out;
	int lux = 500 / (r_ldr / 1000);

	return lux;
}

double f(int x) {
	return 2.202196968876e+02
			+  3.561383996027e-01 * x
	        +  1.276218788985e-04 * pow(x, 2)
	        + -3.470360275448e-07 * pow(x, 3)
	        +  2.082790802069e-10 * pow(x, 4)
	        + -5.306931174991e-14 * pow(x, 5)
	        +  4.787659214703e-18 * pow(x, 6);
}
