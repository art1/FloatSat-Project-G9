/*
 * myMath.h
 *
 *  Created on: Nov 25, 2015
 *      Author: arthur
 */

#ifndef HARDWARE_FILTER_MYMATH_H_
#define HARDWARE_FILTER_MYMATH_H_

#include "../../../basic.h"


void cross_product(Vector3D *A, Vector3D *B, Vector3D *out);
double scalar_product3D(Vector3D *A, Vector3D *B);
double scalar_product4D(Vector4D *A, Vector4D *B);
void normalize3DVector(Vector3D *vec);
void normalize4DVector(Vector4D *vec);
void add3DVector(Vector3D *A, Vector3D *B, Vector3D *out);
float fastInverseSQRT(float number);


#endif /* HARDWARE_FILTER_MYMATH_H_ */
