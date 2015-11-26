/*
 * myMath.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: arthur
 */

#include "myMath.h"


void cross_product(Vector3D *A, Vector3D *B, Vector3D *out){
	out->x = A->y*B->z - A->z*B->y;
	out->y = A->z*B->x - A->x*B->z;
	out->z = A->x*B->y - A->y*B->x;
}

double scalar_product3D(Vector3D *A, Vector3D *B){
	return (A->x*B->x + A->y*B->y + A->z*B->z);
}
double scalar_product4D(Vector4D *A, Vector4D *B){
	return (A->x*B->x + A->y*B->y + A->z*B->z + A->scale*B->scale);
}

void normalize3DVector(Vector3D *vec){
	double norm = (double) fastInverseSQRT((float)(scalar_product3D(vec,vec)));
	vec->x *= norm;
	vec->y *= norm;
	vec->z *= norm;
}
void normalize4DVector(Vector4D *vec){
	double norm = (double) fastInverseSQRT((float)(scalar_product4D(vec,vec)));
	vec->x *= norm;
	vec->y *= norm;
	vec->z *= norm;
	vec->scale *= norm;
}
void normalizeQuaternion(Quaternion *q){
	Vector4D t;
	t.x = q->q0;
	t.y = q->q.x;
	t.z = q->q.y;
	t.scale = q->q.z;
	double norm = (double)fastInverseSQRT((float)scalar_product4D(&t,&t));
	q->q0 *= norm;
	q->q.x *= norm;
	q->q.y *= norm;
	q->q.z *= norm;

}


void add3DVector(Vector3D *A, Vector3D *B, Vector3D *out){
	out->x = A->x + B->x;
	out->y = A->y + B->y;
	out->z = A->z + B->z;
}


float fastInverseSQRT(float number){
	// from Quake III Arena Source Code -> calulation 4x faster, loose of accuracy <0.2%
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
	return y;
}
