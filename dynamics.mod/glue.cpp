/* Copyright (c) 2015-2016 Bruce A Henderson

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

     1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.

     2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.

     3. This notice may not be removed or altered from any source
     distribution.
*/

#include "Newton.h"
#include "dMatrix.h"

extern "C" {

	#include "brl.mod/blitz.mod/blitz.h"

	NewtonWorld * bmx_newtondynamics_NewtonCreate(BBObject *);
	NewtonBody * bmx_newtondynamics_NewtonCreateDynamicBody(BBObject *, const NewtonWorld* const, const NewtonCollision* const, const dFloat* const);
	BBObject * bmx_newtondynamics_NewtonBodyGetUserData(const NewtonBody* const);

	void bmx_newtondynamics_NewtonBodySetOmega(const NewtonBody* const, float, float, float, float);
	void bmx_newtondynamics_NewtonBodySetVelocity(const NewtonBody* const, float, float, float, float);
	void bmx_newtondynamics_NewtonBodySetForce(const NewtonBody* const, float, float, float, float);
	void bmx_newtondynamics_NewtonBodySetTorque(const NewtonBody* const, float, float, float, float);
	void bmx_newtondynamics_NewtonBodySetAngularDamping(const NewtonBody* const, float, float, float);
	void bmx_newtondynamics_NewtonBodyGetAngularDamping(const NewtonBody* const, float*, float*, float*);
	void bmx_newtondynamics_NewtonBodyGetRotation(const NewtonBody* const, float *, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetOmega(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetVelocity(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetForce(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetTorque(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetForceAcc(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetTorqueAcc(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetCentreOfMass(const NewtonBody* const, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyGetAABB(const NewtonBody* const, float *, float *, float *, float *, float *, float *);
	void bmx_newtondynamics_NewtonBodyAddForce(const NewtonBody* const, float, float, float);
	void bmx_newtondynamics_NewtonBodyAddTorque(const NewtonBody* const, float, float, float);
	void bmx_newtondynamics_NewtonBodyCalculateInverseDynamicsForce(const NewtonBody* const, float, float, float, float, float *, float *, float *);

	void bmx_newtondynamics_config_body(BBObject *, NewtonBody *);
	void bmx_newtondynamics_body_destroycallback(const NewtonBody* const);

	NewtonCollision * bmx_newtondynamics_NewtonCreateSphere(BBObject *, const NewtonWorld* const, float, int, float *);
	NewtonCollision * bmx_newtondynamics_NewtonCreateBox(BBObject *, const NewtonWorld* const, float, float, float, int, float *);
	NewtonCollision * bmx_newtondynamics_NewtonCreateTreeCollision(BBObject *, const NewtonWorld* const, int);
	NewtonCollision * bmx_newtondynamics_NewtonCreateCylinder(BBObject *, const NewtonWorld* const, float, float, int, float *);
	
	void bmx_newtondynamics_collision_copycallback(const NewtonWorld* const, NewtonCollision* const, const NewtonCollision* const);
	void bmx_newtondynamics_collision_destroycallback(const NewtonWorld* const, const NewtonCollision* const);
	void bmx_newtondynamics_config_collision(BBObject *, NewtonCollision *);
	
	void bmx_newtondynamics_matrix_GetEulerAngles(float *, float *, float *, float *, float *, float *, float *);

	void bmx_newtondynamics_NewtonWorldRayCast(const NewtonWorld* const, float, float, float, float, float, float, 
		NewtonWorldRayFilterCallback, BBObject *, NewtonWorldRayPrefilterCallback, int);
	BBObject * bmx_newtondynamics_RayCastDelegateFromPtr(void * data);

}

// ********************************************************

void bmx_newtondynamics_body_destroycallback(const NewtonBody* const body) {
	BBObject * obj = (BBObject*) NewtonBodyGetUserData(body);
	if (obj) {
		BBRELEASE(obj);
	}
}

void bmx_newtondynamics_config_body(BBObject * obj, NewtonBody * body) {
	NewtonBodySetUserData(body, obj);
	BBRETAIN(obj);
	
	NewtonBodySetDestructorCallback(body, bmx_newtondynamics_body_destroycallback);
}

void bmx_newtondynamics_collision_copycallback(const NewtonWorld* const world, NewtonCollision* const coll, const NewtonCollision* const source) {
	BBObject * obj = (BBObject*) NewtonCollisionGetUserData(source);
	bmx_newtondynamics_config_collision(obj, coll);
}

void bmx_newtondynamics_collision_destroycallback(const NewtonWorld* const world, const NewtonCollision* const coll) {
	BBObject * obj = (BBObject*) NewtonCollisionGetUserData(coll);
	if (obj) {
		BBRELEASE(obj);
	}
}

void bmx_newtondynamics_config_collision(BBObject * obj, NewtonCollision * coll) {
	NewtonCollisionSetUserData(coll, obj);
	BBRETAIN(obj);
}

// ********************************************************

NewtonWorld * bmx_newtondynamics_NewtonCreate(BBObject *) {
	NewtonWorld * world = NewtonCreate();
	
	NewtonWorldSetCollisionConstructorDestructorCallback(world, bmx_newtondynamics_collision_copycallback, bmx_newtondynamics_collision_destroycallback);
	
	return world;
}

NewtonBody * bmx_newtondynamics_NewtonCreateDynamicBody(BBObject * obj, const NewtonWorld* const world, const NewtonCollision* const coll, const dFloat* const matrix) {
	NewtonBody * body = NewtonCreateDynamicBody(world, coll, matrix);
	bmx_newtondynamics_config_body(obj, body);
	return body;
}

void bmx_newtondynamics_NewtonBodySetOmega(const NewtonBody* const body, float x, float y, float z, float w) {
	dVector v(x, y, z, w);
	NewtonBodySetOmega(body, &v[0]);
}

void bmx_newtondynamics_NewtonBodySetVelocity(const NewtonBody* const body, float x, float y, float z, float w) {
	dVector v(x, y, z, w);
	NewtonBodySetVelocity(body, &v[0]);
}

void bmx_newtondynamics_NewtonBodySetForce(const NewtonBody* const body, float x, float y, float z, float w) {
	dVector v(x, y, z, w);
	NewtonBodySetForce(body, &v[0]);
}

void bmx_newtondynamics_NewtonBodySetTorque(const NewtonBody* const body, float x, float y, float z, float w) {
	dVector v(x, y, z, w);
	NewtonBodySetTorque(body, &v[0]);
}

BBObject * bmx_newtondynamics_NewtonBodyGetUserData(const NewtonBody* const body) {
	return (BBObject*) NewtonBodyGetUserData(body);
}

void bmx_newtondynamics_NewtonBodySetAngularDamping(const NewtonBody* const body, float ax, float ay, float az) {
	dVector v(ax, ay, az, 0.0f);
	NewtonBodySetAngularDamping(body, &v[0]);
}

void bmx_newtondynamics_NewtonBodyGetAngularDamping(const NewtonBody* const body, float * ax, float * ay, float * az) {
	dVector v;
	NewtonBodyGetAngularDamping(body, &v[0]);
	*ax = v.m_x;
	*ay = v.m_y;
	*az = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetRotation(const NewtonBody* const body, float * q0, float * q1, float * q2, float * q3) {
	dVector v;
	NewtonBodyGetRotation(body, &v[0]);
	*q0 = v.m_x;
	*q1 = v.m_y;
	*q2 = v.m_z;
	*q3 = v.m_w;
}

void bmx_newtondynamics_NewtonBodyGetOmega(const NewtonBody* const body, float * ox, float * oy, float * oz) {
	dVector v;
	NewtonBodyGetOmega(body, &v[0]);
	*ox = v.m_x;
	*oy = v.m_y;
	*oz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetVelocity(const NewtonBody* const body, float * vx, float * vy, float * vz) {
	dVector v;
	NewtonBodyGetVelocity(body, &v[0]);
	*vx = v.m_x;
	*vy = v.m_y;
	*vz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetForce(const NewtonBody* const body, float * fx, float * fy, float * fz) {
	dVector v;
	NewtonBodyGetForce(body, &v[0]);
	*fx = v.m_x;
	*fy = v.m_y;
	*fz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetTorque(const NewtonBody* const body, float * tx, float * ty, float * tz) {
	dVector v;
	NewtonBodyGetTorque(body, &v[0]);
	*tx = v.m_x;
	*ty = v.m_y;
	*tz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetForceAcc(const NewtonBody* const body, float * fx, float * fy, float * fz) {
	dVector v;
	NewtonBodyGetForceAcc(body, &v[0]);
	*fx = v.m_x;
	*fy = v.m_y;
	*fz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetTorqueAcc(const NewtonBody* const body, float * tx, float * ty, float * tz) {
	dVector v;
	NewtonBodyGetTorqueAcc(body, &v[0]);
	*tx = v.m_x;
	*ty = v.m_y;
	*tz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetCentreOfMass(const NewtonBody* const body, float * cx, float * cy, float * cz) {
	dVector v;
	NewtonBodyGetCentreOfMass(body, &v[0]);
	*cx = v.m_x;
	*cy = v.m_y;
	*cz = v.m_z;
}

void bmx_newtondynamics_NewtonBodyGetAABB(const NewtonBody* const body, float * p0x, float * p0y, float * p0z, float * p1x, float * p1y, float * p1z) {
	dVector v0, v1;
	NewtonBodyGetAABB(body, &v0[0], &v1[0]);
	*p0x = v0.m_x;
	*p0y = v0.m_y;
	*p0z = v0.m_z;
	*p1x = v1.m_x;
	*p1y = v1.m_y;
	*p1z = v1.m_z;
}

void bmx_newtondynamics_NewtonBodyAddForce(const NewtonBody* const body, float x, float y, float z) {
	dVector v(x, y, z, 0.0f);
	NewtonBodyAddForce(body, &v[0]);
}

void bmx_newtondynamics_NewtonBodyAddTorque(const NewtonBody* const body, float x, float y, float z) {
	dVector v(x, y, z, 0.0f);
	NewtonBodyAddTorque(body, &v[0]);
}

void bmx_newtondynamics_NewtonBodyCalculateInverseDynamicsForce(const NewtonBody* const body, float timestep, float x, float y, float z, float * fx, float * fy, float * fz) {
	dVector v(x, y, z, 0.0f);
	dVector f;
	NewtonBodyCalculateInverseDynamicsForce(body, timestep, &v[0], &f[0]);
	*fx = f.m_x;
	*fy = f.m_y;
	*fz = f.m_z;
}

// ********************************************************

NewtonCollision * bmx_newtondynamics_NewtonCreateSphere(BBObject * obj, const NewtonWorld* const world, float radius, int shapeID, float * offsetMatrix) {
	NewtonCollision * coll = NewtonCreateSphere(world, radius, shapeID, offsetMatrix);
	bmx_newtondynamics_config_collision(obj, coll);
	return coll;
}

NewtonCollision * bmx_newtondynamics_NewtonCreateBox(BBObject * obj, const NewtonWorld* const world, float dx, float dy, float dz, int shapeID, float * offsetMatrix) {
	NewtonCollision * coll = NewtonCreateBox(world, dx, dy, dz, shapeID, offsetMatrix);
	bmx_newtondynamics_config_collision(obj, coll);
	return coll;
}

NewtonCollision * bmx_newtondynamics_NewtonCreateTreeCollision(BBObject * obj, const NewtonWorld* const world, int shapeID) {
	NewtonCollision * coll = NewtonCreateTreeCollision(world, shapeID);
	bmx_newtondynamics_config_collision(obj, coll);
	return coll;
}

NewtonCollision * bmx_newtondynamics_NewtonCreateCylinder(BBObject * obj, const NewtonWorld* const world, float radius, float height, int shapeID, float * offsetMatrix) {
	NewtonCollision * cyl = NewtonCreateCylinder(world, radius, height, shapeID, offsetMatrix);
	bmx_newtondynamics_config_collision(obj, cyl);
	return cyl;
}

void bmx_newtondynamics_NewtonWorldRayCast(const NewtonWorld* const world, float p0x, float p0y, float p0z, float p1x, float p1y, float p1z, 
		NewtonWorldRayFilterCallback fcb, BBObject * delegate, NewtonWorldRayPrefilterCallback pfcb, int threadIndex) {

	dVector p0(p0x, p0y, p0z, 0.0f);
	dVector p1(p1x, p1y, p1z, 0.0f);

	NewtonWorldRayCast(world, &p0[0], &p1[0], fcb, delegate, pfcb, threadIndex);
}

// ********************************************************

void bmx_newtondynamics_matrix_GetEulerAngles(float * frontX, float * pitch0, float * yaw0, float * roll0, float * pitch1, float * yaw1, float * roll1) {
	dMatrix m(frontX);
	
	dVector v0, v1;
	m.GetEulerAngles(v0, v1, m_pitchYawRoll);
	
	*pitch0 = v0[0];
	*yaw0 = v0[1];
	*roll0 = v0[2];

	*pitch1 = v1[0];
	*yaw1 = v1[1];
	*roll1 = v1[2];
}

BBObject * bmx_newtondynamics_RayCastDelegateFromPtr(void * data) {
	return (BBObject*)data;
}

