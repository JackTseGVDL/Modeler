#include <windows.h>
#include <Fl/gl.h>
#include <gl/glu.h>

#include "camera.h"

#pragma warning(push)
#pragma warning(disable : 4244)


#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502
#endif 


const float kMouseRotationSensitivity		= 1.0f/90.0f;
const float kMouseTranslationXSensitivity	= 0.03f;
const float kMouseTranslationYSensitivity	= 0.03f;
const float kMouseZoomSensitivity			= 0.08f;


// Static Function Prototype
void MakeDiagonal	(Mat4f& m, float k);
void MakeHScale		(Mat4f& m, const Vec3f& s);
void MakeHTrans		(Mat4f& m, const Vec3f& s);
void MakeHRotX		(Mat4f& m, float theta);
void MakeHRotY		(Mat4f& m, float theta);
void MakeHRotZ		(Mat4f& m, float theta);
void Normalize		(double *dst);
void Cross			(double *dst, double *src_1, double *src_2);

// Static Function Implementation
void MakeDiagonal(Mat4f &m, float k) {
	register int i,j;

	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			m[i][j] = (i==j) ? k : 0.0f;
}


void MakeHScale(Mat4f &m, const Vec3f &s) {
	MakeDiagonal(m,1.0f);
	m[0][0] = s[0]; m[1][1] = s[1];	m[2][2] = s[2];
}


void MakeHTrans(Mat4f &m, const Vec3f &s) {
	MakeDiagonal(m,1.0f);
	m[0][3] = s[0]; m[1][3] = s[1]; m[2][3] = s[2];
}


void MakeHRotX(Mat4f &m, float theta) {
	MakeDiagonal(m,1.0f);
	float cosTheta = cos(theta);
	float sinTheta = sin(theta);
	m[1][1] = cosTheta;
	m[1][2] = -sinTheta;
	m[2][1] = sinTheta;
	m[2][2] = cosTheta;
}


void MakeHRotY(Mat4f &m, float theta) {
	MakeDiagonal(m,1.0f);
	float cosTheta = cos(theta);
	float sinTheta = sin(theta);
	m[0][0] = cosTheta;
	m[2][0] = -sinTheta;
	m[0][2] = sinTheta;
	m[2][2] = cosTheta;
}


void MakeHRotZ(Mat4f &m, float theta) {
	MakeDiagonal(m,1.0f);
	float cosTheta = cos(theta);
	float sinTheta = sin(theta);
	m[0][0] = cosTheta;
	m[0][1] = -sinTheta;
	m[1][0] = sinTheta;
	m[1][1] = cosTheta;
}


void Normalize(double* dst) {
	double temp = sqrt(dst[0]* dst[0] + dst[1] * dst[1] + dst[2] * dst[2]);
	dst[0] = dst[0] / temp;
	dst[1] = dst[1] / temp;
	dst[2] = dst[2] / temp;
}


void Cross(double* dst, double* src_1, double* src_2) {
	dst[0] = src_1[1] * src_2[2] - src_1[2] * src_2[1];
	dst[1] = src_1[2] * src_2[0] - src_1[0] * src_2[2];
	dst[2] = src_1[0] * src_2[1] - src_1[1] * src_2[0];
}


// Class Function
void Camera::calculateViewingTransformParameters() {
	Mat4f dollyXform;
	Mat4f azimXform;
	Mat4f elevXform;
	Mat4f twistXform;
	Mat4f originXform;

	Vec3f upVector;

	MakeHTrans(dollyXform, Vec3f(0,0,mDolly));
	MakeHRotY(azimXform, mAzimuth);
	MakeHRotX(elevXform, mElevation);
	MakeDiagonal(twistXform, 1.0f);
	MakeHTrans(originXform, mLookAt);
	
	mPosition = Vec3f(0,0,0);
	// grouped for (mat4 * vec3) ops instead of (mat4 * mat4) ops
	mPosition = originXform * (azimXform * (elevXform * (dollyXform * mPosition)));

	if ( fmod((double)mElevation, 2.0*M_PI) < 3*M_PI/2 && fmod((double)mElevation, 2.0*M_PI) > M_PI/2 )
		mUpVector= Vec3f(0,-1,0);
	else
		mUpVector= Vec3f(0,1,0);

	mDirtyTransform = false;
}


Camera::Camera() {
	mElevation = mAzimuth = mTwist = 0.0f;
	mDolly = -20.0f;
	mElevation = 0.2f;
	mAzimuth = (float)M_PI;

	mLookAt = Vec3f( 0, 0, 0 );
	mCurrentMouseAction = kActionNone;

	calculateViewingTransformParameters();
}


void Camera::clickMouse( MouseAction_t action, int x, int y ) {
	mCurrentMouseAction = action;
	mLastMousePosition[0] = x;
	mLastMousePosition[1] = y;
}


void Camera::dragMouse( int x, int y ) {
	Vec3f mouseDelta   = Vec3f(x,y,0.0f) - mLastMousePosition;
	mLastMousePosition = Vec3f(x,y,0.0f);

	switch(mCurrentMouseAction)
	{
	case kActionTranslate:
		{
			calculateViewingTransformParameters();

			double xTrack =  -mouseDelta[0] * kMouseTranslationXSensitivity;
			double yTrack =  mouseDelta[1] * kMouseTranslationYSensitivity;

			Vec3f transXAxis = mUpVector ^ (mPosition - mLookAt);
			transXAxis /= sqrt((transXAxis*transXAxis));
			Vec3f transYAxis = (mPosition - mLookAt) ^ transXAxis;
			transYAxis /= sqrt((transYAxis*transYAxis));

			setLookAt(getLookAt() + transXAxis*xTrack + transYAxis*yTrack);
			
			break;
		}
	case kActionRotate:
		{
			float dAzimuth		=   -mouseDelta[0] * kMouseRotationSensitivity;
			float dElevation	=   mouseDelta[1] * kMouseRotationSensitivity;
			
			setAzimuth(getAzimuth() + dAzimuth);
			setElevation(getElevation() + dElevation);
			
			break;
		}
	case kActionZoom:
		{
			float dDolly = -mouseDelta[1] * kMouseZoomSensitivity;
			setDolly(getDolly() + dDolly);
			break;
		}
	case kActionTwist:
		// Not implemented
	default:
		break;
	}

}


void Camera::releaseMouse( int x, int y ) {
	mCurrentMouseAction = kActionNone;
}


void Camera::applyViewingTransform() {
	if(mDirtyTransform) calculateViewingTransformParameters();

	// something useful:
	// https://www.khronos.org/opengl/wiki/GluLookAt_code
	// Place the camera at mPosition, aim the camera at
	// mLookAt, and twist the camera such that mUpVector is up
	// gluLookAt(mPosition[0], mPosition[1], mPosition[2], mLookAt[0], mLookAt[1], mLookAt[2], mUpVector[0], mUpVector[1], mUpVector[2]);

	double eye[3];
	double at[3];
	double up[3];

	eye[0] = mPosition[0];
	eye[1] = mPosition[1];
	eye[2] = mPosition[2];

	at[0] = mLookAt[0];
	at[1] = mLookAt[1];
	at[2] = mLookAt[2];

	up[0] = mUpVector[0];
	up[1] = mUpVector[1];
	up[2] = mUpVector[2];

	lookAt(eye, at, up);
}


void Camera::lookAt(double *eye, double *at, double *up) {
	double result[16];
	memset(result, 0, sizeof(result));
	result[15] = 1;

	double forward[3];
	forward[0] = at[0] - eye[0];
	forward[1] = at[1] - eye[1];
	forward[2] = at[2] - eye[2];
	Normalize(forward);

	double side[3];
	Cross(side, forward, up);
	Normalize(side);

	double up_2[3];
	Cross(up_2, side, forward);

	result[0] = side[0];
	result[4] = side[1];
	result[8] = side[2];

	result[1] = up_2[0];
	result[5] = up_2[1];
	result[9] = up_2[2];

	result[2] = -forward[0];
	result[6] = -forward[1];
	result[10] = -forward[2];

	glLoadMatrixd(result);
	glTranslated(-eye[0], -eye[1], -eye[2]);
}


#pragma warning(pop)
