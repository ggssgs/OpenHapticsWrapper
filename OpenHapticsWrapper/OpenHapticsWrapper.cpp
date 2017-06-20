// This is the main DLL file.

#include "stdafx.h"

#include "OpenHapticsWrapper.h"

using namespace OpenHapticsWrapper;
using namespace System::Runtime::InteropServices;

// Variables
HHD phantomID;
HDErrorInfo error;
HDErrorInfo lastError;

// Charge (positive/negative). Makes the two HIPs attractive or repulsive
int charge = 1;

// Radius of attractive or repulsive field around each HIP
static double sphereRadius = 6.0;

// To make the force large enough to keep the device from falling
static double forceScale = 0.3;

hduVector3Dd serverDevicePosition, clientDevicePosition, serverDeviceRotation, clientDeviceRotation, clientDeviceJointAngles;

static GCHandle gchForce;

OpenHaptics::OpenHaptics(bool bForce, bool bInput)
{
	inX = 0;
	inY = 0;
	inZ = 0;
	outX = 0;
	outY = 0;
	outZ = 0;

	outBaseX = 0;
	outBaseY = 0;
	outBaseZ = 0;

	inRotX = 0;
	inRotY = 0;
	inRotZ = 0;
	outRotX = 0;
	outRotY = 0;
	outRotZ = 0;

	outRotHandleX = 0;
	outRotHandleY = 0;

	inForceX = 0;
	inForceY = 0;
	inForceZ = 0;

	bButtonState = false;

	gSchedulerCallback = HD_INVALID_HANDLE;

	isFollowing = bForce;
	isForceInput = bInput;
}

OpenHaptics::~OpenHaptics()
{
	
}

bool OpenHaptics::InitDevice()
{
	// Initialize device
	phantomID = hdInitDevice(HD_DEFAULT_DEVICE);

	// Get DOF
	HDint* ptrDOF;
	hdGetIntegerv(HD_OUTPUT_DOF, ptrDOF);
	dof = *ptrDOF;

	deviceModel = Marshal::PtrToStringAnsi((IntPtr)(char*) hdGetString(HD_DEVICE_MODEL_TYPE));

	// return false if an error exists
	if (HD_DEVICE_ERROR(lastError = hdGetError()))
		return false;

	//Enable force if device is found
	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);
	hdDisable(HD_SOFTWARE_VELOCITY_LIMIT);
	hdDisable(HD_SOFTWARE_FORCE_IMPULSE_LIMIT);
	printf("It has been initialized.\n");

	// Start scheduler
	hdStartScheduler();

	// If there an error with scheduler exit application
	if (HD_DEVICE_ERROR(error = hdGetError()))
		return false;

	return true;
}

void OpenHaptics::Run()
{
	// Register force feedback
	MyCallback ^cb = gcnew MyCallback(this, &OpenHaptics::HapticCallback);
	gchForce = GCHandle::Alloc(cb);
	IntPtr ip1 = Marshal::GetFunctionPointerForDelegate(cb);
	HDSchedulerCallback callback = static_cast<HDSchedulerCallback>(ip1.ToPointer());
	//GC::Collect();
	// Register haptic callback function
	gSchedulerCallback = hdScheduleAsynchronous(callback, 0, HD_MAX_SCHEDULER_PRIORITY);
}

void OpenHaptics::CloseDevice()
{
	// Stop scheduler
	if (!lastError.errorCode)
	{
		hdStopScheduler();
		if(gSchedulerCallback != HD_INVALID_HANDLE)
			hdUnschedule(gSchedulerCallback);
	}

	//Disable device and release haptic device handle
	if (phantomID != HD_INVALID_HANDLE)
	{
		hdDisableDevice(phantomID);
		phantomID = HD_INVALID_HANDLE;
	}
}

HDCallbackCode OpenHaptics::HapticCallback(void *data)
{
	userFunctionHandler();
	// Received position
	serverDevicePosition[0] = inX;
	serverDevicePosition[1] = inY;
	serverDevicePosition[2] = inZ;
	// Received rotation
	serverDeviceRotation[0] = inRotX;
	serverDeviceRotation[1] = inRotY;
	serverDeviceRotation[2] = inRotZ;

	int currentButton;

	// Start haptic frame
	hdBeginFrame(phantomID);

	// Get current state of device (HIP and buttons)
	hdGetDoublev(HD_CURRENT_POSITION, clientDevicePosition);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, clientDeviceRotation);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, clientDeviceJointAngles);
	hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButton);

	// Button 1
	bButtonState = currentButton & HD_DEVICE_BUTTON_1;

	clientDevicePosition = hduVector3Dd(clientDevicePosition[0], clientDevicePosition[1], clientDevicePosition[2]);
	
	if (isFollowing)
	{
		// Compute vector distance between local and remote device
		hduVector3Dd pos_diff = clientDevicePosition - serverDevicePosition;
		// Calculate force based on vector distance between the two devices
		hduVector3Dd forceVec = ComputeForce(pos_diff);
		// Set client device force
		hdSetDoublev(HD_CURRENT_FORCE, hduVector3Dd(forceVec[0], forceVec[1], forceVec[2]));

		// Set rotation for 6DOF device
		if (dof == 6)
		{
			// Compute differece in rotation
			hduVector3Dd rot_diff = clientDeviceRotation - serverDeviceRotation;
			// Calculate torque
			hduVector3Dd torque = ComputeTorque(rot_diff);
			// Set torque to follow server rotation
			hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, torque);
		}
	}

	if (isForceInput)
	{
		hdSetDoublev(HD_CURRENT_FORCE, hduVector3Dd(inForceX, inForceY, inForceZ));
	}

	// Flush forces on the first device.
	hdEndFrame(phantomID);

	// Read position
	outX = clientDevicePosition[0];
	outY = clientDevicePosition[1];
	outZ = clientDevicePosition[2];

	// Read rotation
	outRotX = clientDeviceRotation[0];
	outRotY = clientDeviceRotation[1];
	outRotZ = clientDeviceRotation[2];

	// Read joint angles (local)
	jointAngle1 = clientDeviceJointAngles[0];
	jointAngle2 = clientDeviceJointAngles[1];
	jointAngle3 = clientDeviceJointAngles[2];

	// Handle rotation (world)
	outRotHandleX = jointAngle3;
	outRotHandleY = jointAngle1;

	// Calculate position w.r.t. base
	outBaseX = -(ARM1 * Math::Cos(jointAngle2) + ARM2 * Math::Cos(PI / 2 - jointAngle3)) * Math::Sin(jointAngle1);
	outBaseY = ARM1 * Math::Sin(jointAngle2) - ARM2 * Math::Sin(PI / 2 - jointAngle3);
	outBaseZ = (ARM1 * Math::Cos(jointAngle2) + ARM2 * Math::Cos(PI / 2 - jointAngle3)) * Math::Cos(jointAngle1);

	// Check for device error
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		//If it's a scheduler error stop and restart scheduler
		if (hduIsSchedulerError(&error))
		{
			hdStopScheduler();
			hdStartScheduler();
			return HD_CALLBACK_CONTINUE;
		}
	}

	return HD_CALLBACK_CONTINUE;
}

hduVector3Dd OpenHaptics::ComputeForce(hduVector3Dd difference)
{
	double dist = difference.magnitude();
	hduVector3Dd forceVec(0, 0, 0);

	if (dist < sphereRadius * 4.0)
	{
		forceVec = -forceScale*difference;
	}
	else
	{
		hduVector3Dd unitPos = normalize(difference);
		forceVec = -10.0 * unitPos;
	}

	forceVec *= charge;

	return forceVec;
}

hduVector3Dd OpenHaptics::ComputeTorque(hduVector3Dd difference)
{
	
	return -difference * 500;
}