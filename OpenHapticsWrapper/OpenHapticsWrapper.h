#pragma once
#include<string>

using namespace System;

#define PI 3.1415926535897
#define ARM1 138
#define ARM2 133

namespace OpenHapticsWrapper {
	public delegate void UserFunctionDelegate();

	public ref class OpenHaptics
	{
	public:
		// Constructor & Destructor
		OpenHaptics(bool bForce, bool bInput);
		~OpenHaptics();

		// Functions
		bool InitDevice();
		void Run();
		void CloseDevice();
		HDCallbackCode HapticCallback(void *data);
		hduVector3Dd ComputeForce(hduVector3Dd difference);
		hduVector3Dd ComputeTorque(hduVector3Dd difference);

		UserFunctionDelegate^ userFunctionHandler;

		HDSchedulerHandle gSchedulerCallback;

		delegate HDCallbackCode MyCallback(void *data);

		// DOF
		int dof;

		// Force
		bool isFollowing; // Apply force to follow server
		bool isForceInput; // Apply force according to user input

		String^ deviceModel;

		// Received position
		double inX;
		double inY;
		double inZ;

		// Self position
		double outX;
		double outY;
		double outZ;

		// Self position regarding base
		double outBaseX;
		double outBaseY;
		double outBaseZ;

		// Received rotation
		double inRotX;
		double inRotY;
		double inRotZ;

		// Self rotation
		double outRotX;
		double outRotY;
		double outRotZ;

		// Handle rotation
		double outRotHandleX;
		double outRotHandleY;

		// Self joint angles
		double jointAngle1;
		double jointAngle2;
		double jointAngle3;

		// Incoming Force
		double inForceX;
		double inForceY;
		double inForceZ;

		// Button state
		bool bButtonState;
	};
}
