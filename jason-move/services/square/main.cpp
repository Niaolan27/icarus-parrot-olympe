/**
 * Copyright (C) 2026 icarus
 */
#include <csignal>
#include <cstddef>
#include <unistd.h>
#include <vector>

#include <airsdk/airsdk.hpp>
#include <libpomp.hpp>

#include "ulog.h"
ULOG_DECLARE_TAG(square);

#define ULOG_TAG square
#include "ulog.h"

sig_atomic_t run = 1;

struct RelativeMove {
	float dx;
	float dy;
	float dz;
};

struct Trajectory {
	RelativeMove relTarget;
	float heading;
};

static std::vector<Trajectory> mRelativeTrajectory;
static size_t mMoveIndex = 0;
static bool mFirstTimeHovering = false;
static bool mReturnHomeSent = false;
static bool mLandSent = false;
static pomp::Loop *sLoop = nullptr;

extern "C" void sig_handler(int sig)
{
	run = 0;
	if (sLoop != nullptr)
		sLoop->wakeup();
}

/*Callback functions need to follow function signatures
  as described in the SDK*/

static void onConnected(bool success, void *userdata)
{
	ULOGN("ControlInterface is connected : %s",
	      success ? "succeeded" : "failed");
}

static void onDisconnected(bool success, void *userdata)
{
	ULOGN("ControlInterface is Disconnected : %s",
	      success ? "succeeded" : "failed");
}

static void onSent(airsdk::control::ControlInterface *controlInterface,
		   const arsdk_cmd *cmd,
		   bool success,
		   void *userdata)
{
	char buf[128];
	// Format the commands the mission sends to ease their printing
	arsdk_cmd_fmt(cmd, buf, sizeof(buf));
	ULOGI("ControlInterface cmd %s has been sent", buf);
}

static int cmdMoveBy(airsdk::control::ControlInterface *controlInterface,
		     RelativeMove target,
		     float headingRotation,
		     float maxHorSpeed,
		     float maxVertSpeed,
		     float maxYawSpeed)
{
	arsdk_cmd cmdMoveBy;
	arsdk_cmd_init(&cmdMoveBy);
	ULOGN("#SM MOVE BY x:%f y:%f z:%f", target.dx, target.dy, target.dz);
	ULOGN("Config velocities maxHorSpeed:%f maxVertSpeed:%f "
	      "maxYawSpeed:%f",
	      maxHorSpeed,
	      maxVertSpeed,
	      maxYawSpeed);
	arsdk_cmd_enc_Move_Extended_move_by(
		&cmdMoveBy,
		target.dx,       // wanted displacement along the front axis [m]
		target.dy,       // wanted displacement along the right axis [m]
		target.dz,       // wanted displacement along the down axis [m]
		headingRotation, // wanted rotation of heading [rad]
		maxHorSpeed,     // maximum horizontal speed [m/s]
		maxVertSpeed,    // maximum vertical speed [m/s]
		maxYawSpeed);    // maximum yaw rotation speed [degrees/s]
	controlInterface->send(&cmdMoveBy);
	arsdk_cmd_clear(&cmdMoveBy);
	return 0;
}

static int cmdMoveByTrajectory(airsdk::control::ControlInterface *controlInterface,
		     const Trajectory &trajectory)
{
	ULOGI("Sending square side %zu/%zu", mMoveIndex + 1,
	      mRelativeTrajectory.size());
	return cmdMoveBy(
		controlInterface,
		trajectory.relTarget,
		trajectory.heading,
		10.0f,
		10.0f,
		45.0f);
}

static int cmdRTH(airsdk::control::ControlInterface *controlInterface)
{
	if (mReturnHomeSent)
		return 0;

	arsdk_cmd cmdRTH;
	arsdk_cmd_init(&cmdRTH);
	arsdk_cmd_enc_Rth_Return_to_home(&cmdRTH);
	controlInterface->send(&cmdRTH);
	arsdk_cmd_clear(&cmdRTH);
	mReturnHomeSent = true;
	return 0;
}

static int cmdLand(airsdk::control::ControlInterface *controlInterface)
{
	if (mLandSent)
		return 0;

	arsdk_cmd cmdLand;
	arsdk_cmd_init(&cmdLand);
	arsdk_cmd_enc_Ardrone3_Piloting_Landing(&cmdLand);
	controlInterface->send(&cmdLand);
	arsdk_cmd_clear(&cmdLand);
	mLandSent = true;
	return 0;
}

static void onReceived(airsdk::control::ControlInterface *controlInterface,
		       const arsdk_cmd *cmd,
		       void *userdata)
{
	// Send commands to the control interface switch case, that will react
	// to the proper events and perform the appropriate moves
	ULOGI("ControlInterface cmd received with id %d", cmd->id);
	switch (cmd->id) {
	case ARSDK_ID_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED: {
		int32_t state = 0;
		int res = arsdk_cmd_dec_Ardrone3_PilotingState_FlyingStateChanged(
			cmd, &state);
		if (res != 0) {
			ULOG_ERRNO(
				"arsdk_cmd_dec_Ardrone3_PilotingState_FlyingStateChanged",
				-res);
			return;
		}
		if (state == ARSDK_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED) {
			ULOGI("Drone is landed, resetting square state");
			mMoveIndex = 0;
			mFirstTimeHovering = false;
			mReturnHomeSent = false;
			mLandSent = false;
		} else if (state == ARSDK_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING) {
			ULOGI("Drone is hovering");
			if (!mFirstTimeHovering && mMoveIndex < mRelativeTrajectory.size()) {
				cmdMoveByTrajectory(controlInterface,
					  mRelativeTrajectory[mMoveIndex]);
				mMoveIndex++;
				mFirstTimeHovering = true;
			}
		} else {
			ULOGI("Drone is not hovering, state: %d", state);
		}
		break;
	}
	case ARSDK_ID_ARDRONE3_PILOTINGEVENT_MOVEBYEND: {
		float dx = 0.0f;
		float dy = 0.0f;
		float dz = 0.0f;
		float dpsi = 0.0f;
		int32_t error = 0;

		int res = arsdk_cmd_dec_Ardrone3_PilotingEvent_MoveByEnd(
			cmd,
			&dx,
			&dy,
			&dz,
			&dpsi,
			&error);

		if (res != 0) {
			ULOG_ERRNO("arsdk_cmd_dec_Ardrone3_PilotingEvent_MoveByEnd", -res);
			return;
		}

		ULOGI("MoveByEnd: dx=%f dy=%f dz=%f dpsi=%f error=%d",
		      dx,
		      dy,
		      dz,
		      dpsi,
		      error);

		if (error == ARSDK_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_OK) {
			ULOGI("MoveBy completed successfully");
			// check if there are more moves to execute and send the next one, otherwise return to home
			if (mMoveIndex < mRelativeTrajectory.size()) {
				cmdMoveByTrajectory(controlInterface,
					  mRelativeTrajectory[mMoveIndex]);
				mMoveIndex++;
			} else {
				ULOGI("Square complete, returning home");
				cmdRTH(controlInterface);
			}
		} else {
			ULOGW("MoveBy ended with error=%d", error);
			cmdRTH(controlInterface);
		}

		break;
	}
	case ARSDK_ID_RTH_STATE: {
		int32_t state = 0;
		int32_t reason = 0;
		int res = arsdk_cmd_dec_Rth_State(cmd, &state, &reason);
		if (res != 0) {
			ULOG_ERRNO("arsdk_cmd_dec_Rth_State", -res);
			return;
		}

		ULOGI("RTH state=%d reason=%d", state, reason);
		if (reason == ARSDK_RTH_STATE_REASON_FINISHED)
			cmdLand(controlInterface);

		break;
	}

	default:
		break;
	}
}


int main(int argc, char *argv[])
{
	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from square");
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
	signal(SIGPIPE, SIG_IGN);

	mMoveIndex = 0;
	mFirstTimeHovering = false;
	mReturnHomeSent = false;
	mLandSent = false;
	mRelativeTrajectory = {
		{{2.0f, 0.0f, 0.0f}, 0.0f},
		{{0.0f, 2.0f, 0.0f}, 0.0f},
		{{-2.0f, 0.0f, 0.0f}, 0.0f},
		{{0.0f, -2.0f, 0.0f}, 0.0f},
		{{0.0f, 0.0f, -4.0f}, 0.0f}
	};

	// event loop
	pomp::Loop loop;
	sLoop = &loop;
	// initialize control interface here
	airsdk::control::ControlInterface controlItf(loop);

	//listener object
	airsdk::control::Listener<airsdk::control::ControlInterface> listener_cb = {
		.connected_cb = onConnected,
		.disconnected_cb = onDisconnected,
		.sent_cb = onSent,
		.received_cb = onReceived,
		.userdata = nullptr, // set userdata if needed
	};
	// connect listener callbacks to control interface here
	int res = controlItf.connect(listener_cb);
	if (res != 0) {
		ULOGE("Error while connecting control interface");
		ULOG_ERRNO("ControlInterface::connect", -res);
		return res;
	}

	/* Loop code
	 *
	 * The service is assumed to run an infinite loop, and termination
	 * requests are handled via a SIGTERM signal.
	 * If your service exits before this SIGTERM is sent, it will be
	 * considered as a crash, and the system will relaunch the service.
	 * If this happens too many times, the system will no longer start the
	 * service.
	 */
	while (run)
		loop.waitAndProcess(-1);

	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("Cleaning up from square");
	sLoop = nullptr;
	return 0;
}
