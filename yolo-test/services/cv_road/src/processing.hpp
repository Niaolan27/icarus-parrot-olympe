/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#pragma once

#include <condition_variable>
#include <csignal>
#include <mutex>
#include <thread>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#if defined(__has_include)
#if __has_include(<opencv2/dnn.hpp>)
#define CV_ROAD_HAS_OPENCV_DNN 1
#include <opencv2/dnn.hpp>
#else
#define CV_ROAD_HAS_OPENCV_DNN 0
#endif
#else
#define CV_ROAD_HAS_OPENCV_DNN 0
#endif

#include <cfgreader/cfgreader.hpp>
#include <libpomp.hpp>
#include <putils/properties.h>
#include <video-ipc/vipc_client.h>

#include "listener.hpp"
#include "video.hpp"

/* Messages exchanged with Flight Supervisor */
#include <road_runner/cv_road/messages.msghub.h>
#include <road_runner/cv_road/messages.pb.h>

#define MSGHUB_ADDR "unix:/tmp/road-runner-cv-road-service"

static const std::string YOLO_SERVICE_CONFIG_PATH =
	"/etc/services/road_following.cfg";

/* Configuration values */
struct yoloCfg {
	std::string yoloModelPath;
	int yoloInputWidth;
	int yoloInputHeight;
	float yoloConfidenceThreshold;
	float yoloNmsThreshold;
};

class ProcessingListener : public Listener {
public:
	/* Constructor */
	ProcessingListener(void *userdata);
	/* Destructor */
	~ProcessingListener() = default;
	/* processingStep callback */
	int processingStep(void *userdata,
			   const struct vipc_frame *new_frame) override;
};

class Processing : public ::road_runner::service::cv_road::messages::msghub::
			   CommandHandler,
		   public ::msghub::MessageHub::ConnectionHandler {
private:
	/* pomp loop object */
	pomp::Loop *mLoop;

	/* Service context */
	sig_atomic_t mStopRequested;
	bool mStarted;

	/* YOLO configuration Object */
	struct yoloCfg mYoloCfg;

	/* Thread context */
	std::thread *mThread;
	std::mutex mMutex;
	std::condition_variable mCond;

	/* Vipc context */
	const struct vipc_frame *mFrame;
	bool mFrameAvailable;

	/* Listener Object */
	ProcessingListener mProcessingListener;

	/* Msghub objects */
	msghub::Channel *mChannel;
	msghub::MessageHub mMessageHub;

	/* Video Object */
	Video mVideo;

	/* Drone model */
	char mDroneModel[SYS_PROP_VALUE_MAX];
	cv::ColorConversionCodes mColorConversionCodes;

	/* YOLO model */
#if CV_ROAD_HAS_OPENCV_DNN
	cv::dnn::Net mYoloNet;
#endif
	bool mYoloLoadTried;
	bool mYoloReady;

private:
	/* Thread function */
	void threadEntry();

	/**
	 * Load the configuration of the service
	 *
	 * @param configPath path of the config file.
	 * @return 0 in case of success, negative errno in case of error.
	 */
	int loadYoloConfiguration(const std::string &configPath);

	/**
	 * Load the YOLO model configured for the processing stage.
	 *
	 * @return true if the model is ready for inference, false otherwise.
	 */
	bool loadYoloModel(void);

public:
	/**
	 * Constructor
	 * @param loop pomp loop object
	 */
	Processing(pomp::Loop *loop);

	/**
	 * Destructor
	 */
	~Processing();

	/**
	 * Start processing.
	 * @return  0 in case of success, negative errno in case of error.
	 */
	int start(void);

	/**
	 * Stop processing.
	 */
	void stop(void);

	/**
	 * Processing video step
	 * @param new_frame vipc_frame object.
	 */
	int processingStep(const struct vipc_frame *new_frame);

	/* --- Msghub --- */

	/* ConnectionHandler overridden functions */
	virtual void onConnected(::msghub::Channel *channel,
				 pomp::Connection *conn) override;
	virtual void onDisconnected(::msghub::Channel *channel,
				    pomp::Connection *conn) override;

	/**
	 * Override enableCv function (Handler).
	 *
	 * True received  : YOLO video processing needs to be activated
	 * False received : YOLO video processing needs to be disabled
	 *
	 * @param msg bool
	 */
	virtual void enableCv(const bool msg) override;
};
