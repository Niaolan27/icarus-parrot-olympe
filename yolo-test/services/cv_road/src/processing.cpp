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

#include <algorithm>
#include <vector>

#include "processing.hpp"

#define ULOG_TAG processing
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

#define CFG_CHECK(E) ULOG_ERRNO_RETURN_ERR_IF(E < 0, EINVAL)

static std::string resolve_mission_path(const std::string &path)
{
	if (path.empty() || path[0] == '/')
		return path;

	return cfgreader::ConfigReader::insertMissionRootDir(path);
}

namespace cfgreader {
template <>
int SettingReader<struct yoloCfg>::read(const libconfig::Setting &set, T &v)
{
	const char *str = "yoloModelPath";
	CFG_CHECK(ConfigReader::getField(set, str, v.yoloModelPath));

	str = "yoloInputWidth";
	CFG_CHECK(ConfigReader::getField(set, str, v.yoloInputWidth));

	str = "yoloInputHeight";
	CFG_CHECK(ConfigReader::getField(set, str, v.yoloInputHeight));

	str = "yoloConfidenceThreshold";
	CFG_CHECK(ConfigReader::getField(set, str, v.yoloConfidenceThreshold));

	str = "yoloNmsThreshold";
	CFG_CHECK(ConfigReader::getField(set, str, v.yoloNmsThreshold));

	return 0;
}
} // namespace cfgreader

static bool frame_to_bgr(const struct vipc_frame *frame,
			 cv::Mat &frame_bgr,
			 cv::ColorConversionCodes colorConversionCodes)
{
	if (frame->num_planes == 2) {
		/* NV12 */

		/* Y plane */
		cv::Mat yMat(frame->height,
			     frame->width,
			     CV_8UC1,
			     (void *)frame->planes[0].virt_addr,
			     (size_t)frame->planes[0].stride);

		/* UV plane */
		cv::Mat uvMat(frame->height / 2,
			      frame->width / 2,
			      CV_8UC2,
			      (void *)frame->planes[1].virt_addr,
			      (size_t)frame->planes[1].stride);

		cv::cvtColorTwoPlane(
			yMat, uvMat, frame_bgr, colorConversionCodes);

	} else if (frame->num_planes == 3) {
		/* I420 */
		cv::cvtColor(cv::Mat(frame->height * 3 / 2,
				     frame->width,
				     CV_8UC1,
				     (void *)frame->planes[0].virt_addr),
			     frame_bgr,
			     cv::COLOR_YUV2BGR_I420,
			     3);
	} else {
		/* fallback */
		ULOGE("Unsupported frame format: num_planes=%d",
		      frame->num_planes);
		return false;
	}

	return !frame_bgr.empty();
}

#if CV_ROAD_HAS_OPENCV_DNN
static cv::Mat normalize_yolo_output(const cv::Mat &output)
{
	cv::Mat detections = output;
	if (detections.dims == 3) {
		const int rows = detections.size[1];
		const int dimensions = detections.size[2];
		detections = detections.reshape(1, rows);
		if (rows < dimensions)
			cv::transpose(detections, detections);
	}
	return detections;
}

static void run_yolo_on_frame(cv::dnn::Net &net,
			      const struct yoloCfg &cfg,
			      const cv::Mat &frame_bgr)
{
	std::vector<cv::Mat> outputs;
	std::vector<cv::Rect> boxes;
	std::vector<float> confidences;
	std::vector<int> class_ids;
	cv::Mat blob;

	cv::dnn::blobFromImage(frame_bgr,
			       blob,
			       1.0 / 255.0,
			       cv::Size(cfg.yoloInputWidth, cfg.yoloInputHeight),
			       cv::Scalar(),
			       true,
			       false);
	net.setInput(blob);
	try {
		net.forward(outputs, net.getUnconnectedOutLayersNames());
	} catch (const cv::Exception &ex) {
		ULOGE("YOLO inference failed: %s", ex.what());
		return;
	}

	const float x_scale = static_cast<float>(frame_bgr.cols) /
			      static_cast<float>(cfg.yoloInputWidth);
	const float y_scale = static_cast<float>(frame_bgr.rows) /
			      static_cast<float>(cfg.yoloInputHeight);

	for (const cv::Mat &output : outputs) {
		cv::Mat detections = normalize_yolo_output(output);

		for (int i = 0; i < detections.rows; i++) {
			const float *data = detections.ptr<float>(i);
			const int dimensions = detections.cols;
			const bool has_objectness =
				dimensions == 85 || dimensions == 6;
			const float objectness = has_objectness ? data[4] : 1.f;
			const int class_offset = has_objectness ? 5 : 4;
			const int class_count = dimensions - class_offset;

			if (class_count <= 0 || objectness < cfg.yoloConfidenceThreshold)
				continue;

			cv::Mat scores(1, class_count, CV_32FC1,
				       (void *)(data + class_offset));
			cv::Point class_id_point;
			double class_score;
			cv::minMaxLoc(scores,
				      nullptr,
				      &class_score,
				      nullptr,
				      &class_id_point);

			const float confidence =
				objectness * static_cast<float>(class_score);
			if (confidence < cfg.yoloConfidenceThreshold)
				continue;

			const float center_x = data[0] * x_scale;
			const float center_y = data[1] * y_scale;
			const float width = data[2] * x_scale;
			const float height = data[3] * y_scale;
			const int left = std::max(
				0, static_cast<int>(center_x - width / 2));
			const int top = std::max(
				0, static_cast<int>(center_y - height / 2));

			boxes.emplace_back(left,
					   top,
					   static_cast<int>(width),
					   static_cast<int>(height));
			confidences.push_back(confidence);
			class_ids.push_back(class_id_point.x);
		}
	}

	std::vector<int> indices;
	if (!boxes.empty()) {
		cv::dnn::NMSBoxes(boxes,
				  confidences,
				  cfg.yoloConfidenceThreshold,
				  cfg.yoloNmsThreshold,
				  indices);
	}

	ULOGI("YOLO detections: %zu", indices.size());
	for (int index : indices) {
		const cv::Rect &box = boxes[index];
		ULOGI("YOLO class=%d confidence=%.3f box=[x=%d,y=%d,w=%d,h=%d]",
		      class_ids[index],
		      confidences[index],
		      box.x,
		      box.y,
		      box.width,
		      box.height);
	}
}

static void do_step(const struct vipc_frame *frame,
		    cv::ColorConversionCodes colorConversionCodes,
		    cv::dnn::Net &yolo_net,
		    const struct yoloCfg &cfg)
{
	cv::Mat frame_bgr;

	if (!frame_to_bgr(frame, frame_bgr, colorConversionCodes))
		return;

	run_yolo_on_frame(yolo_net, cfg, frame_bgr);
}
#endif

ProcessingListener::ProcessingListener(void *userdata) : Listener(userdata) {}

int ProcessingListener::processingStep(void *userdata,
				       const struct vipc_frame *new_frame)
{
	Processing *processing = (Processing *)userdata;
	return processing->processingStep(new_frame);
}

void Processing::threadEntry()
{
	std::unique_lock<std::mutex> lk(mMutex);

	struct vipc_frame frame;
	const struct vipc_frame *frame_to_release;

	while (!mStopRequested) {
		/* Atomically unlock the mutex, wait for condition and then
		  re-lock the mutex when condition is signaled */
		mCond.wait(lk);

		if (mStopRequested)
			break;
		if (!mFrameAvailable)
			continue;

		/* Copy locally input data */
		memcpy(&frame, mFrame, sizeof(frame));
		frame_to_release = mFrame;
		mFrameAvailable = false;

		/* Do the heavy computation outside lock */
		lk.unlock();
#if CV_ROAD_HAS_OPENCV_DNN
		if (loadYoloModel())
			do_step(&frame, mColorConversionCodes, mYoloNet, mYoloCfg);
#else
		loadYoloModel();
#endif
		lk.lock();

		/* Done with the input frame */
		vipcc_release_safe(frame_to_release);
	}
}

int Processing::loadYoloConfiguration(const std::string &configPath)
{
	cfgreader::FileConfigReader reader(configPath);
	int res = reader.load();
	if (res < 0) {
		ULOG_ERRNO("cannot load %s", -res, configPath.c_str());
		return res;
	}

	res = reader.get("road_following", mYoloCfg);
	if (res < 0) {
		ULOG_ERRNO("cannot read YOLO config", -res);
		return res;
	}

	return 0;
}

bool Processing::loadYoloModel(void)
{
	if (mYoloReady)
		return true;
	if (mYoloLoadTried)
		return false;

	if (mYoloCfg.yoloInputWidth <= 0 ||
	    mYoloCfg.yoloInputHeight <= 0) {
		ULOGE("invalid YOLO input size: %dx%d",
		      mYoloCfg.yoloInputWidth,
		      mYoloCfg.yoloInputHeight);
		mYoloLoadTried = true;
		return false;
	}

	mYoloLoadTried = true;
	const std::string modelPath = resolve_mission_path(mYoloCfg.yoloModelPath);
#if CV_ROAD_HAS_OPENCV_DNN
	try {
		mYoloNet = cv::dnn::readNet(modelPath);
		mYoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		mYoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		mYoloReady = !mYoloNet.empty();
	} catch (const cv::Exception &ex) {
		ULOGE("YOLO model load failed from '%s': %s",
		      modelPath.c_str(),
		      ex.what());
		mYoloReady = false;
	}

	if (mYoloReady) {
		ULOGI("YOLO model loaded from '%s' (%dx%d input)",
		      modelPath.c_str(),
		      mYoloCfg.yoloInputWidth,
		      mYoloCfg.yoloInputHeight);
	} else {
		ULOGE("YOLO model is not ready: '%s'", modelPath.c_str());
	}
#else
	ULOGE("YOLO model '%s' cannot be loaded: this AirSDK OpenCV build "
	      "does not include the opencv_dnn module",
	      modelPath.c_str());
	mYoloReady = false;
#endif

	return mYoloReady;
}

Processing::Processing(pomp::Loop *loop) :
		mProcessingListener(this), mChannel(nullptr),
		mMessageHub(loop, this), mVideo(loop)
{
	int res;

	/* pomp loop object */
	mLoop = loop;

	/* Service context */
	mStopRequested = 0;
	mStarted = false;

	/* Thread context */
	mThread = nullptr;

	/* Vipc context */
	mFrame = nullptr;
	mFrameAvailable = false;

	/* YOLO model */
	mYoloLoadTried = false;
	mYoloReady = false;

	res = loadYoloConfiguration(
		cfgreader::ConfigReader::insertMissionRootDir(
			YOLO_SERVICE_CONFIG_PATH));
	if (res < 0) {
		ULOG_ERRNO("loadYoloConfiguration", -res);
		std::bad_alloc ex;
		throw ex;
	}

	/* Get drone model */
	res = sys_prop_get("ro.model", mDroneModel, "None");
	if (res < 0) {
		ULOG_ERRNO("sys_prop_get", -res);
		throw std::runtime_error("sys_prop_get : failed");
	} else if (strcmp(mDroneModel, "None") == 0) {
		ULOGE("ro.model doesn't exist");
		throw std::runtime_error("sys_prop_get : ro.model doesn't exist");
	}

	if (strcmp(mDroneModel, "anafi3_mil") == 0 ||
	    strcmp(mDroneModel, "anafi3_milp") == 0) {
		mColorConversionCodes = cv::COLOR_YUV2BGR_NV12;
	} else if (strcmp(mDroneModel, "anafi2") == 0) {
		mColorConversionCodes = cv::COLOR_YUV2BGR_NV21;
	} else {
		ULOGE("The drone model %s doesn't exist", mDroneModel);
		std::bad_alloc ex;
		throw ex;
	}
}

Processing::~Processing()
{
	stop();

	mThread = nullptr;
	mFrame = nullptr;
}

int Processing::start(void)
{
	int res = 0;

	/* Start Message Handler */
	mChannel = mMessageHub.startServerChannel(MSGHUB_ADDR, 0666);
	if (!mChannel) {
		ULOGC("failed to create server channel");
		res = -EPERM;
		goto out;
	}
	mMessageHub.enableDump();
	mMessageHub.attachMessageHandler(this);

	/* Create background thread */
	mMutex.lock();
	mStopRequested = 0;
	mMutex.unlock();

	mThread = new std::thread(&Processing::threadEntry, this);
	if (mThread == nullptr) {
		res = -ENOMEM;
		ULOG_ERRNO("new Thread object", -res);
		goto out;
	}

	mStarted = true;

out:
	return res;
}

void Processing::stop()
{
	if (!mThread)
		return;

	/* Ask thread to stop */
	mMutex.lock();
	mStopRequested = 1;
	mCond.notify_one();
	mMutex.unlock();

	/* Wait for thread and release resources */
	mThread->join();
	delete mThread;
	mThread = nullptr;
	mStarted = false;

	/* Cleanup remaining input data if any */
	mMutex.lock();
	if (mFrameAvailable) {
		vipcc_release_safe(mFrame);
		mFrameAvailable = false;
	}
	mMutex.unlock();

	/* Stop Message Handler */
	mMessageHub.stop();
	mMessageHub.detachMessageHandler(this);
	mChannel = nullptr;
}

int Processing::processingStep(const struct vipc_frame *new_frame)
{
	ULOG_ERRNO_RETURN_ERR_IF(new_frame == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!mStarted, EPERM);

	mMutex.lock();

	/* If an input is already pending, release it before overwrite */
	if (mFrameAvailable) {
		vipcc_release_safe(mFrame);
		mFrameAvailable = false;
	}

	/* Copy input data and take ownership of frame */
	mFrame = new_frame;
	mFrameAvailable = true;

	/* Wakeup background thread */
	mCond.notify_one();

	mMutex.unlock();

	return 0;
}

void Processing::onConnected(::msghub::Channel *channel, pomp::Connection *conn)
{
	ULOGN("connected to %s", MSGHUB_ADDR);
};

void Processing::onDisconnected(::msghub::Channel *channel,
				pomp::Connection *conn)
{
	ULOGN("disconnected to %s", MSGHUB_ADDR);
};

void Processing::enableCv(const bool msg)
{
	if (msg)
		mVideo.vipcStart(&mProcessingListener);
	else
		mVideo.vipcStop();
};
