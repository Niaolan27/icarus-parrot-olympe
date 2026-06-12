#include "detector.hpp"

#include <algorithm>
#include <cmath>

#include <ncnn/net.h>

#define ULOG_TAG yolo_detector
#include <ulog.hpp>
ULOG_DECLARE_TAG(ULOG_TAG);

namespace yolo_detector {

namespace {

struct Candidate {
	Detection detection;
	float area;
};

static float intersectionOverUnion(const cv::Rect &a, const cv::Rect &b)
{
	const int x0 = std::max(a.x, b.x);
	const int y0 = std::max(a.y, b.y);
	const int x1 = std::min(a.x + a.width, b.x + b.width);
	const int y1 = std::min(a.y + a.height, b.y + b.height);
	const int w = std::max(0, x1 - x0);
	const int h = std::max(0, y1 - y0);
	const float intersection = static_cast<float>(w * h);
	const float unionArea = static_cast<float>(a.area() + b.area()) -
				intersection;

	return unionArea <= 0.f ? 0.f : intersection / unionArea;
}

static std::vector<Detection> nonMaximumSuppression(
	std::vector<Candidate> &candidates,
	float nmsThreshold)
{
	std::vector<Detection> picked;
	std::vector<bool> removed(candidates.size(), false);

	std::sort(candidates.begin(),
		  candidates.end(),
		  [](const Candidate &a, const Candidate &b) {
			  return a.detection.confidence > b.detection.confidence;
		  });

	for (size_t i = 0; i < candidates.size(); i++) {
		if (removed[i])
			continue;

		picked.push_back(candidates[i].detection);

		for (size_t j = i + 1; j < candidates.size(); j++) {
			if (removed[j])
				continue;

			if (candidates[i].detection.classId ==
				    candidates[j].detection.classId &&
			    intersectionOverUnion(candidates[i].detection.box,
						  candidates[j].detection.box) >
				    nmsThreshold) {
				removed[j] = true;
			}
		}
	}

	return picked;
}

static void addCandidate(std::vector<Candidate> &candidates,
			 const float *values,
			 int dimensions,
			 float confidenceThreshold,
			 float xScale,
			 float yScale,
			 int xPad,
			 int yPad,
			 int frameWidth,
			 int frameHeight)
{
	if (dimensions <= 5)
		return;

	int classId = -1;
	float classScore = 0.f;
	for (int i = 4; i < dimensions; i++) {
		if (values[i] > classScore) {
			classScore = values[i];
			classId = i - 4;
		}
	}

	if (classScore < confidenceThreshold)
		return;

	const float cx = (values[0] - static_cast<float>(xPad)) / xScale;
	const float cy = (values[1] - static_cast<float>(yPad)) / yScale;
	const float width = values[2] / xScale;
	const float height = values[3] / yScale;

	const int left = std::max(0, static_cast<int>(cx - width / 2.f));
	const int top = std::max(0, static_cast<int>(cy - height / 2.f));
	const int right = std::min(frameWidth,
				   static_cast<int>(cx + width / 2.f));
	const int bottom = std::min(frameHeight,
				    static_cast<int>(cy + height / 2.f));
	const int boxWidth = std::max(0, right - left);
	const int boxHeight = std::max(0, bottom - top);

	if (boxWidth == 0 || boxHeight == 0)
		return;

	Detection detection;
	detection.classId = classId;
	detection.confidence = classScore;
	detection.box = cv::Rect(left, top, boxWidth, boxHeight);
	candidates.push_back({detection,
			      static_cast<float>(boxWidth * boxHeight)});
}

static std::vector<Detection> decodeYolov8(const ncnn::Mat &output,
					   const Config &config,
					   int xPad,
					   int yPad,
					   float xScale,
					   float yScale,
					   int frameWidth,
					   int frameHeight)
{
	std::vector<Candidate> candidates;

	if (output.dims == 2 && output.h > 5 && output.w > output.h) {
		std::vector<float> values(output.h);
		for (int col = 0; col < output.w; col++) {
			for (int row = 0; row < output.h; row++)
				values[row] = output.row(row)[col];

			addCandidate(candidates,
				     values.data(),
				     output.h,
				     config.confidenceThreshold,
				     xScale,
				     yScale,
				     xPad,
				     yPad,
				     frameWidth,
				     frameHeight);
		}
	} else if (output.dims == 2 && output.w > 5) {
		for (int row = 0; row < output.h; row++) {
			const float *values = output.row(row);
			addCandidate(candidates,
				     values,
				     output.w,
				     config.confidenceThreshold,
				     xScale,
				     yScale,
				     xPad,
				     yPad,
				     frameWidth,
				     frameHeight);
		}
	} else if (output.dims == 2 && output.h > 5) {
		std::vector<float> values(output.h);
		for (int col = 0; col < output.w; col++) {
			for (int row = 0; row < output.h; row++)
				values[row] = output.row(row)[col];

			addCandidate(candidates,
				     values.data(),
				     output.h,
				     config.confidenceThreshold,
				     xScale,
				     yScale,
				     xPad,
				     yPad,
				     frameWidth,
				     frameHeight);
		}
	} else if (output.dims == 3 && output.c == 1) {
		const ncnn::Mat channel = output.channel(0);
		if (channel.w > 5 && channel.h > channel.w) {
			for (int row = 0; row < channel.h; row++) {
				const float *values = channel.row(row);
				addCandidate(candidates,
					     values,
					     channel.w,
					     config.confidenceThreshold,
					     xScale,
					     yScale,
					     xPad,
					     yPad,
					     frameWidth,
					     frameHeight);
			}
		} else if (channel.h > 5) {
			std::vector<float> values(channel.h);
			for (int col = 0; col < channel.w; col++) {
				for (int row = 0; row < channel.h; row++)
					values[row] = channel.row(row)[col];

				addCandidate(candidates,
					     values.data(),
					     channel.h,
					     config.confidenceThreshold,
					     xScale,
					     yScale,
					     xPad,
					     yPad,
					     frameWidth,
					     frameHeight);
			}
		} else {
			ULOGE("unsupported YOLO channel shape: w=%d h=%d",
			      channel.w,
			      channel.h);
		}
	} else {
		ULOGE("unsupported YOLO output shape: dims=%d w=%d h=%d c=%d",
		      output.dims,
		      output.w,
		      output.h,
		      output.c);
	}

	ULOGI("YOLO candidates before NMS: %zu", candidates.size());
	return nonMaximumSuppression(candidates, config.nmsThreshold);
}

} // namespace

class Detector::Impl {
public:
	Config config;
	ncnn::Net net;
	bool ready = false;
	bool outputShapeLogged = false;
};

Detector::Detector() : mImpl(new Impl) {}

Detector::~Detector()
{
	delete mImpl;
}

bool Detector::load(const Config &config)
{
	mImpl->config = config;
	mImpl->ready = false;
	mImpl->outputShapeLogged = false;
	mImpl->net.clear();
	mImpl->net.opt.use_vulkan_compute = false;
	mImpl->net.opt.num_threads = 2;

	if (config.inputWidth <= 0 || config.inputHeight <= 0) {
		ULOGE("invalid YOLO input size: %dx%d",
		      config.inputWidth,
		      config.inputHeight);
		return false;
	}

	if (mImpl->net.load_param(config.paramPath.c_str()) != 0) {
		ULOGE("failed to load NCNN param from '%s'",
		      config.paramPath.c_str());
		return false;
	}

	if (mImpl->net.load_model(config.binPath.c_str()) != 0) {
		ULOGE("failed to load NCNN model from '%s'",
		      config.binPath.c_str());
		return false;
	}

	mImpl->ready = true;
	ULOGI("loaded NCNN YOLO model param='%s' bin='%s'",
	      config.paramPath.c_str(),
	      config.binPath.c_str());
	return true;
}

bool Detector::isReady() const
{
	return mImpl->ready;
}

std::vector<Detection> Detector::detect(const cv::Mat &frameBgr)
{
	if (!mImpl->ready || frameBgr.empty())
		return {};

	const int frameWidth = frameBgr.cols;
	const int frameHeight = frameBgr.rows;
	const float scale = std::min(
		static_cast<float>(mImpl->config.inputWidth) /
			static_cast<float>(frameWidth),
		static_cast<float>(mImpl->config.inputHeight) /
			static_cast<float>(frameHeight));
	const int resizedWidth = static_cast<int>(std::round(frameWidth * scale));
	const int resizedHeight =
		static_cast<int>(std::round(frameHeight * scale));
	const int xPad = (mImpl->config.inputWidth - resizedWidth) / 2;
	const int yPad = (mImpl->config.inputHeight - resizedHeight) / 2;

	ncnn::Mat input = ncnn::Mat::from_pixels_resize(frameBgr.data,
							ncnn::Mat::PIXEL_BGR2RGB,
							frameWidth,
							frameHeight,
							resizedWidth,
							resizedHeight);
	ncnn::Mat padded;
	ncnn::copy_make_border(input,
			       padded,
			       yPad,
			       mImpl->config.inputHeight - resizedHeight - yPad,
			       xPad,
			       mImpl->config.inputWidth - resizedWidth - xPad,
			       ncnn::BORDER_CONSTANT,
			       114.f);

	const float normVals[3] = {1.f / 255.f, 1.f / 255.f, 1.f / 255.f};
	padded.substract_mean_normalize(nullptr, normVals);

	ncnn::Extractor extractor = mImpl->net.create_extractor();
	int inputResult;
	if (!mImpl->config.inputBlob.empty()) {
		inputResult = extractor.input(mImpl->config.inputBlob.c_str(),
					      padded);
	} else {
		const std::vector<int> &inputs = mImpl->net.input_indexes();
		inputResult = inputs.empty() ? -1 : extractor.input(inputs[0],
								     padded);
	}

	if (inputResult != 0) {
		ULOGE("failed to set NCNN input blob");
		return {};
	}

	ncnn::Mat output;
	int outputResult;
	if (!mImpl->config.outputBlob.empty()) {
		outputResult = extractor.extract(mImpl->config.outputBlob.c_str(),
						 output);
	} else {
		const std::vector<int> &outputs = mImpl->net.output_indexes();
		outputResult = outputs.empty() ? -1 : extractor.extract(outputs[0],
									 output);
	}

	if (outputResult != 0) {
		ULOGE("failed to extract NCNN output blob");
		return {};
	}

	if (!mImpl->outputShapeLogged) {
		ULOGI("NCNN output shape: dims=%d w=%d h=%d c=%d",
		      output.dims,
		      output.w,
		      output.h,
		      output.c);
		mImpl->outputShapeLogged = true;
	}

	return decodeYolov8(output,
			    mImpl->config,
			    xPad,
			    yPad,
			    scale,
			    scale,
			    frameWidth,
			    frameHeight);
}

} // namespace yolo_detector
