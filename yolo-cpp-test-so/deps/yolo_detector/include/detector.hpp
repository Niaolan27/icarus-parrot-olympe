#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace yolo_detector {

struct Config {
	std::string paramPath;
	std::string binPath;
	std::string inputBlob;
	std::string outputBlob;
	int inputWidth = 640;
	int inputHeight = 640;
	float confidenceThreshold = 0.25f;
	float nmsThreshold = 0.45f;
};

struct Detection {
	int classId = -1;
	float confidence = 0.f;
	cv::Rect box;
};

class Detector {
public:
	Detector();
	~Detector();

	bool load(const Config &config);
	bool isReady() const;
	std::vector<Detection> detect(const cv::Mat &frameBgr);

private:
	class Impl;
	Impl *mImpl;
};

} // namespace yolo_detector
