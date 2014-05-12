#ifndef RECOGNITION
#define RECOGNITION
#include <opencv2\opencv.hpp>
#include <string>

namespace BusinessCardReader
{
	namespace Recognition
	{
		std::string RetreiveText(const cv::Mat inputImage);
	}
}
#endif