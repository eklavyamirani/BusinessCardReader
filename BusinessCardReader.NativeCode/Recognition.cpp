#include <iostream>
#include <baseapi.h>
#include <opencv2\opencv.hpp>
#include <string>
#include "tesseract includes\include\leptonica\allheaders.h"

namespace BusinessCardReader
{
	namespace Recognition
	{
		using namespace std;
		string RetreiveText(const cv::Mat inputImage)
		{
			assert(inputImage.data);
			tesseract::TessBaseAPI tesseractAPI;
			tesseractAPI.Init(NULL, "eng", tesseract::OEM_DEFAULT);
			tesseractAPI.SetPageSegMode(tesseract::PSM_AUTO);
			tesseractAPI.SetOutputName("test_output");
			tesseractAPI.SetVariable("tessedit_char_whitelist", "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz@.0123456789,:-() ");
			tesseractAPI.SetVariable("language_model_penalty_non_dict_word", "0.01");
			tesseractAPI.SetVariable("language_model_penalty_non_freq_dict_word", "0.01");
			tesseractAPI.SetVariable("load_system_dawg", "F");
			tesseractAPI.SetVariable("load_freq_dawg", "F");
			cv::Mat thresholdedImage = inputImage;
			tesseractAPI.SetImage(thresholdedImage.data, thresholdedImage.cols, thresholdedImage.rows, thresholdedImage.channels(), thresholdedImage.step1());
			string output = tesseractAPI.GetUTF8Text();
			return output;
		}
	}
}