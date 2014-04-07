#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>
#include <string>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include <boost\graph\adjacency_list.hpp>
#include <boost\graph\connected_components.hpp>

namespace BusinessCardReader
{
	using namespace std;
	using namespace cv;

	struct SWTPoint
	{
		int x;
		int y;
		float SWT;
	};

	void LOG(std::string message)
	{
		std::cout << message << std::endl;
	}

	void LOG(cv::Mat inputImage)
	{
		const std::string WINDOW_NAME = "Input Image";
		cv::namedWindow(WINDOW_NAME);
		cv::imshow(WINDOW_NAME, inputImage);
		cv::waitKey(0);
		cv::destroyWindow(WINDOW_NAME);
	}

	struct Ray
	{
		vector<SWTPoint> points;
		Point p;
		Point q;
	};

	struct ConnectedComponent
	{
		std::vector<SWTPoint> points;
		float variance;
		float mean;
		float height;
		float width;
	};

	typedef std::vector<Ray> Rays;

	void StrokeWidthTransform(Mat edgeImage, Mat gradientX, Mat gradientY, Mat &SWT, Rays &rays)
	{
		for (int i = 0; i < edgeImage.rows; i++)
		{
			//uchar* rowPtr = edgeImage.ptr(i);
			for (int j = 0; j < edgeImage.cols; j++)
			{
				float prec = 0.05f;
				float step = 0.5f;
				uchar tempVal = edgeImage.at<uchar>(i, j);
				if (tempVal > 0)
				{
					float currentX = j;
					float currentY = i;
					Ray ray;
					ray.p = Point(currentX, currentY);
					SWTPoint startingPoint = { currentX, currentY, -1 };
					std::vector<SWTPoint> points;
					points.push_back(startingPoint);

					currentX += step;
					currentY += step;

					int currentPixelX = j;
					int currentPixelY = i;

					float Gx = gradientX.at<float>(i, j);
					float Gy = gradientY.at<float>(i, j);

					float gradientMagnitude = sqrt((Gx * Gx) + (Gy * Gy));
					Gx = -Gx / gradientMagnitude;
					Gy = -Gy / gradientMagnitude;
					//LOG("i,j = " + to_string(i) + to_string(j));
					while (true)
					{
						currentX += Gx * prec;
						currentY += Gy * prec;
						if (floor(currentX) != currentPixelX || floor(currentY) != currentPixelY)
						{
							if (floor(currentX) >= SWT.cols || floor(currentX) < 0 ||
								floor(currentY) >= SWT.rows || floor(currentY) < 0)
							{
								break;
							}

							currentPixelX = floor(currentX);
							currentPixelY = floor(currentY);
							SWTPoint SWTCurrentPoint = { currentX, currentY, -1 };
							points.push_back(SWTCurrentPoint);
							//LOG(to_string(edgeImage.at<uchar>(currentPixelY, currentPixelX)));
							if (edgeImage.at<uchar>(currentPixelY, currentPixelX) > 0)
							{
								ray.q = Point(currentPixelX, currentPixelY);
								float Gxi = gradientX.at<float>(currentPixelY, currentPixelX);
								float Gyi = gradientY.at<float>(currentPixelY, currentPixelX);
								gradientMagnitude = sqrt(Gxi * Gxi + Gyi * Gyi);
								Gxi = -Gxi / gradientMagnitude;
								Gyi = -Gyi / gradientMagnitude;
								float temp = acos(Gx * -Gxi + Gy * -Gyi);
								if (acos(Gx * -Gxi + Gy * -Gyi) < 3.14 / 2.)
								{
									float length = sqrt((ray.q.x - ray.p.x)* (ray.q.x - ray.p.x) +
										(ray.q.y - ray.p.y)* (ray.q.y - ray.p.y));
									for (std::vector<SWTPoint>::iterator point = points.begin(); point != points.end(); point++)
									{
										float SWTValue = SWT.at<float>(point->y, point->x);
										if (SWTValue < 0)
										{
											SWTValue = length;
										}
										else
										{
											SWTValue = std::min(SWTValue, length);
										}
										point->SWT = SWTValue;
										SWT.at<float>(point->y, point->x) = SWTValue;
									}
									ray.points = points;
									rays.push_back(ray);
								}
								break;
							}
						}

					}
				}
			}

		}
	}

	void normalizeImage(Mat input, Mat &output) {
		float maxVal = 0;
		float minVal = 1e100;
		for (int row = 0; row < input.rows; row++){
			const float* ptr = (const float*)(input.ptr(row));
			for (int col = 0; col < input.cols; col++){
				if (*ptr < 0) {}
				else {
					maxVal = std::max(*ptr, maxVal);
					minVal = std::min(*ptr, minVal);
				}
				ptr++;
			}
		}

		float difference = maxVal - minVal;
		for (int row = 0; row < input.rows; row++){
			const float* ptrin = (const float*)(input.ptr(row));
			float* ptrout = (float*)(output.ptr(row));
			for (int col = 0; col < input.cols; col++){
				if (*ptrin < 0) {
					*ptrout = 1;
				}
				else {
					*ptrout = ((*ptrin) - minVal) / difference;
				}
				ptrout++;
				ptrin++;
			}
		}
	}

	void FindSWTMedians(Rays &rays)
	{
		for (Rays::iterator ray = rays.begin(); ray != rays.end(); ray++)
		{
			std::sort(ray->points.begin(), ray->points.end(),
				[](SWTPoint left, SWTPoint right)
			{
				return left.SWT > right.SWT;
			});

			float medianSWT = ray->points[ray->points.size() / 2].SWT;

			for (std::vector<SWTPoint>::iterator point = ray->points.begin(); point != ray->points.end(); point++)
			{
				point->SWT = medianSWT;
			}
		}
	}

	std::vector<ConnectedComponent> FindConnectedComponents(Mat SWTImage)
	{
		std::map<int, int> imageMap;
		std::map<int, SWTPoint> reverseImageMap;
		int vertexCount = 0;
		for (int row = 0; row < SWTImage.rows; row++)
		{
			for (int col = 0; col < SWTImage.cols; col++)
			{
				float SWTValue = SWTImage.at<float>(row, col);
				if (SWTValue > 0)
				{
					SWTPoint vertex = { col, row, SWTValue };
					reverseImageMap[vertexCount] = vertex;
					imageMap[row * SWTImage.cols + col] = vertexCount;
					vertexCount++;
				}
			}
		}

		typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
		Graph graph(vertexCount);

		for (int n = 0; n < vertexCount; n++)
		{
			SWTPoint vertex = reverseImageMap[n];
			int NextPixelX[] = { 1, 1, 0, -1 };
			int NextPixelY[] = { 0, 1, 1, 1 };
			float currentSWTValue = SWTImage.at<float>(vertex.y, vertex.x);
			for (int i = 0; i < 4; i++)
			{
				if (vertex.x + NextPixelX[i] >= SWTImage.cols ||
					vertex.x + NextPixelX[i] < 0 ||
					vertex.y + NextPixelY[i] >= SWTImage.rows ||
					vertex.y + NextPixelY[i] < 0)
				{
					continue;
				}

				Point adjacentPixel = { vertex.x + NextPixelX[i], vertex.y + NextPixelY[i] };
				float adjacentSWTValue = SWTImage.at<float>(adjacentPixel.y, adjacentPixel.x);
				if (adjacentSWTValue <= 0)
				{
					continue;
				}
				const float allowedRatio = 3.0;
				if (adjacentSWTValue / currentSWTValue > allowedRatio || currentSWTValue / adjacentSWTValue > allowedRatio)
				{
					continue;
				}

				boost::add_edge(n, imageMap[adjacentPixel.y * SWTImage.cols + adjacentPixel.x], graph);
			}
		}

		std::vector<int> componentMap(vertexCount);
		int componentCount = connected_components(graph, &componentMap[0]);
		std::vector<ConnectedComponent> connectedComponents;
		LOG("Found " + to_string(componentCount) + " components");

		for (int i = 0; i < componentCount; i++)
		{
			ConnectedComponent component;
			connectedComponents.push_back(component);
		}

		for (int i = 0; i < vertexCount; i++)
		{
			SWTPoint vertex = reverseImageMap[i];
			connectedComponents[componentMap[i]].points.push_back(vertex);
		}

		return connectedComponents;
	}

	void DrawBoundingBox(Mat inputImage, const std::vector<ConnectedComponent> components)
	{
		for (std::vector<ConnectedComponent>::const_iterator component = components.begin(); component != components.end(); component++)
		{
			SWTPoint min = { INT_MAX, INT_MAX, 0 };
			SWTPoint max = { 0, 0, 0 };
			for each (SWTPoint point in component->points)
			{
				min.x = std::min(min.x, point.x);
				min.y = std::min(min.y, point.y);
				max.x = std::max(max.x, point.x);
				max.y = std::max(max.y, point.y);
			}
			rectangle(inputImage,
				cvPoint(min.x, min.y),
				cvPoint(max.x, max.y),
				CV_RGB(255, 0, 0),
				2);
		}
		LOG(inputImage);
	}

	void FindComponentStats(std::vector<ConnectedComponent>& components)
	{
		for (std::vector<ConnectedComponent>::iterator component = components.begin(); component != components.end(); component++)
		{
			float mean = 0;
			float variance = 0;
			int minX = INT_MAX;
			int minY = INT_MAX;
			int maxX = 0;
			int maxY = 0;
			for (std::vector<SWTPoint>::iterator point = component->points.begin(); point != component->points.end(); point++)
			{
				minX = std::min(minX, point->x);
				minY = std::min(minY, point->y);
				maxX = std::max(maxX, point->x);
				maxY = std::max(maxY, point->y);
				mean += point->SWT;
			}

			mean /= component->points.size();

			for (std::vector<SWTPoint>::iterator point = component->points.begin(); point != component->points.end(); point++)
			{
				variance += (point->SWT - mean) * (point->SWT - mean);
			}
			variance /= component->points.size();
			component->mean = mean;
			component->variance = variance;
			component->height = maxY - minY;
			component->width = maxX - minX;
		}
	}

	std::vector<ConnectedComponent> FilterComponents(std::vector<ConnectedComponent> &unfilteredComponents)
	{
		std::vector<ConnectedComponent> filteredComponents;
		FindComponentStats(unfilteredComponents);
		for (std::vector<ConnectedComponent>::iterator component = unfilteredComponents.begin(); component != unfilteredComponents.end(); component++)
		{
			if (component->variance > 0.5 * component->mean)
			{
				continue;
			}
			float aspectRatio = component->width / component->height;
			if (aspectRatio < 0.1 || aspectRatio > 10)
			{
				continue;
			}
			filteredComponents.push_back(*component);
		}
		return filteredComponents;
	}

	void TextDetection(Mat inputImage)
	{
		Mat grayImage = Mat(inputImage.rows, inputImage.cols, CV_8UC1);
		cvtColor(inputImage, grayImage, CV_RGB2GRAY);
		const int HIGH_THRESHOLD = 320;
		const int LOW_THRESHOLD = 175;
		Mat edgeImage = Mat(inputImage.rows, inputImage.cols, CV_8UC1);
		//blur(grayImage, grayImage,Size(3,1));
		Canny(grayImage, edgeImage, LOW_THRESHOLD, HIGH_THRESHOLD, 3);
		LOG(edgeImage);
		//Find Gradients
		Mat gradientX = Mat(inputImage.rows, inputImage.cols, CV_32FC1);
		Mat gradientY = Mat(inputImage.rows, inputImage.cols, CV_32FC1);
		Mat gradient;
		grayImage.convertTo(gradient, CV_32FC1, 1. / 255.);
		GaussianBlur(gradient, gradient, Size(5, 5), 0, 0);

		//LOG(gradient);

		Sobel(gradient, gradientX, CV_32FC1, 1, 0, CV_SCHARR);
		Sobel(gradient, gradientY, CV_32FC1, 0, 1, CV_SCHARR);

		cv::medianBlur(gradientX, gradientX, 3);
		cv::medianBlur(gradientY, gradientY, 3);
		LOG(gradientX);
		LOG(gradientY);
		Mat SWT = Mat(inputImage.rows, inputImage.cols, CV_32FC1, Scalar::all(-1));
		Rays ray;
		StrokeWidthTransform(edgeImage, gradientX, gradientY, SWT, ray);
		Mat normalSWT(SWT.rows, SWT.cols, CV_32FC1);
		normalizeImage(SWT, normalSWT);
		LOG(normalSWT);
		FindSWTMedians(ray);

		//Find Connected Components
		std::vector<ConnectedComponent> components = FindConnectedComponents(SWT);
		std::vector<ConnectedComponent> filteredComponents = FilterComponents(components);
		DrawBoundingBox(inputImage, filteredComponents);
	}

};

int main()
{
	cv::Mat inputImage = cv::imread("C:\\Users\\Eklavya\\Documents\\visual studio 2013\\Projects\\TextDetectcpp\\Debug\\Sample Card.png");
	//assert(inputImage.data);
	//BusinessCardReader::LOG(inputImage);
	//cv::resize(inputImage, inputImage, cv::Size(inputImage.cols/2,inputImage.rows/2));
	BusinessCardReader::TextDetection(inputImage);
	return 0;
}
