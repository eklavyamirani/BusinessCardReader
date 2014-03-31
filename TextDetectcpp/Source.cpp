#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <boost\graph\connected_components.hpp>
#include <boost\graph\adjacency_list.hpp>
#include <boost\graph\floyd_warshall_shortest.hpp>

using namespace std;

const float PI = 3.14159f;

struct Point2d {
	int x;
	int y;
	float SWT;
};

struct Ray {
	Point2d p;
	Point2d q;
	std::vector<Point2d> points;
};

void LOG(string message)
{
	std::cout << message << endl;
}

void StrokeWidthMedianFilter(IplImage* SWTImage, std::vector<Ray>& rays);
std::vector<std::vector<Point2d>> findConnectedComponents(IplImage* SWTImage);

void StrokeWidthTransform(IplImage* edgeImage,
	                      IplImage* gradientX,
						  IplImage* gradientY,
						  IplImage* SWTImage,
						  std::vector<Ray>& rays)
{
	const float step = 0.05f;
	for (int row = 0; row < edgeImage->height; row++)
	{
		const uchar* rowPtr = (uchar*)(edgeImage->imageData + row * edgeImage->widthStep);
		for (int column = 0; column < edgeImage->width; column++)
		{
			if (*rowPtr > 0)
			{
				Ray r;
				Point2d currentPoint;
				currentPoint.x = column;
				currentPoint.y = row;
				r.p = currentPoint;
				std::vector<Point2d> points;
				points.push_back(currentPoint);

				float G_X = CV_IMAGE_ELEM(gradientX, float, row, column);
				float G_Y = CV_IMAGE_ELEM(gradientY, float, row, column);
				int currXPix = column;
				int currYPix = row;

				float nextXPix = (float)column + 0.5f;
				float nextYPix = (float)row + 0.5f;

				float gradientMagnitude = sqrt((G_X * G_X + G_Y * G_Y));

				// For a light background with dark text, 
				// normalise Gradients as negative vectors. (Opposite Direction)
				G_X = - G_X / gradientMagnitude;
				G_Y = - G_Y / gradientMagnitude;

				// Now we trace the ray to the next edge point
				while (true)
				{
					nextXPix += G_X * step;
					nextYPix += G_Y * step;
					if (floor(nextXPix) != currXPix || floor(nextYPix) != currXPix)
					{
						currXPix = floor(nextXPix);
						currYPix = floor(nextYPix);

						if (currXPix < 0 || currXPix > edgeImage->width ||
							currYPix < 0 || currYPix > edgeImage->height)
						{
							// out of bounds
							break;
						}

						Point2d nextPoint;
						nextPoint.x = currXPix;
						nextPoint.y = currYPix;
						points.push_back(nextPoint);

						float G_Xq = CV_IMAGE_ELEM(gradientX, float, currYPix, currXPix);
						float G_Yq = CV_IMAGE_ELEM(gradientY, float, currYPix, currXPix);

						//End point of ray if the next pixel is an edgepoint on the edge image
						if (CV_IMAGE_ELEM(edgeImage, uchar, currYPix, currXPix) > 0)
						{
							r.q = nextPoint;

							// Reverse check
							// If the direction vectors are facing each other
							// then it is a stroke
							if (acos(G_X * -G_Xq + G_Y *-G_Yq) < PI / 2.0)
							{
								//Create the SWT Element
								float length = sqrt((float)r.q.x - (float)r.p.x) * ((float)r.q.x - (float)r.p.x) +
									           ((float)r.q.y - (float)r.p.y) * ((float)r.q.y - (float)r.p.y);

								for (std::vector<Point2d>::iterator pointIterator = points.begin();
									pointIterator != points.end();
									pointIterator++)
								{
									if (CV_IMAGE_ELEM(SWTImage, float, pointIterator->y, pointIterator->x) < 0)
										CV_IMAGE_ELEM(SWTImage, float, pointIterator->y, pointIterator->x) = length;
									else
										CV_IMAGE_ELEM(SWTImage, float, pointIterator->y, pointIterator->x) = std::min(length, CV_IMAGE_ELEM(SWTImage, float, pointIterator->y, pointIterator->x));
								}
								r.points = points;
								rays.push_back(r);
								break;
							}
						}
					}
				}
			}
			*rowPtr++;
		}
	}
}

void TextDetection(IplImage* inputImage)
{
	IplImage* grayScaleImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	cvCvtColor(inputImage, grayScaleImage, CV_RGB2GRAY);

	//Edge detection
	IplImage* edgeImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	double threshold_low = 180;
	double threshold_high = 320;
	cvCanny(grayScaleImage, edgeImage, threshold_low, threshold_high);

	//Find Gradient
	IplImage* gradientImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_32F, 1);
	cvConvertScale(edgeImage, gradientImage, (double)1/(double)255);
	cvSmooth(gradientImage, gradientImage, CV_GAUSSIAN, 5, 5);

	IplImage* gradientX = cvCreateImage(cvGetSize(gradientImage), IPL_DEPTH_32F, 1);
	IplImage* gradientY = cvCreateImage(cvGetSize(gradientImage), IPL_DEPTH_32F, 1);

	cvSobel(gradientImage, gradientX, 1, 0, CV_SCHARR);
	cvSobel(gradientImage, gradientY, 0, 1, CV_SCHARR);

	cvSmooth(gradientX, gradientX, CV_MEDIAN);
	cvSmooth(gradientY, gradientY, CV_MEDIAN);

	//cvReleaseImage(&grayScaleImage);
	cvReleaseImage(&gradientImage);

	//Create SWT Element
	IplImage* SWTImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_32F, 1);
	
	for (int row = 0; row < SWTImage->height; row++)
	{
		float* ptr = (float*)(SWTImage->imageData + row * SWTImage->widthStep);
		for (int column = 0; column < SWTImage->width; column++)
		{
			*ptr = -1;
			ptr++;
		}
	}
	std::vector<Ray> rays;
	StrokeWidthTransform(edgeImage,
		                 gradientX,
		                 gradientY,
		                 SWTImage,
		                 rays);

	// Second Pass
	StrokeWidthMedianFilter(SWTImage, rays);

	findConnectedComponents(SWTImage);
	IplImage* tempImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_32F, 1);
	cvConvertScale(SWTImage, tempImage, 255);
	cvSaveImage("SWT.png", tempImage);
	cvNamedWindow("Window", CV_WINDOW_AUTOSIZE);
	cvShowImage("Window", tempImage);
	cvWaitKey(0);
	cvReleaseImage(&tempImage);
	cvDestroyWindow("Window");
	
}

void StrokeWidthMedianFilter(IplImage* SWTImage, std::vector<Ray>& rays)
{
	for (std::vector<Ray>::iterator ray = rays.begin(); ray != rays.end(); ray++)
	{
		for (std::vector<Point2d>::iterator point = ray->points.begin(); point != ray->points.end(); point++)
		{
			point->SWT = CV_IMAGE_ELEM(SWTImage, float, point->y, point->x);
		}
		
		std::sort(ray->points.begin(), ray->points.end(), [](Point2d lhs, Point2d rhs) {
			return lhs.SWT < rhs.SWT;
		});

		float medianSWTValue = ray->points[ray->points.size() / 2].SWT;

		for (std::vector<Point2d>::iterator point = ray->points.begin(); point != ray->points.end(); point++)
		{
			CV_IMAGE_ELEM(SWTImage, float, point->y, point->x) = std::min(medianSWTValue, point->SWT);
		}
	}
}

std::vector<std::vector<Point2d>> findConnectedComponents(IplImage* SWTImage)
{
	std::map<int, int> imageMap;
	std::map<int, Point2d> reverseImageMap;
	//First Step: No. the vertices in the image to form a graph
	for (int row = 0; row < SWTImage->height; row++)
	{
		float* pixelPtr = (float*)(SWTImage->imageData + row * SWTImage->widthStep);
		int counter = 0;
		for (int col = 0; col < SWTImage->width; col++)
		{
			if (*pixelPtr > 0)
			{
				imageMap[row * SWTImage->width + col] = counter;
				Point2d p;
				p.x = col;
				p.y = row;
				reverseImageMap[counter] = p;
				counter++;
			}
			pixelPtr++;
		}

		boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> graph(counter);

		for (int row = 0; row < SWTImage->height; row++)
		{
			float* pixelPtr = (float*)(SWTImage->imageData + row * SWTImage->width);
			for (int col = 0; col < SWTImage->width; col++)
			{
				float lastSW = 0;
				if (*pixelPtr > 0)
				{
					int currentPixel = imageMap[row * SWTImage->width + col];
					if (col + 1 < SWTImage->width)
					{
						float currentValue = CV_IMAGE_ELEM(SWTImage, float, col + 1, row);
						if (currentValue > 0 && !(*pixelPtr / currentValue > 3.0) && !(currentValue / *pixelPtr > 3.0))
						{
							boost::add_edge(currentPixel, imageMap[row * SWTImage->width + col + 1], graph);
						}
					}

					if (col + 1 < SWTImage->width && row + 1 < SWTImage->height)
					{
						float currentValue = CV_IMAGE_ELEM(SWTImage, float, col + 1, row + 1);
						if (currentValue > 0 && !(*pixelPtr / currentValue > 3.0) && !(currentValue / *pixelPtr > 3.0))
						{
							boost::add_edge(currentPixel, imageMap[(row + 1) * SWTImage->width + col + 1], graph);
						}
					}

					if (row + 1 < SWTImage->height)
					{
						float currentValue = CV_IMAGE_ELEM(SWTImage, float, col, row + 1);
						if (currentValue > 0 && !(*pixelPtr / currentValue > 3.0) && !(currentValue / *pixelPtr > 3.0))
						{
							boost::add_edge(currentPixel, imageMap[(row + 1) * SWTImage->width + col], graph);
						}
					}

					if (col - 1 > 0 && row + 1 < SWTImage->height)
					{
						float currentValue = CV_IMAGE_ELEM(SWTImage, float, col - 1, row + 1);
						if (currentValue > 0 && !(*pixelPtr / currentValue > 3.0) && !(currentValue / *pixelPtr > 3.0))
						{
							boost::add_edge(currentPixel, imageMap[(row + 1) * SWTImage->width + col - 1], graph);
						}
					}
				}
				pixelPtr++;
			}

			std::vector<int> c(counter);
			int componentCount = boost::connected_components(graph, &c[0]);
			std::vector<std::vector<Point2d>> connectedComponents(componentCount);
			LOG("before filtering: " + to_string(componentCount) + " components and " + to_string(counter) + " vertices");

			for (int i = 0; i < componentCount; i++)
			{
				std::vector<Point2d> temp;
				connectedComponents.push_back(temp);
			}

			for (int i = 0; i < counter; i++)
			{
				std::vector<Point2d> currentComponent = connectedComponents[c[i]];
				Point2d currentPoint = reverseImageMap[i];
				currentComponent.push_back(currentPoint);
			}

			return connectedComponents;

		}

	}
}

int main()
{
	IplImage* inputImage = cvLoadImage(
		"C:\\Users\\Eklavya\\Documents\\visual studio 2013\\Projects\\TextDetectcpp\\Debug\\Sample Card.png");
	if (!inputImage)
	{
		throw new exception("File not found");
	}
	TextDetection(inputImage);
	return 0;
}