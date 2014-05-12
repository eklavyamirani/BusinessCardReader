/*
 * PreviewFrameProcessor.cpp
 *
 *  Created on: 11-May-2014
 *      Author: Eklavya
 */
#include "PreviewFrameProcessor.hpp"

#include <bb/cascades/multimedia/Camera>

using namespace bb::cascades;
using namespace bb::cascades::multimedia;

PreviewFrameProcessor::PreviewFrameProcessor()
{
	pCameraObj = NULL;
}

void PreviewFrameProcessor::setCamera(Camera* camera)
{
	pCameraObj = camera;
}

void PreviewFrameProcessor::onPreviewFrameAvailable(SharedUCharPointer previewBuffer,
		quint64 size,
		unsigned int width,
		unsigned int height)
{
	Q_UNUSED(width);
	Q_UNUSED(height);

	pCameraObj->addPreviewBuffer(previewBuffer, size);

}




