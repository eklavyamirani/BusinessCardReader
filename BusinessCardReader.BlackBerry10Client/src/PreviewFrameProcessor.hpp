#ifndef PREVIEWFRAMEBUFFER_HPP
#define PREVIEWFRAMEBUFFER_HPP

#include <QObject>
#include <QThread>

#include <bb/cascades/multimedia/Camera>

namespace bb
{
	namespace cascades
	{
		class Application;
	}
}

class PreviewFrameProcessor : public QObject
{
	Q_OBJECT
	public:
	PreviewFrameProcessor();
	virtual ~PreviewFrameProcessor(){};
	void setCamera(bb::cascades::multimedia::Camera*);

	public slots:
	void onPreviewFrameAvailable(bb::cascades::multimedia::SharedUCharPointer,
								quint64,
								unsigned int,
								unsigned int);

	private:
	bb::cascades::multimedia::Camera* pCameraObj;
};

#endif
