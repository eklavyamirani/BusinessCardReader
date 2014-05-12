/*
 * Copyright (c) 2011-2013 BlackBerry Limited.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "applicationui.hpp"
#include "HttpPost.hpp"

#include <bb/cascades/Application>
#include <bb/cascades/QmlDocument>
#include <bb/cascades/AbstractPane>
#include <bb/cascades/LocaleHandler>
#include <bb/cascades/multimedia/Camera>
#include <bb/cascades/multimedia/CameraSettings>
#include <bb/cascades/multimedia/CameraFocusMode>
#include <bb/cascades/Image>
#include <bb/cascades/ImageView>
#include <bb/cascades/TextArea>
#include <bb/system/SystemToast>
#include <bb/system/InvokeManager>
#include <bb/multimedia/SystemSound>

using namespace bb::cascades;
using namespace bb::cascades::multimedia;
using namespace bb::system;

ApplicationUI::ApplicationUI(bb::cascades::Application *app) :
        QObject(app)
{
    // prepare the localization
    m_pTranslator = new QTranslator(this);
    m_pLocaleHandler = new LocaleHandler(this);

    bool res = QObject::connect(m_pLocaleHandler, SIGNAL(systemLanguageChanged()), this, SLOT(onSystemLanguageChanged()));
    // This is only available in Debug builds
    Q_ASSERT(res);
    // Since the variable is not used in the app, this is added to avoid a
    // compiler warning
    Q_UNUSED(res);

    // initial load
    onSystemLanguageChanged();
    qmlRegisterType<HttpPost>("Network.HttpPost", 1, 0, "HttpPost");

    qmlRegisterType<QTimer>("my.library", 1, 0, "QTimer");

    // Create scene document from main.qml asset, the parent is set
    // to ensure the document gets destroyed properly at shut down.
    QmlDocument *qml = QmlDocument::create("asset:///main.qml").parent(this);
    qml->setContextProperty("_appObject", this);

    // Create root object for the UI
    AbstractPane *root = qml->createRootObject<AbstractPane>();

    // Set created root object as the application scene
    app->setScene(root);

    //Initialize camera
    m_pCamera = root->findChild<Camera*>("qmlCameraObj");
    m_pCardHolder = root->findChild<bb::cascades::ImageView*>("CardHolder");
    m_pResponseHolder = root->findChild<bb::cascades::TextArea*>("ResponseHolder");
    QList<CameraUnit::Type> cameraList = m_pCamera->supportedCameras();

    if (cameraList.empty() || !cameraList.contains(CameraUnit::Rear))
    {
    	showToast("Rear camera Unavailable!");
    	return;
    }


    m_pProcessor = new PreviewFrameProcessor();
    m_pProcessorThread = new QThread();
    m_pCameraSettings = new CameraSettings();

    // Check that all connections to signals are successful.
        bool result;
        Q_UNUSED(result);

        // Connect signals and slots.
        result = connect(m_pCamera,
                         SIGNAL(touch(bb::cascades::TouchEvent*)),
                         this,
                         SLOT(onTouch()));

        Q_ASSERT(result);

        result = connect(m_pCamera,
                         SIGNAL(cameraOpened()),
                         this,
                         SLOT(onCameraOpened()));

        Q_ASSERT(result);

        result = connect(m_pCamera,
                         SIGNAL(shutterFired()),
                         this,
                         SLOT(onShutterFired()));
        Q_ASSERT(result);

        result = connect(m_pCamera,
                         SIGNAL(viewfinderStarted()),
                         this,
                         SLOT(onViewfinderStarted()));

        Q_ASSERT(result);

        result = connect(m_pCamera,
                         SIGNAL(photoSaved(const QString &, quint64)),
                         this,
                         SLOT(onPhotoSaved(const QString &, quint64)));

        Q_ASSERT(result);

        result = connect(m_pCamera,
                         SIGNAL(previewFrameAvailable(bb::cascades::multimedia::SharedUCharPointer, quint64, unsigned int, unsigned int, unsigned int)),
                         m_pProcessor,
                         SLOT(onPreviewFrameAvailable(bb::cascades::multimedia::SharedUCharPointer, quint64, unsigned int, unsigned int)));

        Q_ASSERT(result);

        result = connect(m_pProcessorThread,
                         SIGNAL(finished()),
                         m_pProcessorThread,
                         SLOT(quit()));

        Q_ASSERT(result);


    m_pCamera->getSettings(m_pCameraSettings);

    m_photoBeingTaken = false;

    m_pProcessor->setCamera(m_pCamera);

}

void ApplicationUI::onSystemLanguageChanged()
{
    QCoreApplication::instance()->removeTranslator(m_pTranslator);
    // Initiate, load and install the application translation files.
    QString locale_string = QLocale().name();
    QString file_name = QString("BCRClient_%1").arg(locale_string);
    if (m_pTranslator->load(file_name, "app/native/qm")) {
        QCoreApplication::instance()->installTranslator(m_pTranslator);
    }
}

void ApplicationUI::showToast(QString message)
{
	SystemToast* toast = new SystemToast(this);
	toast->setBody(message);
	toast->setPosition(SystemUiPosition::MiddleCenter);
	toast->show();
}

ApplicationUI::~ApplicationUI()
{
	m_pCamera->close();

	//Clean
	m_pCamera->deleteLater();
	m_pCameraSettings->deleteLater();
	m_pProcessor->deleteLater();
	m_pProcessorThread->deleteLater();
}

void ApplicationUI::onTouch()
{
	if (!m_photoBeingTaken)
	{
		m_photoBeingTaken = true;
		m_pCamera->capturePhoto();
	}
}

void ApplicationUI::onCameraOpened()
{
	unsigned long bufferSize = m_pCamera->previewBufferSize();

	for(int i=0; i<4; i++)
	{
		QSharedPointer<unsigned char> buffer(new unsigned char[bufferSize]);
		m_pCamera->addPreviewBuffer(buffer, bufferSize);
	}

	m_pCamera->getSettings(m_pCameraSettings);
	m_pCameraSettings->setFocusMode(CameraFocusMode::ContinuousAuto);

	m_pCamera->startViewfinder();
}

void ApplicationUI::onViewfinderStarted()
{
	m_photoBeingTaken = false;
}

void ApplicationUI::onShutterFired()
{
	//bb::multimedia::SystemSound::play(SystemSound::CameraShutterEvent);
}

void ApplicationUI::onPhotoSaved(const QString& fileName, quint64 fileSize)
{
	Q_UNUSED(fileName);
	Q_UNUSED(fileSize);

	m_photoBeingTaken = false;
	//m_pCamera->setProperty("visible", false);
	qDebug()<<"set camera visibility to false";
	//disconnect(m_pCamera,
	//		SIGNAL(previewFrameAvailable(bb::cascades::multimedia::SharedUCharPointer, quint64, unsigned int, unsigned int, unsigned int)),
	//			                         m_pProcessor,
	//			                         SLOT(onPreviewFrameAvailable(bb::cascades::multimedia::SharedUCharPointer, quint64, unsigned int, unsigned int)));
	m_pCardHolder->setImage(Image(fileName));
	m_pCardHolder->setProperty("visible", true);
	qStrFileName = fileName;
	//processBusinessCard(fileName);
}

void ApplicationUI::changeFlashMode(int newFlashMode)
{
	m_pCamera->getSettings(m_pCameraSettings);

	m_pCameraSettings->setFlashMode(CameraFlashMode::Type(newFlashMode));
	m_pCamera->applySettings(m_pCameraSettings);
}

void ApplicationUI::processBusinessCard()
{
	m_pCardHolder->setProperty("visible", false);
	QFile file(qStrFileName);
	if(!file.open(QFile::ReadOnly))
	{
		showToast("Error, Could not load file!");
		return;
	}
	QByteArray base64Encoded = file.readAll().toBase64();
	file.close();


	HttpPost* postObject = new HttpPost(this);
	postObject->post(base64Encoded);

	bool ok = connect(postObject,
			SIGNAL(complete(QString&)),
			this,
			SLOT(onGetResponse(QString&)));
	Q_ASSERT(ok);
}

void ApplicationUI::StartCapturing()
{
	m_pCamera->open();
    m_pProcessor->moveToThread(m_pProcessorThread);
    m_pProcessorThread->start();
}

void ApplicationUI::StopCapturing()
{
	m_pProcessorThread->exit(0);
}

void ApplicationUI::onGetResponse(QString& response)
{
	m_pCardHolder->setProperty("visible", false);
	m_pResponseHolder->setProperty("visible", true);
	m_pResponseHolder->setProperty("text", response);

	QDir vcfPath;
	vcfPath.mkdir(QDir::tempPath()+"/data/");
	vcfPath.cd(QDir::tempPath()+"/data/");
	QFile vcfCard(QDir::tempPath()+"/data/readContact.vcf");
	if(!vcfCard.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		showToast("Error Creating new file");
		return;
	}
	QTextStream out(&vcfCard);
	out << response;
	vcfCard.close();

	InvokeManager manager;
	InvokeRequest request;
	request.setTarget("sys.pim.contacts.card.viewer");
	request.setAction("bb.action.VIEW");
	request.setUri("file://" + QDir::tempPath()+"/data/readContact.vcf");
	manager.invoke(request);
}
