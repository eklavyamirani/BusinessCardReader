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

#ifndef ApplicationUI_HPP_
#define ApplicationUI_HPP_

#include <QObject>
#include <bb/cascades/multimedia/Camera>
#include <bb/cascades/multimedia/CameraSettings>
#include <bb/cascades/TouchEvent>
#include <bb/cascades/ImageView>
#include <bb/cascades/TextArea>
#include <bb/cascades/Image>
#include <bb/System/SystemToast>
#include <bb/multimedia/SystemSound>

#include "PreviewFrameProcessor.hpp"

using namespace bb::cascades::multimedia;
using namespace bb::cascades;

namespace bb
{
    namespace cascades
    {
        class Application;
        class LocaleHandler;
    }
}

class QTranslator;

/*!
 * @brief Application object
 *
 *
 */

class ApplicationUI : public QObject
{
    Q_OBJECT
public:
    ApplicationUI(bb::cascades::Application *app);
    ~ApplicationUI();

    Q_INVOKABLE
    void changeFlashMode(int);
public slots:
    void onSystemLanguageChanged();
    void onTouch();
    void onCameraOpened();
    void onViewfinderStarted();
    void onShutterFired();
    void onPhotoSaved(const QString&, quint64 );
    void onGetResponse(QString&);
    void StartCapturing();
    void StopCapturing();
    void processBusinessCard();
private:
    bool m_photoBeingTaken;
    ImageView* m_pCardHolder;
    TextArea* m_pResponseHolder;
    QThread* m_pProcessorThread;
    PreviewFrameProcessor* m_pProcessor;
    Camera* m_pCamera;
    CameraSettings* m_pCameraSettings;
    QString qStrFileName;
    void showToast(QString message);


    QTranslator* m_pTranslator;
    bb::cascades::LocaleHandler* m_pLocaleHandler;
};

#endif /* ApplicationUI_HPP_ */
