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

import bb.cascades 1.2
import bb.multimedia 1.0
import bb.cascades.multimedia 1.0
import Network.HttpPost 1.0

Page {
    titleBar: TitleBar {
        title: "Business Card Reader Client"
    }
    content: Container {
        //Todo: fill me with QML
        Container {
            Camera {
                objectName: "qmlCameraObj"
                id: cameraObject
                visible: false
                onTouch: {
                    buttonSendImage.visible = true;
                    cameraObject.visible = false;
                    _appObject.StopCapturing();
                }
            }
            
            Button {
                id: buttonCaptureImage
                verticalAlignment: VerticalAlignment.Center
                text: "Capture a New Image"
                onClicked: {
                    buttonCaptureImage.visible = false;
                    _appObject.StartCapturing();
                    cameraObject.visible = true;
                }
            }
            
            Button {
                id: buttonSendImage
                text: "Upload Image"
                visible: false;
                onClicked: {
                    _appObject.StopCapturing();
                    _appObject.processBusinessCard();
                    progressIndicator.running = true;
                    progressIndicator.visible = true;
                    buttonSendImage.visible = false;
                }
            }
            ActivityIndicator {
                id: progressIndicator
                horizontalAlignment: HorizontalAlignment.Center
                verticalAlignment: VerticalAlignment.Center                
                visible: false;
            }
            
            ImageView {
                objectName: "CardHolder"
                visible: false
            }
            
            TextArea {
                objectName: "ResponseHolder"
                id: responseHolder
                visible: false
                preferredHeight: 500
                editable: false
                onTextChanged: {
                    progressIndicator.running = false;
                    progressIndicator.visible = false;
                }
            }
            
            attachedObjects: HttpPost {
                id:netPost
                onComplete: {
                    responseHolder.text = info;
                    buttonSendImage.visible = false;
                }
            }
        }
        
        
        
        
    }
}
