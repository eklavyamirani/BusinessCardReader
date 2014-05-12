/*
 * HttpPost.cpp
 *
 *  Created on: 11-May-2014
 *      Author: Eklavya
 */
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>

#include "HttpPost.hpp"

HttpPost::HttpPost(QObject* parent)
		 : QObject(parent),
		   m_pNetworkAccessManager(new QNetworkAccessManager(this))
{

}

void HttpPost::post(const QString &body)
{
    const QUrl url("http://businesscardreader.cloudapp.net/api/values");
    fprintf(stdout ,"POSTing data");
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QNetworkReply* reply = m_pNetworkAccessManager->post(request, "\"" + body.toUtf8() + "\"");
    bool ok = connect(reply, SIGNAL(finished()), this, SLOT(onGetReply()));
    Q_ASSERT(ok);
    Q_UNUSED(ok);
}

void HttpPost::onGetReply()
{
    QNetworkReply* reply = qobject_cast<QNetworkReply*>(sender());

    QString response;
    if (reply) {
        if (reply->error() == QNetworkReply::NoError) {
            const int available = reply->bytesAvailable();
            if (available > 0) {
                const QByteArray buffer(reply->readAll());
                response = QString::fromUtf8(buffer);
            }
        } else {
            response = tr("Error: %1 status: %2").arg(reply->errorString(), reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toString());
            qDebug()<<response;
        }

        reply->deleteLater();
    }

    if (response.trimmed().isEmpty()) {
        response = tr("Unable to retrieve post response");
    }

    qDebug()<<"response is " << response.trimmed();

    emit complete(response);
}




