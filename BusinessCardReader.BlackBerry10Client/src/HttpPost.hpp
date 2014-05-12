/*
 * HttpPost.hpp
 *
 *  Created on: 11-May-2014
 *      Author: Eklavya
 */

#ifndef HTTPPOST_HPP_
#define HTTPPOST_HPP_

#include <QObject>
#include <QNetworkAccessManager>


class HttpPost : public QObject
{
	Q_OBJECT
public:
	HttpPost(QObject* parent = 0);
	virtual ~HttpPost(){};

public Q_SLOTS:
	void post(const QString&);
	public Q_SLOTS:
	void onGetReply();

Q_SIGNALS:
	void complete(QString& info);

private:
	QNetworkAccessManager* m_pNetworkAccessManager;
};


#endif /* HTTPPOST_HPP_ */
