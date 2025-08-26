#ifndef SERIALRECEIVER_H
#define SERIALRECEIVER_H

#pragma once
#include <QObject>
#include <QSerialPort>
#include <QByteArray>

class SerialReceiver : public QObject {
    Q_OBJECT
public:
    explicit SerialReceiver(const QString& portName,
                            int baud = 921600,
                            QObject* parent = nullptr);

    bool open();
    void close();

signals:
    void frameParsed();   // 파싱 완료 알림(옵션, UI에서 필요하면 연결)

private slots:
    void onReadyRead();

private:
    QSerialPort m_port;
    QByteArray  m_buf;

    void parseFrame(const QByteArray& payload); // payload: '['와 ']' 사이
};

#endif // SERIALRECEIVER_H
