#ifndef SERIALRECEIVER_H
#define SERIALRECEIVER_H

#pragma once
#include <QObject>
#include <QSerialPort>
#include <QByteArray>

class SerialReceiver : public QObject {
    Q_OBJECT
public:
    // UART 통신 수신 함수
    explicit SerialReceiver(const QString& portName,
                            int baud = 115200,
                            QObject* parent = nullptr);

    bool open();
    void close();

    // UART 통신 송신 함수
    bool sendTxFrameFromGlobals();

    // (직접 값으로 보낼 때 쓰고 싶으면 이 오버로드 사용)
    bool sendTxFrame(const std::array<double,4>& target,
                     const std::array<double,5>& posx,
                     const std::array<double,5>& posy,
                     const std::array<double,5>& posz);

signals:
    void frameParsed();   // 파싱 완료 알림(옵션, UI에서 필요하면 연결)

private slots:
    void onReadyRead();

private:
    QSerialPort m_port;
    QByteArray  m_buf;

    void parseFrame(const QByteArray& payload); // payload: '['와 ']' 사이

    // 내부 포맷터
    static QByteArray buildTxFrame(const std::array<double,4>& target,
                                   const std::array<double,5>& posx,
                                   const std::array<double,5>& posy,
                                   const std::array<double,5>& posz);
};

#endif // SERIALRECEIVER_H
