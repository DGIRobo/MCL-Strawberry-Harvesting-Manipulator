#include "mainwindow.h"
#include "serialreceiver.h"
#include <QTimer>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow w;
    w.show();

    // 1) 시리얼 열기 (포트명/보레이트는 실제 환경에 맞게 변경)
    auto serial = new SerialReceiver(QStringLiteral("COM3"), 921600, &w);
    if (!serial->open()) {
        qWarning("Failed to open serial port COM3 @ 921600");
        QMessageBox::critical(&w, "Serial", "Open failed");
    }

    // 2) 새 프레임이 파싱될 때마다 UI 갱신
    QObject::connect(serial, &SerialReceiver::frameParsed,
                     &w,      &MainWindow::updateWindow,
                     Qt::QueuedConnection);

    // 3) 송신은 별도 타이머 주기
    auto txTimer = new QTimer(&w);
    txTimer->setTimerType(Qt::PreciseTimer);
    txTimer->setInterval(5); // 원하는 송신 주기(ms)로 조절: 20~50 권장
    QObject::connect(txTimer, &QTimer::timeout, &w, [serial, &w]{
        if (!serial->sendTxFrameFromGlobals()) {
            qWarning("TX failed");
        }
    });
    txTimer->start();

    // 4) 시작 시 1회 그리기(데이터가 비어 있어도 초기 화면 정리 용)
    QTimer::singleShot(0, &w, &MainWindow::updateWindow);

    // (선택) 보조 타이머: 시리얼이 잠시 멈춰도 UI가 주기적으로 살아있는지 확인하고 싶을 때
    // auto keepAlive = new QTimer(&w);
    // QObject::connect(keepAlive, &QTimer::timeout, &w, &MainWindow::updateWindow);
    // keepAlive->start(250); // 4Hz 백업 갱신

    return app.exec();
}
