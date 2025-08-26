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
    auto serial = new SerialReceiver(QStringLiteral("COM3"), 115200, &w);
    if (!serial->open()) {
        qWarning("Failed to open serial port COM3 @ 115200");
        QMessageBox::critical(&w, "Serial", "Open failed");
    }

    // 2) 새 프레임이 파싱되면: (1) 즉시 수신 → (2) 수신값 읽어 전역에 반영 → (3) UI 갱신 → (4) UI값 읽어 전역에 반영 → (5) 즉시 송신
    QObject::connect(
        // (1) 즉시 수신 → (2) 수신값 읽어 전역에 반영
        serial, &SerialReceiver::frameParsed,
        &w, [serial, &w]{
            // (3) UI 갱신
            w.updateWindow();

            // (4) UI값 읽어 전역에 반영 → (5) 전역값을 [ ... ] 로 패킹해 송신
            if (!serial->sendTxFrameFromGlobals()) {
                qWarning("TX failed");
            }
        },
        Qt::QueuedConnection // 나중에 시리얼을 워커 스레드로 옮겨도 안전
    );

    // 3) 시작 시 1회 그리기(데이터가 비어 있어도 초기 화면 정리 용)
    QTimer::singleShot(0, &w, &MainWindow::updateWindow);

    // (선택) 보조 타이머: 시리얼이 잠시 멈춰도 UI가 주기적으로 살아있는지 확인하고 싶을 때
    // auto keepAlive = new QTimer(&w);
    // QObject::connect(keepAlive, &QTimer::timeout, &w, &MainWindow::updateWindow);
    // keepAlive->start(250); // 4Hz 백업 갱신

    return app.exec();
}
