#include "serialreceiver.h"
#include "globalVariables.h"
#include <QStringList>
#include <QLocale>

SerialReceiver::SerialReceiver(const QString& portName, int baud, QObject* parent)
    : QObject(parent)
{
    m_port.setPortName(portName);
    m_port.setBaudRate(baud);
    m_port.setDataBits(QSerialPort::Data8);
    m_port.setParity(QSerialPort::NoParity);
    m_port.setStopBits(QSerialPort::OneStop);
    m_port.setFlowControl(QSerialPort::NoFlowControl);
    connect(&m_port, &QSerialPort::readyRead, this, &SerialReceiver::onReadyRead);
}

bool SerialReceiver::open() {
    return m_port.open(QIODevice::ReadOnly);
}

void SerialReceiver::close() { m_port.close(); }

void SerialReceiver::onReadyRead()
{
    m_buf += m_port.readAll();

    // '[' ... ']' 프레임을 연속 추출
    for (;;) {
        int start = m_buf.indexOf('[');
        if (start < 0) { m_buf.clear(); break; }   // 시작 전 쓰레기 버림
        if (start > 0) m_buf.remove(0, start);     // '[' 앞은 버림

        int end = m_buf.indexOf(']');
        if (end < 0) break;                        // 아직 끝 안 옴 → 다음 readyRead 때 재시도

        int frameEnd = end + 1; // ']' 바로 뒤
        // CRLF 확인(프레임 경계 유효성)
        bool hasCRLF = (frameEnd + 1 < m_buf.size()
                        && m_buf[frameEnd] == '\r'
                        && m_buf[frameEnd+1] == '\n');
        if (!hasCRLF) {
            // ']'는 봤지만 CRLF가 아직 안 왔거나 깨진 경우 → 다음 readyRead까지 대기
            // (만약 오래도록 CRLF가 안 오면, 버퍼를 버리고 재동기화해도 됨)
            break;
        }

        const QByteArray payload = m_buf.mid(1, end - 1); // '['와 ']' 제외
        m_buf.remove(0, frameEnd + 2); // ']' + "\r\n" 제거

        parseFrame(payload);
    }
}

static bool toD(const QString& s, double& out) {
    bool ok=false; out = QLocale::c().toDouble(s.trimmed(), &ok); return ok;
}
static bool toI(const QString& s, int& out) {
    bool ok=false; out = s.trimmed().toInt(&ok, 10); return ok;
}

void SerialReceiver::parseFrame(const QByteArray& payload)
{
    // 예: "  1.234,   0.002,       1,   1,   5, 0.123, ... "
    const QString text = QString::fromLatin1(payload);
    QStringList tok = text.split(',', Qt::SkipEmptyParts);

    // 공백 제거
    for (QString& t : tok) t = t.trimmed();

    // 프레임 형식 유효성: 정확히 30개 (3 + 3*3 + 6*3)
    static constexpr int EXPECTED_TOKENS = 30;
    if (tok.size() != EXPECTED_TOKENS) return;

    Telemetry tmp; int i = 0;

    // 숫자 변환 성공 여부만 검사(형식 유효성의 일부)
    if (!toD(tok[i++], tmp.t))  return;
    if (!toD(tok[i++], tmp.dt)) return;
    if (!toI(tok[i++], tmp.robot_mode)) return;

    // 모터 3 * (id, mode, control)
    for (int m = 0; m < 3; ++m) {
        if (!toI(tok[i++], tmp.motors[m].id))      return;
        if (!toI(tok[i++], tmp.motors[m].mode))    return;
        if (!toD(tok[i++], tmp.motors[m].control)) return;
    }

    auto fill3 = [&](std::array<double,3>& a)->bool {
        return toD(tok[i++], a[0]) && toD(tok[i++], a[1]) && toD(tok[i++], a[2]);
    };

    if (!fill3(tmp.q_bi))       return;
    if (!fill3(tmp.pos_ref))    return;
    if (!fill3(tmp.pos))        return;
    if (!fill3(tmp.vel))        return;
    if (!fill3(tmp.pos_I))      return;
    if (!fill3(tmp.pos_pid))    return;

    // 여기까지 통과하면 "형식상" 정상 프레임 → 전역에 반영 (락으로 보호)
    gTelemetryLock.lockForWrite();
    gTelemetry = tmp;
    gTelemetryLock.unlock();

    emit frameParsed();
}
