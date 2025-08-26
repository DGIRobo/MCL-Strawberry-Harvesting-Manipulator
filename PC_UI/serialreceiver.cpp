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
    // 송/수신 모두 할 것이므로 ReadWrite로 오픈
    return m_port.open(QIODevice::ReadWrite);
}

void SerialReceiver::close() { m_port.close(); }

// ----- 여기는 수신 코드(onReadyRead/parseFrame) -----
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

    // 수정: 수신 필드만 개별 복사
    gTelemetry.t          = tmp.t;
    gTelemetry.dt         = tmp.dt;
    gTelemetry.robot_mode = tmp.robot_mode;

    for (int m = 0; m < 3; ++m) {
        gTelemetry.motors[m] = tmp.motors[m];
    }

    gTelemetry.q_bi    = tmp.q_bi;
    gTelemetry.pos_ref = tmp.pos_ref;
    gTelemetry.pos     = tmp.pos;
    gTelemetry.vel     = tmp.vel;
    gTelemetry.pos_I   = tmp.pos_I;
    gTelemetry.pos_pid = tmp.pos_pid;
    // (Tx 필드: target_position / posx/y/z_pid_gain 은 건드리지 않음)

    gTelemetryLock.unlock();

    emit frameParsed();
}

// ----- 여기는 송신 코드(sendTxFrameFromGlobals/buildTxFrame/) -----
// 1) 전역에서 값 읽어와 보내기
bool SerialReceiver::sendTxFrameFromGlobals()
{
    std::array<double,4> target;
    std::array<double,5> gx, gy, gz;

    {   // 전역 읽기 (락)
        QReadLocker lock(&gTelemetryLock);
        target = gTelemetry.target_position;
        gx     = gTelemetry.posx_pid_gain;
        gy     = gTelemetry.posy_pid_gain;
        gz     = gTelemetry.posz_pid_gain;
    }
    return sendTxFrame(target, gx, gy, gz);
}

// 2) 직접 값으로 보내기
bool SerialReceiver::sendTxFrame(const std::array<double,4>& target,
                                 const std::array<double,5>& posx,
                                 const std::array<double,5>& posy,
                                 const std::array<double,5>& posz)
{
    if (!m_port.isOpen()) return false;
    const QByteArray frame = buildTxFrame(target, posx, posy, posz);

    qint64 sent = 0;
    while (sent < frame.size()) {
        const qint64 n = m_port.write(frame.constData() + sent, frame.size() - sent);
        if (n < 0) return false;                  // write 에러
        sent += n;
        if (!m_port.waitForBytesWritten(50))      // 타임아웃 시 실패 처리
            return false;
    }
    return true;
}

// 3) 실제 프레임 빌더
QByteArray SerialReceiver::buildTxFrame(const std::array<double,4>& target,
                                        const std::array<double,5>& posx,
                                        const std::array<double,5>& posy,
                                        const std::array<double,5>& posz)
{
    auto d = [](double v){ return QLocale::c().toString(v, 'f', 3); }; // 소수 3자리 고정
    QStringList f;

    // target_position (4)
    for (double v : target) f << d(v);

    // posx/y/z_pid_gain (각 5)
    for (double v : posx) f << d(v);
    for (double v : posy) f << d(v);
    for (double v : posz) f << d(v);

    // 결과: 총 4 + 5 + 5 + 5 = 19 토큰
    const QString body = f.join(", ");
    QByteArray out;
    out.reserve(body.size() + 4);
    out += '[';
    out += body.toLatin1();
    out += "]\r\n";
    return out;
}
