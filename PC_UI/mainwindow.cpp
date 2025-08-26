#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    timeTicker.reset(new QCPAxisTickerTime());
    timeTicker->setTimeFormat("%m:%s");

    createPlot(ui->q1Plot);
    createPlot(ui->qmPlot);
    createPlot(ui->qbPlot);

    createPlot(ui->xposPlot);
    createPlot(ui->yposPlot);
    createPlot(ui->zposPlot);

    createPlot(ui->xposIctrlPlot);
    createPlot(ui->yposIctrlPlot);
    createPlot(ui->zposIctrlPlot);

    createPlot(ui->q1CurrentPlot);
    createPlot(ui->qmCurrentPlot);
    createPlot(ui->qbCurrentPlot);

    // 1) 텍스트 확정 시마다 전역에 반영 (valueChanged나 editingFinished 중 택1)
    auto wire = [&](QDoubleSpinBox* sb){
        connect(sb, &QDoubleSpinBox::editingFinished,
                this, &MainWindow::readUIParams);
    };

    // ---- 대상 위치 4개 ----
    wire(ui->TaskTime);
    wire(ui->targetXpos);
    wire(ui->targetYpos);
    wire(ui->targetZpos);

    // ---- PID gains X/Y/Z 각 5개 ----
    wire(ui->xposPctrlGain);
    wire(ui->xposIctrlGain);
    wire(ui->xposDctrlGain);
    wire(ui->xposCutoffFreq);
    wire(ui->xposWindupGain);

    wire(ui->yposPctrlGain);
    wire(ui->yposIctrlGain);
    wire(ui->yposDctrlGain);
    wire(ui->yposCutoffFreq);
    wire(ui->yposWindupGain);

    wire(ui->zposPctrlGain);
    wire(ui->zposIctrlGain);
    wire(ui->zposDctrlGain);
    wire(ui->zposCutoffFreq);
    wire(ui->zposWindupGain);

    // 시작 시 한 번 전역에 초기값 저장
    readUIParams();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateWindow()
{
    // 1) x축 시간 t (초) 계산: 시작 이후 경과시간
    Telemetry data;
    gTelemetryLock.lockForRead();
    data = gTelemetry;                 // 스냅샷
    gTelemetryLock.unlock();

    // 2) LED 상태 갱신 (그대로 유지)
    if (data.motors[0].mode == 1) { q1LedOn(); }
    else { q1LedOff(); }

    if (data.motors[1].mode == 1) { qmLedOn(); }
    else { qmLedOff(); }

    if (data.motors[2].mode == 1) { qbLedOn(); }
    else { qbLedOff(); }

    if (data.robot_mode == 1) { RobotLedOn(); }
    else { RobotLedOff(); }

    // 4) 플롯 갱신: t, y1, [y2] 순으로 호출
    drawPlot(ui->q1Plot,        data.t, data.q_bi[0]);
    drawPlot(ui->qmPlot,        data.t, data.q_bi[1]);
    drawPlot(ui->qbPlot,        data.t, data.q_bi[2]);

    drawPlot(ui->xposPlot,      data.t, data.pos_ref[0],  data.pos[0]);  // 2곡선(실제, 기준)
    drawPlot(ui->yposPlot,      data.t, data.pos_ref[1],  data.pos[1]);
    drawPlot(ui->zposPlot,      data.t, data.pos_ref[2],  data.pos[2]);

    drawPlot(ui->xposIctrlPlot, data.t, data.pos_I[0]);
    drawPlot(ui->yposIctrlPlot, data.t, data.pos_I[1]);
    drawPlot(ui->zposIctrlPlot, data.t, data.pos_I[2]);

    drawPlot(ui->q1CurrentPlot, data.t, data.motors[0].control);
    drawPlot(ui->qmCurrentPlot, data.t, data.motors[1].control);
    drawPlot(ui->qbCurrentPlot, data.t, data.motors[2].control);
    // drawPlot() 안에서 rpQueuedReplot과 rescale(true) 처리하므로 여기서 추가 replot 불필요
}


void MainWindow::setRobotLed(bool on) {
    if (!ui->robot_activation_led) return;
    ui->robot_activation_led->setProperty("on", on);

    // 스타일 재적용(변경 즉시 반영)
    ui->robot_activation_led->style()->unpolish(ui->robot_activation_led);
    ui->robot_activation_led->style()->polish(ui->robot_activation_led);
    ui->robot_activation_led->update();
}
void MainWindow::RobotLedOn()  { setRobotLed(true);  }
void MainWindow::RobotLedOff() { setRobotLed(false); }

void MainWindow::setq1Led(bool on) {
    if (!ui->q1_activation_led) return;
    ui->q1_activation_led->setProperty("on", on);

    // 스타일 재적용(변경 즉시 반영)
    ui->q1_activation_led->style()->unpolish(ui->q1_activation_led);
    ui->q1_activation_led->style()->polish(ui->q1_activation_led);
    ui->q1_activation_led->update();
}
void MainWindow::q1LedOn()  { setq1Led(true);  }
void MainWindow::q1LedOff() { setq1Led(false); }

void MainWindow::setqmLed(bool on) {
    if (!ui->qm_activation_led) return;
    ui->qm_activation_led->setProperty("on", on);

    // 스타일 재적용(변경 즉시 반영)
    ui->qm_activation_led->style()->unpolish(ui->qm_activation_led);
    ui->qm_activation_led->style()->polish(ui->qm_activation_led);
    ui->qm_activation_led->update();
}
void MainWindow::qmLedOn()  { setqmLed(true);  }
void MainWindow::qmLedOff() { setqmLed(false); }

void MainWindow::setqbLed(bool on) {
    if (!ui->qb_activation_led) return;
    ui->qb_activation_led->setProperty("on", on);

    // 스타일 재적용(변경 즉시 반영)
    ui->qb_activation_led->style()->unpolish(ui->qb_activation_led);
    ui->qb_activation_led->style()->polish(ui->qb_activation_led);
    ui->qb_activation_led->update();
}
void MainWindow::qbLedOn()  { setqbLed(true);  }
void MainWindow::qbLedOff() { setqbLed(false); }

static void ensureSecondGraph(QCustomPlot* plot)
{
    if (plot->graphCount() < 2) {
        plot->addGraph();                               // graph(1)
        plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
    }
}

void MainWindow::createPlot(QCustomPlot *plot)
{
    // ── 그래프 0 (기본)
    plot->addGraph();                                   // graph(0)
    plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));

    // x축 시간 포맷
    auto timeTicker = QSharedPointer<QCPAxisTickerDateTime>::create();
    timeTicker->setDateTimeFormat("mm:ss");
    plot->xAxis->setTicker(timeTicker);

    plot->axisRect()->setupFullAxesBox();
    connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));

    // 스타일
    plot->xAxis->setBasePen(QPen(Qt::white, 1));
    plot->yAxis->setBasePen(QPen(Qt::white, 1));
    plot->xAxis->setTickPen(QPen(Qt::white, 1));
    plot->yAxis->setTickPen(QPen(Qt::white, 1));
    plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
    plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
    plot->xAxis->setTickLabelColor(Qt::white);
    plot->yAxis->setTickLabelColor(Qt::white);
    plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    plot->xAxis->grid()->setSubGridVisible(true);
    plot->yAxis->grid()->setSubGridVisible(true);
    plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
    plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
    plot->setBackground(QColor(25, 35, 45));
    plot->axisRect()->setBackground(QColor(25, 35, 45));

    // (주의) insetLayout()->setInsetAlignment(0, ...)를 쓰려면
    // inset 0번이 실제로 존재해야 합니다(예: legend를 inset에 추가한 뒤).
    // 없다면 이 줄은 제거하세요.
    // plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
}

void MainWindow::drawPlot(QCustomPlot* plot, double t, double y1, std::optional<double> y2)
{
    // 1) 데이터 추가 (필수: 그래프 0)
    plot->graph(0)->addData(t, y1);

    // 2) 두 번째 y가 들어오면 그래프 1 자동 생성/표시
    if (y2.has_value()) {
        ensureSecondGraph(plot);
        plot->graph(1)->setVisible(true);
        plot->graph(1)->addData(t, *y2);
    } else {
        // 이번 샘플은 y1만 있는 상황 → 그래프 1은 숨겨 두면 rescale에 영향 X
        if (plot->graphCount() > 1)
            plot->graph(1)->setVisible(false);
    }

    // 3) x축 창 범위 갱신 (t를 오른쪽 끝으로 고정)
    plot->xAxis->setRange(t, Plot_time_window_POS, Qt::AlignRight);

    // 4) 오래된 데이터 삭제 (창 바깥은 제거)
    const double cutoff = t - Plot_time_window_POS;
    plot->graph(0)->data()->removeBefore(cutoff);
    if (plot->graphCount() > 1)
        plot->graph(1)->data()->removeBefore(cutoff);

    // 5) y축 자동 스케일 (가시 요소만 기준)
    plot->yAxis->rescale(true); // onlyVisiblePlottables=true

    // 6) 그리기
    plot->replot(QCustomPlot::rpQueuedReplot); // 즉시 그리기 대신 큐잉(부하↓)
}

void MainWindow::readUIParams()
{
    // UI 값 읽어서 전역 구조체에 저장
    QWriteLocker lock(&gTelemetryLock);   // 쓰기 락

    gTelemetry.target_position = {
        ui->TaskTime->value(),
        ui->targetXpos->value(),
        ui->targetYpos->value(),
        ui->targetZpos->value()
    };

    gTelemetry.posx_pid_gain = {
        ui->xposPctrlGain->value(),
        ui->xposIctrlGain->value(),
        ui->xposDctrlGain->value(),
        ui->xposCutoffFreq->value(),
        ui->xposWindupGain->value()
    };

    gTelemetry.posy_pid_gain = {
        ui->yposPctrlGain->value(),
        ui->yposIctrlGain->value(),
        ui->yposDctrlGain->value(),
        ui->yposCutoffFreq->value(),
        ui->yposWindupGain->value()
    };

    gTelemetry.posz_pid_gain = {
        ui->zposPctrlGain->value(),
        ui->zposIctrlGain->value(),
        ui->zposDctrlGain->value(),
        ui->zposCutoffFreq->value(),
        ui->zposWindupGain->value()
    };
}
