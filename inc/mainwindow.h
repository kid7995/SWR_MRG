#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QElapsedTimer>
#include <QMainWindow>
#include <QPushButton>
#include <QVector>
#include <QThread>
#include <QTextCodec>

#include "robot.h"


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE
class StatusThread;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void ReadCurrPara();
    void SavePara(int index);

    void SaveToolConfig();
    void ReadToolConfig();

    void DelCurrPara();

    void InitButtons();
    void EnableButtons();

    void SetValidator();
    void SetPolishWay(const PolishWay &way);
    void SetBackgroundColor(QPushButton *btn, const QColor &color);

    void ConnectRobot();
    void ConnectAGP();

    void AddHistoryPoint(const QString &strPoint);

    void InitStatusMonitor();

    // ── 新增：机器人类型枚举与工厂 ──────────────────────────────────────────
    // 整数值与 config.ini General/RobotType 保持一致
    enum class RobotType { Hans = 0, Duco = 1, Jaka = 2 };

    // 工厂函数：根据 type new 出对应子类，释放旧实例并迁移配置
    void CreateRobot(RobotType type);

private slots:
    void on_btnDrag_clicked();
    void on_btnSafe_clicked();
    void on_btnPrev_clicked();
    void on_btnNext_clicked();
    void on_btnBegin_clicked();
    void on_btnEnd_clicked();
    void on_btnTryRun_clicked();
    void on_btnRun_clicked();
    void on_btnSetting_clicked();
    void on_btnSettingReturn_clicked();
    void on_btnRobotConnect_clicked();
    void on_btnAGPConnect_clicked();
    void on_btnSaveCurrPara_clicked();
    void on_btnAddNewPara_clicked();
    void on_btnDelCurrPara_clicked();
    void on_btnStop_clicked();
    void on_btnAux_clicked();
    void on_btnMid_clicked();
    void on_btnMid_pressed();
    void on_btnBeginOffset_clicked();
    void on_btnEndOffset_clicked();
    void on_btnClear_clicked();
    void on_btnClearMid_clicked();
    void on_btnDelLastMid_clicked();
    void on_btnOpenWeb_clicked();
    void on_btnMoveToPoint_clicked();
    void on_btnClearHistory_clicked();
    void on_btnCoverPoint_clicked();
    void on_btnStop2_clicked();
    void on_btnInputPoint_clicked();

    void on_leCutinSpeed_editingFinished();
    void on_leMoveSpeed_editingFinished();
    void on_leRotateSpeed_editingFinished();
    void on_leContactForce_editingFinished();
    void on_leTransitionTime_editingFinished();
    void on_leSettingForce_editingFinished();
    void on_leTeachPos_editingFinished();
    void on_leDiscRadius_editingFinished();
    void on_leDiscThickness_editingFinished();
    void on_leGrindAngle_editingFinished();
    void on_leOffsetCount_editingFinished();
    void on_leAddOffsetCount_editingFinished();
    void on_leRaiseCount_editingFinished();
    void on_leFloatCount_editingFinished();
    void on_leTransitionRadius_editingFinished();

    void on_cmbCraftID_currentIndexChanged(int index);
    void on_cmbPolishMode_currentIndexChanged(int index);
    void on_cmbPolishWay_currentIndexChanged(int index);
    void on_cmbCraftID_editTextChanged(const QString &arg1);

    void on_chkMirror_stateChanged(int arg1);

    void updateRobotStatusUI(
        bool robotMoving, bool robotEnable, bool robotError, int robotErrCode,
        bool robotElectrify, bool robotConnect,
        bool agpConnect, bool agpEnable, bool agpError, int agpErrCode,
        bool di1_safeDoor, bool di2_pot1, bool di3_pot2, bool di4_pot3, bool di5_pot4,
        bool di6_magOpen, bool di7_magClose, bool do5_agpAir
        );

    void on_ToolMagazineReset_clicked();
    void on_RobotReconnect_2_clicked();


    void on_btnInputPoint_Vision_clicked();

    void on_btnMoveToPoint_Vision_clicked();

    void on_btnRunPoint_VIsion_clicked();

    void on_btnSetSpeed_Vison_clicked();

    void on_btnStop_Vision_clicked();

    void on_btnTryRun_Vision_clicked();

private:
    Ui::MainWindow *ui;

    // ── 原来：HansRobot robot;（值类型，无法多态）
    // ── 现在：Robot* 指针 + 品牌枚举，由 CreateRobot() 动态分配 ─────────────
    Robot*     robot            = nullptr;      // 多态机器人指针
    RobotType  currentRobotType = RobotType::Duco; // 当前品牌，与 config.ini 同步

    QVector<VisionMotionSegment> visionSegments; // Page6: Vision 导入的动作段列表

    QVector<Craft> crafts;
    int lastPageIdx;
    int currCraftIdx;
    QElapsedTimer midPressDuration;
    StatusThread *m_statusThread;
};

// ─────────────────────────────────────────────────────────────────────────────
// StatusThread：状态监控线程
//
// 变更说明：
//   原版 m_robot 为 HansRobot*，所有状态读取均走 HRIF API。
//   现版 m_robot 改为 Robot*（多态基类指针），通过以下策略兼容多品牌：
//
//   ① HansRobot：沿用 HRIF_ReadRobotState / HRIF_ReadBoxDI 等原有逻辑
//      （通过 dynamic_cast 判断，成功则走 Hans 分支）
//
//   ② DucoRobot / JakaRobot：调用虚函数接口（IsRobotEnabled / IsRobotMoved
//      / IsRobotElectrified），IO 状态（DI/DO）留空，待对应 SDK 接口接入后
//      在 else 分支中扩展。
//
//   AGP 状态读取不依赖品牌，保持不变。
// ─────────────────────────────────────────────────────────────────────────────
class StatusThread : public QThread {
    Q_OBJECT
public:
    // 构造时接受 Robot* 基类指针，兼容所有子类
    explicit StatusThread(Robot* robotPtr, QObject *parent = nullptr)
        : QThread(parent), m_stop(false), m_robot(robotPtr) {}

    void stop() { m_stop = true; }

signals:
    void statusUpdated(
        bool robotMoving, bool robotEnable, bool robotError, int robotErrCode,
        bool robotElectrify, bool robotConnect,
        bool agpConnect, bool agpEnable, bool agpError, int agpErrCode,
        bool di1_safeDoor, bool di2_pot1, bool di3_pot2, bool di4_pot3, bool di5_pot4,
        bool di6_magOpen, bool di7_magClose, bool do5_agpAir
        );

protected:
    void run() override {
        while (!m_stop) {

            // ── 状态变量初始化 ───────────────────────────────────────────────
            bool robotMoving   = false;
            bool robotEnable   = false;
            bool robotError    = false;
            bool robotElectrify = false;
            bool robotConnect  = false;
            int  robotErrCode  = 0;
            int  di1=0, di2=0, di3=0, di4=0, di5=0, di6=0, di7=0, do5=0;

            // ── ① Hans 分支：HRIF API 精确读取 ──────────────────────────────
            HansRobot* hansRobot = dynamic_cast<HansRobot*>(m_robot);
            if (hansRobot != nullptr) {
                int moving=0, enable=0, error=0, errCode=0, errAxis=0;
                int breaking=0, pause=0, estop=0, guard=0;
                int electrify=0, connect=0, blending=0, inpos=0;

                int nRet = HRIF_ReadRobotState(0, 0, moving, enable, error,
                                               errCode, errAxis, breaking, pause,
                                               estop, guard, electrify,
                                               connect, blending, inpos);
                if (nRet != 0) connect = 0;

                robotMoving    = moving    != 0;
                robotEnable    = enable    != 0;
                robotError     = error     != 0;
                robotErrCode   = errCode;
                robotElectrify = electrify != 0;
                robotConnect   = connect   != 0;

                // Hans IO 状态（仅在连接时读取）
                if (robotConnect) {
                    HRIF_ReadBoxDI(0, 0, di1);  // DI0 安全门
                    HRIF_ReadBoxDI(0, 2, di2);  // DI2 刀库1号位
                    HRIF_ReadBoxDI(0, 3, di3);  // DI3 刀库2号位
                    HRIF_ReadBoxDI(0, 4, di4);  // DI4 刀库3号位
                    HRIF_ReadBoxDI(0, 5, di5);  // DI5 刀库4号位
                    HRIF_ReadBoxDI(0, 6, di6);  // DI6 刀库门打开
                    HRIF_ReadBoxDI(0, 7, di7);  // DI7 刀库门关闭
                    HRIF_ReadBoxDO(0, 5, do5);  // DO5 AGP冷却气
                }
            }
            // ── ② Duco / Jaka 分支：调用虚函数接口 ──────────────────────────
            else if (m_robot != nullptr) {
                // 调用虚函数时需防止未连接状态下崩溃，用 try-catch 包裹
                try {
                    robotElectrify = m_robot->IsRobotElectrified();
                    robotEnable    = m_robot->IsRobotEnabled();
                    robotMoving    = m_robot->IsRobotMoved();
                    robotConnect   = robotElectrify; // 能读到状态即视为连接
                    // robotError / DI / DO：待对应品牌 SDK IO 接口接入后在此扩展
                } catch (...) {
                    // 通信异常（机器人未连接等）：保持默认 false
                    robotConnect = false;
                }
            }

            // ── AGP 状态（与品牌无关）───────────────────────────────────────
            bool agpConnect = false, agpEnable = false, agpError = false;
            int  agpErrCode = 0;

            if (m_robot != nullptr && m_robot->GetAGP() != nullptr) {
                AGP* agpPtr = m_robot->GetAGP();
                agpConnect  = agpPtr->is_connected();
                if (agpConnect) {
                    int16_t status = agpPtr->ReadStatus();
                    if (status >= 0) {
                        agpEnable = (status & 0x0001) != 0;
                        agpError  = (status & 0x0010) != 0;
                        if (agpError) {
                            int16_t err = agpPtr->ReadErr();
                            if (err >= 0) agpErrCode = err;
                        }
                    } else {
                        agpConnect = false;
                    }
                }
            }

            // ── 发送 UI 更新信号 ─────────────────────────────────────────────
            emit statusUpdated(
                robotMoving, robotEnable, robotError, robotErrCode,
                robotElectrify, robotConnect,
                agpConnect, agpEnable, agpError, agpErrCode,
                di1!=0, di2!=0, di3!=0, di4!=0, di5!=0,
                di6!=0, di7!=0, do5!=0
                );

            QThread::msleep(100);
        }
    }

private:
    volatile bool m_stop;
    Robot*        m_robot; // 多态基类指针，支持 Hans/Duco/Jaka
};

#endif // MAINWINDOW_H
