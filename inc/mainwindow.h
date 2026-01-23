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

private:
    Ui::MainWindow *ui;
    HansRobot robot;
    QVector<Craft> crafts;
    int lastPageIdx;
    int currCraftIdx;
    QElapsedTimer midPressDuration;
    StatusThread *m_statusThread;
};

//*********************************************************//
// 状态监控线程
class StatusThread : public QThread {
    Q_OBJECT
public:
    explicit StatusThread(HansRobot* robotPtr, QObject *parent = nullptr)
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
            // ========== 1. 机器人状态读取 ==========
            int moving = 0, enable = 0, error = 0, breaking = 0, pause = 0;
            int estop = 0, guard = 0, electrify = 0, connect = 0, blending = 0, inpos = 0;
            int errCode = 0, errAxis = 0;

            int nRet = HRIF_ReadRobotState(0, 0, moving, enable, error, errCode, errAxis,
                                           breaking, pause, estop, guard, electrify,
                                           connect, blending, inpos);
            if (nRet != 0) {
                connect = 0;
            }

            // ========== 2. AGP状态读取(使用GetAGP访问器) ==========
            bool agpConnect = false;
            bool agpEnable = false;
            bool agpError = false;
            int agpErrCode = 0;

            if (m_robot && m_robot->GetAGP()) {
                AGP* agpPtr = m_robot->GetAGP();

                // 检查AGP连接状态
                agpConnect = agpPtr->is_connected();

                if (agpConnect) {
                    // 读取状态寄存器21
                    int16_t status = agpPtr->ReadStatus();

                    if (status >= 0) {
                        // Bit0: 使能状态
                        agpEnable = (status & 0x0001) != 0;

                        // Bit4: 错误状态
                        agpError = (status & 0x0010) != 0;

                        // 如果有错误,读取错误代码(寄存器26)
                        if (agpError) {
                            int16_t err = agpPtr->ReadErr();
                            if (err >= 0) {
                                agpErrCode = err;
                            }
                        }
                    } else {
                        // 读取失败,可能连接中断
                        agpConnect = false;
                    }
                }
            }

            // ========== 3. IO状态读取 ==========
            int di1 = 0, di2 = 0, di3 = 0, di4 = 0, di5 = 0, di6 = 0, di7 = 0;
            int do5 = 0;

            // 只在机器人连接时读取IO
            if (connect != 0) {
                HRIF_ReadBoxDI(0, 1, di1);  // DI1 - 安全门
                HRIF_ReadBoxDI(0, 2, di2);  // DI2 - 刀库1号刀位
                HRIF_ReadBoxDI(0, 3, di3);  // DI3 - 刀库2号刀位
                HRIF_ReadBoxDI(0, 4, di4);  // DI4 - 刀库3号刀位
                HRIF_ReadBoxDI(0, 5, di5);  // DI5 - 刀库4号刀位
                HRIF_ReadBoxDI(0, 6, di6);  // DI6 - 刀库门打开
                HRIF_ReadBoxDI(0, 7, di7);  // DI7 - 刀库门关闭
                HRIF_ReadBoxDO(0, 5, do5);  // DO5 - AGP冷却气
            }

            // ========== 4. 发送信号到UI ==========
            emit statusUpdated(
                moving != 0, enable != 0, error != 0, errCode,
                electrify != 0, connect != 0,
                agpConnect, agpEnable, agpError, agpErrCode,
                di1 != 0, di2 != 0, di3 != 0, di4 != 0, di5 != 0,
                di6 != 0, di7 != 0, do5 != 0
                );

            QThread::msleep(100);
        }
    }

private:
    volatile bool m_stop;
    HansRobot* m_robot;
};

#endif // MAINWINDOW_H
