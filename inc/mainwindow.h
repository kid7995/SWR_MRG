#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QElapsedTimer>
#include <QMainWindow>
#include <QVector>

#include "robot.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

// #pragma execution_character_set("utf-8")
class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

  private:
    void ReadCurrPara();
    void SavePara(int index);
    void DelCurrPara();

    void EnableButtons();
    void SetValidator();
    void SetPolishWay(const PolishWay &way);

    void ConnectRobot();
    void ConnectAGP();

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
    void on_btnStop2_clicked();

    void on_leCutinSpeed_editingFinished();
    void on_leMoveSpeed_editingFinished();
    void on_leRotateSpeed_editingFinished();
    void on_leContactForce_editingFinished();
    void on_leTransitionTime_editingFinished();
    void on_leSettingForce_editingFinished();
    void on_leTeachPos_editingFinished();
    void on_leDiscRadius_editingFinished();
    void on_leGrindAngle_editingFinished();
    void on_leOffsetCount_editingFinished();
    void on_leAddOffsetCount_editingFinished();

    void on_cmbCraftID_currentIndexChanged(int index);
    void on_cmbPolishMode_currentIndexChanged(int index);
    void on_cmbPolishWay_currentIndexChanged(int index);
    void on_cmbCraftID_editTextChanged(const QString &arg1);

  private:
    Ui::MainWindow *ui;
    HansRobot robot;       // 大族机器人
    // DucoRobot robot;       // 新松机器人
    // JakaRobot robot;                // 节卡机器人
    QVector<Craft> crafts;          // 工艺参数列表
    int lastPageIdx;                // 上一个页面编号
    int currCraftIdx;               // 当前工艺参数编号
    QElapsedTimer midPressDuration; // 中间点长按时间间隔，单位：ms
};

#endif // MAINWINDOW_H
