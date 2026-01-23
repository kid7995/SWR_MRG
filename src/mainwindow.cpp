#include <QDebug>
#include <QFile>
#include <QMessageBox>
#include <QSettings>
#include <QTextStream>
#include <QThread>
#include <QFileDialog>

#include "mainwindow.h"
#include "ui_mainwindow.h"

const QColor defaultColor(173, 49, 34);
const QColor greenColor("green");
const QColor goldColor("gold");
const QColor greyColor("grey");

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), lastPageIdx(0),
    currCraftIdx(0) {
    ui->setupUi(this);
    InitButtons();

    QString fileName = QCoreApplication::applicationDirPath();
    fileName += "/config.ini";
    if (!QFile::exists(fileName)) {
        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            out.setCodec("UTF-8");
            out << QString(
                "[CraftParameter]\nsize=1\n1\\CraftName=\\x4e00\n1\\PolishMode="
                "0\n1\\PolishWay=0\n1\\TeachingPointReferencePosition="
                "7\n1\\CutinSpeed=20\n1\\MovingSpeed=80\n1\\RotationSpeed="
                "4500\n1\\ContactForce=10\n1\\SettingForce="
                "80\n1\\TransitionTime=1500\n1\\TransitionRadius="
                "0\n1\\DiscRadius=50\n1\\DiscThickness=8\n1\\GrindAngle="
                "0\n1\\OffsetCount=0\n1\\AddOffsetCount=0\n1\\RaiseCount="
                "0\n1\\FloatCount=0\n1\\IsMirror=false\n"
                "1\\IsMirror=false\n"
                "1\\TotalPolishCount=0\n"
                "1\\TotalPolishLength=0\n"
                "1\\toolChangeThreshold=30000\n"
                "1\\StartToolChange=false\n"
                "1\\ToolChangeDone=false\n"
                "1\\ToolMagStatus=0\n"
                "1\\TargetToolPos=0\n"
                );
            file.close();
        } else {
            setEnabled(false);
            QMessageBox::critical(NULL, "提示", "无法创建工艺参数文件");
            return;
        }
    }
    QSettings settings(fileName, QSettings::IniFormat);
    settings.setIniCodec("UTF-8");
    int size = settings.beginReadArray("CraftParameter");
    if (size == 0) {
        return;
    }
    for (int i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        Craft craft;
        craft.craftID = settings.value("CraftName").toString();
        craft.mode = (PolishMode)settings.value("PolishMode").toInt();
        craft.way = (PolishWay)settings.value("PolishWay").toInt();
        craft.teachPointReferPos =
            settings.value("TeachingPointReferencePosition").toInt();
        craft.cutinSpeed = settings.value("CutinSpeed").toInt();
        craft.moveSpeed = settings.value("MovingSpeed").toInt();
        craft.rotateSpeed = settings.value("RotationSpeed").toInt();
        craft.contactForce = settings.value("ContactForce").toInt();
        craft.settingForce = settings.value("SettingForce").toInt();
        craft.transitionTime = settings.value("TransitionTime").toInt();
        craft.transitionRadius = settings.value("TransitionRadius").toInt();
        craft.discRadius = settings.value("DiscRadius").toInt();
        craft.discThickness = settings.value("DiscThickness").toInt();
        craft.grindAngle = settings.value("GrindAngle").toInt();
        craft.offsetCount = settings.value("OffsetCount").toInt();
        craft.addOffsetCount = settings.value("AddOffsetCount").toInt();
        craft.raiseCount = settings.value("RaiseCount").toInt();
        craft.floatCount = settings.value("FloatCount").toInt();
        craft.isMirror = settings.value("IsMirror").toBool();
        //*********************************************************************//
        //新增换刀逻辑
        craft.totalPolishCount = settings.value("TotalPolishCount", 0).toInt();
        craft.totalPolishLength = settings.value("TotalPolishLength", 0.0).toDouble();
        craft.startToolChange = settings.value("StartToolChange", false).toBool();
        craft.toolChangeDone = settings.value("ToolChangeDone", false).toBool();
        craft.toolMagStatus = settings.value("ToolMagStatus", 0).toInt();
        craft.targetToolPos = settings.value("TargetToolPos", 1).toInt();
        //*********************************************************************//
        crafts.append(craft);
    }
    settings.endArray();
    for (int i = 0; i < size; ++i) {
        ui->cmbCraftID->addItem(crafts.at(i).craftID);
    }
    SetPolishWay(crafts.at(0).way);
    SetValidator();
    ConnectRobot();
    ConnectAGP();
    InitStatusMonitor();
}

MainWindow::~MainWindow() {
    // 停止状态监控线程
    if (m_statusThread && m_statusThread->isRunning()) {
        m_statusThread->stop();
        m_statusThread->wait(100);
        if (m_statusThread->isRunning()) {
            m_statusThread->terminate();
            m_statusThread->wait();
        }
    }

    delete ui;
}

void MainWindow::SavePara(int index) {
    QString fileName = QCoreApplication::applicationDirPath();
    fileName += "/config.ini";
    QSettings settings(fileName, QSettings::IniFormat);

    // 设置编码，防止中文工艺名乱码（建议加上）
    settings.setIniCodec("UTF-8");

    settings.beginWriteArray("CraftParameter");

    // 统一使用传入的 index
    settings.setArrayIndex(index);

    // 1. 原有参数保存
    settings.setValue("CraftName", crafts.at(index).craftID);
    settings.setValue("PolishMode", crafts.at(index).mode);
    settings.setValue("PolishWay", crafts.at(index).way);
    settings.setValue("TeachingPointReferencePosition", crafts.at(index).teachPointReferPos);
    settings.setValue("CutinSpeed", crafts.at(index).cutinSpeed);
    settings.setValue("MovingSpeed", crafts.at(index).moveSpeed);
    settings.setValue("RotationSpeed", crafts.at(index).rotateSpeed);
    settings.setValue("ContactForce", crafts.at(index).contactForce);
    settings.setValue("SettingForce", crafts.at(index).settingForce);
    settings.setValue("TransitionTime", crafts.at(index).transitionTime);
    settings.setValue("TransitionRadius", crafts.at(index).transitionRadius);
    settings.setValue("DiscRadius", crafts.at(index).discRadius);
    settings.setValue("DiscThickness", crafts.at(index).discThickness);
    settings.setValue("GrindAngle", crafts.at(index).grindAngle);
    settings.setValue("OffsetCount", crafts.at(index).offsetCount);
    settings.setValue("AddOffsetCount", crafts.at(index).addOffsetCount);
    settings.setValue("RaiseCount", crafts.at(index).raiseCount);
    settings.setValue("FloatCount", crafts.at(index).floatCount);
    settings.setValue("IsMirror", crafts.at(index).isMirror);

    // 2. 新增换刀逻辑 (依然在同一个 index 下)
    settings.setValue("TotalPolishCount", crafts.at(index).totalPolishCount);
    settings.setValue("TotalPolishLength", crafts.at(index).totalPolishLength);
    settings.setValue("toolChangeThreshold", crafts.at(index).toolChangeThreshold);
    settings.setValue("StartToolChange", crafts.at(index).startToolChange);
    settings.setValue("ToolChangeDone", crafts.at(index).toolChangeDone);
    settings.setValue("ToolMagStatus", crafts.at(index).toolMagStatus);
    settings.setValue("TargetToolPos", crafts.at(index).targetToolPos);

    settings.endArray();
}

void MainWindow::ReadCurrPara() {
    ui->cmbPolishMode->setCurrentIndex(crafts.at(currCraftIdx).mode);
    ui->cmbPolishWay->setCurrentIndex(crafts.at(currCraftIdx).way);
    ui->leTeachPos->setText(
        QString::number(crafts.at(currCraftIdx).teachPointReferPos));
    ui->leCutinSpeed->setText(
        QString::number(crafts.at(currCraftIdx).cutinSpeed));
    ui->leMoveSpeed->setText(
        QString::number(crafts.at(currCraftIdx).moveSpeed));
    ui->leRotateSpeed->setText(
        QString::number(crafts.at(currCraftIdx).rotateSpeed));
    ui->leContactForce->setText(
        QString::number(crafts.at(currCraftIdx).contactForce));
    ui->leSettingForce->setText(
        QString::number(crafts.at(currCraftIdx).settingForce));
    ui->leTransitionTime->setText(
        QString::number(crafts.at(currCraftIdx).transitionTime));
    ui->leTransitionRadius->setText(
        QString::number(crafts.at(currCraftIdx).transitionRadius));
    ui->leDiscRadius->setText(
        QString::number(crafts.at(currCraftIdx).discRadius));
    ui->leDiscThickness->setText(
        QString::number(crafts.at(currCraftIdx).discThickness));
    ui->leGrindAngle->setText(
        QString::number(crafts.at(currCraftIdx).grindAngle));
    ui->leOffsetCount->setText(
        QString::number(crafts.at(currCraftIdx).offsetCount));
    ui->leAddOffsetCount->setText(
        QString::number(crafts.at(currCraftIdx).addOffsetCount));
    ui->leRaiseCount->setText(
        QString::number(crafts.at(currCraftIdx).raiseCount));
    ui->leFloatCount->setText(
        QString::number(crafts.at(currCraftIdx).floatCount));
    ui->chkMirror->setCheckState(
        crafts.at(currCraftIdx).isMirror ? Qt::Checked : Qt::Unchecked);

    robot.teachPos = crafts.at(currCraftIdx).teachPointReferPos;
    robot.discThickness = crafts.at(currCraftIdx).discThickness;
}

void MainWindow::DelCurrPara() {
    crafts.remove(currCraftIdx);
    QString fileName = QCoreApplication::applicationDirPath();
    fileName += "/config.ini";
    QSettings settings(fileName, QSettings::IniFormat);
    settings.beginWriteArray("CraftParameter");
    settings.clear();
    settings.endArray();
    int size = crafts.size();
    for (int i = 0; i < size; ++i) {
        SavePara(i);
    }
}

void MainWindow::InitButtons() {
    SetBackgroundColor(ui->btnDrag, greyColor);
    SetBackgroundColor(ui->btnClear, greyColor);
    SetBackgroundColor(ui->btnClearMid, greyColor);
    SetBackgroundColor(ui->btnDelLastMid, greyColor);
    SetBackgroundColor(ui->btnSafe, greyColor);
    SetBackgroundColor(ui->btnBegin, greyColor);
    SetBackgroundColor(ui->btnEnd, greyColor);
    SetBackgroundColor(ui->btnMid, greyColor);
    SetBackgroundColor(ui->btnAux, greyColor);
    SetBackgroundColor(ui->btnBeginOffset, greyColor);
    SetBackgroundColor(ui->btnEndOffset, greyColor);
    SetBackgroundColor(ui->btnStop, greyColor);
    SetBackgroundColor(ui->btnTryRun, greyColor);
    SetBackgroundColor(ui->btnRun, greyColor);
    SetBackgroundColor(ui->btnMoveToPoint, greyColor);
    SetBackgroundColor(ui->btnClearHistory, greyColor);
    SetBackgroundColor(ui->btnCoverPoint, greyColor);
    SetBackgroundColor(ui->btnStop2, greyColor);

    ui->btnClearHistory->setVisible(true);
    SetBackgroundColor(ui->btnClearHistory, greyColor);
    ui->btnCoverPoint->setVisible(true);
    SetBackgroundColor(ui->btnCoverPoint, greyColor);
    ui->btnMoveToPoint->setVisible(true);
    SetBackgroundColor(ui->btnMoveToPoint, greyColor);
    ui->btnStop2->setVisible(true);
    SetBackgroundColor(ui->btnStop2, greyColor);

    ui->btnRobotConnect->setVisible(true);
    SetBackgroundColor(ui->btnRobotConnect, defaultColor);
    ui->btnAGPConnect->setVisible(true);
    SetBackgroundColor(ui->btnAGPConnect, defaultColor);

    ui->btnInputPoint->setEnabled(true);
    SetBackgroundColor(ui->btnInputPoint, defaultColor);
    ui->ToolMagazineReset->setEnabled(true);
    SetBackgroundColor(ui->ToolMagazineReset, defaultColor);
    // connect(ui->btnInputPoint, &QPushButton::clicked, this, &MainWindow::on_btnInputPoint_clicked);
}

void MainWindow::EnableButtons() {
    ui->btnDrag->setEnabled(true);
    SetBackgroundColor(ui->btnDrag, defaultColor);
    ui->btnClear->setEnabled(true);
    SetBackgroundColor(ui->btnClear, defaultColor);
    ui->btnClearMid->setEnabled(true);
    SetBackgroundColor(ui->btnClearMid, defaultColor);
    ui->btnDelLastMid->setEnabled(true);
    SetBackgroundColor(ui->btnDelLastMid, defaultColor);
    ui->btnSafe->setEnabled(true);
    SetBackgroundColor(ui->btnSafe, defaultColor);
    ui->btnBegin->setEnabled(true);
    SetBackgroundColor(ui->btnBegin, defaultColor);
    ui->btnEnd->setEnabled(true);
    SetBackgroundColor(ui->btnEnd, defaultColor);
    ui->btnMid->setEnabled(true);
    SetBackgroundColor(ui->btnMid, defaultColor);
    ui->btnAux->setEnabled(true);
    SetBackgroundColor(ui->btnAux, defaultColor);
    ui->btnBeginOffset->setEnabled(true);
    SetBackgroundColor(ui->btnBeginOffset, defaultColor);
    ui->btnEndOffset->setEnabled(true);
    SetBackgroundColor(ui->btnEndOffset, defaultColor);
    ui->btnStop->setEnabled(true);
    SetBackgroundColor(ui->btnStop, defaultColor);
    ui->btnTryRun->setEnabled(true);
    SetBackgroundColor(ui->btnTryRun, defaultColor);
    ui->btnRun->setEnabled(true);
    SetBackgroundColor(ui->btnRun, defaultColor);
    ui->btnMoveToPoint->setEnabled(true);
    SetBackgroundColor(ui->btnMoveToPoint, defaultColor);
    ui->btnClearHistory->setEnabled(true);
    SetBackgroundColor(ui->btnClearHistory, defaultColor);
    ui->btnCoverPoint->setEnabled(true);
    SetBackgroundColor(ui->btnCoverPoint, defaultColor);
    ui->btnStop2->setEnabled(true);
    SetBackgroundColor(ui->btnStop2, defaultColor);
}

void MainWindow::SetValidator() {
    ui->leCutinSpeed->setValidator(new QIntValidator(ui->leCutinSpeed));
    ui->leMoveSpeed->setValidator(new QIntValidator(ui->leMoveSpeed));
    ui->leRotateSpeed->setValidator(new QIntValidator(ui->leRotateSpeed));
    ui->leContactForce->setValidator(new QIntValidator(ui->leContactForce));
    ui->leTransitionTime->setValidator(new QIntValidator(ui->leTransitionTime));
    ui->leTransitionRadius->setValidator(
        new QIntValidator(ui->leTransitionRadius));
    ui->leSettingForce->setValidator(new QIntValidator(ui->leSettingForce));
    ui->leTeachPos->setValidator(new QIntValidator(ui->leTeachPos));
    ui->leDiscRadius->setValidator(new QIntValidator(ui->leDiscRadius));
    ui->leDiscThickness->setValidator(new QIntValidator(ui->leDiscThickness));
    ui->leGrindAngle->setValidator(new QIntValidator(ui->leGrindAngle));
    ui->leOffsetCount->setValidator(new QIntValidator(ui->leOffsetCount));
    ui->leAddOffsetCount->setValidator(new QIntValidator(ui->leAddOffsetCount));
    ui->leRaiseCount->setValidator(new QIntValidator(ui->leRaiseCount));
    ui->leFloatCount->setValidator(new QIntValidator(ui->leFloatCount));
}

void MainWindow::SetPolishWay(const PolishWay &way) {
    switch (way) {
    case PolishWay::ArcWay:
        ui->leOffsetCount->setEnabled(false);
        ui->lblBackground->setPixmap(QPixmap(":/pic/arc.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 110);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(false);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::LineWay:
        ui->leOffsetCount->setEnabled(false);
        ui->lblBackground->setPixmap(QPixmap(":/pic/line.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 310);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(false);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::RegionArcWay1:
    case PolishWay::RegionArcWay2:
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/region_arc.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 110);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(true);
        ui->btnEndOffset->setVisible(true);
        break;
    case PolishWay::RegionArcWay_Horizontal:
    case PolishWay::RegionArcWay_Vertical:
    case PolishWay::RegionArcWay_Vertical_Repeat:
    case PolishWay::ConicalFrustum_Concave:
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/region_arc.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 110);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(true);
        ui->btnEndOffset->setVisible(true);
        break;
    case PolishWay::CylinderWay_Horizontal_Convex:
    case PolishWay::CylinderWay_Vertical_Convex:
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/cylinder_convex.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 240);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(true);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::CylinderWay_Horizontal_Concave:
    case PolishWay::CylinderWay_Vertical_Concave:
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/cylinder_concave.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 180);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(true);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::ZLineWay:
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/z_line.png"));
        ui->btnAux->move(150, 130);
        ui->btnAux->setVisible(true);
        ui->btnMid->setVisible(false);
        ui->btnBeginOffset->setVisible(false);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::SpiralLineWay:
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/spiral_line.png"));
        ui->btnAux->move(350, 130);
        ui->btnAux->setVisible(true);
        ui->btnMid->setVisible(false);
        ui->btnBeginOffset->setVisible(false);
        ui->btnEndOffset->setVisible(false);
        break;
    default:
        break;
    }

    switch (way) {
    case PolishWay::RegionArcWay1:
    case PolishWay::RegionArcWay2:
        ui->leDiscRadius->setEnabled(false);
        ui->leDiscThickness->setEnabled(false);
        ui->leGrindAngle->setEnabled(false);
        break;
    default:
        ui->leDiscRadius->setEnabled(true);
        ui->leDiscThickness->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
        break;
    }

    switch (way) {
    case PolishWay::RegionArcWay1:
        ui->leAddOffsetCount->setEnabled(true);
        break;
    default:
        ui->leAddOffsetCount->setEnabled(false);
        break;
    }

    switch (way) {
    case PolishWay::RegionArcWay_Vertical:
        ui->leRaiseCount->setEnabled(true);
        ui->leFloatCount->setEnabled(true);
        break;
    default:
        ui->leRaiseCount->setEnabled(false);
        ui->leFloatCount->setEnabled(false);
        break;
    }

    switch (way) {
    case PolishWay::CylinderWay_Horizontal_Convex:
    case PolishWay::CylinderWay_Vertical_Convex:
    case PolishWay::CylinderWay_Horizontal_Concave:
    case PolishWay::CylinderWay_Vertical_Concave:
        ui->chkMirror->setVisible(true);
        if (crafts.at(currCraftIdx).isMirror) {
            ui->btnBegin->move(930, 210);
            ui->btnEnd->move(150, 210);
            ui->btnBeginOffset->move(930, 360);
        } else {
            ui->btnBegin->move(150, 210);
            ui->btnEnd->move(930, 210);
            ui->btnBeginOffset->move(150, 360);
        }
        break;
    default:
        ui->chkMirror->setVisible(false);
        ui->btnBegin->move(150, 310);
        ui->btnEnd->move(930, 310);
        ui->btnBeginOffset->move(320, 430);
        break;
    }
}

void MainWindow::SetBackgroundColor(QPushButton *btn, const QColor &color) {
    // 1. 禁用 AutoFillBackground，它是导致矩形底色的罪魁祸首
    btn->setAutoFillBackground(false);

    // 2. 动态构建样式表字符串，继承您已有的圆角和尺寸设置
    // 使用 color.name() 将 QColor 转换为 #RRGGBB 格式
    QString style = QString(
                        "QPushButton {"
                        "    background-color: %1;"
                        "    min-height: 60px;"
                        "    max-height: 60px;"
                        "    padding-left: 30px;"
                        "    padding-right: 30px;"
                        "    border-radius: 20px;"
                        "    color: white;"
                        "    font-size: 24px;"
                        "    border: none;"
                        "}"
                        ).arg(color.name());

    btn->setStyleSheet(style);
}

void MainWindow::ConnectRobot() {
    ui->btnRobotConnect->setEnabled(false);
    SetBackgroundColor(ui->btnRobotConnect, goldColor);
    ui->btnRobotConnect->setText("连接中");

    std::thread t([this] {
        bool connected = robot.RobotConnect(ui->leRobotIP->text());
        QMetaObject::invokeMethod(this, [this, connected] {
            if (connected) {
                EnableButtons();
                SetBackgroundColor(ui->btnRobotConnect, greenColor);
                ui->btnRobotConnect->setText("已连接");
            } else {
                ui->btnRobotConnect->setEnabled(true);
                SetBackgroundColor(ui->btnRobotConnect, defaultColor);
                ui->btnRobotConnect->setText("连接");
            }
        });
    });
    t.detach();
}

void MainWindow::ConnectAGP() {
    ui->btnAGPConnect->setEnabled(false);
    SetBackgroundColor(ui->btnAGPConnect, goldColor);
    ui->btnAGPConnect->setText("连接中");

    std::thread t([this] {
        bool connected = robot.AGPConnect(ui->leAGPIP->text());
        QMetaObject::invokeMethod(this, [this, connected] {
            if (connected) {
                SetBackgroundColor(ui->btnAGPConnect, greenColor);
                ui->btnAGPConnect->setText("已连接");
            } else {
                ui->btnAGPConnect->setEnabled(true);
                SetBackgroundColor(ui->btnAGPConnect, defaultColor);
                ui->btnAGPConnect->setText("连接");
            }
        });
    });
    t.detach();
}

void MainWindow::AddHistoryPoint(const QString &strPoint) {
    if (!strPoint.contains("：")) {
        return;
    }
    QList<QListWidgetItem *> pItems = ui->lstHistoryPoint->findItems(
        strPoint.left(strPoint.indexOf("：")), Qt::MatchStartsWith);
    if (pItems.isEmpty()) {
        ui->lstHistoryPoint->addItem(strPoint);
    } else {
        for (const auto &pItem : pItems) {
            pItem->setText(strPoint);
        }
    }
    QListWidgetItem *pCurrItem = ui->lstHistoryPoint->currentItem();
    if (pCurrItem != nullptr) {
        ui->lstHistoryPoint->currentTextChanged(pCurrItem->text());
    }
}

void MainWindow::on_btnDrag_clicked() {
    if (robot.RobotTeach(crafts.at(currCraftIdx).teachPointReferPos)) {
        SetBackgroundColor(ui->btnDrag, greenColor);
    } else {
        SetBackgroundColor(ui->btnDrag, defaultColor);
    }
}

void MainWindow::on_btnSafe_clicked() {
    QString strPoint = "";
    if (robot.GetSafePoint(strPoint)) {
        AddHistoryPoint(strPoint);
        SetBackgroundColor(ui->btnSafe, greenColor);
    } else {
        SetBackgroundColor(ui->btnSafe, defaultColor);
    }
}

void MainWindow::on_btnPrev_clicked() {
    // int size = ui->stackedWidget->count() - 1;
    int size = ui->stackedWidget->count();
    int i = ui->stackedWidget->currentIndex();
    ui->stackedWidget->setCurrentIndex((i - 1 + size) % size);
    ui->lblPageName->setText(
        ui->stackedWidget->currentWidget()->accessibleName());
}

void MainWindow::on_btnNext_clicked() {
    // int size = ui->stackedWidget->count() - 1;
    int size = ui->stackedWidget->count();
    int i = ui->stackedWidget->currentIndex();
    ui->stackedWidget->setCurrentIndex((i + 1) % size);
    ui->lblPageName->setText(
        ui->stackedWidget->currentWidget()->accessibleName());
}

void MainWindow::on_btnBegin_clicked() {
    QString strPoint = "";
    if (robot.GetBeginPoint(strPoint)) {
        AddHistoryPoint(strPoint);
        SetBackgroundColor(ui->btnBegin, greenColor);
    } else {
        SetBackgroundColor(ui->btnBegin, defaultColor);
    }
}

void MainWindow::on_btnEnd_clicked() {
    QString strPoint = "";
    if (robot.GetEndPoint(strPoint)) {
        AddHistoryPoint(strPoint);
        SetBackgroundColor(ui->btnEnd, greenColor);
    } else {
        SetBackgroundColor(ui->btnEnd, defaultColor);
    }
}

void MainWindow::on_btnTryRun_clicked() {
    if (!robot.CheckAllPoints(crafts.at(currCraftIdx).way)) {
        return;
    }
    ui->btnRun->setEnabled(false);
    ui->btnTryRun->setEnabled(false);
    ui->btnMoveToPoint->setEnabled(false);
    SetBackgroundColor(ui->btnTryRun, greenColor);
    ui->btnTryRun->setText("试运行中");
    if (robot.CloseFreeDriver()) {
        SetBackgroundColor(ui->btnDrag, defaultColor);
    }
    std::thread t([this] {
        robot.Run(crafts[currCraftIdx], false);
        QMetaObject::invokeMethod(this, [this] {
            ui->btnRun->setEnabled(true);
            ui->btnTryRun->setEnabled(true);
            ui->btnMoveToPoint->setEnabled(true);
            SetBackgroundColor(ui->btnTryRun, defaultColor);
            ui->btnTryRun->setText("试运行");
        });
    });
    t.detach();
}

void MainWindow::on_btnRun_clicked() {
    if (!robot.CheckAllPoints(crafts.at(currCraftIdx).way)) {
        return;
    }
    ui->btnRun->setEnabled(false);
    ui->btnTryRun->setEnabled(false);
    ui->btnMoveToPoint->setEnabled(false);
    SetBackgroundColor(ui->btnRun, greenColor);
    ui->btnRun->setText("运行中");
    if (robot.CloseFreeDriver()) {
        SetBackgroundColor(ui->btnDrag, defaultColor);
    }
    std::thread t([this] {
        robot.Run(crafts[currCraftIdx], true);
        robot.AGPStop();
        QMetaObject::invokeMethod(this, [this] {
            ui->btnRun->setEnabled(true);
            ui->btnTryRun->setEnabled(true);
            ui->btnMoveToPoint->setEnabled(true);
            SetBackgroundColor(ui->btnRun, defaultColor);
            ui->btnRun->setText("运行");
        });
    });
    t.detach();
}

void MainWindow::on_btnSetting_clicked() {
    int size = ui->stackedWidget->count() - 1;
    lastPageIdx = ui->stackedWidget->currentIndex();
    ui->stackedWidget->setCurrentIndex(size);
}

void MainWindow::on_btnSettingReturn_clicked() {
    ui->stackedWidget->setCurrentIndex(lastPageIdx);
}

void MainWindow::on_btnRobotConnect_clicked() { ConnectRobot(); }

void MainWindow::on_btnAGPConnect_clicked() { ConnectAGP(); }

void MainWindow::on_btnSaveCurrPara_clicked() {
    crafts[currCraftIdx].craftID = ui->cmbCraftID->currentText();
    SavePara(currCraftIdx);
    QMessageBox::information(NULL, "提示", "当前参数已保存");
}

void MainWindow::on_cmbCraftID_currentIndexChanged(int index) {
    currCraftIdx = index;
    ReadCurrPara();
}

void MainWindow::on_cmbPolishMode_currentIndexChanged(int index) {
    crafts[currCraftIdx].mode = (PolishMode)index;
}

void MainWindow::on_leCutinSpeed_editingFinished() {
    QString str = ui->leCutinSpeed->text();
    int pos = 0;
    QIntValidator v(0, 50);
    if (v.validate(str, pos) != QValidator::Acceptable) {
        QMessageBox::warning(
            NULL, "提示",
            QString("切入速度范围：%1~%2").arg(v.bottom()).arg(v.top()));
        ui->leCutinSpeed->setText(
            QString::number(crafts.at(currCraftIdx).cutinSpeed));
    } else {
        crafts[currCraftIdx].cutinSpeed = ui->leCutinSpeed->text().toInt();
    }
}

void MainWindow::on_leMoveSpeed_editingFinished() {
    QString str = ui->leMoveSpeed->text();
    int pos = 0;
    QIntValidator v(0, 350);
    if (v.validate(str, pos) != QValidator::Acceptable) {
        QMessageBox::warning(
            NULL, "提示",
            QString("行进速度范围：%1~%2").arg(v.bottom()).arg(v.top()));
        ui->leMoveSpeed->setText(
            QString::number(crafts.at(currCraftIdx).moveSpeed));
    } else {
        crafts[currCraftIdx].moveSpeed = ui->leMoveSpeed->text().toInt();
    }
}

void MainWindow::on_leRotateSpeed_editingFinished() {
    crafts[currCraftIdx].rotateSpeed = ui->leRotateSpeed->text().toInt();
}

void MainWindow::on_leContactForce_editingFinished() {
    crafts[currCraftIdx].contactForce = ui->leContactForce->text().toInt();
}

void MainWindow::on_leTransitionTime_editingFinished() {
    crafts[currCraftIdx].transitionTime = ui->leTransitionTime->text().toInt();
}

void MainWindow::on_leSettingForce_editingFinished() {
    crafts[currCraftIdx].settingForce = ui->leSettingForce->text().toInt();
}

void MainWindow::on_cmbPolishWay_currentIndexChanged(int index) {
    crafts[currCraftIdx].way = (PolishWay)index;
    SetPolishWay((PolishWay)index);
}

void MainWindow::on_leTeachPos_editingFinished() {
    crafts[currCraftIdx].teachPointReferPos = ui->leTeachPos->text().toInt();
    robot.teachPos = ui->leTeachPos->text().toInt();
}

void MainWindow::on_btnAddNewPara_clicked() {
    crafts.append(crafts.at(currCraftIdx));
    int size = crafts.size();
    crafts[size - 1].craftID = "";
    SavePara(size - 1);
    ui->cmbCraftID->addItem("");
    ui->cmbCraftID->setCurrentIndex(size - 1);
    ui->cmbCraftID->setFocus();
}

void MainWindow::on_btnDelCurrPara_clicked() {
    if (crafts.size() <= 1) {
        QMessageBox::critical(NULL, "提示", "至少保留一份工艺");
        return;
    }
    DelCurrPara();
    ui->cmbCraftID->removeItem(currCraftIdx);
}

void MainWindow::on_btnStop_clicked() {
    ui->btnStop->setEnabled(false);
    SetBackgroundColor(ui->btnStop, greenColor);
    std::thread t([this] {
        bool stopSuccess = robot.Stop();
        QThread::msleep(200);
        QMetaObject::invokeMethod(this, [this, stopSuccess] {
            if (stopSuccess) {
                SetBackgroundColor(ui->btnDrag, defaultColor);
            }
            ui->btnStop->setEnabled(true);
            SetBackgroundColor(ui->btnStop, defaultColor);
        });
    });
    t.detach();
}

void MainWindow::on_btnAux_clicked() {
    QString strPoint = "";
    if (robot.GetAuxPoint(strPoint)) {
        AddHistoryPoint(strPoint);
        SetBackgroundColor(ui->btnAux, greenColor);
    } else {
        SetBackgroundColor(ui->btnAux, defaultColor);
    }
}

void MainWindow::on_btnMid_clicked() {
    QString strPoint = "";
    int size = robot.GetMidPoint(midPressDuration.elapsed(), strPoint);
    if (!strPoint.isEmpty()) {
        AddHistoryPoint(strPoint);
    }
    if (size > 0) {
        SetBackgroundColor(ui->btnMid, greenColor);
    } else {
        SetBackgroundColor(ui->btnMid, defaultColor);
    }
    ui->btnMid->setText("中间点" + QString::number(size));
}

void MainWindow::on_cmbCraftID_editTextChanged(const QString &arg1) {
    ui->cmbCraftID->setItemText(ui->cmbCraftID->currentIndex(), arg1);
    ui->leCraftID->setText(arg1);
}

void MainWindow::on_leOffsetCount_editingFinished() {
    crafts[currCraftIdx].offsetCount = ui->leOffsetCount->text().toInt();
}

void MainWindow::on_btnBeginOffset_clicked() {
    QString strPoint = "";
    if (robot.GetBeginOffsetPoint(strPoint)) {
        AddHistoryPoint(strPoint);
        SetBackgroundColor(ui->btnBeginOffset, greenColor);
    } else {
        SetBackgroundColor(ui->btnBeginOffset, defaultColor);
    }
}

void MainWindow::on_btnEndOffset_clicked() {
    QString strPoint = "";
    if (robot.GetEndOffsetPoint(strPoint)) {
        AddHistoryPoint(strPoint);
        SetBackgroundColor(ui->btnEndOffset, greenColor);
    } else {
        SetBackgroundColor(ui->btnEndOffset, defaultColor);
    }
}

void MainWindow::on_btnClear_clicked() {
    if (robot.ClearPoints()) {
        SetBackgroundColor(ui->btnSafe, defaultColor);
        SetBackgroundColor(ui->btnBegin, defaultColor);
        SetBackgroundColor(ui->btnEnd, defaultColor);
        SetBackgroundColor(ui->btnAux, defaultColor);
        SetBackgroundColor(ui->btnBeginOffset, defaultColor);
        SetBackgroundColor(ui->btnEndOffset, defaultColor);
        SetBackgroundColor(ui->btnMid, defaultColor);
        ui->btnMid->setText("中间点" + QString::number(0));
    }
}

void MainWindow::on_btnOpenWeb_clicked() {
    robot.OpenWeb(ui->leRobotIP->text());
}

void MainWindow::on_leAddOffsetCount_editingFinished() {
    crafts[currCraftIdx].addOffsetCount = ui->leAddOffsetCount->text().toInt();
}

void MainWindow::on_btnMid_pressed() { midPressDuration.start(); }

void MainWindow::on_btnClearMid_clicked() {
    if (robot.ClearMidPoints()) {
        SetBackgroundColor(ui->btnMid, defaultColor);
        ui->btnMid->setText("中间点0");
    }
}

void MainWindow::on_btnDelLastMid_clicked() {
    int size = robot.DelLastMidPoint();
    if (size == 0) {
        SetBackgroundColor(ui->btnMid, defaultColor);
    }
    ui->btnMid->setText("中间点" + QString::number(size));
}

void MainWindow::on_btnMoveToPoint_clicked() {
    QString strPoint = ui->leHistoryPoint->text();
    if (strPoint.contains("：")) {
        QStringList strValues =
            strPoint.mid(strPoint.indexOf("：") + 1).split("、");
        ui->btnRun->setEnabled(false);
        ui->btnTryRun->setEnabled(false);
        ui->btnMoveToPoint->setEnabled(false);
        SetBackgroundColor(ui->btnMoveToPoint, greenColor);
        ui->btnMoveToPoint->setText("移动中");
        if (robot.CloseFreeDriver()) {
            SetBackgroundColor(ui->btnDrag, defaultColor);
        }
        std::thread t([this, strValues] {
            robot.MoveToPoint(strValues);
            QMetaObject::invokeMethod(this, [this] {
                ui->btnRun->setEnabled(true);
                ui->btnTryRun->setEnabled(true);
                ui->btnMoveToPoint->setEnabled(true);
                SetBackgroundColor(ui->btnMoveToPoint, defaultColor);
                ui->btnMoveToPoint->setText("移动到点");
            });
        });
        t.detach();
    }
}

void MainWindow::on_leDiscRadius_editingFinished() {
    crafts[currCraftIdx].discRadius = ui->leDiscRadius->text().toInt();
}

void MainWindow::on_leGrindAngle_editingFinished() {
    crafts[currCraftIdx].grindAngle = ui->leGrindAngle->text().toInt();
}

void MainWindow::on_btnStop2_clicked() {
    ui->btnStop2->setEnabled(false);
    SetBackgroundColor(ui->btnStop2, greenColor);
    std::thread t([this] {
        bool stopSuccess = robot.Stop();
        QThread::msleep(200);
        QMetaObject::invokeMethod(this, [this, stopSuccess] {
            if (stopSuccess) {
                SetBackgroundColor(ui->btnDrag, defaultColor);
            }
            ui->btnStop2->setEnabled(true);
            SetBackgroundColor(ui->btnStop2, defaultColor);
        });
    });
    t.detach();
}

void MainWindow::on_btnInputPoint_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("选择点位文件"), "", tr("文本文件 (*.txt)"));
    if (filePath.isEmpty()) return;

    QString logInfo;
    QStringList loadedPoints; // 用于接收点位字符串

    // 调用修改后的函数
    bool success = robot.ReadInputPoint(filePath, logInfo, loadedPoints);

    if (success) {
        QMessageBox::information(this, tr("导入成功"), logInfo);

        // 遍历所有返回的点位字符串，更新 Page3 列表
        for (const QString &str : loadedPoints) {
            AddHistoryPoint(str); // 这里传入的 str 包含了 "：", 可以被正确处理
        }

        // 可选：更新按钮颜色（如上一条回答所述）
        SetBackgroundColor(ui->btnBegin, greenColor); // 简单粗暴设为绿色，或者根据 loadedPoints 内容判断
        // ...
    } else {
        QMessageBox::warning(this, tr("导入失败"), logInfo);
    }
}

void MainWindow::on_btnClearHistory_clicked() { ui->lstHistoryPoint->clear(); }

void MainWindow::on_btnCoverPoint_clicked() {
    QString strPoint = ui->leHistoryPoint->text();
    if (strPoint.isEmpty()) {
        return;
    }
    robot.CoverPoint(strPoint);
    AddHistoryPoint(strPoint);
}

void MainWindow::on_leRaiseCount_editingFinished() {
    crafts[currCraftIdx].raiseCount = ui->leRaiseCount->text().toInt();
}

void MainWindow::on_leFloatCount_editingFinished() {
    crafts[currCraftIdx].floatCount = ui->leFloatCount->text().toInt();
}

void MainWindow::on_leDiscThickness_editingFinished() {
    crafts[currCraftIdx].discThickness = ui->leDiscThickness->text().toInt();
    robot.discThickness = ui->leDiscThickness->text().toInt();
}

void MainWindow::on_leTransitionRadius_editingFinished() {
    crafts[currCraftIdx].transitionRadius =
        ui->leTransitionRadius->text().toInt();
}

void MainWindow::on_chkMirror_stateChanged(int arg1) {
    switch (arg1) {
    case Qt::Unchecked:
        crafts[currCraftIdx].isMirror = false;
        break;
    case Qt::Checked:
        crafts[currCraftIdx].isMirror = true;
        break;
    default:
        break;
    }
    SetPolishWay(crafts.at(currCraftIdx).way);
}

// 1. 在构造函数或 Init 函数中启动线程
void MainWindow::InitStatusMonitor() {
    // 传递robot指针给线程
    m_statusThread = new StatusThread(&robot, this);

    // 连接信号和槽
    connect(m_statusThread, &StatusThread::statusUpdated,
            this, &MainWindow::updateRobotStatusUI);

    // 启动监控线程
    m_statusThread->start();
}

// 2. 实现 UI 更新逻辑
void MainWindow::updateRobotStatusUI(
    bool robotMoving, bool robotEnable, bool robotError, int robotErrCode,
    bool robotElectrify, bool robotConnect,
    bool agpConnect, bool agpEnable, bool agpError, int agpErrCode,
    bool di1_safeDoor, bool di2_pot1, bool di3_pot2, bool di4_pot3, bool di5_pot4,
    bool di6_magOpen, bool di7_magClose, bool do5_agpAir
    ) {
    // 样式定义
    QString greenStyle = "background-color: green; border-radius: 24px;";
    QString greyStyle = "background-color: grey; border-radius: 24px;";
    QString redStyle = "background-color: red; border-radius: 24px;";

    // ========== 1. 机器人状态更新 ==========

    // 1.1 连接状态
    ui->RobotConnectStatus->setStyleSheet(robotConnect ? greenStyle : greyStyle);

    // 1.2 使能状态(需要连接后才有意义)
    if (!robotConnect) {
        ui->RobotEnableStatus->setStyleSheet(greyStyle);
    } else {
        ui->RobotEnableStatus->setStyleSheet(robotEnable ? greenStyle : redStyle);
    }

    // 1.3 上电信号
    if (!robotConnect) {
        ui->RobotPowerStatus->setStyleSheet(greyStyle);
    } else {
        ui->RobotPowerStatus->setStyleSheet(robotElectrify ? greenStyle : redStyle);
    }

    // 1.4 运动状态
    if (!robotConnect) {
        ui->RobotMotionStatus->setStyleSheet(greyStyle);
    } else {
        ui->RobotMotionStatus->setStyleSheet(robotMoving ? greenStyle : greyStyle);
    }

    // 1.5 报警状态
    if (!robotConnect) {
        ui->RobotAlertStatus->setStyleSheet(greyStyle);
    } else {
        ui->RobotAlertStatus->setStyleSheet(robotError ? redStyle : greenStyle);
    }

    // 1.6 报警代码显示
    if (robotConnect && robotError) {
        ui->RobotErrorCodeText->setPlainText(QString("Error: %1").arg(robotErrCode));
    } else {
        ui->RobotErrorCodeText->clear();
    }

    // ========== 2. AGP状态更新 ==========

    // 2.1 AGP连接状态
    ui->AGPConnectStatus->setStyleSheet(agpConnect ? greenStyle : greyStyle);

    // 2.2 AGP使能状态
    if (!agpConnect) {
        ui->AGPEnableStatus->setStyleSheet(greyStyle);
    } else {
        ui->AGPEnableStatus->setStyleSheet(agpEnable ? greenStyle : redStyle);
    }

    // 2.3 AGP报警状态
    if (!agpConnect) {
        ui->AGPAlertStatus->setStyleSheet(greyStyle);
    } else {
        ui->AGPAlertStatus->setStyleSheet(agpError ? redStyle : greenStyle);
    }

    // 2.4 AGP错误代码
    if (agpConnect && agpError) {
        ui->AGPErrorCodeText->setPlainText(QString("Error: %1").arg(agpErrCode));
    } else {
        ui->AGPErrorCodeText->clear();
    }

    // 2.5 AGP冷却气状态(DO5控制)
    if (!agpConnect) {
        ui->AGPAirCoolingStatus->setStyleSheet(greyStyle);
    } else {
        ui->AGPAirCoolingStatus->setStyleSheet(do5_agpAir ? greenStyle : greyStyle);
    }

    // ========== 3. 刀库IO状态更新 ==========

    // 3.1 安全门状态(DI1)
    // 绿色=门关闭(安全), 红色=门打开(不安全)
    if (!robotConnect) {
        ui->SafeDoorStatus->setStyleSheet(greyStyle);
    } else {
        ui->SafeDoorStatus->setStyleSheet(di1_safeDoor ? greenStyle : redStyle);
    }

    // 3.2 刀库各刀位检测状态(DI2-DI5)
    // 绿色=检测到刀具, 灰色=未检测到刀具
    if (!robotConnect) {
        ui->ToolMagazinePot1Status->setStyleSheet(greyStyle);
        ui->ToolMagazinePot2Status->setStyleSheet(greyStyle);
        ui->ToolMagazinePot3Status->setStyleSheet(greyStyle);
        ui->ToolMagazinePot4Status->setStyleSheet(greyStyle);
    } else {
        ui->ToolMagazinePot1Status->setStyleSheet(di2_pot1 ? greenStyle : greyStyle);
        ui->ToolMagazinePot2Status->setStyleSheet(di3_pot2 ? greenStyle : greyStyle);
        ui->ToolMagazinePot3Status->setStyleSheet(di4_pot3 ? greenStyle : greyStyle);
        ui->ToolMagazinePot4Status->setStyleSheet(di5_pot4 ? greenStyle : greyStyle);
    }

    // 3.3 刀库门开关状态(DI6/DI7)
    if (!robotConnect) {
        ui->ToolMagzaineOpen->setStyleSheet(greyStyle);
        ui->ToolMagzaineClose->setStyleSheet(greyStyle);
    } else {
        ui->ToolMagzaineOpen->setStyleSheet(di6_magOpen ? greenStyle : greyStyle);
        ui->ToolMagzaineClose->setStyleSheet(di7_magClose ? greenStyle : greyStyle);
    }

    // ========== 4. 刀库换刀状态更新(递增红色显示) ==========
    int targetPos = crafts.at(currCraftIdx).targetToolPos;

    // 默认全部绿色
    if(!robotConnect) {
        ui->ToolMagazinePot1Status_changed->setStyleSheet(greyStyle);
        ui->ToolMagazinePot2Status_changed->setStyleSheet(greyStyle);
        ui->ToolMagazinePot3Status_changed->setStyleSheet(greyStyle);
        ui->ToolMagazinePot4Status_changed->setStyleSheet(greyStyle);
    } else {
        ui->ToolMagazinePot1Status_changed->setStyleSheet(greenStyle);
        ui->ToolMagazinePot2Status_changed->setStyleSheet(greenStyle);
        ui->ToolMagazinePot3Status_changed->setStyleSheet(greenStyle);
        ui->ToolMagazinePot4Status_changed->setStyleSheet(greenStyle);
    }


    // 根据targetPos递增设置为红色(表示已换刀)
    if (targetPos >= 1) {
        ui->ToolMagazinePot1Status_changed->setStyleSheet(redStyle);
    }
    if (targetPos >= 2) {
        ui->ToolMagazinePot2Status_changed->setStyleSheet(redStyle);
    }
    if (targetPos >= 3) {
        ui->ToolMagazinePot3Status_changed->setStyleSheet(redStyle);
    }
    if (targetPos >= 4) {
        ui->ToolMagazinePot4Status_changed->setStyleSheet(redStyle);
    }
}

void MainWindow::on_ToolMagazineReset_clicked()
{
    // 1. 重置内存中当前工艺的各项换刀相关数据
    crafts[currCraftIdx].totalPolishCount = 0;
    crafts[currCraftIdx].totalPolishLength = 0.0;
    crafts[currCraftIdx].startToolChange = false;
    crafts[currCraftIdx].toolChangeDone = false;
    crafts[currCraftIdx].toolMagStatus = 0;
    crafts[currCraftIdx].targetToolPos = 0;

    // 2. 将修改后的参数同步到本地配置文件 config.ini 中
    SavePara(currCraftIdx);
}

