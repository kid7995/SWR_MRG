#include <QDebug>
#include <QFile>
#include <QMessageBox>
#include <QSettings>
#include <QTextStream>
#include <QThread>

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
    // ui->lblAddOffsetCount->setVisible(false);
    // ui->leAddOffsetCount->setVisible(false);
    // 读取工艺参数文件
    QString fileName = QCoreApplication::applicationDirPath();
    // QString fileName =
    //     QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    fileName += "/config.ini";
    if (!QFile::exists(fileName)) {
        // 打开文件用于写入，如果文件不存在则创建它
        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            out.setCodec("UTF-8"); // 设置编码，确保文本文件以UTF-8编码保存
            // 将字符串写入文件
            out << QString(
                "[CraftParameter]\nsize=1\n1\\CraftName=\\x4e00\n1\\PolishMode="
                "0\n1\\PolishWay=0\n1\\TeachingPointReferencePosition="
                "7\n1\\CutinSpeed=20\n1\\MovingSpeed=80\n1\\RotationSpeed="
                "4500\n1\\ContactForce=10\n1\\SettingForce="
                "80\n1\\TransitionTime=1500\n1\\DiscRadius=50\n1\\GrindAngle="
                "0\n1\\OffsetCount=0\n1\\AddOffsetCount=0\n1\\RaiseCount="
                "0\n1\\FloatCount=0\n");
            // 关闭文件
            file.close();
        } else {
            // 如果文件无法打开，可以在这里处理错误
            setEnabled(false);
            QMessageBox::critical(NULL, "提示", "无法创建工艺参数文件");
            return;
        }
    }
    QSettings settings(fileName, QSettings::IniFormat);
    int size = settings.beginReadArray("CraftParameter");
    if (size == 0) {
        return;
    }
    // 读取所有工艺参数
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
        craft.discRadius = settings.value("DiscRadius").toInt();
        craft.grindAngle = settings.value("GrindAngle").toInt();
        craft.offsetCount = settings.value("OffsetCount").toInt();
        craft.addOffsetCount = settings.value("AddOffsetCount").toInt();
        craft.raiseCount = settings.value("RaiseCount").toInt();
        craft.floatCount = settings.value("FloatCount").toInt();
        crafts.append(craft);
    }
    settings.endArray();
    // 添加工艺编号
    for (int i = 0; i < size; ++i) {
        ui->cmbCraftID->addItem(crafts.at(i).craftID);
    }
    // 打磨方式首页界面设置
    SetPolishWay(crafts.at(0).way);
    // 设置验证器
    SetValidator();
    // 连接机器人
    ConnectRobot();
    // 连接AGP
    ConnectAGP();
    // test
    // EnableButtons();
    // Point::test();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::SavePara(int index) {
    // 保存工艺参数
    QString fileName = QCoreApplication::applicationDirPath();
    fileName += "/config.ini";
    QSettings settings(fileName, QSettings::IniFormat);
    settings.beginWriteArray("CraftParameter");
    settings.setArrayIndex(index);
    settings.setValue("CraftName", crafts.at(index).craftID);
    settings.setValue("PolishMode", crafts.at(index).mode);
    settings.setValue("PolishWay", crafts.at(index).way);
    settings.setValue("TeachingPointReferencePosition",
                      crafts.at(index).teachPointReferPos);
    settings.setValue("CutinSpeed", crafts.at(index).cutinSpeed);
    settings.setValue("MovingSpeed", crafts.at(index).moveSpeed);
    settings.setValue("RotationSpeed", crafts.at(index).rotateSpeed);
    settings.setValue("ContactForce", crafts.at(index).contactForce);
    settings.setValue("SettingForce", crafts.at(index).settingForce);
    settings.setValue("TransitionTime", crafts.at(index).transitionTime);
    settings.setValue("DiscRadius", crafts.at(index).discRadius);
    settings.setValue("GrindAngle", crafts.at(index).grindAngle);
    settings.setValue("OffsetCount", crafts.at(index).offsetCount);
    settings.setValue("AddOffsetCount", crafts.at(index).addOffsetCount);
    settings.setValue("RaiseCount", crafts.at(index).raiseCount);
    settings.setValue("FloatCount", crafts.at(index).floatCount);
    // 记录工艺参数数量
    settings.setArrayIndex(crafts.size() - 1);
    settings.endArray();
}

void MainWindow::ReadCurrPara() {
    // 读取当前工艺参数
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
    ui->leDiscRadius->setText(
        QString::number(crafts.at(currCraftIdx).discRadius));
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
}

void MainWindow::DelCurrPara() {
    // 删除当前工艺参数
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
    // 首页按钮
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

    // 参数页面按钮
    // SetBackgroundColor(ui->btnAddNewPara, defaultColor);
    // SetBackgroundColor(ui->btnDelCurrPara, defaultColor);
    // SetBackgroundColor(ui->btnSaveCurrPara, defaultColor);

    // 历史点页面按钮
    ui->btnClearHistory->setVisible(true);
    SetBackgroundColor(ui->btnClearHistory, greyColor);
    ui->btnCoverPoint->setVisible(true);
    SetBackgroundColor(ui->btnCoverPoint, greyColor);
    ui->btnMoveToPoint->setVisible(true);
    SetBackgroundColor(ui->btnMoveToPoint, greyColor);
    ui->btnStop2->setVisible(true);
    SetBackgroundColor(ui->btnStop2, greyColor);

    // 设备连接页面按钮
    ui->btnRobotConnect->setVisible(true);
    SetBackgroundColor(ui->btnRobotConnect, defaultColor);
    ui->btnAGPConnect->setVisible(true);
    SetBackgroundColor(ui->btnAGPConnect, defaultColor);
    // SetBackgroundColor(ui->btnSettingReturn, defaultColor);
}

void MainWindow::EnableButtons() {
    // 启用首页按钮
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
    // 设置验证器
    ui->leCutinSpeed->setValidator(new QIntValidator(ui->leCutinSpeed));
    ui->leMoveSpeed->setValidator(new QIntValidator(ui->leMoveSpeed));
    ui->leRotateSpeed->setValidator(new QIntValidator(ui->leRotateSpeed));
    ui->leContactForce->setValidator(new QIntValidator(ui->leContactForce));
    ui->leTransitionTime->setValidator(new QIntValidator(ui->leTransitionTime));
    ui->leSettingForce->setValidator(new QIntValidator(ui->leSettingForce));
    ui->leTeachPos->setValidator(new QIntValidator(ui->leTeachPos));
    ui->leDiscRadius->setValidator(new QIntValidator(ui->leDiscRadius));
    ui->leGrindAngle->setValidator(new QIntValidator(ui->leGrindAngle));
    ui->leOffsetCount->setValidator(new QIntValidator(ui->leOffsetCount));
    ui->leAddOffsetCount->setValidator(new QIntValidator(ui->leAddOffsetCount));
    ui->leRaiseCount->setValidator(new QIntValidator(ui->leRaiseCount));
    ui->leFloatCount->setValidator(new QIntValidator(ui->leFloatCount));
}

void MainWindow::SetPolishWay(const PolishWay &way) {
    switch (way) {
    case PolishWay::ArcWay:
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
        ui->leOffsetCount->setEnabled(false);
        ui->lblBackground->setPixmap(QPixmap(":/pic/arc.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 110);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(false);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::LineWay:
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
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
        ui->leDiscRadius->setEnabled(false);
        ui->leGrindAngle->setEnabled(false);
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
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
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
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
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
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/cylinder_concave.png"));
        ui->btnAux->setVisible(false);
        ui->btnMid->move(550, 180);
        ui->btnMid->setVisible(true);
        ui->btnBeginOffset->setVisible(true);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::ZLineWay:
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
        ui->leOffsetCount->setEnabled(true);
        ui->lblBackground->setPixmap(QPixmap(":/pic/z_line.png"));
        ui->btnAux->move(150, 130);
        ui->btnAux->setVisible(true);
        ui->btnMid->setVisible(false);
        ui->btnBeginOffset->setVisible(false);
        ui->btnEndOffset->setVisible(false);
        break;
    case PolishWay::SpiralLineWay:
        ui->leDiscRadius->setEnabled(true);
        ui->leGrindAngle->setEnabled(true);
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
        ui->btnBegin->move(150, 210);
        ui->btnEnd->move(930, 210);
        ui->btnBeginOffset->move(150, 360);
        break;
    default:
        ui->btnBegin->move(150, 310);
        ui->btnEnd->move(930, 310);
        ui->btnBeginOffset->move(320, 430);
        break;
    }
}

void MainWindow::SetBackgroundColor(QPushButton *btn, const QColor &color) {
    QPalette pal = btn->palette();
    pal.setColor(QPalette::Button, color);
    btn->setPalette(pal);
    btn->setAutoFillBackground(true);
    // btn->show();
}

void MainWindow::ConnectRobot() {
    ui->btnRobotConnect->setEnabled(false);
    SetBackgroundColor(ui->btnRobotConnect, goldColor);
    ui->btnRobotConnect->setText("连接中");
    // 连接机器人
    std::thread t([this] {
        if (robot.RobotConnect(ui->leRobotIP->text())) {
            EnableButtons();
            SetBackgroundColor(ui->btnRobotConnect, greenColor);
            ui->btnRobotConnect->setText("已连接");
        } else {
            ui->btnRobotConnect->setEnabled(true);
            SetBackgroundColor(ui->btnRobotConnect, defaultColor);
            ui->btnRobotConnect->setText("连接");
        }
    });
    t.detach();
}

void MainWindow::ConnectAGP() {
    ui->btnAGPConnect->setEnabled(false);
    SetBackgroundColor(ui->btnAGPConnect, goldColor);
    ui->btnAGPConnect->setText("连接中");
    // 连接AGP
    std::thread t([this] {
        if (robot.AGPConnect(ui->leAGPIP->text())) {
            SetBackgroundColor(ui->btnAGPConnect, greenColor);
            ui->btnAGPConnect->setText("已连接");
        } else {
            ui->btnAGPConnect->setEnabled(true);
            SetBackgroundColor(ui->btnAGPConnect, defaultColor);
            ui->btnAGPConnect->setText("连接");
        }
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
        // ui->lstHistoryPoint->insertItem(0, strPoint);
        ui->lstHistoryPoint->addItem(strPoint);
    } else {
        for (const auto &pItem : pItems) {
            pItem->setText(strPoint);
        }
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
    int size = ui->stackedWidget->count() - 1;
    int i = ui->stackedWidget->currentIndex();
    ui->stackedWidget->setCurrentIndex((i - 1 + size) % size);
    ui->lblPageName->setText(
        ui->stackedWidget->currentWidget()->accessibleName());
}

void MainWindow::on_btnNext_clicked() {
    int size = ui->stackedWidget->count() - 1;
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
        // QThread::msleep(1000);
        SetBackgroundColor(ui->btnDrag, defaultColor);
    }
    // robot.Run(crafts.at(currCraftIdx), false);
    // AGP停止
    std::thread t([this] {
        robot.Run(crafts.at(currCraftIdx), false);
        ui->btnRun->setEnabled(true);
        ui->btnTryRun->setEnabled(true);
        ui->btnMoveToPoint->setEnabled(true);
        SetBackgroundColor(ui->btnTryRun, defaultColor);
        ui->btnTryRun->setText("试运行");
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
        // QThread::msleep(1000);
        SetBackgroundColor(ui->btnDrag, defaultColor);
    }
    // robot.Run(crafts.at(currCraftIdx), true);
    // AGP停止
    std::thread t([this] {
        robot.Run(crafts.at(currCraftIdx), true);
        robot.AGPStop();
        ui->btnRun->setEnabled(true);
        ui->btnTryRun->setEnabled(true);
        ui->btnMoveToPoint->setEnabled(true);
        SetBackgroundColor(ui->btnRun, defaultColor);
        ui->btnRun->setText("运行");
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
        if (robot.Stop()) {
            SetBackgroundColor(ui->btnDrag, defaultColor);
        }
        QThread::msleep(200);
        ui->btnStop->setEnabled(true);
        SetBackgroundColor(ui->btnStop, defaultColor);
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
            ui->btnRun->setEnabled(true);
            ui->btnTryRun->setEnabled(true);
            ui->btnMoveToPoint->setEnabled(true);
            SetBackgroundColor(ui->btnMoveToPoint, defaultColor);
            ui->btnMoveToPoint->setText("移动到点");
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
        if (robot.Stop()) {
            SetBackgroundColor(ui->btnDrag, defaultColor);
        }
        QThread::msleep(200);
        ui->btnStop2->setEnabled(true);
        SetBackgroundColor(ui->btnStop2, defaultColor);
    });
    t.detach();
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
