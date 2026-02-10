#include <Qdebug>
#include <QDesktopServices>
#include <QMessageBox>
#include <QThread>
#include <QUrl>
#include <QFile>
#include <QTextStream>
#include <QStringList>

#include "robot.h"

constexpr int defaultOffset = -30;
constexpr OffsetDirection defaultDirection = OffsetDirection::OffsetZ;
constexpr double defaultVelocity = 200;
constexpr double precision = 1e-4;

int status = -1;
std::string robotIPAddr;

Robot::Robot()
    : agp(nullptr), isTeach(false), isStop(true), discThickness(0),
      teachPos(0) {}

Robot::~Robot() {
    if (agp != nullptr) {
        delete agp;
        agp = nullptr;
    }
}

bool Robot::AGPConnect(QString agpIP) {
    if (agp != nullptr) {
        delete agp;
    }
    agp = new AGP(agpIP.toStdString());
    if (agp != nullptr && agp->AGP_connect()) {
        agp->Control(FUNC::RESET);
        agp->Control(FUNC::ENABLE);
        agp->SetMode(MODE::ForceMode);
        agp->SetLoadWeight(22);
        agp->SetForce(20);
        agp->SetSpeed(0);
        return true;
    }
    return false;
}

void Robot::AGPRun(const Craft &craft, bool isRotated) {
    // 设置AGP参数
    agp->Control(FUNC::RESET);
    agp->Control(FUNC::ENABLE);
    switch (craft.mode) {
    case PolishMode::MomentMode:
        agp->SetMode(MODE::ForceMode);
        break;
    case PolishMode::PositionMode:
        agp->SetMode(MODE::PosMode);
        break;
    default:
        break;
    }
    if (isRotated) {
        agp->SetSpeed(craft.rotateSpeed);
    } else {
        agp->SetSpeed(0);
    }
    agp->SetTouchForce(craft.contactForce);
    agp->SetRampTime(craft.transitionTime);
    agp->SetForce(craft.settingForce);
    agp->SetPos(craft.teachPointReferPos * 100);
}

void Robot::AGPStop() {
    if (agp != nullptr) {
        while (true) {
            if (!IsRobotMoved()) {
                agp->SetSpeed(0);
                break;
            }
        }
    }
}

bool Robot::IsAGPEnabled() {
    if (agp != nullptr) {
        int16_t state = agp->ReadStatus();
        if ((state & 0x01) == 1) {
            return true;
        }
    }
    return false;
}

bool Robot::GetPoint(Point &point) {
    if (!GetTcpPoint(point)) {
        return false;
    }
    double pos = teachPos;
    if (agp != nullptr) {
        pos = agp->ReadPos() / 100.0;
    }
    // point = point.PosRelByTool(defaultDirection, pos + discThickness);
    // 取消打磨片厚度
    point = point.PosRelByTool(defaultDirection, pos);
    return true;
}

bool Robot::ReadInputPoint(const QString &filePath, QString &logInfo, QStringList &loadedPoints) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        logInfo = "无法打开文件: " + filePath;
        return false;
    }

    QTextStream in(&file);
    in.setCodec("UTF-8");

    // 如果是重新导入，建议先清空旧的中间点，防止重复叠加
    // pointSet.midPoints.clear();

    bool hasUpdate = false;
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList parts = line.split(",");
        if (parts.size() < 7) continue;

        QString pointType = parts.at(0);
        float x = parts.at(1).toFloat();
        float y = parts.at(2).toFloat();
        float z = parts.at(3).toFloat();
        float rx = parts.at(4).toFloat();
        float ry = parts.at(5).toFloat();
        float rz = parts.at(6).toFloat();

        Point tempPoint(x, y, z, rx, ry, rz);

        if (pointType == "beginPoint") {
            pointSet.beginPoint = tempPoint;
            pointSet.isBeginPointRecorded = true;
            logInfo += "起始点已更新; ";
            loadedPoints.append("起始点：" + tempPoint.toString());
            hasUpdate = true;
        }
        else if (pointType == "safePoint") {
            pointSet.safePoint = tempPoint;
            pointSet.isSafePointRecorded = true;
            logInfo += "安全点已更新; ";
            loadedPoints.append("安全点：" + tempPoint.toString());
            hasUpdate = true;
        }
        else if (pointType == "endPoint") {
            pointSet.endPoint = tempPoint;
            pointSet.isEndPointRecorded = true;
            logInfo += "结束点已更新; ";
            loadedPoints.append("结束点：" + tempPoint.toString());
            hasUpdate = true;
        }
        else if (pointType == "auxPoint") {
            pointSet.auxPoint = tempPoint;
            pointSet.isAuxPointRecorded = true;
            logInfo += "辅助点已更新; ";
            loadedPoints.append("辅助点：" + tempPoint.toString());
            hasUpdate = true;
        }
        // === 【新增】起始偏移点 ===
        else if (pointType == "beginOffsetPoint") {
            pointSet.beginOffsetPoint = tempPoint;
            pointSet.isBeginOffsetPointRecorded = true;
            logInfo += "起始偏移点已更新; ";
            loadedPoints.append("起始偏移点：" + tempPoint.toString());
            hasUpdate = true;
        }
        // === 【新增】结束偏移点 ===
        else if (pointType == "endOffsetPoint") {
            pointSet.endOffsetPoint = tempPoint;
            pointSet.isEndOffsetPointRecorded = true;
            logInfo += "结束偏移点已更新; ";
            loadedPoints.append("结束偏移点：" + tempPoint.toString());
            hasUpdate = true;
        }
        // === 【新增】中间点 ===
        else if (pointType == "midPoint") {
            // 注意：中间点是列表，通常是 append 添加
            pointSet.midPoints.append(tempPoint);
            int idx = pointSet.midPoints.size();
            logInfo += QString("中间点%1已添加; ").arg(idx);
            loadedPoints.append(QString("中间点%1：%2").arg(idx).arg(tempPoint.toString()));
            hasUpdate = true;
        }
    }

    file.close();
    if (!hasUpdate) logInfo = "未在文件中找到有效的点位信息";
    return hasUpdate;
}

bool Robot::GetSafePoint(QString &strPoint) {
    if (!pointSet.isSafePointRecorded) {
        if (GetPoint(pointSet.safePoint)) {
            pointSet.isSafePointRecorded = true;
            strPoint = QString("安全点：") + pointSet.safePoint.toString();
        }
    } else {
        pointSet.isSafePointRecorded = false;
    }
    return pointSet.isSafePointRecorded;
}

bool Robot::GetBeginPoint(QString &strPoint) {
    if (!pointSet.isBeginPointRecorded) {
        if (GetPoint(pointSet.beginPoint)) {
            pointSet.auxBeginPoint = pointSet.beginPoint.PosRelByTool(
                defaultDirection, defaultOffset);
            pointSet.isBeginPointRecorded = true;
            strPoint = QString("起始点：") + pointSet.beginPoint.toString();
        }
    } else {
        pointSet.isBeginPointRecorded = false;
    }
    return pointSet.isBeginPointRecorded;
}

bool Robot::GetEndPoint(QString &strPoint) {
    if (!pointSet.isEndPointRecorded) {
        if (GetPoint(pointSet.endPoint)) {
            pointSet.auxEndPoint =
                pointSet.endPoint.PosRelByTool(defaultDirection, defaultOffset);
            pointSet.isEndPointRecorded = true;
            strPoint = QString("结束点：") + pointSet.endPoint.toString();
        }
    } else {
        pointSet.isEndPointRecorded = false;
    }
    return pointSet.isEndPointRecorded;
}

bool Robot::GetAuxPoint(QString &strPoint) {
    if (!pointSet.isAuxPointRecorded) {
        if (GetPoint(pointSet.auxPoint)) {
            pointSet.isAuxPointRecorded = true;
            strPoint = QString("辅助点：") + pointSet.auxPoint.toString();
        }
    } else {
        pointSet.isAuxPointRecorded = false;
    }
    return pointSet.isAuxPointRecorded;
}

bool Robot::GetBeginOffsetPoint(QString &strPoint) {
    if (!pointSet.isBeginOffsetPointRecorded) {
        if (GetPoint(pointSet.beginOffsetPoint)) {
            pointSet.isBeginOffsetPointRecorded = true;
            strPoint =
                QString("起始偏移点：") + pointSet.beginOffsetPoint.toString();
        }
    } else {
        pointSet.isBeginOffsetPointRecorded = false;
    }
    return pointSet.isBeginOffsetPointRecorded;
}

bool Robot::GetEndOffsetPoint(QString &strPoint) {
    if (!pointSet.isEndOffsetPointRecorded) {
        if (GetPoint(pointSet.endOffsetPoint)) {
            pointSet.isEndOffsetPointRecorded = true;
            strPoint =
                QString("结束偏移点：") + pointSet.endOffsetPoint.toString();
        }
    } else {
        pointSet.isEndOffsetPointRecorded = false;
    }
    return pointSet.isEndOffsetPointRecorded;
}

int Robot::GetMidPoint(qint64 pressDuration, QString &strPoint) {
    if (pressDuration > 500) {
        DelLastMidPoint();
    } else {
        Point point;
        if (GetPoint(point)) {
            pointSet.midPoints.append(point);
            strPoint = QString("中间点%1：").arg(pointSet.midPoints.size()) +
                       point.toString();
        }
    }
    return pointSet.midPoints.size();
}

bool Robot::ClearPoints() {
    if (pointSet.isSafePointRecorded) {
        pointSet.isSafePointRecorded = false;
    }
    if (pointSet.isBeginPointRecorded) {
        pointSet.isBeginPointRecorded = false;
    }
    if (pointSet.isEndPointRecorded) {
        pointSet.isEndPointRecorded = false;
    }
    if (pointSet.isAuxPointRecorded) {
        pointSet.isAuxPointRecorded = false;
    }
    if (pointSet.isBeginOffsetPointRecorded) {
        pointSet.isBeginOffsetPointRecorded = false;
    }
    if (pointSet.isEndOffsetPointRecorded) {
        pointSet.isEndOffsetPointRecorded = false;
    }
    pointSet.midPoints.clear();
    return true;
}

bool Robot::ClearMidPoints() {
    pointSet.midPoints.clear();
    return true;
}

int Robot::DelLastMidPoint() {
    if (!pointSet.midPoints.isEmpty()) {
        pointSet.midPoints.removeLast();
    }
    return pointSet.midPoints.size();
}

bool Robot::CheckAllPoints(const PolishWay &way) {
    // 检查路径所需点位是否采集
    QVector<QString> check;
    if (!pointSet.isSafePointRecorded) {
        check.append("安全点");
    }
    if (!pointSet.isBeginPointRecorded) {
        check.append("起始点");
    }
    if (!pointSet.isEndPointRecorded) {
        check.append("结束点");
    }
    switch (way) {
    case PolishWay::RegionArcWay1:
    case PolishWay::RegionArcWay2:
    case PolishWay::RegionArcWay_Horizontal:
    case PolishWay::RegionArcWay_Vertical:
    case PolishWay::RegionArcWay_Vertical_Repeat:
        if (!pointSet.isEndOffsetPointRecorded) {
            check.append("结束偏移点");
        }
    case PolishWay::CylinderWay_Horizontal_Convex:
    case PolishWay::CylinderWay_Vertical_Convex:
    case PolishWay::CylinderWay_Horizontal_Concave:
    case PolishWay::CylinderWay_Vertical_Concave:
        if (!pointSet.isBeginOffsetPointRecorded) {
            check.append("起始偏移点");
        }
    case PolishWay::ArcWay:
        if (pointSet.midPoints.isEmpty()) {
            check.append("中间点");
        }
        break;
    case PolishWay::ZLineWay:
    case PolishWay::SpiralLineWay:
        if (!pointSet.isAuxPointRecorded) {
            check.append("辅助点");
        }
        break;
    case PolishWay::LineWay:
    default:
        break;
    }
    int size = check.size();
    if (size != 0) {
        QString tip = "";
        for (int i = 0; i < size; ++i) {
            if (i != 0) {
                tip += "、";
            }
            tip += check.at(i);
        }
        tip += "未记录";
        QMessageBox::critical(NULL, "提示", tip);
        return false;
    }
    switch (way) {
    case PolishWay::ArcWay:
    case PolishWay::RegionArcWay1:
    case PolishWay::RegionArcWay2:
    case PolishWay::RegionArcWay_Horizontal:
    case PolishWay::RegionArcWay_Vertical:
    case PolishWay::RegionArcWay_Vertical_Repeat:
    case PolishWay::CylinderWay_Horizontal_Convex:
    case PolishWay::CylinderWay_Vertical_Convex:
    case PolishWay::CylinderWay_Horizontal_Concave:
    case PolishWay::CylinderWay_Vertical_Concave:
        if (pointSet.midPoints.size() % 2 == 0) {
            QMessageBox::critical(NULL, "提示", "圆弧中间点数量不能为偶数个！");
            return false;
        }
        break;
    default:
        break;
    }
    return true;
}

void Robot::CoverPoint(QString &strPoint) {
    if (strPoint.startsWith("安全点")) {
        if (GetPoint(pointSet.safePoint)) {
            strPoint = QString("安全点：") + pointSet.safePoint.toString();
        }
    } else if (strPoint.startsWith("起始点")) {
        if (GetPoint(pointSet.beginPoint)) {
            strPoint = QString("起始点：") + pointSet.beginPoint.toString();
        }
    } else if (strPoint.startsWith("结束点")) {
        if (GetPoint(pointSet.endPoint)) {
            strPoint = QString("结束点：") + pointSet.endPoint.toString();
        }
    } else if (strPoint.startsWith("起始偏移点")) {
        if (GetPoint(pointSet.beginOffsetPoint)) {
            strPoint =
                QString("起始偏移点：") + pointSet.beginOffsetPoint.toString();
        }
    } else if (strPoint.startsWith("结束偏移点")) {
        if (GetPoint(pointSet.endOffsetPoint)) {
            strPoint =
                QString("结束偏移点：") + pointSet.endOffsetPoint.toString();
        }
    } else if (strPoint.startsWith("辅助点")) {
        if (GetPoint(pointSet.auxPoint)) {
            strPoint = QString("辅助点：") + pointSet.auxPoint.toString();
        }
    } else if (strPoint.startsWith("中间点")) {
        int id = strPoint.left(strPoint.indexOf("：")).mid(3).toInt();
        if (id > pointSet.midPoints.size()) {
            return;
        }
        if (GetPoint(pointSet.midPoints[id - 1])) {
            strPoint = QString("中间点%1：").arg(id) +
                       pointSet.midPoints.at(id - 1).toString();
        }
    }
}

void Robot::MoveL(const Point &point, double dVelocity, double dAcc,
                  double dRadius) {
    if (isStop.load()) {
        return;
    }
    Point tcpPoint =
        point.PosRelByTool(defaultDirection, -(teachPos + discThickness));
    MoveTcpL(tcpPoint, dVelocity, dAcc, dRadius);
}

void Robot::MoveC(const Point &auxPoint, const Point &endPoint,
                  double dVelocity, double dAcc, double dRadius) {
    if (isStop.load()) {
        return;
    }
    Point auxTcpPoint =
        auxPoint.PosRelByTool(defaultDirection, -(teachPos + discThickness));
    Point endTcpPoint =
        endPoint.PosRelByTool(defaultDirection, -(teachPos + discThickness));
    MoveTcpC(auxTcpPoint, endTcpPoint, dVelocity, dAcc, dRadius);
}

void Robot::MoveToPoint(const QStringList &coordinates) {
    if (coordinates.size() != 6) {
        return;
    }
    // 定义空间目标位置
    Point point;
    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = 1;
    point.pos.setX(coordinates.at(0).toDouble());
    point.pos.setY(coordinates.at(1).toDouble());
    point.pos.setZ(coordinates.at(2).toDouble());
    point.rot.setX(coordinates.at(3).toDouble());
    point.rot.setY(coordinates.at(4).toDouble());
    point.rot.setZ(coordinates.at(5).toDouble());
    isStop.store(false);
    MoveL(point, dVelocity, dAcc, dRadius);
    // 等待运动完成
    while (true) {
        if (!IsRobotMoved()) {
            isStop.store(true);
            break;
        }
        QThread::msleep(100);
    }
}

void Robot::MoveBefore(const Craft &craft, bool isAGPRun) {
    // 定义空间目标位置
    Point point;
    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 移到安全点
    point = pointSet.safePoint;
    MoveL(point, dVelocity, dAcc, dRadius);
    // AGP运行
    AGPRun(craft, isAGPRun);
    if (craft.way != PolishWay::RegionArcWay_Vertical &&
        craft.way != PolishWay::RegionArcWay_Vertical_Repeat &&
        craft.way != PolishWay::CylinderWay_Horizontal_Convex &&
        craft.way != PolishWay::CylinderWay_Vertical_Convex &&
        craft.way != PolishWay::CylinderWay_Horizontal_Concave &&
        craft.way != PolishWay::CylinderWay_Vertical_Concave) {
        // 移到起始辅助点
        // point = pointSet.auxBeginPoint;
        if (craft.way == PolishWay::RegionArcWay1 ||
            craft.way == PolishWay::RegionArcWay2) {
            point = pointSet.beginPoint;
        } else {
            point.pos = pointSet.beginPoint.pos + translation;
            point.rot = newRot;
        }
        point = point.PosRelByTool(defaultDirection, defaultOffset);
        MoveL(point, dVelocity, dAcc, dRadius);
        // 移到起始点
        // point = pointSet.beginPoint;
        if (craft.way == PolishWay::RegionArcWay1 ||
            craft.way == PolishWay::RegionArcWay2) {
            point = pointSet.beginPoint;
        } else {
            point.pos = pointSet.beginPoint.pos + translation;
            point.rot = newRot;
        }
        dVelocity = craft.cutinSpeed;
        MoveL(point, dVelocity, dAcc, dRadius);
    }
}

void Robot::MoveAfter(const Craft &craft, Point point) {
    // 定义运动速度
    double dVelocity = craft.cutinSpeed;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 移到结束辅助点
    MoveL(point, dVelocity, dAcc, dRadius);
    // 移到安全点
    point = pointSet.safePoint;
    dVelocity = defaultVelocity;
    MoveL(point, dVelocity, dAcc, dRadius);
}

void Robot::MoveLine(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 定义空间目标位置
    Point point;
    for (int i = 0; i < pointSet.midPoints.size(); ++i) {
        // 定义空间目标位置
        // point = pointSet.midPoints.at(i);
        point.pos = pointSet.midPoints.at(i).pos + translation;
        point.rot = newRot;
        // 执行路点运动
        MoveL(point, dVelocity, dAcc, dRadius);
    }
    // 移到结束点
    // point = pointSet.endPoint;
    point.pos = pointSet.endPoint.pos + translation;
    point.rot = newRot;
    MoveL(point, dVelocity, dAcc, dRadius);
}

void Robot::MoveArc(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 圆弧运动
    int i = 0;
    Point posMidRel, posEndRel;
    for (; i < pointSet.midPoints.size() - 1; i += 2) {
        // 定义空间目标位置
        // posMidRel = pointSet.midPoints.at(i);
        posMidRel.pos = pointSet.midPoints.at(i).pos + translation;
        posMidRel.rot = newRot;
        // posEndRel = pointSet.midPoints.at(i + 1);
        posEndRel.pos = pointSet.midPoints.at(i + 1).pos + translation;
        posEndRel.rot = newRot;
        // 执行路点运动
        MoveC(posMidRel, posEndRel, dVelocity, dAcc, dRadius);
    }
    // 定义空间目标位置
    // posMidRel = pointSet.midPoints.at(i);
    posMidRel.pos = pointSet.midPoints.at(i).pos + translation;
    posMidRel.rot = newRot;
    // posEndRel = pointSet.endPoint;
    posEndRel.pos = pointSet.endPoint.pos + translation;
    posEndRel.rot = newRot;
    // 执行路点运动
    MoveC(posMidRel, posEndRel, dVelocity, dAcc, dRadius);
}

Point Robot::MoveRegionArc1(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 计算单次偏移量
    int count = craft.offsetCount;
    // Position beginOffset = (beginOffsetPoint - beginPoint) / count;
    // Position endOffset = (endOffsetPoint - endPoint) / count;
    // double lenTotal = std::sqrt(std::pow(endPoint.x - beginPoint.x, 2) +
    //                             std::pow(endPoint.y - beginPoint.y, 2) +
    //                             std::pow(endPoint.z - beginPoint.z, 2));
    // QVector<Position> midOffsetList;
    // for (int i = 0; i < arcMidPointList.size(); i++) {
    //     double len =
    //         std::sqrt(std::pow(arcMidPointList.at(i).x - beginPoint.x, 2) +
    //                   std::pow(arcMidPointList.at(i).y - beginPoint.y, 2) +
    //                   std::pow(arcMidPointList.at(i).z - beginPoint.z, 2));
    //     Position midOffset =
    //         beginOffset * (len / lenTotal) + endOffset * (1 - (len /
    //         lenTotal));
    //     midOffsetList.append(midOffset);
    // }
    Point temp1;
    temp1.pos = pointSet.endOffsetPoint.pos - pointSet.beginPoint.pos;
    Point temp2;
    temp2.pos = pointSet.endPoint.pos - pointSet.beginPoint.pos;
    Point midOffset;
    if (count != 0) {
        midOffset.pos =
            (temp1.pos -
             temp2.pos * (QVector3D::dotProduct(temp1.pos, temp2.pos) /
                          QVector3D::dotProduct(temp2.pos, temp2.pos))) /
            count;
    }
    // qDebug() << "midOffset" << midOffset.x << " " << midOffset.y << " "
    //          << midOffset.z << " " << midOffset.rx << " " << midOffset.ry <<
    //          " "
    //          << midOffset.rz;
    // 定义空间目标位置
    // 正向起始、结束
    Point posBeginRel = pointSet.beginPoint;
    // qDebug() << "beginPoint" << beginPoint.x << " " << beginPoint.y << " "
    //          << beginPoint.z << " " << beginPoint.rx << " " << beginPoint.ry
    //          << " " << beginPoint.rz;
    // qDebug() << "posBeginRel" << posBeginRel.x << " " << posBeginRel.y << " "
    //          << posBeginRel.z << " " << posBeginRel.rx << " " <<
    //          posBeginRel.ry
    //          << " " << posBeginRel.rz;
    Point posEndRel = pointSet.endPoint;
    // qDebug() << "endPoint" << endPoint.x << " " << endPoint.y << " "
    //          << endPoint.z << " " << endPoint.rx << " " << endPoint.ry << " "
    //          << endPoint.rz;
    // qDebug() << "posEndRel" << posEndRel.x << " " << posEndRel.y << " "
    //          << posEndRel.z << " " << posEndRel.rx << " " << posEndRel.ry <<
    //          " "
    //          << posEndRel.rz;
    // 反向起始、结束
    Point posBeginRelInv = pointSet.endOffsetPoint;
    posBeginRelInv.pos =
        pointSet.endOffsetPoint.pos - midOffset.pos * (count + 1);
    // qDebug() << "endOffsetPoint" << endOffsetPoint.x << " " <<
    // endOffsetPoint.y
    //          << " " << endOffsetPoint.z << " " << endOffsetPoint.rx << " "
    //          << endOffsetPoint.ry << " " << endOffsetPoint.rz;
    // qDebug() << "posBeginRelInv" << posBeginRelInv.x << " " <<
    // posBeginRelInv.y
    //          << " " << posBeginRelInv.z << " " << posBeginRelInv.rx << " "
    //          << posBeginRelInv.ry << " " << posBeginRelInv.rz;
    Point posEndRelInv = pointSet.beginOffsetPoint;
    posEndRelInv.pos =
        pointSet.beginOffsetPoint.pos - midOffset.pos * (count + 1);
    // qDebug() << "beginOffsetPoint" << beginOffsetPoint.x << " "
    //          << beginOffsetPoint.y << " " << beginOffsetPoint.z << " "
    //          << beginOffsetPoint.rx << " " << beginOffsetPoint.ry << " "
    //          << beginOffsetPoint.rz;
    // qDebug() << "posEndRelInv" << posEndRelInv.x << " " << posEndRelInv.y <<
    // " "
    //          << posEndRelInv.z << " " << posEndRelInv.rx << " "
    //          << posEndRelInv.ry << " " << posEndRelInv.rz;
    QVector<Point> posMidRelList;
    for (int i = 0; i < pointSet.midPoints.size(); i++) {
        Point posMidRel = pointSet.midPoints.at(i);
        posMidRelList.append(posMidRel);
    }
    int idx = 0;
    Point posAux, posEnd;
    for (; idx < posMidRelList.size() - 1; idx += 2) {
        // 正向圆弧运动
        posEnd = posMidRelList.at(idx + 1);
        // posEnd.pos = posMidRelList.at(idx + 1).pos + translation;
        // posEnd.rot = newRot;
        posAux = posMidRelList.at(idx);
        // posAux.pos = posMidRelList.at(idx).pos + translation;
        // posAux.rot = newRot;
        MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
    }
    // 正向圆弧运动
    posEnd = posEndRel;
    // posEnd.pos = posEndRel.pos + translation;
    // posEnd.rot = newRot;
    posAux = posMidRelList.at(idx);
    // posAux.pos = posMidRelList.at(idx).pos + translation;
    // posAux.rot = newRot;
    MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
    Point pos;
    // 新增圆弧次数
    int addCount = craft.addOffsetCount;
    for (int i = 0; i < count + addCount; ++i) {
        if (i % 2 == 0) { // 反向
            // 抬高
            pos = posEndRel;
            // pos.pos = posEndRel.pos + translation;
            // pos.rot = newRot;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            // qDebug() << pos.x << " " << pos.y << " " << pos.z << " " <<
            // pos.rx
            //          << " " << pos.ry << " " << pos.rz;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 改变位姿
            // posBeginRel += beginOffset;
            // posEndRel += endOffset;
            posBeginRelInv.pos += midOffset.pos * 2;
            posEndRelInv.pos += midOffset.pos * 2;
            for (int j = 0; j < posMidRelList.size(); j++) {
                // posMidRelList[j] += midOffsetList[j];
                posMidRelList[j] += midOffset;
            }
            pos = posBeginRelInv;
            // pos.pos = posBeginRelInv.pos + translationInv;
            // pos.rot = newRotInv;
            // pos.rx = endOffsetPoint.rx;
            // pos.ry = endOffsetPoint.ry;
            // pos.rz = endOffsetPoint.rz;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            // qDebug() << pos.x << " " << pos.y << " " << pos.z << " " <<
            // pos.rx
            //          << " " << pos.ry << " " << pos.rz;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 压低
            pos = posBeginRelInv;
            // pos.pos = posBeginRelInv.pos + translationInv;
            // pos.rot = newRotInv;
            // qDebug() << pos.x << " " << pos.y << " " << pos.z << " " <<
            // pos.rx
            //          << " " << pos.ry << " " << pos.rz;
            // pos.rx = endOffsetPoint.rx;
            // pos.ry = endOffsetPoint.ry;
            // pos.rz = endOffsetPoint.rz;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 反向圆弧运动
            idx = 0;
            for (; idx < posMidRelList.size() - 1; idx += 2) {
                posEnd = posMidRelList.at(posMidRelList.size() - idx - 2);
                posEnd.rot = posEnd.rot - posBeginRel.rot +
                             pointSet.beginOffsetPoint.rot;
                // posEnd.pos =
                //     posMidRelList.at(posMidRelList.size() - idx - 2).pos +
                //     translationInv;
                // posEnd.rot = newRotInv;
                posAux = posMidRelList.at(posMidRelList.size() - idx - 1);
                posAux.rot = posAux.rot - posBeginRel.rot +
                             pointSet.beginOffsetPoint.rot;
                // posAux.pos =
                //     posMidRelList.at(posMidRelList.size() - idx - 1).pos +
                //     translationInv;
                // posAux.rot = newRotInv;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
            posEnd = posEndRelInv;
            // posEnd.pos = posEndRelInv.pos + translationInv;
            // posEnd.rot = newRotInv;
            posAux = posMidRelList.at(posMidRelList.size() - idx - 1);
            posAux.rot =
                posAux.rot - posBeginRel.rot + pointSet.beginOffsetPoint.rot;
            // posAux.pos = posMidRelList.at(posMidRelList.size() - idx - 1).pos
            // +
            //              translationInv;
            // posAux.rot = newRotInv;
            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        } else { // 正向
            // 抬高
            pos = posEndRelInv;
            // pos.pos = posEndRelInv.pos + translationInv;
            // pos.rot = newRotInv;
            // pos.rx = beginOffsetPoint.rx;
            // pos.ry = beginOffsetPoint.ry;
            // pos.rz = beginOffsetPoint.rz;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 改变位姿
            // posBeginRel += beginOffset;
            // posEndRel += endOffset;
            posBeginRel.pos += midOffset.pos * 2;
            posEndRel.pos += midOffset.pos * 2;
            for (int j = 0; j < posMidRelList.size(); j++) {
                // posMidRelList[j] += midOffsetList[j];
                posMidRelList[j] += midOffset;
            }
            pos = posBeginRel;
            // pos.pos = posBeginRel.pos + translation;
            // pos.rot = newRot;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 压低
            pos = posBeginRel;
            // pos.pos = posBeginRel.pos + translation;
            // pos.rot = newRot;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 正向圆弧运动
            idx = 0;
            for (; idx < posMidRelList.size() - 1; idx += 2) {
                posEnd = posMidRelList.at(idx + 1);
                // posEnd.pos = posMidRelList.at(idx + 1).pos + translation;
                // posEnd.rot = newRot;
                posAux = posMidRelList.at(idx);
                // posAux.pos = posMidRelList.at(idx).pos + translation;
                // posAux.rot = newRot;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
            posEnd = posEndRel;
            // posEnd.pos = posEndRel.pos + translation;
            // posEnd.rot = newRot;
            posAux = posMidRelList.at(idx);
            // posAux.pos = posMidRelList.at(idx).pos + translation;
            // posAux.rot = newRot;
            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        }
    }
    // return (count + addCount) % 2 == 0 ? posEndRel : posEndRelInv;
    return posEnd;
}

Point Robot::MoveRegionArc2(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 计算单次偏移量
    int count = craft.offsetCount;
    // Position beginOffset = (beginOffsetPoint - beginPoint) / count;
    // Position endOffset = (endOffsetPoint - endPoint) / count;
    Point temp1;
    temp1.pos = pointSet.beginOffsetPoint.pos - pointSet.endPoint.pos;
    Point temp2;
    temp2.pos = pointSet.beginPoint.pos - pointSet.endPoint.pos;
    Point beginOffset;
    if (count != 0) {
        beginOffset.pos =
            (temp1.pos -
             temp2.pos * (QVector3D::dotProduct(temp1.pos, temp2.pos) /
                          QVector3D::dotProduct(temp2.pos, temp2.pos))) /
            count;
    }

    Point temp3;
    temp3.pos = pointSet.endOffsetPoint.pos - pointSet.beginPoint.pos;
    Point temp4;
    temp4.pos = pointSet.endPoint.pos - pointSet.beginPoint.pos;
    Point endOffset;
    if (count != 0) {
        endOffset.pos =
            (temp3.pos -
             temp4.pos * (QVector3D::dotProduct(temp3.pos, temp4.pos) /
                          QVector3D::dotProduct(temp4.pos, temp4.pos))) /
            count;
    }

    double lenTotal =
        (pointSet.endPoint.pos - pointSet.beginPoint.pos).length();
    QVector<Point> midOffsetList;
    for (int i = 0; i < pointSet.midPoints.size(); i++) {
        double len =
            (pointSet.midPoints.at(i).pos - pointSet.beginPoint.pos).length();
        Point midOffset;
        midOffset.pos = beginOffset.pos * (len / lenTotal) +
                        endOffset.pos * (1 - (len / lenTotal));
        midOffsetList.append(midOffset);
    }
    // qDebug() << "midOffset" << midOffset.x << " " << midOffset.y << " "
    //          << midOffset.z << " " << midOffset.rx << " " << midOffset.ry <<
    //          " "
    //          << midOffset.rz;
    // 定义空间目标位置
    // 正向起始、结束
    Point posBeginRel = pointSet.beginPoint;
    // qDebug() << "beginPoint" << beginPoint.x << " " << beginPoint.y << " "
    //          << beginPoint.z << " " << beginPoint.rx << " " << beginPoint.ry
    //          << " " << beginPoint.rz;
    // qDebug() << "posBeginRel" << posBeginRel.x << " " << posBeginRel.y << " "
    //          << posBeginRel.z << " " << posBeginRel.rx << " " <<
    //          posBeginRel.ry
    //          << " " << posBeginRel.rz;
    Point posEndRel = pointSet.endPoint;
    // qDebug() << "endPoint" << endPoint.x << " " << endPoint.y << " "
    //          << endPoint.z << " " << endPoint.rx << " " << endPoint.ry << " "
    //          << endPoint.rz;
    // qDebug() << "posEndRel" << posEndRel.x << " " << posEndRel.y << " "
    //          << posEndRel.z << " " << posEndRel.rx << " " << posEndRel.ry <<
    //          " "
    //          << posEndRel.rz;
    // 反向起始、结束
    Point posBeginRelInv = pointSet.endOffsetPoint;
    posBeginRelInv.pos =
        pointSet.endOffsetPoint.pos - endOffset.pos * (count + 1);
    // qDebug() << "endOffsetPoint" << endOffsetPoint.x << " " <<
    // endOffsetPoint.y
    //          << " " << endOffsetPoint.z << " " << endOffsetPoint.rx << " "
    //          << endOffsetPoint.ry << " " << endOffsetPoint.rz;
    // qDebug() << "posBeginRelInv" << posBeginRelInv.x << " " <<
    // posBeginRelInv.y
    //          << " " << posBeginRelInv.z << " " << posBeginRelInv.rx << " "
    //          << posBeginRelInv.ry << " " << posBeginRelInv.rz;
    Point posEndRelInv = pointSet.beginOffsetPoint;
    posEndRelInv.pos =
        pointSet.beginOffsetPoint.pos - beginOffset.pos * (count + 1);
    // qDebug() << "beginOffsetPoint" << beginOffsetPoint.x << " "
    //          << beginOffsetPoint.y << " " << beginOffsetPoint.z << " "
    //          << beginOffsetPoint.rx << " " << beginOffsetPoint.ry << " "
    //          << beginOffsetPoint.rz;
    // qDebug() << "posEndRelInv" << posEndRelInv.x << " " << posEndRelInv.y <<
    // " "
    //          << posEndRelInv.z << " " << posEndRelInv.rx << " "
    //          << posEndRelInv.ry << " " << posEndRelInv.rz;
    QVector<Point> posMidRelList;
    for (int i = 0; i < pointSet.midPoints.size(); i++) {
        Point posMidRel = pointSet.midPoints.at(i);
        posMidRelList.append(posMidRel);
    }
    int idx = 0;
    Point posAux, posEnd;
    for (; idx < posMidRelList.size() - 1; idx += 2) {
        // 正向圆弧运动
        posEnd = posMidRelList.at(idx + 1);
        // posEnd.pos = posMidRelList.at(idx + 1).pos + translation;
        // posEnd.rot = newRot;
        posAux = posMidRelList.at(idx);
        // posAux.pos = posMidRelList.at(idx).pos + translation;
        // posAux.rot = newRot;
        MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
    }
    // 正向圆弧运动
    posEnd = posEndRel;
    // posEnd.pos = posEndRel.pos + translation;
    // posEnd.rot = newRot;
    posAux = posMidRelList.at(idx);
    // posAux.pos = posMidRelList.at(idx).pos + translation;
    // posAux.rot = newRot;
    MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
    Point pos;
    for (int i = 0; i < count; ++i) {
        if (i % 2 == 0) { // 反向
            // 抬高
            pos = posEndRel;
            // pos.pos = posEndRel.pos + translation;
            // pos.rot = newRot;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            // qDebug() << pos.x << " " << pos.y << " " << pos.z << " " <<
            // pos.rx
            //          << " " << pos.ry << " " << pos.rz;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 改变位姿
            // posBeginRel += beginOffset;
            // posEndRel += endOffset;
            posBeginRelInv.pos += endOffset.pos * 2;
            posEndRelInv.pos += beginOffset.pos * 2;
            for (int j = 0; j < posMidRelList.size(); j++) {
                posMidRelList[j] += midOffsetList[j];
                // posMidRelList[j] += midOffset;
            }
            pos = posBeginRelInv;
            // pos.pos = posBeginRelInv.pos + translationInv;
            // pos.rot = newRotInv;
            // pos.rx = endOffsetPoint.rx;
            // pos.ry = endOffsetPoint.ry;
            // pos.rz = endOffsetPoint.rz;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            // qDebug() << pos.x << " " << pos.y << " " << pos.z << " " <<
            // pos.rx
            //          << " " << pos.ry << " " << pos.rz;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 压低
            pos = posBeginRelInv;
            // pos.pos = posBeginRelInv.pos + translationInv;
            // pos.rot = newRotInv;
            // qDebug() << pos.x << " " << pos.y << " " << pos.z << " " <<
            // pos.rx
            //          << " " << pos.ry << " " << pos.rz;
            // pos.rx = endOffsetPoint.rx;
            // pos.ry = endOffsetPoint.ry;
            // pos.rz = endOffsetPoint.rz;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 反向圆弧运动
            idx = 0;
            for (; idx < posMidRelList.size() - 1; idx += 2) {
                posEnd = posMidRelList.at(posMidRelList.size() - idx - 2);
                posEnd.rot = posEnd.rot - posBeginRel.rot +
                             pointSet.beginOffsetPoint.rot;
                // posEnd.pos =
                //     posMidRelList.at(posMidRelList.size() - idx - 2).pos +
                //     translationInv;
                // posEnd.rot = newRotInv;
                posAux = posMidRelList.at(posMidRelList.size() - idx - 1);
                posAux.rot = posAux.rot - posBeginRel.rot +
                             pointSet.beginOffsetPoint.rot;
                // posAux.pos =
                //     posMidRelList.at(posMidRelList.size() - idx - 1).pos +
                //     translationInv;
                // posAux.rot = newRotInv;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
            posEnd = posEndRelInv;
            // posEnd.pos = posEndRelInv.pos + translationInv;
            // posEnd.rot = newRotInv;
            posAux = posMidRelList.at(posMidRelList.size() - idx - 1);
            posAux.rot =
                posAux.rot - posBeginRel.rot + pointSet.beginOffsetPoint.rot;
            // posAux.pos = posMidRelList.at(posMidRelList.size() - idx - 1).pos
            // +
            //              translationInv;
            // posAux.rot = newRotInv;
            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        } else { // 正向
            // 抬高
            pos = posEndRelInv;
            // pos.pos = posEndRelInv.pos + translationInv;
            // pos.rot = newRotInv;
            // pos.rx = beginOffsetPoint.rx;
            // pos.ry = beginOffsetPoint.ry;
            // pos.rz = beginOffsetPoint.rz;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 改变位姿
            // posBeginRel += beginOffset;
            // posEndRel += endOffset;
            posBeginRel.pos += beginOffset.pos * 2;
            posEndRel.pos += endOffset.pos * 2;
            for (int j = 0; j < posMidRelList.size(); j++) {
                posMidRelList[j] += midOffsetList[j];
                // posMidRelList[j] += midOffset;
            }
            pos = posBeginRel;
            // pos.pos = posBeginRel.pos + translation;
            // pos.rot = newRot;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 压低
            pos = posBeginRel;
            // pos.pos = posBeginRel.pos + translation;
            // pos.rot = newRot;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 正向圆弧运动
            idx = 0;
            for (; idx < posMidRelList.size() - 1; idx += 2) {
                posEnd = posMidRelList.at(idx + 1);
                // posEnd.pos = posMidRelList.at(idx + 1).pos + translation;
                // posEnd.rot = newRot;
                posAux = posMidRelList.at(idx);
                // posAux.pos = posMidRelList.at(idx).pos + translation;
                // posAux.rot = newRot;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
            posEnd = posEndRel;
            // posEnd.pos = posEndRel.pos + translation;
            // posEnd.rot = newRot;
            posAux = posMidRelList.at(idx);
            // posAux.pos = posMidRelList.at(idx).pos + translation;
            // posAux.rot = newRot;
            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        }
    }
    // return count % 2 == 0 ? posEndRel : posEndRelInv;
    return posEnd;
}


// 打磨 扇形环面 添加更换工具逻辑
Point Robot::MoveRegionArcHorizontal(Craft &craft) {
    // 1. 基础参数定义
    double dVelocity = craft.moveSpeed;
    double dAcc = 100;
    double dRadius = craft.transitionRadius;
    int count = craft.offsetCount;

    // 圆弧点集合
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    // 偏移与弧长预计算
    double midOffset = 0.0;
    QVector<QVector3D> offsetList;
    double baseArcLength = 0.0; // 存储单层基础弧长

    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D p1 = posListUp.at(i - 1).pos;
        QVector3D p2 = posListUp.at(i).pos;
        QVector3D p3 = posListUp.at(i + 1).pos;

        QVector3D center = Point::calculateCircumcenter(p1, p2, p3);

        // 计算基础弧长 (L = Radius * Angle)
        double r = (p1 - center).length();
        QVector3D v1 = (p1 - center).normalized();
        QVector3D v2 = (p2 - center).normalized();
        QVector3D v3 = (p3 - center).normalized();
        double angle = qAcos(QVector3D::dotProduct(v1, v2)) + qAcos(QVector3D::dotProduct(v2, v3));
        baseArcLength += (r * angle);

        if (i == 1 && count > 0) {
            midOffset = ((p1 - center).length() - (pointSet.beginOffsetPoint.pos - center).length()) / count;
        }
        offsetList.append((center - p1).normalized() * midOffset);
        offsetList.append((center - p2).normalized() * midOffset);
        offsetList.append((center - p3).normalized() * midOffset);
    }

    Point pos, posAux, posEnd;

    // 2. 开始多层往复运动
    for (int i = 0; i < count + 1; ++i) {
        if (i % 2 == 0) { // 正向运动层
            if (i > 0) {
                // 抬高到上一层断点上方
                pos.pos = posListUp.constFirst().pos + offsetList.constFirst() * (i - 1) + translationInv;
                pos.rot = newRotInv;
                pos = pos.PosRelByTool(defaultDirection, defaultOffset);
                MoveL(pos, dVelocity, dAcc, dRadius);
                // 变换位姿移动到当前层起点上方
                pos.pos = posListUp.constFirst().pos + offsetList.constFirst() * i + translation;
                pos.rot = newRot;
                pos = pos.PosRelByTool(defaultDirection, defaultOffset);
                MoveL(pos, dVelocity, dAcc, dRadius);
                // 压低进入工件
                pos.pos = posListUp.constFirst().pos + offsetList.constFirst() * i + translation;
                pos.rot = newRot;
                MoveL(pos, dVelocity, dAcc, dRadius);
            }
            // 圆弧运动段
            for (int j = 1; j < posListUp.size() - 1; j += 2) {
                posAux.pos = posListUp.at(j).pos + offsetList.at(j) * i + translation;
                posAux.rot = newRot;
                posEnd.pos = posListUp.at(j + 1).pos + offsetList.at(j + 1) * i + translation;
                posEnd.rot = newRot;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
        } else { // 反向运动层
            // 抬高
            pos.pos = posListUp.constLast().pos + offsetList.constLast() * (i - 1) + translation;
            pos.rot = newRot;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 变换位姿
            pos.pos = posListUp.constLast().pos + offsetList.constLast() * i + translationInv;
            pos.rot = newRotInv;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 压低
            pos.pos = posListUp.constLast().pos + offsetList.constLast() * i + translationInv;
            pos.rot = newRotInv;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 反向圆弧运动
            for (int j = posListUp.size() - 2; j > 0; j -= 2) {
                posAux.pos = posListUp.at(j).pos + offsetList.at(j) * i + translationInv;
                posAux.rot = newRotInv;
                posEnd.pos = posListUp.at(j - 1).pos + offsetList.at(j - 1) * i + translationInv;
                posEnd.rot = newRotInv;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
        }

        // 等待当前层轨迹结束
        while (IsRobotMoved()) { QThread::msleep(50); }

        // --- 3. 统计长度与换刀逻辑判断 ---

        // 累加已完成的弧长
        this->toolConfig.totalPolishLength += baseArcLength;

        // 判断是否超过阈值并触发换刀
        if (this->toolConfig.totalPolishLength >= this->toolConfig.toolChangeThreshold && i < count) {
            qDebug() << "Reached threshold, total length:" << this->toolConfig.totalPolishLength;

            // A. 抬起动作：在当前层结束点 posEnd 基础上，沿工具 Z 轴向上抬起 100mm
            Point liftPoint = posEnd.PosRelByTool(defaultDirection, -100.0);
            MoveL(liftPoint, dVelocity, 2000, 0); // 快速移动到抬起点
            while (IsRobotMoved()) { QThread::msleep(50); }

            // B. 执行换刀逻辑
            // 您的 ToolChange 会记录此 liftPoint 并在结束后自动 MoveL 回来
            ToolChange(craft);

            qDebug() << "ToolChange finished. Robot returned to lift point. Continuing...";
        }

        // 如果在运行过程中点击停止
        if (isStop) return posEnd;
    }

    return posEnd;
}


Point Robot::MoveRegionArcVertical(const Craft &craft) {
    // 圆弧上界
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    // 偏移次数
    int count = craft.offsetCount;
    // 计算上圆弧组中圆弧圆心、半径和弧长
    QVector<QVector3D> centerListUp;
    QVector<double> radiusListUp, lengthListUp;
    double totalArcLengthUp = 0.0;
    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListUp.at(i - 1).pos, posListUp.at(i).pos,
            posListUp.at(i + 1).pos);
        centerListUp.append(center);
        double radius = (posListUp.at(i).pos - center).length();
        radiusListUp.append(radius);
        QVector3D OA = posListUp.at(i - 1).pos - center;
        QVector3D OM = posListUp.at(i).pos - center;
        QVector3D OB = posListUp.at(i + 1).pos - center;
        // 适用OA、OB夹角大于180°的情况
        double length = (qAcos(QVector3D::dotProduct(OA, OM) /
                               (OA.length() * OM.length())) +
                         qAcos(QVector3D::dotProduct(OM, OB) /
                               (OM.length() * OB.length()))) *
                        radius;
        lengthListUp.append(length);
        totalArcLengthUp += length;
    }
    // 圆弧上界到下界的偏移距离
    double midOffset =
        radiusListUp.constFirst() -
        (pointSet.beginOffsetPoint.pos - centerListUp.constFirst()).length();
    // 最终圆弧上界与下界
    QVector<QVector3D> finalPosListUp, finalPosListDown;
    if (count > 0) {
        double unitArcLengthUp = totalArcLengthUp / count;
        double arcLengthUp = 0.0;
        for (int i = 0; i < centerListUp.size(); ++i) {
            while (arcLengthUp <= lengthListUp.at(i)) {
                QVector3D axis = QVector3D::crossProduct(
                    posListUp.at(2 * i).pos - centerListUp.at(i),
                    posListUp.at(2 * i + 1).pos - centerListUp.at(i));
                QMatrix3x3 R = Point::toRotationMatrix(
                    axis, qRadiansToDegrees(arcLengthUp / radiusListUp.at(i)));
                QVector3D trans = posListUp.at(2 * i).pos - centerListUp.at(i);
                QVector3D newTrans =
                    QVector3D(R(0, 0) * trans.x() + R(0, 1) * trans.y() +
                                  R(0, 2) * trans.z(),
                              R(1, 0) * trans.x() + R(1, 1) * trans.y() +
                                  R(1, 2) * trans.z(),
                              R(2, 0) * trans.x() + R(2, 1) * trans.y() +
                                  R(2, 2) * trans.z());
                finalPosListUp.append(centerListUp.at(i) + newTrans);
                finalPosListDown.append(centerListUp.at(i) +
                                        newTrans.normalized() *
                                            (radiusListUp.at(i) - midOffset));
                arcLengthUp += unitArcLengthUp;
            }
            arcLengthUp -= lengthListUp.at(i);
        }
    } else {
        finalPosListUp.append(pointSet.beginPoint.pos);
        finalPosListDown.append(pointSet.beginOffsetPoint.pos);
    }

    Q_ASSERT(finalPosListUp.size() == finalPosListDown.size());
    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    int raiseCount = craft.raiseCount;
    int floatCount = craft.floatCount;
    count = finalPosListUp.size() - 1;
    int interval =
        raiseCount < 0 ? count : qCeil(count / (raiseCount + 1)); // 抬起间距

    double radius = craft.discRadius;
    double angle = craft.grindAngle;
    QVector3D rotation = pointSet.beginPoint.rot;
    QVector3D moveDirection;
    // 定义空间目标位置
    Point point;
    // 移到起始辅助点
    moveDirection = finalPosListDown.constFirst() - finalPosListUp.constFirst();
    // 获取新的姿态
    newRot = Point::getNewRotation(rotation, moveDirection, angle);
    // 获取新姿态需要的平移量
    translation = Point::getTranslation(rotation, moveDirection, radius, angle);
    point.pos = finalPosListDown.constFirst() + translation;
    point.rot = newRot;
    point = point.PosRelByTool(defaultDirection, defaultOffset);
    MoveL(point, dVelocity, dAcc, dRadius);
    // 移到起始点
    dVelocity = craft.cutinSpeed;
    for (int i = 0; i < finalPosListUp.size(); ++i) {
        moveDirection = finalPosListDown.at(i) - finalPosListUp.at(i);
        // 获取新的姿态
        newRot = Point::getNewRotation(rotation, moveDirection, angle);
        // 获取新姿态需要的平移量
        translation =
            Point::getTranslation(rotation, moveDirection, radius, angle);

        point.pos = finalPosListDown.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);

        if (interval > 0 && i != 0 && i != count && i % interval == 0) {
            dVelocity = craft.cutinSpeed;
            dAcc = 2000;
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(point, dVelocity, dAcc, dRadius); // 抬起

            for (int j = 0; j < floatCount; ++j) { // 浮动
                point = point.PosRelByTool(defaultDirection, 10);
                MoveL(point, dVelocity, dAcc, dRadius); // 落下

                point = point.PosRelByTool(defaultDirection, -10);
                MoveL(point, dVelocity, dAcc, dRadius); // 抬起
            }

            point = point.PosRelByTool(defaultDirection, -defaultOffset);
            MoveL(point, dVelocity, dAcc, dRadius); // 落下
        }

        dVelocity = craft.moveSpeed;
        dAcc = 100;
        point.pos = finalPosListUp.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);
    }
    point.pos = finalPosListDown.constLast() + translation;
    point.rot = newRot;
    MoveL(point, dVelocity, dAcc, dRadius);

    return point;
}

Point Robot::MoveRegionArcVerticalRepeat(const Craft &craft) {
    // 圆弧上界
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    // 偏移次数
    int count = craft.offsetCount;
    // 计算上圆弧组中圆弧圆心、半径和弧长
    QVector<QVector3D> centerListUp;
    QVector<double> radiusListUp, lengthListUp;
    double totalArcLengthUp = 0.0;
    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListUp.at(i - 1).pos, posListUp.at(i).pos,
            posListUp.at(i + 1).pos);
        centerListUp.append(center);
        double radius = (posListUp.at(i).pos - center).length();
        radiusListUp.append(radius);
        QVector3D OA = posListUp.at(i - 1).pos - center;
        QVector3D OM = posListUp.at(i).pos - center;
        QVector3D OB = posListUp.at(i + 1).pos - center;
        // 适用OA、OB夹角大于180°的情况
        double length = (qAcos(QVector3D::dotProduct(OA, OM) /
                               (OA.length() * OM.length())) +
                         qAcos(QVector3D::dotProduct(OM, OB) /
                               (OM.length() * OB.length()))) *
                        radius;
        lengthListUp.append(length);
        totalArcLengthUp += length;
    }
    // 圆弧上界到下界的偏移距离
    double midOffset =
        radiusListUp.constFirst() -
        (pointSet.beginOffsetPoint.pos - centerListUp.constFirst()).length();
    // 最终圆弧上界与下界
    QVector<QVector3D> finalPosListUp, finalPosListDown;
    if (count > 0) {
        double unitArcLengthUp = totalArcLengthUp / count;
        double arcLengthUp = 0.0;
        for (int i = 0; i < centerListUp.size(); ++i) {
            while (arcLengthUp <= lengthListUp.at(i)) {
                QVector3D axis = QVector3D::crossProduct(
                    posListUp.at(2 * i).pos - centerListUp.at(i),
                    posListUp.at(2 * i + 1).pos - centerListUp.at(i));
                QMatrix3x3 R = Point::toRotationMatrix(
                    axis, qRadiansToDegrees(arcLengthUp / radiusListUp.at(i)));
                QVector3D trans = posListUp.at(2 * i).pos - centerListUp.at(i);
                QVector3D newTrans =
                    QVector3D(R(0, 0) * trans.x() + R(0, 1) * trans.y() +
                                  R(0, 2) * trans.z(),
                              R(1, 0) * trans.x() + R(1, 1) * trans.y() +
                                  R(1, 2) * trans.z(),
                              R(2, 0) * trans.x() + R(2, 1) * trans.y() +
                                  R(2, 2) * trans.z());
                finalPosListUp.append(centerListUp.at(i) + newTrans);
                finalPosListDown.append(centerListUp.at(i) +
                                        newTrans.normalized() *
                                            (radiusListUp.at(i) - midOffset));
                arcLengthUp += unitArcLengthUp;
            }
            arcLengthUp -= lengthListUp.at(i);
        }
    } else {
        finalPosListUp.append(pointSet.beginPoint.pos);
        finalPosListDown.append(pointSet.beginOffsetPoint.pos);
    }

    Q_ASSERT(finalPosListUp.size() == finalPosListDown.size());
    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    double radius = craft.discRadius;
    double angle = craft.grindAngle;
    QVector3D rotation = pointSet.beginPoint.rot;
    QVector3D moveDirection;
    // 定义空间目标位置
    Point point;
    // 移到起始辅助点
    moveDirection = finalPosListDown.constFirst() - finalPosListUp.constFirst();
    // 获取新的姿态
    newRot = Point::getNewRotation(rotation, moveDirection, angle);
    // 获取新姿态需要的平移量
    translation = Point::getTranslation(rotation, moveDirection, radius, angle);
    point.pos = finalPosListDown.constFirst() + translation;
    point.rot = newRot;
    point = point.PosRelByTool(defaultDirection, defaultOffset);
    MoveL(point, dVelocity, dAcc, dRadius);
    // 移到起始点
    dVelocity = craft.cutinSpeed;
    for (int i = 0; i < finalPosListUp.size(); ++i) {
        moveDirection = finalPosListDown.at(i) - finalPosListUp.at(i);
        // 获取新的姿态
        newRot = Point::getNewRotation(rotation, moveDirection, angle);
        // 获取新姿态需要的平移量
        translation =
            Point::getTranslation(rotation, moveDirection, radius, angle);

        point.pos = finalPosListDown.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);

        dVelocity = craft.moveSpeed;
        dAcc = 100;
        point.pos = finalPosListUp.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);
        point.pos = finalPosListDown.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);
    }

    return point;
}


Point Robot::MoveCylinderHorizontal(const Craft &craft, bool isConvex) {
    // 打磨片半径
    double discRadius = craft.discRadius;
    // 打磨角度
    double grindAngle = craft.grindAngle;
    // 计算姿态和平移量
    QVector3D rotation = pointSet.beginPoint.rot;
    QVector3D moveDirection = pointSet.endPoint.pos - pointSet.beginPoint.pos;
    // 获取新的姿态
    newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
    // 获取新姿态需要的平移量
    translation =
        Point::getTranslation(rotation, moveDirection, discRadius, grindAngle);

    // 偏移次数
    int count = craft.offsetCount;
    // 单次偏移距离
    QVector3D posOffset;
    if (count > 0) {
        posOffset =
            (pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos) / count;
    }

    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    Point pos;
    pos.pos = pointSet.endPoint.pos + translation;
    pos.rot = newRot;
    pos = pos.PosRelByTool(defaultDirection, defaultOffset);
    MoveL(pos, dVelocity, dAcc, dRadius);

    dVelocity = craft.cutinSpeed;
    pos.pos = pointSet.endPoint.pos + translation;
    pos.rot = newRot;
    MoveL(pos, dVelocity, dAcc, dRadius);

    dVelocity = craft.moveSpeed;
    dAcc = 100;
    for (int i = 0; i < count + 1; ++i) {
        pos.pos = pointSet.beginPoint.pos + posOffset * i + translation;
        pos.rot = newRot;
        MoveL(pos, dVelocity, dAcc, dRadius);
        if (i != count) {
            pos.pos = pointSet.endPoint.pos + posOffset * (i + 1) + translation;
            pos.rot = newRot;
            MoveL(pos, dVelocity, dAcc, dRadius);
        } else {
            pos.pos = pointSet.endPoint.pos + posOffset * i + translation;
            pos.rot = newRot;
            MoveL(pos, dVelocity, dAcc, dRadius);
        }
    }

    return pos;
}

// ============================================================================
// 改进版本: MoveCylinderVertical - 基于示教点位姿态的柱面打磨
// ============================================================================
//
// 【改进说明】
// 1. 原版本问题:
//    - 通过纯几何计算姿态,使用 getNormalRotation(normal, axis) 生成理想姿态
//    - 假设工具坐标系严格对齐(Y轴平行柱体轴线,X轴向左)
//    - 实际上工具安装存在偏差,导致计算姿态与实际需求不匹配
//
// 2. 改进方案:
//    - 以示教点(beginPoint)的姿态为参考基准
//    - 计算示教点姿态对应的工具坐标系
//    - 在后续点位上,基于示教点的工具坐标系进行增量调整
//    - 保持与示教点相同的工具安装偏差特性
//
// 3. 核心改进:
//    - 新增函数: getAdjustedRotation() - 基于参考姿态的增量调整
//    - 替换原来的: Point::getNormalRotation(normal, axis)
//    - 保留打磨角度补偿逻辑
// ============================================================================

Point Robot::MoveCylinderVertical(Craft &craft, bool isConvex) {
    // ========== 1. 基础参数定义 ==========
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    // ========== 2. 计算上圆弧组中圆弧圆心、半径和弧长 ==========
    QVector<QVector3D> centerListUp;
    QVector<double> radiusListUp, lengthListUp;
    double totalArcLengthUp = 0.0;

    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListUp.at(i - 1).pos, posListUp.at(i).pos,
            posListUp.at(i + 1).pos);
        centerListUp.append(center);

        double radius = (posListUp.at(i).pos - center).length();
        radiusListUp.append(radius);

        QVector3D OA = posListUp.at(i - 1).pos - center;
        QVector3D OM = posListUp.at(i).pos - center;
        QVector3D OB = posListUp.at(i + 1).pos - center;

        double length = (qAcos(QVector3D::dotProduct(OA, OM) /
                               (OA.length() * OM.length())) +
                         qAcos(QVector3D::dotProduct(OM, OB) /
                               (OM.length() * OB.length()))) *
                        radius;
        lengthListUp.append(length);
        totalArcLengthUp += length;
    }

    // ========== 3. 圆弧上界到下界的偏移距离 ==========
    QVector3D posOffset =
        pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos;

    // ========== 4. 【关键改进】提取示教点的参考坐标系 ==========
    // 4.1 获取示教点(beginPoint)的姿态旋转矩阵
    QVector3D referenceRotation = pointSet.beginPoint.rot;
    QMatrix3x3 R_reference = Point::toRotationMatrix(referenceRotation);

    // 4.2 提取示教点工具坐标系的三个轴向量(实际安装状态)
    // X轴: R的第一列
    QVector3D referenceX(R_reference(0, 0), R_reference(1, 0), R_reference(2, 0));
    // Y轴: R的第二列
    QVector3D referenceY(R_reference(0, 1), R_reference(1, 1), R_reference(2, 1));
    // Z轴: R的第三列 (工具方向)
    QVector3D referenceZ(R_reference(0, 2), R_reference(1, 2), R_reference(2, 2));

    // 4.3 计算示教点处的几何法向量(用于确定调整方向)
    QVector3D trans_reference = posListUp.at(0).pos - centerListUp.at(0);
    QVector3D aux_reference = posOffset.normalized();
    QVector3D axis_reference =
        QVector3D::crossProduct(
            posListUp.at(0).pos - centerListUp.at(0),
            posListUp.at(1).pos - centerListUp.at(0))
            .normalized();

    QVector3D normal_reference =
        QVector3D::crossProduct(
            QVector3D::crossProduct(aux_reference, -trans_reference),
            aux_reference)
            .normalized();

    if (!isConvex) {
        normal_reference = -normal_reference;
        axis_reference = -axis_reference;
    }
    if (craft.isMirror) {
        axis_reference = -axis_reference;
    }

    // ========== 5. 生成最终圆弧上界与下界点集 ==========
    double discRadius = craft.discRadius;
    double grindAngle = craft.grindAngle;
    int count = craft.offsetCount;

    QVector<QVector3D> finalPosListUp, finalPosListDown;
    QVector<QVector3D> newRotList, translationList;

    if (count > 0) {
        double unitArcLengthUp = totalArcLengthUp / count / 2;
        double arcLengthUp = 0.0;

        for (int i = 0; i < centerListUp.size(); ++i) {
            while (arcLengthUp <= lengthListUp.at(i)) {
                // 5.1 计算当前点的几何参数
                QVector3D axis =
                    QVector3D::crossProduct(
                        posListUp.at(2 * i).pos - centerListUp.at(i),
                        posListUp.at(2 * i + 1).pos - centerListUp.at(i))
                        .normalized();

                QMatrix3x3 R = Point::toRotationMatrix(
                    axis, qRadiansToDegrees(arcLengthUp / radiusListUp.at(i)));

                QVector3D trans = posListUp.at(2 * i).pos - centerListUp.at(i);
                QVector3D newTrans =
                    QVector3D(R(0, 0) * trans.x() + R(0, 1) * trans.y() +
                                  R(0, 2) * trans.z(),
                              R(1, 0) * trans.x() + R(1, 1) * trans.y() +
                                  R(1, 2) * trans.z(),
                              R(2, 0) * trans.x() + R(2, 1) * trans.y() +
                                  R(2, 2) * trans.z());

                finalPosListUp.append(centerListUp.at(i) + newTrans);
                finalPosListDown.append(centerListUp.at(i) + newTrans + posOffset);

                // 5.2 计算当前点的几何法向量
                QVector3D aux = posOffset.normalized();
                QVector3D normal =
                    QVector3D::crossProduct(
                        QVector3D::crossProduct(aux, -newTrans), aux)
                        .normalized();

                if (!isConvex) {
                    normal = -normal;
                    axis = -axis;
                }
                if (craft.isMirror) {
                    axis = -axis;
                }

                // ========== 【核心改进】基于示教点姿态计算调整后的姿态 ==========
                // 5.3 计算当前法向与参考法向之间的旋转
                // 方法: 使用旋转轴和旋转角度来调整参考坐标系

                // 计算从参考法向到当前法向的旋转轴
                QVector3D rotationAxis = QVector3D::crossProduct(normal_reference, normal);
                float rotationAxisLength = rotationAxis.length();

                QVector3D adjustedRotation;

                if (rotationAxisLength > 1e-6) {
                    // 存在旋转
                    rotationAxis.normalize();

                    // 计算旋转角度
                    float dotProduct = QVector3D::dotProduct(normal_reference, normal);
                    dotProduct = qBound(-1.0f, dotProduct, 1.0f); // 限制范围
                    float rotationAngle = qRadiansToDegrees(qAcos(dotProduct));

                    // 构建从参考姿态到当前姿态的增量旋转矩阵
                    QMatrix3x3 R_adjustment = Point::toRotationMatrix(rotationAxis, rotationAngle);

                    // 应用增量旋转到参考姿态
                    QMatrix3x3 R_adjusted = R_adjustment * R_reference;

                    // 转换回欧拉角
                    adjustedRotation = Point::toEulerAngles(R_adjusted);
                } else {
                    // 法向基本相同,直接使用参考姿态
                    adjustedRotation = referenceRotation;
                }

                // 5.4 应用打磨角度补偿(在调整后的姿态基础上)
                QVector3D moveDirection = posOffset.normalized();
                newRot = Point::getNewRotation(adjustedRotation, moveDirection, grindAngle);
                newRotList.append(newRot);

                // 5.5 计算姿态补偿的平移量
                translation = Point::getTranslation(adjustedRotation, moveDirection,
                                                    discRadius, grindAngle);
                translationList.append(translation);

                arcLengthUp += unitArcLengthUp;
            }
            arcLengthUp -= lengthListUp.at(i);
        }
    } else {
        // 处理不偏移的特殊情况
        finalPosListUp.append(pointSet.beginPoint.pos);
        finalPosListDown.append(pointSet.beginOffsetPoint.pos);

        // 直接使用示教点姿态
        QVector3D moveDirection = posOffset.normalized();
        newRot = Point::getNewRotation(referenceRotation, moveDirection, grindAngle);
        newRotList.append(newRot);

        translation = Point::getTranslation(referenceRotation, moveDirection,
                                            discRadius, grindAngle);
        translationList.append(translation);
    }

    // 安全校验
    Q_ASSERT(finalPosListUp.size() == finalPosListDown.size());
    Q_ASSERT(finalPosListUp.size() == newRotList.size());
    Q_ASSERT(finalPosListUp.size() == translationList.size());

    // ========== 6. 定义运动参数 ==========
    double dVelocity = defaultVelocity;
    double dAcc = 2000;
    double dRadius = craft.transitionRadius;

    Point pos, posAux, posEnd;

    // ========== 7. 移动到下界轨迹第一个点(带姿态补偿) ==========
    pos.pos = finalPosListDown.constFirst() + translationList.constFirst();
    pos.rot = newRotList.constFirst();
    pos = pos.PosRelByTool(defaultDirection, defaultOffset);
    MoveL(pos, dVelocity, dAcc, dRadius);

    dVelocity = craft.cutinSpeed;
    pos.pos = finalPosListDown.constFirst() + translationList.constFirst();
    pos.rot = newRotList.constFirst();
    MoveL(pos, dVelocity, dAcc, dRadius);

    dVelocity = craft.moveSpeed;
    dAcc = 100;

    // ========== 8. 主打磨循环(含换刀逻辑) ==========
    for (int i = 0; i < count + 1; ++i) {
        // 8.1 移动到上界点
        pos.pos = finalPosListUp.at(2 * i) + translationList.at(2 * i);
        pos.rot = newRotList.at(2 * i);
        MoveL(pos, dVelocity, dAcc, dRadius);

        if (i != count) {
            // 8.2 圆弧运动到下一层下界点
            posAux.pos = (finalPosListUp.at(2 * i + 1) +
                          finalPosListDown.at(2 * i + 1)) / 2 +
                         translationList.at(2 * i + 1);
            posAux.rot = newRotList.at(2 * i + 1);

            posEnd.pos = finalPosListDown.at(2 * i + 2) + translationList.at(2 * i + 2);
            posEnd.rot = newRotList.at(2 * i + 2);

            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        } else {
            // 8.3 最后一层直接下压到下界
            pos.pos = finalPosListDown.at(2 * i) + translationList.at(2 * i);
            pos.rot = newRotList.at(2 * i);
            MoveL(pos, dVelocity, dAcc, dRadius);
        }

        // ========== 8.4 等待当前层轨迹结束 ==========
        while (IsRobotMoved()) {
            QThread::msleep(50);
        }

        // ========== 8.5 统计长度与换刀逻辑判断 ==========
        double currentLayerLength = 0.0;
        if (i != count) {
            currentLayerLength = totalArcLengthUp / (count + 1);
        } else {
            currentLayerLength = (finalPosListUp.at(2 * i).distanceToPoint(
                finalPosListDown.at(2 * i)));
        }

        this->toolConfig.totalPolishLength += currentLayerLength;

        if (this->toolConfig.totalPolishLength >= this->toolConfig.toolChangeThreshold && i < count) {
            qDebug() << "Reached threshold, total length:" << this->toolConfig.totalPolishLength;

            Point liftPoint;
            if (i != count) {
                liftPoint = posEnd;
            } else {
                liftPoint = pos;
            }
            liftPoint = liftPoint.PosRelByTool(defaultDirection, -100.0);
            MoveL(liftPoint, dVelocity, 2000, 0);
            while (IsRobotMoved()) { QThread::msleep(50); }

            ToolChange(craft);

            qDebug() << "MoveCylinderVertical: ToolChange finished. Continuing...";
        }

        if (isStop) {
            if (i != count) {
                return posEnd;
            } else {
                return pos;
            }
        }
    }

    // ========== 9. 返回最终位置 ==========
    return pos;
}

// 打磨柱面侧面 添加换刀逻辑 26.02.10版本
// Point Robot::MoveCylinderVertical(Craft &craft, bool isConvex) {
//     // ========== 1. 基础参数定义 ==========
//     // 圆弧上界
//     QVector<Point> posListUp;
//     posListUp.append(pointSet.beginPoint);
//     posListUp.append(pointSet.midPoints);
//     posListUp.append(pointSet.endPoint);

//     // ========== 2. 计算上圆弧组中圆弧圆心、半径和弧长 ==========
//     QVector<QVector3D> centerListUp;
//     QVector<double> radiusListUp, lengthListUp;
//     double totalArcLengthUp = 0.0;

//     // 每3个点定一段圆弧, i 指向每段的中间点
//     for (int i = 1; i < posListUp.size() - 1; i += 2) {
//         // 计算当前段 ABC 的外心坐标
//         QVector3D center = Point::calculateCircumcenter(
//             posListUp.at(i - 1).pos, posListUp.at(i).pos,
//             posListUp.at(i + 1).pos);
//         centerListUp.append(center);

//         // 计算外心圆到各个点的距离作为半径
//         double radius = (posListUp.at(i).pos - center).length();
//         radiusListUp.append(radius);

//         // 构建圆心指向各样点的向量,用于计算圆心角
//         QVector3D OA = posListUp.at(i - 1).pos - center;
//         QVector3D OM = posListUp.at(i).pos - center;
//         QVector3D OB = posListUp.at(i + 1).pos - center;

//         // 适用OA、OB夹角大于180°的情况
//         // 弧长 L = (∠AOM + ∠MOB) * Radius
//         double length = (qAcos(QVector3D::dotProduct(OA, OM) /
//                                (OA.length() * OM.length())) +
//                          qAcos(QVector3D::dotProduct(OM, OB) /
//                                (OM.length() * OB.length()))) *
//                         radius;
//         lengthListUp.append(length);
//         totalArcLengthUp += length;
//     }

//     // ========== 3. 圆弧上界到下界的偏移距离 ==========
//     // 计算上界点到下界点的固定偏移向量,用于生成平行的加工面
//     QVector3D posOffset =
//         pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos;

//     // ========== 4. 生成最终圆弧上界与下界点集 ==========
//     double discRadius = craft.discRadius;
//     double grindAngle = craft.grindAngle;
//     int count = craft.offsetCount; // 偏移次数

//     QVector<QVector3D> finalPosListUp, finalPosListDown;
//     QVector<QVector3D> newRotList, translationList;

//     if (count > 0) {
//         // 根据偏移次数计算每个插补点的单位弧长步长(除以2通常是为了MoveC的中间点储备)
//         double unitArcLengthUp = totalArcLengthUp / count / 2;
//         double arcLengthUp = 0.0;

//         for (int i = 0; i < centerListUp.size(); ++i) {
//             // 在当前圆弧段长度范围内循环进行步进插值
//             while (arcLengthUp <= lengthListUp.at(i)) {
//                 // [旋转轴提取] 计算起点向量与中间点向量的叉乘,得到圆弧平面的法向量轴 axis
//                 QVector3D axis =
//                     QVector3D::crossProduct(
//                         posListUp.at(2 * i).pos - centerListUp.at(i),
//                         posListUp.at(2 * i + 1).pos - centerListUp.at(i))
//                         .normalized();

//                 // [Rodrigues旋转] 将当前走过的弧长转换为对应的角度,生成3D旋转矩阵 R
//                 QMatrix3x3 R = Point::toRotationMatrix(
//                     axis, qRadiansToDegrees(arcLengthUp / radiusListUp.at(i)));

//                 // [坐标旋转变换] 将半径向量 trans 绕轴 axis 旋转R角度,得到当前插值点位置
//                 QVector3D trans = posListUp.at(2 * i).pos - centerListUp.at(i);
//                 QVector3D newTrans =
//                     QVector3D(R(0, 0) * trans.x() + R(0, 1) * trans.y() +
//                                   R(0, 2) * trans.z(),
//                               R(1, 0) * trans.x() + R(1, 1) * trans.y() +
//                                   R(1, 2) * trans.z(),
//                               R(2, 0) * trans.x() + R(2, 1) * trans.y() +
//                                   R(2, 2) * trans.z());

//                 finalPosListUp.append(centerListUp.at(i) + newTrans);
//                 finalPosListDown.append(centerListUp.at(i) + newTrans + posOffset);

//                 // ========== 计算姿态和对应偏移 ==========
//                 QVector3D aux = posOffset.normalized(); // 表面偏移方向参考

//                 // [表面法线推导] 通过双叉乘确定当前加工点的局部法向方向 normal
//                 QVector3D normal =
//                     QVector3D::crossProduct(
//                         QVector3D::crossProduct(aux, -newTrans), aux)
//                         .normalized();

//                 // 根据工件的凹凸属性及镜像设置调整法向和旋转轴方向
//                 if (!isConvex) {
//                     normal = -normal;
//                     axis = -axis;
//                 }
//                 if (craft.isMirror) {
//                     axis = -axis;
//                 }

//                 // [姿态映射] 将几何法向转化为机器人的 Euler 角/四元数姿态
//                 QVector3D rotation = Point::getNormalRotation(normal, axis);
//                 QVector3D moveDirection = posOffset.normalized();

//                 // 获取新的姿态
//                 newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
//                 newRotList.append(newRot);

//                 // 获取新姿态需要的平移量
//                 translation = Point::getTranslation(rotation, moveDirection,
//                                                     discRadius, grindAngle);
//                 translationList.append(translation);

//                 arcLengthUp += unitArcLengthUp;
//             }
//             arcLengthUp -= lengthListUp.at(i);
//         }
//     } else {
//         // 处理不偏移的特殊情况(仅执行起点逻辑)
//         finalPosListUp.append(pointSet.beginPoint.pos);
//         finalPosListDown.append(pointSet.beginOffsetPoint.pos);

//         // 计算姿态和对应偏移
//         QVector3D axis =
//             QVector3D::crossProduct(posListUp.at(0).pos - centerListUp.at(0),
//                                     posListUp.at(1).pos - centerListUp.at(0))
//                 .normalized();
//         QVector3D trans = posListUp.at(0).pos - centerListUp.at(0);
//         QVector3D aux = posOffset.normalized();
//         QVector3D normal =
//             QVector3D::crossProduct(QVector3D::crossProduct(aux, -trans), aux)
//                 .normalized();

//         if (!isConvex) {
//             normal = -normal;
//             axis = -axis;
//         }
//         if (craft.isMirror) {
//             axis = -axis;
//         }

//         QVector3D rotation = Point::getNormalRotation(normal, axis);
//         QVector3D moveDirection = posOffset.normalized();

//         // 获取新的姿态
//         newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
//         newRotList.append(newRot);

//         // 获取新姿态需要的平移量
//         translation = Point::getTranslation(rotation, moveDirection, discRadius,
//                                             grindAngle);
//         translationList.append(translation);
//     }

//     // 安全校验
//     Q_ASSERT(finalPosListUp.size() == finalPosListDown.size());
//     Q_ASSERT(finalPosListUp.size() == newRotList.size());
//     Q_ASSERT(finalPosListUp.size() == translationList.size());

//     // ========== 5. 定义运动参数 ==========
//     double dVelocity = defaultVelocity;
//     double dAcc = 2000;
//     double dRadius = craft.transitionRadius;

//     // 定义空间目标位置
//     Point pos, posAux, posEnd;

//     // ========== 6. 移动到下界轨迹第一个点(带姿态补偿) ==========
//     pos.pos = finalPosListDown.constFirst() + translationList.constFirst();
//     pos.rot = newRotList.constFirst();
//     pos = pos.PosRelByTool(defaultDirection, defaultOffset);
//     MoveL(pos, dVelocity, dAcc, dRadius);

//     dVelocity = craft.cutinSpeed;
//     pos.pos = finalPosListDown.constFirst() + translationList.constFirst();
//     pos.rot = newRotList.constFirst();
//     MoveL(pos, dVelocity, dAcc, dRadius);

//     dVelocity = craft.moveSpeed;
//     dAcc = 100;

//     // ========== 7. 主打磨循环(含换刀逻辑) ==========
//     for (int i = 0; i < count + 1; ++i) {
//         // 7.1 移动到上界点
//         pos.pos = finalPosListUp.at(2 * i) + translationList.at(2 * i);
//         pos.rot = newRotList.at(2 * i);
//         MoveL(pos, dVelocity, dAcc, dRadius);

//         if (i != count) {
//             // 7.2 圆弧运动到下一层下界点
//             posAux.pos = (finalPosListUp.at(2 * i + 1) +
//                           finalPosListDown.at(2 * i + 1)) / 2 +
//                          translationList.at(2 * i + 1);
//             posAux.rot = newRotList.at(2 * i + 1);

//             posEnd.pos = finalPosListDown.at(2 * i + 2) + translationList.at(2 * i + 2);
//             posEnd.rot = newRotList.at(2 * i + 2);

//             MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
//         } else {
//             // 7.3 最后一层直接下压到下界
//             pos.pos = finalPosListDown.at(2 * i) + translationList.at(2 * i);
//             pos.rot = newRotList.at(2 * i);
//             MoveL(pos, dVelocity, dAcc, dRadius);
//         }

//         // ========== 7.4 等待当前层轨迹结束 ==========
//         while (IsRobotMoved()) {
//             QThread::msleep(50);
//         }

//         // ========== 7.5 统计长度与换刀逻辑判断 ==========
//         // 计算当前层完成的弧长(基于单层弧长)
//         double currentLayerLength = 0.0;
//         if (i != count) {
//             // 非最后一层: 计算从上界到下界的完整弧长
//             // 这里简化为使用 totalArcLengthUp / (count + 1) 作为单层估算
//             currentLayerLength = totalArcLengthUp / (count + 1);
//         } else {
//             // 最后一层可能只是直线下压,长度较小
//             currentLayerLength = (finalPosListUp.at(2 * i).distanceToPoint(
//                                       finalPosListDown.at(2 * i)));
//         }

//         // 累加已完成的弧长
//         this->toolConfig.totalPolishLength += currentLayerLength;

//         // 判断是否超过阈值并触发换刀
//         if (this->toolConfig.totalPolishLength >= this->toolConfig.toolChangeThreshold && i < count) {
//             qDebug() << "Reached threshold, total length:" << this->toolConfig.totalPolishLength;

//             // A. 抬起动作: 在当前层结束点 posEnd 基础上,沿工具 Z 轴向上抬起 100mm
//             Point liftPoint;
//             if (i != count) {
//                 liftPoint = posEnd; // 使用圆弧运动的结束点
//             } else {
//                 liftPoint = pos; // 使用直线下压的结束点
//             }
//             liftPoint = liftPoint.PosRelByTool(defaultDirection, -100.0);
//             MoveL(liftPoint, dVelocity, 2000, 0); // 快速移动到抬起点
//             while (IsRobotMoved()) { QThread::msleep(50); }

//             // B. 执行换刀逻辑
//             // 您的 ToolChange 会记录此 liftPoint 并在结束后自动 MoveL 回来
//             ToolChange(craft);

//             qDebug() << "MoveCylinderVertical: ToolChange finished. Robot returned to lift point. Continuing...";
//         }

//         // 如果在运行过程中点击停止
//         if (isStop) {
//             if (i != count) {
//                 return posEnd;
//             } else {
//                 return pos;
//             }
//         }
//     }

//     // ========== 8. 返回最终位置 ==========
//     return pos;
// }


Point Robot::MoveConicalFrustum(Craft &craft) {
    /*
     * 圆台侧面打磨 - 凹面(内侧)
     *
     * 1. 每层生成完整插值点 (用于计算几何)
     * 2. 简化为5个关键点: approach, start, mid, end, retract
     * 3. 实际运动: MoveL(approach) → MoveL(start) → MoveC(mid, end) → MoveL(retract)
     * 4. 之字形往复: 奇数层翻转点序
     *
     * - Run负责: 全局MoveBefore(到安全点+AGP) 和 MoveAfter(回安全点)
     * - 本函数负责: 各层内部运动 + 层间过渡
     */

    // ========== 阶段1: 圆弧几何参数提取 ==========

    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    QVector<QVector3D> centerListUp;
    QVector<double> radiusListUp, lengthListUp;
    double totalArcLengthUp = 0.0;

    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListUp.at(i - 1).pos,
            posListUp.at(i).pos,
            posListUp.at(i + 1).pos
        );
        centerListUp.append(center);

        double radius = (posListUp.at(i).pos - center).length();
        radiusListUp.append(radius);

        QVector3D OA = posListUp.at(i - 1).pos - center;
        QVector3D OM = posListUp.at(i).pos - center;
        QVector3D OB = posListUp.at(i + 1).pos - center;

        double length = (qAcos(QVector3D::dotProduct(OA, OM) / (OA.length() * OM.length())) +
                         qAcos(QVector3D::dotProduct(OM, OB) / (OM.length() * OB.length())))
                        * radius;
        lengthListUp.append(length);
        totalArcLengthUp += length;
    }

    // ========== 阶段2: 分层路径生成 (完整插值点) ==========

    int count = craft.offsetCount;
    double discRadius = craft.discRadius;
    double grindAngle = craft.grindAngle;

    QVector3D vec_AB = posListUp.at(1).pos - posListUp.at(0).pos;
    QVector3D vec_AC = posListUp.at(2).pos - posListUp.at(0).pos;
    QVector3D axis = QVector3D::crossProduct(vec_AB, vec_AC).normalized();

    QVector<QVector<QPair<QVector3D, QVector3D>>> layerTempGeom;
    int pointsPerLayer = 20;

    for (int layer = 0; layer <= count; ++layer) {
        double t_layer = (count > 0) ? static_cast<double>(layer) / count : 0.0;

        QVector3D L_i = pointSet.beginPoint.pos * (1 - t_layer) +
                        pointSet.beginOffsetPoint.pos * t_layer;
        QVector3D R_i = pointSet.endPoint.pos * (1 - t_layer) +
                        pointSet.endOffsetPoint.pos * t_layer;

        QVector3D base_rot_start = pointSet.beginPoint.rot;
        QVector3D base_rot_end = pointSet.endPoint.rot;
        QVector3D center_up = centerListUp.first();

        QVector3D vec_start = L_i - center_up;
        QVector3D vec_end = R_i - center_up;
        double r_start = vec_start.length();
        double r_end = vec_end.length();

        double cos_theta = QVector3D::dotProduct(vec_start, vec_end) / (r_start * r_end + 1e-6);
        double layer_total_angle_rad = qAcos(qBound(-1.0, cos_theta, 1.0));

        QVector<QPair<QVector3D, QVector3D>> currentLayerGeom;

        for (int p = 0; p <= pointsPerLayer; ++p) {
            double t_point = static_cast<double>(p) / pointsPerLayer;
            double r_current = r_start * (1 - t_point) + r_end * t_point;
            double current_angle_deg = qRadiansToDegrees(layer_total_angle_rad * t_point);

            QMatrix3x3 R = Point::toRotationMatrix(axis, current_angle_deg);
            QVector3D dir_start = vec_start.normalized();
            QVector3D rotated_dir = QVector3D(
                R(0,0)*dir_start.x() + R(0,1)*dir_start.y() + R(0,2)*dir_start.z(),
                R(1,0)*dir_start.x() + R(1,1)*dir_start.y() + R(1,2)*dir_start.z(),
                R(2,0)*dir_start.x() + R(2,1)*dir_start.y() + R(2,2)*dir_start.z()
            );

            QVector3D pos = center_up + rotated_dir * r_current;
            if (p == pointsPerLayer) pos = R_i;

            QVector3D rot_interp = base_rot_start * (1 - t_point) + base_rot_end * t_point;
            currentLayerGeom.append(qMakePair(pos, rot_interp));
        }

        layerTempGeom.append(currentLayerGeom);
    }

    // ========== 阶段3: 计算最终点 (姿态+补偿) ==========

    QVector<QVector<Point>> layerFinalPoints;

    for (int layer = 0; layer <= count; ++layer) {
        QVector<QPair<QVector3D, QVector3D>> geomList = layerTempGeom[layer];

        if (layer % 2 != 0) {
            std::reverse(geomList.begin(), geomList.end());
        }

        QVector<Point> finalPoints;
        int numPoints = geomList.size();

        for (int i = 0; i < numPoints; ++i) {
            QVector3D curr_pos = geomList[i].first;
            QVector3D curr_base_rot = geomList[i].second;

            QVector3D move_dir;
            if (i < numPoints - 1) {
                move_dir = geomList[i + 1].first - curr_pos;
            } else if (i > 0) {
                move_dir = curr_pos - geomList[i - 1].first;
            } else {
                move_dir = QVector3D(1, 0, 0);
            }

            double norm = move_dir.length();
            if (norm > 1e-6) move_dir = move_dir / norm;

            double limited_angle = qBound(-30.0, static_cast<double>(grindAngle), 30.0);
            QVector3D final_rot = Point::getNewRotation(curr_base_rot, move_dir, limited_angle);

            // 打磨片半径补偿
            QVector3D compensated_pos = curr_pos;

            if (discRadius > 0.0) {
                QMatrix3x3 R_tool = Point::toRotationMatrix(final_rot);
                QVector3D tool_z = QVector3D(R_tool(0,2), R_tool(1,2), R_tool(2,2));

                QVector3D plane_normal = QVector3D::crossProduct(tool_z, move_dir);
                double norm_pn = plane_normal.length();

                if (norm_pn > 1e-6) {
                    plane_normal = plane_normal / norm_pn;
                    QVector3D offset_direction = QVector3D::crossProduct(plane_normal, tool_z);
                    double norm_offset = offset_direction.length();
                    if (norm_offset > 1e-9) {
                        offset_direction = offset_direction / norm_offset;
                        compensated_pos = curr_pos + offset_direction * discRadius;
                    }
                } else {
                    QVector3D tool_x = QVector3D(R_tool(0,0), R_tool(1,0), R_tool(2,0));
                    compensated_pos = curr_pos + tool_x * discRadius;
                }
            }

            Point finalPoint(compensated_pos.x(), compensated_pos.y(), compensated_pos.z(),
                             final_rot.x(), final_rot.y(), final_rot.z());
            finalPoints.append(finalPoint);
        }

        layerFinalPoints.append(finalPoints);
    }

    // ========== 阶段4: 简化为5点 + 执行运动 ==========

    double dVelocity = craft.moveSpeed;
    double dAcc = 2000;
    double dRadius = craft.transitionRadius;
    double lift_distance = 80.0;

    Point pos;  // ← 用于记录最后一次运动的目标点

    for (int layer = 0; layer <= count; ++layer) {
        QVector<Point> &fullPoints = layerFinalPoints[layer];
        int numPoints = fullPoints.size();

        if (numPoints < 2) continue;

        // 简化为5个关键点
        Point start_pt = fullPoints[0];
        Point end_pt = fullPoints[numPoints - 1];
        int mid_idx = numPoints / 2;
        Point mid_pt = fullPoints[mid_idx];

        Point approach_pt = start_pt.PosRelByTool(defaultDirection, -lift_distance);
        Point retract_pt = end_pt.PosRelByTool(defaultDirection, -lift_distance);

        // 1. 下压到起点
        // if (layer == 0) {
        //     MoveL(approach_pt, dVelocity, dAcc, dRadius);
        // }

        MoveL(start_pt, craft.cutinSpeed, dAcc, dRadius);

        // 2. 圆弧运动
        MoveC(mid_pt, end_pt, dVelocity, dAcc, dRadius);

        // 3. 抬起 (这是当前层的最后一次运动)
        MoveL(retract_pt, craft.cutinSpeed, dAcc, dRadius);
        pos = retract_pt;  // ← 记录最后一次MoveL的目标点

        while (IsRobotMoved()) { QThread::msleep(50); }

        // 换刀逻辑
        double currentLayerLength = totalArcLengthUp / (count + 1);
        this->toolConfig.totalPolishLength += currentLayerLength;

        if (this->toolConfig.totalPolishLength >= this->toolConfig.toolChangeThreshold && layer < count) {
            qDebug() << "Reached threshold:" << toolConfig.totalPolishLength << "mm. Starting ToolChange...";
            qDebug() << "threshold:" << toolConfig.toolChangeThreshold << "mm. Starting ToolChange...";

            Point liftPoint = retract_pt.PosRelByTool(defaultDirection, -20.0);
            MoveL(liftPoint, dVelocity, 2000, 0);
            pos = liftPoint;  // ← 更新pos
            while (IsRobotMoved()) { QThread::msleep(50); }

            ToolChange(craft);
            toolConfig.totalPolishLength = 0;

            qDebug() << "ToolChange finished. Continuing...";
        }

        // 层间过渡
        if (layer < count) {
            QVector<Point> &nextLayerPoints = layerFinalPoints[layer + 1];
            Point next_start = nextLayerPoints[0];
            Point next_approach = next_start.PosRelByTool(defaultDirection, -lift_distance);

            MoveL(next_approach, dVelocity, dAcc, dRadius);
            pos = next_approach;  // ← 更新pos
        }

        if (isStop) return pos;  // ← 中途停止时返回当前pos
    }

    // ← 返回最后一次运动的目标点 (最后一层的retract_pt或next_approach)
    return pos;
}


void Robot::MoveZLine(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    int size = craft.offsetCount + 1;
    float factor = 1.0 / size;

    Point point = pointSet.beginPoint;
    Point pointOffset, pos;
    pointOffset.pos = pointSet.auxPoint.pos - pointSet.beginPoint.pos;
    pointOffset.rot = pointSet.auxPoint.rot - pointSet.beginPoint.rot;
    for (int i = 1; i <= size; ++i) {
        point += pointOffset;
        pos.pos = point.pos + translation;
        pos.rot = newRot;
        MoveL(pos, dVelocity, dAcc, dRadius);
        point =
            Point::scale(pointSet.beginPoint, pointSet.endPoint, factor * i);
        pos.pos = point.pos + translation;
        pos.rot = newRot;
        MoveL(pos, dVelocity, dAcc, dRadius);
    }
}

void Robot::MoveSpiralLine(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    int size = craft.offsetCount + 1;
    float factor = 1.0 / (2 * size + 2);

    // 定义空间目标位置
    Point pointEnd =
        Point::scale(pointSet.beginPoint, pointSet.endPoint, factor * 4);
    QVector3D O = Point::calculateCircumcenter(
        pointSet.beginPoint.pos, pointSet.auxPoint.pos, pointEnd.pos);
    Point pointAux =
        Point::scale(pointSet.beginPoint, pointSet.endPoint, factor * 2);
    QVector3D temp = (pointAux.pos - O);
    QVector3D upOffset =
        temp.normalized() * pointSet.beginPoint.pos.distanceToPoint(O) - temp;
    QVector3D downOffset = temp * (-0.5);
    pointAux.pos += upOffset;
    Point posAux, posEnd;
    posAux.pos = pointAux.pos + translation;
    posAux.rot = newRot;
    posEnd.pos = pointEnd.pos + translation;
    posEnd.rot = newRot;
    // 执行路点运动
    MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
    for (int i = 1; i < size; ++i) {
        // 小圆弧
        pointEnd = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i));
        pointAux = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i + 1));
        pointAux.pos += downOffset;
        posAux.pos = pointAux.pos + translation;
        posAux.rot = newRot;
        posEnd.pos = pointEnd.pos + translation;
        posEnd.rot = newRot;
        MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        // 大圆弧
        pointEnd = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i + 4));
        pointAux = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i + 2));
        pointAux.pos += upOffset;
        posAux.pos = pointAux.pos + translation;
        posAux.rot = newRot;
        posEnd.pos = pointEnd.pos + translation;
        posEnd.rot = newRot;
        MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
    }
}

/**
 * @brief 校验换刀安全性
 * @param errorCode 错误码返回 (1:当前位有刀, 2:高一位无刀, 3:已是最后一把)
 * @return true 允许换刀 / false 拦截
 */
bool Robot::CanPerformToolChange(int &errorCode) {
    int status = this->toolConfig.toolMagStatus;
    int currentIdx = this->toolConfig.targetToolPos; // 0-3
    int nextIdx = currentIdx + 1;

    // 1. 检查当前位置 (必须为空，才能放回旧刀)
    int currentMask = (1 << currentIdx);
    if ((status & currentMask) != 0) {
        errorCode = 1; // 当前位有刀，会碰撞
        return false;
    }

    // 2. 检查是否有下一把刀
    if (nextIdx > 3) {
        errorCode = 3; // 已经是4号位，没法再往高处换了
        return false;
    }

    // 3. 检查高一位 (必须有刀，才能取出新刀)
    int nextMask = (1 << nextIdx);
    if ((status & nextMask) == 0) {
        errorCode = 2; // 高一位是空的，没刀可拿
        return false;
    }

    errorCode = 0; // 校验通过
    return true;
}

/**
 * @brief 执行换刀流程
 * @param craft 传入工艺引用以更新 targetToolPos
 */
void Robot::ToolChange(Craft &craft) {
    Point point;    // 定义空间目标位置
    double dVelocity = defaultVelocity;    // 定义运动速度
    double dAcc = 2000;    // 定义运动加速度
    double dRadius = craft.transitionRadius;    // 定义过渡半径
    point = pointSet.safePoint;    // 移到安全点

    // 0. 检测 craft 参数
    if (this->toolConfig.targetToolPos >= 4) {
        QMessageBox::warning(nullptr, "换刀提醒", "当前已是最后一把工具（pos=4），不可继续换刀！");
        return;
    }

    // 1. 获取当前机器人位置（用于后续第6步复位）
    Point currentPos;
    if (!GetTcpPoint(currentPos)) {
        qDebug() << "获取当前位置失败，无法执行换刀";
        return;
    }

    // 刀库状态检测 罩壳是否打开
    int di6 = 0; // DI6 - 刀库门打开信号
    HRIF_ReadBoxDI(0, 6, di6);
    if (di6 != 1) {
        HRIF_SetBoxDO(0, 6, 1);
    }
    // 刀库刀具状态与更换参数是否对应
    // 检测更换参数是否是4 如果是 则弹窗提醒 并且不触发换刀 直接回到安全点 继续打磨
    int err = 0;
    if (!CanPerformToolChange(err)) {
        QString msg;
        if (err == 1) msg = QString("错误：当前刀位 %1 已有刀，无法放回！").arg(this->toolConfig.targetToolPos + 1);
        else if (err == 2) msg = QString("错误：预备刀位 %1 是空的，无刀可换！").arg(this->toolConfig.targetToolPos + 2);
        else if (err == 3) msg = "提示：当前已是 4 号刀位，换刀流程已结束。";

        QMessageBox::critical(nullptr, "换刀校验未通过", msg +
            QString("\n(当前状态码: %1)").arg(this->toolConfig.toolMagStatus));

        // 机器人移动到安全点 (确保姿态安全)
        MoveL(point, dVelocity, dAcc, dRadius);
        while (IsRobotMoved()) { QThread::msleep(50); }
        // 机器人移动到之前记录的位置（复位）
        MoveL(currentPos, dVelocity, dAcc, dRadius);
        while (IsRobotMoved()) { QThread::msleep(50); }
        // 启动打磨头
        // AGPRun(craft, true);
        return;
    }
    // 打磨头停转 调整到位置模式
    AGPStop();
    if (agp != nullptr) {

            // 设置AGP默认参数
            agp->Control(FUNC::RESET);
            agp->Control(FUNC::ENABLE);
            agp->SetMode(MODE::PosMode);

            agp->SetPos(1000);
            agp->SetForce(200);
            agp->SetTouchForce(0);
            agp->SetRampTime(0);
            // 检查并确保使能状态
            if (!IsAGPEnabled()) {
                agp->Control(FUNC::ENABLE);
            }
        }

    // 2. 机器人移动到安全点位 (采用 craft 中的运动参数)
    MoveL(point, dVelocity, dAcc, dRadius);
    while (IsRobotMoved()) { QThread::msleep(50); }

    // 3. 基于 craft 参数设置模拟量 IO 数值 (1.2 - 1.8)
    // 映射逻辑：pos 0->1.2, 1->1.4, 2->1.6, 3->1.8
    // 更新逻辑: 只能退1换2 退2换3 退3换4
    double sendVal = 1.2 + (this->toolConfig.targetToolPos * 0.2);
    // boxID=0, nBit=0, nMode=1 (通常为电压模式)
    HRIF_SetBoxAOVal(0, 0, sendVal, 1);

    // 4. 等待并读取 IO，当机器人 IO 变为 2.0 时继续
    int currentMode = 0;
    double readBackVal = 0.0;
    bool isDone = false;

    while (!isDone) {
        // 使用您提供的 HRIF_ReadBoxAO 接口
        int nRet = HRIF_ReadBoxAO(0, 0, currentMode, readBackVal);
        if (nRet == 0) {
            // 判断是否达到 2.0（允许微小浮点误差）
            if (qAbs(readBackVal - 2.0) < 0.01) {
                isDone = true;
                qDebug() << "检测到 2.0，换刀已完成";
            }
        }
        // 如果点击了停止按钮或发生急停，应退出循环
        if (isStop) return;

        QThread::msleep(100); // 轮询间隔
    }
    // 5. 换刀完毕后，机器人再次移动到安全点 (确保姿态安全)
    MoveL(point, dVelocity, dAcc, dRadius);
    while (IsRobotMoved()) { QThread::msleep(50); }
    // 6. 机器人移动到之前第 1 步中记录的位置（复位）
    MoveL(currentPos, dVelocity, dAcc, dRadius);
    while (IsRobotMoved()) { QThread::msleep(50); }
    //换刀结束 启动打磨头
    AGPRun(craft, true);
    // 7. craft 中的 targetToolPos + 1
    this->toolConfig.targetToolPos += 1;
    // 重置累计数据
    this->toolConfig.totalPolishLength = 0;
    // 重置 AO 信号
    HRIF_SetBoxAOVal(0, 0, 1.0, 0);
    // qDebug() << "换刀流程圆满结束，当前工具位已更新为:" << craft.targetToolPos;
}

void Robot::Run(Craft &craft, bool isAGPRun) {
    //机器人控制脚本启动

    // QThread::msleep(100);
    double radius = craft.discRadius;
    double angle = craft.grindAngle;
    QVector3D rotation = pointSet.beginPoint.rot;
    QVector3D moveDirection = pointSet.endPoint.pos - pointSet.beginPoint.pos;
    // if (craft.way == PolishWay::RegionArcWay_Vertical_Repeat) {
    //     moveDirection = pointSet.beginOffsetPoint.pos -
    //     pointSet.beginPoint.pos;
    // }
    // 获取新的姿态
    newRot = Point::getNewRotation(rotation, moveDirection, angle);
    // qDebug() << rotation;
    // qDebug() << newRot;
    // 获取新姿态需要的平移量
    translation = Point::getTranslation(rotation, moveDirection, radius, angle);
    // qDebug() << moveDirection;
    // qDebug() << translation;
    // 获取反向姿态和平移量
    rotation = pointSet.endPoint.rot;
    moveDirection = pointSet.beginPoint.pos - pointSet.endPoint.pos;
    newRotInv = Point::getNewRotation(rotation, moveDirection, angle);
    translationInv =
        Point::getTranslation(rotation, moveDirection, radius, angle);
    // 开始运动
    isStop.store(false);
    MoveBefore(craft, isAGPRun);//机器人运动到安全点
    // Point point = pointSet.auxEndPoint;
    Point point;
    point.pos = pointSet.endPoint.pos + translation;
    point.rot = newRot;
    //打开AGP冷却气
    HRIF_SetBoxDO(0, 5, 1);
    // 选择打磨方式
    switch (craft.way) {
    case PolishWay::ArcWay:
        MoveArc(craft);
        break;
    case PolishWay::LineWay:
        MoveLine(craft);
        break;
    case PolishWay::RegionArcWay1:
        point = MoveRegionArc1(craft);
        break;
    case PolishWay::RegionArcWay2:
        point = MoveRegionArc2(craft);
        break;
    case PolishWay::RegionArcWay_Horizontal:
        point = MoveRegionArcHorizontal(craft);
        break;
    case PolishWay::RegionArcWay_Vertical:
        point = MoveRegionArcVertical(craft); // 扇形环面打磨
        break;
    case PolishWay::RegionArcWay_Vertical_Repeat:
        point = MoveRegionArcVerticalRepeat(craft);
        break;
    case PolishWay::CylinderWay_Horizontal_Convex:
        point = MoveCylinderHorizontal(craft, true);
        break;
    case PolishWay::CylinderWay_Vertical_Convex:
        point = MoveCylinderVertical(craft, true); // 柱体外侧面打磨 凸面打磨
        break;
    case PolishWay::CylinderWay_Horizontal_Concave:
        point = MoveCylinderHorizontal(craft, false);
        break;
    case PolishWay::CylinderWay_Vertical_Concave:
        point = MoveCylinderVertical(craft, false);
        break;
    case PolishWay::ConicalFrustum_Concave: // 倒圆台侧面打磨
        point = MoveConicalFrustum(craft);
        break;
    case PolishWay::ZLineWay:
        MoveZLine(craft);
        break;
    case PolishWay::SpiralLineWay:
        MoveSpiralLine(craft);
        break;
    default:
        break;
    }
    point = point.PosRelByTool(defaultDirection, defaultOffset);
    MoveAfter(craft, point); //机器人从终点抬起 运动到安全点
    // 等待运动完成
    while (true) {
        if (!IsRobotMoved()) {
            isStop.store(true);
            break;
        }
        QThread::msleep(100);
    }
    //关闭AGP冷却气
    HRIF_SetBoxDO(0, 5, 0);
}

HansRobot::HansRobot() {}

HansRobot::~HansRobot() {
    HRIF_GrpCloseFreeDriver(0, 0);
    HRIF_GrpDisable(0, 0);
}

bool HansRobot::RobotConnect(QString robotIP) {
    int nRet = -1;
    int nRet2 = -1;
    std::string ip = robotIP.toStdString();
    const char *hostname = ip.c_str();
    unsigned short nPort = 10003;
    nRet = HRIF_Connect(0, hostname, nPort);
    qDebug() << "start"<<nRet;
    if (nRet == 0) {
        // 机器人上电
        HRIF_Electrify(0);
        // 机器人使能
        HRIF_GrpEnable(0, 0);
        // 设置速度比
        HRIF_SetOverride(0, 0, 1.0);
        // 重置AO换刀信号
        HRIF_SetBoxAOVal(0, 0, 1.0, 0);//重置换刀AO信号
        // 启动机器人预设脚本
        QThread::msleep(1000);
        nRet2=HRIF_StartScript(0);
        qDebug() << "HRIF_StartScript return:" << nRet2;
        return true;
    }
    return false;
}

bool HansRobot::IsRobotElectrified() {
    // 定义需要读取的机器人状态变量
    int nMovingState = 0;
    int nEnableState = 0;
    int nErrorState = 0;
    int nErrorCode = 0;
    int nErrorAxis = 0;
    int nBreaking = 0;
    int nPause = 0;
    int nEmergencyStop = 0;
    int nSaftyGuard = 0;
    int nElectrify = 0;
    int nIsConnectToBox = 0;
    int nBlendingDone = 0;
    int nInPos = 0;
    // 读取状态
    HRIF_ReadRobotState(0, 0, nMovingState, nEnableState, nErrorState,
                        nErrorCode, nErrorAxis, nBreaking, nPause,
                        nEmergencyStop, nSaftyGuard, nElectrify,
                        nIsConnectToBox, nBlendingDone, nInPos);
    return nElectrify == 1 ? true : false;
}

bool HansRobot::IsRobotEnabled() {
    // 定义需要读取的机器人状态变量
    int nMovingState = 0;
    int nEnableState = 0;
    int nErrorState = 0;
    int nErrorCode = 0;
    int nErrorAxis = 0;
    int nBreaking = 0;
    int nPause = 0;
    int nBlendingDone = 0;
    // 读取状态
    HRIF_ReadRobotFlags(0, 0, nMovingState, nEnableState, nErrorState,
                        nErrorCode, nErrorAxis, nBreaking, nPause,
                        nBlendingDone);
    return nEnableState == 1 ? true : false;
}

bool HansRobot::IsRobotMoved() {
    // 定义需要读取的机器人状态变量
    int nMovingState = 0;
    int nEnableState = 0;
    int nErrorState = 0;
    int nErrorCode = 0;
    int nErrorAxis = 0;
    int nBreaking = 0;
    int nPause = 0;
    int nBlendingDone = 0;
    // 读取状态
    HRIF_ReadRobotFlags(0, 0, nMovingState, nEnableState, nErrorState,
                        nErrorCode, nErrorAxis, nBreaking, nPause,
                        nBlendingDone);
    return nMovingState == 1 ? true : false;
}

bool HansRobot::RobotTeach(int pos) {
    if (!isTeach) {
        if (agp != nullptr) {
            // 设置AGP默认参数
            agp->Control(FUNC::RESET);
            agp->Control(FUNC::ENABLE);
            agp->SetMode(MODE::PosMode);
            agp->SetPos(pos * 100);
            agp->SetForce(200);
            agp->SetTouchForce(0);
            agp->SetRampTime(0);
            if (!IsAGPEnabled()) {
                agp->Control(FUNC::ENABLE);
            }
        }
        if (!IsRobotElectrified()) {
            // 机器人上电
            HRIF_Electrify(0);
            if (!IsRobotElectrified()) {
                return isTeach;
            }
        }
        if (!IsRobotEnabled()) {
            // 机器人使能
            HRIF_GrpEnable(0, 0);
            QThread::msleep(1500);
            if (!IsRobotEnabled()) {
                return isTeach;
            }
        }
        // 设置速度比
        HRIF_SetOverride(0, 0, 1.0);
        // 启用自由拖拽
        int nRet = HRIF_GrpOpenFreeDriver(0, 0);
        if (nRet == 0) {
            isTeach = true;
        }
    } else {
        // 关闭自由拖拽
        int nRet = HRIF_GrpCloseFreeDriver(0, 0);
       if (nRet == 0) {
           isTeach = false;
        }
    }
    return isTeach;
}

bool HansRobot::CloseFreeDriver() {
    // 关闭自由拖拽
    int nRet = HRIF_GrpCloseFreeDriver(0, 0);
    if (nRet == 0) {
        isTeach = false;
        return true;
    }
    return false;
}

bool HansRobot::GetTcpPoint(Point &point) {
    // 获取位姿信息
    // int nRet = HRIF_ReadCmdTcpPos(0, 0, point.x, point.y, point.z, point.rx,
    //                               point.ry, point.rz);
    double x = 0;
    double y = 0;
    double z = 0;
    double rx = 0;
    double ry = 0;
    double rz = 0;
    int nRet = HRIF_ReadActTcpPos(0, 0, x, y, z, rx, ry, rz);
    if (nRet == 0) {
        point.pos.setX(x);
        point.pos.setY(y);
        point.pos.setZ(z);
        point.rot.setX(rx);
        point.rot.setY(ry);
        point.rot.setZ(rz);

        qDebug() << "HansRobot TCP Pos [X, Y, Z]:" << x << "," << y << "," << z;
        qDebug() << "HansRobot TCP Rot [RX, RY, RZ]:" << rx << "," << ry << "," << rz;

        return true;
    } else {
        return false;
    }
}

bool HansRobot::Stop() {
    // 机器人停止
    isStop.store(true);
    HRIF_GrpStop(0, 0);
    // HRIF_StopScript(0);
    // AGP停止
    if (agp != nullptr) {
        agp->SetSpeed(0);
    }
    // 机器人复位
    HRIF_GrpReset(0, 0);
    // AGP复位
    if (agp != nullptr) {
        agp->Control(FUNC::RESET);
    }
    // 自由拖拽复位
    isTeach = false;

    return true;
}

void HansRobot::OpenWeb(QString ip) {
    QDesktopServices::openUrl(QUrl("http://" + ip + "/dist"));
}

void HansRobot::MoveTcpL(const Point &point, double velocity, double acc,
                         double radius) {
    // 定义运动类型
    int nMoveType = 1;
    // 定义关节目标位置
    double dJ1 = 0;
    double dJ2 = 0;
    double dJ3 = 0;
    double dJ4 = 0;
    double dJ5 = 0;
    double dJ6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP_AGP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = velocity;
    // 定义运动加速度
    double dAcc = 1000;
    // 定义过渡半径
    double dRadius = radius;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // int nIsUseJoint = 0;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 直线运动
    qDebug()<<"moveL:4575";
   int ret= HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
     qDebug()<<"moveL:"<<ret;
}

void HansRobot::MoveTcpC(const Point &auxPoint, const Point &endPoint,
                         double velocity, double acc, double radius) {
    // 定义运动类型
    int nMoveType = 2;
    // 定义关节目标位置
    double dJ1 = 0;
    double dJ2 = 0;
    double dJ3 = 0;
    double dJ4 = 0;
    double dJ5 = 0;
    double dJ6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP_AGP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = velocity;
    // 定义运动加速度
    double dAcc = 1000;
    // 定义过渡半径
    double dRadius = radius;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 圆弧运动
    HRIF_WayPoint2(0, 0, nMoveType, endPoint.pos.x(), endPoint.pos.y(),
                   endPoint.pos.z(), endPoint.rot.x(), endPoint.rot.y(),
                   endPoint.rot.z(), auxPoint.pos.x(), auxPoint.pos.y(),
                   auxPoint.pos.z(), auxPoint.rot.x(), auxPoint.rot.y(),
                   auxPoint.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                   sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                   nIOBit, nIOState, strCmdID);
}
