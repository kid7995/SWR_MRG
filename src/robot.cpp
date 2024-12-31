#include <QDebug>
#include <QDesktopServices>
#include <QMessageBox>
#include <QThread>
#include <QUrl>

#include "robot.h"

#include "HR_Pro.h"

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
    point = point.PosRelByTool(defaultDirection, pos + discThickness);
    return true;
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

Point Robot::MoveRegionArcHorizontal(const Craft &craft) {
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;
    // 计算单次偏移量
    int count = craft.offsetCount;

    // 圆弧上界
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    // 圆弧上界到下界的偏移距离
    double midOffset = 0.0;
    QVector<QVector3D> offsetList;
    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListUp.at(i - 1).pos, posListUp.at(i).pos,
            posListUp.at(i + 1).pos);
        if (i == 1) {
            if (count > 0) {
                midOffset =
                    ((posListUp.at(i - 1).pos - center).length() -
                     (pointSet.beginOffsetPoint.pos - center).length()) /
                    count;
            }
            offsetList.append((center - posListUp.at(i - 1).pos).normalized() *
                              midOffset);
        }
        offsetList.append((center - posListUp.at(i).pos).normalized() *
                          midOffset);
        offsetList.append((center - posListUp.at(i + 1).pos).normalized() *
                          midOffset);
    }

    Point pos, posAux, posEnd;
    for (int i = 0; i < count + 1; ++i) {
        if (i % 2 == 0) { // 正向
            if (i > 0) {
                // 抬高
                pos.pos = posListUp.constFirst().pos +
                          offsetList.constFirst() * (i - 1) + translationInv;
                pos.rot = newRotInv;
                pos = pos.PosRelByTool(defaultDirection, defaultOffset);
                MoveL(pos, dVelocity, dAcc, dRadius);
                // 改变位姿
                pos.pos = posListUp.constFirst().pos +
                          offsetList.constFirst() * i + translation;
                pos.rot = newRot;
                pos = pos.PosRelByTool(defaultDirection, defaultOffset);
                MoveL(pos, dVelocity, dAcc, dRadius);
                // 压低
                pos.pos = posListUp.constFirst().pos +
                          offsetList.constFirst() * i + translation;
                pos.rot = newRot;
                MoveL(pos, dVelocity, dAcc, dRadius);
            }
            // 正向圆弧运动
            for (int j = 1; j < posListUp.size() - 1; j += 2) {
                posAux.pos =
                    posListUp.at(j).pos + offsetList.at(j) * i + translation;
                posAux.rot = newRot;
                posEnd.pos = posListUp.at(j + 1).pos +
                             offsetList.at(j + 1) * i + translation;
                posEnd.rot = newRot;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
        } else { // 反向
            // 抬高
            pos.pos = posListUp.constLast().pos +
                      offsetList.constLast() * (i - 1) + translation;
            pos.rot = newRot;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 改变位姿
            pos.pos = posListUp.constLast().pos + offsetList.constLast() * i +
                      translationInv;
            pos.rot = newRotInv;
            pos = pos.PosRelByTool(defaultDirection, defaultOffset);
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 压低
            pos.pos = posListUp.constLast().pos + offsetList.constLast() * i +
                      translationInv;
            pos.rot = newRotInv;
            MoveL(pos, dVelocity, dAcc, dRadius);
            // 反向圆弧运动
            for (int j = posListUp.size() - 2; j > 0; j -= 2) {
                posAux.pos =
                    posListUp.at(j).pos + offsetList.at(j) * i + translationInv;
                posAux.rot = newRotInv;
                posEnd.pos = posListUp.at(j - 1).pos +
                             offsetList.at(j - 1) * i + translationInv;
                posEnd.rot = newRotInv;
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
        }
    }

    return posEnd;
}

/*
Point Robot::MoveRegionArcVertical(const Craft &craft) {
    // Position beginOffset = (beginOffsetPoint - beginPoint);
    // Position endOffset = (endOffsetPoint - endPoint);
    Point temp1;
    temp1.pos = pointSet.beginOffsetPoint.pos - pointSet.endPoint.pos;
    Point temp2;
    temp2.pos = pointSet.beginPoint.pos - pointSet.endPoint.pos;
    Point beginOffset;
    beginOffset.pos =
        (temp1.pos - temp2.pos * (QVector3D::dotProduct(temp1.pos, temp2.pos) /
                                  QVector3D::dotProduct(temp2.pos, temp2.pos)));

    Point temp3;
    temp3.pos = pointSet.endOffsetPoint.pos - pointSet.beginPoint.pos;
    Point temp4;
    temp4.pos = pointSet.endPoint.pos - pointSet.beginPoint.pos;
    Point endOffset;
    endOffset.pos =
        (temp3.pos - temp4.pos * (QVector3D::dotProduct(temp3.pos, temp4.pos) /
                                  QVector3D::dotProduct(temp4.pos, temp4.pos)));

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

    QVector<Point> posListUp, posListDown;
    // 圆弧上界
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);
    // 圆弧下界
    posListDown.append(pointSet.beginOffsetPoint);
    for (int i = 0; i < pointSet.midPoints.size(); ++i) {
        Point point = pointSet.midPoints.at(i);
        point += midOffsetList.at(i);
        posListDown.append(point);
    }
    posListDown.append(pointSet.endOffsetPoint);

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
        // qDebug() << (QVector3D::dotProduct(OA, OB) /
        //              (OA.length() * OB.length()));
        // qDebug() << length;
        lengthListUp.append(length);
        totalArcLengthUp += length;
    }
    QVector<QVector3D> finalPosListUp;
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
                qDebug() << (arcLengthUp / radiusListUp.at(i));
                qDebug() << qRadiansToDegrees(arcLengthUp / radiusListUp.at(i));
                QVector3D trans = posListUp.at(2 * i).pos - centerListUp.at(i);
                QVector3D newTrans =
                    QVector3D(R(0, 0) * trans.x() + R(0, 1) * trans.y() +
                                  R(0, 2) * trans.z(),
                              R(1, 0) * trans.x() + R(1, 1) * trans.y() +
                                  R(1, 2) * trans.z(),
                              R(2, 0) * trans.x() + R(2, 1) * trans.y() +
                                  R(2, 2) * trans.z());
                finalPosListUp.append(centerListUp.at(i) + newTrans);
                arcLengthUp += unitArcLengthUp;
            }
            arcLengthUp -= lengthListUp.at(i);
        }
    } else {
        finalPosListUp.append(pointSet.beginPoint.pos);
    }
    qDebug() << finalPosListUp;
    // 计算下圆弧组中圆弧圆心、半径和弧长
    QVector<QVector3D> centerListDown;
    QVector<double> radiusListDown, lengthListDown;
    double totalArcLengthDown = 0.0;
    for (int i = 1; i < posListDown.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListDown.at(i - 1).pos, posListDown.at(i).pos,
            posListDown.at(i + 1).pos);
        centerListDown.append(center);
        double radius = (posListDown.at(i).pos - center).length();
        radiusListDown.append(radius);
        QVector3D OA = posListDown.at(i - 1).pos - center;
        QVector3D OM = posListDown.at(i).pos - center;
        QVector3D OB = posListDown.at(i + 1).pos - center;
        // 适用OA、OB夹角大于180°的情况
        double length = (qAcos(QVector3D::dotProduct(OA, OM) /
                               (OA.length() * OM.length())) +
                         qAcos(QVector3D::dotProduct(OM, OB) /
                               (OM.length() * OB.length()))) *
                        radius;
        lengthListDown.append(length);
        totalArcLengthDown += length;
    }
    QVector<QVector3D> finalPosListDown;
    if (count > 0) {
        double unitArcLengthDown = totalArcLengthDown / count;
        double arcLengthDown = 0.0;
        for (int i = 0; i < centerListDown.size(); ++i) {
            while (arcLengthDown <= lengthListDown.at(i)) {
                QVector3D axis = QVector3D::crossProduct(
                    posListDown.at(2 * i).pos - centerListDown.at(i),
                    posListDown.at(2 * i + 1).pos - centerListDown.at(i));
                QMatrix3x3 R = Point::toRotationMatrix(
                    axis,
                    qRadiansToDegrees(arcLengthDown / radiusListDown.at(i)));
                QVector3D trans =
                    posListDown.at(2 * i).pos - centerListDown.at(i);
                QVector3D newTrans =
                    QVector3D(R(0, 0) * trans.x() + R(0, 1) * trans.y() +
                                  R(0, 2) * trans.z(),
                              R(1, 0) * trans.x() + R(1, 1) * trans.y() +
                                  R(1, 2) * trans.z(),
                              R(2, 0) * trans.x() + R(2, 1) * trans.y() +
                                  R(2, 2) * trans.z());
                finalPosListDown.append(centerListDown.at(i) + newTrans);
                arcLengthDown += unitArcLengthDown;
            }
            arcLengthDown -= lengthListDown.at(i);
        }
    } else {
        finalPosListDown.append(pointSet.beginOffsetPoint.pos);
    }
    qDebug() << finalPosListDown;

    Q_ASSERT(finalPosListUp.size() == finalPosListDown.size());
    // 定义运动速度
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
    // 定义空间目标位置
    Point point;
    for (int i = 0; i < finalPosListUp.size(); ++i) {
        point.pos = finalPosListDown.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);
        point.pos = finalPosListUp.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);
        point.pos = finalPosListDown.at(i) + translation;
        point.rot = newRot;
        MoveL(point, dVelocity, dAcc, dRadius);
    }

    return point;
}
*/

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
    // 圆弧上界
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

    // 打磨片半径
    double discRadius = craft.discRadius;
    // 打磨角度
    double grindAngle = craft.grindAngle;
    // 计算各点对应弧长和姿态
    double totalArcLengthUp = 0.0;
    QVector<double> lengthListUp;
    lengthListUp.append(totalArcLengthUp);
    QVector<QVector3D> newRotList, translationList;
    QVector3D aux =
        (pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos).normalized();
    for (int i = 1; i < posListUp.size() - 1; i += 2) {
        QVector3D center = Point::calculateCircumcenter(
            posListUp.at(i - 1).pos, posListUp.at(i).pos,
            posListUp.at(i + 1).pos);
        double radius = (posListUp.at(i).pos - center).length();
        QVector3D OA = posListUp.at(i - 1).pos - center;
        QVector3D OM = posListUp.at(i).pos - center;
        QVector3D OB = posListUp.at(i + 1).pos - center;
        double lengthAM =
            qAcos(QVector3D::dotProduct(OA, OM) / (OA.length() * OM.length())) *
            radius;
        totalArcLengthUp += lengthAM;
        lengthListUp.append(totalArcLengthUp);
        double lengthMB =
            qAcos(QVector3D::dotProduct(OM, OB) / (OM.length() * OB.length())) *
            radius;
        totalArcLengthUp += lengthMB;
        lengthListUp.append(totalArcLengthUp);

        // 计算各点姿态与对应偏移
        QVector3D axis, normal, rotation, moveDirection;
        axis = QVector3D::crossProduct(OA, OM).normalized();
        if (!isConvex) {
            axis = -axis;
        }
        if (craft.isMirror) {
            axis = -axis;
        }
        if (i == 1) {
            normal =
                QVector3D::crossProduct(QVector3D::crossProduct(aux, -OA), aux)
                    .normalized();
            if (!isConvex) {
                normal = -normal;
            }
            rotation = Point::getNormalRotation(normal, axis);
            moveDirection = QVector3D::crossProduct(normal, axis).normalized();
            if (craft.isMirror) {
                moveDirection = -moveDirection;
            }
            // 获取新的姿态
            newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
            newRotList.append(newRot);
            // 获取新姿态需要的平移量
            translation = Point::getTranslation(rotation, moveDirection,
                                                discRadius, grindAngle);
            translationList.append(translation);
        }
        normal = QVector3D::crossProduct(QVector3D::crossProduct(aux, -OM), aux)
                     .normalized();
        if (!isConvex) {
            normal = -normal;
        }
        rotation = Point::getNormalRotation(normal, axis);
        moveDirection = QVector3D::crossProduct(normal, axis).normalized();
        if (craft.isMirror) {
            moveDirection = -moveDirection;
        }
        // 获取新的姿态
        newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
        newRotList.append(newRot);
        // 获取新姿态需要的平移量
        translation = Point::getTranslation(rotation, moveDirection, discRadius,
                                            grindAngle);
        translationList.append(translation);

        normal = QVector3D::crossProduct(QVector3D::crossProduct(aux, -OB), aux)
                     .normalized();
        if (!isConvex) {
            normal = -normal;
        }
        rotation = Point::getNormalRotation(normal, axis);
        moveDirection = QVector3D::crossProduct(normal, axis).normalized();
        if (craft.isMirror) {
            moveDirection = -moveDirection;
        }
        // 获取新的姿态
        newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
        newRotList.append(newRot);
        // 获取新姿态需要的平移量
        translation = Point::getTranslation(rotation, moveDirection, discRadius,
                                            grindAngle);
        translationList.append(translation);
    }
    Q_ASSERT(lengthListUp.size() == posListUp.size());
    Q_ASSERT(newRotList.size() == posListUp.size());
    Q_ASSERT(translationList.size() == posListUp.size());

    // 偏移次数
    int count = craft.offsetCount;
    // 单次偏移距离
    QVector3D posOffset;
    if (count > 0) {
        posOffset =
            (pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos) / count;
    }
    QVector<QVector3D> offsetList;
    for (int i = 0; i < posListUp.size(); ++i) {
        offsetList.append(posOffset * (lengthListUp.at(i) / totalArcLengthUp));
    }

    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    Point pos, posAux, posEnd;
    pos.pos = posListUp.constLast().pos + translationList.constLast();
    pos.rot = newRotList.constLast();
    pos = pos.PosRelByTool(defaultDirection, defaultOffset);
    MoveL(pos, dVelocity, dAcc, dRadius);

    dVelocity = craft.cutinSpeed;
    pos.pos = posListUp.constLast().pos + translationList.constLast();
    pos.rot = newRotList.constLast();
    MoveL(pos, dVelocity, dAcc, dRadius);

    dVelocity = craft.moveSpeed;
    dAcc = 100;
    for (int i = 0; i < count + 1; ++i) {
        // 圆弧运动
        for (int j = posListUp.size() - 2; j > 0; j -= 2) {
            posAux.pos =
                posListUp.at(j).pos + posOffset * i + translationList.at(j);
            posAux.rot = newRotList.at(j);
            posEnd.pos = posListUp.at(j - 1).pos + posOffset * i +
                         translationList.at(j - 1);
            posEnd.rot = newRotList.at(j - 1);
            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        }
        if (i != count) {
            for (int j = 1; j < posListUp.size() - 1; j += 2) {
                posAux.pos = posListUp.at(j).pos + posOffset * i +
                             offsetList.at(j) + translationList.at(j);
                posAux.rot = newRotList.at(j);
                posEnd.pos = posListUp.at(j + 1).pos + posOffset * i +
                             offsetList.at(j + 1) + translationList.at(j + 1);
                posEnd.rot = newRotList.at(j + 1);
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
        } else {
            for (int j = 1; j < posListUp.size() - 1; j += 2) {
                posAux.pos =
                    posListUp.at(j).pos + posOffset * i + translationList.at(j);
                posAux.rot = newRotList.at(j);
                posEnd.pos = posListUp.at(j + 1).pos + posOffset * i +
                             translationList.at(j + 1);
                posEnd.rot = newRotList.at(j + 1);
                MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
            }
        }
    }

    return posEnd;
}

Point Robot::MoveCylinderVertical(const Craft &craft, bool isConvex) {
    // 圆弧上界
    QVector<Point> posListUp;
    posListUp.append(pointSet.beginPoint);
    posListUp.append(pointSet.midPoints);
    posListUp.append(pointSet.endPoint);

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
    QVector3D posOffset =
        pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos;
    // 最终圆弧上界与下界
    double discRadius = craft.discRadius;
    double grindAngle = craft.grindAngle;
    int count = craft.offsetCount; // 偏移次数
    QVector<QVector3D> finalPosListUp, finalPosListDown;
    QVector<QVector3D> newRotList, translationList;
    if (count > 0) {
        double unitArcLengthUp = totalArcLengthUp / count / 2;
        double arcLengthUp = 0.0;
        for (int i = 0; i < centerListUp.size(); ++i) {
            while (arcLengthUp <= lengthListUp.at(i)) {
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
                finalPosListDown.append(centerListUp.at(i) + newTrans +
                                        posOffset);
                // 计算姿态和对应偏移
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
                QVector3D rotation = Point::getNormalRotation(normal, axis);
                QVector3D moveDirection = posOffset.normalized();
                // 获取新的姿态
                newRot =
                    Point::getNewRotation(rotation, moveDirection, grindAngle);
                newRotList.append(newRot);
                // 获取新姿态需要的平移量
                translation = Point::getTranslation(rotation, moveDirection,
                                                    discRadius, grindAngle);
                translationList.append(translation);
                arcLengthUp += unitArcLengthUp;
            }
            arcLengthUp -= lengthListUp.at(i);
        }
    } else {
        finalPosListUp.append(pointSet.beginPoint.pos);
        finalPosListDown.append(pointSet.beginOffsetPoint.pos);
        // 计算姿态和对应偏移
        QVector3D axis =
            QVector3D::crossProduct(posListUp.at(0).pos - centerListUp.at(0),
                                    posListUp.at(1).pos - centerListUp.at(0))
                .normalized();
        QVector3D trans = posListUp.at(0).pos - centerListUp.at(0);
        QVector3D aux = posOffset.normalized();
        QVector3D normal =
            QVector3D::crossProduct(QVector3D::crossProduct(aux, -trans), aux)
                .normalized();
        if (!isConvex) {
            normal = -normal;
            axis = -axis;
        }
        if (craft.isMirror) {
            axis = -axis;
        }
        QVector3D rotation = Point::getNormalRotation(normal, axis);
        QVector3D moveDirection = posOffset.normalized();
        // 获取新的姿态
        newRot = Point::getNewRotation(rotation, moveDirection, grindAngle);
        newRotList.append(newRot);
        // 获取新姿态需要的平移量
        translation = Point::getTranslation(rotation, moveDirection, discRadius,
                                            grindAngle);
        translationList.append(translation);
    }
    Q_ASSERT(finalPosListUp.size() == finalPosListDown.size());
    Q_ASSERT(finalPosListUp.size() == newRotList.size());
    Q_ASSERT(finalPosListUp.size() == translationList.size());

    // 定义运动速度
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = craft.transitionRadius;

    // 定义空间目标位置
    Point pos, posAux, posEnd;

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
    for (int i = 0; i < count + 1; ++i) {
        pos.pos = finalPosListUp.at(2 * i) + translationList.at(2 * i);
        pos.rot = newRotList.at(2 * i);
        MoveL(pos, dVelocity, dAcc, dRadius);
        if (i != count) {
            posAux.pos = (finalPosListUp.at(2 * i + 1) +
                          finalPosListDown.at(2 * i + 1)) /
                             2 +
                         translationList.at(2 * i + 1);
            posAux.rot = newRotList.at(2 * i + 1);
            posEnd.pos =
                finalPosListDown.at(2 * i + 2) + translationList.at(2 * i + 2);
            posEnd.rot = newRotList.at(2 * i + 2);
            MoveC(posAux, posEnd, dVelocity, dAcc, dRadius);
        } else {
            pos.pos = finalPosListDown.at(2 * i) + translationList.at(2 * i);
            pos.rot = newRotList.at(2 * i);
            MoveL(pos, dVelocity, dAcc, dRadius);
        }
    }

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

void Robot::Run(const Craft &craft, bool isAGPRun) {
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
    MoveBefore(craft, isAGPRun);
    // Point point = pointSet.auxEndPoint;
    Point point;
    point.pos = pointSet.endPoint.pos + translation;
    point.rot = newRot;
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
        point = MoveRegionArcVertical(craft);
        break;
    case PolishWay::RegionArcWay_Vertical_Repeat:
        point = MoveRegionArcVerticalRepeat(craft);
        break;
    case PolishWay::CylinderWay_Horizontal_Convex:
        point = MoveCylinderHorizontal(craft, true);
        break;
    case PolishWay::CylinderWay_Vertical_Convex:
        point = MoveCylinderVertical(craft, true);
        break;
    case PolishWay::CylinderWay_Horizontal_Concave:
        point = MoveCylinderHorizontal(craft, false);
        break;
    case PolishWay::CylinderWay_Vertical_Concave:
        point = MoveCylinderVertical(craft, false);
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
    MoveAfter(craft, point);
    // 等待运动完成
    while (true) {
        if (!IsRobotMoved()) {
            isStop.store(true);
            break;
        }
        QThread::msleep(100);
    }
}

HansRobot::HansRobot() {}

HansRobot::~HansRobot() {
    HRIF_GrpCloseFreeDriver(0, 0);
    HRIF_GrpDisable(0, 0);
}

bool HansRobot::RobotConnect(QString robotIP) {
    int nRet = -1;
    std::string ip = robotIP.toStdString();
    const char *hostname = ip.c_str();
    unsigned short nPort = 10003;
    nRet = HRIF_Connect(0, hostname, nPort);
    if (nRet == 0) {
        // 机器人上电
        HRIF_Electrify(0);
        // 机器人使能
        HRIF_GrpEnable(0, 0);
        // 设置速度比
        HRIF_SetOverride(0, 0, 1.0);
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
        return true;
    } else {
        return false;
    }
}
/*
void HansRobot::MoveBefore(const Craft &craft, bool isAGPRun) {
    // 定义运动类型
    int nMoveType = 1;
    // 定义空间目标位置
    Point point;
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
    double dVelocity = defaultVelocity;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = 1;
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
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    // 移到安全点
    point = pointSet.safePoint;
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    // AGP运行
    AGPRun(craft, isAGPRun);
    // 移到起始辅助点
    point = pointSet.auxBeginPoint.PosRelByTool(direction, offset);
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    // 移到起始点
    point = pointSet.beginPoint.PosRelByTool(direction, offset);
    dVelocity = craft.cutinSpeed;
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
}

void HansRobot::MoveAfter(const Craft &craft, Point point) {
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
    double dVelocity = craft.cutinSpeed;
    // 定义运动加速度
    double dAcc = 2000;
    // 定义过渡半径
    double dRadius = 1;
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
    // 移到结束辅助点
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    // 移到安全点
    point = pointSet.safePoint;
    // 定义运动速度
    dVelocity = defaultVelocity;
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
}

void HansRobot::MoveLine(const Craft &craft) {
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
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
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
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;

    Point point;
    for (int i = 0; i < pointSet.midPoints.size(); ++i) {
        // 定义空间目标位置
        point = pointSet.midPoints[i].PosRelByTool(direction, offset);
        // 执行路点运动
        HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                      point.pos.z(), point.rot.x(), point.rot.y(),
                      point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                      sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                      nIOBit, nIOState, strCmdID);
    }
    // 移到结束点
    point = pointSet.endPoint.PosRelByTool(direction, offset);
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
}

void HansRobot::MoveArc(const Craft &craft) {
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
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
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
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    // 定义空间目标位置
    Point posMidRel = pointSet.auxPoint.PosRelByTool(direction, offset);
    Point posEndRel = pointSet.endPoint.PosRelByTool(direction, offset);
    // 执行路点运动
    HRIF_WayPoint2(0, 0, nMoveType, posEndRel.pos.x(), posEndRel.pos.y(),
                   posEndRel.pos.z(), posEndRel.rot.x(), posEndRel.rot.y(),
                   posEndRel.rot.z(), posMidRel.pos.x(), posMidRel.pos.y(),
                   posMidRel.pos.z(), posMidRel.rot.x(), posMidRel.rot.y(),
                   posMidRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                   sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                   nIOBit, nIOState, strCmdID);
}

// 平面圆弧
// Position HansRobot::MoveRegionArc(double offset, OffsetDirection direction)
// {
//     // 定义运动类型
//     int nMoveType = 2;
//     // 定义关节目标位置
//     double dJ1 = 0;
//     double dJ2 = 0;
//     double dJ3 = 0;
//     double dJ4 = 0;
//     double dJ5 = 0;
//     double dJ6 = 0;
//     // 定义工具坐标变量
//     string sTcpName = "TCP_AGP";
//     // 定义用户坐标变量
//     string sUcsName = "Base";
//     // 定义运动速度
//     double dVelocity = crafts.at(currCraftIdx).moveSpeed;
//     // 定义运动加速度
//     double dAcc = 100;
//     // 定义过渡半径
//     double dRadius = 1;
//     // 定义是否使用关节角度
//     int nIsUseJoint = 1;
//     // 定义是否使用检测 DI 停止
//     int nIsSeek = 0;
//     // 定义检测的 DI 索引
//     int nIOBit = 0;
//     // 定义检测的 DI 状态
//     int nIOState = 0;
//     // 定义路点 ID
//     string strCmdID = "0";
//     // 计算单次偏移量
//     int count = crafts.at(currCraftIdx).offsetCount;
//     Position beginOffset = (beginOffsetPoint - beginPoint) / count;
//     Position endOffset = (endOffsetPoint - endPoint) / count;
//     Position midOffset = (beginOffset + endOffset) / 2;
//     // 定义空间目标位置
//     Position posBeginRel = PosRelByTool(beginPoint, offset, direction);
//     Position posEndRel = PosRelByTool(endPoint, offset, direction);
//     Position posMidRel = PosRelByTool(auxPoint, offset, direction);
//     // 正向圆弧运动
//     HRIF_WayPoint2(0, 0, nMoveType, posEndRel.x, posEndRel.y, posEndRel.z,
//                    posBeginRel.rx, posBeginRel.ry, posBeginRel.rz,
//                    posMidRel.x, posMidRel.y, posMidRel.z, posBeginRel.rx,
//                    posBeginRel.ry, posBeginRel.rz, dJ1, dJ2, dJ3, dJ4, dJ5,
//                    dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                    nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//     Position pos;
//     for (int i = 0; i < count; ++i) {
//         if (i % 2 == 0) {
//             // 抬高
//             pos = posEndRel;
//             pos.rx = posBeginRel.rx;
//             pos.ry = posBeginRel.ry;
//             pos.rz = posBeginRel.rz;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             nMoveType = 1;
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 改变位姿
//             posBeginRel += beginOffset;
//             posEndRel += endOffset;
//             posMidRel += midOffset;
//             pos = posEndRel;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 压低
//             HRIF_WayPoint(0, 0, nMoveType, posEndRel.x, posEndRel.y,
//                           posEndRel.z, posEndRel.rx, posEndRel.ry,
//                           posEndRel.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
//                           sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                           nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//             // 反向圆弧运动
//             nMoveType = 2;
//             HRIF_WayPoint2(0, 0, nMoveType, posBeginRel.x, posBeginRel.y,
//                            posBeginRel.z, posEndRel.rx, posEndRel.ry,
//                            posEndRel.rz, posMidRel.x, posMidRel.y,
//                            posMidRel.z, posEndRel.rx, posEndRel.ry,
//                            posEndRel.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
//                            sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                            nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//         } else {
//             // 抬高
//             pos = posBeginRel;
//             pos.rx = posEndRel.rx;
//             pos.ry = posEndRel.ry;
//             pos.rz = posEndRel.rz;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             nMoveType = 1;
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 改变位姿
//             posBeginRel += beginOffset;
//             posEndRel += endOffset;
//             posMidRel += midOffset;
//             pos = posBeginRel;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 压低
//             HRIF_WayPoint(0, 0, nMoveType, posBeginRel.x, posBeginRel.y,
//                           posBeginRel.z, posBeginRel.rx, posBeginRel.ry,
//                           posBeginRel.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
//                           sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                           nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//             // 正向圆弧运动
//             nMoveType = 2;
//             HRIF_WayPoint2(0, 0, nMoveType, posEndRel.x, posEndRel.y,
//                            posEndRel.z, posBeginRel.rx, posBeginRel.ry,
//                            posBeginRel.rz, posMidRel.x, posMidRel.y,
//                            posMidRel.z, posBeginRel.rx, posBeginRel.ry,
//                            posBeginRel.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
//                            sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                            nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//         }
//     }
//     return count % 2 == 0
//                ? Position{posEndRel.x,    posEndRel.y,    posEndRel.z,
//                           posBeginRel.rx, posBeginRel.ry, posBeginRel.rz}
//                : Position{posBeginRel.x, posBeginRel.y, posBeginRel.z,
//                           posEndRel.rx,  posEndRel.ry,  posEndRel.rz};
// }

// 柱面圆弧，辅助点姿态取平均
// Position HansRobot::MoveRegionArc(double offset, OffsetDirection direction)
// {
//     // 定义运动类型
//     int nMoveType = 2;
//     // 定义关节目标位置
//     double dJ1 = 0;
//     double dJ2 = 0;
//     double dJ3 = 0;
//     double dJ4 = 0;
//     double dJ5 = 0;
//     double dJ6 = 0;
//     // 定义工具坐标变量
//     string sTcpName = "TCP_AGP";
//     // 定义用户坐标变量
//     string sUcsName = "Base";
//     // 定义运动速度
//     double dVelocity = crafts.at(currCraftIdx).moveSpeed;
//     // 定义运动加速度
//     double dAcc = 100;
//     // 定义过渡半径
//     double dRadius = 1;
//     // 定义是否使用关节角度
//     int nIsUseJoint = 1;
//     // 定义是否使用检测 DI 停止
//     int nIsSeek = 0;
//     // 定义检测的 DI 索引
//     int nIOBit = 0;
//     // 定义检测的 DI 状态
//     int nIOState = 0;
//     // 定义路点 ID
//     string strCmdID = "0";
//     // 计算单次偏移量
//     int count = crafts.at(currCraftIdx).offsetCount;
//     Position beginOffset = (beginOffsetPoint - beginPoint) / count;
//     Position endOffset = (endOffsetPoint - endPoint) / count;
//     Position midOffset = (beginOffset + endOffset) / 2;
//     // 定义空间目标位置
//     Position posBeginRel = PosRelByTool(beginPoint, offset, direction);
//     Position posEndRel = PosRelByTool(endPoint, offset, direction);
//     Position posMidRel = PosRelByTool(auxPoint, offset, direction);
//     // 正向圆弧运动，辅助点姿态取起始和结束点姿态的平均值
//     HRIF_WayPoint2(0, 0, nMoveType, posEndRel.x, posEndRel.y, posEndRel.z,
//                    posEndRel.rx, posEndRel.ry, posEndRel.rz, posMidRel.x,
//                    posMidRel.y, posMidRel.z,
//                    (posBeginRel.rx + posEndRel.rx) / 2,
//                    (posBeginRel.ry + posEndRel.ry) / 2,
//                    (posBeginRel.rz + posEndRel.rz) / 2, dJ1, dJ2, dJ3, dJ4,
//                    dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                    nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//     Position pos;
//     for (int i = 0; i < count; ++i) {
//         if (i % 2 == 0) { // 反向
//             // 抬高
//             pos = posEndRel;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             nMoveType = 1;
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 改变位姿
//             posBeginRel += beginOffset;
//             posEndRel += endOffset;
//             posMidRel += midOffset;
//             pos = posEndRel;
//             pos.rx = endOffsetPoint.rx;
//             pos.ry = endOffsetPoint.ry;
//             pos.rz = endOffsetPoint.rz;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 压低
//             pos = posEndRel;
//             pos.rx = endOffsetPoint.rx;
//             pos.ry = endOffsetPoint.ry;
//             pos.rz = endOffsetPoint.rz;
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 反向圆弧运动
//             nMoveType = 2;
//             HRIF_WayPoint2(
//                 0, 0, nMoveType, posBeginRel.x, posBeginRel.y, posBeginRel.z,
//                 beginOffsetPoint.rx, beginOffsetPoint.ry,
//                 beginOffsetPoint.rz, posMidRel.x, posMidRel.y, posMidRel.z,
//                 (beginOffsetPoint.rx + endOffsetPoint.rx) / 2,
//                 (beginOffsetPoint.ry + endOffsetPoint.ry) / 2,
//                 (beginOffsetPoint.rz + endOffsetPoint.rz) / 2, dJ1, dJ2, dJ3,
//                 dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
//                 nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
//         } else { // 正向
//             // 抬高
//             pos = posBeginRel;
//             pos.rx = beginOffsetPoint.rx;
//             pos.ry = beginOffsetPoint.ry;
//             pos.rz = beginOffsetPoint.rz;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             nMoveType = 1;
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 改变位姿
//             posBeginRel += beginOffset;
//             posEndRel += endOffset;
//             posMidRel += midOffset;
//             pos = posBeginRel;
//             pos = PosRelByTool(pos, defaultOffset, defaultDirection);
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 压低
//             pos = posBeginRel;
//             HRIF_WayPoint(0, 0, nMoveType, pos.x, pos.y, pos.z, pos.rx,
//             pos.ry,
//                           pos.rz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                           sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
//                           nIsSeek, nIOBit, nIOState, strCmdID);
//             // 正向圆弧运动
//             nMoveType = 2;
//             HRIF_WayPoint2(
//                 0, 0, nMoveType, posEndRel.x, posEndRel.y, posEndRel.z,
//                 posEndRel.rx, posEndRel.ry, posEndRel.rz, posMidRel.x,
//                 posMidRel.y, posMidRel.z, (posBeginRel.rx + posEndRel.rx) /
//                 2, (posBeginRel.ry + posEndRel.ry) / 2, (posBeginRel.rz +
//                 posEndRel.rz) / 2, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
//                 sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
//                 nIOBit, nIOState, strCmdID);
//         }
//     }
//     return count % 2 == 0 ? Position{posEndRel.x,  posEndRel.y,  posEndRel.z,
//                                      posEndRel.rx, posEndRel.ry,
//                                      posEndRel.rz}
//                           : Position{posBeginRel.x,       posBeginRel.y,
//                                      posBeginRel.z, beginOffsetPoint.rx,
//                                      beginOffsetPoint.ry,
//                                      beginOffsetPoint.rz};
// }

// 柱面圆弧，辅助点姿态不取平均
Point HansRobot::MoveRegionArc1(const Craft &craft) {
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
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
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
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    // 计算单次偏移量
    int count = craft.offsetCount;
    Point beginOffset;
    beginOffset.pos =
        (pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos) / count;
    Point endOffset;
    endOffset.pos =
        (pointSet.endOffsetPoint.pos - pointSet.endPoint.pos) / count;
    Point midOffset;
    midOffset.pos = (beginOffset.pos + endOffset.pos) / 2;
    // 定义空间目标位置
    Point posBeginRel = pointSet.beginPoint.PosRelByTool(direction, offset);
    Point posEndRel = pointSet.endPoint.PosRelByTool(direction, offset);
    Point posMidRel = pointSet.auxPoint.PosRelByTool(direction, offset);
    // 正向圆弧运动
    HRIF_WayPoint2(0, 0, nMoveType, posEndRel.pos.x(), posEndRel.pos.y(),
                   posEndRel.pos.z(), posEndRel.rot.x(), posEndRel.rot.y(),
                   posEndRel.rot.z(), posMidRel.pos.x(), posMidRel.pos.y(),
                   posMidRel.pos.z(), posMidRel.rot.x(), posMidRel.rot.y(),
                   posMidRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                   sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                   nIOBit, nIOState, strCmdID);
    Point point;
    for (int i = 0; i < count; ++i) {
        if (i % 2 == 0) { // 反向
            // 抬高
            point = posEndRel;
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            nMoveType = 1;
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 改变位姿
            posBeginRel += beginOffset;
            posEndRel += endOffset;
            posMidRel += midOffset;
            point = posEndRel;
            point.rot.setX(pointSet.endOffsetPoint.rot.x());
            point.rot.setY(pointSet.endOffsetPoint.rot.y());
            point.rot.setZ(pointSet.endOffsetPoint.rot.z());
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 压低
            point = posEndRel;
            point.rot.setX(pointSet.endOffsetPoint.rot.x());
            point.rot.setY(pointSet.endOffsetPoint.rot.y());
            point.rot.setZ(pointSet.endOffsetPoint.rot.z());
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 反向圆弧运动
            nMoveType = 2;
            HRIF_WayPoint2(0, 0, nMoveType, posBeginRel.pos.x(),
                           posBeginRel.pos.y(), posBeginRel.pos.z(),
                           pointSet.beginOffsetPoint.rot.x(),
                           pointSet.beginOffsetPoint.rot.y(),
                           pointSet.beginOffsetPoint.rot.z(), posMidRel.pos.x(),
                           posMidRel.pos.y(), posMidRel.pos.z(),
                           (posMidRel.rot.x() - posBeginRel.rot.x() +
                            pointSet.beginOffsetPoint.rot.x()),
                           (posMidRel.rot.y() - posBeginRel.rot.y() +
                            pointSet.beginOffsetPoint.rot.y()),
                           (posMidRel.rot.z() - posBeginRel.rot.z() +
                            pointSet.beginOffsetPoint.rot.z()),
                           dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
                           dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                           nIOBit, nIOState, strCmdID);
        } else { // 正向
            // 抬高
            point = posBeginRel;
            point.rot.setX(pointSet.beginOffsetPoint.rot.x());
            point.rot.setY(pointSet.beginOffsetPoint.rot.y());
            point.rot.setZ(pointSet.beginOffsetPoint.rot.z());
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            nMoveType = 1;
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 改变位姿
            posBeginRel += beginOffset;
            posEndRel += endOffset;
            posMidRel += midOffset;
            point = posBeginRel;
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 压低
            point = posBeginRel;
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 正向圆弧运动
            nMoveType = 2;
            HRIF_WayPoint2(
                0, 0, nMoveType, posEndRel.pos.x(), posEndRel.pos.y(),
                posEndRel.pos.z(), posEndRel.rot.x(), posEndRel.rot.y(),
                posEndRel.rot.z(), posMidRel.pos.x(), posMidRel.pos.y(),
                posMidRel.pos.z(), posMidRel.rot.x(), posMidRel.rot.y(),
                posMidRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                nIOBit, nIOState, strCmdID);
        }
    }
    return count % 2 == 0
               ? Point{posEndRel.pos.x(), posEndRel.pos.y(), posEndRel.pos.z(),
                       posEndRel.rot.x(), posEndRel.rot.y(), posEndRel.rot.z()}
               : Point{posBeginRel.pos.x(),
                       posBeginRel.pos.y(),
                       posBeginRel.pos.z(),
                       pointSet.beginOffsetPoint.rot.x(),
                       pointSet.beginOffsetPoint.rot.y(),
                       pointSet.beginOffsetPoint.rot.z()};
}

Point HansRobot::MoveRegionArc2(const Craft &craft) {
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
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
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
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    // 计算单次偏移量
    int count = craft.offsetCount;
    Point beginOffset;
    beginOffset.pos =
        (pointSet.beginOffsetPoint.pos - pointSet.beginPoint.pos) / count;
    Point endOffset;
    endOffset.pos =
        (pointSet.endOffsetPoint.pos - pointSet.endPoint.pos) / count;
    Point midOffset;
    midOffset.pos = (beginOffset.pos + endOffset.pos) / 2;
    // 定义空间目标位置
    Point posBeginRel = pointSet.beginPoint.PosRelByTool(direction, offset);
    Point posEndRel = pointSet.endPoint.PosRelByTool(direction, offset);
    Point posMidRel = pointSet.auxPoint.PosRelByTool(direction, offset);
    // 正向圆弧运动
    HRIF_WayPoint2(0, 0, nMoveType, posEndRel.pos.x(), posEndRel.pos.y(),
                   posEndRel.pos.z(), posEndRel.rot.x(), posEndRel.rot.y(),
                   posEndRel.rot.z(), posMidRel.pos.x(), posMidRel.pos.y(),
                   posMidRel.pos.z(), posMidRel.rot.x(), posMidRel.rot.y(),
                   posMidRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                   sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                   nIOBit, nIOState, strCmdID);
    Point point;
    for (int i = 0; i < count; ++i) {
        if (i % 2 == 0) { // 反向
            // 抬高
            point = posEndRel;
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            nMoveType = 1;
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 改变位姿
            posBeginRel += beginOffset;
            posEndRel += endOffset;
            posMidRel += midOffset;
            point = posEndRel;
            point.rot.setX(pointSet.endOffsetPoint.rot.x());
            point.rot.setY(pointSet.endOffsetPoint.rot.y());
            point.rot.setZ(pointSet.endOffsetPoint.rot.z());
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 压低
            point = posEndRel;
            point.rot.setX(pointSet.endOffsetPoint.rot.x());
            point.rot.setY(pointSet.endOffsetPoint.rot.y());
            point.rot.setZ(pointSet.endOffsetPoint.rot.z());
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 反向圆弧运动
            nMoveType = 2;
            HRIF_WayPoint2(0, 0, nMoveType, posBeginRel.pos.x(),
                           posBeginRel.pos.y(), posBeginRel.pos.z(),
                           pointSet.beginOffsetPoint.rot.x(),
                           pointSet.beginOffsetPoint.rot.y(),
                           pointSet.beginOffsetPoint.rot.z(), posMidRel.pos.x(),
                           posMidRel.pos.y(), posMidRel.pos.z(),
                           (posMidRel.rot.x() - posBeginRel.rot.x() +
                            pointSet.beginOffsetPoint.rot.x()),
                           (posMidRel.rot.y() - posBeginRel.rot.y() +
                            pointSet.beginOffsetPoint.rot.y()),
                           (posMidRel.rot.z() - posBeginRel.rot.z() +
                            pointSet.beginOffsetPoint.rot.z()),
                           dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
                           dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                           nIOBit, nIOState, strCmdID);
        } else { // 正向
            // 抬高
            point = posBeginRel;
            point.rot.setX(pointSet.beginOffsetPoint.rot.x());
            point.rot.setY(pointSet.beginOffsetPoint.rot.y());
            point.rot.setZ(pointSet.beginOffsetPoint.rot.z());
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            nMoveType = 1;
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 改变位姿
            posBeginRel += beginOffset;
            posEndRel += endOffset;
            posMidRel += midOffset;
            point = posBeginRel;
            point = point.PosRelByTool(defaultDirection, defaultOffset);
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 压低
            point = posBeginRel;
            HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(),
                          point.pos.z(), point.rot.x(), point.rot.y(),
                          point.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                          sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint,
                          nIsSeek, nIOBit, nIOState, strCmdID);
            // 正向圆弧运动
            nMoveType = 2;
            HRIF_WayPoint2(
                0, 0, nMoveType, posEndRel.pos.x(), posEndRel.pos.y(),
                posEndRel.pos.z(), posEndRel.rot.x(), posEndRel.rot.y(),
                posEndRel.rot.z(), posMidRel.pos.x(), posMidRel.pos.y(),
                posMidRel.pos.z(), posMidRel.rot.x(), posMidRel.rot.y(),
                posMidRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                nIOBit, nIOState, strCmdID);
        }
    }
    return count % 2 == 0
               ? Point{posEndRel.pos.x(), posEndRel.pos.y(), posEndRel.pos.z(),
                       posEndRel.rot.x(), posEndRel.rot.y(), posEndRel.rot.z()}
               : Point{posBeginRel.pos.x(),
                       posBeginRel.pos.y(),
                       posBeginRel.pos.z(),
                       pointSet.beginOffsetPoint.rot.x(),
                       pointSet.beginOffsetPoint.rot.y(),
                       pointSet.beginOffsetPoint.rot.z()};
}

void HansRobot::MoveZLine(const Craft &craft) {
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
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
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

    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    int size = craft.offsetCount + 1;
    float factor = 1.0 / size;

    Point point = pointSet.beginPoint;
    Point pointRel;
    Point pointOffset;
    pointOffset.pos = pointSet.auxPoint.pos - pointSet.beginPoint.pos;
    pointOffset.rot = pointSet.auxPoint.rot - pointSet.beginPoint.rot;
    for (int i = 1; i <= size; ++i) {
        point += pointOffset;
        pointRel = point.PosRelByTool(direction, offset);
        HRIF_WayPoint(0, 0, nMoveType, pointRel.pos.x(), pointRel.pos.y(),
                      pointRel.pos.z(), pointRel.rot.x(), pointRel.rot.y(),
                      pointRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                      sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                      nIOBit, nIOState, strCmdID);
        point =
            Point::scale(pointSet.beginPoint, pointSet.endPoint, factor * i);
        pointRel = point.PosRelByTool(direction, offset);
        HRIF_WayPoint(0, 0, nMoveType, pointRel.pos.x(), pointRel.pos.y(),
                      pointRel.pos.z(), pointRel.rot.x(), pointRel.rot.y(),
                      pointRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
                      sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek,
                      nIOBit, nIOState, strCmdID);
    }
}

void HansRobot::MoveSpiralLine(const Craft &craft) {
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
    double dVelocity = craft.moveSpeed;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 1;
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

    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
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
    Point pointEndRel = pointEnd.PosRelByTool(direction, offset);
    Point pointAuxRel = pointAux.PosRelByTool(direction, offset);
    // 执行路点运动
    HRIF_WayPoint2(0, 0, nMoveType, pointEndRel.pos.x(), pointEndRel.pos.y(),
                   pointEndRel.pos.z(), pointEndRel.rot.x(),
                   pointEndRel.rot.y(), pointEndRel.rot.z(),
                   pointAuxRel.pos.x(), pointAuxRel.pos.y(),
                   pointAuxRel.pos.z(), pointAuxRel.rot.x(),
                   pointAuxRel.rot.y(), pointAuxRel.rot.z(), dJ1, dJ2, dJ3, dJ4,
                   dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                   nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    for (int i = 1; i < size; ++i) {
        // 小圆弧
        pointEnd = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i));
        pointAux = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i + 1));
        pointAux.pos += downOffset;
        pointEndRel = pointEnd.PosRelByTool(direction, offset);
        pointAuxRel = pointAux.PosRelByTool(direction, offset);
        HRIF_WayPoint2(
            0, 0, nMoveType, pointEndRel.pos.x(), pointEndRel.pos.y(),
            pointEndRel.pos.z(), pointEndRel.rot.x(), pointEndRel.rot.y(),
            pointEndRel.rot.z(), pointAuxRel.pos.x(), pointAuxRel.pos.y(),
            pointAuxRel.pos.z(), pointAuxRel.rot.x(), pointAuxRel.rot.y(),
            pointAuxRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
            sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,
            nIOState, strCmdID);
        // 大圆弧
        pointEnd = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i + 4));
        pointAux = Point::scale(pointSet.beginPoint, pointSet.endPoint,
                                factor * (2 * i + 2));
        pointAux.pos += upOffset;
        pointEndRel = pointEnd.PosRelByTool(direction, offset);
        pointAuxRel = pointAux.PosRelByTool(direction, offset);
        HRIF_WayPoint2(
            0, 0, nMoveType, pointEndRel.pos.x(), pointEndRel.pos.y(),
            pointEndRel.pos.z(), pointEndRel.rot.x(), pointEndRel.rot.y(),
            pointEndRel.rot.z(), pointAuxRel.pos.x(), pointAuxRel.pos.y(),
            pointAuxRel.pos.z(), pointAuxRel.rot.x(), pointAuxRel.rot.y(),
            pointAuxRel.rot.z(), dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName,
            sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,
            nIOState, strCmdID);
    }
}

void HansRobot::Run(const Craft &craft, bool isAGPRun) {
    // QThread::msleep(100);
    MoveBefore(craft, isAGPRun);
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    Point point = pointSet.auxEndPoint.PosRelByTool(direction, offset);
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
        point = point.PosRelByTool(defaultDirection, defaultOffset);
        break;
    case PolishWay::RegionArcWay2:
        point = MoveRegionArc2(craft);
        point = point.PosRelByTool(defaultDirection, defaultOffset);
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
    MoveAfter(craft, point);
}
*/
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
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 直线运动
    HRIF_WayPoint(0, 0, nMoveType, point.pos.x(), point.pos.y(), point.pos.z(),
                  point.rot.x(), point.rot.y(), point.rot.z(), dJ1, dJ2, dJ3,
                  dJ4, dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                  nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
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
/*
DucoRobot::DucoRobot() : ducoCobot(nullptr) {}

DucoRobot::~DucoRobot() {
    if (ducoCobot != nullptr) {
        ducoCobot->end_teach_mode(true);
        ducoCobot->disable(true);
        ducoCobot->close();
        delete ducoCobot;
        ducoCobot = nullptr;
    }
}

bool DucoRobot::RobotConnect(QString robotIP) {
    robotIPAddr = robotIP.toStdString();
    if (ducoCobot != nullptr) {
        delete ducoCobot;
    }
    ducoCobot = new DucoRPC::DucoCobot(robotIPAddr, 7003);
    if (ducoCobot != nullptr && ducoCobot->open() == 0) {
        // 机器人上电
        ducoCobot->power_on(true);
        // 设置速度比
        ducoCobot->speed(100);
        return true;
    }
    return false;
}

bool DucoRobot::RobotTeach(int pos) {
    qDebug() << "start";
    if (!isTeach) {
        qDebug() << "if:" << isTeach;
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
        if (!IsRobotEnabled()) {
            // 机器人使能
            qDebug() << "robot not enable1";
            ducoCobot->enable(true);
            // QThread::msleep(1500);
            if (!IsRobotEnabled()) {
                qDebug() << "robot not enable2";
                return isTeach;
            }
        }
        qDebug() << "open driver";
        // 启用自由拖拽
        int nRet = ducoCobot->teach_mode(false);
        qDebug() << "open teach: " << nRet;
        // if (nRet == 0) {
        isTeach = true;
        // }
    } else {
        qDebug() << "else:" << isTeach;
        // 关闭自由拖拽
        int nRet = ducoCobot->end_teach_mode(true);
        qDebug() << "close teach: " << nRet;
        // if (nRet == 0) {
        isTeach = false;
        // }
    }
    qDebug() << "end" << isTeach;
    return isTeach;
}

bool DucoRobot::GetTcpPoint(Point &point) {
    // 获取位姿信息
    std::vector<double> data(6);
    ducoCobot->get_tcp_pose(data);
    point.pos.setX(data.at(0));
    point.pos.setY(data.at(1));
    point.pos.setZ(data.at(2));
    point.rot.setX(data.at(3));
    point.rot.setY(data.at(4));
    point.rot.setZ(data.at(5));
    qDebug() << data;
    return true;
}

void DucoRobot::MoveBefore(DucoRPC::DucoCobot *robot, const Craft &craft,
                           bool isAGPRun) {
    vector<double> p(6);
    double v = defaultVelocity * 0.001;
    double a = 2;
    double rad = 0.001;
    vector<double> q_near(6);
    string tool = "TCP_AGP";
    string wobj = "default";
    bool block = true;
    // DucoRPC::OP op;
    // bool def_acc = true;
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;

    // 移到安全点
    Point point = pointSet.safePoint;
    robot->get_tcp_pose(p);
    qDebug() << " now:" << p;
    if (std::fabs(p[0] - point.pos.x()) > precision ||
        std::fabs(p[1] - point.pos.y()) > precision ||
        std::fabs(p[2] - point.pos.z()) > precision ||
        std::fabs(p[3] - point.rot.x()) > precision ||
        std::fabs(p[4] - point.rot.y()) > precision ||
        std::fabs(p[5] - point.rot.z()) > precision) {
        p[0] = point.pos.x();
        p[1] = point.pos.y();
        p[2] = point.pos.z();
        p[3] = point.rot.x();
        p[4] = point.rot.y();
        p[5] = point.rot.z();
        qDebug() << "move:" << p;
        status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
        qDebug() << "result:" << status;
        if (status != DucoRPC::TaskState::ST_Interrupt) {
            return;
        }
    }
    // AGP运行
    AGPRun(craft, isAGPRun);
    // 移到起始辅助点
    point = pointSet.auxBeginPoint.PosRelByTool(direction, offset);
    p[0] = point.pos.x();
    p[1] = point.pos.y();
    p[2] = point.pos.z();
    p[3] = point.rot.x();
    p[4] = point.rot.y();
    p[5] = point.rot.z();
    qDebug() << "move:" << p;
    status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
    qDebug() << "result:" << status;
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        return;
    }
    // 移到起始点
    point = pointSet.beginPoint.PosRelByTool(direction, offset);
    v = craft.cutinSpeed * 0.001;
    p[0] = point.pos.x();
    p[1] = point.pos.y();
    p[2] = point.pos.z();
    p[3] = point.rot.x();
    p[4] = point.rot.y();
    p[5] = point.rot.z();
    qDebug() << "move:" << p;
    status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
    qDebug() << "result:" << status;
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        return;
    }
}

void DucoRobot::MoveAfter(DucoRPC::DucoCobot *robot, const Craft &craft,
                          Point point) {
    vector<double> p(6);
    double v = craft.cutinSpeed * 0.001;
    double a = 2;
    double rad = 0.001;
    vector<double> q_near(6);
    string tool = "TCP_AGP";
    string wobj = "default";
    bool block = true;
    // DucoRPC::OP op;
    // bool def_acc = true;

    // 移到结束辅助点
    p[0] = point.pos.x();
    p[1] = point.pos.y();
    p[2] = point.pos.z();
    p[3] = point.rot.x();
    p[4] = point.rot.y();
    p[5] = point.rot.z();
    qDebug() << "move:" << p;
    status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
    qDebug() << "result:" << status;
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        return;
    }
    // 移到安全点
    point = pointSet.safePoint;
    // 定义运动速度
    v = defaultVelocity * 0.001;
    p[0] = point.pos.x();
    p[1] = point.pos.y();
    p[2] = point.pos.z();
    p[3] = point.rot.x();
    p[4] = point.rot.y();
    p[5] = point.rot.z();
    qDebug() << "move:" << p;
    status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
    qDebug() << "result:" << status;
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        return;
    }
}

void DucoRobot::MoveLine(DucoRPC::DucoCobot *robot, const Craft &craft) {
    vector<double> p(6);
    double v = craft.moveSpeed * 0.001;
    double a = 2;
    double rad = 0.001;
    vector<double> q_near(6);
    string tool = "TCP_AGP";
    string wobj = "default";
    bool block = true;
    // DucoRPC::OP op;
    // bool def_acc = true;
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;

    Point point;
    for (int i = 0; i < pointSet.midPoints.size(); ++i) {
        // 定义空间目标位置
        point = pointSet.midPoints[i].PosRelByTool(direction, offset);
        // 执行路点运动
        p[0] = point.pos.x();
        p[1] = point.pos.y();
        p[2] = point.pos.z();
        p[3] = point.rot.x();
        p[4] = point.rot.y();
        p[5] = point.rot.z();
        qDebug() << "move:" << p;
        status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
        qDebug() << "result:" << status;
        if (status != DucoRPC::TaskState::ST_Interrupt) {
            return;
        }
    }
    // 移到结束点
    point = pointSet.endPoint.PosRelByTool(direction, offset);
    p[0] = point.pos.x();
    p[1] = point.pos.y();
    p[2] = point.pos.z();
    p[3] = point.rot.x();
    p[4] = point.rot.y();
    p[5] = point.rot.z();
    qDebug() << "move:" << p;
    status = robot->movel(p, v, a, rad, q_near, tool, wobj, block);
    qDebug() << "result:" << status;
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        return;
    }
}

void DucoRobot::MoveArc(DucoRPC::DucoCobot *robot, const Craft &craft) {
    vector<double> p1(6);
    vector<double> p2(6);
    double v = craft.moveSpeed * 0.001;
    double a = 2;
    double r = 0.001;
    int mode = 1;
    vector<double> q_near(6);
    string tool = "TCP_AGP";
    string wobj = "default";
    bool block = true;
    // DucoRPC::OP op;
    // bool def_acc = true;
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;

    // 定义空间目标位置
    Point posMidRel = pointSet.auxPoint.PosRelByTool(direction, offset);
    p1[0] = posMidRel.pos.x();
    p1[1] = posMidRel.pos.y();
    p1[2] = posMidRel.pos.z();
    p1[3] = posMidRel.rot.x();
    p1[4] = posMidRel.rot.y();
    p1[5] = posMidRel.rot.z();
    Point posEndRel = pointSet.endPoint.PosRelByTool(direction, offset);
    p2[0] = posEndRel.pos.x();
    p2[1] = posEndRel.pos.y();
    p2[2] = posEndRel.pos.z();
    p2[3] = posEndRel.rot.x();
    p2[4] = posEndRel.rot.y();
    p2[5] = posEndRel.rot.z();
    // 执行路点运动
    status = robot->movec(p1, p2, v, a, r, mode, q_near, tool, wobj, block);
    qDebug() << "result:" << status;
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        return;
    }
}

Point DucoRobot::MoveRegionArc1(const Craft &craft) { return Point(); }
Point DucoRobot::MoveRegionArc2(const Craft &craft) { return Point(); }
void DucoRobot::MoveZLine(const Craft &craft) {}
void DucoRobot::MoveSpiralLine(const Craft &craft) {}

bool DucoRobot::IsRobotEnabled() {
    // 定义需要读取的机器人状态变量
    vector<int8_t> data;
    // 读取状态
    ducoCobot->get_robot_state(data);
    qDebug() << data;
    return data[0] == DucoRPC::StateRobot::SR_Enable ? true : false;
}

bool DucoRobot::IsRobotMoved() { return ducoCobot->robotmoving(); }

bool DucoRobot::CloseFreeDriver() {
    // 关闭自由拖拽
    int nRet = ducoCobot->end_teach_mode(true);
    qDebug() << "close teach: " << nRet;
    // if (nRet == 0) {
    isTeach = false;
    return true;
    // }
    // return false;
}

void DucoRobot::Run(const Craft &craft, bool isAGPRun) {
    DucoRPC::DucoCobot *robot = new DucoRPC::DucoCobot(robotIPAddr, 7003);
    robot->open();
    MoveBefore(robot, craft, isAGPRun);
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        robot->close();
        delete robot;
        return;
    }
    // 偏移
    OffsetDirection direction = craft.offsetDirection;
    double offset = craft.offsetDistance;
    Point point = pointSet.auxEndPoint.PosRelByTool(direction, offset);
    // 选择打磨方式
    switch (craft.way) {
    case PolishWay::ArcWay:
        MoveArc(robot, craft);
        break;
    case PolishWay::LineWay:
        MoveLine(robot, craft);
        break;
    case PolishWay::RegionArcWay1:
        point = MoveRegionArc1(craft);
        point = point.PosRelByTool(defaultDirection, defaultOffset);
        break;
    case PolishWay::RegionArcWay2:
        point = MoveRegionArc2(craft);
        point = point.PosRelByTool(defaultDirection, defaultOffset);
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
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        robot->close();
        delete robot;
        return;
    }
    MoveAfter(robot, craft, point);
    if (status != DucoRPC::TaskState::ST_Interrupt) {
        robot->close();
        delete robot;
        return;
    }
    while (true) {
        if (!IsRobotMoved()) {
            break;
        }
    }
    robot->close();
    delete robot;
}

bool DucoRobot::Stop() {
    qDebug() << "stop begin";
    // 机器人停止
    ducoCobot->stop(true);
    // AGP停止
    if (agp != nullptr) {
        agp->SetSpeed(0);
    }
    qDebug() << "stop end";
    // 机器人复位
    ducoCobot->disable(true);
    // AGP复位
    if (agp != nullptr) {
        agp->Control(FUNC::RESET);
    }
    // 自由拖拽复位
    isTeach = false;
    qDebug() << "reset end";

    return true;
}

void DucoRobot::OpenWeb(QString ip) {
    QDesktopServices::openUrl(QUrl("http://" + ip + ":7000"));
}

*/

JakaRobot::JakaRobot() {}

JakaRobot::~JakaRobot() {
    jakaRobot.drag_mode_enable(FALSE);
    // jakaRobot.disable_robot();
    // jakaRobot.login_out();
}

bool JakaRobot::RobotConnect(QString robotIP) {
    // std::string ip = robotIP.toStdString();
    // std::string ip = "192.168.1.20";
    std::string ip = "10.5.5.100";
    const char *hostname = ip.c_str();
    // 连接机器人
    errno_t ret = jakaRobot.login_in(hostname);
    if (ret != ERR_SUCC) {
        return false;
    }
    // 机器人上电
    // jakaRobot.power_on();
    // 机器人使能
    // jakaRobot.enable_robot();
    // 设置速度比
    // jakaRobot.set_rapidrate(1.0);
    return true;
}

bool JakaRobot::GetTcpPoint(Point &point) {
    CartesianPose tcp_pos;
    errno_t ret = jakaRobot.get_tcp_position(&tcp_pos);
    if (ret != ERR_SUCC) {
        return false;
    }
    point.pos.setX(tcp_pos.tran.x);
    point.pos.setY(tcp_pos.tran.y);
    point.pos.setZ(tcp_pos.tran.z);
    point.rot.setX(qRound(qRadiansToDegrees(tcp_pos.rpy.rx) * 1000.0) / 1000.0);
    point.rot.setY(qRound(qRadiansToDegrees(tcp_pos.rpy.ry) * 1000.0) / 1000.0);
    point.rot.setZ(qRound(qRadiansToDegrees(tcp_pos.rpy.rz) * 1000.0) / 1000.0);
    return true;
}

bool JakaRobot::RobotTeach(int pos) {
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
        // if (!IsRobotElectrified()) {
        //     // 机器人上电
        //     jakaRobot.power_on();
        //     if (!IsRobotElectrified()) {
        //         return isTeach;
        //     }
        // }
        if (!IsRobotEnabled()) {
            // 机器人使能
            jakaRobot.enable_robot();
            // QThread::msleep(1500);
            if (!IsRobotEnabled()) {
                return isTeach;
            }
        }
        // 启用自由拖拽
        errno_t ret = jakaRobot.drag_mode_enable(TRUE);
        if (ret == ERR_SUCC) {
            isTeach = true;
        }
    } else {
        // 关闭自由拖拽
        errno_t ret = jakaRobot.drag_mode_enable(FALSE);
        if (ret == ERR_SUCC) {
            isTeach = false;
        }
    }
    return isTeach;
}

bool JakaRobot::CloseFreeDriver() {
    // 关闭自由拖拽
    errno_t ret = jakaRobot.drag_mode_enable(FALSE);
    if (ret == ERR_SUCC) {
        isTeach = false;
        return true;
    }
    return false;
}

bool JakaRobot::Stop() {
    // 机器人停止
    jakaRobot.motion_abort();
    // AGP停止
    if (agp != nullptr) {
        agp->SetSpeed(0);
    }
    // 机器人复位
    // jakaRobot.disable_robot();
    // AGP复位
    if (agp != nullptr) {
        agp->Control(FUNC::RESET);
    }
    // 自由拖拽复位
    isTeach = false;

    return true;
}

bool JakaRobot::IsRobotElectrified() {
    RobotStatus robstatus;
    jakaRobot.get_robot_status(&robstatus);

    return robstatus.powered_on;
}

bool JakaRobot::IsRobotEnabled() {
    RobotStatus robstatus;
    jakaRobot.get_robot_status(&robstatus);

    return robstatus.enabled;
}

bool JakaRobot::IsRobotMoved() {
    BOOL in_pos;
    jakaRobot.is_in_pos(&in_pos);

    return !in_pos;
}

void JakaRobot::OpenWeb(QString ip) {}

void JakaRobot::MoveTcpL(const Point &point, double dVelocity, double dAcc,
                         double dRadius) {
    CartesianPose pos;
    pos.tran.x = point.pos.x();
    pos.tran.y = point.pos.y();
    pos.tran.z = point.pos.z();
    pos.rpy.rx = qDegreesToRadians(point.rot.x());
    pos.rpy.ry = qDegreesToRadians(point.rot.y());
    pos.rpy.rz = qDegreesToRadians(point.rot.z());

    jakaRobot.linear_move(&pos, MoveMode::ABS, FALSE, dVelocity, dAcc, 0.1,
                          NULL, 3.14 / 10, 12.56 / 10);
}

void JakaRobot::MoveTcpC(const Point &auxPoint, const Point &endPoint,
                         double dVelocity, double dAcc, double dRadius) {
    CartesianPose midPos, endPos;
    midPos.tran.x = auxPoint.pos.x();
    midPos.tran.y = auxPoint.pos.y();
    midPos.tran.z = auxPoint.pos.z();
    midPos.rpy.rx = qDegreesToRadians(auxPoint.rot.x());
    midPos.rpy.ry = qDegreesToRadians(auxPoint.rot.y());
    midPos.rpy.rz = qDegreesToRadians(auxPoint.rot.z());

    endPos.tran.x = endPoint.pos.x();
    endPos.tran.y = endPoint.pos.y();
    endPos.tran.z = endPoint.pos.z();
    endPos.rpy.rx = qDegreesToRadians(endPoint.rot.x());
    endPos.rpy.ry = qDegreesToRadians(endPoint.rot.y());
    endPos.rpy.rz = qDegreesToRadians(endPoint.rot.z());

    jakaRobot.circular_move(&endPos, &midPos, MoveMode::ABS, FALSE, dVelocity,
                            dAcc, 0.1, NULL);
}
