#include <QGenericMatrix>
#include <QMatrix4x4>
#include <QQuaternion>

#include "point.h"

Point::Point() {}

Point::Point(float x, float y, float z, float rx, float ry, float rz)
    : pos(x, y, z), rot(rx, ry, rz) {}

// 计算工具方向向量
// static Eigen::Vector3d calculateToolDirection(double a, double b, double c,
//                                               OffsetDirection direction) {
QVector3D Point::calculateToolDirection(OffsetDirection direction,
                                        QVector3D rotation) const {
    // 转换角度为弧度
    rotation.setX(qDegreesToRadians(rotation.x()));
    rotation.setY(qDegreesToRadians(rotation.y()));
    rotation.setZ(qDegreesToRadians(rotation.z()));
    // rotation.setX(rotation.x());
    // rotation.setY(rotation.y());
    // rotation.setZ(rotation.z());

    // 定义旋转矩阵
    QMatrix3x3 R_x;
    R_x.setToIdentity();
    R_x(1, 1) = qCos(rotation.x());
    R_x(1, 2) = -qSin(rotation.x());
    R_x(2, 1) = qSin(rotation.x());
    R_x(2, 2) = qCos(rotation.x());

    QMatrix3x3 R_y;
    R_y.setToIdentity();
    R_y(0, 0) = qCos(rotation.y());
    R_y(0, 2) = qSin(rotation.y());
    R_y(2, 0) = -qSin(rotation.y());
    R_y(2, 2) = qCos(rotation.y());

    QMatrix3x3 R_z;
    R_z.setToIdentity();
    R_z(0, 0) = qCos(rotation.z());
    R_z(0, 1) = -qSin(rotation.z());
    R_z(1, 0) = qSin(rotation.z());
    R_z(0, 1) = qCos(rotation.z());

    // 计算总的旋转矩阵
    QMatrix3x3 R = R_z * R_y * R_x;

    // 工具方向向量
    QVector3D default_direction(0, 0, 0);
    switch (direction) {
    case OffsetDirection::OffsetX:
        default_direction.setX(1);
        break;
    case OffsetDirection::OffsetY:
        default_direction.setY(1);
        break;
    case OffsetDirection::OffsetZ:
        default_direction.setZ(1);
        break;
    default:
        break;
    }

    // 计算工具方向向量
    // return R * default_direction;
    return QVector3D(
        R(0, 0) * default_direction.x() + R(0, 1) * default_direction.y() +
            R(0, 2) * default_direction.z(),
        R(1, 0) * default_direction.x() + R(1, 1) * default_direction.y() +
            R(1, 2) * default_direction.z(),
        R(2, 0) * default_direction.x() + R(2, 1) * default_direction.y() +
            R(2, 2) * default_direction.z());
}

// 计算新的坐标
// static Position PosRelByTool(Position oldPos, double offset,
//                              OffsetDirection direction) {
Point Point::PosRelByTool(const OffsetDirection &direction,
                          const double &offset) const {
    // 计算工具方向向量
    QVector3D tool_direction = calculateToolDirection(direction, rot);
    // 计算新的坐标
    Point newPoint;
    newPoint.pos.setX(pos.x() + offset * tool_direction.x());
    newPoint.pos.setY(pos.y() + offset * tool_direction.y());
    newPoint.pos.setZ(pos.z() + offset * tool_direction.z());
    // newPoint.pos.setX(pos.x() + offset * tool_direction.x() * 0.001);
    // newPoint.pos.setY(pos.y() + offset * tool_direction.y() * 0.001);
    // newPoint.pos.setZ(pos.z() + offset * tool_direction.z() * 0.001);
    newPoint.rot.setX(rot.x());
    newPoint.rot.setY(rot.y());
    newPoint.rot.setZ(rot.z());

    return newPoint;
}

void Point::operator+=(const Point &point) {
    pos += point.pos;
    // rot += point.rot;
}

Point Point::scale(const Point &pointBegin, const Point &pointEnd, float t) {
    Point point;
    point.pos = (pointEnd.pos - pointBegin.pos) * t + pointBegin.pos;
    // point.rot =
    //     QQuaternion::nlerp(QQuaternion::fromEulerAngles(pointBegin.rot),
    //                        QQuaternion::fromEulerAngles(pointEnd.rot), t)
    //         .toEulerAngles();
    point.rot = pointBegin.rot;
    return point;
}

QVector3D Point::calculateCircumcenter(const QVector3D &A, const QVector3D &B,
                                       const QVector3D &C) {
    QVector3D AB = B - A;
    QVector3D AC = C - A;
    QVector3D N = QVector3D::crossProduct(AB, AC);

    QVector3D toCircumcenter =
        (QVector3D::crossProduct(N, AB) * AC.lengthSquared() +
         QVector3D::crossProduct(AC, N) * AB.lengthSquared()) /
        (2.f * N.lengthSquared());
    // float radius = toCircumcenter.length();

    QVector3D circumcenter = A + toCircumcenter;

    return circumcenter;
}

void Point::test() {
    // 定义三个点
    QVector3D A(1, 2, 3);
    QVector3D B(4, 6, 8);
    QVector3D C(7, 8, 9);

    // 计算外心
    QVector3D circumcenter = Point::calculateCircumcenter(A, B, C);

    // 输出外心坐标
    qDebug() << "The circumcenter of the triangle is at:" << circumcenter;

    // 输出半径
    qDebug() << "radius: " << A.distanceToPoint(circumcenter);
    qDebug() << "radius: " << B.distanceToPoint(circumcenter);
    qDebug() << "radius: " << C.distanceToPoint(circumcenter);
}

PointSet::PointSet()
    : isSafePointRecorded(false), isBeginPointRecorded(false),
      isEndPointRecorded(false), isAuxPointRecorded(false),
      isBeginOffsetPointRecorded(false), isEndOffsetPointRecorded(false) {}
