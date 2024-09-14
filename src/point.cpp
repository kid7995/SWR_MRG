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
    R_z(1, 1) = qCos(rotation.z());

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

QString Point::toString() const {
    QString str = QString("%1、%2、%3、%4、%5、%6")
                      .arg(pos.x())
                      .arg(pos.y())
                      .arg(pos.z())
                      .arg(rot.x())
                      .arg(rot.y())
                      .arg(rot.z());
    return str;
}

void Point::operator+=(const Point &point) {
    pos += point.pos;
    // rot += point.rot;
}

QVector3D Point::gemv(const QMatrix3x3 &matrix, const QVector3D &vector) {
    return QVector3D(matrix(0, 0) * vector.x() + matrix(0, 1) * vector.y() +
                         matrix(0, 2) * vector.z(),
                     matrix(1, 0) * vector.x() + matrix(1, 1) * vector.y() +
                         matrix(1, 2) * vector.z(),
                     matrix(2, 0) * vector.x() + matrix(2, 1) * vector.y() +
                         matrix(2, 2) * vector.z());
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

QVector3D
Point::calculateSpherecenter(const QVector3D &A, const QVector3D &B,
                             const QVector3D &C,
                             const QVector3D &D) { // todo 共面等情况处理
    QVector3D circumcenter = Point::calculateCircumcenter(A, B, C);
    QVector3D N = QVector3D::crossProduct(B - A, C - A).normalized();

    double toLine = D.distanceToLine(circumcenter, N);
    double toPlane = D.distanceToPlane(circumcenter, N);
    QVector3D toSpherecenter = ((qPow(toLine, 2) + qPow(toPlane, 2) -
                                 (A - circumcenter).lengthSquared()) /
                                (2.f * toPlane)) *
                               N;

    QVector3D spherecenter = circumcenter + toSpherecenter;

    return spherecenter;
}

QMatrix3x3 Point::toRotationMatrix(const QVector3D &rotation) {
    // 转换角度为弧度
    QVector3D rot_rad{qDegreesToRadians(rotation.x()),
                      qDegreesToRadians(rotation.y()),
                      qDegreesToRadians(rotation.z())};

    // 定义旋转矩阵
    QMatrix3x3 R_x;
    R_x.setToIdentity();
    R_x(1, 1) = qCos(rot_rad.x());
    R_x(1, 2) = -qSin(rot_rad.x());
    R_x(2, 1) = qSin(rot_rad.x());
    R_x(2, 2) = qCos(rot_rad.x());

    QMatrix3x3 R_y;
    R_y.setToIdentity();
    R_y(0, 0) = qCos(rot_rad.y());
    R_y(0, 2) = qSin(rot_rad.y());
    R_y(2, 0) = -qSin(rot_rad.y());
    R_y(2, 2) = qCos(rot_rad.y());

    QMatrix3x3 R_z;
    R_z.setToIdentity();
    R_z(0, 0) = qCos(rot_rad.z());
    R_z(0, 1) = -qSin(rot_rad.z());
    R_z(1, 0) = qSin(rot_rad.z());
    R_z(1, 1) = qCos(rot_rad.z());

    // 返回总的旋转矩阵
    return R_z * R_y * R_x;
}

QMatrix3x3 Point::toRotationMatrix(const QVector3D &axis, float angle) {
    return QQuaternion::fromAxisAndAngle(axis, angle).toRotationMatrix();
}

QVector3D Point::toEulerAngles(const QMatrix3x3 &matrix) {
    double sy = qSqrt(qPow(matrix(0, 0), 2) + qPow(matrix(1, 0), 2));
    double rx, ry, rz;
    if (sy < 1e-6) {
        rx = qAtan2(-matrix(1, 2), matrix(1, 1));
        ry = qAtan2(-matrix(2, 0), sy);
        rz = 0.0;
    } else {
        rx = qAtan2(matrix(2, 1), matrix(2, 2));
        ry = qAtan2(-matrix(2, 0), sy);
        rz = qAtan2(matrix(1, 0), matrix(0, 0));
    }
    return QVector3D(qRadiansToDegrees(rx), qRadiansToDegrees(ry),
                     qRadiansToDegrees(rz));
}

QVector3D Point::getNewRotation(const QVector3D &rotation,
                                const QVector3D &moveDirection, float angle) {
    QMatrix3x3 R = toRotationMatrix(rotation);
    QVector3D axis = QVector3D::crossProduct(
        QVector3D(R(0, 2), R(1, 2), R(2, 2)), moveDirection);
    QMatrix3x3 newR = toRotationMatrix(axis, angle) * R;
    return toEulerAngles(newR);
}

QVector3D Point::getTranslation(const QVector3D &rotation,
                                const QVector3D &moveDirection, float radius,
                                float angle) {
    QVector3D translation = moveDirection.normalized() * radius;
    QMatrix3x3 R = toRotationMatrix(rotation);
    QVector3D axis = QVector3D::crossProduct(
        QVector3D(R(0, 2), R(1, 2), R(2, 2)), moveDirection);
    QMatrix3x3 newR = toRotationMatrix(axis, angle);
    return QVector3D(
        newR(0, 0) * translation.x() + newR(0, 1) * translation.y() +
            newR(0, 2) * translation.z(),
        newR(1, 0) * translation.x() + newR(1, 1) * translation.y() +
            newR(1, 2) * translation.z(),
        newR(2, 0) * translation.x() + newR(2, 1) * translation.y() +
            newR(2, 2) * translation.z());
}

void Point::test() {
    // 定义四个点
    // QVector3D A(1, 0, 0);
    // QVector3D B(0, 1, 0);
    // QVector3D C(0, -1, 0);
    // QVector3D D(0, 0, 1);

    // QVector3D A(2224.050, -124.478, 1430.000);
    // QVector3D B(2084.930, -32.543, 1166.900);
    // QVector3D C(1933.380, 25.694, 785.350);
    // QVector3D D(2405.680, 657.809, 1103.340);
    // QVector3D D(1972.400, -1091.640, 1078.600);

    // QVector3D A(2420.060, 967.272, 578.049);
    // QVector3D B(2602.570, 652.510, 1378.670);
    // QVector3D C(2267.240, -389.142, 1564.540);
    // QVector3D D(1811.140, -1053.850, 668.848);

    // QVector3D A(1555.11, 534.88, 1890.92);
    // QVector3D B(-672.46, -600.79, 2338.14);
    // QVector3D C(-1029.48, 1011.24, 2048.81);
    // QVector3D D(-1987.66, 1156.67, 995.67);

    QVector3D A(1428.600, 1075.290, 1689.900);
    QVector3D B(1261.610, -8.888, 1967.420);
    QVector3D C(1143.090, 1136.160, 1084.010);
    QVector3D D(812.207, -596.970, 1014.130);

    // QVector3D A(886.973328, 2276.283936, 558.298828);
    // QVector3D B(1248.014038, 1278.356079, 1757.019043);
    // QVector3D C(1739.933350, 449.416870, 1746.533936);
    // // QVector3D D(2141.301025, 1046.215820, 774.156738);
    // // QVector3D D(899.932068, 754.736450, 2213.685547);
    // QVector3D D(-1123.303345, 530.642700, 2176.362549);

    // 计算外心
    QVector3D spherecenter = Point::calculateSpherecenter(A, B, C, D);
    // spherecenter = QVector3D(3186.714, -124.496, 367.3);
    // 输出外心坐标
    qDebug() << "The spherecenter of the triangle is at:" << spherecenter;
    // 输出半径
    qDebug() << "radius: " << A.distanceToPoint(spherecenter);
    qDebug() << "radius: " << B.distanceToPoint(spherecenter);
    qDebug() << "radius: " << C.distanceToPoint(spherecenter);
    qDebug() << "radius: " << D.distanceToPoint(spherecenter);
}

// void Point::test() {
//     // 定义三个点
//     // QVector3D A(1, 2, 3);
//     // QVector3D B(4, 6, 8);
//     // QVector3D C(7, 8, 9);
//     QVector3D A(1945.180, 103.261, 742.669);
//     QVector3D B(2054.380, -349.238, 1246.420);
//     QVector3D C(1812.530, -488.013, 731.441);

//     // 计算外心
//     QVector3D circumcenter = Point::calculateCircumcenter(A, B, C);

//     // 输出外心坐标
//     qDebug() << "The circumcenter of the triangle is at:" << circumcenter;

//     // 输出半径
//     qDebug() << "radius: " << A.distanceToPoint(circumcenter);
//     qDebug() << "radius: " << B.distanceToPoint(circumcenter);
//     qDebug() << "radius: " << C.distanceToPoint(circumcenter);
// }

// void Point::test() {
//     QVector3D position{4, 5, 6};
//     QVector3D rotation{-179.051, -0.956, -48.983};
//     // QVector3D moveDirection{3, 1, 2};
//     QVector3D moveDirection{-3, -1, -2};
//     float angle = 45;
//     QVector3D newRot = getNewRotation(rotation, moveDirection, angle);
//     qDebug() << newRot;
//     QVector3D offset = getTranslation(rotation, moveDirection, 50, angle);
//     qDebug() << offset;
// }

PointSet::PointSet()
    : isSafePointRecorded(false), isBeginPointRecorded(false),
      isEndPointRecorded(false), isAuxPointRecorded(false),
      isBeginOffsetPointRecorded(false), isEndOffsetPointRecorded(false) {}
