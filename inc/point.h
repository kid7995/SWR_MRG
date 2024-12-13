#ifndef POINT_H
#define POINT_H

#include <QGenericMatrix>
#include <QVector3D>
#include <QVector>
#include <QtMath>

#include "craft.h"

// 长度单位（毫米，米）
enum LengthUnit { MM, M };
// 角度单位（角度，弧度）
enum AngleUnit { Deg, Rad };

#pragma execution_character_set("utf-8")
class Point {
  public:
    Point();
    Point(float x, float y, float z, float rx, float ry, float rz);

    QVector3D calculateToolDirection(OffsetDirection direction,
                                     QVector3D rotation) const;
    Point PosRelByTool(const OffsetDirection &direction,
                       const double &offset) const;
    QString toString() const;

    void operator+=(const Point &point);

    static Point scale(const Point &pointBegin, const Point &pointEnd, float t);
    // static Point circumcenter(const Point &pointBegin, const Point &pointAux,
    //                     const Point &pointEnd);
    // 三点求圆弧圆心
    static QVector3D calculateCircumcenter(const QVector3D &A,
                                           const QVector3D &B,
                                           const QVector3D &C);
    // 四点求球心
    static QVector3D calculateSpherecenter(const QVector3D &A,
                                           const QVector3D &B,
                                           const QVector3D &C,
                                           const QVector3D &D);
    // 欧拉角转旋转矩阵
    static QMatrix3x3 toRotationMatrix(const QVector3D &rotation);
    // 计算指定旋转轴和夹角的旋转矩阵
    static QMatrix3x3 toRotationMatrix(const QVector3D &axis, float angle);
    // 旋转矩阵转欧拉角
    static QVector3D toEulerAngles(const QMatrix3x3 &matrix);
    // 计算新的姿态
    static QVector3D getNewRotation(const QVector3D &rotation,
                                    const QVector3D &moveDirection,
                                    float angle);
    // 计算新姿态需要的偏移量
    static QVector3D getTranslation(const QVector3D &rotation,
                                    const QVector3D &moveDirection,
                                    float radius, float angle);
    // 计算法向量姿态
    static QVector3D getNormalRotation(const QVector3D &normal,
                                       const QVector3D &aux);
    static void test();

  private:
    QVector3D pos; // Position, Unit: mm
    QVector3D rot; // Rotation, Unit: degree

    friend class Robot;
    friend class HansRobot;
    friend class DucoRobot;
    friend class JakaRobot;
};

class PointSet {
  public:
    PointSet();

  private:
    Point safePoint;           // 安全点
    bool isSafePointRecorded;  // 安全点是否记录
    Point beginPoint;          // 起始点
    bool isBeginPointRecorded; // 起始点是否记录
    Point auxBeginPoint;       // 辅助起始点
    Point endPoint;            // 结束点
    bool isEndPointRecorded;   // 结束点是否记录
    Point auxEndPoint;         // 辅助结束点
    Point auxPoint;            // 辅助点（圆弧中间点、Z偏移点）
    bool isAuxPointRecorded;   // 圆弧路径中点是否记录
    QVector<Point> midPoints;  // 直线或圆弧路径中间点列表
    Point beginOffsetPoint;    // 起始偏移点
    bool isBeginOffsetPointRecorded; // 起始偏移点是否记录
    Point endOffsetPoint;            // 结束偏移点
    bool isEndOffsetPointRecorded;   // 结束偏移点是否记录

    friend class Robot;
    friend class HansRobot;
    friend class DucoRobot;
};

#endif // POINT_H
