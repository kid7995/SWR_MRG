#ifndef POINT_H
#define POINT_H

#include <QVector3D>
#include <QVector>
#include <QtMath>

#include "craft.h"

// 长度单位（毫米，米）
enum LengthUnit { MM, M };
// 角度单位（角度，弧度）
enum AngleUnit { Deg, Rad };

class Point {
  public:
    Point();
    Point(float x, float y, float z, float rx, float ry, float rz);

    QVector3D calculateToolDirection(OffsetDirection direction,
                                     QVector3D rotation) const;
    Point PosRelByTool(const OffsetDirection &direction, const double &offset) const;

    void operator+=(const Point &point);

    static Point scale(const Point &pointBegin, const Point &pointEnd, float t);
    // static Point circumcenter(const Point &pointBegin, const Point &pointAux,
    //                     const Point &pointEnd);
    static QVector3D calculateCircumcenter(const QVector3D &A,
                                           const QVector3D &B,
                                           const QVector3D &C);
    static void test();

  private:
    QVector3D pos; // Position
    QVector3D rot; // Rotation

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
