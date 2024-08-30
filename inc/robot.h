#ifndef ROBOT_H
#define ROBOT_H

#include "AGP.h"
#include "DucoCobot.h"
#include "JAKAZuRobot.h"
#include "point.h"

class Robot {
  public:
    Robot();
    ~Robot();

    bool AGPConnect(QString agpIP);                  // 连接打磨头
    void AGPRun(const Craft &craft, bool isRotated); // 打磨头运行
    void AGPStop();                                  // 打磨头停止
    bool IsAGPEnabled();                             // 打磨头是否使能

    virtual bool RobotConnect(QString robotIP) = 0; // 连接机器人
    virtual bool GetPoint(Point &point) = 0;        // 获取点位
    virtual bool RobotTeach(int pos) = 0;           // 开始示教
    virtual bool CloseFreeDriver() = 0;             // 结束示教
    virtual bool Stop() = 0;                        // 急停
    virtual bool IsRobotElectrified() = 0;          // 是否上电
    virtual bool IsRobotEnabled() = 0;              // 是否使能
    virtual bool IsRobotMoved() = 0;                // 是否正在移动
    virtual void OpenWeb(QString ip) = 0;           // 打开网页示教器
    virtual void MoveL(const Point &point, double dVelocity, double dAcc,
                       double dRadius) = 0; // 直线运动
    virtual void MoveC(const Point &auxPoint, const Point &endPoint,
                       double dVelocity, double dAcc,
                       double dRadius) = 0; // 圆弧运动

    bool GetSafePoint(QString &strPoint);
    bool GetBeginPoint(QString &strPoint);
    bool GetEndPoint(QString &strPoint);
    bool GetAuxPoint(QString &strPoint);
    bool GetBeginOffsetPoint(QString &strPoint);
    bool GetEndOffsetPoint(QString &strPoint);
    int GetMidPoint(qint64 pressDuration, QString &strPoint);
    bool ClearPoints();
    bool ClearMidPoints();
    int DelLastMidPoint();
    bool CheckAllPoints(const PolishWay &way);

    void MoveToPoint(const QStringList &coordinates);
    void MoveBefore(const Craft &craft, bool isAGPRun);
    void MoveAfter(const Craft &craft, Point point);
    void MoveLine(const Craft &craft);
    void MoveArc(const Craft &craft);
    Point MoveRegionArc1(const Craft &craft);
    Point MoveRegionArc2(const Craft &craft);
    Point MoveRegionArcVertical(const Craft &craft);
    void MoveZLine(const Craft &craft);
    void MoveSpiralLine(const Craft &craft);
    void Run(const Craft &craft, bool isAGPRun);

  protected:
    AGP *agp;              // AGP
    PointSet pointSet;     // 点位集合
    bool isTeach;          // 自由拖拽是否启用
    QVector3D newRot;      // 倾斜指定角度后的姿态
    QVector3D translation; // 变换姿态后需要的平移量
    QVector3D newRotInv;      // 倾斜指定角度后的姿态
    QVector3D translationInv; // 变换姿态后需要的平移量
};

class HansRobot : public Robot {
  public:
    HansRobot();
    ~HansRobot();

    bool RobotConnect(QString robotIP);
    bool RobotTeach(int pos);
    bool GetPoint(Point &point);
    // void MoveBefore(const Craft &craft, bool isAGPRun);
    // void MoveAfter(const Craft &craft, Point point);
    // void MoveLine(const Craft &craft);
    // void MoveArc(const Craft &craft);
    // Point MoveRegionArc1(const Craft &craft);
    // Point MoveRegionArc2(const Craft &craft);
    // void MoveZLine(const Craft &craft);
    // void MoveSpiralLine(const Craft &craft);
    bool IsRobotElectrified();
    bool IsRobotEnabled();
    bool IsRobotMoved();
    bool CloseFreeDriver();
    // void Run(const Craft &craft, bool isAGPRun);
    bool Stop();

    void OpenWeb(QString ip); // 打开网页示教器
    void MoveL(const Point &point, double velocity, double acc,
               double radius); // 直线运动
    void MoveC(const Point &auxPoint, const Point &endPoint, double velocity,
               double acc,
               double radius); // 圆弧运动
};
/*
class DucoRobot : public Robot {
  public:
    DucoRobot();
    ~DucoRobot();

    bool RobotConnect(QString robotIP);
    bool RobotTeach(int pos);
    bool GetPoint(Point &point);
    void MoveBefore(DucoRPC::DucoCobot *robot, const Craft &craft,
                    bool isAGPRun);
    void MoveAfter(DucoRPC::DucoCobot *robot, const Craft &craft, Point point);
    void MoveLine(DucoRPC::DucoCobot *robot, const Craft &craft);
    void MoveArc(DucoRPC::DucoCobot *robot, const Craft &craft);
    Point MoveRegionArc1(const Craft &craft);
    Point MoveRegionArc2(const Craft &craft);
    void MoveZLine(const Craft &craft);
    void MoveSpiralLine(const Craft &craft);
    bool IsRobotEnabled();
    bool IsRobotMoved();
    bool CloseFreeDriver();
    void Run(const Craft &craft, bool isAGPRun);
    bool Stop();
    void OpenWeb(QString ip);

  private:
    DucoRPC::DucoCobot *ducoCobot;
};
*/
class JakaRobot : public Robot {
  public:
    JakaRobot();
    ~JakaRobot();

    bool RobotConnect(QString robotIP); // 连接机器人
    bool GetPoint(Point &point);        // 获取点位
    bool RobotTeach(int pos);           // 开始示教
    bool CloseFreeDriver();             // 结束示教
    bool Stop();                        // 急停
    bool IsRobotElectrified();          // 是否上电
    bool IsRobotEnabled();              // 是否使能
    bool IsRobotMoved();                // 是否正在移动
    void OpenWeb(QString ip);           // 打开网页示教器
    void MoveL(const Point &point, double dVelocity, double dAcc,
               double dRadius); // 直线运动
    void MoveC(const Point &auxPoint, const Point &endPoint, double dVelocity,
               double dAcc,
               double dRadius); // 圆弧运动

  private:
    JAKAZuRobot jakaRobot;
};

#endif // ROBOT_H
