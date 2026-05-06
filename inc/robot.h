#ifndef ROBOT_H
#define ROBOT_H

#include "AGP.h"
#include "DucoCobot.h"
#include "JAKAZuRobot.h"
#include "HR_Pro.h"
#include "point.h"
#include "craft.h"

// 机器人品牌枚举，供 MainWindow / StatusThread 做类型分发
// 整数值对应 config.ini 中 General/RobotType 存储的数字
enum class RobotType {
    Hans = 0,  // 汉斯机器人（默认）
    Duco = 1,  // 都科机器人
    // Jaka = 2   // 节卡机器人（暂未编入工程）
};

// Vision 动作段数据结构（用于 Page6 导入 txt 动作文件）
struct VisionMotionSegment {
    QString motionType;    // "L" = 直线运动（后续可扩展 "C" 圆弧等）
    double velocity;       // 速度 mm/s
    double acceleration;   // 加速度
    double radius;         // 交融半径
    QVector<Point> points; // 该段内的目标点位列表
};

class Robot {
public:
    Robot();
    ~Robot();

    bool AGPConnect(QString agpIP);                  // 连接打磨头
    void AGPRun(const Craft &craft, bool isRotated); // 打磨头运行
    void AGPStop();                                  // 打磨头停止
    bool IsAGPEnabled();                             // 打磨头是否使能

    AGP* GetAGP() const { return agp; }  // 获取AGP指针 用于状态监控

    virtual bool RobotConnect(QString robotIP) = 0; // 连接机器人
    virtual bool GetTcpPoint(Point &point) = 0;     // 获取TCP点位
    virtual bool RobotTeach(int pos) = 0;           // 开始示教
    virtual bool CloseFreeDriver() = 0;             // 结束示教
    virtual bool Stop() = 0;                        // 急停
    virtual bool IsRobotElectrified() = 0;          // 是否上电
    virtual bool IsRobotEnabled() = 0;              // 是否使能
    virtual bool IsRobotMoved() = 0;                // 是否正在移动
    virtual void OpenWeb(QString ip) = 0;           // 打开网页示教器
    virtual void SetGlobalSpeed(int speed) {}
    virtual void MoveTcpL(const Point &point, double dVelocity, double dAcc,
                          double dRadius) = 0; // 直线运动
    virtual void MoveTcpC(const Point &auxPoint, const Point &endPoint,
                          double dVelocity, double dAcc,
                          double dRadius) = 0; // 圆弧运动

    bool GetPoint(Point &point);

    bool ReadInputPoint(const QString &filePath, QString &logInfo, QStringList &loadedPoints); //添加读取点位函数

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
    void CoverPoint(QString &strPoint);

    void MoveL(const Point &point, double dVelocity, double dAcc,
               double dRadius);
    void MoveC(const Point &auxPoint, const Point &endPoint, double dVelocity,
               double dAcc, double dRadius);
    void MoveToPoint(const QStringList &coordinates);
    void MoveBefore(const Craft &craft, bool isAGPRun);
    void MoveAfter(const Craft &craft, Point point);
    void MoveLine(const Craft &craft);
    void MoveArc(const Craft &craft);
    Point MoveRegionArc1(const Craft &craft);
    Point MoveRegionArc2(const Craft &craft);
    // Point MoveRegionArcHorizontal(const Craft &craft);
    Point MoveRegionArcHorizontal(Craft &craft);
    Point MoveRegionArcVertical(const Craft &craft);
    Point MoveRegionArcVerticalRepeat(const Craft &craft);
    Point MoveCylinderHorizontal(const Craft &craft, bool isConvex);
    Point MoveCylinderVertical(Craft &craft, bool isConvex);

    Point MoveConicalFrustum(Craft &craft); // 倒圆台侧面打磨

    void MoveZLine(const Craft &craft);
    void MoveSpiralLine(const Craft &craft);

    virtual void ToolChange(Craft &craft);// 换刀功能（virtual：允许子类覆盖以适配不同硬件IO）
    bool CanPerformToolChange(int &errorCode); // 换刀逻辑校验函数：返回 true 表示可以换刀，false 表示不能

    virtual void Run(Craft &craft, bool isAGPRun); // virtual：允许DucoRobot等子类绕过Hans专用API调用

    // Vision 动作点位执行（Page6 导入 txt 后调用）
    // isAGPRun=true 时启动 AGP（使用 craft 中的 AGP 参数），false 则不启动
    void RunVisionPoints(const QVector<VisionMotionSegment> &segments,
                         bool isAGPRun, const Craft &craft);

protected:
    AGP *agp;                 // AGP
    PointSet pointSet;        // 点位集合
    bool isTeach;             // 自由拖拽是否启用
    QVector3D newRot;         // 倾斜指定角度后的姿态
    QVector3D translation;    // 变换姿态后需要的平移量
    QVector3D newRotInv;      // 倾斜指定角度后的姿态
    QVector3D translationInv; // 变换姿态后需要的平移量
    std::atomic<bool> isStop; // 是否停止
    bool isAGPRunning;        // 是否正在转动
    bool isConnected = false; // 是否已连接机器人（RobotConnect 成功后置 true）

public:
    // 连接状态查询（StatusThread 用；非 Hans 机器人通过此方法判断是否在线）
    bool IsRobotConnected() const { return isConnected; }

public:
    ToolConfig toolConfig;
    int discThickness; // 打磨片厚度，mm
    int teachPos;      // 示教点参考位置，mm
};

class HansRobot : public Robot {
public:
    HansRobot();
    ~HansRobot();

    bool RobotConnect(QString robotIP);
    bool RobotTeach(int pos);
    bool GetTcpPoint(Point &point);
    bool IsRobotElectrified();
    bool IsRobotEnabled();
    bool IsRobotMoved();
    bool CloseFreeDriver();
    bool Stop();

    void OpenWeb(QString ip); // 打开网页示教器
    void MoveTcpL(const Point &point, double velocity, double acc,
                  double radius); // 直线运动
    void MoveTcpC(const Point &auxPoint, const Point &endPoint, double velocity,
                  double acc,
                  double radius); // 圆弧运动
};
class DucoRobot : public Robot {
public:
    DucoRobot();
    ~DucoRobot();

    bool RobotConnect(QString robotIP) override;  // 连接机器人
    bool GetTcpPoint(Point &point) override;      // 获取TCP点位（单位与Point保持一致）
    bool RobotTeach(int pos) override;            // 开始示教（拖动）
    bool CloseFreeDriver() override;              // 结束示教
    bool Stop() override;                         // 急停
    bool IsRobotElectrified() override;           // 是否上电（SR_Disable/SR_Enable均视为已上电）
    bool IsRobotEnabled() override;               // 是否使能（SR_Enable）
    bool IsRobotMoved() override;                 // 是否正在运动
    void OpenWeb(QString ip) override;            // 打开网页示教器（端口7000）
    void SetGlobalSpeed(int speed) override;      // 设置机器人全局速度

    // 底层运动接口：velocity单位mm/s（内部转换为m/s传给DucoCobot API）
    void MoveTcpL(const Point &point, double velocity, double acc,
                  double radius) override;
    void MoveTcpC(const Point &auxPoint, const Point &endPoint,
                  double velocity, double acc, double radius) override;

    // 覆盖Run以避免调用Hans专用API（HRIF_StartScript / HRIF_SetBoxDO 冷却气）
    void Run(Craft &craft, bool isAGPRun) override;

    // 覆盖ToolChange以避免调用Hans专用IO（HRIF_ReadBoxAO/SetBoxAOVal等）
    // 当前为桩实现：Duco换刀IO尚未接入，如有需要可在此扩展
    void ToolChange(Craft &craft) override;

private:
    // 运动控制专用连接（movel/movec/teach等阻塞调用，运动线程使用
    DucoRPC::DucoCobot *ducoCobot;
    // 状态查询专用连接（StatusThread 独占使用）
    DucoRPC::DucoCobot *ducoCobot_state;
};
// JakaRobot 已屏蔽（源文件未编入工程，如需启用请添加 robot_jaka.cpp）
// class JakaRobot : public Robot {
//   public:
//     JakaRobot();
//     ~JakaRobot();
//
//     bool RobotConnect(QString robotIP); // 连接机器人
//     bool GetTcpPoint(Point &point);     // 获取点位
//     bool RobotTeach(int pos);           // 开始示教
//     bool CloseFreeDriver();             // 结束示教
//     bool Stop();                        // 急停
//     bool IsRobotElectrified();          // 是否上电
//     bool IsRobotEnabled();              // 是否使能
//     bool IsRobotMoved();                // 是否正在移动
//     void OpenWeb(QString ip);           // 打开网页示教器
//     void MoveTcpL(const Point &point, double dVelocity, double dAcc,
//                   double dRadius); // 直线运动
//     void MoveTcpC(const Point &auxPoint, const Point &endPoint,
//                   double dVelocity, double dAcc,
//                   double dRadius); // 圆弧运动
//
//   private:
//     JAKAZuRobot jakaRobot;
// };

#endif // ROBOT_H
