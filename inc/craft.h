#ifndef CRAFT_H
#define CRAFT_H

#include <QString>

// 打磨方式（力矩模式、位置模式）
enum PolishMode { MomentMode, PositionMode };
// 打磨方式（圆弧、直线、区域圆弧）
enum PolishWay {
    ArcWay,
    LineWay,
    RegionArcWay_Horizontal,
    RegionArcWay_Vertical,
    RegionArcWay_Vertical_Repeat,
    RegionArcWay1,
    RegionArcWay2,
    CylinderWay_Horizontal,
    CylinderWay_Vertical,
    ZLineWay,
    SpiralLineWay
};
// 偏移方向（工具坐标系X、Y、Z方向）
enum OffsetDirection { OffsetX, OffsetY, OffsetZ };

// 工艺参数
class Craft {
  private:
    QString craftID;        // 工艺名
    PolishMode mode;        // 打磨模式
    PolishWay way;          // 打磨方式
    int teachPointReferPos; // 示教点参考位置，mm
    int cutinSpeed;         // 切入速度，mm/s
    int moveSpeed;          // 行进速度，mm/s
    int rotateSpeed;        // 转速，r/min
    int contactForce;       // 接触力，N
    int settingForce;       // 设定力，N
    int transitionTime;     // 过渡时间，ms
    int discRadius;         // 打磨片半径，mm
    int grindAngle;         // 打磨角度，°
    int offsetCount;        // 偏移次数
    int addOffsetCount;     // 新增偏移次数
    int raiseCount;         // 中途抬起次数
    int floatCount;         // 浮动次数

    friend class MainWindow;
    friend class Robot;
    friend class HansRobot;
    friend class DucoRobot;
};

#endif // CRAFT_H
