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
    CylinderWay_Horizontal_Convex,
    CylinderWay_Vertical_Convex,
    CylinderWay_Horizontal_Concave,
    CylinderWay_Vertical_Concave,
    ConicalFrustum_Concave,// 倒圆台打磨
    ZLineWay,
    SpiralLineWay
};
// 偏移方向（工具坐标系X、Y、Z方向）
enum OffsetDirection { OffsetX, OffsetY, OffsetZ };

// 工艺参数
// 全局换刀参数结构体
struct ToolConfig {
    int totalPolishCount = 0;          // 累计打磨次数
    double totalPolishLength = 0.0;    // 累计打磨长度
    double toolChangeThreshold = 30000.0; // 换刀阈值长度
    bool startToolChange = false;      // 启动换刀
    bool toolChangeDone = false;       // 换刀完毕
    int toolMagStatus = 0;             // 刀库状态 (0-正常, 1-空, 2-故障)
    int targetToolPos = 0;             // 换刀刀位指定 (当前正在使用的刀具位置或即将换的)
};

// 工艺参数
class Craft {
private:
    QString craftID;        // 工艺名
    PolishMode mode;        // 打磨模式
    PolishWay way;          // 打磨方式
    int teachPointReferPos; // 示教点参考位置
    int cutinSpeed;         // 切入速度
    int moveSpeed;          // 行进速度
    int rotateSpeed;        // 转速
    int contactForce;       // 接触力
    int settingForce;       // 设定力
    int transitionTime;     // 过渡时间
    int discRadius;         // 打磨片半径
    int discThickness;      // 打磨片厚度
    int grindAngle;         // 打磨角度
    int offsetCount;        // 偏移次数
    int addOffsetCount;     // 新增偏移次数
    int raiseCount;         // 中途抬起次数
    int floatCount;         // 浮动次数
    int transitionRadius;   // 过渡半径
    bool isMirror;          // 圆柱打磨是否采用镜像

    friend class MainWindow;
    friend class Robot;
    friend class HansRobot;
    friend class DucoRobot;
};

#endif // CRAFT_H
