#ifndef ROBOT_H
#define ROBOT_H
#include "Eigen/Dense"
#include "LinkM.h"
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <QObject>
#include <QFile>
#include <QFileInfo>
#include <QDebug>
#include <QtMath>
#include <QList>
#include <QDir>

using namespace std;
using namespace Eigen;

class Robot
{
    bool FileExists(QString path);
    MatrixXd _leftLegRoute;
    MatrixXd _rightLegRoute;
    QList<QByteArray> GetContentOfRobot(QString name, QByteArray content);
    MatrixXd ExtractionOfMatrix(QByteArray data);
    MatrixXd Rodrigues(MatrixXd omega, double angle);
    QList<LinkM> CreateRobotLinks(QByteArray content);

    MatrixXi _route;

    MatrixXi FindRoutfromRoot2Destination(QString Destination);
    void MoveJoints(MatrixXi route, MatrixXd deltaJointAngle);
    Vector3d Rot2omega(Matrix3d rotation);
    MatrixXd CalcTaskSpaceError(LinkM Target, QString current);
    MatrixXd CalcJacobian(MatrixXi route);
    double IKLevenbergMarquardt(QString from, LinkM Target, QString current);
public:
    void ForwardKinematic(int input);
    Robot();
    QList<LinkM> Links;
    QList<LinkM> GetLinks();
    MatrixXi Getroute();
    QHash <QString,int> MapingName2ID;
    QHash <int,QString> MapingID2Name;
    MatrixXi GetLeftLegRoute();
    void SetLeftLegRoute(MatrixXd leftLegRoute );
    MatrixXi GetRightLegRoute();
    void SetRightLegRoute(MatrixXd rightLegRoute);
    void SetJointAngle(MatrixXd JointAngle, MatrixXi Route);
    QByteArray content;
    void doIKHand(QString from, int var, QString link);
    void doIK(QString link, MatrixXd PoseLink, QString root, MatrixXd PoseRoot);
    MatrixXd IKAnalytical(LinkM Body, double D, double E, double A, double B, LinkM Foot);
    MatrixXd RPitch(double theta);
    MatrixXd RRoll(double phi);
    MatrixXd RPY2R(Matrix3d rpy);
    MatrixXi FindRoutfromBeginning2Destination(QString Beginning, QString Destination);
    void ForwardKinematicCoM(int input);
    void CalcMC(int input);
    void doIKhipRollModify(QString link, MatrixXd PoseLink, QString root, MatrixXd PoseRoot, double rollAngle);
    MatrixXi FindRoutfromDestination2Beginning(QString Beginning, QString Destination);
    //void doIKOnline(QString link, MatrixXd PoseLink, QString root, MatrixXd PoseRoot);
    void doIKOnline(QString link,MatrixXd PoseLink,MatrixXd OrientationLink,QString root,MatrixXd PoseRoot);
private:
    int Sign(double v);
    void ForwardKinematicPrimary(int input);
};

#endif // ROBOT_H
