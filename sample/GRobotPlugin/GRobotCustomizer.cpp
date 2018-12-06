/**
   @author Shin'ichiro Nakaoka
   @author Rafael Cisneros Limon
*/

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <cnoid/BodyCustomizerInterface>
#include <cnoid/EigenUtil>
#include <vector>
#include <cstring>
#include <cmath>

#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#define DLL_IMPORT __declspec(dllimport)
#else 
#define DLL_EXPORT 
#define DLL_IMPORT
#endif


using namespace std;
using namespace std::placeholders;
using namespace Eigen;
using namespace cnoid;

typedef Matrix<double, 6, 6> Matrix6;

static BodyInterface* bodyInterface = 0;
static BodyCustomizerInterface bodyCustomizerInterface;
enum ModelType {
    GR001 = 1,
    RIC30 = 2
};
enum IkType { WAIST_TO_RFOOT = 1, WAIST_TO_LFOOT, NUM_IK_TYPES };


struct JointPathValSet
{
    double* anglePtrs[6];
};

struct GRobotCustomizer
{
    BodyHandle bodyHandle;
    int modelType;
    JointPathValSet jointPathValSets[NUM_IK_TYPES];
    typedef std::function<bool(const Vector3& p, const Matrix3& R)> IkFunc;    
    IkFunc ikFuncs[NUM_IK_TYPES];

    double l0;
    double l1;
    double lo1;
    double l2;
    double lo2;
    double l3;
    double l4;
    double lo5;

    // joint angle offset of HIP_P and KNEE_P affected by lo3.
    double q2q3offset;
};


class IKSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GRobotCustomizer* robot;
    const Vector3& p;
    const Matrix3& R;
    const double sign;
    
    Vector6 q;
    Vector3 rpy;
    double cy;
    double sy;
    double cp;
    double sp;
    double cr;
    double sr;
    double slo1;

    Matrix4d Af;
    Matrix4d TB0;
    Matrix4d TB1;
    Matrix4d TB2;
    Matrix4d TB3;
    Matrix4d TB4;
    Matrix4d TB5;
    Matrix4d TB6;
    Matrix4d TBF;

    Vector6 dp;
    Vector6 dq;

    IKSolver(GRobotCustomizer* robot, const Vector3& p, const Matrix3& R, const double& sign)
        : robot(robot), p(p), R(R), sign(sign) {

    }

    bool calcEndPositionDifference() {

        // Direct Kinematics Error Calculation
        double c1 = cos(q[0]);
        double s1 = sin(q[0]);
        double c2 = cos(q[1]);
        double s2 = sin(q[1]);
        double c3 = cos(q[2]);
        double s3 = sin(q[2]);
        double c4 = cos(q[3]);
        double s4 = sin(q[3]);
        double c5 = cos(q[4]);
        double s5 = sin(q[4]);
        double c6 = cos(q[5]);
        double s6 = sin(q[5]);
    
        Matrix4d A1, A2, A3, A4, A5, A6;

        A1 <<
            c1,   0.0, -s1,  -slo1 * c1,
            s1,   0.0,  c1,  -slo1 * s1,
            0.0, -1.0,  0.0, -robot->l1,
            0.0,  0.0,  0.0,  1.0;
    
        A2 <<
            -s2,  0.0, sign * c2, -robot->lo2 * s2,
            c2,  0.0, sign * s2,  robot->lo2 * c2,
            0.0, sign, 0.0,      -robot->l2,
            0.0, 0.0, 0.0,       1.0;
    
        A3 <<
            c3, -s3,  0.0, robot->l3 * c3,
            s3,  c3,  0.0, robot->l3 * s3,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
        
        A4 <<
            c4,  s4,   0.0, robot->l4 * c4,
            s4, -c4,   0.0, robot->l4 * s4,
            0.0, 0.0, -1.0, 0.0,
            0.0, 0.0,  0.0, 1.0;
        
        A5 <<
            c5,   0.0, -sign * s5, -robot->lo5 * c5,
            s5,   0.0,  sign * c5, -robot->lo5 * s5,
            0.0, -sign,  0.0,       0.0,
            0.0,  0.0,  0.0,       1.0;
        
        A6 <<
            -s6,   0.0, -c6,  0.0,
            c6,   0.0, -s6,  0.0,
            0.0, -1.0,  0.0, 0.0,
            0.0,  0.0,  0.0, 1.0;
        
        TB1.noalias() = TB0 * A1;
        TB2.noalias() = TB1 * A2;
        TB3.noalias() = TB2 * A3;
        TB4.noalias() = TB3 * A4;
        TB5.noalias() = TB4 * A5;
        TB6.noalias() = TB5 * A6;
        TBF.noalias() = TB6 * Af; 
   
        double yaw = atan2(TBF(1,0), TBF(0,0));
        double pitch = asin(-TBF(2,0));
        double roll = atan2(TBF(2,1), TBF(2,2));
        double dYaw = rpy[2] - yaw;
        double dPitch = rpy[1] - pitch;
        double dRoll = rpy[0] - roll;
    
        Vector3 pi = TBF.block<3,1>(0,3);

        dp <<
            (p - pi),
            (-sy * dPitch + cy * cp * dRoll),
            (cy * dPitch + sy * cp * dRoll),
            (dYaw - sp * dRoll);

        double errsqr = dp.head(3).squaredNorm() + dp.tail(3).squaredNorm();
        return (errsqr < 1.0e-6 * 1.0e-6);
    }


    bool calcLegAngles(double** out_q) {

        rpy = rpyFromRot(R);

        cy = cos(rpy[2]);
        sy = sin(rpy[2]);
        cp = cos(rpy[1]);
        sp = sin(rpy[1]);
        cr = cos(rpy[0]);
        sr = sin(rpy[0]);

        Vector3 O1(0.0, sign * robot->l0, -robot->l1);
        const Vector3& F = p;
        Vector3 V1 = F - O1;
        Vector3 XF(cy * cp, sy * cp, -sp);
        Vector3 V1xXF = V1.cross(XF);
        Vector3 Z2 = -sign * V1xXF / V1xXF.norm();

        q[0] = atan2(-sign * Z2[0], sign * Z2[1]);
        q[1] = asin(-sign * Z2[2]);

        double c1 = cos(q[0]);
        double s1 = sin(q[0]);
        double c2 = cos(q[1]);
        double s2 = sin(q[1]);
        double s1s2 = s1 * s2;
        double c1s2 = c1 * s2;
        slo1 = sign * robot->lo1;

        TB2 <<
            s1s2, -sign * c1, -sign * s1 * c2,  slo1 * s1 + robot->l2 * c1 + robot->lo2 * s1s2,
            -c1s2, -sign * s1,  sign * c1 * c2,  sign * robot->l0 - slo1 * c1 + robot->l2 * s1 - robot->lo2 * c1s2,
            -c2,    0.0,      -sign * s2,      -robot->l1 - robot->lo2 * c2,
            0.0,   0.0,       0.0,            1.0;

        Vector3 V2 = (TB2.inverse() * Vector4d(F[0], F[1], F[2], 1.0)).head(3);
        double D = (V2.squaredNorm() - robot->l3 * robot->l3 - robot->l4 * robot->l4) / (2.0 * robot->l3 * robot->l4);

        if(fabs(D) > 1.0){
            return false;
        }

        q[3] = atan2(-sign * sqrt(1.0 - D * D), D);
        double c4 = cos(q[3]);
        double s4 = sin(q[3]);
        
        double beta = atan2(-V2[1], sqrt(V2[0] * V2[0] + V2[2] * V2[2]));
        double alpha = atan2(robot->l4 * s4, robot->l3 + robot->l4 * c4);
        q[2] = -(beta - alpha);
        
        q[3] = -q[3];
        
        double c3 = cos(q[2]);
        double s3 = sin(q[2]);
        double q2q3 = q[2] + q[3];
        double c34 = cos(q2q3);
        double s34 = sin(q2q3);
        
        Matrix4d T24;
        T24 <<
            c34,  s34,  0, robot->l3 * c3 + robot->l4 * c34,
            s34, -c34,  0, robot->l3 * s3 + robot->l4 * s34,
            0.0,  0.0, -1, 0.0,
            0.0,  0.0,  0, 1.0;
        
        TB4.noalias() = TB2 * T24;

        double spsr = sp * sr;
        double spcr = sp * cr;
    
        TBF <<
            cy * cp, -sy * cr + cy * spsr,  sy * sr + cy * spcr, p.x(),
            sy * cp,  cy * cr + sy * spsr, -cy * sr + sy * spcr, p.y(),
            -sp,       cp * sr,              cp * cr,             p.z(),
            0,        0,                    0,                   1.0;
        
        Matrix4d T4F;
        T4F.noalias() = TB4.inverse() * TBF;

        q[4] = atan2(-sign * T4F(0,0),  sign * T4F(1,0));
        q[5] = atan2( sign * T4F(2,2), -sign * T4F(2,1));

        // Numerical refining
        
        TB0 <<
            0.0, -1.0, 0.0, 0.0,
            1.0,  0.0, 0.0, sign * robot->l0,
            0.0,  0.0, 1.0, 0.0,
            0.0,  0.0, 0.0, 1.0;
        
        Af <<
            0.0, 1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        bool solved = false;

        int i;
        for(i=0; i < 30; ++i){

            if(calcEndPositionDifference()){
                solved = true;
                break;
            }

            // Jacobian Calculation

            Vector3 Z0 = TB0.block<3,1>(0,2);
            Vector3 Z1 = TB1.block<3,1>(0,2);
            Vector3 Z2 = TB2.block<3,1>(0,2);
            Vector3 Z3 = TB3.block<3,1>(0,2);
            Vector3 Z4 = TB4.block<3,1>(0,2);
            Vector3 Z5 = TB5.block<3,1>(0,2);
            
            Vector3 O0 = TB0.block<3,1>(0,3);
            Vector3 O1 = TB1.block<3,1>(0,3);
            Vector3 O2 = TB2.block<3,1>(0,3);
            Vector3 O3 = TB3.block<3,1>(0,3);
            Vector3 O4 = TB4.block<3,1>(0,3);
            Vector3 O5 = TB5.block<3,1>(0,3);
            Vector3 O6 = TB6.block<3,1>(0,3);
            
            Matrix6 J;
            J <<
                Z0.cross(O6 - O0), Z1.cross(O6 - O1), Z2.cross(O6 - O2), Z3.cross(O6 - O3), Z4.cross(O6 - O4), Z5.cross(O6 - O5),
                Z0,                Z1,                Z2,                Z3,                Z4,                Z5               ;
            
            // Levenberg-Marquardt Method
            
            const double lambda = 0.001;
            
            Matrix6 C;
            C.noalias() = J.transpose() * (J * J.transpose() + Matrix6::Identity() * lambda * lambda).inverse();
            
            dq.noalias() = C * dp;
            q += dq;

            if(dq.norm() <= 1.0e-5){
                break;
            }
        }

        if(!solved){
            solved = calcEndPositionDifference();
        }

        if(solved){
            *out_q[0] = q[0];
            *out_q[1] = q[1];
            *out_q[2] = q[2] + sign * robot->q2q3offset;
            *out_q[3] = q[3] - sign * robot->q2q3offset;
            *out_q[4] = q[4];
            *out_q[5] = q[5];
        }
        return solved;
    }
    
};
    
    
static const char* gr001RightLegLinks[] = {
    "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R"
};
static const char* gr001LeftLegLinks[] = {
    "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R"
};


static const char** getTargetModelNames()
{
    static const char* names[] = { 
        "GR001",
        "RIC30",
        0 };
	
    return names;
}


static int initializeIkPath(GRobotCustomizer* customizer, int ikType, const char* linkNames[])
{
    BodyHandle body = customizer->bodyHandle;

    JointPathValSet& pathValSet = customizer->jointPathValSets[ikType];

    for(int i=0; i < 6; ++i){
        int linkIndex = bodyInterface->getLinkIndexFromName(body, linkNames[i]);
        if(linkIndex < 0){
            ikType = 0;
            break;
        }
        pathValSet.anglePtrs[i] = bodyInterface->getJointValuePtr(body, linkIndex);
    }

    return ikType;
}


static bool calcLegAngles(GRobotCustomizer* robot, const Vector3& p, const Matrix3& R, double sign, double** out_q)
{
    IKSolver solver(robot, p, R, sign);
    return solver.calcLegAngles(out_q);
}


static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
    GRobotCustomizer* customizer = 0;
	
    int modelType = 0;
    string name(modelName);
	
    if(name == "GR001"){
        modelType = GR001;
    } else if(name == "RIC30"){
        modelType = RIC30;
    }

    if(modelType){
		
        customizer = new GRobotCustomizer;

        customizer->modelType = modelType;
        customizer->bodyHandle = bodyHandle;
		
        switch(modelType){
			
        case GR001:
            customizer->l0  = 0.0221;
            customizer->l1  = 0.028;
            customizer->lo1 = 0.0025;
            customizer->l2  = 0.029;
            customizer->lo2 = 0.005;
            customizer->l3  = 0.048;
            customizer->l4  = 0.059;
            customizer->lo5 = 0.0005;
            customizer->q2q3offset = 0.0;
            break;

        case RIC30:
            customizer->l0  = 0.0245;
            customizer->l1  = 0.02;
            customizer->lo1 = 0.0;
            customizer->l2  = 0.0026;
            customizer->lo2 = 0.0;

            // x-offset from HIP_P to KNEE_P.
            // l3 and q2q3offset are arranged by this value.
            const double lo3 = -0.001;
            
            const double l3  = 0.0635;
            customizer->l3 = sqrt(l3*l3 + lo3*lo3);
            customizer->q2q3offset = atan(lo3 / l3);
                
            customizer->l4  = 0.067;
            customizer->lo5 = 0.0;
            break;
        }

        if(initializeIkPath(customizer, WAIST_TO_RFOOT, gr001RightLegLinks)){
            customizer->ikFuncs[WAIST_TO_RFOOT] =
                std::bind(calcLegAngles, customizer, _1, _2, -1.0,
                     customizer->jointPathValSets[WAIST_TO_RFOOT].anglePtrs);
        }
        if(initializeIkPath(customizer, WAIST_TO_LFOOT, gr001LeftLegLinks)){
            customizer->ikFuncs[WAIST_TO_LFOOT] =
                std::bind(calcLegAngles, customizer, _1, _2, 1.0,
                     customizer->jointPathValSets[WAIST_TO_LFOOT].anglePtrs);
        }
    }

    return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
    GRobotCustomizer* customizer = static_cast<GRobotCustomizer*>(customizerHandle);
    if(customizer){
        delete customizer;
    }
}


static inline bool match(const char* s1, const char* s2)
{
    return (strcmp(s1, s2) == 0);
}


static int initializeIk(BodyCustomizerHandle customizerHandle, int baseLinkIndex, int targetLinkIndex)
{
    GRobotCustomizer* customizer = static_cast<GRobotCustomizer*>(customizerHandle);

    int ikType = 0;

    const char* baseLinkName = bodyInterface->getLinkName(customizer->bodyHandle, baseLinkIndex);
    const char* targetLinkName = bodyInterface->getLinkName(customizer->bodyHandle, targetLinkIndex);

    switch (customizer->modelType){

    case GR001:
    case RIC30:
        if(match(baseLinkName, "WAIST")){
            if(match(targetLinkName, "R_ANKLE_R")){
                ikType = WAIST_TO_RFOOT;
            }
            else if(match(targetLinkName, "L_ANKLE_R")){
                ikType = WAIST_TO_LFOOT;
            }
        } 
        break;
		
    default:
        break;
    }

    if(!customizer->ikFuncs[ikType]){
        ikType = 0;
    }
	
    return ikType;
}


static bool calcAnalyticIk(BodyCustomizerHandle customizerHandle, int ikType, const Vector3& p, const Matrix3& R)
{
    if(ikType < 0 && ikType >= NUM_IK_TYPES){
        return false;
    }

    GRobotCustomizer* customizer = static_cast<GRobotCustomizer*>(customizerHandle);
    if(customizer->ikFuncs[ikType]){
        return customizer->ikFuncs[ikType](p, R);
    }
    return false;
}


extern "C" DLL_EXPORT
BodyCustomizerInterface* getHrpBodyCustomizerInterface(BodyInterface* bodyInterface_)
{
    bodyInterface = bodyInterface_;

    bodyCustomizerInterface.version = BODY_CUSTOMIZER_INTERFACE_VERSION;
    bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
    bodyCustomizerInterface.create = create;
    bodyCustomizerInterface.destroy = destroy;
    bodyCustomizerInterface.initializeAnalyticIk = initializeIk;
    bodyCustomizerInterface.calcAnalyticIk = calcAnalyticIk;
    bodyCustomizerInterface.setVirtualJointForces = 0;

    return &bodyCustomizerInterface;
}
