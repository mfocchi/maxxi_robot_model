#ifndef MOTION_MODELS_H
#define MOTION_MODELS_H

#include <assert.h>  // debugging tool
// #include "unicycle_controller/generalPurposeEKF.h"
#include "generalPurpose.h"
#include <map>

class MotionModel;
class UnicycleModel;
class DroneModel;

typedef std::shared_ptr<MotionModel> MotionModelPtr;
typedef std::shared_ptr<UnicycleModel> UnicycleModelPtr;
typedef std::shared_ptr<DroneModel> DroneModelPtr;

enum Integration {euler, trapezoid};
// *************************************************************************************************
// CLASS: MOTION MODEL
// *************************************************************************************************

class MotionModel
{
protected:
    VectorX_t u; // control input
    data_t    dt;
    VectorX_t u_prev; // previous control input
    MatrixX_t Q; // control input covariance matrix
    int size;
    
public:
    MotionModel(){}
    MotionModel(const MatrixX_t& Q, int size, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    ~MotionModel(){}
    // define virtual void methods
    virtual void compute_fu(const VectorX_t& x, const VectorX_t& uk, VectorX_t* fu) const = 0;
    virtual void computeJacobian_Fx(const VectorX_t& x, MatrixX_t* Fx) const = 0;
    virtual void computeJacobian_Fu(const VectorX_t& x, MatrixX_t* Fu) const = 0;
    void integrate(const VectorX_t& x, int nx, VectorX_t* x_next, Integration type);
    void integrateEuler(const VectorX_t& x, int nx, VectorX_t* x_next);
    void integrateTrapezoid(const VectorX_t& x, int nx, VectorX_t* x_next); 
    //getters
    void   getControlInput(VectorX_t *u_out) const {*u_out = u;}
    void   getQ(MatrixX_t* Q_out){*Q_out = Q;}

    data_t getStepTime() const {return dt;}
    int    getSize()     const {return size;}                    
    //setters
    void setU(const VectorX_t& u){this->u = u;}
    void setU_prev(const VectorX_t& u){this->u_prev = u;}
    void setQ(const MatrixX_t& Q){this->Q = Q;}
    void setStepTime(data_t dt){this->dt = dt;}
};



class UnicycleModel : public MotionModel
{
public:
    // u=[v,w]
    UnicycleModel(const MatrixX_t& Q, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    void compute_fu(const VectorX_t& x, const VectorX_t& u, VectorX_t* fu) const override;
    
    void computeJacobian_Fx(const VectorX_t& x, MatrixX_t* Fx) const override;
    void computeJacobian_Fu(const VectorX_t& x, MatrixX_t* Fu) const override;
};

class DroneModel : public MotionModel
{
protected:
    void computeGbody(const VectorX_t& x, Vector3_t* g) const;  
    void computeSomegaQuaternion(MatrixX_t* Somega) const;
    void computeSomegaEuler(Matrix3_t* Somega) const;
public:
    // u=[ax,ay,az,omegax,omegay,omegaz]
    DroneModel(const MatrixX_t& Q, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    void compute_fu(const VectorX_t& x, const VectorX_t& u, VectorX_t* fu) const override;
    void computeJacobian_Fx(const VectorX_t& x, MatrixX_t* Fx) const override;
    void computeJacobian_Fu(const VectorX_t& x, MatrixX_t* Fu) const override;
};

class KinematicTrackedVehicleModel : public MotionModel
{
protected:
    data_t r; // [m] sproket_radius
    data_t B; // [m] distance_between_tracks
public:
    // u=[omega_left, omega_right, slip_long_left, slip_long_right]
    KinematicTrackedVehicleModel(const MatrixX_t& Q, const VectorX_t& uInit, const std::map<std::string, data_t>& params);
    void compute_fu(const VectorX_t& x, const VectorX_t& u, VectorX_t* fu) const override;
    void computeJacobian_Fx(const VectorX_t& x, MatrixX_t* Fx) const override;
    void computeJacobian_Fu(const VectorX_t& x, MatrixX_t* Fu) const override;
}

#endif