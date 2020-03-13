#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include <ekf/kalman_filter.h>

//using namespace std;

KalmanFilter::KalmanFilter():
  R((Eigen::Matrix3d() << 0.1, 0, 0,
                          0, 0.1, 0,
                          0, 0, 0.1).finished()),
  H(Eigen::Matrix3d::Identity()),
  state(KFStates::START)
{}

KalmanFilter::~KalmanFilter() {}

// Initializing using first measurement and measurement covariance
void KalmanFilter::init(const Eigen::Vector3d &z, const Eigen::Matrix3d &Q){
    // Initial Pose
    x = z;

    // Initial state cov
    P = Q;

    state = KFStates::RUN;

}

void KalmanFilter::predict(float dt, const Eigen::Vector2d &u){
    // dt is change in time
    // u is [twist.linear.x twist.angular.z]

    //Update the state transition matrix, F
    Eigen::Matrix3d F;

    F <<  1, 0, -u(0)*sin(x(2))*dt,
          0, 1, u(0)*cos(x(2))*dt,
          0, 0, 1;

    // Update the state
    x = x + Eigen::Vector3d(u(0)*cos(x(2))*dt,
                            u(0)*sin(x(2))*dt,
                            u(1)*dt);

    fixYaw(x);

    // Update the state covariance
    P = F * P * F.transpose() + R;
}

void KalmanFilter::update(const Eigen::Vector3d &z, const Eigen::Matrix3d &Q){
    // z is [x y theta(yaw)]

    Eigen::Vector3d y;
    y = z - H*x;
    fixYaw(y);

    Eigen::Matrix3d Ht = H.transpose();
    Eigen::Matrix3d B = H*P*Ht + Q;
    Eigen::Matrix3d K = P*Ht * B.inverse();

    // Update state
    x = x + K*y;
    fixYaw(x);
    // Update covariance matrix
    P = (Eigen::Matrix3d::Identity() - K*H) * P;
}

void KalmanFilter::fixYaw(Eigen::Vector3d &vec)
{
    if (vec(2) > M_PI)
        vec(2) -= 2*M_PI;
    else if (vec(2) < -M_PI)
        vec(2) += 2*M_PI;
}
