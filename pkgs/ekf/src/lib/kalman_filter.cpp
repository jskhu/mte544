#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include <ekf/kalman_filter.h>

//using namespace std;

KalmanFilter::KalmanFilter():
  Q((Eigen::Matrix3d() << 0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.01).finished()),
  H(Eigen::Matrix3d::Identity()),
  state(KFStates::START)
{}

KalmanFilter::~KalmanFilter() {}

// Initialize using (non-degraded??) IPS information
void KalmanFilter::init(const Eigen::Vector3d &z, const Eigen::Matrix3d &R){
    // Initial Pose
    x = z;

    // Initial state cov
    P = R;

    state = KFStates::RUN;

}

void KalmanFilter::predict(float dt, const Eigen::Vector2d &u){
    // dt is change in time
    // u is [twist.linear.x twist.angular.z]

    //Update the state transition matrix, F
    Eigen::Matrix3d F;
    if (abs(u(1)) > 0.001) // Circular motion model
    {
        F <<  1, 0, u(0)/u(1)*(-cos(x(2))+cos(x(2)+u(1)*dt)),
              0, 1, u(0)/u(1)*(-sin(x(2))+sin(x(2)+u(1)*dt)),
              0, 0, 1;

        // Update the state
        x = x + Eigen::Vector3d(u(0)/u(1)*(-sin(x(2))+sin(x(2)+u(1)*dt)),
                                u(0)/u(1)*(cos(x(2))-cos(x(2)+u(1)*dt)),
                                u(1)*dt);
    }
    else // Linear motion model
    {
        F <<  1, 0, -u(0)*sin(x(2))*dt,
              0, 1, u(0)*cos(x(2))*dt,
              0, 0, 1;

        // Update the state
        x = x + Eigen::Vector3d(u(0)*cos(x(2))*dt,
                                u(0)*sin(x(2))*dt,
                                0);
    }
    fixYaw(x);

    // Update the state covariance
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Vector3d &z, const Eigen::Matrix3d &R){
    // z is [x y theta(yaw)]

    Eigen::Vector3d y;
    y = z - H*x;
    fixYaw(y);

    Eigen::Matrix3d Ht = H.transpose();
    Eigen::Matrix3d B = H*P*Ht + R;
    Eigen::Matrix3d K = P*Ht * B.inverse();

    // Update state
    x = x + K*y;
    fixYaw(y);
    // Update covariance matrix
    P = (Eigen::Matrix3d::Identity() - K*H) * P;
}

geometry_msgs::PoseWithCovariance KalmanFilter::createMessage(){
    geometry_msgs::Point point;
    point.x = x(0);
    point.y = x(1);

    geometry_msgs::PoseWithCovariance msg;
    msg.pose.position = point;
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(x(2));
    msg.covariance[0] = P(0,0);
    msg.covariance[1] = P(0,1);
    msg.covariance[5] = P(0,2);
    msg.covariance[6] = P(1,0);
    msg.covariance[7] = P(1,1);
    msg.covariance[11] = P(1,2);
    msg.covariance[30] = P(2,0);
    msg.covariance[31] = P(2,1);
    msg.covariance[35] = P(2,2);
}

void KalmanFilter::fixYaw(Eigen::Vector3d &vec)
{
    if (vec(2) > M_PI)
        vec(2) -= 2*M_PI;
    else if (vec(2) < -M_PI)
        vec(2) += 2*M_PI;
}
