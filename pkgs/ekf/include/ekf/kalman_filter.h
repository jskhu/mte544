#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Dense> // need to find how to include this

namespace KFStates
{
    enum KFState
    {
        STOP,
        START, // Waiting on IPS to initialize
        RUN    // Setup complete
    };
}
typedef KFStates::KFState KFState;

class KalmanFilter {
public:
    // State vector:
    Eigen::Vector3d x;

    // State covar matrix
    Eigen::Matrix3d P;

    // Current state
    KFState state;

    // Constructor
    KalmanFilter();

    // Destructor
    virtual ~KalmanFilter();

    void init(const Eigen::Vector3d &z, const Eigen::Matrix3d &Q);

    void predict(float dt, const Eigen::Vector2d &u); // Motion model predictions

    // Measurment (z at t+1) model predictions
    void update(const Eigen::Vector3d &z,
                const Eigen::Matrix3d &Q);

    void fixYaw(Eigen::Vector3d &vec);

private:

    // state transition matrix
    //Eigen::Matrix3d F;

    // process covariance matrix
    const Eigen::Matrix3d R;

    // measurement matrix
    const Eigen::Matrix3d H;
};
