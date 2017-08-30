#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises


  */
  // measurement matrix- laser
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  // measurement matrix -radar

  Hj_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian
    coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    Eigen::VectorXd x_in = Eigen::VectorXd(4);
    Eigen::MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

    Eigen::MatrixXd F_in = MatrixXd(4, 4);
    F_in << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

    Eigen::MatrixXd Q_in = MatrixXd(4, 4);
    Q_in << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      /**
      Initialize state.
      */

      Eigen::VectorXd x_in_raw = measurement_pack.raw_measurements_;
      x_in(0) = x_in_raw(0) * cos(x_in_raw(1));
      x_in(1) = x_in_raw(0) * sin(x_in_raw(1));
      // To be checked
      x_in(2) = 0;
      x_in(3) = 0;
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      Eigen::VectorXd x_in_raw = measurement_pack.raw_measurements_;
      x_in(0)= x_in_raw (0);
      x_in(1)= x_in_raw (1);
      x_in(2)= 0;
      x_in(3)= 0;
      
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.

     */
  // compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) /
             1000000.0; // dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // 2. Set the process covariance matrix Q
  int noise_ax = 9;
  int noise_ay = 9;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0, 0,
      dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay, dt3 / 2 * noise_ax, 0,
      dt2 * noise_ax, 0, 0, dt3 / 2 * noise_ay, 0, dt2 * noise_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // measurement matrix
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = MatrixXd(3, 3);
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    // measurement matrix
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ = H_laser_;

    // measurement covariance
    ekf_.R_ = MatrixXd(2, 4);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
