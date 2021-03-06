/**
 * @file
 * @date July 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of so3_att.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <helper_funcs/helper_funcs.h>
#include <att_so3/so3_att.h>
#include <iostream>   



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

SO3Att::SO3Att(config_params params) 
{

  // initialize usefull vectors
  Eigen::Vector3d g_e(cos(params.lat),0,sin(params.lat));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  R_ni = params.R0;

  K_g_     = params.K_g;
  K_north_ = params.K_north;
  
  w_E_n_ = get_R_en(params.lat).transpose()*w_E;
  a_n_  = get_R_en(params.lat).transpose()*a_e;

  P_ = R_ni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*R_ni;

  start_ = 0;

  prev_t_ = 0;


}

SO3Att::~SO3Att(void)
{
}

void SO3Att::step(ImuPacket measurement)
{


  if (!start_)
  {
      
    prev_t_ = measurement.t;
      
    start_ = 1;
      
  }

  
  /**************************************************************
   * Attitude Estimator
   **************************************************************/

  double dt = measurement.t - prev_t_;

  P_ = R_ni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*R_ni;

  // Define local level (g_error_) and heading (h_error_) error terms
  g_error_ = skew((measurement.acc).normalized())*R_ni.transpose()*a_n_.normalized();

  Eigen::Vector3d north_est = (Eigen::Matrix3d::Identity() - P_)*measurement.mag;

  h_error_ = P_*(skew(north_est.normalized())*R_ni.transpose().block<3,1>(0,0));
  
  R_ni     =  R_ni*((skew(K_g_*g_error_ + K_north_*h_error_ + measurement.ang - R_ni.transpose()*w_E_n_)*dt).exp());

  prev_t_ = measurement.t;
  
}
