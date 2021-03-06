/**
 * @file
 * @date July 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for attitude adaptive identification on SO(3). 
 */


#ifndef SO3_ATT_H
#define SO3_ATT_H

#include <Eigen/Core>
#include <helper_funcs/helper_funcs.h>




/**
 * @brief Class for attitude adaptive identificaiton on SO(3).
 */
class SO3Att
{
public:

  /**
   * @brief Constructor.
   *
   * @param Estimator parameters.
   *
   */
  SO3Att(config_params params);

  
  virtual ~SO3Att(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param IMU measurement.
   *
   */
  void step(ImuPacket measurement);
  
  Eigen::Matrix3d R_ni; /**< Estimation of NED to instrument rotation. */
 


 private:
  
  Eigen::Vector3d g_error_; /**< Local level error term. */
  Eigen::Vector3d h_error_; /**< Heading error term. */

  Eigen::Matrix3d K_g_; /**< Gravity vector estimation gain. */
  Eigen::Matrix3d K_north_; /**< North vector estimation gain. */

  Eigen::Matrix3d P_; /**< Projection matrix: \f${}^N_i\hat{R}^T(t){}^N\bar{a}_g{}^N\bar{a}^T_g{}^N_i\hat{R}(t)\f$. */
  Eigen::Vector3d w_E_n_; /**< \f${}^Nw_E\f$ vector. */
  Eigen::Vector3d a_n_; /**< \f${}^Na_g\f$ Vector. */

  double prev_t_; /**< Previous IMU measurement time. */

  int start_;

};

#endif
