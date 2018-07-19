#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <helper_funcs/helper_funcs.h>
//#include <mems_bias/mems_bias.h>
#include <att_so3/so3_att.h>
//#include <truenorth/gyro_data.h>
#include <string>
#include <iostream>

/**
 *
 * @brief Struct for storing parameters for attitude post processing.
 * 
 */
struct config_params {
  
  int hz;
  std::string o_file;
  std::string i_file;
  std::string last_mod;
  Eigen::Matrix3d K_acc;
  Eigen::Matrix3d K_mag;
  Eigen::Matrix3d K_ang_bias;
  Eigen::Matrix3d K_acc_bias;
  Eigen::Matrix3d K_mag_bias;
  Eigen::Matrix3d K_g;
  Eigen::Matrix3d K_north;
  Eigen::Matrix3d R0;
  Eigen::Matrix3d R_align;
  
};

config_params load_params(char* config_file)
{

  printf("LOADING CONFIG FILE: %s\n",config_file);
  
  config_params params;
  std::ifstream infile(config_file);
  std::string line;

  /**********************************
   *  LOAD IN LINE FROM CONFIG FILE
   **********************************/
  while (std::getline(infile, line))
  {

    char field[128] = "";
    char data[128]  = "";

    /*********************************
     *          PARSE LINE
     *********************************/
    sscanf(line.c_str(),"%s = %s",field,data);
    
    if((std::string(field))=="hz")
    {

      sscanf(data,"%d",&params.hz);

    }
    else if((std::string(field))=="o_file")
    {

      char str[128];
      sscanf(data,"%s",str);
      params.o_file = std::string(str);
      params.o_file.erase(remove(params.o_file.begin(),params.o_file.end(), '\"' ),params.o_file.end());

    }
    else if((std::string(field))=="i_file")
    {
	
      char str[128];
      sscanf(data,"%s",str);
      params.i_file = std::string(str);
      params.i_file.erase(remove(params.i_file.begin(),params.i_file.end(), '\"' ),params.i_file.end());
      
    }
    else if((std::string(field))=="k_acc")
    {

      params.K_acc = stringToDiag(data);

    }
    else if((std::string(field))=="k_mag")
    {

      params.K_mag = stringToDiag(data);

    }
    else if((std::string(field))=="k_ang_bias")
    {

      params.K_ang_bias = stringToDiag(data);

    }
    else if((std::string(field))=="k_acc_bias")
    {

      params.K_acc_bias = stringToDiag(data);

    }
    else if((std::string(field))=="k_mag_bias")
    {

      params.K_mag_bias = stringToDiag(data);

    }
    else if((std::string(field))=="rpy_align")
    {
      Eigen::Vector3d rpy_align;
      sscanf(data,"[%lf,%lf,%lf]",&rpy_align(0),&rpy_align(1),&rpy_align(2));
      params.R_align = rpy2rot(rpy_align);

    }
    else if((std::string(field))=="rpy_r0")
    {
      Eigen::Vector3d rpy_r0;
      sscanf(data,"[%lf,%lf,%lf]",&rpy_r0(0),&rpy_r0(1),&rpy_r0(2));
      params.R0 = rpy2rot(rpy_r0);
    }
    else if ((std::string(field))=="last_mod")
    {

      char str[128];
      sscanf(data,"%s",str);
      params.last_mod = std::string(str);
      params.last_mod.erase(remove(params.last_mod.begin(),params.last_mod.end(), '\"' ),params.last_mod.end());
	
    }
    
  }

  return params;

}

void print_loaded_params(config_params params)
{

  printf("***********************************\n");
  printf("           LOADED PARAMS \n");
  printf("***********************************\n");
  printf(" last_mod: %s\n",params.last_mod.c_str());
  printf("       hz: %d (s^-1)\n",params.hz);
  printf("   o_file: %s\n",params.o_file.c_str());
  printf("   i_file: %s\n",params.i_file.c_str());
  std::cout<<"R0 = \n"<< params.R0 <<"\n";
  std::cout<<"R_align = \n"<< params.R_align <<"\n";
  std::cout<<"K_acc = \n"<< params.K_acc <<"\n";
  std::cout<<"K_mag = \n"<< params.K_mag <<"\n";
  std::cout<<"K_ang_bias = \n"<< params.K_ang_bias <<"\n";
  std::cout<<"K_acc_bias = \n"<< params.K_acc_bias <<"\n";
  std::cout<<"K_mag_bias = \n"<< params.K_mag_bias <<"\n";

}


int main(int argc, char* argv[])
{

  /**************************************************
   *         CHECK FOR CORRECT ARGUMENTS
   **************************************************/
  if (argc != 2)
  {

    printf("USAGE: post_process CONFIG_FILE.m\n");
    return 1;
    
  }  
  
  /***************************************************
   *
   * LOAD CONFIG FILE
   *
   ***************************************************/

  const config_params params = load_params(argv[1]);
  print_loaded_params(params);


  /***************************************************
   *
   * INITIALIZE ESTIMATOR
   *   
   ***************************************************/
  
  // MEMSBias bias(params);
  // SO3Att att(params);
  // GyroData gyro_data(params.hz);

  // char msg_type[32];

  // printf("***********************************\n");
  // printf("    RUNNING ATTITUDE ESTIMATION\n");
  // printf("***********************************\n");

  // std::ifstream infile(params.i_file.c_str());
  // FILE *outfile;
  // outfile = fopen(params.o_file.c_str(),"w");

  // std::string line;
  // double time_start = 0.0;
  // bool start = false;
  // int hours = 0;
  // int minutes = 0;

  // int cnt = 1;
  

  // fprintf(outfile,"PARAMS,%s,%d,%s,%s,%f,%f,%f,%f,%f,%f,%f\n",params.last_mod.c_str(),params.hz,params.o_file.c_str(),params.i_file.c_str(),params.k(0),params.k(1),params.k(2),params.k(3),params.k(4),params.k(5),params.k(6));

  // while (std::getline(infile, line))
  // {

  //   sscanf(line.c_str(),"%[^,],%lf,%lf,%lf,%lf,%lf,%lf, %lf,%lf,%lf, %f, %d, %lf,%lf, %*d, %*d, %*d, %*d, %*d, %*d,%*f,%*f,%*f \n",msg_type,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp,&gyro_data.comp_timestamp);
 

  //   if (!start)
  //   {

  //     start = true;
  //     time_start = gyro_data.timestamp;
      
  //   }
  //   float time = gyro_data.timestamp - time_start;



  //   bias.step(gyro_data.ang,gyro_data.acc,gyro_data.mag,((float) 1)/(float)params.hz,gyro_data.timestamp);

  //   Eigen::Vector3d rph_mems = rot2rph(bias.Rni);

  //   if (1)//(cnt % (params.hz/100)) == 0)
  //   {
  //     fprintf(outfile,"BIAS_PRO,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gyro_data.timestamp,bias.acc_hat(0),bias.acc_hat(1),bias.acc_hat(2),bias.mag_hat(0),bias.mag_hat(1),bias.mag_hat(2),bias.a_b(0),bias.a_b(1),bias.a_b(2),bias.w_b(0),bias.w_b(1),bias.w_b(2),bias.m_b(0),bias.m_b(1),bias.m_b(2),rph_mems(0),rph_mems(1),rph_mems(2));
  //   }
    
  //   if ((((int)time) % (60) == 0) && ((int)time/60 != minutes)) {
      
  //     hours   = ((int) time)/3600;
  //     minutes = ((int) time - hours*3600)/60;
  //     char buffer [256];
  //     int n = sprintf(buffer,"%02d:%02d:00 OF DATA PROCESSED",hours,minutes);
  //     std::cout<<"\r"<<buffer<<std::flush;
  //   }

  //   cnt++;

  // }
  // printf("\n");
  // infile.close();
  // fclose(outfile);

  return 0;
  
  

}
