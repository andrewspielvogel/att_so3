#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <helper_funcs/helper_funcs.h>
#include <mems_bias/mems_bias.h>
#include <att_so3/so3_att.h>
#include <string>
#include <iostream>



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
    else if((std::string(field))=="k_north")
    {
      Eigen::Vector3d k_north;
      sscanf(data,"[%lf,%lf,%lf]",&k_north(0),&k_north(1),&k_north(2));
      params.K_north = rpy2rot(k_north);
    }
    else if((std::string(field))=="k_g")
    {
      Eigen::Vector3d k_g;
      sscanf(data,"[%lf,%lf,%lf]",&k_g(0),&k_g(1),&k_g(2));
      params.K_north = rpy2rot(k_g);
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
  printf("  last_mod: %s\n",params.last_mod.c_str());
  printf("        hz: %d (s^-1)\n",params.hz);
  printf("    o_file: %s\n",params.o_file.c_str());
  printf("    i_file: %s\n",params.i_file.c_str());
  printf("        r0: [%f,%f,%f] (rpy)\n",rot2rph(params.R0)(0),rot2rph(params.R0)(1),rot2rph(params.R0)(2));
  printf("   r_align: [%f,%f,%f] (rpy)\n",rot2rph(params.R_align)(0),rot2rph(params.R_align)(1),rot2rph(params.R_align)(2));
  printf("     k_acc: [%f,%f,%f] (diag)\n",params.K_acc(0,0),params.K_acc(1,1),params.K_acc(2,2));
  printf("     k_mag: [%f,%f,%f] (diag)\n",params.K_mag(0,0),params.K_mag(1,1),params.K_mag(2,2));
  printf("k_ang_bias: [%f,%f,%f] (diag)\n",params.K_ang_bias(0,0),params.K_ang_bias(1,1),params.K_ang_bias(2,2));
  printf("k_acc_bias: [%f,%f,%f] (diag)\n",params.K_acc_bias(0,0),params.K_acc_bias(1,1),params.K_acc_bias(2,2));
  printf("k_mag_bias: [%f,%f,%f] (diag)\n",params.K_mag_bias(0,0),params.K_mag_bias(1,1),params.K_mag_bias(2,2));
  printf("       k_g: [%f,%f,%f] (diag)\n",params.K_g(0,0),params.K_g(1,1),params.K_g(2,2));
  printf("   k_north: [%f,%f,%f] (diag)\n",params.K_north(0,0),params.K_north(1,1),params.K_north(2,2));
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
  
  MEMSBias bias(params);
  SO3Att att(params);

  ImuPacket packet;
  packet.dt = 1.0/(float)params.hz;
  
  char msg_type[32];
  int year;
  int month;
  int day;
  int hour;
  int minute;
  float second;
  
  float rov_time;
  float ros_time;

  printf("***********************************\n");
  printf("    RUNNING ATTITUDE ESTIMATION\n");
  printf("***********************************\n");

  std::ifstream infile(params.i_file.c_str());
  FILE *outfile;
  outfile = fopen(params.o_file.c_str(),"w");

  std::string line;
  double time_start = 0.0;
  bool start = false;
  int hours = 0;
  int minutes = 0;

  int cnt = 1;
  

  fprintf(outfile,"PARAMS,%s,%d,%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",params.last_mod.c_str(),params.hz,params.o_file.c_str(),params.i_file.c_str(),rot2rph(params.R0)(0),rot2rph(params.R0)(1),rot2rph(params.R0)(2),rot2rph(params.R_align)(0),rot2rph(params.R_align)(1),rot2rph(params.R_align)(2),params.K_acc(0,0),params.K_acc(1,1),params.K_acc(2,2),params.K_mag(0,0),params.K_mag(1,1),params.K_mag(2,2),params.K_ang_bias(0,0),params.K_ang_bias(1,1),params.K_ang_bias(2,2),params.K_acc_bias(0,0),params.K_acc_bias(1,1),params.K_acc_bias(2,2),params.K_mag_bias(0,0),params.K_mag_bias(1,1),params.K_mag_bias(2,2),params.K_g(0,0),params.K_g(1,1),params.K_g(2,2),params.K_north(0,0),params.K_north(1,1),params.K_north(2,2));

  while (std::getline(infile, line))
    {

      sscanf(line.c_str(),"%s %d/%d/%d %d:%d:%f %f %f %lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%f\n",msg_type,&year,&month,&day,&hour,&minute,&second,&rov_time,&ros_time,&packet.t,&packet.ang(0),&packet.ang(1),&packet.ang(2),&packet.acc(0),&packet.acc(1),&packet.acc(2),&packet.mag(0),&packet.mag(1),&packet.mag(2),&packet.fluid_pressure);
 

    if (!start)
    {

      start = true;
      time_start = packet.t;
      
    }
    float time = packet.t - time_start;



    bias.step(packet);
    att.step(bias.imu_corrected);

    Eigen::Vector3d rph_mems = rot2rph(att.R_ni*params.R_align.transpose());

    if ((cnt % (params.hz/100)) == 0)
    {
      fprintf(outfile,"ATT_PRO,%d,%02d,%02d,%02d,%02d,%02f,%f,%f,%f,%f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",year,month,day,hour,minute,second,packet.t,rph_mems(0),rph_mems(1),rph_mems(2),bias.ang_bias(0),bias.ang_bias(1),bias.ang_bias(2),bias.acc_bias(0),bias.acc_bias(1),bias.acc_bias(2),bias.mag_bias(0),bias.mag_bias(1),bias.mag_bias(2),bias.acc_hat(0),bias.acc_hat(1),bias.acc_hat(2),bias.mag_hat(0),bias.mag_hat(1),bias.mag_hat(2),packet.acc(0),packet.acc(1),packet.acc(2),packet.ang(0),packet.ang(1),packet.ang(2),packet.mag(0),packet.mag(1),packet.mag(2),packet.fluid_pressure);
    }
    
    if ((((int)time) % (60) == 0) && ((int)time/60 != minutes)) {
      
      hours   = ((int) time)/3600;
      minutes = ((int) time - hours*3600)/60;
      char buffer [256];
      int n = sprintf(buffer,"%02d:%02d:00 OF DATA PROCESSED",hours,minutes);
      std::cout<<"\r"<<buffer<<std::flush;
    }

    cnt++;

    }
  printf("\n");
  infile.close();
  fclose(outfile);

  return 0;
  
  

}
