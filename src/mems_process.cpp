#include <Eigen/Core>
#include <Eigen/Dense>
#include <helper_funcs/helper_funcs.h>
#include <mems_bias/mems_bias.h>
#include <att_so3/so3_att.h>
#include <string>
#include <iostream>
#include <fstream>



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
  
  double rov_time;
  double ros_time;

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
  int seconds = 0;

  int cnt = 1;
  

  fprintf(outfile,"PARAMS,%s,%d,%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",params.last_mod.c_str(),params.hz,params.o_file.c_str(),params.i_file.c_str(),rot2rph(params.R0)(0),rot2rph(params.R0)(1),rot2rph(params.R0)(2),rot2rph(params.R_align)(0),rot2rph(params.R_align)(1),rot2rph(params.R_align)(2),params.K_acc(0,0),params.K_acc(1,1),params.K_acc(2,2),params.K_mag(0,0),params.K_mag(1,1),params.K_mag(2,2),params.K_ang_bias(0,0),params.K_ang_bias(1,1),params.K_ang_bias(2,2),params.K_acc_bias(0,0),params.K_acc_bias(1,1),params.K_acc_bias(2,2),params.K_mag_bias(0,0),params.K_mag_bias(1,1),params.K_mag_bias(2,2),params.K_g(0,0),params.K_g(1,1),params.K_g(2,2),params.K_north(0,0),params.K_north(1,1),params.K_north(2,2));

  while (std::getline(infile, line))
    {

      sscanf(line.c_str(),"%s %d/%d/%d %d:%d:%f %lf %lf %lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%f\n",msg_type,&year,&month,&day,&hour,&minute,&second,&rov_time,&ros_time,&packet.t,&packet.ang(0),&packet.ang(1),&packet.ang(2),&packet.acc(0),&packet.acc(1),&packet.acc(2),&packet.mag(0),&packet.mag(1),&packet.mag(2),&packet.fluid_pressure);
 

    if (!start)
    {

      start = true;
      time_start = packet.t;
      
    }
    float time = packet.t - time_start;



    bias.step(packet);
    att.step(bias.imu_corrected);

    Eigen::Vector3d rph_mems = rot2rph(att.R_ni*params.R_align.transpose());

    if (1)//(cnt % (params.hz/2)) == 0)
    {
      fprintf(outfile,"ATT_PRO,%d,%02d,%02d,%02d,%02d,%02f,%f,%f,%f,%f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",year,month,day,hour,minute,second,packet.t,rph_mems(0),rph_mems(1),rph_mems(2),bias.ang_bias(0),bias.ang_bias(1),bias.ang_bias(2),bias.acc_bias(0),bias.acc_bias(1),bias.acc_bias(2),bias.mag_bias(0),bias.mag_bias(1),bias.mag_bias(2),bias.acc_hat(0),bias.acc_hat(1),bias.acc_hat(2),bias.mag_hat(0),bias.mag_hat(1),bias.mag_hat(2),packet.acc(0),packet.acc(1),packet.acc(2),packet.ang(0),packet.ang(1),packet.ang(2),packet.mag(0),packet.mag(1),packet.mag(2),packet.fluid_pressure);
    }
    
    if ((((int)time) % (30) == 0)) {
      
      hours   = ((int) time)/3600;
      minutes = ((int) time - hours*3600)/60;
      int seconds_ = ((int) time - hours*3600 - minutes*60);
      if (seconds_ != seconds) {
	seconds = seconds_;
	char buffer [256];
	int n = sprintf(buffer,"%02d:%02d:%02d OF DATA PROCESSED",hours,minutes,seconds);
	std::cout<<"\r"<<buffer<<"\n";//std::flush;
      }
    }

    cnt++;

    }
  printf("\n");
  infile.close();
  fclose(outfile);

  return 0;
  
  

}
