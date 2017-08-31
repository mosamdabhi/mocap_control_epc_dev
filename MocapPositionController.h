#ifndef MOCAP_POSITION_CONTROLLER_H
#define MOCAP_POSITION_CONTROLLER_H

#include <cstdio>
#include <cstdint>
#include <uORB/uORB.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/mocap_position_command.h>
#include <uORB/topics/mocap_position_command_gains.h>
#include <time.h>
//#include "math3d.h"
#include "c_matrixmath.h"


/*
#include <px4_eigen.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <vector> 
#include <stdio.h>
#include <stdint.h>
*/

#include "ParameterUtils.h"
#include "L1PositionObserver.h"

//#define MILLION  1000000L;
/*
static uint16_t curr_id;
  //static uint32_t ctr = 0;
static bool init_epc = false;

static float ctrl_use[6];  
  struct Element
  {
    uint16_t id;
    uint16_t active_set_size;
    uint16_t* active_set;
  };
#include "c_epc_autogen.h"

*/
class MocapPositionController
{
public:
  MocapPositionController() :
    ctrl_state_set(false),
    local_pos_set(false),
    cmd_time(0)
  {
    e3.zero();
    e3(2) = 1.0f;
    gravity.zero();
    current_rpm_cmd.zero();
  }
  ~MocapPositionController() { }

  bool initialize()
  {
    if (!loadParameters())
    {
      puts("[MPC] failed to load parameters");
      return false;
    }

    if (!registerCallbacks())
    {
      puts("[MPC] failed to register callbacks");
      return false;
    }

    l1_pos_observer.loadParameters();

    return true;
  }

  bool update()
  {
    bool cmd_updated = false;
    bool updated;

    orb_check(local_position_sub, &updated);
    if (updated)
    {
      struct vehicle_local_position_s local_position;
      orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
      localPositionMessageCallback(local_position);
    }

    orb_check(mocap_position_cmd_sub, &updated);
    if (updated)
    {
      struct mocap_position_command_s cmd_in;
      orb_copy(ORB_ID(mocap_position_command), mocap_position_cmd_sub, &cmd_in);
      cmd_updated = mocapPositionCommandMessageCallback(cmd_in);
    }

    orb_check(mocap_position_cmd_gains_sub, &updated);
    if (updated)
    {
      struct mocap_position_command_gains_s cmd_in;
      orb_copy(ORB_ID(mocap_position_command_gains), mocap_position_cmd_gains_sub, &cmd_in);
      mocapPositionCommandGainsMessageCallback(cmd_in);
    }

    hrt_abstime tnow = hrt_absolute_time();

    if (cmd_updated)
      cmd_time = tnow;

    if (tnow - cmd_time > cmd_ttl_us)
      return false;

    return updatePositionController();
  }

  void finalize()
  {
    closeSubscriptions();
  }

  void setControlState(const control_state_s& ctrl_state_)
  {
    memcpy(&ctrl_state, &ctrl_state_, sizeof(ctrl_state_));
    ctrl_state_set = true;
  }

  void setCurrentRPMCommand(const MotorManager::rpm_cmd_t& in)
  {
    for (unsigned int i = 0; i < 4; i++)
      current_rpm_cmd(i) = in.motor[i];
  }

  void getCascadedAttitudeCommand(cascaded_attitude_command_t& out)
  {
    out = att_cmd;
  }

private:
  void closeSubscriptions()
  {
    close(mocap_position_cmd_sub);
    close(mocap_position_cmd_gains_sub);
    close(local_position_sub);
  }

  bool loadParameters()
  {
    cmd_ttl_us =
      static_cast<hrt_abstime>(parameter_utils::getFloatParam("MCC_CMD_TTL")*1.0e6f);
    mass = parameter_utils::getFloatParam("MCC_TOTAL_MASS");
    gravity(2) = parameter_utils::getFloatParam("MCC_GRAVITY");

  /*  dt_ = parameter_utils::getFloatParam("DT_");  // PARAMETER OF DT_
    // int values
    HORIZON_LENGTH = parameter_utils::getIntParam("HORIZON_LENGTH");
    NUM_STATES = parameter_utils::getIntParam("NUM_STATES");
    NUM_INPUTS = parameter_utils::getIntParam("NUM_INPUTS");
  */

    return true;
  }

  bool registerCallbacks()
  {
    mocap_position_cmd_sub = orb_subscribe(ORB_ID(mocap_position_command));
    if (mocap_position_cmd_sub < 0)
    {
      puts("[MPC] mocap_position_cmd_sub failed");
      return false;
    }

    mocap_position_cmd_gains_sub = orb_subscribe(ORB_ID(mocap_position_command_gains));
    if (mocap_position_cmd_gains_sub < 0)
    {
      puts("[MPC] mocap_position_cmd_gains_sub failed");
      return false;
    }

    local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    if (local_position_sub < 0)
    {
      puts("[MPC] local_position_sub failed");
      return false;
    }

    return true;
  }
/*
void initEPCMats()
  {
    initDatabase();

    curr_id = 0;

    for (uint16_t i=0; i<6; i++)
      ctrl_use[i] = 0;

    init_epc = true;
  }  
  Eigen::MatrixXf blkdiag(const Eigen::MatrixXf& a, int count)
  {
      Eigen::MatrixXf bdm = Eigen::MatrixXf::Zero(a.rows() * count, a.cols() * count);
      for (int i = 0; i < count; ++i)
      {
          bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;
      }

      return bdm;
  }

  bool isGreaterThanZero(Eigen::MatrixXf& Z)
  {
    bool check = true;
    for (int lam_var=0; lam_var < Z.size(); lam_var++)
    {
      if (Z(lam_var) <= 0)
      {
        check = false;
        break;
      }                 
    }
    return check; 
  }


  bool isEmpty(Eigen::MatrixXf& Z)
  {
     bool check = true;
     for (int row(0); row < Z.rows(); ++row)
      for (int col(0); col < Z.cols(); ++col){
          if ( abs(Z(row,col)) > 1e-8){
              check = false;
              break;
          }
       }
     return check;
  }  

*/
/*
  Eigen::Vector3f cov(float dt_for_func, float mass_for_func, const int HORIZON_LENGTH_for_func, const int NUM_STATES_for_func, const int NUM_INPUTS_for_func, const Eigen::Vector3f& e_pos, const Eigen::Vector3f& e_vel)
  { 


    struct timespec start, stop;
    double accum;

    if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      perror( "clock gettime" );
      exit( EXIT_FAILURE );
    }

   
    printf("Entering EPC logic\n"); 


    // Keeeping testing debug values of e_pos and e_vel as of now
    //Vector3f e_pos_here = Vector3f::Zero(3);
    //e_pos_here << 0.9f,0.0f,0.0f;
    
    //Vector3f e_vel_here = Vector3f::Zero(3);  

    // Desired state values. // Would be obtained in PositionController.h // Taking random values initially
    Eigen::VectorXf cal_r_temp(e_pos.rows() + e_vel.rows()); // Concatenating vertically
    cal_r_temp << e_pos,e_vel;
    Eigen::VectorXf cal_r = cal_r_temp.replicate(HORIZON_LENGTH_for_func, 1);

    Eigen::MatrixXf c_temp(6,1), cal_c;
    c_temp << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;   // Desired Pos_x | Desired Pos_y | Desired Pos_z | Desired Vel_x | Desired Vel_y | Desired Vel_z 
    cal_c = c_temp.replicate(HORIZON_LENGTH_for_func, 1);     

    // Creating cal_Q
    // Taking the Q values from Vishnu's script matching the PD Gains for danaus quads for N=2
    Eigen::VectorXf Q_tmp(NUM_STATES_for_func);
    Q_tmp << 75.340570f, 75.340570f, 74.654191f, 0.281723f, 0.281723f, 74.496852f; // This is for N=2
    //Q_tmp <<  44.911248, 44.911248, 78.319160, 0.758221, 0.758221, 1.625917; // for N=10
    Eigen::MatrixXf Q1 = Q_tmp.asDiagonal();
    Eigen::MatrixXf cal_Q = blkdiag(Q1,HORIZON_LENGTH_for_func);    

    
    // Creating cal_R
    // Taking the R values from Vishnu's script matching the PD Gains for danaus quads for N=2
    Eigen::Vector3f R_tmp(0.001306f, 0.001306f, 0.233827f);  // FOR N=2
    //Vector3f R_tmp(0.017063, 0.017063, 0.023259); // FOR N=10
    Eigen::Matrix3f R1 = R_tmp.asDiagonal();
    Eigen::MatrixXf cal_R = blkdiag(R1,HORIZON_LENGTH);     

    Eigen::MatrixXf A_discrete(6,6);
      A_discrete << 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f,
                    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::MatrixXf B_discrete(6,3);
      B_discrete << 6.30517e-05f, 0.0f, 0.0f,
                    0.0f, 6.30517e-05f, 0.0f,
                    0.0f, 0.0f, 6.30517e-05f,
                    0.0126103f, 0.0f, 0.0f,
                    0.0f, 0.0126103f, 0.0f,
                    0.0f, 0.0f, 0.0126103f;
    // Creating cal_B
    Eigen::MatrixXf Z_for_cal_B = Eigen::MatrixXf::Zero(6,3);


    Eigen::MatrixXf cal_B = Eigen::MatrixXf::Zero(HORIZON_LENGTH_for_func*6,(6*HORIZON_LENGTH_for_func)/2);
    cal_B.block<6,3>(0,0) << B_discrete;
    for (int j=1;j<HORIZON_LENGTH_for_func;j++)
    {
      cal_B.block<6,3>(0,3*j) << Z_for_cal_B;
    }    
    for (int j=6;j<HORIZON_LENGTH_for_func*6;j+=6)
    {
      for (int k=0;k<HORIZON_LENGTH_for_func*3;k+=3)
      {
          if(k==0)
          {
            cal_B.block<6,3>(j,k) << A_discrete*cal_B.block<6,3>(j-6,k);
          }
          else
          {
            cal_B.block<6,3>(j,k) << cal_B.block<6,3>(j-6,k-3);
          }
      }
    }    

    //printf("HORIZON_LENGTH is : %d\n",HORIZON_LENGTH_for_func);
    //printf("NUM_STATES is : %d\n",NUM_STATES_for_func);
    //printf("NUM_INPUTS is : %d\n",NUM_INPUTS_for_func);

    // Calculating cal_H
    Eigen::MatrixXf cal_H = ((cal_B.transpose()*cal_Q)*cal_B) + cal_R;
    Eigen::MatrixXf inverse_cal_H = cal_H.inverse();

    
    // Calculating cal_h
    Eigen::MatrixXf cal_h = ((cal_B.transpose()*cal_Q))*(cal_c - cal_r); 
    //cout << "Diff between actual and this: " << endl << cal_h_dummy - cal_h << endl;


    // Now adding rest of the parts for other database
    // Creating constriant matrices || Single matrices and not block diagonal matrices
    Matrix3f Gtmp1 = Matrix3f::Zero();
    Matrix3f Gtmp2 = Matrix3f::Identity();
    MatrixXf C(Gtmp1.rows(), Gtmp1.cols()+Gtmp2.cols()); //Concatenating Matrices horizontally
    C << Gtmp1, Gtmp2;     

    // Creating G_x
    MatrixXf G_x(C.rows() + C.rows(), C.cols()); // Concatenating vertically
    G_x << C,-1*C; // Constrainsts on velocity upper and lower bounds || L.H.S.
    
    // Creating G_u
    MatrixXf G_u(Gtmp2.rows() + Gtmp2.rows(), Gtmp2.cols());
    G_u << Gtmp2, -1*Gtmp2; // Constrainst on forces upper and lower bounds || R.H.S.

    
    /////////// Removing them as of now and using alternate cal_g_x and cal_g_u for small_gamma_a 
    /////////// Write the logic and confirm once with Vishnu or Alex
    
    
    // Creating g_x and cal_g_x
    MatrixXf g_x(NUM_STATES_for_func,1), cal_g_x;
    g_x << 2.6000f, 2.6000f, 3.5000f, 2.6000f, 2.6000f, 3.5000f; //Constrainsts on velocity upper and lower bounds || R.H.S.
    cal_g_x = g_x.replicate(HORIZON_LENGTH_for_func, 1);
    
    
    // Creating g_u
    MatrixXf g_u(NUM_STATES_for_func,1), cal_g_u;
    g_u << 2.9000f, 2.9000f, 2.9000f, 2.9000f, 2.9000f, 2.9000f; //Constrainsts on forces upper and lower bounds || R.H.S.
    cal_g_u = g_u.replicate(HORIZON_LENGTH_for_func, 1);

      
      //////////// Alternate cal_g_x and cal_g_u
    MatrixXf alternate_g_x(NUM_STATES_for_func,1), alternate_cal_g_x;
    alternate_g_x << 2.6000f, 2.6000f, 3.5000f, -2.6000f, -2.6000f, -3.5000f; //Constrainsts on velocity upper and lower bounds || R.H.S.
    alternate_cal_g_x = alternate_g_x.replicate(HORIZON_LENGTH_for_func, 1);

    // Creating g_u
    MatrixXf alternate_g_u(NUM_STATES_for_func,1), alternate_cal_g_u;
    alternate_g_u << 2.9000f, 2.9000f, 2.9000f, -2.9000f, -2.9000f, -2.9000f; //Constrainsts on forces upper and lower bounds || R.H.S.
    alternate_cal_g_u = alternate_g_u.replicate(HORIZON_LENGTH_for_func, 1);     

    // Creating cal_. values of all these constraints
    MatrixXf cal_G_x = blkdiag(G_x,HORIZON_LENGTH_for_func);
    MatrixXf cal_G_u = blkdiag(G_u,HORIZON_LENGTH_for_func);    

    // Calculating capital_gamma
    MatrixXf capital_gamma(cal_G_x.rows() + cal_G_u.rows(), cal_G_u.cols()); // Concatenating vertically
    capital_gamma << cal_G_x*cal_B,cal_G_u; 


    // Calculating small_gamma
    MatrixXf small_gamma(cal_g_x.rows() + cal_g_u.rows(), cal_g_u.cols()); // Concatenating vertically
    small_gamma << cal_g_x,cal_g_u; 

    // Calculating alternate_small_gamma
    MatrixXf alternate_small_gamma(alternate_cal_g_x.rows() + alternate_cal_g_u.rows(), alternate_cal_g_u.cols()); // Concatenating vertically
    alternate_small_gamma << alternate_cal_g_x,alternate_cal_g_u;

    VectorXf u_semi_final = VectorXf::Zero(6);

    if (!init_epc)
    {
       initEPCMats(); 
    }

    //uint8_t ctrl_db_itr = 0;

    for (int var_dtbs=0; var_dtbs < DB_SIZE; ++var_dtbs)
    {
      if(!ctrl_db[var_dtbs].active_set_size == 0)
      {
        MatrixXf P(1,ctrl_db[var_dtbs].active_set_size);
        for (int w=0; w<ctrl_db[var_dtbs].active_set_size; w++)
        {
            P.col(w) << ctrl_db[var_dtbs].active_set[w];    
        }   
        // Generating capital_gamma_a
        MatrixXf capital_gamma_a = MatrixXf::Zero(P.size(),capital_gamma.cols());    
        for (int q=0;q<P.size();q++)
        {
            capital_gamma_a.row(q) << capital_gamma.row(P(q));
        }
        //cout << var_dtbs << endl;

        // Generating alternate_small_gamma_a
        MatrixXf alternate_small_gamma_a = MatrixXf::Zero(P.size(),1);
        for (int q=0;q<P.size();q++)
        {
            alternate_small_gamma_a.row(q) << alternate_small_gamma.row(P(q));
        }

          // Creating E_1 to E_6 matrices
          // One another idea for optimization. Pre calculate all transpose values and assign 
          // different variables to them so that you don't have to recalculate them all over again

          MatrixXf E_1(capital_gamma_a.rows(),inverse_cal_H.cols());
          MatrixXf E_2(E_1.rows(),capital_gamma_a.transpose().cols());
          MatrixXf E_3(E_1.transpose().rows(),E_2.cols());
          MatrixXf E_4(E_3.rows(),E_1.cols());
          MatrixXf E_5(E_4.rows(),cal_Q.cols());
          MatrixXf E_6(E_3.transpose().rows(),cal_Q.cols());
          MatrixXf lambda_a = MatrixXf::Zero(E_6.rows(),alternate_small_gamma.cols());
          MatrixXf final_kkt_cond = MatrixXf::Zero(capital_gamma.rows(),u_semi_final.cols());

          E_1 << capital_gamma_a*inverse_cal_H;
          E_2 << -1*((E_1*capital_gamma_a.transpose()).inverse());
          E_3 << (E_1.transpose()*E_2);
          E_4 << inverse_cal_H + (E_3*E_1);
          E_5 << (E_4*cal_B.transpose())*cal_Q;
          E_6 << (E_3.transpose()*cal_B.transpose())*cal_Q;
          lambda_a << (-E_6*cal_r) + (E_6*cal_c + E_2*alternate_small_gamma_a);

          //const char* jambo_flag_var;
          if (isGreaterThanZero(lambda_a))
          {
            u_semi_final = ((E_5*cal_r) - (E_5*cal_c) - (E_3*alternate_small_gamma_a));
            final_kkt_cond = (small_gamma - (capital_gamma*u_semi_final));  
            if (isGreaterThanZero(final_kkt_cond))
            {
              epcSolutionFound = true;
              break;              
            }
            else
            {
              //jambo_flag_var = "Reached last condition but didn't fulfilled";
              //cout << jambo_flag_var << endl;
              continue;                                   
            }
          }
          else
          //jambo_flag_var = "First condition of lambda_a>=0 is not fulfilled so exiting";
          //cout << lambda_a << endl;
          //cout << jambo_flag_var << endl;
          continue;            
      }
      else
      {
        u_semi_final = inverse_cal_H * (-1*cal_h);
        epcSolutionFound = true;
        break;        
      }
    }

  Eigen::Vector3f fd_w = Eigen::Vector3f::Zero();
  
  if (epcSolutionFound == true)
  {
    printf("EPC Solution found here\n");
    fd_w << u_semi_final.block<3,1>(0,0);   
  }
  else
  {
    printf("No EPC Solution found\n");
  }
  
  //cout << endl << fd_w << endl;
  //cout << solutionFound << endl;
  //printf("EPC SOLUTION VALUES ARE: %.2f\n",double(u_semi_final(0))); 
   if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) {
      perror( "clock gettime" );
      exit( EXIT_FAILURE );
    }

    accum = ( stop.tv_sec - start.tv_sec )
          + ( stop.tv_nsec - start.tv_nsec )/MILLION;

    printf( "Time elapsed EPC: %3.4f\n", double(accum)); 

  
  return fd_w;  // Change it to fd_w soon after debug until here gets complted     

  }

  timespec diff(timespec start, timespec end)
  {
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
      temp.tv_sec = end.tv_sec-start.tv_sec-1;
      temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
      temp.tv_sec = end.tv_sec-start.tv_sec;
      temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }

    return temp;
  }  
*/
  bool updatePositionController()
  {
    //tic();
  
/*  timespec time1, time2;
  //int temp;
  clock_gettime(CLOCK_REALTIME, &time1);
*/

    if (!ctrl_state_set || !local_pos_set)
      return false;

    if (!cmd.cmd_set || !cmd_gains.gains_set)
    {
#if 0
      static unsigned int counter = 0;
      if (counter++ > 100)
      {
        cmd.print();
        cmd_gains.print();
        counter = 0;
      }
#endif
      return false;
    }

    math::Vector<3> pos(local_pos.x, local_pos.y, local_pos.z);
    math::Vector<3> vel(local_pos.vx, local_pos.vy, local_pos.vz);
    math::Quaternion q(ctrl_state.q);
    math::Matrix<3, 3> R = q.to_dcm();
    math::Vector<3> Rde3 = R*e3;


  printf("got local position x: %3.4f y: %3.4f z: %3.4f\n",
          (double) pos(0),
          (double) pos(1),
          (double) pos(2)); 
  

    math::Vector<3> e_pos = pos - cmd.pos;
    math::Vector<3> e_vel = vel - cmd.vel;

/*    Eigen::Vector3f e_pos_in_eigen = Eigen::Vector3f::Zero();
    e_pos_in_eigen << e_pos(0),e_pos(1),e_pos(2),e_pos(3);

    Eigen::Vector3f e_vel_in_eigen = Eigen::Vector3f::Zero();
    e_vel_in_eigen << e_vel(0),e_vel(1),e_vel(2);
*/  
    //Eigen::Vector3f fd_w_in_EigenForm = Eigen::Vector3f::Zero();
    //fd_w_in_EigenForm << MocapPositionController::cov(dt_, mass, HORIZON_LENGTH, NUM_STATES, NUM_INPUTS, e_pos_in_eigen, e_vel_in_eigen);



    // World force in NED frame
    math::Vector<3> fd_w =
      (-cmd_gains.kp.emult(e_pos) - cmd_gains.kd.emult(e_vel) + cmd.acc - gravity)*mass;

    // L1 Position Observe (provides world force error in NED frame)
    math::Vector<3> l1_fw = l1_pos_observer.update(R, vel, current_rpm_cmd);

    // Total force
    fd_w -= l1_fw;

    // thrust magnitude (in NED)
    att_cmd.thrust = fd_w(0)*Rde3(0) + fd_w(1)*Rde3(1) + fd_w(2)*Rde3(2);

#if 0
    static unsigned int debug_counter1 = 0;
    if (debug_counter1++ > 100)
    {
      puts("e_pos");
      e_pos.print();
      puts("e_vel");
      e_vel.print();
      puts("fd_w");
      fd_w.print();
      puts("l1_fw");
      l1_fw.print();
      debug_counter1 = 0;
    }
#endif

    // Attitude calculation requires the negative
    math::Vector<3> fd_w_att = -fd_w;

    math::Vector<3> b3;
    if (fd_w_att.length_squared() < 1e-5f)
      b3 = e3;
    else
      b3 = fd_w_att.normalized();

    math::Vector<3> c1(cosf(cmd.heading(0)), sinf(cmd.heading(0)), 0.0f);
    // b3 x c1
    math::Vector<3> b2 = b3 % c1;
    b2.normalize();
    // b2 x b3
    math::Vector<3> b1 = b2 % b3;

    math::Matrix<3, 3> desired_rotation;
    desired_rotation.set_col(0, b1);
    desired_rotation.set_col(1, b2);
    desired_rotation.set_col(2, b3);

    att_cmd.q.from_dcm(desired_rotation);
    computeAngularReference(desired_rotation, att_cmd.thrust, e_vel,
                            att_cmd.ang_vel, att_cmd.ang_acc);
    math::Vector<3> desired_euler_zyx = desired_rotation.to_euler();
    computeHeadingReference(desired_euler_zyx, att_cmd.ang_vel, att_cmd.ang_acc);
    att_cmd.cmd_set = true;

#if 0
    static unsigned int debug_counter = 0;
    if (debug_counter++ > 100)
    {
      att_cmd.print();
      puts("desired rotation");
      desired_rotation.print();
      debug_counter = 0;
    }
#endif

  /*
  clock_gettime(CLOCK_REALTIME, &time2);
  //printf("In seconds: %1.0f\n", double(diff(time1,time2).tv_sec));
  printf("In milliseconds: %9.9f\n", double(diff(time1,time2).tv_nsec));
  */
    return true;
  }



  void computeHeadingReference(const math::Vector<3>& desired_euler_zyx,
                               math::Vector<3>& omg_des,
                               math::Vector<3>& omg_ddes)
  {
    float phi = desired_euler_zyx(0);
    float theta = desired_euler_zyx(1);

    float cph = cosf(phi);
    float sph = sinf(phi);

    float cth = cosf(theta);
    float sth = sinf(theta);

    float psi_d = cmd.heading(1);
    float psi_dd = cmd.heading(2);

    float p = omg_des(0);
    float q = omg_des(1);
    float q_d = omg_ddes(1);

    if (fabsf(cph) < 1.0e-5f)
    {
      omg_des(2) = 0.0f;
      omg_ddes(2) = 0.0f;
    }
    else
    {
      omg_des(2) = (psi_d*powf(cph,2.0f)*cth - q*sph + psi_d*cth*powf(sph,2.0f))/cph;
      omg_ddes(2) = -(p*q + 2.0f*psi_d*q*sth - psi_dd*cph*cth + q_d*cph*sph - 2.0f*powf(psi_d,2.0f)*cth*sph*sth - p*psi_d*cth*sph)/powf(cph,2.0f);
    }
  }

  void computeAngularReference(const math::Matrix<3,3>& R,
                               const float& thrust_des,
                               const math::Vector<3>& vel_err,
                               math::Vector<3>& omg_des,
                               math::Vector<3>& omg_ddes)
  {
    float R11 = R(0,0);
    float R12 = R(0,1);
    float R13 = R(0,2);
    float R21 = R(1,0);
    float R22 = R(1,1);
    float R23 = R(1,2);
    float R31 = R(2,0);
    float R32 = R(2,1);
    float R33 = R(2,2);
    float Td = fabsf(thrust_des);

    math::Vector<3> acc_err; acc_err.zero();
    math::Vector<3> K =
      (-cmd_gains.kp.emult(vel_err) - cmd_gains.kd.emult(acc_err) + cmd.jerk)*mass;

    float d =
      R11*R22*R33 - R11*R23*R32 - R12*R21*R33 + R12*R23*R31 + R13*R21*R32 - R13*R22*R31;
    float w1 =
      -(K(0)*(R21*R33 - R23*R31) + K(1)*(R13*R31 - R11*R33) + K(2)*(R11*R23 - R13*R21))/(Td*d);
    float w2 =
      -(K(0)*(R22*R33 - R23*R32) + K(1)*(R13*R32 - R12*R33) + K(2)*(R12*R23 - R13*R22))/(Td*d);
    float Tdd =
      -(K(0)*(R21*R32 - R22*R31) + K(1)*(R12*R31 - R11*R32) + K(2)*(R11*R22 - R12*R21))/d;

    omg_des(0) = w1;
    omg_des(1) = w2;
    omg_des(2) = 0.0f;

    omg_ddes(0) = -2.0f*w1*Tdd/Td;
    omg_ddes(1) = -2.0f*w2*Tdd/Td;
    omg_ddes(2) = 0.0f;
  }

  typedef struct CascadedPositionCommand
  {
    math::Vector<3> pos;
    math::Vector<3> vel;
    math::Vector<3> acc;
    math::Vector<3> jerk;
    math::Vector<3> heading;
    bool cmd_set;

    CascadedPositionCommand() :
      cmd_set(false)
    {
      pos.zero();
      vel.zero();
      acc.zero();
      jerk.zero();
      heading.zero();
    }

    void print()
    {
      // Hack to work around NuttX printf float bug
      puts("pos cmd:");
      char buf[128];
      sprintf(buf, "pos = [%0.2f, %0.2f, %0.2f]",
              double(pos(0)), double(pos(1)), double(pos(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "vel = [%0.2f, %0.2f, %0.2f]",
              double(vel(0)), double(vel(1)), double(vel(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "acc = [%0.2f, %0.2f, %0.2f]",
              double(acc(0)), double(acc(1)), double(acc(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "jerk = [%0.2f, %0.2f, %0.2f]",
              double(jerk(0)), double(jerk(1)), double(jerk(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "heading = [%0.2f, %0.2f, %0.2f]",
              double(heading(0)), double(heading(1)), double(heading(2)));
      printf("\t%s\n", buf);
      printf("\tcmd_set = %s\n", cmd_set ? "true" : "false");
    }
  } cascaded_position_command_t;

  typedef struct CascadedPositionCommandGains
  {
    math::Vector<3> kp;
    math::Vector<3> kd;

    bool gains_set;

    CascadedPositionCommandGains() : gains_set(false)
    {
      kp.zero();
      kd.zero();
    }

    void print()
    {
      // Hack to work around NuttX printf float bug
      puts("pos cmd gains:");
      char buf[128];
      sprintf(buf, "kp gains = [%0.2f, %0.2f, %0.2f]",
              double(kp(0)), double(kp(1)), double(kp(2)));
      printf("\t%s\n", buf);
      sprintf(buf, "kd gains = [%0.2f, %0.2f, %0.2f]",
              double(kd(0)), double(kd(1)), double(kd(2)));
      printf("\t%s\n", buf);
      printf("\tgains_set = %s\n", gains_set ? "true" : "false");
    }
  } cascaded_position_command_gains_t;

  bool mocapPositionCommandMessageCallback(const mocap_position_command_s& in)
  {
    if (!cmd_gains.gains_set)
      return false;

    cmd.pos.set(in.pos);
    cmd.vel.set(in.vel);
    cmd.acc.set(in.acc);
    cmd.jerk.set(in.jerk);
    cmd.heading.set(in.heading);
    cmd.cmd_set = true;

    return true;
  }

  bool mocapPositionCommandGainsMessageCallback(const mocap_position_command_gains_s& in)
  {
    cmd_gains.kp.set(in.kp);
    cmd_gains.kd.set(in.kd);
    cmd_gains.gains_set = true;

    return true;
  }

  void printLocalPosition()
  {
    // Hack to work around NuttX printf float bug
    puts("local position:");
    char buf[128];
    sprintf(buf, "pos = [%0.2f, %0.2f, %0.2f]",
            double(local_pos.x), double(local_pos.y), double(local_pos.z));
    printf("%s\n", buf);
    sprintf(buf, "vel = [%0.2f, %0.2f, %0.2f]",
            double(local_pos.vx), double(local_pos.vy), double(local_pos.vz));
    printf("%s\n", buf);
  }

  void localPositionMessageCallback(const vehicle_local_position_s& local_pos_)
  {
    memcpy(&local_pos, &local_pos_, sizeof(local_pos_));
    local_pos_set = true;

#if 0
    static unsigned int counter = 0;
    if (counter++ > 100)
    {
      printLocalPosition();
      counter = 0;
    }
#endif
  }

  bool ctrl_state_set;
  bool local_pos_set;

  hrt_abstime cmd_time;
  hrt_abstime cmd_ttl_us;

  struct control_state_s ctrl_state;
  struct vehicle_local_position_s local_pos;
  cascaded_attitude_command_t att_cmd;
  cascaded_position_command_t cmd;
  cascaded_position_command_gains_t cmd_gains;

  int mocap_position_cmd_sub;
  int mocap_position_cmd_gains_sub;
  int local_position_sub;

  math::Vector<3> gravity;
  math::Vector<3> e3;
  float mass;
  
  /*float dt_;  // THIS IS EPC CODE BASED dt, replace this with dt obtained like Moses's code later
  
  int HORIZON_LENGTH;
  int NUM_STATES;
  int NUM_INPUTS;
  bool epcSolutionFound = false;  
  */

  L1PositionObserver l1_pos_observer;
  math::Vector<4> current_rpm_cmd;
};

#endif
