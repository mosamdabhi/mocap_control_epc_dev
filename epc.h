#ifndef EPC_H
#define EPC_H

#pragma once

void initEPCMats();


#include "epc_matrixmath.h"
    
  
#include "epc_autogen.h"
CONSTRUCT_STATIC_MAT_ONLY_COLS_NO_ROWS(C_r_minus_c, N*NUM_STATES, 1, float);  // cal_r
CONSTRUCT_STATIC_MAT(temp1, 3, 6, float);
CONSTRUCT_STATIC_MAT(matrix_K, 3, 6, float);
CONSTRUCT_STATIC_MAT(C_fin_ctrl, 3, 1, float);
//CONSTRUCT_STATIC_MAT(C_BtQ, N*NUM_CTRLS, N*NUM_STATES, float);

class epc
{
public:
	epc() {}
	~epc() {}	
  void instantiate_r_minus_c(math::Vector<3> &pos_inside_setPoint_,
                             math::Vector<3> &vel_inside_setPoint_,
                             math::Vector<3> &cmd_pos_inside_setPoint_,
                             math::Vector<3> &cmd_vel_inside_setPoint_)
  {
    for (uint16_t i=0; i<N*NUM_STATES; i+=NUM_STATES)
    {
      C_r_minus_c[i] = (cmd_pos_inside_setPoint_(0) - pos_inside_setPoint_(0));
      C_r_minus_c[i+1] = (cmd_pos_inside_setPoint_(1) - pos_inside_setPoint_(1));
      C_r_minus_c[i+2] = (cmd_pos_inside_setPoint_(2) - pos_inside_setPoint_(2));
      C_r_minus_c[i+3] = (cmd_vel_inside_setPoint_(0) - vel_inside_setPoint_(0));
      C_r_minus_c[i+4] = (cmd_vel_inside_setPoint_(1) - vel_inside_setPoint_(1));
      C_r_minus_c[i+5] = (cmd_vel_inside_setPoint_(2) - vel_inside_setPoint_(2));
    }
  }


	//bool epc_logic(struct vec* force_world, float g_vehicleMass)
	// Change this to pointer based logic
	bool epc_logic(math::Vector<3> &world_force, math::Vector<3> &pos_, math::Vector<3> &vel_,
					math::Vector<3> &cmd_pos_, math::Vector<3> &cmd_vel_, math::Vector<3> &cmd_acc_, float g_vehicleMass, float GRAV)
	{
    
    instantiate_r_minus_c(pos_, vel_, cmd_pos_, cmd_vel_);

    

    
    MULTIPLYING_MATRIX(C_Hinv, cal_B_transpose, temp1);
    MULTIPLYING_MATRIX(temp1, cal_Q, matrix_K);    

    DIVIDE_MASS_MATRIX(matrix_K);  
    #if 0  
    printf("matrix (1,1) is: %3.6f\n", double(matrix_K[0]));
    printf("matrix (1,2) is: %3.6f\n", double(matrix_K[1]));
    printf("matrix (1,3) is: %3.6f\n", double(matrix_K[2]));
    printf("matrix (1,4) is: %3.6f\n", double(matrix_K[3]));
    printf("matrix (1,5) is: %3.6f\n", double(matrix_K[4]));
    printf("matrix (1,6) is: %3.6f\n", double(matrix_K[5]));
    printf("matrix (2,1) is: %3.6f\n", double(matrix_K[6]));
    printf("matrix (2,2) is: %3.6f\n", double(matrix_K[7]));
    printf("matrix (2,3) is: %3.6f\n", double(matrix_K[8]));
    printf("matrix (2,4) is: %3.6f\n", double(matrix_K[9]));
    printf("matrix (2,5) is: %3.6f\n", double(matrix_K[10]));
    printf("matrix (2,6) is: %3.6f\n", double(matrix_K[11]));
    printf("matrix (3,1) is: %3.6f\n", double(matrix_K[12]));
    printf("matrix (3,2) is: %3.6f\n", double(matrix_K[13]));
    printf("matrix (3,3) is: %3.6f\n", double(matrix_K[14]));
    printf("matrix (3,4) is: %3.6f\n", double(matrix_K[15]));
    printf("matrix (3,5) is: %3.6f\n", double(matrix_K[16]));
    printf("matrix (3,6) is: %3.6f\n", double(matrix_K[17]));    
    #endif
    MULTIPLYING_MATRIX(matrix_K, C_r_minus_c, C_fin_ctrl);





    //printf("matrix K: %3.5f\n", double(matrix_K[125]+matrix_K[131]+matrix_K[137]+matrix_K[143]+matrix_K[149]+matrix_K[155]+matrix_K[161]+matrix_K[167]+matrix_K[173]+matrix_K[179]));
    
    world_force(0) = (C_fin_ctrl[0] + cmd_acc_(0))*g_vehicleMass;
    world_force(1) = (C_fin_ctrl[1] + cmd_acc_(0))*g_vehicleMass;
    world_force(2) = (C_fin_ctrl[2] + cmd_acc_(0) - GRAV)*g_vehicleMass;
       
        

    #if 0
    //DIVIDE_MASS_MATRIX(matrix_K);
    printf("C_r_minus_c is: %3.6f\n", double(C_r_minus_c[0]));
    printf("C_r_minus_c is: %3.6f\n", double(C_r_minus_c[1]));
    printf("C_r_minus_c is: %3.6f\n", double(C_r_minus_c[2]));
    printf("C_r_minus_c is: %3.6f\n", double(C_r_minus_c[3]));
    printf("C_r_minus_c is: %3.6f\n", double(C_r_minus_c[4]));
    printf("C_r_minus_c is: %3.6f\n", double(C_r_minus_c[5]));
    #endif

    //#endif
    //world_force(0) = (C_ctrl[0] + cmd_acc_(0))  * g_vehicleMass;
    //world_force(1) = (C_ctrl[1] + cmd_acc_(1)) * g_vehicleMass; 
    //world_force(2) = (C_ctrl[2] + (cmd_acc_(2) - GRAV)) * g_vehicleMass;    
    return true;
  }

};
#endif