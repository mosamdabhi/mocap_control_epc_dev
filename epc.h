#ifndef EPC_H
#define EPC_H

#pragma once

void initEPCMats();


#include "epc_matrixmath.h"

#define MULT_GAMMAROW_MATRIX(ROWS, R, OUT) \
    multGamRowsMat(ROWS ## _rows, ROWS, R ## _cols, R, OUT); \
    OUT ## _rows = ROWS ## _rows; \
    OUT ## _cols = R ## _cols;

#define MULT_MATRIX_GAMMAROW_TRANSPOSED(ROWS, L, OUT) \
    multMatGamRowsT(ROWS ## _rows, ROWS, L ## _rows, L ## _cols, L, OUT); \
    OUT ## _rows = L ## _rows; \
    OUT ## _cols = ROWS ## _rows; \

#define GAMMAVECTOR_ROWS(IDXS) \
    getCgamRows(IDXS ## _rows, IDXS); \
    C_gam_offset_rows = IDXS ## _rows;    

    
struct Element
{
  uint8_t id;
  uint8_t active_set_size;
  uint8_t* active_set;
  uint8_t successors_size;
  uint8_t* successors;
};  
#include "epc_autogen.h"
CONSTRUCT_STATIC_MAT_ONLY_ROWS_NO_COLS(C_idxs, N*NUM_CONSTR, 1, uint16_t);  
CONSTRUCT_STATIC_MAT_ONLY_COLS_NO_ROWS(C_r_minus_c, N*NUM_STATES, 1, float);  // cal_r
CONSTRUCT_STATIC_MAT(temp1, N*NUM_CTRLS, N*NUM_STATES, float);
//CONSTRUCT_STATIC_MAT(matrix_K, N*NUM_CTRLS, N*NUM_STATES, float);
CONSTRUCT_STATIC_MAT(C_ctrl, N*NUM_CTRLS, 1, float);
CONSTRUCT_STATIC_MAT(C_BtQrc, N*NUM_CTRLS, 1, float);
//CONSTRUCT_STATIC_MAT(C_BtQ, N*NUM_CTRLS, N*NUM_STATES, float);


CONSTRUCT_STATIC_MAT(C_E1, N*NUM_CONSTR, N*NUM_CTRLS, float);     // E1
CONSTRUCT_STATIC_MAT(C_E2, N*NUM_CONSTR, N*NUM_CONSTR, float);    // E2
CONSTRUCT_STATIC_MAT(C_E3, N*NUM_CTRLS, N*NUM_CONSTR, float);     // E3
CONSTRUCT_STATIC_MAT(C_E4, N*NUM_CTRLS, N*NUM_CTRLS, float);
CONSTRUCT_STATIC_MAT(C_gam_offset, N*NUM_CONSTR, 1, float);
CONSTRUCT_STATIC_MAT(C_lam_a, N*NUM_CONSTR, 1, float);


static bool init_epc = false;
static uint16_t curr_id;
static float ctrl_use[6];

static void multGamRowsMat(uint16_t num_rows, uint16_t* rows,
                           uint16_t R_cols, float* R, float* out)
{
  uint16_t row_idx, col_idx, r_i, c_i, i, j, kk;
  for (i=0; i<num_rows; i++)
  {
    r_i = rows[i];
    // Populate row i of out via:
    if (r_i < 2*N*NUM_CONSTR_STATES)
    {
      int neg = -1;
      if (r_i < N*NUM_CONSTR_STATES)
        neg = 1;

      row_idx = r_i%NUM_CONSTR_STATES;
      col_idx = N*NUM_CTRLS - ((r_i/NUM_CONSTR_STATES)%N)*NUM_CTRLS;
      //multiply appropriate row of C_GxB with neg * all cols of R for rows up to num remaining cols in C_GxB
      for (j=0; j<R_cols; j++)
      {
        out[i*R_cols+j] = 0;
        for (kk = 0; kk<C_GxB_cols-col_idx+NUM_CTRLS; kk++)
        {
          out[i*R_cols+j] += neg * C_GxB[row_idx*C_GxB_cols+kk] * R[kk*R_cols+j];
        }
      }
    }
    else
    {
      c_i = (r_i - 2*N*NUM_CONSTR_STATES)%(N*NUM_CTRLS);
      memcpy(out+(i*R_cols), R+(c_i*R_cols), R_cols*sizeof(float));
      if (r_i >= N*NUM_CONSTR_STATES + N*NUM_CTRLS)
      {
        for (j=0; j<R_cols; j++)
        {
          out[i*R_cols+j] = -out[i*R_cols+j];
        }
      }
    }
  }
}

static void multMatGamRowsT(uint16_t num_idxs, uint16_t* idxs,
                            uint16_t L_rows, uint16_t L_cols,
                            float* L, float* out)
{
  uint16_t row_idx, col_idx, r_j, c_j, i, j, kk;
  for (i=0; i<L_rows; i++)
  {
    for (j=0; j<num_idxs; j++)
    {
      c_j = idxs[j];

      if (c_j < 2*N*NUM_CONSTR_STATES)
      {
        int neg = -1;
        if (c_j < N*NUM_CONSTR_STATES)
          neg = 1;

        col_idx = c_j%NUM_CONSTR_STATES;
        row_idx = N*NUM_CTRLS - ((c_j/NUM_CONSTR_STATES)%N)*NUM_CTRLS;
        out[i*num_idxs+j] = 0;
        for (kk = 0; kk<C_GxB_cols-row_idx+NUM_CTRLS; kk++)
        {
          out[i*num_idxs+j] += neg * L[i*L_cols+kk] * C_GxB[kk+L_cols*col_idx];
        }
      }
      else
      {
        r_j = (c_j - 2*N*NUM_CONSTR_STATES)%(N*NUM_CTRLS);

        int neg = -1;
        if (c_j < 2*N*NUM_CONSTR_STATES + N*NUM_CTRLS)
          neg = 1;

        out[i*num_idxs+j] = neg * L[i*L_cols+r_j];
      }
    }
  }
}

static void getCgamRows(uint16_t num_rows, uint16_t* rows)
{
  uint16_t rows_i, idx;
  for (uint16_t i=0; i<num_rows; i++)
  {
    rows_i = rows[i];
    if (rows_i < 2*N*NUM_CONSTR_STATES)
    {
      idx = rows_i%NUM_CONSTR_STATES;
      if (rows_i >= N*NUM_CONSTR_STATES)
        idx += NUM_CONSTR_STATES;
    }
    else
    {
      rows_i -= 2*N*NUM_CONSTR_STATES;
      idx = rows_i%NUM_CTRLS + 2*NUM_CONSTR_STATES;
      if (rows_i >= N*NUM_CTRLS)
        idx += NUM_CTRLS;
//       std::cout << rows[i] << ", " << (int)rows_i << ", " << (int)idx << std::endl;
    }
    C_gam_offset[i] = C_gam[idx];
//     DEBUG_PRINT("%f\n",C_gam[idx]-C_gam[ rows[i] ]);
  }
}

static int16_t failed_idx = -1;
static float failed_constr = 0;

static bool check_Constraints(math::Vector<3> &pos_for_checkConstraints,
                              math::Vector<3> &vel_for_checkConstraints)
{
  uint16_t i, row_idx, col_idx, kk, c_i;
  bool meets_constraints = true;
  for(i=0; i<N*NUM_CONSTR;i++)
  {
    float constr_lhs = 0;
    float constr_rhs; // = C_gam[i];

    if (i < 2*N*NUM_CONSTR_STATES)
    {

      row_idx = i%NUM_CONSTR_STATES;
      col_idx = N*NUM_CTRLS - ((i/NUM_CONSTR_STATES)%N)*NUM_CTRLS;      

      int neg = -1;
      if (i < N*NUM_CONSTR_STATES)
      {
        constr_rhs = C_gam[row_idx];
        neg = 1;
      }
      else
      {
        constr_rhs = C_gam[row_idx+NUM_CONSTR_STATES];
      }

      // Implements Gx*x_0
      if ((i%NUM_CONSTR_STATES)==0) constr_lhs += neg*vel_for_checkConstraints(0);
      if ((i%NUM_CONSTR_STATES)==1) constr_lhs += neg*vel_for_checkConstraints(1);      


      for (kk = 0; kk<C_GxB_cols-col_idx+NUM_CTRLS; kk++)
      {
        constr_lhs += neg * C_GxB[row_idx*C_GxB_cols+kk] * C_ctrl[kk];
      }      
    }
    else
    {
      c_i = (i - 2*N*NUM_CONSTR_STATES)%(N*NUM_CTRLS);
      constr_lhs = C_ctrl[c_i];

      row_idx = 2*NUM_CONSTR_STATES + (i-2*N*NUM_CONSTR_STATES)%(NUM_CTRLS);
      if (i >= 2*N*NUM_CONSTR_STATES + N*NUM_CTRLS)
      {
        constr_lhs = -constr_lhs;
        row_idx += NUM_CTRLS;
      }
      constr_rhs = C_gam[row_idx];
    }



    if ( (constr_lhs - constr_rhs) > 1e-2f )
    {
      meets_constraints = false;
      failed_idx = i;
      failed_constr = constr_lhs;
      break;    
    }
    else
    {
      failed_idx = -1;
      failed_constr = 0;
    }
  }
  return meets_constraints; 
}

class epc
{
public:
	epc() {}
	~epc() {}	

    void initEPCMats()
    {
      initDatabase();

      curr_id = 0;

      for (uint16_t i=0; i<6; i++)
        ctrl_use[i] = 0;

      init_epc = true;
      //printf("check 1\n");
    }   

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
    MULTIPLYING_MATRIX(cal_B_transpose, cal_Q, temp1);
    MULTIPLYING_MATRIX(temp1, C_r_minus_c, C_BtQrc);
    //printf("check 3\n");
  }


	//bool epc_logic(struct vec* force_world, float g_vehicleMass)
	// Change this to pointer based logic
	bool epc_logic(math::Vector<3> &world_force, math::Vector<3> &pos_, math::Vector<3> &vel_,
					math::Vector<3> &cmd_pos_, math::Vector<3> &cmd_vel_, math::Vector<3> &cmd_acc_, float g_vehicleMass, float GRAV)
	{
        // initially, ctrl_set logic is kept false
        //bool ctrl_set = false;

        // initializing EPC database
        if(!init_epc)
            initEPCMats();

        // Fetching the database
        for(uint16_t idx=0; idx < 20 && idx < ctrl_db[curr_id].successors_size; idx++)
        {
            uint16_t s_idx = ctrl_db[curr_id].successors[idx];

            // Parse each active set
            for(uint16_t i=0; i<ctrl_db[s_idx].active_set_size; i++)
                C_idxs[i] = ctrl_db[s_idx].active_set[i];
            C_idxs_rows = ctrl_db[s_idx].active_set_size;


            // If active set isn't empty, computing Lagrange Multipliers
            if(C_idxs_rows > 0)
            {

                MULT_GAMMAROW_MATRIX(C_idxs, C_Hinv, C_E1)
                MULT_MATRIX_GAMMAROW_TRANSPOSED(C_idxs, C_E1, C_E2)
                INV_MAT_SYMPD(C_E2, C_E2)
                NEGATE_MATRIX(C_E2)
                MULT_MATRIX_TRANSPOSED_MATRIX(C_E1, C_E2, C_E3)

                instantiate_r_minus_c(pos_, vel_, cmd_pos_, cmd_vel_);
                MULT_NEGATE_MATRIX_TRANSPOSED_MATRIX(C_E3, C_BtQrc, C_lam_a)
                GAMMAVECTOR_ROWS(C_idxs);

                //Implementing Gx*x_0
                for(uint16_t j=0; j<C_idxs_rows; j++)
                {
                    int8_t neg = -1;
                    if ( C_idxs[j] < N*NUM_CONSTR_STATES )
                      neg = 1;

                    if ((C_idxs[j]%NUM_CONSTR_STATES)==0) C_gam_offset[j] -= neg*vel_(0);
                    if ((C_idxs[j]%NUM_CONSTR_STATES)==1) C_gam_offset[j] -= neg*vel_(1);
                }
                MAC_MATRIX(C_lam_a, C_E2, C_gam_offset)

            }
            else
            {
              C_lam_a_rows = 0;
            }
            // If Lagrange multipliers are negative, controller is not optimal, discard it
            if(C_lam_a_rows > 0)
            {
                bool failed = false;
                for(uint16_t i=0; i<C_lam_a_rows; i++)
                {
                    if(C_lam_a[i] < -1e-6f)
                    {
                        failed = true;
                        break;
                    }
                }
                if(failed)
                    continue;
            }

            if (C_lam_a_rows > 0)
            {

                COPY_MATRIX(C_Hinv, C_E4)
                MAC_MATRIX(C_E4, C_E3, C_E1)
                MULTIPLYING_MATRIX(C_E4, C_BtQrc, C_ctrl)
                MAC_NEGATE_MATRIX(C_ctrl, C_E3, C_gam_offset)

            }
            else
            {  
                instantiate_r_minus_c(pos_, vel_, cmd_pos_, cmd_vel_);
                MULTIPLYING_MATRIX(C_Hinv, C_BtQrc, C_ctrl)
            }
        }


    
//    instantiate_r_minus_c(pos_, vel_, cmd_pos_, cmd_vel_);  
//    MULTIPLYING_MATRIX(C_Hinv, cal_B_transpose, temp1);
//    MULTIPLYING_MATRIX(temp1, cal_Q, matrix_K);    

    //MULTIPLYING_MATRIX(C_Hinv, C_BtQ, matrix_K);

    //DIVIDE_MASS_MATRIX(matrix_K);  
    
    #if 0
    printf("x_pos_gain is: %3.6f\n", double(matrix_K[0]+matrix_K[6]));
    printf("y_pos_gain is: %3.6f\n", double(matrix_K[13]+matrix_K[19]));
    printf("z_pos_gain is: %3.6f\n", double(matrix_K[26]+matrix_K[32]));
    printf("x_vel_gain is: %3.6f\n", double(matrix_K[3]+matrix_K[9]));
    printf("y_vel_gain is: %3.6f\n", double(matrix_K[16]+matrix_K[22]));
    printf("z_vel_gain is: %3.6f\n", double(matrix_K[29]+matrix_K[35]));
    
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
//    MULTIPLYING_MATRIX(matrix_K, C_r_minus_c, C_fin_ctrl);


    if(check_Constraints(pos_, vel_))
    {
      world_force(0) = C_ctrl[0] + (cmd_acc_(0)*g_vehicleMass);
      world_force(1) = C_ctrl[1] + (cmd_acc_(0)*g_vehicleMass);
      world_force(2) = C_ctrl[2] + ((cmd_acc_(0) - GRAV)*g_vehicleMass);      
      //printf("Got the fin\n");

      return true;
      //printf("Check here\n");
    }
    else
      return false;


    //printf("matrix K: %3.5f\n", double(matrix_K[125]+matrix_K[131]+matrix_K[137]+matrix_K[143]+matrix_K[149]+matrix_K[155]+matrix_K[161]+matrix_K[167]+matrix_K[173]+matrix_K[179]));
    

       
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
  }

};
#endif