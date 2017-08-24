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

#define MULT_BQr() \
    multBQr(); \
    C_BtQrc_rows = N*C_BtQrow_rows; \
    C_BtQrc_cols = 1;    
    

struct Element
{
  uint16_t id;
  uint16_t active_set_size;
  uint16_t* active_set;
  uint16_t successors_size;
  uint16_t* successors;
};

#include "epc_autogen.h"

CONSTRUCT_STATIC_MAT_ONLY_ROWS_NO_COLS(C_idxs, N*NUM_CONSTR, 1, uint16_t);  
CONSTRUCT_STATIC_MAT(C_E1, N*NUM_CONSTR, N*NUM_CTRLS, float);     // E1
CONSTRUCT_STATIC_MAT(C_E2, N*NUM_CONSTR, N*NUM_CONSTR, float);    // E2
CONSTRUCT_STATIC_MAT(C_E3, N*NUM_CTRLS, N*NUM_CONSTR, float);     // E3

CONSTRUCT_STATIC_MAT_NO_ROWS_COLS(C_r_minus_c, N*NUM_STATES, 1, float);  // cal_r
CONSTRUCT_STATIC_MAT(C_BtQrc, N*NUM_CTRLS, 1, float);
CONSTRUCT_STATIC_MAT(C_lam_a, N*NUM_CONSTR, 1, float);
CONSTRUCT_STATIC_MAT(C_gam_offset, N*NUM_CONSTR, 1, float);

CONSTRUCT_STATIC_MAT(C_E4, N*NUM_CTRLS, N*NUM_CTRLS, float);
CONSTRUCT_STATIC_MAT(C_ctrl, N*NUM_CTRLS, 1, float);


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

static void multBQr()
{
  uint16_t row_idx, col_idx, i, kk;
  for (i=0; i<N*NUM_CTRLS; i++)
  {
    row_idx = i%NUM_CTRLS;
    col_idx = ((i/NUM_CTRLS)%N)*NUM_STATES;

    if (i % 3 == 2)
      C_BtQrc[i] = 0.183054852; // NOTE: R*u_nom in z (u_nom = m*g)
    else
      C_BtQrc[i] = 0;
    for (kk = 0; kk<(N*NUM_STATES - col_idx); kk++)
    {
      C_BtQrc[i] += C_BtQrow[row_idx*C_BtQrow_cols+kk] * C_r_minus_c[col_idx+kk];
    }
  }
}

static int16_t failed_idx = -1;
static float failed_constr = 0;

static bool check_for_Constraints(math::Vector<3> &pos_for_checkConstraints,
                                  math::Vector<3> &vel_for_checkConstraints)
{
  uint16_t i, row_idx, col_idx, kk, c_i;
  bool meets_constraints = true;
  for(i=0; i<N*NUM_CONSTR;i++)
  {
    float constr_lhs = 0;
    float constr_rhs = C_gam[i];

    if (i < 2*N*NUM_CONSTR_STATES)
    {
      int neg = -1;
      if (i < N*NUM_CONSTR_STATES)
        neg = 1;

      // Implements Gx*x_0
      if ((i%NUM_CONSTR_STATES)==0) constr_lhs += neg*vel_for_checkConstraints(0);
      if ((i%NUM_CONSTR_STATES)==1) constr_lhs += neg*vel_for_checkConstraints(1);      

      row_idx = i%NUM_CONSTR_STATES;
      col_idx = N*NUM_CTRLS - ((i/NUM_CONSTR_STATES)%N)*NUM_CTRLS;

      for (kk = 0; kk<C_GxB_cols-col_idx+NUM_CTRLS; kk++)
      {
        constr_lhs += neg * C_GxB[row_idx*C_GxB_cols+kk] * C_ctrl[kk];
      }      
    }
    else
    {
      c_i = (i - 2*N*NUM_CONSTR_STATES)%(N*NUM_CTRLS);
      constr_lhs = C_ctrl[c_i];
      if (i >= N*NUM_CONSTR_STATES + N*NUM_CTRLS)
      {
        constr_lhs = -constr_lhs;
      }      
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
	}	

  void instantiate_r_minus_c(math::Vector<3> &pos_inside_setPoint_,
                             math::Vector<3> &vel_inside_setPoint_,
                             math::Vector<3> &cmd_pos_inside_setPoint_,
                             math::Vector<3> &cmd_vel_inside_setPoint_)
  {
    for (uint16_t i=0; i<N*NUM_STATES; i+=NUM_STATES)
    {
      C_r_minus_c[i] = (cmd_pos_inside_setPoint_(0) - pos_inside_setPoint_(0)) - C_affine_c[0];
      C_r_minus_c[i+1] = (cmd_pos_inside_setPoint_(1) - pos_inside_setPoint_(1)) - C_affine_c[1];
      C_r_minus_c[i+2] = (cmd_pos_inside_setPoint_(2) - pos_inside_setPoint_(2)) - C_affine_c[2];
      C_r_minus_c[i+3] = (cmd_vel_inside_setPoint_(0) - vel_inside_setPoint_(0)) - C_affine_c[3];
      C_r_minus_c[i+4] = (cmd_vel_inside_setPoint_(1) - vel_inside_setPoint_(1)) - C_affine_c[4];
      C_r_minus_c[i+5] = (cmd_vel_inside_setPoint_(2) - vel_inside_setPoint_(2)) - C_affine_c[5];
    }
    MULT_BQr()
  }


	//bool epc_logic(struct vec* force_world, float g_vehicleMass)
	// Change this to pointer based logic
	bool epc_logic(math::Vector<3> &world_force, math::Vector<3> &pos_, math::Vector<3> &vel_, 
					math::Vector<3> &cmd_pos_, math::Vector<3> &cmd_vel_, math::Vector<3> &cmd_acc_, float g_vehicleMass, float GRAV)
	{
		//printf("H_inv is: %3.6f\n", double(C_Hinv[899]));
		//printf("C_affine_c is %f\n", double(C_affine_c[0]));
		//printf("E1 is %f\n", double(C_E1[0]));

    bool ctrl_set = false;

	#if 0
    printf("local position inside x: %3.4f y: %3.4f z: %3.4f\n",
        (double) pos_(0),
        (double) pos_(1),
        (double) pos_(2));
    #endif		
	

		if(!init_epc)
			initEPCMats();

		
		for (uint16_t idx=0; idx < 20 && idx < ctrl_db[curr_id].successors_size; idx++)
		{
			uint16_t s_idx = ctrl_db[curr_id].successors[idx];

		    // Parse each active set
		    for (uint16_t i=0; i<ctrl_db[s_idx].active_set_size; i++)
		      C_idxs[i] = ctrl_db[s_idx].active_set[i];
		    C_idxs_rows = ctrl_db[s_idx].active_set_size;

		    // If active set is not empty, compute Lagrange multipliers
		    if (C_idxs_rows > 0)
		    {
		    	MULT_GAMMAROW_MATRIX(C_idxs, C_Hinv, C_E1)
          MULT_MATRIX_GAMMAROW_TRANSPOSED(C_idxs, C_E1, C_E2)
          INV_MAT_SYMPD(C_E2, C_E2)
          NEGATE_MATRIX(C_E2)
          MULT_MATRIX_TRANSPOSED_MATRIX(C_E1, C_E2, C_E3)

          instantiate_r_minus_c(pos_, vel_, cmd_pos_, cmd_vel_);
          MULT_NEGATE_MATRIX_TRANSPOSED_MATRIX(C_E3, C_BtQrc, C_lam_a)

          MATRIX_ROWS(C_gam, C_gam_offset, C_idxs)


          // Implements Gx*x_0
          for (uint16_t j=0; j<C_idxs_rows; j++)
          {
            int8_t neg = -1;
            if ( C_idxs[j] < N*NUM_CONSTR_STATES )
              neg = 1;

            if ((C_idxs[j]%NUM_CONSTR_STATES)==0) C_gam_offset[j] -= neg*vel_(0);
            if ((C_idxs[j]%NUM_CONSTR_STATES)==1) C_gam_offset[j] -= neg*vel_(1);
          }

          MAC_MATRIX(C_lam_a, C_E2, C_gam_offset)
          //printf("Still loop here?\n");
          printf("cal_r aew: %3.4f\n", double(C_r_minus_c[0]));
		    }
        else
        {
          C_lam_a_rows = 0;
        }

        // If any Lagrange multipliers are negative, this controller is not optimal

        if( C_lam_a_rows > 0 )
        {
          //printf("dabhi\n");
          bool failed = false;
          for (uint16_t i=0; i<C_lam_a_rows; i++)
          {
            if(C_lam_a[i] < -1e-6f)
            {
              failed = true;
              break;
            }
          }

          if (failed)
            continue;
        }
        //printf("C_lam_a_rows ismis: %3.4f\n", double(C_lam_a_rows));
        

        ///////////////////////////////////////////     NEW PART : 8/24/2017 | 1351 EST
        if ( C_lam_a_rows > 0 )
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

        // Checking that all the constraints are met
        if(check_for_Constraints(pos_, vel_))
        {
            ctrl_set = true;
            curr_id = s_idx;            

            for (uint16_t i=0; i<5; i++)
              ctrl_use[i] = ctrl_use[i+1];

            static float itr = 0;
            ctrl_use[5] = ((float)curr_id)*100;
            ctrl_use[5] += (++itr)*1e-6f;                          
            break;
        }
		}


    if(ctrl_set)
    {
      //#if 0
        world_force(0) = g_vehicleMass * cmd_acc_(0) + C_ctrl[0];
        world_force(1) = g_vehicleMass * cmd_acc_(1) + C_ctrl[1];
        world_force(2) = g_vehicleMass * (cmd_acc_(2) + GRAV) + C_ctrl[2];
      //#endif

        printf("Got the control input\n");
        return true;
    }
    else
      return false;
		
	}

};
#endif