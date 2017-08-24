#pragma once

#include "cholsl.h"

// For initializing and constructing static dimensioned matrices (without data)
#define CONSTRUCT_STATIC_MAT(NAME, R, C, TYPE) \
    static uint16_t NAME ## _rows = R; \
    static uint16_t NAME ## _cols = C; \
    static TYPE NAME[R*C];

// For initializing and constructing static dimensioned matrices (without data) with no rows&cols
#define CONSTRUCT_STATIC_MAT_NO_ROWS_COLS(NAME, R, C, TYPE) \
    static TYPE NAME[R*C];

// For initializing and constructing static dimensioned matrices (without data) with only rows no cols
#define CONSTRUCT_STATIC_MAT_ONLY_ROWS_NO_COLS(NAME, R, C, TYPE) \
    static uint16_t NAME ## _rows = R; \
    static TYPE NAME[R*C];

// For initializing and constructing static dimensioned matrices (without data) with only cols no rows
#define CONSTRUCT_STATIC_MAT_ONLY_COLS_NO_ROWS(NAME, R, C, TYPE) \
    static uint16_t NAME ## _cols = C; \
    static TYPE NAME[R*C];    


// Same as above, just adding actual matrix data here
#define CONSTRUCT_STATIC_MAT_WITH_DATA(NAME, R, C, TYPE, DATA) \
    static uint16_t NAME ## _rows = R; \
    static uint16_t NAME ## _cols = C; \
    static TYPE NAME[R*C] = DATA;

// Same as above, just adding actual matrix data here for C_affine_c, C_Hinv, C_gam
#define CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_NO_ROWS_COLS(NAME, R, C, TYPE, DATA) \
    static TYPE NAME[R*C] = DATA;

// Same as above, just adding actual matrix data here for only cols and no rows
#define CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_ONLY_COLS_NO_ROWS(NAME, R, C, TYPE, DATA) \
    static uint16_t NAME ## _cols = C; \
    static TYPE NAME[R*C] = DATA;        

// For adding data in above DEF, this is used
#define ARG_DATA(...) __VA_ARGS__

// Copy the Matrix
#define COPY_MATRIX(IN, OUT) \
	memcpy(OUT, IN, sizeof(IN));

// Negating the matroix
#define NEGATE_MATRIX(IN) \
	for(uint16_t i=0; i < IN ## _rows * IN ## _cols; i++) IN[i] *= -1; 	

/*Multiplying Matrix L*R and outputting in matrix OUT*/
#define MULTIPLYING_MATRIX(L,R,OUT) \
    multMat(L ## _rows, L ## _cols, R ## _cols, L, R, OUT, 1); \
    OUT ## _rows = L ## _rows; \
    OUT ## _cols = R ## _cols;

/* IN = IN+(L*R) */
#define MAC_MATRIX(IN,L,R) \
    macMat(L ## _rows, L ## _cols, R ## _cols, IN, L, R, 1); \

/* IN = IN + (L*-R) */
#define MAC_NEGATE_MATRIX(IN,L,R) \
    macMat(L ## _rows, L ## _cols, R ## _cols, IN, L, R, -1); \

/* OUT = (L'*R) */
#define MULT_MATRIX_TRANSPOSED_MATRIX(L,R,OUT) \
    multMatTMat(L ## _rows, L ## _cols, R ## _cols, L, R, OUT, 1); \
    OUT ## _rows = L ## _cols; \
    OUT ## _cols = R ## _cols;

/* OUT = (-L'*R) */
#define MULT_NEGATE_MATRIX_TRANSPOSED_MATRIX(L,R,OUT) \
    multMatTMat(L ## _rows, L ## _cols, R ## _cols, L, R, OUT, -1); \
    OUT ## _rows = L ## _cols; \
    OUT ## _cols = R ## _cols;

/* OUT = inverse(IN) */
    // USe when needed. Comment now

#define INV_MAT_SYMPD(IN,OUT) \
    { \
      float scratch[IN ## _rows]; \
      cholsl(IN, OUT, scratch, IN ## _rows); \
      OUT ## _rows = IN ## _rows; \
      OUT ## _cols = IN ## _cols; \
    }

    

/* Extracting Rows of Matrices */
/* OUT = IN([ROWS]) */
#define MATRIX_ROWS(IN,OUT,ROWS) \
    getMatRows(IN ## _cols, IN, OUT, ROWS ## _rows, ROWS); \
    OUT ## _rows = ROWS ## _rows; \
    OUT ## _cols = IN ## _cols;    


static void multMat(uint16_t L_rows, uint16_t L_cols,
                    uint16_t R_cols, float* L, float* R, float* out, int8_t neg)
{
  for (uint16_t i=0; i<L_rows; i++)
  {
    for (uint16_t j=0; j<R_cols; j++)
    {
      // multiply row i of L by col j of R
      out[i*R_cols+j] = 0;
      for (uint16_t kk = 0; kk<L_cols; kk++)
      {
        out[i*R_cols+j] += neg * L[i*L_cols+kk] * R[kk*R_cols+j];
      }
    }
  }
}

static void multMatTMat(uint16_t L_rows, uint16_t L_cols,
                        uint16_t R_cols,float* L, float* R,
                        float* out, int8_t neg)
{
  for (uint16_t i=0; i<L_cols; i++)
  {
    for (uint16_t j=0; j<R_cols; j++)
    {
      // multiply col i of L by col j of R
      out[i*R_cols+j] = 0;
      for (uint16_t kk = 0; kk<L_rows; kk++)
      {
        out[i*R_cols+j] += neg * L[i+L_cols*kk] * R[kk*R_cols+j];
      }
    }
  }
}

static void macMat(uint16_t L_rows, uint16_t L_cols, uint16_t R_cols,
                   float* in, float* L, float* R, int8_t neg)
{
  for (uint16_t i=0; i<L_rows; i++)
  {
    for (uint16_t j=0; j<R_cols; j++)
    {
      // multiply row i of L by col j of R
      for (uint16_t kk = 0; kk<L_cols; kk++)
      {
        in[i*R_cols+j] += neg * L[i*L_cols+kk] * R[kk*R_cols+j];
      }
    }
  }
}

static void getMatRows(uint16_t in_cols, float* in, float*  out,
                       uint16_t num_rows, uint16_t* rows)
{
  for (uint16_t i=0; i<num_rows; i++)
  {
    for (uint16_t j=0; j<in_cols; j++)
    {
      out[i*in_cols+j] = in[ rows[i]*in_cols + j ];
    }
  }
}