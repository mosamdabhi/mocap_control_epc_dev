#pragma once


// For initializing and constructing static dimensioned matrices (without data)
#define CONSTRUCT_STATIC_MAT(NAME, R, C, TYPE) \
    static uint16_t NAME ## _rows = R; \
    static uint16_t NAME ## _cols = C; \
    static TYPE NAME[R*C];

// For initializing and constructing static dimensioned matrices (without data) with no rows&cols
#define CONSTRUCT_STATIC_MAT_NO_ROWS_COLS(NAME, R, C, TYPE) \
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

#define CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_ONLY_COLS_NO_ROWS(NAME, R, C, TYPE, DATA) \
    static uint16_t NAME ## _cols = C; \
    static TYPE NAME[R*C] = DATA;    


// For adding data in above DEF, this is used
#define ARG_DATA(...) __VA_ARGS__


// Negating the matroix
#define DIVIDE_MASS_MATRIX(IN) \
  for(uint16_t i=0; i < IN ## _rows * IN ## _cols; i++) IN[i] /= 0.7880f;    

/*Multiplying Matrix L*R and outputting in matrix OUT*/
#define MULTIPLYING_MATRIX(L,R,OUT) \
    multMat(L ## _rows, L ## _cols, R ## _cols, L, R, OUT, 1); \
    OUT ## _rows = L ## _rows; \
    OUT ## _cols = R ## _cols;

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
