/*******************************/
/* Auto-generated. Do not edit */
/*******************************/
// c_epc_autogen - 4

#pragma once
void initDatabase();

enum
{
  N = 10,
  NUM_STATES = 6,
  NUM_CTRLS = 3,
  NUM_CONSTR_STATES = 3,
  NUM_CONSTR = 2*(NUM_CONSTR_STATES+NUM_CTRLS),
  DB_SIZE = 25,
  MAX_SUCCESSORS = 20
};

static struct Element ctrl_db[DB_SIZE];


/*Auto-generated values from c_epc_autogen.h*/
/*Some matrices like these are generated from this part*/




CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_NO_ROWS_COLS(C_affine_c, NUM_STATES, 1, float,
  ARG_DATA({0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000}));


CONSTRUCT_STATIC_MAT_WITH_DATA(C_BtQrow, NUM_CTRLS, N*NUM_STATES, float,
  ARG_DATA({0.002832, 0, 0, 0.009561, 0, 0, 0.008495, 0, 0, 0.009561, 0, 0, 0.014159, 0, 0, 0.009561, 0, 0, 0.019822, 0, 0, 0.009561, 0, 0, 0.025486, 0, 0, 0.009561, 0, 0, 0.031149, 0, 0, 0.009561, 0, 0, 0.036812, 0, 0, 0.009561, 0, 0, 0.042476, 0, 0, 0.009561, 0, 0, 0.048139, 0, 0, 0.009561, 0, 0, 0.053803, 0, 0, 0.009561, 0, 0, 0, 0.002832, 0, 0, 0.009561, 0, 0, 0.008495, 0, 0, 0.009561, 0, 0, 0.014159, 0, 0, 0.009561, 0, 0, 0.019822, 0, 0, 0.009561, 0, 0, 0.025486, 0, 0, 0.009561, 0, 0, 0.031149, 0, 0, 0.009561, 0, 0, 0.036812, 0, 0, 0.009561, 0, 0, 0.042476, 0, 0, 0.009561, 0, 0, 0.048139, 0, 0, 0.009561, 0, 0, 0.053803, 0, 0, 0.009561, 0, 0, 0, 0.004938, 0, 0, 0.020503, 0, 0, 0.014814, 0, 0, 0.020503, 0, 0, 0.024691, 0, 0, 0.020503, 0, 0, 0.034567, 0, 0, 0.020503, 0, 0, 0.044443, 0, 0, 0.020503, 0, 0, 0.054320, 0, 0, 0.020503, 0, 0, 0.064196, 0, 0, 0.020503, 0, 0, 0.074072, 0, 0, 0.020503, 0, 0, 0.083949, 0, 0, 0.020503, 0, 0, 0.093825, 0, 0, 0.020503}));

CONSTRUCT_STATIC_MAT_WITH_DATA(C_Hinv, N*NUM_CTRLS, N*NUM_CTRLS, float,
  ARG_DATA({54.788049, 0, 0, -3.332081, 0, 0, -2.871675, 0, 0, -2.435978, 0, 0, -2.023948, 0, 0, -1.634612, 0, 0, -1.267068, 0, 0, -0.920502, 0, 0, -0.594193, 0, 0, -0.287526, 0, 0, 0, 54.788049, 0, 0, -3.332081, 0, 0, -2.871675, 0, 0, -2.435978, 0, 0, -2.023948, 0, 0, -1.634612, 0, 0, -1.267068, 0, 0, -0.920502, 0, 0, -0.594193, 0, 0, -0.287526, 0, 0, 0, 39.167599, 0, 0, -3.305647, 0, 0, -2.823582, 0, 0, -2.376885, 0, 0, -1.962365, 0, 0, -1.577062, 0, 0, -1.218233, 0, 0, -0.883349, 0, 0, -0.570076, 0, 0, -0.276278, -3.332081, 0, 0, 55.328278, 0, 0, -2.833517, 0, 0, -2.411000, 0, 0, -2.009590, 0, 0, -1.628411, 0, 0, -1.266645, 0, 0, -0.923545, 0, 0, -0.598442, 0, 0, -0.290755, 0, 0, 0, -3.332081, 0, 0, 55.328278, 0, 0, -2.833517, 0, 0, -2.411000, 0, 0, -2.009590, 0, 0, -1.628411, 0, 0, -1.266645, 0, 0, -0.923545, 0, 0, -0.598442, 0, 0, -0.290755, 0, 0, 0, -3.305647, 0, 0, 39.717365, 0, 0, -2.806687, 0, 0, -2.369564, 0, 0, -1.962323, 0, 0, -1.582113, 0, 0, -1.226282, 0, 0, -0.892361, 0, 0, -0.578060, 0, 0, -0.281259, -2.871675, 0, 0, -2.833517, 0, 0, 55.791109, 0, 0, -2.402933, 0, 0, -2.009326, 0, 0, -1.633630, 0, 0, -1.275106, 0, 0, -0.933067, 0, 0, -0.606889, 0, 0, -0.296023, 0, 0, 0, -2.871675, 0, 0, -2.833517, 0, 0, 55.791109, 0, 0, -2.402933, 0, 0, -2.009326, 0, 0, -1.633630, 0, 0, -1.275106, 0, 0, -0.933067, 0, 0, -0.606889, 0, 0, -0.296023, 0, 0, 0, -2.823582, 0, 0, -2.806687, 0, 0, 40.173307, 0, 0, -2.388425, 0, 0, -1.983962, 0, 0, -1.604646, 0, 0, -1.247881, 0, 0, -0.911235, 0, 0, -0.592432, 0, 0, -0.289348, -2.435978, 0, 0, -2.411000, 0, 0, -2.402933, 0, 0, 56.194826, 0, 0, -2.022987, 0, 0, -1.650172, 0, 0, -1.292405, 0, 0, -0.949056, 0, 0, -0.619544, 0, 0, -0.303344, 0, 0, 0, 
  -2.435978, 0, 0, -2.411000, 0, 0, -2.402933, 0, 0, 56.194826, 0, 0, -2.022987, 0, 0, -1.650172, 0, 0, -1.292405, 0, 0, -0.949056, 0, 0, -0.619544, 0, 0, -0.303344, 0, 0, 0, -2.376885, 0, 0, -2.369564, 0, 0, -2.388425, 0, 0, 40.560687, 0, 0, -2.027315, 0, 0, -1.644742, 0, 0, -1.283139, 0, 0, -0.940083, 0, 0, -0.613290, 0, 0, -0.300606, -2.023948, 0, 0, -2.009590, 0, 0, -2.009326, 0, 0, -2.022987, 0, 0, 56.555927, 0, 0, -1.677947, 0, 0, -1.318504, 0, 0, -0.971510, 0, 0, -0.636419, 0, 0, -0.312732, 0, 0, 0, -2.023948, 0, 0, -2.009590, 0, 0, -2.009326, 0, 0, -2.022987, 0, 0, 56.555927, 0, 0, -1.677947, 0, 0, -1.318504, 0, 0, -0.971510, 0, 0, -0.636419, 0, 0, -0.312732, 0, 0, 0, -1.962365, 0, 0, -1.962323, 0, 0, -1.983962, 0, 0, -2.027315, 0, 0, 40.901565, 0, 0, -1.702591, 0, 0, -1.332249, 0, 0, -0.979083, 0, 0, -0.640772, 0, 0, -0.315111, -1.634612, 0, 0, -1.628411, 0, 0, -1.633630, 0, 0, -1.650172, 0, 0, -1.677947, 0, 0, 56.889464, 0, 0, -1.353375, 0, 0, -1.000431, 0, 0, -0.657534, 0, 0, -0.324204, 0, 0, 0, -1.634612, 0, 0, -1.628411, 0, 0, -1.633630, 0, 0, -1.650172, 0, 0, -1.677947, 0, 0, 56.889464, 0, 0, -1.353375, 0, 0, -1.000431, 0, 0, -0.657534, 0, 0, -0.324204, 0, 0, 0, -1.577062, 0, 0, -1.582113, 0, 0, -1.604646, 0, 0, -1.644742, 0, 0, -1.702591, 0, 0, 41.215618, 0, 0, -1.395492, 0, 0, -1.028472, 0, 0, -0.675056, 0, 0, -0.332963, -1.267068, 0, 0, -1.266645, 0, 0, -1.275106, 0, 0, -1.292405, 0, 0, -1.318504, 0, 0, -1.353375, 0, 0, 57.209347, 0, 0, -1.035826, 0, 0, -0.682909, 0, 0, -0.337780, 0, 0, 0, -1.267068, 0, 0, -1.266645, 0, 0, -1.275106, 0, 0, -1.292405, 0, 0, -1.318504, 0, 0, -1.353375, 0, 0, 57.209347, 0, 0, -1.035826, 0, 0, -0.682909, 0, 0, -0.337780, 0, 0, 0, 
  -1.218233, 0, 0, -1.226282, 0, 0, -1.247881, 0, 0, -1.283139, 0, 0, -1.332249, 0, 0, -1.395492, 0, 0, 41.520880, 0, 0, -1.088554, 0, 0, -0.716365, 0, 0, -0.354282, -0.920502, 0, 0, -0.923545, 0, 0, -0.933067, 0, 0, -0.949056, 0, 0, -0.971510, 0, 0, -1.000431, 0, 0, -1.035826, 0, 0, 57.528635, 0, 0, -0.712568, 0, 0, -0.353478, 0, 0, 0, -0.920502, 0, 0, -0.923545, 0, 0, -0.933067, 0, 0, -0.949056, 0, 0, -0.971510, 0, 0, -1.000431, 0, 0, -1.035826, 0, 0, 57.528635, 0, 0, -0.712568, 0, 0, -0.353478, 0, 0, 0, -0.883349, 0, 0, -0.892361, 0, 0, -0.911235, 0, 0, -0.940083, 0, 0, -0.979083, 0, 0, -1.028472, 0, 0, -1.088554, 0, 0, 41.834417, 0, 0, -0.764959, 0, 0, -0.379209, -0.594193, 0, 0, -0.598442, 0, 0, -0.606889, 0, 0, -0.619544, 0, 0, -0.636419, 0, 0, -0.657534, 0, 0, -0.682909, 0, 0, -0.712568, 0, 0, 57.859805, 0, 0, -0.371320, 0, 0, 0, -0.594193, 0, 0, -0.598442, 0, 0, -0.606889, 0, 0, -0.619544, 0, 0, -0.636419, 0, 0, -0.657534, 0, 0, -0.682909, 0, 0, -0.712568, 0, 0, 57.859805, 0, 0, -0.371320, 0, 0, 0, -0.570076, 0, 0, -0.578060, 0, 0, -0.592432, 0, 0, -0.613290, 0, 0, -0.640772, 0, 0, -0.675056, 0, 0, -0.716365, 0, 0, -0.764959, 0, 0, 42.172966, 0, 0, -0.407905, -0.287526, 0, 0, -0.290755, 0, 0, -0.296023, 0, 0, -0.303344, 0, 0, -0.312732, 0, 0, -0.324204, 0, 0, -0.337780, 0, 0, -0.353478, 0, 0, -0.371320, 0, 0, 58.215014, 0, 0, 0, -0.287526, 0, 0, -0.290755, 0, 0, -0.296023, 0, 0, -0.303344, 0, 0, -0.312732, 0, 0, -0.324204, 0, 0, -0.337780, 0, 0, -0.353478, 0, 0, -0.371320, 0, 0, 58.215014, 0, 0, 0, -0.276278, 0, 0, -0.281259, 0, 0, -0.289348, 0, 0, -0.300606, 0, 0, -0.315111, 0, 0, -0.332963, 0, 0, -0.354282, 0, 0, -0.379209, 0, 0, -0.407905, 0, 0, 42.553556}));

CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_ONLY_COLS_NO_ROWS(C_GxB, NUM_CONSTR_STATES, N*NUM_CTRLS, float,
  ARG_DATA({0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_NO_ROWS_COLS(C_gam, NUM_CONSTR, 1, float,
  ARG_DATA({1.800000, 1.800000, 1.800000, 1.800000, 1.800000, 1.800000, 1.800000, 22.552353, 14.775680, 22.552353, 22.552353, 14.775680}));


static uint16_t active_set0[0] = {};
static uint16_t active_set1[1] = {29};
static uint16_t active_set2[2] = {35, 38};
static uint16_t active_set3[3] = {32, 35, 38};
static uint16_t active_set4[2] = {32, 35};
static uint16_t active_set5[1] = {32};
static uint16_t active_set6[4] = {49, 52, 55, 58};
static uint16_t active_set7[7] = {40, 43, 46, 49, 52, 55, 58};
static uint16_t active_set8[10] = {31, 34, 37, 40, 43, 46, 49, 52, 55, 58};
static uint16_t active_set9[3] = {31, 34, 37};
static uint16_t active_set10[9] = {31, 34, 37, 40, 43, 46, 49, 52, 55};
static uint16_t active_set11[8] = {31, 34, 37, 40, 43, 46, 49, 52};
static uint16_t active_set12[7] = {31, 34, 37, 40, 43, 46, 49};
static uint16_t active_set13[6] = {31, 34, 37, 40, 43, 46};
static uint16_t active_set14[4] = {19, 22, 25, 28};
static uint16_t active_set15[10] = {1, 4, 7, 10, 13, 16, 19, 22, 25, 28};
static uint16_t active_set16[9] = {1, 4, 7, 10, 13, 16, 19, 22, 25};
static uint16_t active_set17[8] = {1, 4, 7, 10, 13, 16, 19, 22};
static uint16_t active_set18[6] = {4, 7, 10, 13, 16, 19};
static uint16_t active_set19[6] = {1, 4, 7, 10, 13, 16};
static uint16_t active_set20[5] = {1, 4, 7, 10, 13};
static uint16_t active_set21[3] = {22, 25, 28};
static uint16_t active_set22[1] = {7};
static uint16_t active_set23[8] = {7, 10, 13, 16, 19, 22, 25, 28};
static uint16_t active_set24[2] = {1, 4};


static uint16_t successors0[3] = {0, 10, 1};
static uint16_t successors1[2] = {1, 0};
static uint16_t successors2[4] = {2, 23, 4, 0};
static uint16_t successors3[4] = {3, 24, 22, 0};
static uint16_t successors4[4] = {4, 3, 24, 0};
static uint16_t successors5[3] = {5, 3, 0};
static uint16_t successors6[9] = {6, 12, 24, 23, 22, 21, 20, 19, 0};
static uint16_t successors7[10] = {7, 12, 24, 23, 22, 21, 20, 19, 18, 0};
static uint16_t successors8[11] = {8, 12, 24, 23, 22, 21, 20, 19, 18, 17, 0};
static uint16_t successors9[3] = {9, 8, 0};
static uint16_t successors10[3] = {10, 12, 0};
static uint16_t successors11[14] = {11, 12, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 0};
static uint16_t successors12[3] = {12, 21, 0};
static uint16_t successors13[3] = {13, 9, 0};
static uint16_t successors14[2] = {14, 0};
static uint16_t successors15[2] = {15, 0};
static uint16_t successors16[4] = {16, 1, 9, 0};
static uint16_t successors17[4] = {17, 2, 23, 0};
static uint16_t successors18[3] = {18, 17, 0};
static uint16_t successors19[3] = {19, 17, 0};
static uint16_t successors20[3] = {20, 10, 0};
static uint16_t successors21[3] = {21, 12, 0};
static uint16_t successors22[4] = {22, 3, 2, 0};
static uint16_t successors23[4] = {23, 22, 17, 0};
static uint16_t successors24[4] = {24, 3, 4, 0};


void initDatabase()
{
  ctrl_db[0].id = 0;
  ctrl_db[0].active_set = active_set0;
  ctrl_db[0].active_set_size = 0;
  ctrl_db[0].successors = successors0;
  ctrl_db[0].successors_size = 3;
  ctrl_db[1].id = 1;
  ctrl_db[1].active_set = active_set1;
  ctrl_db[1].active_set_size = 1;
  ctrl_db[1].successors = successors1;
  ctrl_db[1].successors_size = 2;
  ctrl_db[2].id = 2;
  ctrl_db[2].active_set = active_set2;
  ctrl_db[2].active_set_size = 2;
  ctrl_db[2].successors = successors2;
  ctrl_db[2].successors_size = 4;
  ctrl_db[3].id = 3;
  ctrl_db[3].active_set = active_set3;
  ctrl_db[3].active_set_size = 3;
  ctrl_db[3].successors = successors3;
  ctrl_db[3].successors_size = 4;
  ctrl_db[4].id = 4;
  ctrl_db[4].active_set = active_set4;
  ctrl_db[4].active_set_size = 2;
  ctrl_db[4].successors = successors4;
  ctrl_db[4].successors_size = 4;
  ctrl_db[5].id = 5;
  ctrl_db[5].active_set = active_set5;
  ctrl_db[5].active_set_size = 1;
  ctrl_db[5].successors = successors5;
  ctrl_db[5].successors_size = 3;
  ctrl_db[6].id = 6;
  ctrl_db[6].active_set = active_set6;
  ctrl_db[6].active_set_size = 4;
  ctrl_db[6].successors = successors6;
  ctrl_db[6].successors_size = 9;
  ctrl_db[7].id = 7;
  ctrl_db[7].active_set = active_set7;
  ctrl_db[7].active_set_size = 7;
  ctrl_db[7].successors = successors7;
  ctrl_db[7].successors_size = 10;
  ctrl_db[8].id = 8;
  ctrl_db[8].active_set = active_set8;
  ctrl_db[8].active_set_size = 10;
  ctrl_db[8].successors = successors8;
  ctrl_db[8].successors_size = 11;
  ctrl_db[9].id = 9;
  ctrl_db[9].active_set = active_set9;
  ctrl_db[9].active_set_size = 3;
  ctrl_db[9].successors = successors9;
  ctrl_db[9].successors_size = 3;
  ctrl_db[10].id = 10;
  ctrl_db[10].active_set = active_set10;
  ctrl_db[10].active_set_size = 9;
  ctrl_db[10].successors = successors10;
  ctrl_db[10].successors_size = 3;
  ctrl_db[11].id = 11;
  ctrl_db[11].active_set = active_set11;
  ctrl_db[11].active_set_size = 8;
  ctrl_db[11].successors = successors11;
  ctrl_db[11].successors_size = 14;
  ctrl_db[12].id = 12;
  ctrl_db[12].active_set = active_set12;
  ctrl_db[12].active_set_size = 7;
  ctrl_db[12].successors = successors12;
  ctrl_db[12].successors_size = 3;
  ctrl_db[13].id = 13;
  ctrl_db[13].active_set = active_set13;
  ctrl_db[13].active_set_size = 6;
  ctrl_db[13].successors = successors13;
  ctrl_db[13].successors_size = 3;
  ctrl_db[14].id = 14;
  ctrl_db[14].active_set = active_set14;
  ctrl_db[14].active_set_size = 4;
  ctrl_db[14].successors = successors14;
  ctrl_db[14].successors_size = 2;
  ctrl_db[15].id = 15;
  ctrl_db[15].active_set = active_set15;
  ctrl_db[15].active_set_size = 10;
  ctrl_db[15].successors = successors15;
  ctrl_db[15].successors_size = 2;
  ctrl_db[16].id = 16;
  ctrl_db[16].active_set = active_set16;
  ctrl_db[16].active_set_size = 9;
  ctrl_db[16].successors = successors16;
  ctrl_db[16].successors_size = 4;
  ctrl_db[17].id = 17;
  ctrl_db[17].active_set = active_set17;
  ctrl_db[17].active_set_size = 8;
  ctrl_db[17].successors = successors17;
  ctrl_db[17].successors_size = 4;
  ctrl_db[18].id = 18;
  ctrl_db[18].active_set = active_set18;
  ctrl_db[18].active_set_size = 6;
  ctrl_db[18].successors = successors18;
  ctrl_db[18].successors_size = 3;
  ctrl_db[19].id = 19;
  ctrl_db[19].active_set = active_set19;
  ctrl_db[19].active_set_size = 6;
  ctrl_db[19].successors = successors19;
  ctrl_db[19].successors_size = 3;
  ctrl_db[20].id = 20;
  ctrl_db[20].active_set = active_set20;
  ctrl_db[20].active_set_size = 5;
  ctrl_db[20].successors = successors20;
  ctrl_db[20].successors_size = 3;
  ctrl_db[21].id = 21;
  ctrl_db[21].active_set = active_set21;
  ctrl_db[21].active_set_size = 3;
  ctrl_db[21].successors = successors21;
  ctrl_db[21].successors_size = 3;
  ctrl_db[22].id = 22;
  ctrl_db[22].active_set = active_set22;
  ctrl_db[22].active_set_size = 1;
  ctrl_db[22].successors = successors22;
  ctrl_db[22].successors_size = 4;
  ctrl_db[23].id = 23;
  ctrl_db[23].active_set = active_set23;
  ctrl_db[23].active_set_size = 8;
  ctrl_db[23].successors = successors23;
  ctrl_db[23].successors_size = 4;
  ctrl_db[24].id = 24;
  ctrl_db[24].active_set = active_set24;
  ctrl_db[24].active_set_size = 2;
  ctrl_db[24].successors = successors24;
  ctrl_db[24].successors_size = 4;
}

