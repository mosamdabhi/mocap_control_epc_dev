/*******************************/
/* Auto-generated. Do not edit */
/*******************************/
// c_epc_autogen - 1

#pragma once
void initDatabase();

enum
{
  N = 10,
  NUM_STATES = 6,
  NUM_CTRLS = 3,
  NUM_CONSTR_STATES = 3,
  NUM_CONSTR = 2*(NUM_CONSTR_STATES+NUM_CTRLS),
  DB_SIZE = 63,
  MAX_SUCCESSORS = 20
};

static struct Element ctrl_db[DB_SIZE];


/*Auto-generated values from c_epc_autogen.h*/
/*Some matrices like these are generated from this part*/




CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_NO_ROWS_COLS(C_affine_c, NUM_STATES, 1, float,
  ARG_DATA({0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000}));


CONSTRUCT_STATIC_MAT_WITH_DATA(C_BtQrow, NUM_CTRLS, N*NUM_STATES, float,
  ARG_DATA({0.001319, 0, 0, 0.008843, 0, 0, 0.003957, 0, 0, 0.008843, 0, 0, 0.006595, 0, 0, 0.008843, 0, 0, 0.009233, 0, 0, 0.008843, 0, 0, 0.011871, 0, 0, 0.008843, 0, 0, 0.014509, 0, 0, 0.008843, 0, 0, 0.017147, 0, 0, 0.008843, 0, 0, 0.019785, 0, 0, 0.008843, 0, 0, 0.022423, 0, 0, 0.008843, 0, 0, 0.025062, 0, 0, 0.008843, 0, 0, 0, 0.001319, 0, 0, 0.008843, 0, 0, 0.003957, 0, 0, 0.008843, 0, 0, 0.006595, 0, 0, 0.008843, 0, 0, 0.009233, 0, 0, 0.008843, 0, 0, 0.011871, 0, 0, 0.008843, 0, 0, 0.014509, 0, 0, 0.008843, 0, 0, 0.017147, 0, 0, 0.008843, 0, 0, 0.019785, 0, 0, 0.008843, 0, 0, 0.022423, 0, 0, 0.008843, 0, 0, 0.025062, 0, 0, 0.008843, 0, 0, 0, 0.002396, 0, 0, 0.022690, 0, 0, 0.007187, 0, 0, 0.022690, 0, 0, 0.011978, 0, 0, 0.022690, 0, 0, 0.016769, 0, 0, 0.022690, 0, 0, 0.021560, 0, 0, 0.022690, 0, 0, 0.026351, 0, 0, 0.022690, 0, 0, 0.031142, 0, 0, 0.022690, 0, 0, 0.035933, 0, 0, 0.022690, 0, 0, 0.040724, 0, 0, 0.022690, 0, 0, 0.045515, 0, 0, 0.022690}));

CONSTRUCT_STATIC_MAT_WITH_DATA(C_Hinv, N*NUM_CTRLS, N*NUM_CTRLS, float,
  ARG_DATA({25.768344, 0, 0, -0.684018, 0, 0, -0.599352, 0, 0, -0.516908, 0, 0, -0.436658, 0, 0, -0.358576, 0, 0, -0.282639, 0, 0, -0.208826, 0, 0, -0.137123, 0, 0, -0.067516, 0, 0, 0, 25.768344, 0, 0, -0.684018, 0, 0, -0.599352, 0, 0, -0.516908, 0, 0, -0.436658, 0, 0, -0.358576, 0, 0, -0.282639, 0, 0, -0.208826, 0, 0, -0.137123, 0, 0, -0.067516, 0, 0, 0, 19.805548, 0, 0, -0.953562, 0, 0, -0.830016, 0, 0, -0.711916, 0, 0, -0.598784, 0, 0, -0.490163, 0, 0, -0.385615, 0, 0, -0.284719, 0, 0, -0.187074, 0, 0, -0.092292, -0.684018, 0, 0, 25.861764, 0, 0, -0.594521, 0, 0, -0.513503, 0, 0, -0.434436, 0, 0, -0.357301, 0, 0, -0.282076, 0, 0, -0.208746, 0, 0, -0.137295, 0, 0, -0.067715, 0, 0, 0, -0.684018, 0, 0, 25.861764, 0, 0, -0.594521, 0, 0, -0.513503, 0, 0, -0.434436, 0, 0, -0.357301, 0, 0, -0.282076, 0, 0, -0.208746, 0, 0, -0.137295, 0, 0, -0.067715, 0, 0, 0, -0.953562, 0, 0, 19.938104, 0, 0, -0.828318, 0, 0, -0.711315, 0, 0, -0.599017, 0, 0, -0.490975, 0, 0, -0.386755, 0, 0, -0.285941, 0, 0, -0.188132, 0, 0, -0.092943, -0.599352, 0, 0, -0.594521, 0, 0, 25.947835, 0, 0, -0.511611, 0, 0, -0.433496, 0, 0, -0.357079, 0, 0, -0.282345, 0, 0, -0.209280, 0, 0, -0.137873, 0, 0, -0.068114, 0, 0, 0, -0.599352, 0, 0, -0.594521, 0, 0, 25.947835, 0, 0, -0.511611, 0, 0, -0.433496, 0, 0, -0.357079, 0, 0, -0.282345, 0, 0, -0.209280, 0, 0, -0.137873, 0, 0, -0.068114, 0, 0, 0, -0.830016, 0, 0, -0.828318, 0, 0, 20.057043, 0, 0, -0.714954, 0, 0, -0.602821, 0, 0, -0.494713, 0, 0, -0.390200, 0, 0, -0.288867, 0, 0, -0.190312, 0, 0, -0.094149, -0.516908, 0, 0, -0.513503, 0, 0, -0.511611, 0, 0, 26.028059, 0, 0, -0.433826, 0, 0, -0.357904, 0, 0, -0.283442, 0, 0, -0.210428, 0, 0, -0.138854, 0, 0, -0.068713, 0, 0, 0, 
  -0.516908, 0, 0, -0.513503, 0, 0, -0.511611, 0, 0, 26.028059, 0, 0, -0.433826, 0, 0, -0.357904, 0, 0, -0.283442, 0, 0, -0.210428, 0, 0, -0.138854, 0, 0, -0.068713, 0, 0, 0, -0.711916, 0, 0, -0.711315, 0, 0, -0.714954, 0, 0, 20.165764, 0, 0, -0.610203, 0, 0, -0.501388, 0, 0, -0.395962, 0, 0, -0.293508, 0, 0, -0.193622, 0, 0, -0.095913, -0.436658, 0, 0, -0.434436, 0, 0, -0.433496, 0, 0, -0.433826, 0, 0, 26.103863, 0, 0, -0.359768, 0, 0, -0.285361, 0, 0, -0.212187, 0, 0, -0.140239, 0, 0, -0.069511, 0, 0, 0, -0.436658, 0, 0, -0.434436, 0, 0, -0.433496, 0, 0, -0.433826, 0, 0, 26.103863, 0, 0, -0.359768, 0, 0, -0.285361, 0, 0, -0.212187, 0, 0, -0.140239, 0, 0, -0.069511, 0, 0, 0, -0.598784, 0, 0, -0.599017, 0, 0, -0.602821, 0, 0, -0.610203, 0, 0, 20.267416, 0, 0, -0.511021, 0, 0, -0.404060, 0, 0, -0.299880, 0, 0, -0.198074, 0, 0, -0.098244, -0.358576, 0, 0, -0.357301, 0, 0, -0.357079, 0, 0, -0.357904, 0, 0, -0.359768, 0, 0, 26.176613, 0, 0, -0.288100, 0, 0, -0.214556, 0, 0, -0.142026, 0, 0, -0.070508, 0, 0, 0, -0.358576, 0, 0, -0.357301, 0, 0, -0.357079, 0, 0, -0.357904, 0, 0, -0.359768, 0, 0, 26.176613, 0, 0, -0.288100, 0, 0, -0.214556, 0, 0, -0.142026, 0, 0, -0.070508, 0, 0, 0, -0.490163, 0, 0, -0.490975, 0, 0, -0.494713, 0, 0, -0.501388, 0, 0, -0.511021, 0, 0, 20.364956, 0, 0, -0.414522, 0, 0, -0.308008, 0, 0, -0.203686, 0, 0, -0.101149, -0.282639, 0, 0, -0.282076, 0, 0, -0.282345, 0, 0, -0.283442, 0, 0, -0.285361, 0, 0, -0.288100, 0, 0, 26.247624, 0, 0, -0.217531, 0, 0, -0.144215, 0, 0, -0.071705, 0, 0, 0, -0.282639, 0, 0, -0.282076, 0, 0, -0.282345, 0, 0, -0.283442, 0, 0, -0.285361, 0, 0, -0.288100, 0, 0, 26.247624, 0, 0, -0.217531, 0, 0, -0.144215, 0, 0, -0.071705, 0, 0, 0, 
  -0.385615, 0, 0, -0.386755, 0, 0, -0.390200, 0, 0, -0.395962, 0, 0, -0.404060, 0, 0, -0.414522, 0, 0, 20.461214, 0, 0, -0.317921, 0, 0, -0.210478, 0, 0, -0.104641, -0.208826, 0, 0, -0.208746, 0, 0, -0.209280, 0, 0, -0.210428, 0, 0, -0.212187, 0, 0, -0.214556, 0, 0, -0.217531, 0, 0, 26.318168, 0, 0, -0.146805, 0, 0, -0.073102, 0, 0, 0, -0.208826, 0, 0, -0.208746, 0, 0, -0.209280, 0, 0, -0.210428, 0, 0, -0.212187, 0, 0, -0.214556, 0, 0, -0.217531, 0, 0, 26.318168, 0, 0, -0.146805, 0, 0, -0.073102, 0, 0, 0, -0.284719, 0, 0, -0.285941, 0, 0, -0.288867, 0, 0, -0.293508, 0, 0, -0.299880, 0, 0, -0.308008, 0, 0, -0.317921, 0, 0, 20.558945, 0, 0, -0.218477, 0, 0, -0.108733, -0.137123, 0, 0, -0.137295, 0, 0, -0.137873, 0, 0, -0.138854, 0, 0, -0.140239, 0, 0, -0.142026, 0, 0, -0.144215, 0, 0, -0.146805, 0, 0, 26.389483, 0, 0, -0.074698, 0, 0, 0, -0.137123, 0, 0, -0.137295, 0, 0, -0.137873, 0, 0, -0.138854, 0, 0, -0.140239, 0, 0, -0.142026, 0, 0, -0.144215, 0, 0, -0.146805, 0, 0, 26.389483, 0, 0, -0.074698, 0, 0, 0, -0.187074, 0, 0, -0.188132, 0, 0, -0.190312, 0, 0, -0.193622, 0, 0, -0.198074, 0, 0, -0.203686, 0, 0, -0.210478, 0, 0, -0.218477, 0, 0, 20.660888, 0, 0, -0.113441, -0.067516, 0, 0, -0.067715, 0, 0, -0.068114, 0, 0, -0.068713, 0, 0, -0.069511, 0, 0, -0.070508, 0, 0, -0.071705, 0, 0, -0.073102, 0, 0, -0.074698, 0, 0, 26.462785, 0, 0, 0, -0.067516, 0, 0, -0.067715, 0, 0, -0.068114, 0, 0, -0.068713, 0, 0, -0.069511, 0, 0, -0.070508, 0, 0, -0.071705, 0, 0, -0.073102, 0, 0, -0.074698, 0, 0, 26.462785, 0, 0, 0, -0.092292, 0, 0, -0.092943, 0, 0, -0.094149, 0, 0, -0.095913, 0, 0, -0.098244, 0, 0, -0.101149, 0, 0, -0.104641, 0, 0, -0.108733, 0, 0, -0.113441, 0, 0, 20.769817}));

CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_ONLY_COLS_NO_ROWS(C_GxB, NUM_CONSTR_STATES, N*NUM_CTRLS, float,
  ARG_DATA({0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0.012610, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

CONSTRUCT_STATIC_MAT_WITH_DATA_FOR_NO_ROWS_COLS(C_gam, NUM_CONSTR, 1, float,
  ARG_DATA({1.500000, 1.620000, 1.500000, 1.500000, 1.620000, 1.500000, 1.500000, 22.552353, 14.775680, 22.552353, 22.552353, 14.775680}));


static uint16_t active_set0[0] = {};
static uint16_t active_set1[1] = {29};
static uint16_t active_set2[2] = {35, 38};
static uint16_t active_set3[3] = {32, 35, 38};
static uint16_t active_set4[1] = {35};
static uint16_t active_set5[2] = {32, 35};
static uint16_t active_set6[1] = {32};
static uint16_t active_set7[4] = {48, 51, 54, 57};
static uint16_t active_set8[10] = {30, 33, 36, 39, 42, 45, 48, 51, 54, 57};
static uint16_t active_set9[9] = {30, 33, 36, 39, 42, 45, 48, 51, 54};
static uint16_t active_set10[8] = {30, 33, 36, 39, 42, 45, 48, 51};
static uint16_t active_set11[7] = {30, 33, 36, 39, 42, 45, 48};
static uint16_t active_set12[6] = {30, 33, 36, 39, 42, 45};
static uint16_t active_set13[5] = {30, 33, 36, 39, 42};
static uint16_t active_set14[4] = {30, 33, 36, 39};
static uint16_t active_set15[1] = {38};
static uint16_t active_set16[2] = {54, 57};
static uint16_t active_set17[8] = {36, 39, 42, 45, 48, 51, 54, 57};
static uint16_t active_set18[4] = {39, 42, 45, 48};
static uint16_t active_set19[3] = {35, 38, 41};
static uint16_t active_set20[4] = {32, 35, 38, 41};
static uint16_t active_set21[2] = {25, 28};
static uint16_t active_set22[7] = {10, 13, 16, 19, 22, 25, 28};
static uint16_t active_set23[10] = {1, 4, 7, 10, 13, 16, 19, 22, 25, 28};
static uint16_t active_set24[9] = {1, 4, 7, 10, 13, 16, 19, 22, 25};
static uint16_t active_set25[8] = {1, 4, 7, 10, 13, 16, 19, 22};
static uint16_t active_set26[7] = {1, 4, 7, 10, 13, 16, 19};
static uint16_t active_set27[6] = {1, 4, 7, 10, 13, 16};
static uint16_t active_set28[5] = {1, 4, 7, 10, 13};
static uint16_t active_set29[4] = {1, 4, 7, 10};
static uint16_t active_set30[5] = {46, 49, 52, 55, 58};
static uint16_t active_set31[5] = {43, 46, 49, 52, 55};
static uint16_t active_set32[8] = {31, 34, 37, 40, 43, 46, 49, 52};
static uint16_t active_set33[7] = {31, 34, 37, 40, 43, 46, 49};
static uint16_t active_set34[6] = {31, 34, 37, 40, 43, 46};
static uint16_t active_set35[3] = {37, 40, 43};
static uint16_t active_set36[3] = {34, 37, 40};
static uint16_t active_set37[3] = {52, 55, 58};
static uint16_t active_set38[4] = {16, 19, 22, 25};
static uint16_t active_set39[6] = {7, 10, 13, 16, 19, 22};
static uint16_t active_set40[3] = {7, 10, 13};
static uint16_t active_set41[3] = {4, 7, 10};
static uint16_t active_set42[3] = {1, 4, 7};
static uint16_t active_set43[4] = {40, 43, 46, 49};
static uint16_t active_set44[2] = {24, 27};
static uint16_t active_set45[6] = {12, 15, 18, 21, 24, 27};
static uint16_t active_set46[10] = {0, 3, 6, 9, 12, 15, 18, 21, 24, 27};
static uint16_t active_set47[9] = {0, 3, 6, 9, 12, 15, 18, 21, 24};
static uint16_t active_set48[8] = {0, 3, 6, 9, 12, 15, 18, 21};
static uint16_t active_set49[7] = {0, 3, 6, 9, 12, 15, 18};
static uint16_t active_set50[6] = {0, 3, 6, 9, 12, 15};
static uint16_t active_set51[5] = {0, 3, 6, 9, 12};
static uint16_t active_set52[4] = {0, 3, 6, 9};
static uint16_t active_set53[7] = {6, 9, 12, 15, 18, 21, 24};
static uint16_t active_set54[7] = {15, 18, 21, 24, 27, 35, 38};
static uint16_t active_set55[8] = {9, 12, 15, 18, 21, 24, 27, 35};
static uint16_t active_set56[11] = {3, 6, 9, 12, 15, 18, 21, 24, 27, 32, 35};
static uint16_t active_set57[11] = {0, 3, 6, 9, 12, 15, 18, 21, 24, 32, 35};
static uint16_t active_set58[10] = {0, 3, 6, 9, 12, 15, 18, 21, 32, 35};
static uint16_t active_set59[4] = {9, 12, 15, 18};
static uint16_t active_set60[4] = {6, 9, 12, 15};
static uint16_t active_set61[4] = {3, 6, 9, 12};
static uint16_t active_set62[2] = {30, 33};


static uint16_t successors0[4] = {0, 1, 6, 11};
static uint16_t successors1[2] = {1, 0};
static uint16_t successors2[4] = {2, 15, 18, 0};
static uint16_t successors3[3] = {3, 12, 0};
static uint16_t successors4[3] = {4, 11, 0};
static uint16_t successors5[4] = {5, 6, 18, 0};
static uint16_t successors6[3] = {6, 14, 0};
static uint16_t successors7[4] = {7, 4, 15, 0};
static uint16_t successors8[4] = {8, 43, 36, 0};
static uint16_t successors9[3] = {9, 16, 0};
static uint16_t successors10[4] = {10, 13, 11, 0};
static uint16_t successors11[3] = {11, 4, 0};
static uint16_t successors12[3] = {12, 43, 0};
static uint16_t successors13[4] = {13, 10, 21, 0};
static uint16_t successors14[3] = {14, 6, 0};
static uint16_t successors15[3] = {15, 7, 0};
static uint16_t successors16[4] = {16, 1, 4, 0};
static uint16_t successors17[5] = {17, 16, 6, 13, 0};
static uint16_t successors18[4] = {18, 35, 6, 0};
static uint16_t successors19[3] = {19, 12, 0};
static uint16_t successors20[3] = {20, 19, 0};
static uint16_t successors21[4] = {21, 13, 4, 0};
static uint16_t successors22[5] = {22, 23, 20, 2, 0};
static uint16_t successors23[4] = {23, 22, 32, 0};
static uint16_t successors24[4] = {24, 23, 44, 0};
static uint16_t successors25[3] = {25, 24, 0};
static uint16_t successors26[4] = {26, 4, 25, 0};
static uint16_t successors27[4] = {27, 6, 26, 0};
static uint16_t successors28[4] = {28, 8, 7, 0};
static uint16_t successors29[5] = {29, 48, 16, 28, 0};
static uint16_t successors30[4] = {30, 60, 4, 0};
static uint16_t successors31[3] = {31, 29, 0};
static uint16_t successors32[5] = {32, 8, 60, 44, 0};
static uint16_t successors33[5] = {33, 36, 35, 28, 0};
static uint16_t successors34[3] = {34, 5, 0};
static uint16_t successors35[3] = {35, 18, 0};
static uint16_t successors36[4] = {36, 44, 3, 0};
static uint16_t successors37[3] = {37, 10, 0};
static uint16_t successors38[3] = {38, 37, 0};
static uint16_t successors39[4] = {39, 8, 10, 0};
static uint16_t successors40[4] = {40, 53, 10, 0};
static uint16_t successors41[4] = {41, 3, 10, 0};
static uint16_t successors42[3] = {42, 41, 0};
static uint16_t successors43[3] = {43, 12, 0};
static uint16_t successors44[3] = {44, 58, 0};
static uint16_t successors45[3] = {45, 9, 0};
static uint16_t successors46[4] = {46, 44, 9, 0};
static uint16_t successors47[3] = {47, 31, 0};
static uint16_t successors48[4] = {48, 49, 47, 0};
static uint16_t successors49[3] = {49, 44, 0};
static uint16_t successors50[4] = {50, 31, 48, 0};
static uint16_t successors51[3] = {51, 49, 0};
static uint16_t successors52[3] = {52, 49, 0};
static uint16_t successors53[3] = {53, 29, 0};
static uint16_t successors54[3] = {54, 29, 0};
static uint16_t successors55[3] = {55, 37, 0};
static uint16_t successors56[3] = {56, 55, 0};
static uint16_t successors57[4] = {57, 49, 44, 0};
static uint16_t successors58[3] = {58, 44, 0};
static uint16_t successors59[5] = {59, 32, 33, 30, 0};
static uint16_t successors60[4] = {60, 32, 44, 0};
static uint16_t successors61[3] = {61, 57, 0};
static uint16_t successors62[3] = {62, 57, 0};




void initDatabase()
{
  ctrl_db[0].id = 0;
  ctrl_db[0].active_set = active_set0;
  ctrl_db[0].active_set_size = 0;
  ctrl_db[0].successors = successors0;
  ctrl_db[0].successors_size = 4;
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
  ctrl_db[3].successors_size = 3;
  ctrl_db[4].id = 4;
  ctrl_db[4].active_set = active_set4;
  ctrl_db[4].active_set_size = 1;
  ctrl_db[4].successors = successors4;
  ctrl_db[4].successors_size = 3;
  ctrl_db[5].id = 5;
  ctrl_db[5].active_set = active_set5;
  ctrl_db[5].active_set_size = 2;
  ctrl_db[5].successors = successors5;
  ctrl_db[5].successors_size = 4;
  ctrl_db[6].id = 6;
  ctrl_db[6].active_set = active_set6;
  ctrl_db[6].active_set_size = 1;
  ctrl_db[6].successors = successors6;
  ctrl_db[6].successors_size = 3;
  ctrl_db[7].id = 7;
  ctrl_db[7].active_set = active_set7;
  ctrl_db[7].active_set_size = 4;
  ctrl_db[7].successors = successors7;
  ctrl_db[7].successors_size = 4;
  ctrl_db[8].id = 8;
  ctrl_db[8].active_set = active_set8;
  ctrl_db[8].active_set_size = 10;
  ctrl_db[8].successors = successors8;
  ctrl_db[8].successors_size = 4;
  ctrl_db[9].id = 9;
  ctrl_db[9].active_set = active_set9;
  ctrl_db[9].active_set_size = 9;
  ctrl_db[9].successors = successors9;
  ctrl_db[9].successors_size = 3;
  ctrl_db[10].id = 10;
  ctrl_db[10].active_set = active_set10;
  ctrl_db[10].active_set_size = 8;
  ctrl_db[10].successors = successors10;
  ctrl_db[10].successors_size = 4;
  ctrl_db[11].id = 11;
  ctrl_db[11].active_set = active_set11;
  ctrl_db[11].active_set_size = 7;
  ctrl_db[11].successors = successors11;
  ctrl_db[11].successors_size = 3;
  ctrl_db[12].id = 12;
  ctrl_db[12].active_set = active_set12;
  ctrl_db[12].active_set_size = 6;
  ctrl_db[12].successors = successors12;
  ctrl_db[12].successors_size = 3;
  ctrl_db[13].id = 13;
  ctrl_db[13].active_set = active_set13;
  ctrl_db[13].active_set_size = 5;
  ctrl_db[13].successors = successors13;
  ctrl_db[13].successors_size = 4;
  ctrl_db[14].id = 14;
  ctrl_db[14].active_set = active_set14;
  ctrl_db[14].active_set_size = 4;
  ctrl_db[14].successors = successors14;
  ctrl_db[14].successors_size = 3;
  ctrl_db[15].id = 15;
  ctrl_db[15].active_set = active_set15;
  ctrl_db[15].active_set_size = 1;
  ctrl_db[15].successors = successors15;
  ctrl_db[15].successors_size = 3;
  ctrl_db[16].id = 16;
  ctrl_db[16].active_set = active_set16;
  ctrl_db[16].active_set_size = 2;
  ctrl_db[16].successors = successors16;
  ctrl_db[16].successors_size = 4;
  ctrl_db[17].id = 17;
  ctrl_db[17].active_set = active_set17;
  ctrl_db[17].active_set_size = 8;
  ctrl_db[17].successors = successors17;
  ctrl_db[17].successors_size = 5;
  ctrl_db[18].id = 18;
  ctrl_db[18].active_set = active_set18;
  ctrl_db[18].active_set_size = 4;
  ctrl_db[18].successors = successors18;
  ctrl_db[18].successors_size = 4;
  ctrl_db[19].id = 19;
  ctrl_db[19].active_set = active_set19;
  ctrl_db[19].active_set_size = 3;
  ctrl_db[19].successors = successors19;
  ctrl_db[19].successors_size = 3;
  ctrl_db[20].id = 20;
  ctrl_db[20].active_set = active_set20;
  ctrl_db[20].active_set_size = 4;
  ctrl_db[20].successors = successors20;
  ctrl_db[20].successors_size = 3;
  ctrl_db[21].id = 21;
  ctrl_db[21].active_set = active_set21;
  ctrl_db[21].active_set_size = 2;
  ctrl_db[21].successors = successors21;
  ctrl_db[21].successors_size = 4;
  ctrl_db[22].id = 22;
  ctrl_db[22].active_set = active_set22;
  ctrl_db[22].active_set_size = 7;
  ctrl_db[22].successors = successors22;
  ctrl_db[22].successors_size = 5;
  ctrl_db[23].id = 23;
  ctrl_db[23].active_set = active_set23;
  ctrl_db[23].active_set_size = 10;
  ctrl_db[23].successors = successors23;
  ctrl_db[23].successors_size = 4;
  ctrl_db[24].id = 24;
  ctrl_db[24].active_set = active_set24;
  ctrl_db[24].active_set_size = 9;
  ctrl_db[24].successors = successors24;
  ctrl_db[24].successors_size = 4;
  ctrl_db[25].id = 25;
  ctrl_db[25].active_set = active_set25;
  ctrl_db[25].active_set_size = 8;
  ctrl_db[25].successors = successors25;
  ctrl_db[25].successors_size = 3;
  ctrl_db[26].id = 26;
  ctrl_db[26].active_set = active_set26;
  ctrl_db[26].active_set_size = 7;
  ctrl_db[26].successors = successors26;
  ctrl_db[26].successors_size = 4;
  ctrl_db[27].id = 27;
  ctrl_db[27].active_set = active_set27;
  ctrl_db[27].active_set_size = 6;
  ctrl_db[27].successors = successors27;
  ctrl_db[27].successors_size = 4;
  ctrl_db[28].id = 28;
  ctrl_db[28].active_set = active_set28;
  ctrl_db[28].active_set_size = 5;
  ctrl_db[28].successors = successors28;
  ctrl_db[28].successors_size = 4;
  ctrl_db[29].id = 29;
  ctrl_db[29].active_set = active_set29;
  ctrl_db[29].active_set_size = 4;
  ctrl_db[29].successors = successors29;
  ctrl_db[29].successors_size = 5;
  ctrl_db[30].id = 30;
  ctrl_db[30].active_set = active_set30;
  ctrl_db[30].active_set_size = 5;
  ctrl_db[30].successors = successors30;
  ctrl_db[30].successors_size = 4;
  ctrl_db[31].id = 31;
  ctrl_db[31].active_set = active_set31;
  ctrl_db[31].active_set_size = 5;
  ctrl_db[31].successors = successors31;
  ctrl_db[31].successors_size = 3;
  ctrl_db[32].id = 32;
  ctrl_db[32].active_set = active_set32;
  ctrl_db[32].active_set_size = 8;
  ctrl_db[32].successors = successors32;
  ctrl_db[32].successors_size = 5;
  ctrl_db[33].id = 33;
  ctrl_db[33].active_set = active_set33;
  ctrl_db[33].active_set_size = 7;
  ctrl_db[33].successors = successors33;
  ctrl_db[33].successors_size = 5;
  ctrl_db[34].id = 34;
  ctrl_db[34].active_set = active_set34;
  ctrl_db[34].active_set_size = 6;
  ctrl_db[34].successors = successors34;
  ctrl_db[34].successors_size = 3;
  ctrl_db[35].id = 35;
  ctrl_db[35].active_set = active_set35;
  ctrl_db[35].active_set_size = 3;
  ctrl_db[35].successors = successors35;
  ctrl_db[35].successors_size = 3;
  ctrl_db[36].id = 36;
  ctrl_db[36].active_set = active_set36;
  ctrl_db[36].active_set_size = 3;
  ctrl_db[36].successors = successors36;
  ctrl_db[36].successors_size = 4;
  ctrl_db[37].id = 37;
  ctrl_db[37].active_set = active_set37;
  ctrl_db[37].active_set_size = 3;
  ctrl_db[37].successors = successors37;
  ctrl_db[37].successors_size = 3;
  ctrl_db[38].id = 38;
  ctrl_db[38].active_set = active_set38;
  ctrl_db[38].active_set_size = 4;
  ctrl_db[38].successors = successors38;
  ctrl_db[38].successors_size = 3;
  ctrl_db[39].id = 39;
  ctrl_db[39].active_set = active_set39;
  ctrl_db[39].active_set_size = 6;
  ctrl_db[39].successors = successors39;
  ctrl_db[39].successors_size = 4;
  ctrl_db[40].id = 40;
  ctrl_db[40].active_set = active_set40;
  ctrl_db[40].active_set_size = 3;
  ctrl_db[40].successors = successors40;
  ctrl_db[40].successors_size = 4;
  ctrl_db[41].id = 41;
  ctrl_db[41].active_set = active_set41;
  ctrl_db[41].active_set_size = 3;
  ctrl_db[41].successors = successors41;
  ctrl_db[41].successors_size = 4;
  ctrl_db[42].id = 42;
  ctrl_db[42].active_set = active_set42;
  ctrl_db[42].active_set_size = 3;
  ctrl_db[42].successors = successors42;
  ctrl_db[42].successors_size = 3;
  ctrl_db[43].id = 43;
  ctrl_db[43].active_set = active_set43;
  ctrl_db[43].active_set_size = 4;
  ctrl_db[43].successors = successors43;
  ctrl_db[43].successors_size = 3;
  ctrl_db[44].id = 44;
  ctrl_db[44].active_set = active_set44;
  ctrl_db[44].active_set_size = 2;
  ctrl_db[44].successors = successors44;
  ctrl_db[44].successors_size = 3;
  ctrl_db[45].id = 45;
  ctrl_db[45].active_set = active_set45;
  ctrl_db[45].active_set_size = 6;
  ctrl_db[45].successors = successors45;
  ctrl_db[45].successors_size = 3;
  ctrl_db[46].id = 46;
  ctrl_db[46].active_set = active_set46;
  ctrl_db[46].active_set_size = 10;
  ctrl_db[46].successors = successors46;
  ctrl_db[46].successors_size = 4;
  ctrl_db[47].id = 47;
  ctrl_db[47].active_set = active_set47;
  ctrl_db[47].active_set_size = 9;
  ctrl_db[47].successors = successors47;
  ctrl_db[47].successors_size = 3;
  ctrl_db[48].id = 48;
  ctrl_db[48].active_set = active_set48;
  ctrl_db[48].active_set_size = 8;
  ctrl_db[48].successors = successors48;
  ctrl_db[48].successors_size = 4;
  ctrl_db[49].id = 49;
  ctrl_db[49].active_set = active_set49;
  ctrl_db[49].active_set_size = 7;
  ctrl_db[49].successors = successors49;
  ctrl_db[49].successors_size = 3;
  ctrl_db[50].id = 50;
  ctrl_db[50].active_set = active_set50;
  ctrl_db[50].active_set_size = 6;
  ctrl_db[50].successors = successors50;
  ctrl_db[50].successors_size = 4;
  ctrl_db[51].id = 51;
  ctrl_db[51].active_set = active_set51;
  ctrl_db[51].active_set_size = 5;
  ctrl_db[51].successors = successors51;
  ctrl_db[51].successors_size = 3;
  ctrl_db[52].id = 52;
  ctrl_db[52].active_set = active_set52;
  ctrl_db[52].active_set_size = 4;
  ctrl_db[52].successors = successors52;
  ctrl_db[52].successors_size = 3;
  ctrl_db[53].id = 53;
  ctrl_db[53].active_set = active_set53;
  ctrl_db[53].active_set_size = 7;
  ctrl_db[53].successors = successors53;
  ctrl_db[53].successors_size = 3;
  ctrl_db[54].id = 54;
  ctrl_db[54].active_set = active_set54;
  ctrl_db[54].active_set_size = 7;
  ctrl_db[54].successors = successors54;
  ctrl_db[54].successors_size = 3;
  ctrl_db[55].id = 55;
  ctrl_db[55].active_set = active_set55;
  ctrl_db[55].active_set_size = 8;
  ctrl_db[55].successors = successors55;
  ctrl_db[55].successors_size = 3;
  ctrl_db[56].id = 56;
  ctrl_db[56].active_set = active_set56;
  ctrl_db[56].active_set_size = 11;
  ctrl_db[56].successors = successors56;
  ctrl_db[56].successors_size = 3;
  ctrl_db[57].id = 57;
  ctrl_db[57].active_set = active_set57;
  ctrl_db[57].active_set_size = 11;
  ctrl_db[57].successors = successors57;
  ctrl_db[57].successors_size = 4;
  ctrl_db[58].id = 58;
  ctrl_db[58].active_set = active_set58;
  ctrl_db[58].active_set_size = 10;
  ctrl_db[58].successors = successors58;
  ctrl_db[58].successors_size = 3;
  ctrl_db[59].id = 59;
  ctrl_db[59].active_set = active_set59;
  ctrl_db[59].active_set_size = 4;
  ctrl_db[59].successors = successors59;
  ctrl_db[59].successors_size = 5;
  ctrl_db[60].id = 60;
  ctrl_db[60].active_set = active_set60;
  ctrl_db[60].active_set_size = 4;
  ctrl_db[60].successors = successors60;
  ctrl_db[60].successors_size = 4;
  ctrl_db[61].id = 61;
  ctrl_db[61].active_set = active_set61;
  ctrl_db[61].active_set_size = 4;
  ctrl_db[61].successors = successors61;
  ctrl_db[61].successors_size = 3;
  ctrl_db[62].id = 62;
  ctrl_db[62].active_set = active_set62;
  ctrl_db[62].active_set_size = 2;
  ctrl_db[62].successors = successors62;
  ctrl_db[62].successors_size = 3;
}

