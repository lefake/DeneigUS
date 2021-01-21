#ifndef CONSTANTS_H
#define CONSTANTS_H

#define NBS_SONARS                    8
#define NBS_BATTERIES                 2
#define NBS_MOTORS                    5

#define NBS_DATA_POS                  3
#define NBS_DATA_SONRARS              2
#define NBS_DATA_ESTOP                1
#define NBS_DATA_BATTERIES            4
#define NBS_DATA_TOURELLES            5
#define NBS_DATA_MOTORS               5

#define POS_MSG_ARRAY_LEN             NBS_DATA_POS
#define OBS_POS_MSG_ARRAY_LEN         NBS_DATA_SONRARS * NBS_SONARS
#define ESTOP_STATE_MSG_ARRAY_LEN     NBS_DATA_ESTOP
#define TELE_BATT_MSG_ARRAY_LEN       NBS_DATA_BATTERIES * NBS_BATTERIES
#define POS_TOURELLE_MSG_ARRAY_LEN    NBS_DATA_TOURELLES
#define DEBUG_MOT_MSG_ARRAY_LEN       NBS_DATA_MOTORS * NBS_MOTORS

#endif
