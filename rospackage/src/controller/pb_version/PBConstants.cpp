#include "PBConstants.h"

const pb_msgdesc_t* MSGS_TYPE[NBS_MSG_TYPES] = 
{ 
  Twist_fields,
  FloatArray_fields 
};

const int TOPIC_2_MSG_TYPE[_NBS_TOPICS] = 
{
  FLOATARRAY,
  TWIST,
  TWIST,
  TWIST, 
  FLOATARRAY, 
  FLOATARRAY, 
  FLOATARRAY, 
  FLOATARRAY, 
  FLOATARRAY, 
  FLOATARRAY, 
  FLOATARRAY,
};
