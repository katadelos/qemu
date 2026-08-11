/* Cypress TrueTouch Gen3 controller used by the Kindle Celeste. */

#ifndef HW_I2C_CYTTSP_H
#define HW_I2C_CYTTSP_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_CYTTSP "cyttsp"
OBJECT_DECLARE_SIMPLE_TYPE(CYTTSPState, CYTTSP)

#endif
