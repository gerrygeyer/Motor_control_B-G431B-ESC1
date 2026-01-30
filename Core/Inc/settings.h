/*
 * settings.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_
#include "foc.h"
#include "parameter.h"

// ####### MOTOR NUMBER ######################
#define MOTOR_NUMBER					2
// ####### SAFTY SETTINGS ####################
#define AUTOMATIC_SWITCH_OFF			OFF		// stop the motor if loosing connection to master (deactivate for debug)
#define MOTOR_DECOUPLE_CURRENT			OFF		// decouples the motor from current. OFF => the motor stopp if the system go to STOP state
// ####### CURRENT CONTROL SETTINGS ##########
#define ANTI_WINDUP						ON

#define RATE_LIMIT_SPEED				OFF
#define ANTI_WINDUP_SPEED				ON
#define BLENDING_PI_IP					ON		// Enable the two degree of freedom Speed control (read the PI_speed() function for more information)


// ####### Modulation Type ###################
//#define SPACE_VECTOR_MODULAION 			OFF // ENABLE Space Vector; DISABLE Sinusal
#define SPACE_VECTOR_MODULAION 			ON


// ####### DEBUG MODE ########################
/*
 * DEBUG MODE:
 * 	ENABLE for run the motor directly without
 * 	pressing the start button (For run code in Debug Mode)
 */

//#define DEBUG_MODE 						ON
#define DEBUG_MODE 						OFF


#endif /* INC_SETTINGS_H_ */
