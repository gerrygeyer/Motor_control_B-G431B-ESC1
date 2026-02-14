/*
 * settings.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "parameter.h"

// ####### MOTOR NUMBER ######################
#define MOTOR_NUMBER					2
// ####### SAFTY SETTINGS ####################
#define AUTOMATIC_SWITCH_OFF			OFF		// stop the motor if loosing connection to master (deactivate for debug)

#define MIN_RPM_COMMAND_FROM_MASTER     90     // in RPM
#define LOSE_CONNECTION_TIME            0.2f // in seconds    
#define INIT_SAFETY_TIME                5 // time in seconds bevor we can go again in GOTOSTART in seconds

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
