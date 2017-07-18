/*
 * Controller.h
 *
 *  Created on: Mar 19, 2016
 *      Author: klchow
 */

#include "geometry_msgs/PoseStamped.h"

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

class Controller {
public:
	Controller();
	virtual ~Controller();

	virtual void SetGoal(geometry_msgs::PoseStamped ps_goal);
	virtual geometry_msgs::PoseStamped GetGoal();



};

#endif /* CONTROLLER_H_ */
