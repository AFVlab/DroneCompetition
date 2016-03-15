/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>

uint8_t safeToGoForwards=FALSE;
int tresholdColorCount = 1000; //was 200
int32_t incrementForAvoidance;

void orange_avoider_init() {
	// Initialise the variables of the colorfilter to accept orange
	color_lum_min=0;
	color_lum_max=131;
	color_cb_min=93;
	color_cb_max=255;
	color_cr_min=134;
	color_cr_max=255;
	// Initialise random values
	srand(time(NULL));
	chooseRandomIncrementAvoidance();
}
void orange_avoider_periodic() {
	// Check the amount of orange. If this is above a threshold
	// you want to turn a certain amount of degrees
	safeToGoForwards = color_count < tresholdColorCount;
	printf("Checking if this funciton is called %d treshold: %d now: %d \n", color_count, tresholdColorCount, safeToGoForwards);
}


/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}




uint8_t change_waypoint_random_inside_obstacle(uint8_t waypoint)
{
	struct EnuCoor_i new_coor;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
	float maxy = 51.990657;
	float miny = 51.990601;
	float minx = 4.376748;
	float maxx = 4.376845;
	
	srand(time(NULL));
	
	
	// Get random number
 	float ry = (float)rand()*(maxy-miny)+miny;
	float rx = (float)rand()*(maxx-minx)+minx;

	// determine the random place of the waypoint inside the obstacle zone
	
	new_coor.x = rx;
	new_coor.y = ry;
	new_coor.z = pos->z; // Keep the height the same	

	// Set the waypoint to the calculated position
	waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
	
  return FALSE;
}

uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}



uint8_t chooseRandomIncrementAvoidance(){

	int r = rand() % 2;
	if(r==0){
		incrementForAvoidance=1000; //was 350
	}
	else{
		incrementForAvoidance=-1000; // was -350
	}
	return FALSE;
}

