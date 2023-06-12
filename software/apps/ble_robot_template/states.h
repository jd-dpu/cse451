#ifndef STATES_H_
#define STATES_H_

#include <stdio.h>

typedef enum {
    OFF=0,
    DRIVING,
	BACKUP,
	RIGHT,
	LEFT,
	REORIENT,
} states;

typedef enum {
    LEFT_TURN,
	RIGHT_TURN,
	STRAIGHT,
} turns;

typedef enum{
	FIRST_LEG,
	RIGHT_ANGLE,
	BASE_LEG,
	OBTUSE_ANGLE,
	HYPOTNUSE,
	DONE
}triangle_path_states;

#endif /* STATES_H_ */
