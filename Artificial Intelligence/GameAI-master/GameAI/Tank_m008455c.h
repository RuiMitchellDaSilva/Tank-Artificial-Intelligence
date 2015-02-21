#ifndef Tank_m008455c_H
#define Tank_m008455c_H

#include "BaseTank.h"
#include <SDL.h>
#include "Commons.h"
#include "ObstacleManager.h"
#include "WaypointManager.h"
#include "Waypoint.h"
#include "C2DMatrix.h"

#include "Tank_m008455c.h"

#include <iostream>

using namespace std;

enum BehaviourState
{
	THINKING,
	SEEK,
	FLEE,
	ARRIVE,
	PURSUIT,
	EVADE,
	WANDERING,
	OBSTACLE_AVOIDANCE,
	PATHFIND,
	ATTACK,
	FOLLOWAYPOINT // DEBUG
};

//---------------------------------------------------------------

class Tank_m008455c : private BaseTank
{
	//---------------------------------------------------------------
public:
	Tank_m008455c(SDL_Renderer* renderer, TankSetupDetails details);
	~Tank_m008455c();

	void ChangeState(BASE_TANK_STATE newState);

	void Update(float deltaTime, SDL_Event e);

	//---------------------------------------------------------------
protected:
	Rect2D	GetAdjustedBoundingBox();
	void	MoveInHeadingDirection(float deltaTime);

private:
	TURN_DIRECTION  mTankTurnDirection;
	bool			mTankTurnKeyDown;
	MOVE_DIRECTION  mTankMoveDirection;
	bool			mTankMoveKeyDown;
	TURN_DIRECTION  mManTurnDirection;
	bool			mManKeyDown;
	bool			mFireKeyDown;

	Vector2D mousePoint;

	double previousMousePointX = NULL;
	double previousMousePointY = NULL;

	BehaviourState currentBehaviourState = SEEK;

	Vector2D targetTankPos;

	int currentWaypointID = 0;

	Vector2D sideVector;
	
	void TankMove(float deltaTime);

	void CheckMouseInput(SDL_Event e);

	void Execute(float deltaTime, SDL_Event e);
	void ChangeBehaviour(BehaviourState newBehaviourState) { currentBehaviourState = newBehaviourState; };

	Vector2D CalculateForce(Vector2D targetPos);

	void RotateTank(Vector2D targetPos);

	float mRadius = (float)sqrt((GetAdjustedBoundingBox().height * GetAdjustedBoundingBox().height) + (GetAdjustedBoundingBox().width * GetAdjustedBoundingBox().width));

	// All Behavioural States that the tank can have
	void Thinking();
	Vector2D Seek(Vector2D targetPos);
	Vector2D Flee(Vector2D targetPos);
	Vector2D Arrive(Vector2D targetPos);
	Vector2D Pursuit(BaseTank* targetTank);
	void Evade(float deltaTime, SDL_Event e);
	void Wandering(float deltaTime, SDL_Event e);


	Vector2D ObstacleAvoidance(Vector2D targetPos);

	void Pathfind(float deltaTime, SDL_Event e);
	Vector2D FollowWaypoint();
};

//---------------------------------------------------------------



#endif //CONTROLLEDTANK_H