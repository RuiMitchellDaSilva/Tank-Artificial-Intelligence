#ifndef Tank_12008455_H
#define Tank_12008455_H

#include "BaseTank.h"
#include <SDL.h>
#include "Commons.h"
#include "ObstacleManager.h"
#include "WaypointManager.h"
#include "TankManager.h"
#include "Waypoint.h"
#include "C2DMatrix.h"
#include "Math.h"

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

class Tank_12008455 : protected BaseTank
{
	//---------------------------------------------------------------
public:
	Tank_12008455(SDL_Renderer* renderer, TankSetupDetails details);
	~Tank_12008455();

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

	float mRadius;

	SDL_Renderer* mRenderer;


	// All Behavioural States that the tank can have
	void Thinking();
	Vector2D Seek(Vector2D targetPos);
	Vector2D Flee(Vector2D targetPos);
	Vector2D Arrive(Vector2D targetPos);
	Vector2D Pursuit(BaseTank* targetTank);
	Vector2D Evade(float deltaTime, SDL_Event e);
	Vector2D Wandering();

	bool CheckObstacleCollision(Vector2D position, GameObject* obstacle);
	bool CheckRadialCollision(GameObject* obstacle);
	Vector2D ObstacleAvoidance(Vector2D targetPos);

	void Pathfind(float deltaTime, SDL_Event e);
	Vector2D FollowWaypoint();

	void DebugLines();
	void DrawLine(Vector2D startPoint, Vector2D endPoint, int r, int g, int b);
};

//---------------------------------------------------------------



#endif //CONTROLLEDTANK_H