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

struct EdgeCost
{
	double edgeCost;
	Waypoint * waypointOne;
	Waypoint * waypointTwo;
};

//---------------------------------------------------------------

struct WaypointStruct
{
	Waypoint * waypoint;
	bool visited = false;
	double cost;
	vector<EdgeCost> edges;
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
	// All constants

	float detectionDistance = 50.0f;


	// All member variables

	TURN_DIRECTION  mTankTurnDirection;
	bool			mTankTurnKeyDown;
	MOVE_DIRECTION  mTankMoveDirection;
	bool			mTankMoveKeyDown;
	TURN_DIRECTION  mManTurnDirection;
	bool			mManKeyDown;
	bool			mFireKeyDown;

	double mPreviousMousePointX = NULL;
	double mPreviousMousePointY = NULL;

	BehaviourState mCurrentBehaviourState = SEEK;

	Vector2D mMousePoint;
	Vector2D mSideVector;
	Vector2D mTargetTankPos;
	float mRadius;
	int mCurrentWaypointID = 0;
	SDL_Renderer* mRenderer;
	bool mCloseToTwoObstacles = false;
	bool mCloseToObstacle = false;
	bool mDrawDebugLines = true;

	// Waypoint/Edge list
	std::vector<WaypointStruct*> mainList;
	std::vector<EdgeCost*> edgeList;




	//DEUG
	float colour1 = 0.0f;
	float colour2 = 0.0f;
	float colour3 = 0.0f;
	float colour4 = 0.0f;
	float colour5 = 0.0f;

	// All methods

	void TankMove(float deltaTime);
	void CheckMouseInput(SDL_Event e);
	void Execute(float deltaTime, SDL_Event e);
	void ChangeBehaviour(BehaviourState newBehaviourState) { mCurrentBehaviourState = newBehaviourState; };
	Vector2D CalculateForce(Vector2D targetPos);
	void DebugLines(GameObject* obstacle);
	void DrawLine(Vector2D startPoint, Vector2D endPoint, int r, int g, int b);
	double CalculateAngleDiff(Vector2D heading);

	// All Behavioural States that the tank can have and their relevant methods

	void Thinking();
	Vector2D Seek(Vector2D targetPos);
	Vector2D Flee(Vector2D targetPos);
	Vector2D Arrive(Vector2D targetPos);
	Vector2D Pursuit(BaseTank* targetTank);
	Vector2D Evade(float deltaTime, SDL_Event e);
	Vector2D Wandering();
	Vector2D ObstacleAvoidance(Vector2D targetPos);
	bool CheckObstacleCollision(Vector2D position, GameObject* obstacle, bool withRadius);
	bool CheckRadialCollision(GameObject* obstacle);
	void Pathfind(float deltaTime, SDL_Event e);
	Vector2D FollowWaypoint();

	void PlotBestPath(Vector2D endPoint);
	void SetupWaypointData();
	void DEBUGDRAWWAYPOINTLINES();
};

//---------------------------------------------------------------



#endif //CONTROLLEDTANK_H