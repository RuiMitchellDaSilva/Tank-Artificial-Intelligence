#include "Tank_m008455c.h"
#include "Commons.h"

//--------------------------------------------------------------------------------------------------

Tank_m008455c::Tank_m008455c(SDL_Renderer* renderer, TankSetupDetails details)
: BaseTank(renderer, details)
{
	mTankTurnDirection = DIRECTION_UNKNOWN;
	mTankTurnKeyDown = false;
	mTankMoveDirection = DIRECTION_NONE;
	mTankMoveKeyDown = false;
	mManTurnDirection = DIRECTION_UNKNOWN;
	mManKeyDown = false;
	mFireKeyDown = false;

	mousePoint.x = GetPosition().x;
	mousePoint.y = GetPosition().y;
}

//--------------------------------------------------------------------------------------------------

void Tank_m008455c::TankMove(float deltaTime)
{
}

//--------------------------------------------------------------------------------------------------

void Tank_m008455c::ChangeState(BASE_TANK_STATE newState)
{
	BaseTank::ChangeState(newState);
}

//--------------------------------------------------------------------------------------------------

void Tank_m008455c::Update(float deltaTime, SDL_Event e)
{
	CheckMouseInput(e);
	//Call parent update.
	BaseTank::Update(deltaTime, e);
}

//--------------------------------------------------------------------------------------------------

void Tank_m008455c::MoveInHeadingDirection(float deltaTime)
{
	deltaTime *= 5;

	//Get the force that propels in current heading.
	Vector2D force = CalculateForce(mousePoint);

	//Acceleration = Force/Mass
	Vector2D acceleration = force / GetMass();

	//Update velocity.
	mVelocity += acceleration * deltaTime;

	//Don't allow the tank does not go faster than max speed.
	mVelocity.Truncate(GetMaxSpeed()); //TODOL: Add Penalty for going faster than MAX Speed.
	

	//Finally, update the position.
	Vector2D newPosition = GetPosition();
	newPosition.x += mVelocity.x*deltaTime;
	newPosition.y += (mVelocity.y/**-1.0f*/)*deltaTime;	//Y flipped as adding to Y moves down screen.
	SetPosition(newPosition);
}

//--------------------------------------------------------------------------------------------------

Rect2D Tank_m008455c::GetAdjustedBoundingBox()
{
	//Return an adjusted bounding box to use for collisions.
	Rect2D boundingBox = BaseTank::GetAdjustedBoundingBox();

	//Move the box position in slightly.
	boundingBox.x += boundingBox.width*0.2f;
	//Resize the width to encompass just the car image.
	boundingBox.width *= 0.6f;

	return boundingBox;
}

//--------------------------------------------------------------------------------------------------

void Tank_m008455c::SetObstacleAvoidanceArea()
{
	Rect2D avoidanceArea = GetAdjustedBoundingBox();

	avoidanceArea.height += avoidanceArea.height;
	avoidanceArea.width += avoidanceArea.width;

	//mAvoidanceArea = avoidanceArea;
}

//--------------------------------------------------------------------------------------------------


void Tank_m008455c::CheckMouseInput(SDL_Event e)
{
	if (e.button.type == SDL_MOUSEBUTTONDOWN)
	{
		mousePoint.x = (double)e.button.x;
		mousePoint.y = (double)e.button.y;
	}
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_m008455c::CalculateForce(Vector2D targetPos)
{
	Vector2D netForce = Vector2D(0.0f, 0.0f);

	netForce += Arrive(targetPos);

	return netForce;
}

void Tank_m008455c::Execute(float deltaTime, SDL_Event e)
{
	CheckMouseInput(e);

}

void Tank_m008455c::Thinking()
{
	// Determine next action to take depending on current situation.

	// Find out what information is available to the tank.
}

Vector2D Tank_m008455c::Seek(Vector2D targetPos)
{	
	// Set a unit vector towards the target position.
	Vector2D desiredVelocity = Vec2DNormalize(targetPos - GetCentrePosition())
		* GetMaxSpeed();

	return (desiredVelocity - mVelocity);
}

Vector2D Tank_m008455c::Flee(Vector2D targetPos)
{
	// Opposite direction to Seek.
	Vector2D desiredVelocity = Vec2DNormalize(GetCentrePosition() - targetPos)
		* mMaxSpeed;

	return (desiredVelocity + mVelocity);
}

Vector2D Tank_m008455c::Arrive(Vector2D targetPos)
{
	// Similair to seek, except that the tank deccelerates as it closes in on the target.
	// Unit vector to target.
	Vector2D targetVector = targetPos - GetCentrePosition();

	// Distance to target.
	double distance = targetVector.Length();

	if (distance > 0)
	{
		double speed = distance / 3.0f;
		speed = min(speed, mMaxSpeed);
		Vector2D desiredVelocity = targetVector * (speed / distance);

		return (desiredVelocity - mVelocity);
	}

	return Vector2D(0.0f, 0.0f);
}

Vector2D Tank_m008455c::Pursuit(BaseTank* targetTank)
{
	// This behaviour is used for moving targets as the tank predicts the position of
	// the moving target and drives towards that point.
	//Vector2D targetVector = targetTank->GetPosition() - GetPosition();




	return Vector2D(0.0f, 0.0f);

	//double predictTimeAhead = targetVector.Length() / (mMaxSpeed + targetTank->
}

void Tank_m008455c::Evade(float deltaTime, SDL_Event e)
{
	// Opposite direction of Pursuit().
	// (flee from predicted position of the moving target).
}

void Tank_m008455c::Wandering(float deltaTime, SDL_Event e)
{
	// Perform random movement by choosing a random point on a
	// projected circle to seek towards.
	// Radius of the circle and it's projection distance can be altered to give
	// a more random wander.
}

void Tank_m008455c::ObstacleAvoidance(float deltaTime, SDL_Event e)
{
	// Avoid obstacles that will collide with the tank if the tank
	// continues it's path.
}

//Vector2D SteeringBehavior::ObstacleAvoidance(const std::vector<BaseGameEntity*>& obstacles)
//{
//	//the detection box length is proportional to the agent's velocity
//	m_dDBoxLength = Prm.MinDetectionBoxLength +
//		(m_pVehicle->Speed() / m_pVehicle->MaxSpeed()) *
//		Prm.MinDetectionBoxLength;
//
//	//tag all obstacles within range of the box for processing
//	m_pVehicle->World()->TagObstaclesWithinViewRange(m_pVehicle, m_dDBoxLength);
//
//	//this will keep track of the closest intersecting obstacle (CIB)
//	BaseGameEntity* ClosestIntersectingObstacle = NULL;
//
//	//this will be used to track the distance to the CIB
//	double DistToClosestIP = MaxDouble;
//
//	//this will record the transformed local coordinates of the CIB
//	Vector2D LocalPosOfClosestObstacle;
//
//	std::vector<BaseGameEntity*>::const_iterator curOb = obstacles.begin();
//
//	while (curOb != obstacles.end())
//	{
//		//if the obstacle has been tagged within range proceed
//		if ((*curOb)->IsTagged())
//		{
//			//calculate this obstacle's position in local space
//			Vector2D LocalPos = PointToLocalSpace((*curOb)->Pos(),
//				m_pVehicle->Heading(),
//				m_pVehicle->Side(),
//				m_pVehicle->Pos());
//
//			//if the local position has a negative x value then it must lay
//			//behind the agent. (in which case it can be ignored)
//			if (LocalPos.x >= 0)
//			{
//				//if the distance from the x axis to the object's position is less
//				//than its radius + half the width of the detection box then there
//				//is a potential intersection.
//				double ExpandedRadius = (*curOb)->BRadius() + m_pVehicle->BRadius();
//
//				if (fabs(LocalPos.y) < ExpandedRadius)
//				{
//					//now to do a line/circle intersection test. The center of the 
//					//circle is represented by (cX, cY). The intersection points are 
//					//given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
//					//We only need to look at the smallest positive value of x because
//					//that will be the closest point of intersection.
//					double cX = LocalPos.x;
//					double cY = LocalPos.y;
//
//					//we only need to calculate the sqrt part of the above equation once
//					double SqrtPart = sqrt(ExpandedRadius*ExpandedRadius - cY*cY);
//
//					double ip = cX - SqrtPart;
//
//					if (ip <= 0.0)
//					{
//						ip = cX + SqrtPart;
//					}
//
//					//test to see if this is the closest so far. If it is keep a
//					//record of the obstacle and its local coordinates
//					if (ip < DistToClosestIP)
//					{
//						DistToClosestIP = ip;
//
//						ClosestIntersectingObstacle = *curOb;
//
//						LocalPosOfClosestObstacle = LocalPos;
//					}
//				}
//			}
//		}
//
//		++curOb;
//	}
//
//	//if we have found an intersecting obstacle, calculate a steering 
//	//force away from it
//	Vector2D SteeringForce;
//
//	if (ClosestIntersectingObstacle)
//	{
//		//the closer the agent is to an object, the stronger the 
//		//steering force should be
//		double multiplier = 1.0 + (m_dDBoxLength - LocalPosOfClosestObstacle.x) /
//			m_dDBoxLength;
//
//		//calculate the lateral force
//		SteeringForce.y = (ClosestIntersectingObstacle->BRadius() -
//			LocalPosOfClosestObstacle.y)  * multiplier;
//
//		//apply a braking force proportional to the obstacles distance from
//		//the vehicle. 
//		const double BrakingWeight = 0.2;
//
//		SteeringForce.x = (ClosestIntersectingObstacle->BRadius() -
//			LocalPosOfClosestObstacle.x) *
//			BrakingWeight;
//	}
//
//	//finally, convert the steering vector from local to world space
//	return VectorToWorldSpace(SteeringForce,
//		m_pVehicle->Heading(),
//		m_pVehicle->Side());
//}

void Tank_m008455c::Pathfind(float deltaTime, SDL_Event e)
{
	// Move to the target position, finding the quickest route around objects 
	// and obstacles.
}