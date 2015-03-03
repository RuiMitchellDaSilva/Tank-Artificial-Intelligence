#include "Tank_12008455.h"

//--------------------------------------------------------------------------------------------------

Tank_12008455::Tank_12008455(SDL_Renderer* renderer, TankSetupDetails details)
: BaseTank(renderer, details)
{
	mTankTurnDirection = DIRECTION_UNKNOWN;
	mTankTurnKeyDown = false;
	mTankMoveDirection = DIRECTION_NONE;
	mTankMoveKeyDown = false;
	mManTurnDirection = DIRECTION_UNKNOWN;
	mManKeyDown = false;
	mFireKeyDown = false;

	mousePoint.x = GetCentralPosition().x;
	mousePoint.y = GetCentralPosition().y;

	mRenderer = renderer;

	mRadius = (float)sqrt(((GetAdjustedBoundingBox().height / 2) * (GetAdjustedBoundingBox().height / 2)) + ((GetAdjustedBoundingBox().width / 2) * (GetAdjustedBoundingBox().width / 2)));
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::TankMove(float deltaTime)
{
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::ChangeState(BASE_TANK_STATE newState)
{
	BaseTank::ChangeState(newState);
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::Update(float deltaTime, SDL_Event e)
{
	CheckMouseInput(e);
	//Call parent update.
	BaseTank::Update(deltaTime, e);
	DebugLines();
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::MoveInHeadingDirection(float deltaTime)
{
	deltaTime *= 1;

	//Get the force that propels in current heading.
	Vector2D force = CalculateForce(mousePoint);

	//Acceleration = Force/Mass
	Vector2D acceleration = force / GetMass();

	//Update velocity.
	mVelocity += acceleration * deltaTime;

	//Don't allow the tank does not go faster than max speed.
	mVelocity.Truncate(GetMaxSpeed()); //TODOL: Add Penalty for going faster than MAX Speed.
	
	mVelocity.Normalize();

	if ((mousePoint - GetCentralPosition()).Length() > 5)
	{ 
		RotateHeadingToFacePosition(GetCentrePosition() + mVelocity);
	}	

	// If the tank is not pointing towards its velocity direction, then don't move and turn first.
	Vector2D tankVector = Vec2DNormalize(GetCentrePosition() + mVelocity);
	Vector2D velocityVector = Vec2DNormalize(mVelocity);
	float dotProduct = ((tankVector.x * velocityVector.x) + (tankVector.y * velocityVector.y)) / (tankVector.Length() * velocityVector.Length());
	float angle = acos(dotProduct);


	cout << (float)angle << endl;

	//Finally, update the position.
	Vector2D newPosition = GetPosition();
	newPosition.x += (-mHeading.x * 50)*deltaTime;
	newPosition.y += (-mHeading.y * 50)*deltaTime;
	SetPosition(newPosition);
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::RotateTank(Vector2D targetPos)
{
	Vector2D targetVector = targetPos - GetCentrePosition();

	// Distance to target.
	double distance = targetVector.Length();

	// Do not rotate if at the target position.
	if (distance > 20.0f)
		RotateHeadingToFacePosition(mousePoint);
}

//--------------------------------------------------------------------------------------------------

Rect2D Tank_12008455::GetAdjustedBoundingBox()
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

void Tank_12008455::DrawLine(Vector2D startPoint, Vector2D endPoint, int r, int g, int b)
{
	SDL_SetRenderDrawColor(mRenderer, r, g, b, SDL_ALPHA_OPAQUE);
	SDL_RenderDrawLine(mRenderer, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
	SDL_RenderDrawLine(mRenderer, startPoint.x + 1.0f, startPoint.y, endPoint.x + 1.0f, endPoint.y);
	SDL_RenderDrawLine(mRenderer, startPoint.x, startPoint.y + 1.0f, endPoint.x, endPoint.y + 1.0f);
	SDL_RenderPresent(mRenderer);
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::DebugLines()
{
	// Tank
	DrawLine(GetCentralPosition(), GetCentralPosition() + (mHeading * -50), 255.0f, 0.0f, 0.0f); 
	//DrawLine(GetCentralPosition(), mousePoint, 255.0f, 0.0f, 255.0f);

	vector<GameObject*> obstacleList = ObstacleManager::Instance()->GetObstacles();

	for each (GameObject* obstacles in obstacleList)
	{
		DrawLine(obstacles->GetPosition() - Vector2D(mRadius, mRadius),
			Vector2D(obstacles->GetPosition().x + mRadius + obstacles->GetAdjustedBoundingBox().width, obstacles->GetPosition().y - mRadius), 255.0f, 0.0f, 0.0f);

		DrawLine(obstacles->GetPosition() - Vector2D(mRadius, mRadius),
			Vector2D(obstacles->GetPosition().x - mRadius, obstacles->GetPosition().y + mRadius + obstacles->GetAdjustedBoundingBox().height), 255.0f, 0.0f, 0.0f);

		DrawLine(Vector2D(obstacles->GetPosition().x - mRadius, obstacles->GetPosition().y + mRadius + obstacles->GetAdjustedBoundingBox().height),
			Vector2D(obstacles->GetPosition().x + mRadius + obstacles->GetAdjustedBoundingBox().width, obstacles->GetPosition().y + mRadius + obstacles->GetAdjustedBoundingBox().height), 255.0f, 0.0f, 0.0f);

		DrawLine(Vector2D(obstacles->GetPosition().x + mRadius + obstacles->GetAdjustedBoundingBox().width, obstacles->GetPosition().y - mRadius),
			Vector2D(obstacles->GetPosition().x + mRadius + obstacles->GetAdjustedBoundingBox().width, obstacles->GetPosition().y + mRadius + obstacles->GetAdjustedBoundingBox().height), 255.0f, 0.0f, 0.0f);
	}

}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::CheckMouseInput(SDL_Event e)
{
	if (e.button.type == SDL_MOUSEBUTTONDOWN)
	{
		mousePoint.x = (double)e.button.x;
		mousePoint.y = (double)e.button.y;
	}
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::CalculateForce(Vector2D targetPos)
{
	Vector2D netForce = Vector2D(0.0f, 0.0f);

	netForce += FollowWaypoint();
	//netForce += Arrive(targetPos);

	return netForce;
}

void Tank_12008455::Execute(float deltaTime, SDL_Event e)
{
	//CheckMouseInput(e);

}

void Tank_12008455::Thinking()
{
	// Determine next action to take depending on current situation.

	// Find out what information is available to the tank.
}

Vector2D Tank_12008455::Seek(Vector2D targetPos)
{	
	// Set a unit vector towards the target position.
	Vector2D desiredVelocity = Vec2DNormalize(targetPos - GetCentrePosition())
		* GetMaxSpeed();

	return (desiredVelocity - mVelocity);
}

Vector2D Tank_12008455::Flee(Vector2D targetPos)
{
	vector<BaseTank*> tankList = TankManager::Instance()->GetTanks();

	for each (BaseTank* tank in tankList)
	{
		if (tank != this)
		{
			// Opposite direction to Seek.
			//Vector2D desiredVelocity = Vec2DNormalize(GetCentrePosition() - tank->GetCentralPosition())
			//	* GetMaxSpeed();

		//	return (desiredVelocity + mVelocity);
		}
	}

	return Vector2D(0.0f, 0.0f);
}

Vector2D Tank_12008455::Arrive(Vector2D targetPos)
{
	// Similair to seek, except that the tank deccelerates as it closes in on the target.
	// Unit vector to target.
	Vector2D targetVector = targetPos - GetCentrePosition();

	// Distance to target.
	double distance = targetVector.Length();

	if (distance > 0)
	{
		double speed = distance / 3.0f;
		speed = min(speed, GetMaxSpeed());
		Vector2D desiredVelocity = targetVector * (speed / distance);

		return (desiredVelocity - mVelocity);
	}

	return Vector2D(0.0f, 0.0f);
}

Vector2D Tank_12008455::Pursuit(BaseTank* targetTank)
{
	// This behaviour is used for moving targets as the tank predicts the position of
	// the moving target and drives towards that point.
	//Vector2D targetVector = targetTank->GetPosition() - GetPosition();




	return Vector2D(0.0f, 0.0f);

	//double predictTimeAhead = targetVector.Length() / (mMaxSpeed + targetTank->
}

Vector2D Tank_12008455::Evade(float deltaTime, SDL_Event e)
{
	// Opposite direction of Pursuit().
	// (flee from predicted position of the moving target).

	Vector2D targetPos;
	return targetPos;
}

Vector2D Tank_12008455::Wandering()
{
	Vector2D targetPos;
	return targetPos;

	// Perform random movement by choosing a random point on a
	// projected circle to seek towards.
	// Radius of the circle and it's projection distance can be altered to give
	// a more random wander.
}

Vector2D Tank_12008455::ObstacleAvoidance(Vector2D targetPos)
{
	// Avoid obstacles that will collide with the tank if the tank
	// continues it's path.

	int count = 0;
	
	Vector2D targetVector = Vec2DNormalize(targetPos - GetPosition());
	
	sideVector.x = -targetVector.y;
	sideVector.y = targetVector.x;
	
	vector<GameObject*> obstacleList = ObstacleManager::Instance()->GetObstacles();
	
	double obstacleRadius;
	
	GameObject* closestCollObst = NULL;
	
	double distToClosestObst = MaxDouble;
	
	Vector2D localPosOfClosestObst;

	Vector2D position = GetCentralPosition() + (targetVector * (100 * (mVelocity.Length() / GetMaxSpeed())));

	Vector2D steeringVector = { 0.0f, 0.0f };
	
	for (unsigned int i = 0; i < obstacleList.size(); i++)
	{
		//if(CheckRadialCollision(obstacleList[i]))
			if (CheckObstacleCollision(position, obstacleList[i]))
			{
				DrawLine(obstacleList[i]->GetCentralPosition(), GetCentralPosition(), 255.0f, 255.0f, 255.0f);

				steeringVector.x = sideVector.x * (100 * (mVelocity.Length() / GetMaxSpeed()));
				steeringVector.y = sideVector.y * (100 * (mVelocity.Length() / GetMaxSpeed()));

				float checkSide1 = (obstacleList[i]->GetCentralPosition() - GetPosition()).Length();
				float checkSide2 = (obstacleList[i]->GetCentralPosition() - (GetPosition() + steeringVector)).Length();

				if (checkSide1 > checkSide2)
				{
					steeringVector.x = -steeringVector.x;
					steeringVector.y = -steeringVector.y;
				}
				else
					int i = 0;

				return steeringVector;
			}
	}


	//for (unsigned int i = 0; i < obstacleList.size(); i++)
	//{
	//	Vector2D to = obstacleList[i]->GetCentralPosition() - GetPosition();
	//
	//	double heightSq = (obstacleList[i]->GetAdjustedBoundingBox().height) * (obstacleList[i]->GetAdjustedBoundingBox().height);
	//	double widthSq = (obstacleList[i]->GetAdjustedBoundingBox().width) * (obstacleList[i]->GetAdjustedBoundingBox().width);
	//
	//	obstacleRadius = sqrt(heightSq + widthSq);
	//
	//	double range = obstacleRadius;
	//
	//	float distance = to.Length();
	//
	//	if (distance < range)
	//	{
	//		Vector2D localPos = obstacleList[i]->GetCentralPosition();
	//		
	//		double transformX = -GetCentralPosition().Dot(targetVector);
	//		double transformY = -GetCentralPosition().Dot(sideVector);
	//		
	//		C2DMatrix transformMat;
	//		
	//		transformMat._11(targetVector.x);
	//		transformMat._21(targetVector.y);
	//		transformMat._31(transformX);
	//		
	//		transformMat._12(sideVector.x);
	//		transformMat._22(sideVector.y);
	//		transformMat._32(transformY);
	//
	//		transformMat.TransformVector2Ds(localPos);
	//
	//		// If obstacle is behind ignore
	//		if (localPos.x >= 0.0f)
	//		{
	//			double ExpandedRadius = obstacleRadius + mRadius;
	//
	//			// If possibility of intersection
	//			if (fabs(localPos.y) < ExpandedRadius)
	//			{
	//				double cX = localPos.x;
	//				double cY = localPos.y;
	//
	//				double SqrtPart = sqrt(ExpandedRadius*ExpandedRadius - cY*cY);
	//
	//				double ip = cX - SqrtPart;
	//
	//				if (ip <= 0.0)
	//				{
	//					ip = cX + SqrtPart;
	//				}
	//
	//				//test to see if this is the closest so far. If it is keep a
	//				//record of the obstacle and its local coordinates
	//				if (ip < distToClosestObst)
	//				{
	//					count++;
	//
	//					distToClosestObst = ip;
	//
	//					closestCollObst = obstacleList[i];
	//
	//					localPosOfClosestObst = localPos;
	//				}	
	//			}
	//		}
	//	}
	//}



	

	if (closestCollObst)
	{
		//cout << closestCollObst->GetCentralPosition << endl << endl << endl << endl << endl << endl << endl << endl << endl << endl << endl << endl << endl;

		count = 0;
		//obstacleRadius = sqrt(((closestCollObst->GetAdjustedBoundingBox().height / 1.5) * (closestCollObst->GetAdjustedBoundingBox().height / 1.5)) +
		//	((closestCollObst->GetAdjustedBoundingBox().width / 1.5) * (closestCollObst->GetAdjustedBoundingBox().width / 1.5)));
		//
		//double multiplier = 1.0f + (5 - localPosOfClosestObst.x) / 5;
		//
		//steeringVector.y = (obstacleRadius - localPosOfClosestObst.y) * multiplier;
		//
		//const double brakeForce = 0.2f;
		//
		//steeringVector.x = (obstacleRadius - localPosOfClosestObst.x) * 0.2f;
		//
		//C2DMatrix transformMat;
		//
		//transformMat.Rotate(targetVector, sideVector);
		//
		//transformMat.TransformVector2Ds(steeringVector);	

		//Vector2D aheadVector;
		//aheadVector = GetCentralPosition() + Vec2DNormalize(mVelocity) * (mVelocity.Length()/GetMaxSpeed());
		
		//if (CheckObstacleCollision(aheadVector, closestCollObst))
		//{
		steeringVector.x = 5 - closestCollObst->GetCentralPosition().x;
		steeringVector.y = 5 - closestCollObst->GetCentralPosition().y;
		
		steeringVector.Normalize();
		steeringVector.x *= ((mVelocity.Length() / GetMaxSpeed()));
		steeringVector.y *= ((mVelocity.Length() / GetMaxSpeed()));
		
		return steeringVector;
		//}
		//return steeringVector;
	}

	return Arrive(targetPos);
}

bool Tank_12008455::CheckObstacleCollision(Vector2D position, GameObject* obstacle)
{
	float width = obstacle->GetAdjustedBoundingBox().width;
	float height = obstacle->GetAdjustedBoundingBox().height;
	float x = obstacle->GetAdjustedBoundingBox().x;
	float y = obstacle->GetAdjustedBoundingBox().y;

	if (position.x > x + width)
		return false;
	else if (position.x < x)
		return false;
	else if (position.y > y + height)
		return false;
	else if (position.y < y)
		return false;

	return true;
}

bool Tank_12008455::CheckRadialCollision(GameObject* obstacle)
{
	Vector2D to = obstacle->GetCentralPosition() - GetPosition();

	double heightSq = (obstacle->GetAdjustedBoundingBox().height) * (obstacle->GetAdjustedBoundingBox().height);
	double widthSq = (obstacle->GetAdjustedBoundingBox().width) * (obstacle->GetAdjustedBoundingBox().width);
	
	float obstacleRadius = sqrt(heightSq + widthSq);
	
	double range = obstacleRadius;
	
	float distance = to.Length();
	
	if (distance < range)
		return true;

	return false;
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

void Tank_12008455::Pathfind(float deltaTime, SDL_Event e)
{
	// Move to the target position, finding the quickest route around objects 
	// and obstacles.
}

Vector2D Tank_12008455::FollowWaypoint()
{
	Vector2D force = { 0.0f, 0.0f };

	if (WaypointManager::Instance()->GetWaypointWithID(currentWaypointID))
	{
		Vector2D wayPointPos = WaypointManager::Instance()->GetWaypointWithID(currentWaypointID)->GetPosition();		

		double distanceToWaypoint = Vec2DLength(wayPointPos - GetCentralPosition());

		DrawLine(GetCentralPosition(), wayPointPos, 255.0f, 0.0f, 255.0f);
		//cout << (int)distanceToWaypoint << endl;

		// If the waypoint has been reached.
		if (distanceToWaypoint < 50.0f)
		{ 
			currentWaypointID++;
			return{ 0.0f, 0.0f };
		}		
		else
			return ObstacleAvoidance(wayPointPos); // Else seek towards the waypoint.
	}
	else
	{
		currentWaypointID = 0;
		return{ 0.0f, 0.0f };
	}		
}

