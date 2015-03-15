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

	mMousePoint.x = GetCentralPosition().x;
	mMousePoint.y = GetCentralPosition().y;

	mRenderer = renderer;

	mRadius = (float)sqrt(((GetAdjustedBoundingBox().height / 2) * (GetAdjustedBoundingBox().height / 2)) + 
		((GetAdjustedBoundingBox().width / 2) * (GetAdjustedBoundingBox().width / 2)));

	//SetupWaypointData();
	//PlotBestPath({100.0f, 100.0f});
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
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::MoveInHeadingDirection(float deltaTime)
{
	deltaTime *= 1;

	//Get the force that propels in current heading.
	Vector2D force = CalculateForce(mMousePoint);

	//Acceleration = Force/Mass
	Vector2D acceleration = force / GetMass();

	//Update velocity.
	mVelocity += acceleration * deltaTime;

	//Don't allow the tank does not go faster than max speed.
	mVelocity.Truncate(GetMaxSpeed()); //TODOL: Add Penalty for going faster than MAX Speed.
	
	if ((mMousePoint - GetCentralPosition()).Length() > 5)
		RotateHeadingToFacePosition(GetCentrePosition() + Vec2DNormalize(mVelocity));

	// Finally, update the position. The vehicle will first have to face towards the location to move.
	Vector2D correctHeading = mHeading * -1;

	//if (!mCloseToObstacle)
	//{
		Vector2D newPosition = GetPosition();
		newPosition.x += (correctHeading.x * (mVelocity.Length() * CalculateAngleDiff(correctHeading)))*deltaTime;
		newPosition.y += (correctHeading.y * (mVelocity.Length() * CalculateAngleDiff(correctHeading)))*deltaTime;
		SetPosition(newPosition);
	//}
	//else
	//{

	//}
}

//--------------------------------------------------------------------------------------------------

double Tank_12008455::CalculateAngleDiff(Vector2D heading)
{
	// Angle Calulation
	Vector2D largeHeadingVector = (GetCentralPosition() + (heading * 100.0f)) - GetCentralPosition();
	Vector2D largeTargetVector = mMousePoint - GetCentralPosition();

	float dotProduct = largeHeadingVector.Dot(largeTargetVector);
	float magnitudes = largeTargetVector.Length() * largeHeadingVector.Length();

	if (magnitudes <= 0.0f)
		return 0.0f;

	double angle = acos(dotProduct / magnitudes);

	//DrawLine(GetCentralPosition(), GetCentralPosition() + largeHeadingVector, 0.0f, 255.0f, 0.0f);
	//DrawLine(GetCentralPosition(), GetCentralPosition() + largeTargetVector, 100.0f, 255.0f, 0.0f);
	//DrawLine(GetCentralPosition() + largeHeadingVector, GetCentralPosition() + largeTargetVector, 100.0f, 255.0f, 100.0f);

	double angleDifference = (Pi - angle) / Pi;

	return angleDifference;
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
	if (mDrawDebugLines)
	{
		SDL_SetRenderDrawColor(mRenderer, r, g, b, SDL_ALPHA_OPAQUE);
		SDL_RenderDrawLine(mRenderer, startPoint.x, startPoint.y, endPoint.x, endPoint.y);
		SDL_RenderDrawLine(mRenderer, startPoint.x + 1.0f, startPoint.y, endPoint.x + 1.0f, endPoint.y);
		SDL_RenderDrawLine(mRenderer, startPoint.x, startPoint.y + 1.0f, endPoint.x, endPoint.y + 1.0f);
		SDL_RenderPresent(mRenderer);
	}
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::DebugLines(GameObject* obstacle)
{

		DrawLine(obstacle->GetPosition() - Vector2D(mRadius, mRadius),
			Vector2D(obstacle->GetPosition().x + mRadius + obstacle->GetAdjustedBoundingBox().width, obstacle->GetPosition().y - mRadius), 255.0f, 0.0f, 0.0f);

		DrawLine(obstacle->GetPosition() - Vector2D(mRadius, mRadius),
			Vector2D(obstacle->GetPosition().x - mRadius, obstacle->GetPosition().y + mRadius + obstacle->GetAdjustedBoundingBox().height), 255.0f, 0.0f, 0.0f);

		DrawLine(Vector2D(obstacle->GetPosition().x - mRadius, obstacle->GetPosition().y + mRadius + obstacle->GetAdjustedBoundingBox().height),
			Vector2D(obstacle->GetPosition().x + mRadius + obstacle->GetAdjustedBoundingBox().width, obstacle->GetPosition().y + mRadius + obstacle->GetAdjustedBoundingBox().height), 255.0f, 0.0f, 0.0f);

		DrawLine(Vector2D(obstacle->GetPosition().x + mRadius + obstacle->GetAdjustedBoundingBox().width, obstacle->GetPosition().y - mRadius),
			Vector2D(obstacle->GetPosition().x + mRadius + obstacle->GetAdjustedBoundingBox().width, obstacle->GetPosition().y + mRadius + obstacle->GetAdjustedBoundingBox().height), 255.0f, 0.0f, 0.0f);
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::CheckMouseInput(SDL_Event e)
{
	if (e.button.type == SDL_MOUSEBUTTONDOWN)
	{
		mMousePoint.x = (double)e.button.x;
		mMousePoint.y = (double)e.button.y;
	}
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::CalculateForce(Vector2D targetPos)
{
	Vector2D netForce = Vector2D(0.0f, 0.0f);

	mMousePoint = FollowWaypoint();

	netForce += ObstacleAvoidance(mMousePoint);

	netForce += Arrive(mMousePoint);

	//DEBUGDRAWWAYPOINTLINES();

	return netForce;
}

//--------------------------------------------------------------------------------------------------


void Tank_12008455::Execute(float deltaTime, SDL_Event e)
{
	//CheckMouseInput(e);

}

//--------------------------------------------------------------------------------------------------


void Tank_12008455::Thinking()
{
	// Determine next action to take depending on current situation.

	// Find out what information is available to the tank.
}

//--------------------------------------------------------------------------------------------------


Vector2D Tank_12008455::Seek(Vector2D targetPos)
{	
	// Set a unit vector towards the target position.
	Vector2D desiredVelocity = Vec2DNormalize(targetPos - GetCentrePosition())
		* GetMaxSpeed();

	return (desiredVelocity - mVelocity);
}

//--------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::Arrive(Vector2D targetPos)
{
	// Similair to seek, except that the tank deccelerates as it closes in on the target.
	// Unit vector to target.
	Vector2D targetVector = targetPos - GetCentrePosition();

	// Distance to target.
	double distance = targetVector.Length();

	// If the distance is more than 100 move at maximun speed, otherwise slow down as you approach target position.
	if (distance > 100)
	{
		double speed = GetMaxSpeed();
		speed = min(speed, GetMaxSpeed());
		Vector2D desiredVelocity = targetVector * (speed / distance);

		Vector2D result = desiredVelocity - mVelocity;

		return result;
	}
	else if (distance > 0)
	{
		double speed = distance / 3.0f;
		speed = min(speed, GetMaxSpeed());
		Vector2D desiredVelocity = targetVector * (speed / distance);

		Vector2D result = desiredVelocity - mVelocity;

		return result;
	}

	return Vector2D(0.0f, 0.0f);
}

//--------------------------------------------------------------------------------------------------

// This behaviour is used for moving targets as the tank predicts the position of
// the moving target and drives towards that point.
Vector2D Tank_12008455::Pursuit(BaseTank* targetTank)
{

	//Vector2D targetVector = targetTank->GetPosition() - GetPosition();




	return Vector2D(0.0f, 0.0f);

	//double predictTimeAhead = targetVector.Length() / (mMaxSpeed + targetTank->
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::Evade(float deltaTime, SDL_Event e)
{
	// Opposite direction of Pursuit().
	// (flee from predicted position of the moving target).

	Vector2D targetPos;
	return targetPos;
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::Wandering()
{
	Vector2D targetPos;
	return targetPos;

	// Perform random movement by choosing a random point on a
	// projected circle to seek towards.
	// Radius of the circle and it's projection distance can be altered to give
	// a more random wander.
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::ObstacleAvoidance(Vector2D targetPos)
{
	// Avoid obstacles that will collide with the tank if the tank
	// continues it's path.



	// Acquire the obstacle list
	vector<GameObject*> obstacleList = ObstacleManager::Instance()->GetObstacles();

	// The Steering Vector thats returned
	Vector2D steeringVector = { 0.0f, 0.0f };

	// Update sideVectors
	mSideVector.x = mHeading.y;
	mSideVector.y = -mHeading.x;
	
	// Only apply collision resolution to the nearest obstacle
	GameObject* closestCollObst = NULL;
	
	// NEEDS WORK
	// Acquire the total number of obstacles the vehicle is near to
	//int numOfNearObstacles = 0;


	for (unsigned int i = 0; i < obstacleList.size(); i++)
	{


		////////////////////////////////////////////////////////////////////////////////////////////
		// A detection vector that scales with the object's speed, this will allow the vehicle to
		// detect obstacles in front.
		//
		Vector2D detectionScaling = ((mHeading * -detectionDistance) * (mVelocity.Length() / GetMaxSpeed()));

		Vector2D position = GetCentralPosition() + (mHeading * -detectionDistance) + detectionScaling;

		// DEBUG
		// Draw detection box
		Vector2D front = GetCentralPosition() + (mHeading * -(GetAdjustedBoundingBox().height / 2));
		Vector2D rightCorner = front + (mSideVector * (GetAdjustedBoundingBox().width / 2));
		Vector2D leftCorner = front - (mSideVector * (GetAdjustedBoundingBox().width / 2));
		DrawLine(rightCorner, leftCorner, 255.0f, 255.0f, 255.0f);
		DrawLine(rightCorner, rightCorner + detectionScaling, 255.0f, 255.0f, 255.0f);
		DrawLine(leftCorner, leftCorner + detectionScaling, 255.0f, 255.0f, 255.0f);
		DrawLine(rightCorner + detectionScaling, leftCorner + detectionScaling, 255.0f, 255.0f, 255.0f);
		//
		//	if (CheckObstacleCollision(position, obstacleList[i], true)
		//
		//
		////////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// Have an additional obstacle avoidance check with a shorter range in case the first
		// detection surpasses the size of the obstacle depth.
		//
		Vector2D shortPosition = GetCentralPosition() + (mHeading * -(detectionDistance / 3)) + ((mHeading * (-detectionDistance / 3)) *
			(mVelocity.Length() / GetMaxSpeed()));

		DrawLine(GetCentralPosition(), shortPosition, 0.0f, 0.0f, colour1);

		//
		//	CheckObstacleCollision(shortPosition, obstacleList[i], true);
		//
		///////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// Detection coming from the sides of the tank.
		//
		Vector2D rotated = mSideVector * ((mRadius * 1.5f) * (mVelocity.Length() / GetMaxSpeed()));

		Vector2D leftSideDetection = GetCentralPosition() - rotated;

		DrawLine(GetCentralPosition(), leftSideDetection, 0.0f, colour3, 0.0f);
		//
		//   CheckObstacleCollision(leftSideDetection, obstacleList[i], false)
		//
		Vector2D rightSideDetection = GetCentralPosition() + rotated;

		DrawLine(GetCentralPosition(), rightSideDetection, 0.0f, colour2, 0.0f);
		//
		//	CheckObstacleCollision(rightSideDetection, obstacleList[i], false)
		//
		///////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// Detection coming from the corners of the tank.
		//
		Vector2D leftDiagonalSideDetection = Vector2D((rotated.x * cos(Pi / 4)) - (rotated.y * sin(Pi / 4)), (rotated.x * sin(Pi / 4)) + (rotated.y * cos(Pi / 4)));

		leftDiagonalSideDetection = GetCentralPosition() - leftDiagonalSideDetection;

		DrawLine(GetCentralPosition(), leftDiagonalSideDetection, 0.0f, colour4, 0.0f);
		//
		//   CheckObstacleCollision(leftSideDetection, obstacleList[i], false)
		//
		Vector2D rightDiagonalSideDetection = Vector2D((rotated.x * cos((3 * Pi) / 4)) - (rotated.y * sin((3 * Pi) / 4)),
			(rotated.x * sin((3 * Pi) / 4)) + (rotated.y * cos((3 * Pi) / 4)));

		rightDiagonalSideDetection = GetCentralPosition() - rightDiagonalSideDetection;

		DrawLine(GetCentralPosition(), rightDiagonalSideDetection, 0.0f, colour5, 0.0f);
		//
		//	CheckObstacleCollision(rightSideDetection, obstacleList[i], false)
		//
		///////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// Compare radii of obstacle and tank to see if they're near enough for a collision to occur.
		//
		double obstacleRadius;

		Vector2D to = obstacleList[i]->GetCentralPosition() - GetCentralPosition();

		double heightSq = (obstacleList[i]->GetAdjustedBoundingBox().height / 2) * (obstacleList[i]->GetAdjustedBoundingBox().height / 2);
		double widthSq = (obstacleList[i]->GetAdjustedBoundingBox().width / 2) * (obstacleList[i]->GetAdjustedBoundingBox().width / 2);

		obstacleRadius = sqrt(heightSq + widthSq);

		// Extend the range to compensate for the obstacle corners
		double range = obstacleRadius + (mRadius * 2);

		float distance = to.Length();
		//
		//	if (distance < range)
		//
		///////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// If obstacle is behind ignore.
		//
		Vector2D localPos = obstacleList[i]->GetCentralPosition();

		Vector2D heading = mHeading * -1;

		double transformX = -GetCentralPosition().Dot(heading);
		double transformY = -GetCentralPosition().Dot(mSideVector);

		C2DMatrix transformMat;

		transformMat._11(heading.x);
		transformMat._21(heading.y);
		transformMat._31(transformX);

		transformMat._12(mSideVector.x);
		transformMat._22(mSideVector.y);
		transformMat._32(transformY);

		transformMat.TransformVector2Ds(localPos);
		//
		//	if (localPos.x >= 0.0f)
		//
		///////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// Continuation of the last collision check, determine if the obstacle falls within the 
		// width of the detection range box.
		//
		double expandedRadius = obstacleRadius + mRadius;
		//
		//	fabs(localPos.y) < expandedRadius
		//
		///////////////////////////////////////////////////////////////////////////////////////////


		////////////////////////////////////////////////////////////////////////////////////////////
		// Find the closest object to the vehicle.
		//
		double distToClosestObst = MaxDouble;

		double cX = localPos.x;
		double cY = localPos.y;

		double sqrtPart = sqrt(expandedRadius * expandedRadius - cY * cY);

		double ip = cX - sqrtPart;

		if (ip <= 0.0)
		{
			ip = cX + sqrtPart;
		}
		//
		//	if (ip < distToClosestObst)
		//
		///////////////////////////////////////////////////////////////////////////////////////////






		// 1st Check :
		// Check if the vehicle is within range of the obstacle by using the vehicle's maximun
		// detection range as a radius
		if (distance < range)
		{
			// 2nd Check :
			// Check to see if the obstacle lies within the width of the detection box
			//if (fabs(localPos.y) < expandedRadius)
			//{

			if (CheckObstacleCollision(position, obstacleList[i], true))
				colour1 = 255.0f;
			else
				colour1 = 0.0f;

			if (CheckObstacleCollision(rightSideDetection, obstacleList[i], false))
				colour2 = 255.0f;
			else
				colour2 = 0.0f;

			if (CheckObstacleCollision(leftSideDetection, obstacleList[i], false))
				colour3 = 255.0f;
			else
				colour3 = 0.0f;

			if (CheckObstacleCollision(leftDiagonalSideDetection, obstacleList[i], false))
				colour4 = 255.0f;
			else
				colour4 = 0.0f;

			if (CheckObstacleCollision(rightDiagonalSideDetection, obstacleList[i], false))
				colour5 = 255.0f;
			else
				colour5 = 0.0f;

				// 3rd Check :
				// Check all detectors
				if (CheckObstacleCollision(position, obstacleList[i], true) || CheckObstacleCollision(rightSideDetection, obstacleList[i], false) ||
					CheckObstacleCollision(leftSideDetection, obstacleList[i], false) ||
					CheckObstacleCollision(leftDiagonalSideDetection, obstacleList[i], false) || CheckObstacleCollision(rightDiagonalSideDetection, obstacleList[i], false))
				{

					// 4th Check :
					// Determine the closest obstacle
					if (ip < distToClosestObst)
					{
						distToClosestObst = ip;

						closestCollObst = obstacleList[i];
					}	

					//if (mCloseToObstacle)
					//	numOfNearObstacles++;
					//
					//// If there's more than 2 obstacles nearby, then use a more narrow collision
					//if (numOfNearObstacles > 1)
					//	mCloseToTwoObstacles = true;
				}
			//}
		}
	}

	if (closestCollObst)
	{
		mCloseToObstacle = true;

		DrawLine(closestCollObst->GetCentralPosition(), GetCentralPosition(), 255.0f, 255.0f, 255.0f);

		// TEST BEGIN

		// Check which Side of the Obstacle the vehicle is on

		float width = closestCollObst->GetAdjustedBoundingBox().width;
		float height = closestCollObst->GetAdjustedBoundingBox().height;
		float x = closestCollObst->GetAdjustedBoundingBox().x;
		float y = closestCollObst->GetAdjustedBoundingBox().y;

		//std::string corner = "";

		//if (GetCentralPosition().x > x + width)
		//	side = "topRightCorner";
		//else if (GetCentralPosition().x < x)
		//	side = "topLeftCorner";
		//else if (GetCentralPosition().y > y + height)
		//	side = "bottomRightCorner";
		//else
		//	side = "bottomLeftCorner";


		//double distanceToObstacle = (closestCollObst->GetCentralPosition() - GetCentralPosition()).Length();

		float multiplierValue = 100.0f;

		float velocityLength = mVelocity.Length();
		float minimunValue = 75.0f;

		if (mVelocity.Length() < minimunValue)
			if (mVelocity.Length() > 10.0f)
				velocityLength = minimunValue;


		steeringVector.x = mSideVector.x * (multiplierValue * (velocityLength / GetMaxSpeed()));
		steeringVector.y = mSideVector.y * (multiplierValue * (velocityLength / GetMaxSpeed()));

		cout << "" << velocityLength << endl;


		// TEST END

		float checkSide1 = (closestCollObst->GetCentralPosition() - GetPosition()).Length();
		float checkSide2 = (closestCollObst->GetCentralPosition() - (GetPosition() + steeringVector)).Length();

		if (checkSide1 > checkSide2)
		{
			steeringVector.x = -steeringVector.x;
			steeringVector.y = -steeringVector.y;
		}
	}
	else
		mCloseToObstacle = false;



	return steeringVector;
}

//--------------------------------------------------------------------------------------------------

bool Tank_12008455::CheckObstacleCollision(Vector2D position, GameObject* obstacle, bool withRadius)
{
	float radius = 0.0f;

	if (withRadius)
		radius = GetAdjustedBoundingBox().height/2;

	float width = obstacle->GetAdjustedBoundingBox().width;
	float height = obstacle->GetAdjustedBoundingBox().height;
	float x = obstacle->GetAdjustedBoundingBox().x;
	float y = obstacle->GetAdjustedBoundingBox().y;

	DrawLine(obstacle->GetCentralPosition(), Vector2D(x, y), 0.0f, 0.0f, 255.0f);

	if (position.x - radius > x + width)
		return false;
	else if (position.x + radius < x)
		return false;
	else if (position.y - radius > y + height)
		return false;
	else if (position.y + radius < y)
		return false;

	return true;
}

//--------------------------------------------------------------------------------------------------

bool Tank_12008455::CheckRadialCollision(GameObject* obstacle)
{
	Vector2D to = obstacle->GetCentralPosition() - GetPosition();

	double heightSq = (obstacle->GetAdjustedBoundingBox().height / 2) * (obstacle->GetAdjustedBoundingBox().height / 2);
	double widthSq = (obstacle->GetAdjustedBoundingBox().width / 2) * (obstacle->GetAdjustedBoundingBox().width / 2);
	
	float obstacleRadius = sqrt(heightSq + widthSq);
	
	double range = obstacleRadius;
	
	float distance = to.Length();
	
	if (distance < range)
		return true;

	return false;
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::Pathfind(float deltaTime, SDL_Event e)
{
	// Move to the target position, finding the quickest route around objects 
	// and obstacles.
}

//--------------------------------------------------------------------------------------------------

Vector2D Tank_12008455::FollowWaypoint()
{
	if (WaypointManager::Instance()->GetWaypointWithID(mCurrentWaypointID))
	{
		Vector2D wayPointPos = WaypointManager::Instance()->GetWaypointWithID(mCurrentWaypointID)->GetPosition();

		double distanceToWaypoint = Vec2DLength(wayPointPos - GetCentralPosition());

		DrawLine(GetCentralPosition(), wayPointPos, 255.0f, 0.0f, 255.0f);

		// If the waypoint has been reached.
		if (distanceToWaypoint < 50.0f)
		{ 
			mCurrentWaypointID++;
			return GetCentralPosition();
		}		
		else
			return wayPointPos; // Else seek towards the waypoint.
	}
	else
	{
		mCurrentWaypointID = 0;
		return GetCentralPosition();
	}		
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::DEBUGDRAWWAYPOINTLINES()
{
	for (int i = 0; i < edgeList.size(); i++)	
		DrawLine(edgeList[i]->waypointOne->GetPosition(), edgeList[i]->waypointTwo->GetPosition(), 0.0f, 255.0f, 255.0f);
	
}

//--------------------------------------------------------------------------------------------------

void Tank_12008455::SetupWaypointData()
{
	int i = 0;

	while (true)
	{
		WaypointStruct* waypoint = new WaypointStruct();

		if (!WaypointManager::Instance()->GetWaypointWithID(i))
			break;

		waypoint->waypoint = WaypointManager::Instance()->GetWaypointWithID(i);
		waypoint->cost = MaxDouble;

		mainList.push_back(waypoint);

		i++;
	}

	// Set up edge costs
	for (int j = 0; j < mainList.size(); j++)
	{
		vector<int> connectedList = mainList.at(j)->waypoint->GetConnectedWaypointIDs();

		for (int k = 0; k < connectedList.size(); k++)
		{
			Vector2D pointPos = mainList.at(connectedList[k])->waypoint->GetPosition();

			double distance = (pointPos - mainList.at(j)->waypoint->GetPosition()).Length();

			EdgeCost* edge = new EdgeCost;
			edge->edgeCost = distance;
			edge->waypointOne = mainList.at(j)->waypoint;
			edge->waypointTwo = mainList.at(connectedList[k])->waypoint;

			bool alreadyListed = false;

			// Check to see if the edge is already on the list, else don't add
			
			for (int h = 0; h < edgeList.size(); h++)
			{

				if (edgeList.at(h)->waypointOne == edge->waypointOne)
				{
					if (edgeList.at(h)->waypointTwo == edge->waypointTwo)
						alreadyListed = true;
				}
				else if (edgeList.at(h)->waypointOne == edge->waypointTwo)
				{
					if (edgeList.at(h)->waypointTwo == edge->waypointOne)
						alreadyListed = true;
				}
							
			}

			if (!alreadyListed)
				edgeList.push_back(edge);
		}
	}

}

//--------------------------------------------------------------------------------------------------