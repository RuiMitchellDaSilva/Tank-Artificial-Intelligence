#include "TankManager.h"
#include "GameObject.h"
#include "BaseTank.h"
#include "ControlledTank.h"
#include "Tank_m008455c.h"
#include <SDL.h>
#include "TinyXML\tinyxml.h"
#include "Commons.h"
#include "Collisions.h"
#include <cassert>

//--------------------------------------------------------------------------------------------------

TankManager::TankManager(SDL_Renderer* renderer)
{
	LoadTanks(renderer);
}

//--------------------------------------------------------------------------------------------------

TankManager::~TankManager()
{
	for(unsigned int i = 0; i < mTanks.size(); i++)
		delete mTanks[i];
	mTanks.clear();
}

//--------------------------------------------------------------------------------------------------

void TankManager::UpdateTanks(float deltaTime, SDL_Event e)
{
	for(unsigned int i = 0; i < mTanks.size(); i++)
	{
		mTanks[i]->Update(deltaTime, e);

		//If the health is below zero, delete this tank.
		if(mTanks[i]->GetHealth() <= 0)
			mTankIndicesToDelete.push_back(i);
	}

	//Remove one Tank a frame.
	if(mTankIndicesToDelete.size() > 0)
	{
		mTanks.erase(mTanks.begin()+mTankIndicesToDelete[0]);
		mTankIndicesToDelete.erase(mTankIndicesToDelete.begin());
	}
}

//--------------------------------------------------------------------------------------------------

void TankManager::RenderTanks()
{
	for(unsigned int i = 0; i < mTanks.size(); i++)
		mTanks[i]->Render();
}

//--------------------------------------------------------------------------------------------------

void TankManager::LoadTanks(SDL_Renderer* renderer)
{
	string imagePath;

	//Get the whole xml document.
	TiXmlDocument doc;
	if(!doc.LoadFile(kTankPath))
	{
		cerr << doc.ErrorDesc() << endl;
	}

	//Now get the root element.
	TiXmlElement* root = doc.FirstChildElement();
	if(root == NULL)
	{
		cerr << "Failed to load file: No root element." << endl;
		doc.Clear();
	}
	else
	{
		TankSetupDetails details;
				
		//Jump to the first 'tank' element - within 'data'
		for(TiXmlElement* tankElement = root->FirstChildElement("tank"); tankElement != NULL; tankElement = tankElement->NextSiblingElement())
		{
			details.StudentName			= tankElement->Attribute("studentName");
			details.TankType			= atoi(tankElement->Attribute("tankType"));
			details.TankImagePath		= tankElement->Attribute("tankPath");
			details.ManImagePath		= tankElement->Attribute("manPath");
			details.TurnRate			= (float)atof(tankElement->Attribute("turnRate"));
			details.StartPosition		= Vector2D((float)atof(tankElement->Attribute("x")), (float)atof(tankElement->Attribute("y")));
			details.Health				= atoi(tankElement->Attribute("health"));
			details.NumOfBullets		= atoi(tankElement->Attribute("bullets"));
			details.NumOfRockets		= atoi(tankElement->Attribute("rockets"));
			details.NumOfMines			= atoi(tankElement->Attribute("mines"));
			details.Fuel				= (float)atof(tankElement->Attribute("fuel"));
			details.Mass				= (float)atof(tankElement->Attribute("mass"));
			details.MaxSpeed			= (float)atof(tankElement->Attribute("maxspeed"));
			details.LeftCannonAttached	= (bool)atoi(tankElement->Attribute("leftCannon"));
			details.RightCannonAttached = (bool)atoi(tankElement->Attribute("rightCannon"));

			//Add the new waypoint with the read in details.
			mTanks.push_back(GetTankObject(renderer, details));
		}
	}
}

//--------------------------------------------------------------------------------------------------

BaseTank* TankManager::GetTankObject(SDL_Renderer* renderer, TankSetupDetails details)
{
	//Create a new tank of required type, but then cast it to BaseTank for the vector list.
	BaseTank* newBaseTank = NULL;

	if(details.StudentName == "ControlledTank")
	{
		ControlledTank* newControlledTank = new ControlledTank(renderer, details);
		newBaseTank = (BaseTank*)newControlledTank;
	}
	else if(details.StudentName == "m008455c")
	{
		Tank_m008455c* tank_m008455c = new Tank_m008455c(renderer, details);
		newBaseTank = (BaseTank*)tank_m008455c;
	}

	//Assert if no tank was setup.
	assert(newBaseTank != NULL);

	//Return our new tank.
	return newBaseTank;
}

//--------------------------------------------------------------------------------------------------

void TankManager::CheckForCollisions(vector<GameObject*> listOfObjects)
{
	Vector2D tl, tr, bl, br;

	for(unsigned int i = 0; i < listOfObjects.size(); i++)
	{
		Rect2D rect = listOfObjects[i]->GetAdjustedBoundingBox();
		for(unsigned int j = 0; j < mTanks.size(); j++)
		{
			//mTanks[j]->GetCornersOfTank(&tl,&tr,&bl,&br);
			//if(Collisions::Instance()->PointInBox(tl, rect) || Collisions::Instance()->PointInBox(tr, rect) || 
			//   Collisions::Instance()->PointInBox(bl, rect) || Collisions::Instance()->PointInBox(bl, rect))
			if(Collisions::Instance()->PointInBox(mTanks[j]->GetPointAtFrontOfTank(), rect) ||
			   Collisions::Instance()->PointInBox(mTanks[j]->GetPointAtRearOfTank(), rect))
			{
				mTanks[j]->Rebound(listOfObjects[i]->GetCentralPosition());
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------
