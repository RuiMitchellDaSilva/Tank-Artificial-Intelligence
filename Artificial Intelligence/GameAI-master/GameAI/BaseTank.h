#ifndef BASETANK_H
#define BASETANK_H

#include "GameObject.h"
#include "Texture2D.h"
#include <SDL.h>

//---------------------------------------------------------------

class BaseTank : protected GameObject
{
	//---------------------------------------------------------------
public:
	BaseTank(SDL_Renderer* renderer, TankSetupDetails details);
	~BaseTank();

	virtual void	ChangeState(BASE_TANK_STATE newState);

	virtual void	Update(float deltaTime, SDL_Event e);
	virtual void	Render();
	Rect2D			GetAdjustedBoundingBox();

	//Attributes.
	int				GetHealth()												{return mHealth;}
	int				GetBullets()											{return mBullets;}
	int				GetRockets()											{return mRockets;}
	int				GetFuel()												{return mFuel;}
	int				GetMines()												{return mMines;}

	//Movement.
	double			GetMass()												{return mMass;}
	double			GetCurrentSpeed()										{return mCurrentSpeed;}
	double			GetMaxSpeed()											{return mMaxSpeed;}
	double			GetMaxForce()											{return mMaxForce;}
	double			GetMaxTurnRate()										{return mMaxTurnRate;}

	void			SetPosition(Vector2D newPosition)						{mPosition = newPosition;}
	Vector2D		GetPointAtFrontOfTank();
	Vector2D		GetPointAtRearOfTank();
	Vector2D		GetCentrePosition()										{return GetCentralPosition();}
	void			GetCornersOfTank(Vector2D* topLeft, Vector2D* topRight, Vector2D* bottomLeft, Vector2D* bottomRight);

	void			IncrementTankRotationAngle(double deg);
	void			IncrementManRotationAngle(double deg);

	void			DeductABullet()											{mBullets--;}
	void			DeductAMine()											{mMines--;}
	void			DeductARocket()											{mRockets--;}
	void			TakeDamage(GAMEOBJECT_TYPE projectile);

	void			Rebound(Vector2D position);

	Vector2D		GetHeading()											{return mHeading;}
	Vector2D		GetVelocity()											{return mVelocity;}
	Vector2D		GetSide()												{return mSide;}
	//---------------------------------------------------------------
protected:
	virtual void	MoveInHeadingDirection(float deltaTime);

	bool			RotateHeadingToFacePosition(Vector2D target);
	virtual void	RotateHeadingByRadian(double radian, int sign);		//Sign is direction of turn. Either -1 or 1.
	void			RotateManByRadian(double radian, int sign);

	void			SetHeading(Vector2D newHeading);

	void			FireABullet();
	void			FireRockets();
	void			DropAMine();

	//---------------------------------------------------------------
private:
	SDL_Rect		GetCurrentManSprite();
	SDL_Rect		GetCurrentCannonSprite();

	//---------------------------------------------------------------
private:
	//We need this to pass on to bullets.
	SDL_Renderer*	mRenderer;

	TANK_TYPE		mTankType;

	//Animating man in tank.
	Texture2D*		mManSpritesheet;					//The man in the tank.
	int				mManSingleSpriteHeight;
	int				mManSingleSpriteWidth;

	bool			mManFireFrame;
	float			mManFireTime;
	Vector2D		mManFireDirection;

	Vector2D		mManOffset;
	double			mManRotationAngle;

	//Animating Cannon details.
	Texture2D*		mCannonSpritesheet;					//The man in the tank.
	int				mCannonSingleSpriteHeight;
	int				mCannonSingleSpriteWidth;
	bool			mCannonAttachedRight;
	bool			mCannonAttachedLeft;
	bool			mCannonFireFrame;
	float			mCannonFireTime;
	bool			mFiringRocket;
	//Identifying details.
	string			mStudentName;

	//Attributes.
	int				mHealth;
	int				mBullets;
	int				mFuel;
	int				mMines;
	int				mRockets;
	


	//---------------------------------------------------------------
protected:
	BASE_TANK_STATE mCurrentState;

	//Movement.
	double			mCurrentSpeed;
	Vector2D	    mVelocity;
	Vector2D		mHeading;
	Vector2D		mSide;	
	
	//Movement.
	double			mMass;
	double			mMaxSpeed;
	double			mMaxForce;
	double			mMaxTurnRate;	
};

//---------------------------------------------------------------

#endif //BASETANK_H