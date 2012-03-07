//
//  HelloWorldLayer.mm
//  HighwayChaser
//
//  Created by Shervin Ghazazani on 12-03-03.
//  Copyright __MyCompanyName__ 2012. All rights reserved.
//


// Import the interfaces
#import "HelloWorldLayer.h"

//Pixel to metres ratio. Box2D uses metres as the unit for measurement.
//This ratio defines how many pixels correspond to 1 Box2D "metre"
//Box2D is optimized for objects of 1x1 metre therefore it makes sense
//to define the ratio so that your most common object type is 1x1 metre.
#define PTM_RATIO 32

// enums that will be used as tags
enum {
	kTagTileMap = 1,
	kTagBatchNode = 1,
	kTagAnimation1 = 1,
};


// HelloWorldLayer implementation
@implementation HelloWorldLayer

+(CCScene *) scene
{
	// 'scene' is an autorelease object.
	CCScene *scene = [CCScene node];
	
	// 'layer' is an autorelease object.
	HelloWorldLayer *layer = [HelloWorldLayer node];
	
	// add layer as a child to scene
	[scene addChild: layer];
	
	// return the scene
	return scene;
}

// on "init" you need to initialize your instance
-(id) init
{
	// always call "super" init
	// Apple recommends to re-assign "self" with the "super" return value
	if( (self=[super init])) {
		
		// enable touches
		self.isTouchEnabled = YES;
		
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
		
		CGSize screenSize = [CCDirector sharedDirector].winSize;
		CCLOG(@"Screen width %0.2f screen height %0.2f",screenSize.width,screenSize.height);
		
		// Define the gravity vector.
		b2Vec2 gravity;
		gravity.Set(0.0f, 0.0f);
		
		// Do we want to let bodies sleep?
		// This will speed up the physics simulation
		bool doSleep = true;
		
		// Construct a world object, which will hold and simulate the rigid bodies.
		world = new b2World(gravity, doSleep);
		
		world->SetContinuousPhysics(true);
		
		// Debug Draw functions
		m_debugDraw = new GLESDebugDraw( PTM_RATIO );
		world->SetDebugDraw(m_debugDraw);
		
		uint32 flags = 0;
		flags += b2DebugDraw::e_shapeBit;
//		flags += b2DebugDraw::e_jointBit;
//		flags += b2DebugDraw::e_aabbBit;
//		flags += b2DebugDraw::e_pairBit;
//		flags += b2DebugDraw::e_centerOfMassBit;
		m_debugDraw->SetFlags(flags);		
		
		
		// Define the ground body.
		b2BodyDef groundBodyDef;
		groundBodyDef.position.Set(0, 0); // bottom-left corner
		
		// Call the body factory which allocates memory for the ground body
		// from a pool and creates the ground box shape (also from a pool).
		// The body is also added to the world.
		b2Body* groundBody = world->CreateBody(&groundBodyDef);
		
		// Define the ground box shape.
		b2PolygonShape groundBox;		
		
		// bottom
		groundBox.SetAsEdge(b2Vec2(0,0), b2Vec2(screenSize.width/PTM_RATIO,0));
		groundBody->CreateFixture(&groundBox,0);
		
		// top
		groundBox.SetAsEdge(b2Vec2(0,screenSize.height/PTM_RATIO), b2Vec2(screenSize.width/PTM_RATIO,screenSize.height/PTM_RATIO));
		groundBody->CreateFixture(&groundBox,0);
		
		// left
		groundBox.SetAsEdge(b2Vec2(0,screenSize.height/PTM_RATIO), b2Vec2(0,0));
		groundBody->CreateFixture(&groundBox,0);
		
		// right
		groundBox.SetAsEdge(b2Vec2(screenSize.width/PTM_RATIO,screenSize.height/PTM_RATIO), b2Vec2(screenSize.width/PTM_RATIO,0));
		groundBody->CreateFixture(&groundBox,0);
        
		//Set up sprite
		
		CCSpriteBatchNode *batch = [CCSpriteBatchNode batchNodeWithFile:@"blocks.png" capacity:150];
		[self addChild:batch z:0 tag:kTagBatchNode];
		[self addNewSpriteWithCoords:ccp(screenSize.width/2, screenSize.height/2)];
		[self schedule: @selector(tick:)];
	}
	return self;
}

-(void) draw
{
	// Default GL states: GL_TEXTURE_2D, GL_VERTEX_ARRAY, GL_COLOR_ARRAY, GL_TEXTURE_COORD_ARRAY
	// Needed states:  GL_VERTEX_ARRAY, 
	// Unneeded states: GL_TEXTURE_2D, GL_COLOR_ARRAY, GL_TEXTURE_COORD_ARRAY
	glDisable(GL_TEXTURE_2D);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
	world->DrawDebugData();
	
	// restore default GL states
	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

}

-(void) addNewSpriteWithCoords:(CGPoint)p
{
	CCLOG(@"Adding car %0.2f x %02.f",p.x,p.y);
	CCSpriteBatchNode *batch = (CCSpriteBatchNode*) [self getChildByTag:kTagBatchNode];
	
	//We have a 64x64 sprite sheet with 4 different 32x32 images.  The following code is
	//just randomly picking one of the images
	int idx = (CCRANDOM_0_1() > .5 ? 0:1);
	int idy = (CCRANDOM_0_1() > .5 ? 0:1);
	CCSprite *sprite = [CCSprite spriteWithBatchNode:batch rect:CGRectMake(32 * idx,32 * idy,32,32)];
	[batch addChild:sprite];
	
	sprite.position = ccp( p.x, p.y);
	
	// Define the dynamic body.
	//Set up a 1m squared box in the physics world
	b2BodyDef bodyDef;
    bodyDef.linearDamping = 1;
    bodyDef.angularDamping = 1;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
	bodyDef.userData = sprite;
	b2Body *body = world->CreateBody(&bodyDef);
    body->ResetMassData();
    
    b2BodyDef leftWheelDef;
    leftWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    leftWheelDef.position += b2Vec2(-1.5f, -1.90f);
    leftWheelDef.type = b2_dynamicBody;
    leftWheelBody = world->CreateBody(&leftWheelDef);
    leftWheelBody->ResetMassData();
    
    b2BodyDef rightWheelDef;
    rightWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    rightWheelDef.position += b2Vec2(1.5f, -1.90f);
    rightWheelDef.type = b2_dynamicBody;
    rightWheelBody = world->CreateBody(&rightWheelDef);
    rightWheelBody->ResetMassData();
    
    b2BodyDef leftRearWheelDef;
    leftRearWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    leftRearWheelDef.position += b2Vec2(-1.5f,1.90f);
    leftRearWheelDef.type = b2_dynamicBody;
    leftRearWheelBody = world->CreateBody(&leftRearWheelDef);
    leftRearWheelBody->ResetMassData();
	
    b2BodyDef rightRearWheelDef;
    rightRearWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    rightRearWheelDef.position += b2Vec2(1.5f, 1.9f);
    rightRearWheelDef.type = b2_dynamicBody;
    rightRearWheelBody = world->CreateBody(&rightRearWheelDef);
    rightRearWheelBody->ResetMassData();
    
	// Define another box shape for our dynamic body.
	b2PolygonShape bodyBox;
	bodyBox.SetAsBox(1.5f, 2.5f);
    //These are mid points for our 1m box
    
    b2PolygonShape wheelBox;
    wheelBox.SetAsBox(0.2, 0.5);
    
	// Define the dynamic body fixture.
	b2FixtureDef bodyFixtureDef;
	bodyFixtureDef.shape = &bodyBox;	
	bodyFixtureDef.density = 1.0f;
    
    b2FixtureDef wheelFixtureDef;
    wheelFixtureDef.shape = &wheelBox;
    wheelFixtureDef.density = 1;
    
    body->CreateFixture(&bodyFixtureDef);
    leftWheelBody->CreateFixture(&wheelFixtureDef);
    rightWheelBody->CreateFixture(&wheelFixtureDef);
    leftRearWheelBody->CreateFixture(&wheelFixtureDef);
    rightRearWheelBody->CreateFixture(&wheelFixtureDef);
    
    b2RevoluteJointDef leftWheelFrontJointDef;
    leftWheelFrontJointDef.Initialize(body, leftWheelBody, leftWheelBody->GetWorldCenter());
    leftWheelFrontJointDef.enableMotor = true;
    leftWheelFrontJointDef.maxMotorTorque = 100;
    
    b2RevoluteJointDef rightWheelFrontJointDef;
    rightWheelFrontJointDef.Initialize(body, rightWheelBody, rightWheelBody->GetWorldCenter());
    rightWheelFrontJointDef.enableMotor = true;
    rightWheelFrontJointDef.maxMotorTorque = 100;
    
    b2PrismaticJointDef leftRearWheelJointDef;
    leftRearWheelJointDef.Initialize(body, leftRearWheelBody, leftRearWheelBody->GetWorldCenter(), b2Vec2(1, 0));
    leftRearWheelJointDef.enableLimit = true;
    leftRearWheelJointDef.lowerTranslation = leftRearWheelJointDef.upperTranslation = 0;
    
    b2PrismaticJointDef rightRearWheelJointDef;
    rightRearWheelJointDef.Initialize(body, rightRearWheelBody, rightRearWheelBody->GetWorldCenter(), b2Vec2(1, 0));
    rightRearWheelJointDef.enableLimit = true;
    rightRearWheelJointDef.lowerTranslation = rightRearWheelJointDef.upperTranslation = 0;
    
    leftWheelFrontJoint = (b2RevoluteJoint*)(world->CreateJoint(&leftWheelFrontJointDef));
    rightWheelFrontJoint = (b2RevoluteJoint*)(world->CreateJoint(&rightWheelFrontJointDef));
    
    world->CreateJoint(&leftRearWheelJointDef);
    world->CreateJoint(&rightRearWheelJointDef);
}

-(void) updateSpriteWithDeltaTime:(ccTime) dt
{
    //[self killOrthogonalVelocity:leftWheelBody];
    //[self killOrthogonalVelocity:rightWheelBody];
    //[self killOrthogonalVelocity:leftWheelBody];
    //[self killOrthogonalVelocity:leftWheelBody];
    
    
    b2Vec2 lDirection = leftWheelBody->GetTransform().R.col2;
    lDirection*=40.0f;
    
    b2Vec2 rDirection = rightWheelBody->GetTransform().R.col2;
    rDirection*=40.0f;
    
    leftWheelBody->ApplyForce(lDirection, leftWheelBody->GetPosition());
    rightWheelBody->ApplyForce(rDirection, rightWheelBody->GetPosition());
    
    /*
     myWorld.Step(1/30, 8);
     killOrthogonalVelocity(leftWheel);
     killOrthogonalVelocity(rightWheel);
     killOrthogonalVelocity(leftRearWheel);
     killOrthogonalVelocity(rightRearWheel);
     
     //Driving
     var ldirection = leftWheel.GetXForm().R.col2.Copy();
     ldirection.Multiply(engineSpeed);
     var rdirection = rightWheel.GetXForm().R.col2.Copy()
     rdirection.Multiply(engineSpeed);
     leftWheel.ApplyForce(ldirection, leftWheel.GetPosition());
     rightWheel.ApplyForce(rdirection, rightWheel.GetPosition());
     
     //Steering
     var mspeed:Number;
     mspeed = steeringAngle - leftJoint.GetJointAngle();
     leftJoint.SetMotorSpeed(mspeed * STEER_SPEED);
     mspeed = steeringAngle - rightJoint.GetJointAngle();
     rightJoint.SetMotorSpeed(mspeed * STEER_SPEED);
     */
}

-(void) killOrthogonalVelocity:(b2Body*) body 
{
    b2Vec2 localPoint = b2Vec2_zero;
    b2Vec2 velocity = body->GetLinearVelocityFromLocalPoint(localPoint);
    
    b2Vec2 sidewaysAxis = body->GetTransform().R.col2;
    sidewaysAxis*=(b2Dot(velocity, sidewaysAxis));
    
    body->SetLinearVelocity(sidewaysAxis);
}



-(void) tick: (ccTime) dt
{
	//It is recommended that a fixed time step is used with Box2D for stability
	//of the simulation, however, we are using a variable time step here.
	//You need to make an informed choice, the following URL is useful
	//http://gafferongames.com/game-physics/fix-your-timestep/
	
	int32 velocityIterations = 8;
	int32 positionIterations = 1;
	
	// Instruct the world to perform a single step of simulation. It is
	// generally best to keep the time step and iterations fixed.
	world->Step(dt, velocityIterations, positionIterations);

	//Iterate over the bodies in the physics world
	for (b2Body* b = world->GetBodyList(); b; b = b->GetNext())
	{
		if (b->GetUserData() != NULL) {
			//Synchronize the AtlasSprites position and rotation with the corresponding body
			CCSprite *myActor = (CCSprite*)b->GetUserData();
			myActor.position = CGPointMake( b->GetPosition().x * PTM_RATIO, b->GetPosition().y * PTM_RATIO);
			myActor.rotation = -1 * CC_RADIANS_TO_DEGREES(b->GetAngle());
		}	
	}
    
    [self updateSpriteWithDeltaTime:dt];
}

- (void)ccTouchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
	//Add a new body/atlas sprite at the touched location
	for( UITouch *touch in touches ) {
		CGPoint location = [touch locationInView: [touch view]];
		
		location = [[CCDirector sharedDirector] convertToGL: location];
	}
}

- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{	
	static float prevX=0, prevY=0;
	
	//#define kFilterFactor 0.05f
#define kFilterFactor 1.0f	// don't use filter. the code is here just as an example
	
	float accelX = (float) acceleration.x * kFilterFactor + (1- kFilterFactor)*prevX;
	float accelY = (float) acceleration.y * kFilterFactor + (1- kFilterFactor)*prevY;
	
	prevX = accelX;
	prevY = accelY;
	
	// accelerometer values are in "Portrait" mode. Change them to Landscape left
	// multiply the gravity by 10
	b2Vec2 gravity( -accelY * 10, accelX * 10);
	
	world->SetGravity( gravity );
}

// on "dealloc" you need to release all your retained objects
- (void) dealloc
{
	// in case you have something to dealloc, do it in this method
	delete world;
	world = NULL;
	
	delete m_debugDraw;

	// don't forget to call "super dealloc"
	[super dealloc];
}
@end
