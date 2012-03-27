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
		
		// This will speed up the physics simulation
		// Construct a world object, which will hold and simulate the rigid bodies.
		world = new b2World(gravity);
		
		world->SetContinuousPhysics(true);
		
		// Debug Draw functions
		m_debugDraw = new GLESDebugDraw( PTM_RATIO );
		world->SetDebugDraw(m_debugDraw);
		
		uint32 flags = 0;
		flags += b2Draw::e_shapeBit;
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
		wallBody = world->CreateBody(&groundBodyDef);
        
        /*
        b2PolygonShape edgeShape;
		b
		// left
		edgeShape.SetAsEdge(b2Vec2(0,0), b2Vec2(0,screenSize.height/PTM_RATIO * 3));
		leftEdge = wallBody->CreateFixture(&edgeShape,0);
        		
		// right
		edgeShape.SetAsEdge(b2Vec2(screenSize.width/PTM_RATIO,0), b2Vec2(screenSize.width/PTM_RATIO,(screenSize.height/PTM_RATIO) * 3));
		rightEdge =  wallBody->CreateFixture(&edgeShape,0);
        */
         
        steeringAngle = 0;
        
		//Set up sprite batch node
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
	player = [CCSprite spriteWithBatchNode:batch rect:CGRectMake(32 * idx,32 * idy,32,32)];
	[batch addChild:player];
	
	player.position = ccp( p.x, p.y);
	
	// Define the dynamic body.
	//Set up a 1m squared box in the physics world
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
	bodyDef.userData = player;
	carBody = world->CreateBody(&bodyDef);
    carBody->ResetMassData();
    
    b2BodyDef leftWheelDef;
    leftWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    leftWheelDef.position += b2Vec2(-0.5f, -0.90f);
    leftWheelDef.type = b2_dynamicBody;
    leftWheelBody = world->CreateBody(&leftWheelDef);
    leftWheelBody->ResetMassData();
    
    b2BodyDef rightWheelDef;
    rightWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    rightWheelDef.position += b2Vec2(0.5f, -0.90f);
    rightWheelDef.type = b2_dynamicBody;
    rightWheelBody = world->CreateBody(&rightWheelDef);
    rightWheelBody->ResetMassData();
    
    b2BodyDef leftRearWheelDef;
    leftRearWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    leftRearWheelDef.position += b2Vec2(-0.5f,0.90f);
    leftRearWheelDef.type = b2_dynamicBody;
    leftRearWheelBody = world->CreateBody(&leftRearWheelDef);
    leftRearWheelBody->ResetMassData();
	
    b2BodyDef rightRearWheelDef;
    rightRearWheelDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    rightRearWheelDef.position += b2Vec2(0.5f, 0.9f);
    rightRearWheelDef.type = b2_dynamicBody;
    rightRearWheelBody = world->CreateBody(&rightRearWheelDef);
    rightRearWheelBody->ResetMassData();
    
	// Define another box shape for our dynamic body.
	b2PolygonShape bodyBox;
	bodyBox.SetAsBox(0.5f, 1.25f);
    //These are mid points for our 1m box
    
    b2PolygonShape wheelBox;
    wheelBox.SetAsBox(0.1, 0.25);
    
	// Define the dynamic body fixture.
	b2FixtureDef bodyFixtureDef;
	bodyFixtureDef.shape = &bodyBox;	
	bodyFixtureDef.density = 1.0f;
    bodyFixtureDef.friction = 1;
    
    b2FixtureDef wheelFixtureDef;
    wheelFixtureDef.shape = &wheelBox;
    wheelFixtureDef.density = 1;
    wheelFixtureDef.friction = 1;
    
    carBody->CreateFixture(&bodyFixtureDef);
    leftWheelBody->CreateFixture(&wheelFixtureDef);
    rightWheelBody->CreateFixture(&wheelFixtureDef);
    leftRearWheelBody->CreateFixture(&wheelFixtureDef);
    rightRearWheelBody->CreateFixture(&wheelFixtureDef);
    
    carBody->ResetMassData();
    leftWheelBody->ResetMassData();
    rightWheelBody->ResetMassData();
    leftRearWheelBody->ResetMassData();
    rightRearWheelBody->ResetMassData();
    
    b2RevoluteJointDef leftWheelFrontJointDef;
    leftWheelFrontJointDef.Initialize(carBody, leftWheelBody, leftWheelBody->GetWorldCenter());
    leftWheelFrontJointDef.enableMotor = true;
    leftWheelFrontJointDef.maxMotorTorque = 100;
    
    b2RevoluteJointDef rightWheelFrontJointDef;
    rightWheelFrontJointDef.Initialize(carBody, rightWheelBody, rightWheelBody->GetWorldCenter());
    rightWheelFrontJointDef.enableMotor = true;
    rightWheelFrontJointDef.maxMotorTorque = 100;
    
    b2PrismaticJointDef leftRearWheelJointDef;
    leftRearWheelJointDef.Initialize(carBody, leftRearWheelBody, leftRearWheelBody->GetWorldCenter(), b2Vec2(1, 0));
    leftRearWheelJointDef.enableLimit = true;
    leftRearWheelJointDef.lowerTranslation = leftRearWheelJointDef.upperTranslation = 0;
    
    b2PrismaticJointDef rightRearWheelJointDef;
    rightRearWheelJointDef.Initialize(carBody, rightRearWheelBody, rightRearWheelBody->GetWorldCenter(), b2Vec2(1, 0));
    rightRearWheelJointDef.enableLimit = true;
    rightRearWheelJointDef.lowerTranslation = rightRearWheelJointDef.upperTranslation = 0;
    
    leftWheelFrontJoint = (b2RevoluteJoint*)(world->CreateJoint(&leftWheelFrontJointDef));
    rightWheelFrontJoint = (b2RevoluteJoint*)(world->CreateJoint(&rightWheelFrontJointDef));
    
    world->CreateJoint(&leftRearWheelJointDef);
    world->CreateJoint(&rightRearWheelJointDef);
}

-(void) updateSpriteWithDeltaTime:(ccTime) dt
{
    [self killOrthogonalVelocityWithB2Body:leftWheelBody];
    [self killOrthogonalVelocityWithB2Body:rightWheelBody];
    [self killOrthogonalVelocityWithB2Body:leftRearWheelBody];
    [self killOrthogonalVelocityWithB2Body:rightRearWheelBody];
    
    b2Vec2 lDirection = leftWheelBody->GetTransform().q.GetYAxis();
    lDirection*=-4.0f;
    
    b2Vec2 rDirection = rightWheelBody->GetTransform().q.GetYAxis();
    rDirection*=-4.0f;
    
    leftWheelBody->ApplyForce(lDirection, leftWheelBody->GetPosition());
    rightWheelBody->ApplyForce(rDirection, rightWheelBody->GetPosition());
    
    //Steering
    float mSpeed;
    
    mSpeed = steeringAngle - leftWheelFrontJoint->GetJointAngle();
    leftWheelFrontJoint->SetMotorSpeed(mSpeed * 1.5f);
    
    mSpeed = steeringAngle - rightWheelFrontJoint->GetJointAngle();
    rightWheelFrontJoint->SetMotorSpeed(mSpeed * 1.5f);
}

-(void) killOrthogonalVelocityWithB2Body:(b2Body*) body
{
    b2Vec2 killVelocityVector = body->GetLinearVelocity();
    b2Vec2 localPoint = b2Vec2_zero;
    b2Vec2 velocity = body->GetLinearVelocityFromLocalPoint(localPoint);
    
    b2Vec2 sidewaysAxis = body->GetTransform().q.GetYAxis();
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
    
    CGSize winSize = [[CCDirector sharedDirector] winSize];
    int x = MAX(player.position.x, winSize.width * 0.5);
    int y = MAX(player.position.y, winSize.height * 0.5);
    CGPoint actualPosition = ccp(x, y);
    
    CGPoint centerOfView = ccp(winSize.width/2, winSize.height/2);
    CGPoint viewPoint = ccpSub(centerOfView,actualPosition);
    self.position = viewPoint;
    
    CCLOG(@"%@",NSStringFromCGPoint(viewPoint));
    
    /*
    b2PolygonShape *testShape;

    for(b2Fixture *fixture = wallBody->GetFixtureList();fixture;fixture = fixture->GetNext()) {
        b2Shape *fixtureShape = fixture->GetShape();
        
        if (fixtureShape->GetType() == 1) {
            b2PolygonShape *polygonShape = (b2PolygonShape*)fixtureShape;
            b2Vec2 *shapeVertices = polygonShape->m_vertices;
            int32 verticesCount = polygonShape->m_vertexCount;
            
            if (verticesCount > 0 && verticesCount < 8) {
                CGFloat shapeVerticesY = shapeVertices[verticesCount].y;
                if (shapeVerticesY < -viewPoint.y) {
                    CCLOG(@"Destroying Fixture!");
                    wallBody->DestroyFixture(fixture);
                }
            }
        }
    } 
    
    
    // left
    b2PolygonShape newEdgeShape;
    newEdgeShape.SetAsEdge(b2Vec2(0,-viewPoint.y/PTM_RATIO), b2Vec2(0,(winSize.height + (-viewPoint.y))/PTM_RATIO));
    leftEdge = wallBody->CreateFixture(&newEdgeShape,wallBody->GetAngle());
    
    // right
    newEdgeShape.SetAsEdge(b2Vec2(winSize.width/PTM_RATIO,-viewPoint.y), b2Vec2(winSize.width/PTM_RATIO,(winSize.height + (-viewPoint.y))/PTM_RATIO));
    rightEdge =  wallBody->CreateFixture(&newEdgeShape,wallBody->GetAngle());
    */
     
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
	steeringAngle = -accelX * 2;
	prevX = accelX;
	prevY = accelY;
    
    //CCLOG(@"Accel Y:%f",accelY);
	
	// accelerometer values are in "Portrait" mode. Change them to Landscape left
	// multiply the gravity by 10
	//b2Vec2 gravity( -accelY * 50, accelX * 50);
	
	//world->SetGravity( gravity );
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
