//
//  HelloWorldLayer.h
//  HighwayChaser
//
//  Created by Shervin Ghazazani on 12-03-03.
//  Copyright __MyCompanyName__ 2012. All rights reserved.
//


// When you import this file, you import all the cocos2d classes
#import "cocos2d.h"
#import "Box2D.h"
#import "GLES-Render.h"

// HelloWorldLayer
@interface HelloWorldLayer : CCLayer
{
	b2World* world;
	GLESDebugDraw *m_debugDraw;
    float engineSpeed;
    float steeringAngle;
    b2RevoluteJoint *leftWheelFrontJoint;
    b2RevoluteJoint *rightWheelFrontJoint;
    b2Body *leftWheelBody;
    b2Body *rightWheelBody;
    b2Body *leftRearWheelBody;
    b2Body *rightRearWheelBody;
    CCSprite *player;
}

// returns a CCScene that contains the HelloWorldLayer as the only child
+(CCScene *) scene;
// adds a new sprite at a given coordinate
-(void) addNewSpriteWithCoords:(CGPoint)p;
-(void) killOrthogonalVelocity:(b2Body*) body;

@end
