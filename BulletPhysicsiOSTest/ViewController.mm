//
//  ViewController.m
//  BulletPhysicsiOSTest
//
//  Created by Dae-Yeong Kim on 13. 2. 26..
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

#import "ViewController.h"

#include "btBulletDynamicsCommon.h"


#define BUFFER_OFFSET(i) ((char *)NULL + (i))

// Uniform index.
enum
{
    UNIFORM_MODELVIEWPROJECTION_MATRIX,
    UNIFORM_NORMAL_MATRIX,
    NUM_UNIFORMS
};
GLint uniforms[NUM_UNIFORMS];

// Attribute index.
enum
{
    ATTRIB_VERTEX,
    ATTRIB_NORMAL,
    NUM_ATTRIBUTES
};
/*
GLfloat gCubeVertexData[216] = 
{
    // Data layout for each line below is:
    // positionX, positionY, positionZ,     normalX, normalY, normalZ,
    0.5f, -0.5f, -0.5f,        1.0f, 0.0f, 0.0f,
    0.5f, 0.5f, -0.5f,         1.0f, 0.0f, 0.0f,
    0.5f, -0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
    0.5f, -0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
    0.5f, 0.5f, 0.5f,          1.0f, 0.0f, 0.0f,
    0.5f, 0.5f, -0.5f,         1.0f, 0.0f, 0.0f,
    
    0.5f, 0.5f, -0.5f,         0.0f, 1.0f, 0.0f,
    -0.5f, 0.5f, -0.5f,        0.0f, 1.0f, 0.0f,
    0.5f, 0.5f, 0.5f,          0.0f, 1.0f, 0.0f,
    0.5f, 0.5f, 0.5f,          0.0f, 1.0f, 0.0f,
    -0.5f, 0.5f, -0.5f,        0.0f, 1.0f, 0.0f,
    -0.5f, 0.5f, 0.5f,         0.0f, 1.0f, 0.0f,
    
    -0.5f, 0.5f, -0.5f,        -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,       -1.0f, 0.0f, 0.0f,
    -0.5f, 0.5f, 0.5f,         -1.0f, 0.0f, 0.0f,
    -0.5f, 0.5f, 0.5f,         -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,       -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, 0.5f,        -1.0f, 0.0f, 0.0f,
    
    -0.5f, -0.5f, -0.5f,       0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, -0.5f,        0.0f, -1.0f, 0.0f,
    -0.5f, -0.5f, 0.5f,        0.0f, -1.0f, 0.0f,
    -0.5f, -0.5f, 0.5f,        0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, -0.5f,        0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, 0.5f,         0.0f, -1.0f, 0.0f,
    
    0.5f, 0.5f, 0.5f,          0.0f, 0.0f, 1.0f,
    -0.5f, 0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    0.5f, -0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    0.5f, -0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    -0.5f, 0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f, 0.5f,        0.0f, 0.0f, 1.0f,
    
    0.5f, -0.5f, -0.5f,        0.0f, 0.0f, -1.0f,
    -0.5f, -0.5f, -0.5f,       0.0f, 0.0f, -1.0f,
    0.5f, 0.5f, -0.5f,         0.0f, 0.0f, -1.0f,
    0.5f, 0.5f, -0.5f,         0.0f, 0.0f, -1.0f,
    -0.5f, -0.5f, -0.5f,       0.0f, 0.0f, -1.0f,
    -0.5f, 0.5f, -0.5f,        0.0f, 0.0f, -1.0f
};
*/

double pastms = [[NSDate date] timeIntervalSince1970];
double generateInterval = 0.0f;

@interface ViewController () {
    GLuint _program;
    
    GLKMatrix4 _modelViewProjectionMatrix;
    GLKMatrix3 _normalMatrix;
    float _rotation;
    
    GLuint _vertexArray;
    GLuint _vertexBuffer;
    
    // BulletPhysics variables
    btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    btAlignedObjectArray<btCollisionShape*> collisionShapes;
}
@property (strong, nonatomic) EAGLContext *context;
@property (strong, nonatomic) GLKBaseEffect *effect;

@property (nonatomic, strong) UILabel *debug;

- (void)setupGL;
- (void)tearDownGL;

- (BOOL)loadShaders;
- (BOOL)compileShader:(GLuint *)shader type:(GLenum)type file:(NSString *)file;
- (BOOL)linkProgram:(GLuint)prog;
- (BOOL)validateProgram:(GLuint)prog;
@end

@implementation ViewController

@synthesize debug;

@synthesize context = _context;
@synthesize effect = _effect;

- (void)_debug_data:(float)fps box_num:(int)box_num {
    self.debug.text = [NSString stringWithFormat:@"fps : %f, box : %d", fps, box_num];
}
- (void)_debug_message:(NSString *)message {
    self.debug.text = @"test";
}

-(void)addBox:(btVector3)position {
    //create a dynamic rigidbody
    
    btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
    //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
    collisionShapes.push_back(colShape);
    
    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();
    
    btScalar	mass(1.f);
    
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        colShape->calculateLocalInertia(mass,localInertia);
    
    startTransform.setOrigin(position);
    
    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    dynamicsWorld->addRigidBody(body);
}
- (void)viewDidLoad
{
    [super viewDidLoad];
    
    // add debug label
    CGRect sumFrame = CGRectMake(50, 50, 300, 30);
    self.debug = [[UILabel alloc] initWithFrame:sumFrame];
    self.debug.text = @"debug";
    self.debug.font = [UIFont boldSystemFontOfSize:15];
    self.debug.textAlignment = UITextAlignmentLeft;
    self.debug.backgroundColor = [UIColor clearColor];
    [self.view addSubview:self.debug];
    

	///-----initialization_start-----
    
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();
    
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new	btCollisionDispatcher(collisionConfiguration);
    
	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();
    
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;
    
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);
    
	dynamicsWorld->setGravity(btVector3(0,-10,0));
    
    ///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
    
	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
//	btAlignedObjectArray<btCollisionShape*> collisionShapes;
    
	collisionShapes.push_back(groundShape);
    
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));
    
	{
		btScalar mass(0.);
        
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
        
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);
        
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
        
        
        
		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}
    
    
	{
        /*
		[self addBox:btVector3(2,10,0)];
		[self addBox:btVector3(2,15,0)];        
        [self addBox:btVector3(2,19,0)];
        [self addBox:btVector3(5,10,0)];
        [self addBox:btVector3(1,10,0)];
        [self addBox:btVector3(1,10,0)];
        [self addBox:btVector3(1,22,0)];
        [self addBox:btVector3(1,25,3)];
        [self addBox:btVector3(1,27,1)];
        [self addBox:btVector3(1,35,2)];
         */
	}
    

    
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];

    if (!self.context) {
        NSLog(@"Failed to create ES context");
    }
    
    GLKView *view = (GLKView *)self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    self.preferredFramesPerSecond = 60;
    
    [self setupGL];
}

- (void)viewDidUnload
{    
    [super viewDidUnload];
    
    ///-----cleanup_start-----
    int i;
	//remove the rigidbodies from the dynamics world and delete them
	for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
    
	//delete collision shapes
	for (int j=0;j<collisionShapes.size();j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}
    
	//delete dynamics world
	delete dynamicsWorld;
    
	//delete solver
	delete solver;
    
	//delete broadphase
	delete overlappingPairCache;
    
	//delete dispatcher
	delete dispatcher;
    
	delete collisionConfiguration;
    
	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	//collisionShapes.clear();
    
    [self tearDownGL];
    
    if ([EAGLContext currentContext] == self.context) {
        [EAGLContext setCurrentContext:nil];
    }
	self.context = nil;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Release any cached data, images, etc. that aren't in use.
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    // Return YES for supported orientations
    if ([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPhone) {
        return (interfaceOrientation != UIInterfaceOrientationPortraitUpsideDown);
    } else {
        return YES;
    }
}

- (void)setupGL
{
    [EAGLContext setCurrentContext:self.context];
    
    [self loadShaders];
    
    //http://developer.apple.com/library/ios/#documentation/GLkit/Reference/GLKEffectPropertyLight_ClassRef/Reference/Reference.html
    
    /*
     
     If the w component of the position is 0.0, the light is calculated using the directional light formula. The x, y, and z components of the vector specify the direction the light shines. The light is assumed to be infinitely far away; attenuation and spotlight properties are ignored.
     
     If the w component of the position is a non-zero value, the coordinates specify the position of the light in homogenous coordinates, and the light is either calculated as a point light or a spotlight, depending on the value of the spotCutoff property.
     
     */
    
    self.effect = [[GLKBaseEffect alloc] init];
    self.effect.light0.enabled = GL_TRUE;
    self.effect.light0.position = GLKVector4Make(1.0f, 1.0f, 1.0f, 0.0f);
    self.effect.light0.ambientColor = GLKVector4Make(0.6f, 0.4f, 0.6f, 0.0f);
    //self.effect.light0.constantAttenuation = 100.0f;
    //self.effect.light0.linearAttenuation = 100.0f;
    //self.effect.light0.quadraticAttenuation = 100.0f;
//    
//    self.effect.light0.diffuseColor = GLKVector4Make(0.6f, 0.4f, 0.6f, 1.0f);
    //self.effect.light0.spotDirection = GLKVector3Make(0.0f, -1.0f, 0.0f);
    //self.effect.light0.position = GLKVector4Make(2.0f, 1.0f, 3.0f, 0.0f);
    //self.effect.light0.diffuseColor = GLKVector4Make(0.6f, 0.4f, 0.6f, 1.0f);

    //self.effect.light1.enabled = GL_TRUE;
    //self.effect.light1.position = GLKVector4Make(-2.0f, -1.0f, 3.0f, 0.0f);
    //self.effect.light1.diffuseColor = GLKVector4Make(0.2f, 0.9f, 0.6f, 1.0f);
    
    glEnable(GL_DEPTH_TEST);
    
    glGenVertexArraysOES(1, &_vertexArray);
    glBindVertexArrayOES(_vertexArray);
    
    glGenBuffers(1, &_vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(gCubeVertexData), gCubeVertexData, GL_STATIC_DRAW);
    /*
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 24, BUFFER_OFFSET(0));
    glEnableVertexAttribArray(GLKVertexAttribNormal);
    glVertexAttribPointer(GLKVertexAttribNormal, 3, GL_FLOAT, GL_FALSE, 24, BUFFER_OFFSET(12));
     
    glBindVertexArrayOES(0);
    */

}

- (void)tearDownGL
{
    [EAGLContext setCurrentContext:self.context];
    
    glDeleteBuffers(1, &_vertexBuffer);
    glDeleteVertexArraysOES(1, &_vertexArray);
    
    self.effect = nil;
    
    if (_program) {
        glDeleteProgram(_program);
        _program = 0;
    }
}

#pragma mark - GLKView and GLKViewController delegate methods


- (void)update
{
    float aspect = fabsf(self.view.bounds.size.width / self.view.bounds.size.height);
    GLKMatrix4 projectionMatrix = GLKMatrix4MakePerspective(GLKMathDegreesToRadians(65.0f), aspect, 0.1f, 10000.0f);
    
    self.effect.transform.projectionMatrix = projectionMatrix;
    
    GLKMatrix4 baseModelViewMatrix = GLKMatrix4MakeTranslation(0.0f, -5.0f, -20.0f);
    baseModelViewMatrix = GLKMatrix4Rotate(baseModelViewMatrix, _rotation, 1.0f, 0.0f, 0.0f);
    
    // Compute the model view matrix for the object rendered with GLKit
    GLKMatrix4 modelViewMatrix = GLKMatrix4MakeTranslation(0.0f, 0.0f, 0.0f);
    //modelViewMatrix = GLKMatrix4Rotate(modelViewMatrix, _rotation, 1.0f, 0.0f, 0.0f);
    modelViewMatrix = GLKMatrix4Multiply(baseModelViewMatrix, modelViewMatrix);
    
    self.effect.transform.modelviewMatrix = modelViewMatrix;
    
    /*
    // Compute the model view matrix for the object rendered with ES2
    modelViewMatrix = GLKMatrix4MakeTranslation(0.0f, 0.0f, 1.5f);
    modelViewMatrix = GLKMatrix4Rotate(modelViewMatrix, _rotation, 1.0f, 1.0f, 1.0f);
    modelViewMatrix = GLKMatrix4Multiply(baseModelViewMatrix, modelViewMatrix);
    
    _normalMatrix = GLKMatrix3InvertAndTranspose(GLKMatrix4GetMatrix3(modelViewMatrix), NULL);
    
    _modelViewProjectionMatrix = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
    */
    //_rotation += self.timeSinceLastUpdate * 0.5f;
    _rotation = 0.4f;
}
- (void) getTriangleFromVertices:(btVector3*) vertex vertexList:(GLfloat*)vertexList {
    
    //return nil;
}

- (void) getCubeFromVertices:(btCollisionObject*) obj vertexList:(GLfloat*)vertexList {
    
    /*
     vertex index = 0, pos = 1.000000,1.000000,1.000000
     vertex index = 1, pos = -1.000000,1.000000,1.000000
     vertex index = 2, pos = 1.000000,-1.000000,1.000000
     vertex index = 3, pos = -1.000000,-1.000000,1.000000
     vertex index = 4, pos = 1.000000,1.000000,-1.000000
     vertex index = 5, pos = -1.000000,1.000000,-1.000000
     vertex index = 6, pos = 1.000000,-1.000000,-1.000000
     vertex index = 7, pos = -1.000000,-1.000000,-1.000000
     
     set triangles : clockwise
     
     1 : 6 2 4 0 4 2
     2 : 0 1 4 5 4 1
     3 : 3 7 1 5 1 7
     4 : 6 7 2 3 2 7
     5 : 2 3 0 1 0 3
     6 : 7 6 5 4 5 6
     
     */
    
    
    //DISABLE_SIMULATION
    btBoxShape* boxShape = dynamic_cast<btBoxShape *>(obj->getCollisionShape());    
    btRigidBody* body = btRigidBody::upcast(obj);
    if (!(body && body->getMotionState())) return;
    
    float invMass = body->getInvMass();    
    //NSLog(@"getInvMass = %f", invMass);    
    
    btVector3 vertex[8];
    int numofvertex = boxShape->getNumVertices();
    for(int i=0;i<numofvertex;i++){
        boxShape->getVertex(i, vertex[i]);
        /*
        btVector3 vertexBasis;
        boxShape->getVertex(i, vertexBasis);
        vertex[i] = trans*vertexBasis;
         */
        //vertex[i];
        //btVector3 vertexAfterTransform = trans*vertexBasis;
        
        //NSLog(@"vertex index = %d, pos = %f,%f,%f",i,vertexBasis.getX(),vertexBasis.getY(),vertexBasis.getZ());
        //NSLog(@"vertex index = %d, pos = %f,%f,%f",i,vertexAfterTransform.getX(),vertexAfterTransform.getY(),vertexAfterTransform.getZ());
    }
    
    // TODO : add 3d model objective file parser
    
    GLint vertexMapList[6][6] = {
        {6, 2, 4, 0, 4, 2},
        {0, 1, 4, 5, 4, 1},
        {3, 7, 1, 5, 1, 7},
        {6, 7, 2, 3, 2, 7},
        {2, 3, 0, 1, 0, 3},
        {7, 6, 5, 4, 5, 6}
    };
    
    GLfloat normalList[6][3] = {
        {1.0f, .0f, .0f},
        {.0f, 1.0f, .0f},
        {-1.0f, .0f, .0f},
        {.0f, -1.0f, .0f},
        {.0f, .0f, 1.0f},
        {.0f, .0f, -1.0f}
    };

    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);  
    
    for(int i=0;i<6;i++){
        //NSLog(@"plane index : %d", i);
        for(int j=0;j<6;j++){
            //vertexList
            int vertexIndex = vertexMapList[i][j];
            btVector3 curVertex = vertex[vertexIndex];
            btVector3 curNormal(normalList[i][0], normalList[i][1], normalList[i][2]);
            curVertex = trans*curVertex;
//            curVertex = trans*curVertex;
            if(invMass > 0.000001f) {
                curNormal = trans*curNormal;
            } else {
            //    curNormal *= 1000;
            }
            // get normal with vector dot equation
            
            //curNormal = trans*curNormal;
            
            //NSLog(@"vertex index : %d", vertexIndex);
            //int vertexListIndex;
            /*
            btVector3 vertexBasis;
            boxShape->getVertex(i, vertexBasis);
            vertex[i] = trans*vertexBasis;
            */
            
            //vertex
            vertexList[i*36 + j*6 + 0] = curVertex.getX();
            vertexList[i*36 + j*6 + 1] = curVertex.getY();
            vertexList[i*36 + j*6 + 2] = curVertex.getZ();
            //NSLog(@"vertex : %f, %f, %f", curVertex.getX(), curVertex.getY(), curVertex.getZ());
            
            //normal
            vertexList[i*36 + j*6 + 3] = curNormal.getX();
            vertexList[i*36 + j*6 + 4] = curNormal.getY();
            vertexList[i*36 + j*6 + 5] = curNormal.getZ();         
            //NSLog(@"normal : %f, %f, %f", curNormal.getX(), curNormal.getY(), curNormal.getZ());
            
        }
    }    
    //return nil;
}
int numofboxes = 0;
- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    
    //print fps
    double curms = [[NSDate date] timeIntervalSince1970];
    double intervalms = curms - pastms;
    //NSLog(@"fps = %f", 1 / intervalms);
    pastms = curms;
    
    [self _debug_data:(1 / intervalms) box_num:numofboxes];
    
    //check interval
    generateInterval += intervalms;
    
    int startx = -2;
    int endx = 10;
    
    if(generateInterval > 0.5f) {        
        double curx = (double)(arc4random() % endx) + startx;
        double cury = 30;
        double curz = 1;
        
        [self addBox:btVector3(curx,cury,curz)];
        
        NSLog(@"new box = %f %f %f",curx, cury, curz);
        numofboxes++;
        generateInterval = 0.0f;
        //NSLog(@"fps = %f", 1 / intervalms);
    }
    
    glClearColor(0.65f, 0.65f, 0.65f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    dynamicsWorld->stepSimulation(1.f/60.f,10);
    
    GLfloat gCubeVertexList[216];
    
    //print positions of all objects
    for (int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {            
            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);
            [self getCubeFromVertices:obj vertexList:gCubeVertexList];      
            
            glBufferData(GL_ARRAY_BUFFER, sizeof(gCubeVertexList), gCubeVertexList, GL_STATIC_DRAW);            
            
            glEnableVertexAttribArray(GLKVertexAttribPosition);
            glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 24, BUFFER_OFFSET(0));
            glEnableVertexAttribArray(GLKVertexAttribNormal);
            glVertexAttribPointer(GLKVertexAttribNormal, 3, GL_FLOAT, GL_FALSE, 24, BUFFER_OFFSET(12));
            
            glBindVertexArrayOES(_vertexArray);
            
            // Render the object with GLKit
            [self.effect prepareToDraw];
            
            // 36 = 216 / 6
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }    
        
    }
    
    /*
    // Render the object again with ES2
    glUseProgram(_program);
    
    glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, _modelViewProjectionMatrix.m);
    glUniformMatrix3fv(uniforms[UNIFORM_NORMAL_MATRIX], 1, 0, _normalMatrix.m);
    
    glDrawArrays(GL_TRIANGLES, 0, 36);
     */
}

#pragma mark -  OpenGL ES 2 shader compilation

- (BOOL)loadShaders
{
    GLuint vertShader, fragShader;
    NSString *vertShaderPathname, *fragShaderPathname;
    
    // Create shader program.
    _program = glCreateProgram();
    
    // Create and compile vertex shader.
    vertShaderPathname = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"];
    if (![self compileShader:&vertShader type:GL_VERTEX_SHADER file:vertShaderPathname]) {
        NSLog(@"Failed to compile vertex shader");
        return NO;
    }
    
    // Create and compile fragment shader.
    fragShaderPathname = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"];
    if (![self compileShader:&fragShader type:GL_FRAGMENT_SHADER file:fragShaderPathname]) {
        NSLog(@"Failed to compile fragment shader");
        return NO;
    }
    
    // Attach vertex shader to program.
    glAttachShader(_program, vertShader);
    
    // Attach fragment shader to program.
    glAttachShader(_program, fragShader);
    
    // Bind attribute locations.
    // This needs to be done prior to linking.
    glBindAttribLocation(_program, ATTRIB_VERTEX, "position");
    glBindAttribLocation(_program, ATTRIB_NORMAL, "normal");
    
    // Link program.
    if (![self linkProgram:_program]) {
        NSLog(@"Failed to link program: %d", _program);
        
        if (vertShader) {
            glDeleteShader(vertShader);
            vertShader = 0;
        }
        if (fragShader) {
            glDeleteShader(fragShader);
            fragShader = 0;
        }
        if (_program) {
            glDeleteProgram(_program);
            _program = 0;
        }
        
        return NO;
    }
    
    // Get uniform locations.
    uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = glGetUniformLocation(_program, "modelViewProjectionMatrix");
    uniforms[UNIFORM_NORMAL_MATRIX] = glGetUniformLocation(_program, "normalMatrix");
    
    // Release vertex and fragment shaders.
    if (vertShader) {
        glDetachShader(_program, vertShader);
        glDeleteShader(vertShader);
    }
    if (fragShader) {
        glDetachShader(_program, fragShader);
        glDeleteShader(fragShader);
    }
    
    return YES;
}

- (BOOL)compileShader:(GLuint *)shader type:(GLenum)type file:(NSString *)file
{
    GLint status;
    const GLchar *source;
    
    source = (GLchar *)[[NSString stringWithContentsOfFile:file encoding:NSUTF8StringEncoding error:nil] UTF8String];
    if (!source) {
        NSLog(@"Failed to load vertex shader");
        return NO;
    }
    
    *shader = glCreateShader(type);
    glShaderSource(*shader, 1, &source, NULL);
    glCompileShader(*shader);
    
#if defined(DEBUG)
    GLint logLength;
    glGetShaderiv(*shader, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0) {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetShaderInfoLog(*shader, logLength, &logLength, log);
        NSLog(@"Shader compile log:\n%s", log);
        free(log);
    }
#endif
    
    glGetShaderiv(*shader, GL_COMPILE_STATUS, &status);
    if (status == 0) {
        glDeleteShader(*shader);
        return NO;
    }
    
    return YES;
}

- (BOOL)linkProgram:(GLuint)prog
{
    GLint status;
    glLinkProgram(prog);
    
#if defined(DEBUG)
    GLint logLength;
    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0) {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetProgramInfoLog(prog, logLength, &logLength, log);
        NSLog(@"Program link log:\n%s", log);
        free(log);
    }
#endif
    
    glGetProgramiv(prog, GL_LINK_STATUS, &status);
    if (status == 0) {
        return NO;
    }
    
    return YES;
}

- (BOOL)validateProgram:(GLuint)prog
{
    GLint logLength, status;
    
    glValidateProgram(prog);
    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
    if (logLength > 0) {
        GLchar *log = (GLchar *)malloc(logLength);
        glGetProgramInfoLog(prog, logLength, &logLength, log);
        NSLog(@"Program validate log:\n%s", log);
        free(log);
    }
    
    glGetProgramiv(prog, GL_VALIDATE_STATUS, &status);
    if (status == 0) {
        return NO;
    }
    
    return YES;
}

@end
