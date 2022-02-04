/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import java.io.FileReader;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18; //walls
FBox b1, b2; 
FBlob bl1;
FBox l1; 
FBox cover;

/* define start and stop button */
FCircle           c1, c2;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
 
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();

  /* Set maze barriers */
  w1 = new FBox(3, 0.2);
  w1.setPosition(3.5,2); w1.setFill(0); w1.setNoStroke(); w1.setStaticBody(true);
  world.add(w1);
  
  w2 = new FBox(0.2, 2);
  w2.setPosition(2, 2.9); w2.setFill(0); w2.setNoStroke(); w2.setStaticBody(true);
  world.add(w2);
  
  w3 = new FBox(8, 0.2);
  w3.setPosition(6, 3.8); w3.setFill(0); w3.setNoStroke(); w3.setStaticBody(true);
  world.add(w3);
  
  w4 = new FBox(5, 0.2);
  w4.setPosition(2.5, 6); w4.setFill(0); w4.setNoStroke(); w4.setStaticBody(true);
  world.add(w4);
  
  w5 = new FBox(5, 0.2);
  w5.setPosition(4.5, 7.5); w5.setFill(0); w5.setNoStroke(); w5.setStaticBody(true);
  world.add(w5);
  
  w6 = new FBox(0.2, 1.7);
  w6.setPosition(7, 6.75); w6.setFill(0); w6.setNoStroke(); w6.setStaticBody(true);
  world.add(w6);
  
  w7 = new FBox(3, 0.2);
  w7.setPosition(8.5, 6); w7.setFill(0); w7.setNoStroke(); w7.setStaticBody(true);
  world.add(w7);
  
  w8 = new FBox(0.2, 4);
  w8.setPosition(10, 4.1); w8.setFill(0); w8.setNoStroke(); w8.setStaticBody(true);
  world.add(w8);
  
  w9 = new FBox(0.2, 3.5);
  w9.setPosition(7.5, 2); w9.setFill(0); w9.setNoStroke(); w9.setStaticBody(true);
  world.add(w9);
  
  w10 = new FBox(0.2, 1.7);
  w10.setPosition(9,8.4); w10.setFill(0, 255, 0); w10.setNoStroke(); w10.setStaticBody(true);
  world.add(w10);
  
  w11 = new FBox(0.2, 2);
  w11.setPosition(12,6.5); w11.setFill(0); w11.setNoStroke(); w11.setStaticBody(true);
  world.add(w11);

  w12 = new FBox(6.5, 0.2);
  w12.setPosition(15.15, 7.5); w12.setFill(0); w12.setNoStroke(); w12.setStaticBody(true);
  world.add(w12);
  
  w13= new FBox(0.2, 4);
  w13.setPosition(18.3,5.5); w13.setFill(0, 255, 0); w13.setNoStroke(); w13.setStaticBody(true);
  world.add(w13);
  
  w14= new FBox(4, 0.2);
  w14.setPosition(16.4, 3.5); w14.setFill(0); w14.setNoStroke(); w14.setStaticBody(true);
  world.add(w14);
  
  w15 = new FBox(0.2, 1.5);
  w15.setPosition(14.5, 2.8); w15.setFill(0); w15.setNoStroke(); w15.setStaticBody(true);
  world.add(w15);
  
  w16= new FBox(7, 0.2);
  w16.setPosition(17.9, 2); w16.setFill(0); w16.setNoStroke(); w16.setStaticBody(true);
  world.add(w16);
  
  w17= new FBox(4, 0.2);
  w17.setPosition(23, 3.5); w17.setFill(0); w17.setNoStroke(); w17.setStaticBody(true);
  world.add(w17);
  
  w18= new FBox(0.2,4);
  w18.setPosition(21, 5.4); w18.setFill(0, 255, 0); w18.setNoStroke(); w18.setStaticBody(true);
  world.add(w18);
  
  b1 = new FBox(2, 1.65);
  b1.setPosition(16, 8.2); b1.setFill(0); b1.setNoStroke(); b1.setStaticBody(false);
  world.add(b1);
  
  b2 = new FBox(4, 1);
  b2.setPosition(6, 5); b2.setFill(0); b2.setNoStroke(); b2.setStaticBody(false);
  world.add(b2);
  
  bl1 = new FBlob();
  float sca = random(4,5); sca = sca/2;
  bl1.setAsCircle(22, 8, sca, 15);
  bl1.setStroke(255,255,255);
  bl1.setStrokeWeight(2);
  bl1.setFill(255);
  bl1.setFriction(0);
  bl1.setDensity(18);
  world.add(bl1);
  

  /* Set viscous layer */
  l1                  = new FBox(27,4);
  l1.setPosition(24.5/2,8.5);
  l1.setFill(150,150,255,80);
  l1.setDensity(100);
  l1.setSensor(true);
  l1.setNoStroke();
  l1.setStatic(true);
  world.add(l1);


  //cover = new FBox(25, 10);
  //cover.setPosition(12.5, 5);
  //cover.setFill(255,255,255);
  //cover.setStaticBody(true);
  //world.add(cover);
  
    /* Start Button */
  c1                  = new FCircle(1.0); // diameter is 2
  c1.setPosition(edgeTopLeftX+6.6, edgeTopLeftY+worldHeight/2.0-3.5);
  c1.setFill(0, 255, 0);
  c1.setStaticBody(true);
  world.add(c1);
  
    /* Finish Button */
  c2                  = new FCircle(1.0);
  c2.setPosition(worldWidth-1.5, edgeTopLeftY+worldHeight/2.0-0.6);
  c2.setFill(200,0,0);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  

 
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 18);
 
    if(gameStart){
      fill(0, 0, 0);
      textAlign(CENTER);
      text("Push the ball to the red circle.", width/2, 50);
      textAlign(CENTER);
      text("Touching any green wall will reset the game.", width/2, 70);
      w1.setFill(0,0,0); w2.setFill(0,0,0); w3.setFill(0,0,0);
      w4.setFill(0,0,0); w5.setFill(0,0,0); w6.setFill(0,0,0);
      w7.setFill(0,0,0); w8.setFill(0,0,0); w9.setFill(0,0,0);
      w10.setFill(0,255,0); w11.setFill(0,0,0); w12.setFill(0,0,0);
      w13.setFill(0,255,0); w14.setFill(0,0,0); w15.setFill(0,0,0);
      w16.setFill(0,0,0); w17.setFill(0,0,0);w18.setFill(0,255,0);
      c2.setFill(200,0,0); c2.setStroke(0,0,0); 
      bl1.setStroke(0,0,0); l1.setFill(150,150,255,80);
      b1.setFill(0,0,0); b2.setFill(0,0,0);
    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch the green circle to start the maze", width/2, 60);
      w1.setFill(0,0,0,0); w2.setFill(0,0,0,0); w3.setFill(0,0,0,0);
      w4.setFill(0,0,0,0); w5.setFill(0,0,0,0); w6.setFill(0,0,0,0);
      w7.setFill(0,0,0,0); w8.setFill(0,0,0,0); w9.setFill(0,0,0,0);
      w10.setFill(0,255,0,0); w11.setFill(0,0,0,0); w12.setFill(0,0,0,0);
      w13.setFill(0,255,0,0); w14.setFill(0,0,0,0); w15.setFill(0,0,0,0);
      w16.setFill(0,0,0,0); w17.setFill(0,0,0,0);w18.setFill(0,255,0,0);
      c2.setFill(200,0,0,0); c2.setStroke(200,0,0,0);bl1.setStroke(0,0,0,0); 
      l1.setFill(150,150,255,0);
      b1.setFill(0,0,0,0); b2.setFill(0,0,0,0);
    }
  
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if (s.h_avatar.isTouchingBody(c1)){
      gameStart = true;
      s.h_avatar.setSensor(false);
    }
    
    if (gameStart && (s.h_avatar.isTouchingBody(w10)
      || s.h_avatar.isTouchingBody(w13) || s.h_avatar.isTouchingBody(w18))){
      b1.setPosition(16, 8.4);
      b2.setPosition(6, 5.5);
      s.h_avatar.setPosition(edgeTopLeftX+6.6, edgeTopLeftY+worldHeight/2.0-1.5);
      float sca = random(4,5); sca = sca/2;
      bl1.setAsCircle(22, 8, sca, 15);
      s.h_avatar.setSensor(false);
      
      //widgetOne.device_read_data();
    
      //angles.set(widgetOne.get_device_angles()); 
      //posEE.set(widgetOne.get_device_position(angles.array()));
      //posEE.set(posEE.copy().mult(200));  
      
      //  s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
      //s.updateCouplingForce();
    }
  
    if (s.h_avatar.isTouchingBody(c2)){
      gameStart = false;
      s.h_avatar.setSensor(true);
    }
  
  
  
    /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(l1)){
      s.h_avatar.setDamping(700);
    }
    else{
      s.h_avatar.setDamping(10); 
    }

    s.h_avatar.addForce(0, gravityAcceleration);
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
