 import processing.net.*;
 
 //ArrayList<Graph> graphs;
PShape base, shoulder, upArm, loArm, end;
float rotX, rotY;
float posX=1, posY=40, posZ=50;
float alpha, beta, gamma;
float F = 50;
float T = 70;
float millisOld, gTime, gSpeed = 4;

float[] Xsphere = new float[999];
float[] Ysphere = new float[999];
float[] Zsphere = new float[999];

float cubeSize = 125;
float cubePosX = 0, cubePosY = 0, cubePosZ = -6;

ArrayList<Float> draw_coords_x = new ArrayList<Float>();
ArrayList<Float> draw_coords_y = new ArrayList<Float>();
ArrayList<Boolean> draw_coords_flag = new ArrayList<Boolean>();

ArrayList<String> coords;
int index = 0;
int face = 0;

Robot robot_ = null;
boolean flag = true;

Server server;
Client client;

int dataIn; 

void IK(){
  // Calculate inverse kinematics for robot joints 
  float X = posX;
  float Y = posY;
  float Z = posZ;

  float L = sqrt(Y*Y+X*X);
  float dia = sqrt(Z*Z+L*L);

  alpha = PI/2-(atan2(L, Z)+acos((T*T-F*F-dia*dia)/(-2*F*dia)));
  beta = -PI+acos((dia*dia-T*T-F*F)/(-2*F*T));
  gamma = atan2(Y, X);
  //System.out.println("Inverse");
 }
 
void setTime(){
  gTime += ((float)millis()/1000 - millisOld)*(gSpeed/4);
  if(gTime >= 4)  gTime = 0;  
  millisOld = (float)millis()/1000;
}

void writePos(int j, int face){
  IK();
  setTime();

  float posDX = -draw_coords_x.get(j);
  float posDY = -draw_coords_y.get(j);
  
  switch(face){
    case 1:
      posY = posDX;
      posX = posDY;
      posZ = -cubeSize/2 + cubePosZ*2;
    break;
    case 2:
      posY = posDX;
      posZ = posDY + cubePosZ;
      posX = cubeSize/2;
    break;
    case 3:
      posX = posDX;
      posZ = posDY + cubePosZ;
      posY = cubeSize/2;
    break;
    case 4:
      posY = posDX;
      posZ = posDY + cubePosZ;
      posX = -cubeSize/2;
    break;
    case 5:
      posX = posDX;
      posZ = posDY + cubePosZ;
      posY = -cubeSize/2;
    break;
    case 6:
      println("Nel");
    break;
  }
}

void linearInterpolation(ArrayList<FloatList> coords, int stepsBetweenPoints){
  float x0, y0, x1, y1, next_x, next_y;
  for (int i = 0; i < robot_.num_coords.size(); i++){
    x0 = coords.get(i).get(0);
    y0 = coords.get(i).get(1);
    x1 = coords.get(i).get(2);
    y1 = coords.get(i).get(3);
    for (int j = 0; j < stepsBetweenPoints; j++){
      draw_coords_x.add(x0 + (x1 - x0) * j / stepsBetweenPoints);
      draw_coords_y.add(y0 + (y1 - y0) * j / stepsBetweenPoints);
      draw_coords_flag.add(true);
    }
    if (i < coords.size() - 1){
      next_x = coords.get(i+1).get(0);
      next_y = coords.get(i+1).get(1);
      if (next_x != x1 || next_y != y1){
        for (int j = 0; j < stepsBetweenPoints; j++){
          draw_coords_x.add(x1 + (next_x - x1) * j / stepsBetweenPoints);
          draw_coords_y.add(y1 + (next_y - y1) * j / stepsBetweenPoints);
          draw_coords_flag.add(false);
        }
      }
    }
  }
}


void setup(){
  size(1200, 800, OPENGL);
  loadShapes();
  robot_ = new Robot();
  coords = new ArrayList<String>();

   // Start a server on port 65432
  server = new Server(this, 65432);
  println("Server started on port 65432");
}

void draw(){
   // Wait to recieve information from socket
   //if (myClient.available() > 0) { 
   //   dataIn = myClient.read(); 
   //} 
   //if (flag){//(client = server.available()) != null && flag) {
    //if(flag){
    if ((client = server.available()) != null && flag) {     //System.out.println("------------------------------");
     //String data = "WALTER WHITE;3;", phrase="";//
     String data = client.readString(), phrase = "";
     println("Received: " + data);
     
     for (int i = 0; i < data.length(); i++){
       if (data.charAt(i) == ';'){
         face =  data.charAt(i+1) - '0';
         break;
       }
       phrase += data.charAt(i);
     }
     index = 0;
     robot_.cleanCords();
     robot_.writePhrase(phrase);
     
      draw_coords_x.clear();
      draw_coords_y.clear();
      draw_coords_flag.clear();
     //for (String ss : robot_.coords) System.out.println(ss + " "); 
     //System.out.println("------------------------------");
     flag = false;
     for (int i = 0; i < robot_.num_coords.size(); i++){
       for (int j = 0; j < robot_.num_coords.get(i).size(); j++){
         System.out.print(robot_.num_coords.get(i).get(j));
         System.out.print(" ");
       }
        System.out.println();
     }
     // wait 10000 ms
     linearInterpolation(robot_.num_coords, 5);
     Xsphere = new float[draw_coords_x.size()];
      Ysphere = new float[draw_coords_x.size()];
      Zsphere = new float[draw_coords_x.size()];
   } else if (draw_coords_x.size() == 0){
    return;
   }

  if (index >= draw_coords_x.size() - 1 && flag == false){
    System.out.println("End of the movement");
    flag = true;
  }

  writePos(index, face);

  if (flag == false){
    index++;
  }



   background(255);
   smooth();
   lights(); 
   directionalLight(51, 102, 126, -1, 0, 0);
    
    for (int i=0; i< Xsphere.length - 1; i++) {
    Xsphere[i] = Xsphere[i + 1];
    Ysphere[i] = Ysphere[i + 1];
    Zsphere[i] = Zsphere[i + 1];
    }
      
    Xsphere[Xsphere.length - 1] = posX;
    Ysphere[Ysphere.length - 1] = posY;
    Zsphere[Zsphere.length - 1] = posZ;
   
   noStroke();
   
   translate(width/2,height/2);
   rotateX(rotX);
   rotateY(-rotY);
   scale(-2);
   
   for (int i=0; i < Xsphere.length; i++) {
     pushMatrix();
     translate(-Ysphere[i], -Zsphere[i]-11, -Xsphere[i]);
     fill (#D003FF, 200);
     int sphereSize = 2;
     int ball_index = i - (draw_coords_x.size() - index);
     if (ball_index<0){
      ball_index = draw_coords_x.size() + ball_index;
     }
     if (draw_coords_flag.get(ball_index)==false){
      sphereSize = 0;
     }
     sphere(sphereSize);
     popMatrix();
    }
    
   stroke(#000000);
   noFill();
   translate(-cubePosY, -cubePosZ, -cubePosX);
   box(cubeSize); 
   
   noStroke();
    
   fill(#FFE308);  
   translate(cubePosY,-40+cubePosZ,cubePosX);   
     shape(base);
     
   translate(0, 4, 0);
   rotateY(gamma);
     shape(shoulder);
      
   translate(0, 25, 0);
   rotateY(PI);
   rotateX(alpha);
     shape(upArm);
      
   translate(0, 0, 50);
   rotateY(PI);
   rotateX(beta);
     shape(loArm);
      
   translate(0, 0, -50);
   rotateY(PI);
     shape(end);
}

void mouseDragged(){
    rotY -= (mouseX - pmouseX) * 0.01;
    rotX -= (mouseY - pmouseY) * 0.01;
}

 void loadShapes(){
    base = loadShape("r5.obj");
    shoulder = loadShape("r1.obj");
    upArm = loadShape("r2.obj");
    loArm = loadShape("r3.obj");
    end = loadShape("r4.obj");
    
    shoulder.disableStyle();
    upArm.disableStyle();
    loArm.disableStyle(); 
  }
