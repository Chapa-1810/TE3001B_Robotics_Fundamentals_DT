 import processing.net.*;
 
 //ArrayList<Graph> graphs;
PShape base, shoulder, upArm, loArm, end;
float rotX, rotY;
float posX=1, posY=50, posZ=50;
float alpha, beta, gamma;
float F = 50;
float T = 70;
float millisOld, gTime, gSpeed = 4;

float[] Xsphere = new float[99];
float[] Ysphere = new float[99];
float[] Zsphere = new float[99];


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

void writePos(ArrayList<String> coords, int j, int face){
  if (j >= coords.size() - 1){
    flag = true;
    return;
  } else if (coords.get(j) == " ") return;
  
  
  IK();
  setTime();
  String curr_coords = coords.get(j);
  
  float[] p_coords = {0,0,0,0};
  String dummy = "";
  int f_index = 0;
  for (int i = 0; i < curr_coords.length(); i++){
    if (curr_coords.charAt(i) == ',' || curr_coords.charAt(i) == ';'){
      //System.out.print(dummy + ";  ");
      p_coords[f_index++] = Float.parseFloat(dummy);
      dummy = "";
      continue;
    }
    
    dummy += curr_coords.charAt(i);
  }
  
  //System.out.println("---------");
  
  float curr_x = p_coords[0], curr_y = p_coords[1], next_x = p_coords[2], next_y = p_coords[3];
  
  posX = curr_x * 20;
  posZ = -curr_y * 10;
  
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
   if ((client = server.available()) != null && flag) {
     //System.out.println("------------------------------");
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
     robot_.coords.clear();
     robot_.writePhrase(phrase);
     //for (String ss : robot_.coords) System.out.println(ss + " "); 
     //System.out.println("------------------------------");
     flag = false;
   }
   if (robot_.coords.size() > 0)
     writePos(robot_.coords, index++, face);
   
   
   background(32);
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
     fill (#D003FF, 25);
     sphere (float(i) / 20);
     popMatrix();
    }
    
   fill(#FFE308);  
   translate(0,-40,0);   
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
