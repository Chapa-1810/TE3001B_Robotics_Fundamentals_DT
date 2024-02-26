class Robot {
  PShape base, shoulder, upArm, loArm, end;
  float alpha, beta, gamma;
  float posX=1, posY=50, posZ=50;
  float rotX, rotY;
  
  float F = 50;
  float T = 70;
  
  float millisOld, gTime, gSpeed = 4;
  
  float Xsphere;
  float Ysphere;
  float Zsphere;
  
  ArrayList<Graph> graphs;
  int start_node = 6;
  
  String path = "C:/Users/alexi/OneDrive/Escritorio/Semestre 6/TE3001B_Robotics_Fundamentals_DT/letters.json";
  JSONObject json;
  
  int scale, x_scaling_factor, face;
  
  public Robot(){
    this.loadRobot();
  }
  
  public void loadRobot(){
     this.loadGraphs(path);
     this.loadShapes();
   }
  
  public void IK(){
    // Calculate inverse kinematics for robot joints 
    float X = posX;
    float Y = posY;
    float Z = posZ;
  
    float L = sqrt(Y*Y+X*X);
    float dia = sqrt(Z*Z+L*L);
  
    alpha = PI/2-(atan2(L, Z)+acos((T*T-F*F-dia*dia)/(-2*F*dia)));
    beta = -PI+acos((dia*dia-T*T-F*F)/(-2*F*T));
    gamma = atan2(Y, X);
   }
   
   public void setTime(){
      gTime += ((float)millis()/1000 - millisOld)*(gSpeed/4);
      if(gTime >= 4)  gTime = 0;  
      millisOld = (float)millis()/1000;
    }
    
    // Function that on right click rotates view of arm
    public void mouseDragged(){
      if (mousePressed && (mouseButton == RIGHT)){
        rotY -= (mouseX - pmouseX) * 0.01;
        rotX -= (mouseY - pmouseY) * 0.01;
      }
    }
    
    public void writePos(int curr_x, int curr_y, int next_x, int next_y, int face){
      IK();
      setTime();
      // Map mouse coordinates to window size, changing origin to center
      
      // SWITCH 
      // if prevx <= curr_x
      posX =+ scale / 10;
       // if prevy <= curr_y
      posY =+ scale / 10;
      // if face no se que 
      posZ =+ scale / 10;
    }
   
   public void loadGraphs(String path){
    graphs = new ArrayList<Graph>(26);
    json = loadJSONObject(path);
    
    JSONArray json_arr = json.getJSONArray("letters");
    
    for (int i = 0; i < json_arr.size(); i++) {
      JSONObject letter = json_arr.getJSONObject(i); 
      
      String key_ = str(char(i + 'A'));
      String encoded = letter.getString(key_); 
      
      System.out.print(encoded + " " );
  
      Graph key_graph = new Graph();
      
      key_graph.BuildGraphFromEncoded(encoded);
      System.out.print(key_ + " " );
      graphs.add(key_graph);
  
      System.out.println(graphs.size());
    }
   }
   
   // INPUT: Indice que representa el caracter que queremos dibujar; Cordenadas de inicio (x,y)
   // Se crea vector de nodos visitados
   // Se reseta el set que sera modificado
   // Se callea traverse DFS con sus parametros
   public void DFS(int index, int start_x, int start_y){
    boolean visited[] = {false, false, false, false, false, false, false, false, false}; 
     
    graphs.get(index).resetCopySet();
    traverseDFS(graphs.get(index), start_node , visited, start_x, start_y);
  }
  
  // INPUT: Grafo por el que nos movemos (LETRA QUE VAMOS A DIBUJAR), NODO DE INICIO, VECTOR DE VISITADOS, CORDENADAS ACTUALES DE (X,Y)
  public void traverseDFS(Graph curr_graph, int start, boolean[] visited, int curr_x, int curr_y) {
    visited[start] = true;
    for (int i = 0; i < curr_graph.nodes.get(start).neighbours.size(); i++) { // Se itera sobre una lista de adjacencias para todos los vecinos del nodo
      int next = curr_graph.nodes.get(start).neighbours.get(i);
      
      int sm_node = next < start ? next : start;
      int bg_node = next > start ? next : start;
      
      if (!visited[next] || curr_graph.copy_set.contains(sm_node + ", " + bg_node)) { // Checamos si fue visitado o si le conexion entre nodos ya se hizo (SET<String>)
        curr_graph.copy_set.remove(sm_node + ", " + bg_node); // ELIMINAMOS CONEXION EN EL SET
        int diff = start - next;
        System.out.print(" " + start + " -> " + next + " ");
        int prev_x = curr_x, prev_y = curr_y;
        
        // Calculo logico de nuevas cordenadas (x,y) para el siguiente nodo
        if (abs(diff) == 3) curr_y += (abs(diff) / diff) * scale;
        else if (abs(diff) == 1) curr_x += (abs(diff) / diff) * scale/x_scaling_factor;
        else if (abs(diff) == 4) {
          if (start < next) {
            curr_y+=scale;
            curr_x+=scale/x_scaling_factor;
          } else {
            curr_y-=scale;
            curr_x-=scale/x_scaling_factor;
          }
        } else {
          if (start < next) {
            curr_y+=scale;
            curr_x-=scale/x_scaling_factor;
          } else {
            curr_y-=scale;
            curr_x+=scale/x_scaling_factor;
          }
        }
        // Recursive function
        this.moveArm(prev_x, prev_y, curr_x, curr_y, face);
        this.traverseDFS(curr_graph, next, visited, curr_x, curr_y);
      }
    }
  }
  
  // Input: Frase que se va escribir; Cara donde se va escribir
  // Apartir de las dimensiones de lac caja, se escoge un inicio en cordenadas x,y 2D
  // Apartir del tamanio de la frase y el espacio disponible se calcula el escalamiento que se tendra en las letras. AKA: FONT SIZE
  // Iteramos sobre la palabra, 
  // Checamos que no sea un espacio el caracter
  //   mandamos el indice de cada caracter de esta misma a la funcion DFS, en esta tambien incluimos la posiciones de inicio calculadas.
  // Nos vemos de espacio de origen dependiendo de FONT SIZE
  public void writePhrase(String phrase, int face){
    // inicio caja (x,y)
    int x = 0, y = 0;
    // FONT SIZE DETERMINADO POR TAMANIO DE FRASE Y ESPACIO DISPONIBLE
    scale = 0; x_scaling_factor = 0;
    this.face = face;
    
    for (int i =0 ; i < phrase.length(); i++){
      if (phrase.charAt(i) != ' ')
        DFS(int(phrase.charAt(i) - 'A'), x, y);
      
      x += 0; // FONT SIZE
      y += 0; // FONT SIZE
    }
  }
   
   public void loadShapes(){
      base = loadShape("C:/Users/alexi/Downloads/drive-download-20240222T014058Z-001/r5.obj");
      shoulder = loadShape("C:/Users/alexi/Downloads/drive-download-20240222T014058Z-001/r1.obj");
      upArm = loadShape("C:/Users/alexi/Downloads/drive-download-20240222T014058Z-001/r2.obj");
      loArm = loadShape("C:/Users/alexi/Downloads/drive-download-20240222T014058Z-001/r3.obj");
      end = loadShape("C:/Users/alexi/Downloads/drive-download-20240222T014058Z-001/r4.obj");
      
      shoulder.disableStyle();
      upArm.disableStyle();
      loArm.disableStyle(); 
    }
    
    // INPUT: cordenadas actuales (x,y); cordenadas siguientes (x,y); Cara en donde dibujamos
    // Se llama la funcion writePos, que es encargada de calcular movimentos de todas las joints de nuestro brazo, dependiendo de cordenadas actuales y cordenadas siguientes, y su cara
    // Processing functions, encargas de mover el brazo del robot en la simulacion
    //
    public void moveArm(int curr_x, int curr_y, int next_x, int next_y, int face){ 
      boolean goalReached = false;
      while (!goalReached){
       this.writePos(curr_x, curr_y, next_x, next_y, face);
       
       // DO NOT CARE
       background(32);
       smooth();
       lights(); 
       directionalLight(51, 102, 126, -1, 0, 0);
    
       // logica para dibujar y moverse al siguiente punto
       // Modelado del cubo
       
       
       // DO NOT CARE
       sphere (2);
       popMatrix();
        
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
       
        // DIRECCION DE NODO
        // TODO: determinar condicion para que el goal haya sido completado, es decir que el brazo haya terminado el trazado de Nodo A a Nodo B
        if (curr_x == next_x && curr_y == next_y) goalReached = true;
      }
    }
}
