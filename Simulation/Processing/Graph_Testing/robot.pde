import java.util.*;

class Robot {
  ArrayList<Graph> graphs;
  ArrayList<String> coords;
  int start_node = 6;
  
  String path = "C:/Users/alexi/OneDrive/Escritorio/Semestre 6/TE3001B_Robotics_Fundamentals_DT/letters.json";
  JSONObject json;
  
  float scale, x_scaling_factor;
  
  public Robot(){
    this.loadRobot();
  }
  
  public void loadRobot(){
     this.loadGraphs(path);
     
     this.coords = new ArrayList<String>();
     //this.loadShapes();
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
  public void traverseDFS(Graph curr_graph, int start, boolean[] visited, float curr_x, float curr_y) {
    visited[start] = true;
    for (int i = 0; i < curr_graph.nodes.get(start).neighbours.size(); i++) { // Se itera sobre una lista de adjacencias para todos los vecinos del nodo
      int next = curr_graph.nodes.get(start).neighbours.get(i);
      
      int sm_node = next < start ? next : start;
      int bg_node = next > start ? next : start;
      
      if (!visited[next] || curr_graph.copy_set.contains(sm_node + ", " + bg_node)) { // Checamos si fue visitado o si le conexion entre nodos ya se hizo (SET<String>)
        curr_graph.copy_set.remove(sm_node + ", " + bg_node); // ELIMINAMOS CONEXION EN EL SET
        int diff = start - next;
        System.out.print(" " + start + " -> " + next + " ");
        float next_x = curr_x, next_y = curr_y;
        
        // Calculo logico de nuevas cordenadas (x,y) para el siguiente nodo
        if (abs(diff) == 3) next_y += (abs(diff) / diff) * scale;
        else if (abs(diff) == 1) next_x += (abs(diff) / diff) * scale/x_scaling_factor;
        else if (abs(diff) == 4) {
          if (start < next) {
            next_y+=scale;
            next_x+=scale/x_scaling_factor;
          } else {
            next_y-=scale;
            next_x-=scale/x_scaling_factor;
          }
        } else {
          if (start < next) {
            next_y+=scale;
            next_x-=scale/x_scaling_factor;
          } else {
            next_y-=scale;
            next_x+=scale/x_scaling_factor;
          }
        }
        // Recursive function
        coords.add(curr_x + "," + curr_y + "," + next_x + "," + next_y + ";");
        //this.moveArm(prev_x, prev_y, curr_x, curr_y, face);
        this.traverseDFS(curr_graph, next, visited, next_x, next_y);
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
  public void writePhrase(String phrase){
    // inicio caja (x,y)
    int x = 0, y = 0;
    // FONT SIZE DETERMINADO POR TAMANIO DE FRASE Y ESPACIO DISPONIBLE
    scale = 1; x_scaling_factor = 1;
    
    for (int i =0 ; i < phrase.length(); i++){
      if (phrase.charAt(i) != ' '){
        System.out.print(phrase.charAt(i) + " ");
        DFS(int(phrase.charAt(i) - 'A'), x, y);
        System.out.println(" ");
      } else {
         coords.add(" ");
      }
      
      x += 4; // FONT SIZE
      y += 0; // FONT SIZE
    }
  }
  
  public void cleanCords(){
    coords.clear();
  }
}
