ArrayList<Graph> graphs;

// ArrayList<string>
String path = "C:/Users/alexi/OneDrive/Escritorio/Semestre 6/TE3001B_Robotics_Fundamentals_DT/letters.json";

JSONObject json;
void setup(){
  graphs = new ArrayList<Graph>(26);
  json = loadJSONObject(path);
  
  JSONArray json_arr = json.getJSONArray("letters");
  
  for (int i = 0; i < json_arr.size(); i++) {
    JSONObject letter = json_arr.getJSONObject(i); 
    
    String key_ = str(char(i + 'A'));
    String encoded = letter.getString(key_); 
    
    System.out.print(encoded + " " );

    Graph key_graph = new Graph(1);
    
    key_graph.BuildGraphFromEncoded(encoded);
    System.out.print(key_ + " " );
    graphs.add(key_graph);

    System.out.println(graphs.size());
  }
  
  /*
  graphs = new ArrayList<Graph>();
  
  graphs.add(new Graph(1));
  
  for (int i = 0; i < graphs.size(); i++){
    Graph curr_graph = graphs.get(i);
    
    curr_graph.addEdge(0,2);
    curr_graph.addEdge(0,3);
    curr_graph.addEdge(0,4);
    curr_graph.addEdge(1,5);
    curr_graph.addEdge(1,6);
    curr_graph.addEdge(1,7);
    curr_graph.addEdge(2,6);
    curr_graph.addEdge(3,7);
    curr_graph.addEdge(4,8);
    curr_graph.addEdge(5,7);
  }
  */
}

void draw(){
  String palabra = "Hola ";
  for (int i = 0; i < 26; i++){
    Graph curr_graph = new Graph();
    curr_graph = graphs.get(i);
    System.out.print(str(char(i + 'A')) + " ");
    curr_graph.DFS(0,0);
    System.out.println(" " + "Finished");
  }
  System.out.println("------------------------------------------------------");
}
