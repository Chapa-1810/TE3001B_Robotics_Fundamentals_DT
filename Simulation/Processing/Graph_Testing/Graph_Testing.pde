ArrayList<Graph> graphs;

void setup(){
  graphs = new ArrayList<Graph>();

  graphs.add(new Graph(0,0,1));
  
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
}

void draw(){
  boolean visited[] = {false, false, false, false, false, false, false, false, false}; 
  graphs.get(0).traverseDFS(0, visited);
  
  System.out.println(" " + Finished");
}
