class Graph {
    int origin_x;
    int origin_y;
    int scale;
    final int amount_nodes = 9; 
    ArrayList<Node> nodes;
    
    public Graph(int origin_x, int origin_y, int scale){
        this.origin_x = origin_x;
        this.origin_y = origin_y;
        this.scale = scale;
        this.nodes = new ArrayList<Node>();
        
        for (int i = 0; i < amount_nodes; i++) nodes.add(new Node(i));
    }

    public void traverseDFS(int start, boolean[] visited){
        visited[start] = true;
        System.out.print(" " + start);
        for(int i = 0; i < nodes.get(start).neighbours.size(); i++){
            if(!visited[nodes.get(start).neighbours.get(i)]){
                traverseDFS(nodes.get(start).neighbours.get(i), visited);
            }
        }
    }
    
    public void addEdge(int node_a, int node_b){
      nodes.get(node_a).neighbours.append(node_b);
      nodes.get(node_b).neighbours.append(node_a);
    }
}
