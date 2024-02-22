class Graph {
    int origin_x;
    int origin_y;
    int scale;

    ArrayList<Node> nodes;
    
    public Graph(){
        this.origin_x = 0;
        this.origin_y = 0;
        this.scale = 1;
        this.nodes = new ArrayList<Node>(9);
    }

    public void traverseDFS(int start, boolean[] visited){
        visited[start] = true;
        System.out.println(start);
        for(int i = 0; i < nodes.get(start).neighbors.size(); i++){
            if(!visited[nodes.get(start).neighbors.get(i)]){
                traverseDFS(nodes.get(start).neighbors.get(i), visited);
            }
        }
    }
}