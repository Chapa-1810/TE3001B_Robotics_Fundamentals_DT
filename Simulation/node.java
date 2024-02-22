    class Node{
        int node_id;
        IntList neighbors;

        public Node(char node_id){
            this.node_id = node_id;
            this.neighbors = new IntList();
        }

        public void addNeighbor(int neighbor){
            this.neighbors.add(neighbor);
        }
    }