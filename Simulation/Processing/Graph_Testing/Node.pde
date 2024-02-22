    class Node{
        int node_id;
        IntList neighbours;

        public Node(int node_id){
            this.node_id = node_id;
            this.neighbours = new IntList();
        }

        public void addNeighbor(int neighbor){
            this.neighbours.append(neighbor);
        }
    }
