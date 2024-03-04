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
        
        public void printAdjcList(){
          System.out.print(node_id);
          
          for (int i = 0; i < neighbours.size(); i++){
            System.out.print(" " + neighbours.get(i));
          }
          
           System.out.println(" ");
        }
    }
