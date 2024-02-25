import java.util.*;

class Graph {
  int scale;

  final int x_scaling_factor = 2;
  final int amount_nodes = 9;
  ArrayList<Node> nodes;
  HashSet<String> set;
  HashSet<String> copy_set;

  String[] headers = {"a", "b", "c", "d", "e", "f", "g1", "g2", "h", "i", "j", "k", "l", "m"};

  public Graph(int scale) {
    this.scale = scale;
    this.nodes = new ArrayList<Node>();
    this.set = new HashSet<String>();
    this.copy_set = new HashSet<String>();

    for (int i = 0; i < amount_nodes; i++) this.nodes.add(new Node(i));
  }

  public int[] computeEdges(int index) {
    int[] edges;
    
    switch (this.headers[index]) {
      case "a":
        edges = new int[]{7,8,9};
      break;
      case "b":
        edges = new int[]{6,9};
      break;
      case "c":
        edges = new int[]{3,6};
      break;
      case "d":
        edges = new int[]{1,2,3};
      break;
      case "e":
        edges = new int[]{1,4};
      break;
      case "f":
        edges = new int[]{4,7};
      break;
      case "g1":
        edges = new int[]{4,5};
      break;
      case "g2":
        edges = new int[]{5,6};
      break;
      case "h":
        edges = new int[]{5,7};
      break;
      case "i":
        edges = new int[]{5,8};
      break;
      case "j":
        edges = new int[]{5,9};
      break;
      case "k":
        edges = new int[]{3,5};
      break;
      case "l":
        edges = new int[]{2,5};
      break;
      case "m":
        edges = new int[]{1,5};
      break;
      default:
        edges = new int[]{0,0};
      break;
    }
    
    return edges;
  }

  public void traverseDFS(int start, boolean[] visited, int curr_x, int curr_y) {
    visited[start] = true;
    for (int i = 0; i < this.nodes.get(start).neighbours.size(); i++) {
      int next = this.nodes.get(start).neighbours.get(i);
      
      int sm_node = next < start ? next : start;
      int bg_node = next > start ? next : start;
      
      if (!visited[next] || this.copy_set.contains(sm_node + ", " + bg_node)) {
        this.copy_set.remove(sm_node + ", " + bg_node);
        int diff = start - next;
        System.out.print(" " + start + " -> " + next + " ");
        //int prev_x = curr_x, prev_y = curr_y;
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
        //draw(prev_x, prev_y, curr_x, curr_y);
        this.traverseDFS(next, visited, curr_x, curr_y);
      }
    }
  }
  
  public void printAdjcList(){
    System.out.println(" ");
    for (int i = 0; i < amount_nodes; i++){
      this.nodes.get(i).printAdjcList();
    }
  }
  
  
  // Constrain, node a must be smaller than node b
  public void addEdge(int node_a, int node_b) {
    // TODO: ADD CODDED CONSTRAIN
    this.nodes.get(node_a).neighbours.append(node_b);
    this.nodes.get(node_b).neighbours.append(node_a);
    
    this.set.add(node_a + ", " + node_b);
  }

  public void BuildGraphFromEncoded(String encoded) {
    for (int i = 0; i < encoded.length(); i++) {
      if (encoded.charAt(i) == '1'){
        int []edges = computeEdges(i);
        
        this.addEdge(edges[0] - 1, edges[1] - 1);
        
        if (headers[i] == "a" || headers[i] == "d") this.addEdge(edges[1] - 1, edges[2] - 1);
      }
    }
  }
  
  public void iterateOverSet(){
    for (String ss : set)
      System.out.print("(" + ss + ")");
    
    System.out.println("");
  }
  
  public void resetCopySet(){
    copy_set = (HashSet)set.clone();
  }
}
