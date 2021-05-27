// Standard C++ libraries
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>

#include <typeinfo>

// Custom header files
#include "gls/datastructures.hpp"
#include "gls/io.hpp"

using namespace gls::datastructures;

std::ostream& operator<<(std::ostream& output, const gls::datastructures::IEdge& e) {
      return output << "(" << e.first << ", " << e.second << ")";
}

// Transition function from one state to another
// Each new vertex must have a unique vertex_descriptor for lookup
// The same state should ALWAYS have the same descriptor
// AKA use the state to create the descriptor
std::vector<IVertex> transition_function(IVertex vi, gls::io::MotionPrimitiveReader *mReader){
    std::vector<IVertex> neighbors; 

    // Readout state from vertex_descriptor 
    int x_idx = vi.find("|");
    int y_idx = vi.find("|", x_idx+1);
    int x = std::stoi(vi.substr(0, x_idx));
    int y = std::stoi(vi.substr(x_idx+1, y_idx-x_idx-1));
    int theta = std::stoi(vi.substr(y_idx+1, vi.size()-y_idx-1));

    // Neighbors
    int nx, ny, ntheta;
    IVertex neighbor;
    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    for (gls::io::MotionPrimitive mprim : mprims){

        // prims for current theta
        if(mprim.starttheta_c == theta){
            nx = mprim.endcell.x + x;
            ny = mprim.endcell.y + y;
            ntheta = mprim.endcell.theta;
            neighbor = std::to_string(nx) + 
                "|" + std::to_string(ny) + 
                "|" + std::to_string(ntheta);
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

int main (int argc, char const *argv[]) {

  // Load Motion Primitives
  gls::io::MotionPrimitiveReader* mReader = new gls::io::MotionPrimitiveReader();
  mReader->ReadMotionPrimitives("/home/schmittle/Research/boxes/pysbpl/pysbpl/mprim/mushr.mprim");

  // Create implicit graph
  // Bind so we can pass mReader
  ImplicitGraph g(std::bind(&transition_function, std::placeholders::_1, mReader));
  IVertex v_0 = "1|20|1";
  IVertex v_3 = "3|3|3";
  g.addVertex(v_0);
  g.addVertex(v_3);

  std::cout << "Vertices, outgoing edges, and adjacent vertices" << std::endl;
  std::vector<IVertex> verts = vertices(g);
  IVertexIter vi_end = verts.end();
  for (IVertexIter vi = verts.begin(); vi != vi_end; vi++) {
    IVertex u = *vi;
    std::cout << "Vertex " << u << ": ";

    std::cout << " Adjacent vertices ";
    std::vector<IVertex> ajs = adjacent_vertices(u, g);
    IVertexIter ai_end = ajs.end();
    IVertexIter ai = ajs.begin();
    for (IVertexIter ai = ajs.begin(); ai != ai_end; ai++) {
      IVertex ni = *ai;
      std::cout << ni << " ";
    }
    
    std::cout << std::endl;
  }
  std::cout << num_vertices(g) << " vertices" << std::endl << std::endl;

  std::cout << "Edges and weights" << std::endl;
  std::vector<IEdge> edgs = edges(g);
  IEdgeIter ei_end = edgs.end();
  for (IEdgeIter ei = edgs.begin(); ei != ei_end; ei++) {
    IEdge e = *ei;
    std::cout << e << std::endl;
  }
  std::cout << num_edges(g) << " edges"  << std::endl;

  return 0;
}
