/*main.cpp*/

//
// Aryann Chodankar
// U. of Illinois, Chicago
// CS 251: Spring 2020
// Project #07: open street maps, graphs, and Dijkstra's alg
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:  
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <string>
#include <limits>
#include <queue>
#include <algorithm>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "osm.h"
#include "graph.h"

using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max();

//class functor for our priority_queue
class prioritize{
   public:
   bool operator()(const pair<long long, double>&p1, const pair<long long, double>& p2) const{
      if(p1.second==p2.second){
         return p1.first > p2.first;
      }
      return p1.second > p2.second;
   }
};

//function for doing the Dijkstra graph traversal and returns a vector of visited, a map of closest distances and a map to predecessors
vector<long long> Dijkstra(graph<long long, double>& G, 
  long long startV, 
  map<long long, double>& distances,
  map<long long, long long>& predecessor)
{
  vector<long long>  visited;
  vector<long long> vertices=G.getVertices();
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;
  set <long long> neighbors;
  double edgeWeight, altDist;
  for(long long e: vertices){
     unvisitedQueue.push(make_pair(e,INF));
     distances[e]=INF;
	 predecessor[e]=0;
  }
  long long currentV;
  unvisitedQueue.push(make_pair(startV, 0));
  distances[startV]=0;
  while(!unvisitedQueue.empty()){
     currentV=unvisitedQueue.top().first;
     unvisitedQueue.pop();
     if(distances[currentV]==INF){
        break;
     } else if(count(visited.begin(), visited.end(), currentV)){
        continue;
     } else {
           visited.push_back(currentV);
     }
     neighbors=G.neighbors(currentV);
     for(long long adjV:neighbors){
        G.getWeight(currentV, adjV, edgeWeight);
        altDist=distances.at(currentV)+edgeWeight;
        if(altDist<distances.at(adjV)){
           distances[adjV]=altDist;
           unvisitedQueue.push(make_pair(adjV,altDist));
		   predecessor[adjV]=currentV;
        }
     }
  }
  

  return visited;
}

//////////////////////////////////////////////////////////////////
//
// main
//
int main()
{
  map<long long, Coordinates>  Nodes;     // maps a Node ID to it's coordinates (lat, lon)
  vector<FootwayInfo>          Footways;  // info about each footway, in no particular order
  vector<BuildingInfo>         Buildings; // info about each building, in no particular order
  XMLDocument                  xmldoc;
  graph <long long, double> nodeGraph;
  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "")
  {
    filename = def_filename;
  }

  //
  // Load XML-based map file 
  //
  if (!LoadOpenStreetMap(filename, xmldoc))
  {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }
  
  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);
	
  //adds all the vertices to the map
  for(auto node: Nodes){
	  nodeGraph.addVertex(node.first);
  }
	
  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);
	
  //adds all the edges to the map, both ways
  for(auto edge: Footways){
	  for(unsigned int i=0; i<edge.Nodes.size()-1; i++){
		  nodeGraph.addEdge(edge.Nodes[i],edge.Nodes[i+1],distBetween2Points(Nodes[edge.Nodes[i]].Lat, Nodes[edge.Nodes[i]].Lon, Nodes[edge.Nodes[i+1]].Lat, Nodes[edge.Nodes[i+1]].Lon));
		  nodeGraph.addEdge(edge.Nodes[i+1],edge.Nodes[i],distBetween2Points(Nodes[edge.Nodes[i]].Lat, Nodes[edge.Nodes[i]].Lon, Nodes[edge.Nodes[i+1]].Lat, Nodes[edge.Nodes[i+1]].Lon));
	  }
  }
  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert((unsigned)nodeCount == Nodes.size());
  assert((unsigned)footwayCount == Footways.size());
  assert((unsigned)buildingCount == Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;
  cout << "# of vertices: " << nodeGraph.NumVertices() << endl;
  cout << "# of edges: " << nodeGraph.NumEdges() << endl;
  cout << endl;

  //
  // Navigation from building to building
  //
  string startBuilding, destBuilding;
  BuildingInfo startBldng, destBldng; //for storing the startBuilding and destBuilding information
  cout << "Enter start (partial name or abbreviation), or #> ";
  getline(cin, startBuilding);

  while (startBuilding != "#")
  {
    cout << "Enter destination (partial name or abbreviation)> ";
    getline(cin, destBuilding);
    int check=0;
	for(auto building: Buildings){ //searches every Building for the entered abbreviation
		if(startBuilding==building.Abbrev){
			startBldng=building;
			check=1;
			break;
		}
	}
	if(check==0){
		for(auto building: Buildings){
			if(building.Fullname.find(startBuilding) != std::string::npos){ //searches every Building for the entered fullname
				startBldng=building;
				check=1;
				break;
			}
		}
	}
	  
	if(check==0){
		cout << "Start building not found" << endl;
	} else {
		check=0;
		for(auto building: Buildings){
			if(destBuilding==building.Abbrev){ //searches every Building for the entered abbreviation
				destBldng=building;
				check=1;
				break;
			}
		}
		if(check==0){
			for(auto building: Buildings){
				if(building.Fullname.find(destBuilding) != std::string::npos){ //searches every Building for the entered fullname
					destBldng=building;
					check=1;
					break;
				}
			}
		}
		if(check==0){ 
			cout << "Destination building not found" << endl;
		} else { 
			//if u reached here it means the inputs are valid 
			cout<<"Starting point:"<<endl; 
			cout<<" "<<startBldng.Fullname<<endl;
			cout<<" ("<<startBldng.Coords.Lat<<", "<<startBldng.Coords.Lon<<")"<<endl;
			cout<<"Destination point:"<<endl;
			cout<<" "<<destBldng.Fullname<<endl;
			cout<<" ("<<destBldng.Coords.Lat<<", "<<destBldng.Coords.Lon<<")"<<endl;
			long long startId, destId;
			double startCompare=INF;
			double destCompare=INF;
			for(auto node: Footways){ //going through every footway
				for(auto footNode: node.Nodes){ //going through every connected node of each footway and finds the min distance (closest nodes)
					if(distBetween2Points(startBldng.Coords.Lat, startBldng.Coords.Lon, Nodes[footNode].Lat, Nodes[footNode].Lon)<startCompare){
						startId=footNode;
						startCompare=distBetween2Points(startBldng.Coords.Lat, startBldng.Coords.Lon, Nodes[footNode].Lat, Nodes[footNode].Lon);
					}
					if(distBetween2Points(destBldng.Coords.Lat, destBldng.Coords.Lon, Nodes[footNode].Lat, Nodes[footNode].Lon)<destCompare){
						destId=footNode;
						destCompare=distBetween2Points(destBldng.Coords.Lat, destBldng.Coords.Lon, Nodes[footNode].Lat, Nodes[footNode].Lon);
					}
				}
			}
			//Outputting eveything correctly
			cout<<endl;
			cout<<"Nearest start node:"<<endl;
			cout<<" "<<startId<<endl;
			cout<<" ("<<Nodes[startId].Lat<<", "<<Nodes[startId].Lon<<")"<<endl;
			cout<<"Nearest destination node:"<<endl;
			cout<<" "<<destId<<endl;
			cout<<" ("<<Nodes[destId].Lat<<", "<<Nodes[destId].Lon<<")"<<endl<<endl;
			//making maps necessary for Dijkstra
			map<long long, double> distances;
			map<long long, long long> predecessor;
			Dijkstra(nodeGraph, startId, distances, predecessor); //Dijkstra is done on graph using the startId
			cout<<"Navigating with Dijkstra..."<<endl;
			long long currentV=destId;
			stack<long long> pathList;
			if(predecessor[currentV]==0&&startId!=destId){ //If there is not predecessor and they both are not at the same footway 
				cout<<"Sorry, destination unreachable"<<endl;
			} else {
				cout<<"Distance to dest: "<<distances[destId]<<" miles"<<endl;
				while(predecessor[currentV]!=0){
					pathList.push(predecessor[currentV]); //pushes all to stack so we can later print out in reverse order whilst popping
					currentV=predecessor[currentV];
				}
				cout<<"Path: ";
				while(!pathList.empty()){
					cout<<pathList.top()<<"->"; //prints in reverse order
					pathList.pop();
				}
				cout<<destId<<endl;
			}
		}
	}

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter start (partial name or abbreviation), or #> ";
    getline(cin, startBuilding);
  }

  //
  // done:
  //
  cout << "** Done **" << endl;

  return 0;
}
