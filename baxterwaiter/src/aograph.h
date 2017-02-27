//===============================================================================//
// Name			: aograph.h
// Author(s)	: Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: AND-OR graph
//===============================================================================//

#ifndef AOGRAPH_H
#define AOGRAPH_H

#include <algorithm>
#include <fstream>

#include "aonode.h"

using namespace std;

//! class "Path" for each unique path traversing the graph from the head to the leaves
class Path
{        
    public:
        int pIndex;                 //!< index of the path
        int pCost;                  //!< overall cost of all the nodes in the path
        bool pComplete;             //!< complete: the path fully traverses the graph
        vector<AOnode*> pathNodes;  //!< set of the nodes in the path
        vector<int> pathArcs;       //!< set of the hyperarcs in the path
        vector<bool> checkedNodes;  //!< checked: the node has been analysed
        
        //! constructor
		Path(int cost, int index);
        
        //! copy constructor
        Path(const Path &toBeCopied, int index);
        
        //! display path information
        void printPathInfo();
        
        //! add a node in the path
        void addNode(AOnode* node);
        
        //! update the path information (when a node is solved)
        void updatePath(string nameNode, int cost);
        
        //! find the feasible node to suggest
        AOnode* suggestNode();
        
        //! destructor
		~Path()
		{
			//DEBUG:cout<<endl <<"Destroying Path object" <<endl;
		}		
};

//! class "AOgraph" for the AND-OR graph
class AOgraph
{    
    protected:
        //** GRAPH INITIALIZATION **//
        //! add a node in the graph
        void addNode(string nameNode, int cost);
        

        
        //! update the feasibility status of the nodes in the graph
        void updateNodeFeasibility();
		
        //! compute the cost to add to a path
        int computeAddCost(AOnode &node, int hIndex);
        
        //! generate all possible paths navigating the graph
        void generatePaths();
        
        //! set up a graph
        void setupGraph();
        
        //** GRAPH NAVIGATION **//
        //! find the hyperarc connecting a parent to a child node
        HyperArc* findHyperarc(AOnode &parent, AOnode &child);
        
        //! compute the overall update cost (intermediate step to update the path cost)
        int computeOverallUpdate(AOnode &node);
        
        //! update all paths (update path costs when a node is solved)
        void updatePaths(AOnode &solved);
        
        //! find the optimal path (long-sighted strategy)
        int findOptimalPath();
    
    public:
        string gName;           //!< name of the graph
        vector<AOnode> graph;   //!< set of nodes in the AND-OR graph
        AOnode* head;           //!< pointer to the node = final assembly
        vector<Path> paths;     //!< set of paths in the AND-OR graph
        vector<int> pIndices;   //!< indices of the updated paths
        vector<int> pUpdate;    //!< costs subtracted to the updated paths
        
        //! constructor
		AOgraph(string name);
        
        //! load the graph description from a file
        void loadFromFile(string fileName);
        
        //! display graph information
        void printGraphInfo();

		//! find a node by name
        AOnode* findByName(string nameNode);
        
        //! suggest the node to solve
        string suggestNext(bool strategy);
        
        //! solve a node, finding it by name
        void solveByName(string nameNode);
        
        //! destructor
		~AOgraph()
		{
			//DEBUG:cout<<endl <<"Destroying AOgraph object" <<endl;
		}
};

#endif
