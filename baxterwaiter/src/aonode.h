//===============================================================================//
// Name			: aonode.h
// Author(s)	: Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: Generic node element of an AND-OR graph
//===============================================================================//

#ifndef AONODE_H
#define AONODE_H

#include <iostream>
#include <vector>

#include "element.h"

using namespace std;

// empty declaration (required by HyperArc)
class AOnode;

//! class "HyperArc" for the hyperarc connecting one parent node to a number of child nodes in an AND relationship among themselves
class HyperArc
{
    public:
        int hIndex;                 //!< index of the hyperarc
        vector<AOnode*> children;   //!< set of child nodes connected via the hyperarc
        int hCost;                  //!< cost of the hyperarc
        
        //! constructor
		HyperArc(int index, vector<AOnode*> nodes, int cost);
        
        //! display hyperarc information
        void printArcInfo();
        
        //! destructor
		~HyperArc()
		{
			//DEBUG:cout<<endl <<"Destroying HyperArc object" <<endl;
		}
};

//! class "AOnode" for the generic node element of an AND-OR graph
class AOnode
{
    public:
        NodeElement* nElement;      //!< pointer to the application-specific element associated with the node        
        string nName;               //!< name of the node
        int nCost;                  //!< cost of the node
        bool nSolved;               //!< solved: the operation has been performed
        bool nFeasible;             //!< feasible: >=1 hyperarc has all child nodes solved
        vector<HyperArc> arcs;      //!< hyperarcs connecting the node to child nodes
        vector<AOnode*> parents;    //!< nodes having this node as a child node

        //! constructor
		AOnode(string name, int cost);

        //! associate the application-specific element with the node
        void addElement(NodeElement* element);
        
        //! add an hyperarc to child nodes
        void addArc(int hyperarcIndex, vector<AOnode*> nodes, int hyperarcCost);
        
        //! display node information
        void printNodeInfo();
        
        //! determine whether the node is feasible
        void isFeasible();
        
        //! set the node as solved
        bool setSolved();
        
        //! destructor
		~AOnode()
		{
			//DEBUG:cout<<endl <<"Destroying AOnode object" <<endl;
		}
};

#endif
