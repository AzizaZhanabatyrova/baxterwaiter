//===============================================================================//
// Name			: aonode.cpp
// Author(s)	: Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: Generic node element of an AND-OR graph
//===============================================================================//

#include "aonode.h"

//! constructor of class HyperArc
//! @param[in] index    index of the hyperarc
//! @param[in] nodes    set of child nodes connected via the hyperarc
//! @param[in] cost     generic hyperarc cost
HyperArc::HyperArc(int index, vector<AOnode*> nodes, int cost)
{
    hIndex = index;
    children = nodes;
    hCost = cost;
    
    //DEBUG:printArcInfo();
}

//! display hyperarc information
void HyperArc::printArcInfo()
{
    cout<<"Info of hyperarc: " <<hIndex <<endl;
    cout<<"Hyperarc cost: " <<hCost <<endl;
    cout<<"Child nodes: ";
    for (int i=0; i< (int)children.size(); i++)
        cout<<children[i]->nName <<" ";
    cout<<endl;
}

//! constructor of class AOnode
//! @param[in] name	   name of the node
//! @param[in] cost    generic node cost
AOnode::AOnode(string name, int cost)
{
    nName = name;
	nCost = cost;
    nFeasible = false;
    nSolved = false;
    
    //DEBUG:printNodeInfo();
}

//! associate the application-specific element with the node
//! @param[in] element    pointer to the element to associate
void AOnode::addElement(NodeElement* element)
{
    nElement = element;
}

//! add an hyperarc to child nodes
//! @param[in] hyperarcIndex    hyperarc index
//! @param[in] nodes            set of child nodes connected via the hyperarc
//! @param[in] hyperarcCost     hyperarc cost
void AOnode::addArc(int hyperarcIndex, vector<AOnode*> nodes, int hyperarcCost)
{
    // create the hyperarc
    HyperArc toAdd(hyperarcIndex, nodes, hyperarcCost);
    
    // add this node to the vector of parents of each child node
    for (int i=0; i< (int)nodes.size(); i++)
        nodes[i]->parents.push_back(this);
    
    // add it to the set of hyperarcs
    arcs.push_back(toAdd);
}

//! display node information
void AOnode::printNodeInfo()
{
    cout<<endl;
    cout<<"Info of node: " <<nName <<endl;
    cout<<"Node cost: " <<nCost <<endl;
    cout<<"Is feasible? " <<boolalpha <<nFeasible <<endl;
    cout<<"Is solved? " <<boolalpha <<nSolved <<endl;
    cout<<"Parent nodes: ";
    for (int i=0; i< (int)parents.size(); i++)
        cout<<parents[i]->nName <<" ";
    cout<<endl;
    cout<<"Number of hyperarcs: " <<arcs.size() <<endl;
    for (int i=0; i< (int)arcs.size(); i++)
        arcs[i].printArcInfo();
    
    // display info of the associated element
    if (nElement != NULL)
    {
        cout<<"Info of associated element" <<endl;
        nElement->printNodeElementInfo();
    }
    else
        cout<<"[REPORT] No application-specific element is associated with this node." <<endl;    
}

//! determine whether the node is feasible
void AOnode::isFeasible()
{
    bool temp_isFeasible = false;
    
    // 1. the node is feasible if it is already feasible
    if (nFeasible == true)
        temp_isFeasible = true;
        
    // 2. the node is feasible if it is a terminal node (no hyperarcs)
    if (arcs.size() == 0)
        temp_isFeasible = true;
    
    // 3. the node is feasible if it has >=1 hyperarcs with all child nodes solved
    // iterate on the hyperarcs of the node
    for (int i=0; i<(int)arcs.size(); i++)
    {
        bool allSolved = true;
            
        // iterate on the child nodes
        for (int j=0; j<(int)arcs[i].children.size(); j++)
        {
            // if the node is not solved --> break
            if (arcs[i].children[j]->nSolved == false)
            {
                allSolved = false;
                break;
            }
        }
        // if all the child nodes are solved, this node is feasible
        if (allSolved == true)
        {
            temp_isFeasible = true;
            break;
        }
    }
    
    // set the feasibility property
    nFeasible = temp_isFeasible;
}

//! set the node as solved
//! @return result of the operation (true = done, false = not done)
bool AOnode::setSolved()
{
    // issue a warning if the node is already solved
    if(nSolved == true)
    {
        cout<<"[WARNING] The node is already solved." <<endl;
        return false;
    }
    // a node can be solved only if it's feasible
    if (nFeasible == true)
        nSolved = true;
    else
        cout<<"[ERROR] The node is not feasible. Are you sure it is solved?" <<endl;
    
    return nSolved;
}