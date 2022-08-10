#include <iostream>
#include "Astar.h"
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <bits/stdc++.h>

using namespace std;



// this code has many bugs, and just used as a refernce refure to A_star.









// ---------------------- Node -----------------------------------------------------------------------------------------------
/*
Every Node is defined with:
Pose (x,y,z)
Parent Node
H cost := Straight Line distance between EndPose(x,y,z) and CurrentPose(x,y,z)
G cost := Path taken distance between CurrentPose(x,y,z) and BeginningPose(x,y,z)
F cost
*/
/* Default Constructor */
AstarContainer::Node::Node() {
    x = 0;
    y = 0;
    z = 0;
    pose.push_back(x);
    pose.push_back(y);
    pose.push_back(z);
    gost = -1;
}

/* Override Constructor */
AstarContainer::Node::Node(double ix, double iy, double iz, Node &iparent, double ihost) {
    x = ix;
    y = iy;
    z = iz;
    pose.push_back(x);
    pose.push_back(y);
    pose.push_back(z);

    parent = &iparent;

    host = ihost; // cost from current to goal node

    gost = gostCalc(); // cost from start node to current node

    fost = fostCalc(); // sum of g-cost and h-cost

}

/* Default Deconstructor */
AstarContainer::Node::~Node() {}

/* Calculates G Cost --> makes local calculation and adds on to parent g cost */
double AstarContainer::Node::gostCalc() {
    return sqrt(pow(x-parent->x,2) + pow(y-parent->y,2) + pow(z-parent->z,2)) + parent->gost;
}

/* Calculates F cost */
double AstarContainer::Node::fostCalc() {
    return gost + host;
}


// ---------------------- Node ------------------------------------------------------------------------------------------------






// ---------------------- Astar ------------------------------------------------------------------------------------------------
/* 
Every cordinate is defined by a Node --> containing its cordinates, the most previous node needed to get to it {parent node}, and the cost to get to it.
*/
/* Overide Constructor */
AstarContainer::Astar::Astar(Eigen::Matrix<int,3,3> iobjectXYZ, Eigen::Matrix<int,3,1> endCord, int istep_mm, int icushion, Eigen::Matrix<double,3,1> startCord) {
    PathIsCompleted = false;

    step_mm = istep_mm;
    step_m = (double)step_mm/1000;

    endXYZ_mm = endCord;
    shiftEndNode();

    startPos.push_back(startCord(0)); // m
    startPos.push_back(startCord(1));
    startPos.push_back(startCord(2));
    addNode(startPos,deadNode);
    openNodes[startPos].gost = 0;

    objectXYZ_mm = iobjectXYZ;
    shiftObjects();
    
    //
    explore(openNodes[startPos]); // start exploring starting at the starting node
}

/* Default Destructor */
AstarContainer::Astar::~Astar(){}

/* H cost of node. Calculated within the Astar class because the end pos will not be know until path planner is initiated */
double AstarContainer::Astar::nodeHost(double xpos,double ypos,double zpos) {
    return sqrt(pow(endPos[0] - xpos,2) + pow(endPos[1] - ypos,2) + pow(endPos[2] - zpos,2));
}

/*
Shifts the specified object cordinates into step size spacing
Does this by taking each cordinate and subtracting its modulo by the step with the current cordinate unit direction
Finishes by making a node out of the cordinates
*/
void AstarContainer::Astar::shiftObjects(){                                        // Object prep work
    std::vector<double> objectXYZ_m;
    for (int i = 0; i < objectXYZ_mm.cols(); i++) {
        objectXYZ_m.clear();
        for (int j = 0; j < objectXYZ_mm.rows(); j++) { // for the x y z cordinates of each obstical:
            if (objectXYZ_mm(j,i)%step_mm != 0) {
                int shift = objectXYZ_mm(j,i)%step_mm;
                objectXYZ_mm(j,i) = objectXYZ_mm(j,i) - shift;
                objectXYZ_m.push_back((double)objectXYZ_mm(j,i)/1000);
            }
            else {
                objectXYZ_m.push_back((double)objectXYZ_mm(j,i)/1000);
            }
            objectContainer(j,i) = objectXYZ_m[j];
        }
        initobjectVector.push_back(objectXYZ_m);

        //addNode(objectXYZ_m,deadNode);
        //Node node(objectXYZ_m[0],objectXYZ_m[1],objectXYZ_m[2],deadNode,-1);
        ///openNodes[objectXYZ_m] = node;

    }
    //std::cout << initobjectVector[0][0] << std::endl;
    enlargeObjects(3);

}

/*
Shifts the specified goal cordinates into step size spacing
Does this by taking each cordinate and subtracting its modulo by the step with the current cordinate unit direction
Finishes by making a node out of the cordinates
*/
void AstarContainer::Astar::shiftEndNode() {                                    // End node prep work
    std::vector<double> endXYZ_m;
    for (int i = 0; i < endXYZ_mm.cols(); i++) {
        endXYZ_m.clear();
        for (int j = 0; j < endXYZ_mm.rows(); j++) { // for the x y z cordinates of each obstical:
            if (endXYZ_mm(j,i)%step_mm != 0) {
                int shift = endXYZ_mm(j,i)%step_mm;
                endXYZ_mm(j,i) = endXYZ_mm(j,i) - shift;
                endXYZ_m.push_back((double)endXYZ_mm(j,i)/1000);
            }
            else {
                endXYZ_m.push_back((double)endXYZ_mm(j,i)/1000);
            }
        }
        endPos.push_back(endXYZ_m[0]); // m
        endPos.push_back(endXYZ_m[1]);
        endPos.push_back(endXYZ_m[2]);
        addNode(endXYZ_m,deadNode);
    }
}

void AstarContainer::Astar::enlargeObjects(int stepsOuts) {                         // Object prep work
    std::vector<double> addObject;
    for (int n=0; n<initobjectVector.size(); n++) {
        addObject.clear();
        addObject.push_back(0);
        addObject.push_back(0);
        addObject.push_back(0);
        for (int i=0;i<stepsOuts; i++) {
            for (int j=0;j<stepsOuts; j++) {
                for(int k=0;k<stepsOuts; k++) {
                    addObject[0] = initobjectVector[n][0]+step_m*i;
                    addObject[1] = initobjectVector[n][1]+step_m*j;
                    addObject[2] = initobjectVector[n][2]+step_m*k;
                    //Node node(initobjectVector[n][0]+step_m*i,initobjectVector[n][1]+step_m*j,initobjectVector[n][2]+step_m*k,deadNode,-1);
                    //openNodes[addObject] = node;
                    objectVector.push_back(addObject);
                    addNode(addObject,deadNode);
                }
            }
        }
    }
    //std::cout << "Finish\n";
}


/*
openNodes is an ordered map that is storing all of the values ov every single node that is opened.
whenever searching occured and we want to add a new node we first check to see if that node has already
been created.
*/ 
void AstarContainer::Astar::addNode(std::vector<double> cordinates,Node& parent) {
    std::map<std::vector<double>,Node>::iterator it;
    it = openNodes.find(cordinates);
    if (it != openNodes.end()) {    // the cordinates were found
        if (CompareVectors(endPos,cordinates,.01)) {    // check to see if these cordinates represent the ending cordinates
            double host = 0; // H cost gets set to zero due to this signifying the goal node
            Node node(endPos[0],endPos[1],endPos[2],parent,host);
            openNodes[endPos] = node;
            finalizePath(parent); // plot path node cordinates to terminal and create array of path nodes
        }
        else {   // determine if node should be the next node
            checkGostIsMinimum(cordinates,parent);
        }
    }
    else { // the cordiantes were not found
        if (ObjectCheck(objectVector,cordinates,.01)) { // determine if new cordinate is an object
            double host = -1;                                                                           // objects get defined by their h-cost being equal to (-1)
            Node node(cordinates[0],cordinates[1],cordinates[2],deadNode,host);
            openNodes[cordinates] = node;
        }
        else {
            double host = nodeHost(cordinates[0],cordinates[1],cordinates[2]); // determine the h-cost of the node
            Node node(cordinates[0],cordinates[1],cordinates[2],parent,host); // create node
            openNodes[cordinates] = node; // add node to all nodes list
            if (cordinates != startPos && node.parent->gost != -1) { // && node.parent->gost != deadNode.gost
                addChoosenNodes(node);
        }
        }
    }
}

bool AstarContainer::Astar::CompareVectors(std::vector<double> object, std::vector<double> node, double tol) {
    if (abs(object[0] - node[0]) <= abs(step_m-tol) && abs(object[1] - node[1]) <= abs(step_m-tol) && abs(object[2] - node[2]) <= abs(step_m-tol)) {
        return true;
    }
}

bool AstarContainer::Astar::ObjectCheck(std::vector<std::vector<double>> objects, std::vector<double> node, double tol) {
    int j = 0;
    int NumOfObjectsLocated = objects.size();
    if (NumOfObjectsLocated >= 1) {
        for (int i=0; i < NumOfObjectsLocated; i++) {
            if (abs(objects[i][0] - node[0]) <= abs(step_m-tol) && abs(objects[i][1] - node[1]) <= abs(step_m-tol) && abs(objects[i][2] - node[2]) <= abs(step_m-tol)) {
                j = 1;
            }
        }
    }
    if (j == 1){
        return true;
    }
    else {
        return false;
    }
}

void AstarContainer::Astar::checkGostIsMinimum(std::vector<double> cordinates, Node& possibleNewParent) {
    Node possibleNewNode(openNodes[cordinates].x,openNodes[cordinates].y,openNodes[cordinates].z,possibleNewParent,openNodes[cordinates].host);
    if (possibleNewNode.gost < openNodes[cordinates].gost) {
        openNodes[cordinates] = possibleNewNode;
        addChoosenNodes(possibleNewNode);
    }
}


void AstarContainer::Astar::explore(Node& currentNode) { // search through each surrounding potential node canidates.
    std::vector<double> V;
    V.clear();
    V.push_back(0);
    V.push_back(0);
    V.push_back(0);
    for (double a = -step_m; a<=step_m; a=a+step_m) {
        for (double b = -step_m; b<=step_m; b=b+step_m) {
            for (double c = -step_m; c<=step_m; c=c+step_m) {
                V[0] = a + currentNode.x;
                V[1] = b + currentNode.y;
                V[2] = c + currentNode.z;
                addNode(V,currentNode); // cordinates, parent node
            }
        }
    }

    if (PathIsCompleted == false) {
        chooseNextNode(pq.top());
    }

    //chooseNextNode(pq.top());
}

void AstarContainer::Astar::chooseNextNode(Node nextNode) {
    if (nextNode.parent->pose != openNodes[nextNode.pose].parent->pose) {
        pq.pop();
        chooseNextNode(pq.top());
    }
    else {
        pq.pop();
        explore(nextNode);
    }
}

void AstarContainer::Astar::addChoosenNodes(Node& node) {
    pq.push(node);
}

void AstarContainer::Astar::finalizePath(Node& node_in_Path) {
    if (node_in_Path.parent->gost == -1) {
        Path.push(node_in_Path);
        Path.push(openNodes[startPos]);
        PathIsCompleted = true;
    }
    else {
        Path.push(node_in_Path); // add node to array of path nodes
        std::cout << node_in_Path.parent->x << " " << node_in_Path.parent->y << " " << node_in_Path.parent->z << std::endl; // print values for plotting
        finalizePath(openNodes[node_in_Path.parent->pose]);
    }
}

bool AstarContainer::Astar::Comp::operator()(const Node& a, const Node& b) {
    if (a.fost > b.fost) {
        return true;
    }
    else if (a.fost < b.fost) {
        return false;
    }
    else {
        if (a.gost > b.gost) {
            return true;
        }
        else if (a.gost < b.gost) {
            return false;
        }
    }
}


void AstarContainer::Astar::displayOpenNodes() {                                        // Printing
    ///*
    for (auto& t : openNodes){
        std::cout << t.second.x << " " << t.second.y << " " << t.second.z << "\n";// << t.second.x << " "  << std::endl;
        //std::cout << t.second.host << std::endl << std::endl;
        //std::cout << t.first[0] << " " << t.first[1] << " " << t.first[2] << "\n \n";// << t.second.x << " "  << std::endl;
    }
//*/
}

// ---------------------- Astar ------------------------------------------------------------------------------------------------


