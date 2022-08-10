#ifndef ASTAR_h
#define ASTAR_H
#include <Eigen/Dense>
#include <vector>
#include <stack>
#include <map>
#include <bits/stdc++.h>
#include <queue>

namespace AstarContainer {
    class Node {
        public:
            Node* parent; // parent node

            double gost; // from node to root -> G cost

            double host; // from node to end -> H cost

            double fost; // gost + host -> F cost

            double x; // x cordinate of node

            double y; // y cordinate of node

            double z; // z cordinate of node

            std::vector<double> pose;

            Node(); // Default Contructor

            Node(double, double, double, Node&, double); // Overload Constructor

            ~Node(); // Destructor

            double gostCalc(); // calc gost value

            double fostCalc();  // calc fost value
    };

    class Astar {
        private:

            Eigen::Matrix<int,3,1> endXYZ_mm;
            std::vector<double> endPos;
            std::vector<double> startPos;

            Eigen::Matrix<double,3,3> objectContainer;

            bool PathIsCompleted;

            std::stack<Node> Path;

            std::stack<Node> choosenNodes;

            double step_m; // step size in meters

            int step_mm; // step size in millimeters --> CAN NOT CONTAIN DECIMALS

            int cushion; // number of step given for cushion from obsticals


            Node deadNode; // used for initiating object nodes

            Node nullNode; // use to initites non object nodes without parents i.e. end/goal node

        public:
            std::vector<std::vector<double>> objectVector;

            std::vector<std::vector<double>> initobjectVector;


            static int count;

            struct Comp {
                bool operator()(const Node&, const Node&);
            };

            Eigen::Matrix<int,3,3> objectXYZ_mm; // [mm] matrix containing object cordinates in millimeters --> CAN NOT CONTAIN DECIMALS

            std::priority_queue<Node, std::vector<Node>, Comp> pq;

            std::map<std::vector<double>,Node> openNodes; // defined/opened nodes.. including objects, start/finish... all node types

            Astar(Eigen::Matrix<int,3,3>, Eigen::Matrix<int,3,1>, int, int, Eigen::Matrix<double,3,1>); // Overide Constructor

            ~Astar(); // Destructor

            void enlargeObjects(int);

            double nodeHost(double,double,double); // needed to determine the host value for node.  I kept the end cordinaes out of the node class so I had to do it this way

            void search(Node&); // main Astar algorithm

            void shiftObjects(); // shifts the cordinates of teh object nodes to fit step

            void shiftEndNode(); // shifts cordinates of end goal node to fit step

            void addObjectCushion(); // adds objects for cushion between obsticals and robot

            void addNode(std::vector<double>,Node&); // checks map for cordinates and adds node if cordinates have not been opened

            void explore(Node&); // explores nearby nodes 

            void generatePath(); // generates loop until Astar converges

            void finalizePath(Node&);

            void addChoosenNodes(Node&); //,std::vector<double>);

            void checkGostIsMinimum(std::vector<double>,Node&);

            void chooseNextNode(Node);

            bool CompareVectors(std::vector<double>, std::vector<double>, double);

            bool ObjectCheck(std::vector<std::vector<double>>, std::vector<double>, double);

            //bool std::operator()(const Node&, const Node&);

            void displayOpenNodes();

    };
}

#endif // ASTAR_H