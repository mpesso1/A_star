#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include "A_star.h"


using namespace std;

struct NodeCostPointer {
    bool operator()(star::Node* const& a,star::Node* const& b){
        if (std::tie(a->cordinates(0), a->cordinates(1), a->cordinates(2)) == std::tie(b->cordinates(0), b->cordinates(1), b->cordinates(2))) {
            return (std::tie(a->cordinates(0), a->cordinates(1), a->cordinates(2)) == std::tie(b->cordinates(0), b->cordinates(1), b->cordinates(2)));
        }
        else {
            return std::tie(a->f_cost,a->h_cost,a->cordinates(0)) < std::tie(b->f_cost,b->h_cost,b->cordinates(0));
        }
    }
};


int main() {

    Eigen::Matrix<float,3,1> obj; // int --> float
    obj(0,0) = 10.2;
    obj(1,0) = 9.6;
    obj(2,0) = 10.2;

    Eigen::Matrix<float,3,1> end; // int --> float
    end(0) = 9;
    end(1) = 9;
    end(2) = 9;

    Eigen::Matrix<float,3,1> start; // double --> float
    start(0) = 12;
    start(1) = 10;
    start(2) = 12;

    bool Astar = true;

    if (Astar) {
        star::A_star A(start, end, obj, .3);
    }
    //star::A_star A(start, end, obj, .5);

    if (!Astar) {
        cout << "Begin creating nodes\n";

        map<star::Node*,star::Node*> test; //openNdes

        cout << "map created\n";

        star::Node* nodetest = new star::Node(start,0,0);

        cout << "node 1 created\n";

        star::Node* nodetest2 = new star::Node(start,0,0);

        // cout << ((*nodetest2) == (nodetest)) << endl;

        cout << "node 2 created\n";

        star::Node* nodetest3 = new star::Node(start,5,std::numeric_limits<float>::infinity());

        cout << "node 3 created\n";

        test[nodetest] = nodetest3;

        cout << "node 3 stored in map w/ node 1 as key\n";

        if (test.find(nodetest2) != test.end()) {
            cout << "key already exist\n\n";
        }

        nodetest2->closedNode = true;

        test[nodetest2] = nodetest3;

        cout << "node 3 stored in map w/ node 2 as key\n";

        // cout << ((*test[nodetest2] < (*test[nodetest]))) << endl;

        cout << "direct comparison on map has been made\n";

        test[nodetest]->f_cost = 3;

        // cout << "direct assignmnet to map has been made \n";

        cout << test[nodetest]->cordinates << endl;

        // star::Node* fuckyouKey = new star::Node(start, 7, 0);

        // test[fuckyouKey] = fuckyouKey;

        cout << "iteration begins\n";

        map<star::Node*,star::Node*,NodeCostPointer>::iterator itt;
        for(itt=test.begin();itt!=test.end();itt++){
            cout << itt->first->cordinates << endl;

            cout << "  \n";
        }

        cout << "iteration ends\n";

        // cout << ((*nodetest2) < (*nodetest)) << endl;

        cout << "direct comparison on nodes has been made\n";

        cout << "number of key value pairs: " << test.size() << endl;

        std::cout << "done with script\n";
    }
    


    // map<star::Node*, star::Node*> yo;

    // yo[new star::Node(start,5,5)] = new star::Node(end,3,2);

    // map<int,int> name;
    // name[4] = 7;



/*
Will map current Node to parent Node.  

*/

//    map<star::Node, star::Node, NodeCost> opened;

//    star::Node data;

//     Eigen::Matrix<star::Node,Eigen::Dynamic,Eigen::Dynamic> just;
//     // just.resize(3,1);
//     // just(0) = data;

/*

Astar algorith:

    Add start node to opened list

    Add objects to opened list defining their parent as dead node and their cost as infinite


    loop:

    determine smallest cost in opened list

    search around that node and add that node to closed list

    check if searched cordinates match the end cordinates.  if they do match then path complete. iterate through closed list following chain of parents from end node storing each nodes cordinates in lsit

    add each serched node to openlist defining their parent node as the current node. if serached node is already in openlist then check cost. if cost is lower then update parent node (check then store)





*/

    // cout << (std::numeric_limits<float>::infinity()) << endl;

    return 0;
}