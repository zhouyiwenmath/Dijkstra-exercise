//============================================================================
// Name        : Jarnik-Prim algorithm.cpp
// Author      : Yiwen Zhou
// Version     :
// Copyright   : Your copyright notice
// Description : Jarnik-Prim algorithm, read graph from file, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

//const string name = "/ta/yiwen/workspace/Homework 3/src/graph_data.txt";


class Graph                                 // definition of an graph (un-directed for our purpose)
{
public:

	Graph();
	Graph(const string& name);
	Graph(vector<double> vexs, vector<vector<double> > arcs);
	Graph& operator=(const Graph& other);
	virtual ~Graph();

	int GetV();              //returns the number of vertices in the graph
	int GetE();              //returns the number of edges in the graph
    bool Adjacent(int x, int y);    //tests whether there is an edge from node x to node y
    vector<int> Neighbors (int x);  //lists all nodes y such that there is an edge between x and y
    void Add(int x, int y);        //adds to G the edge between x and y, if it is not there
    void Remove(int x, int y);     //removes the edge between x and y, if it is there
    double Get_node_value (int x);   //returns the value associated with the node x
    void Set_node_value(int x, double a);   //sets the value associated with the node x to a
	double Get_edge_value(int x, int y);    // returns the value associated to the edge (x,y).
	void Set_edge_value (int x, int y, double v);  // sets the value associated to the edge (x,y) to v.

protected:

    int vexcount_;                 //the number of vertices in the graph
    int arccount_;               //the number of edges in the graph
    vector<double> vexs_;         //information (value) of the vertex
    vector<vector<double> > arcs_;    //information (value) of the edges

};

Graph::Graph()                                 // default constructor: empty graph
: arccount_(0), vexcount_(0), vexs_(0), arcs_(0)
{
}

Graph::Graph(const string& name)
{
	vector<int> read;
	ifstream myfile (name.c_str());                    //store the file information into the vector v
		if (myfile.is_open())
	    {
			int a;
			while (myfile >> a)
		    {
		        read.push_back(a);
		    }
		    myfile.close();
	    }
		else
		{
			cout << "Unable to open file" << endl;
		}

	vexcount_=read[0];                           //v[0] is the number of vertices

	vector<double> v (read[0], 1.0);             //initialize the vertices of the default value 1
	vexs_ = v;

	vector<double> row (read[0], 0.0);
	vector<vector<double> > a (read[0], row);      //initialize the adjacent matrix of the correct size
	arcs_ = a;

//	for (int i=0; i<read.size(); i++)    ////////////////////////////////////////////
//	{                                 ////////////////////////////////////////////
//		cout << read[i] << " ; ";        ////////////////////////////////////////////
//	}                                   ////////////////////////////////////////////
//
//	cout << "\n\n" <<  arcs_[0].size() << " ; ";        ////////////////////////////////////////////

	int count=0;

	for (int i=1; i<read.size(); i=i+3)
	{
		arcs_[read[i]][read[i+1]]=arcs_[read[i+1]][read[i]]=static_cast<double>(read[i+2]);
		count++;

//		cout << "\n\n" << i << " ; " <<  arcs_[0].size() << " ; ";        ////////////////////////////////////////////

	}

//	cout << "\n\n" << vexs_.size() << "\n" << a[0].size() << "\n\n" <<  arcs_[0].size() << "\n"<< count << " ; ";        ////////////////////////////////////////////

	arccount_=count;
}


Graph::Graph(vector<double> vexs, vector<vector<double> > arcs)
{
	vexcount_=vexs.size();

	int count=0;
	for (int i=0; i<arcs.size(); ++i)                   // count non-zero entries of the adjacent matrix "arcs"
	{
		for (int j=0; j<arcs.size(); ++j)
		{
			if (arcs[i][j]!=0)
			{
				count++;
			}
		}
	}
	arccount_=count;

	vexs_=vexs;
	arcs_=arcs;
}

Graph& Graph::operator=(const Graph& other)
{
    if (this != &other)
    {
    vexcount_ = other.vexcount_;
    arccount_ = other.arccount_;
    vexs_ = other.vexs_;
    arcs_ = other.arcs_;
    }
    return *this;
}


Graph::~Graph()
{
//	cout<<"\n\nGraph destructor used\n\n"<< endl;
}


int Graph::GetV()
{
	return vexcount_;
}

int Graph::GetE()
{
	return arccount_;
}

bool Graph::Adjacent(int x, int y)   //tests whether there is an edge from node x to node y
{
	if (arcs_[x][y]==0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

vector<int> Graph::Neighbors (int x)  //lists all nodes y such that there is an edge from x to y
{
	vector<int> v;
	if (x<0||x>=vexcount_)
	{
		return v;
	}
	else
	{
		for (int i=0; i<arcs_.size(); ++i)
			{
			    if (arcs_[x][i]!=0)
			    v.push_back(i);
			}
		return v;
	}
}

void Graph::Add(int x, int y)        //adds to G the edge from x to y, if it is not there, WITH DEFAULT VALUE 1
{
	if (arcs_[x][y]==0)
	{
		arcs_[x][y]=1;
	}
}

void Graph::Remove(int x, int y)     //removes the edge from x to y, if it is there
{
	arcs_[x][y]=0;
}

double Graph::Get_node_value (int x)   //returns the value associated with the node x
{
	return vexs_[x];
}

void Graph::Set_node_value(int x, double a)   //sets the value associated with the node x to a
{
	vexs_[x]=a;
}

double Graph::Get_edge_value(int x, int y)    // returns the value associated to the edge (x,y).
{
	return arcs_[x][y];
}

void Graph::Set_edge_value (int x, int y, double v)     //sets the value associated with the edge x to y to v
{
	arcs_[x][y]=v;
}



///////////////////////////////////End of definition of Class Graph//////////////////////////////////////////////




struct Path
{
	double length;                         //priority
	int prevex;                            //the previous vertex in the shortest path (from vertex (fixed) to vertex vex)
	int vex;

	Path();
	Path(double length, int prevex, int vex);
	Path& operator=(const Path& other);
	~Path();
};

Path::Path()
{
	length=0;
	prevex = -1;
	vex=0;
}

Path::Path(double length, int prevex, int vex)
{
	this->length=length;
	this->prevex=prevex;
	this->vex=vex;
}

Path& Path::operator=(const Path& other)
{
	if (this != &other)
	    {
		    length=other.length;
		    prevex=other.prevex;
		    vex=other.vex;
	    }
}

Path::~Path()
{
//	cout<<"\n\nPath destructor used\n\n"<< endl;
}

class PriorityQueue
{
public:
	PriorityQueue();
	PriorityQueue(vector<Path> queue);
	~PriorityQueue();

	void ChangePriority(Path P);      // changes the priority (node value) of queue element.
	void MinPriority();                             // removes the top element of the queue.
	bool ContainsQueue(int vex);                        // does the queue contain queue_element with vex
	bool IsEmpty();                                     // is the priorityqueue empty
	void InsertQueue(Path a);                          // insert queue_element (Path a) into queue
	Path TopQueue();                                 //returns the top element of the queue.
	int SizeQueue();                                 //return the number of queue_elements.

private:
	vector<Path> queue_;

};

PriorityQueue::PriorityQueue()
{
	vector<Path> q;
	queue_=q;
}

PriorityQueue::PriorityQueue(vector<Path> queue)
{
	queue_=queue;
}

PriorityQueue::~PriorityQueue()
{
//	cout<<"\n\nPriorityQueue destructor used\n\n"<< endl;
}

void PriorityQueue::ChangePriority(Path P)         // changes the priority (node value) of queue element, only when the new value is smaller.
{
    int i=0;
    while (i<queue_.size())
    {
    	if (queue_[i].vex==P.vex)
    	{
    		if(queue_[i].length>P.length)
    		{
    			queue_[i].length=P.length;
    			queue_[i].prevex=P.prevex;
   	    		break;
    		}
    	}
    	i++;
    }
}

void PriorityQueue::MinPriority()                             // removes the top element of the queue.
{
	int i=0;
	int p=0;
	double len=queue_[0].length;
	while (i<queue_.size())
	{
	    if (queue_[i].length<len)
	    {
	    	len=queue_[i].length;
	    	p=i;
	    }
	    i++;
	}
	queue_.erase (queue_.begin()+p);
}


bool PriorityQueue::ContainsQueue(int vex)                        // does the queue contain queue_element with vex
{
	int i=0;
	while (i<queue_.size())
	{
		if (queue_[i].vex==vex)
		{
			return true;
		}
		i++;
	}
	return false;
}




bool PriorityQueue::IsEmpty()
{
	if (queue_.size()>0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void PriorityQueue::InsertQueue(Path a)                          // insert queue_element (Path a) into queue
{
	queue_.push_back(a);
}

Path PriorityQueue::TopQueue()                                 //returns the top element of the queue.
{
	int i=0;
	int p=0;
	double len=queue_[0].length;
	while (i<queue_.size())
	{
	    if (queue_[i].length<len)
	    {
	    	len=queue_[i].length;
	    	p=i;
	    }
	    i++;
	}
	return queue_[p];
}

int PriorityQueue::SizeQueue()                                 //return the number of queue_elements.
{
	return queue_.size();
}



///////////////////////////////////End of definition of PriorityQueue//////////////////////////////////////////////

class MinTree                         //always starts with vertex 0!
{
public:
	MinTree();
	MinTree(vector<int> done, vector<Path> queue);

	~MinTree();

	void PrintEdges();   //print the edges in the Minimum Spanding Tree
	double ShowSize();       //return the total cost of the Minimal Spanding Tree
    void Add(Path P);          //extend the minimal tree to P.vex
    bool IsDone(int x);          //is the vertex x in the Minimal Spanding Tree?

private:
    vector<int> done_;            //mark the vertices that are already done
    vector<Path> queue_;
};


MinTree::MinTree()
{
	vector<Path> q;
	queue_=q;
	vector<int> d(1,0);
	done_=d;
}

MinTree::MinTree(vector<int> done, vector<Path> queue)
{
	done_=done;
	queue_=queue;
}

MinTree::~MinTree()
{
//	cout<<"\n\nShortestPath destructor used\n\n"<< endl;
}

void MinTree::PrintEdges()
{
	for (int i=0; i<queue_.size(); i++)
	{
		cout << queue_[i].prevex << "---[" << queue_[i].length << "]---" << queue_[i].vex << endl;
	}
}


double MinTree::ShowSize()
{
	double sum=0;
	for (int i=0; i<queue_.size(); i++)
	{
		sum = sum + queue_[i].length;
	}
	return sum;
}


void MinTree::Add(Path P)
{
	queue_.push_back(P);
	done_.push_back(P.vex);
}


bool MinTree::IsDone(int x)
{
	for (int i=0; i<done_.size(); i++)
	{
		if (x==done_[i])
		{
			return true;
		}
	}
	return false;
}



///////////////////////////////////End of definition of MinTree//////////////////////////////////////////////


void JarnikPrim(Graph &graph, MinTree &MT)
{
	PriorityQueue priorityqueue;

	for (int i=1; i<graph.GetV(); i++)                             // initializing the priority queue
	{
		if (graph.Get_edge_value(0,i)!=0)
		{
			Path temp = Path(graph.Get_edge_value(0,i), 0, i);
			priorityqueue.InsertQueue(temp);
		}
	}

	while(!priorityqueue.IsEmpty())
	    {
	    	Path current = priorityqueue.TopQueue();
	    	int x = current.vex;

	    	priorityqueue.MinPriority();

	    	MT.Add(current);

	    	for (int i=1; i<graph.GetV(); i++)
	    	{
	    		if(!MT.IsDone(i)&& graph.Get_edge_value(x,i))
	    		{

	    			Path temp(graph.Get_edge_value(x,i), x, i);


	    			if (!(priorityqueue.ContainsQueue(i)))
	    			{

	    				priorityqueue.InsertQueue(temp);

	    			}
	    			else
	    			{
	    				priorityqueue.ChangePriority(temp);

	    			}
	    		}
	    	}
	    }

}



////////////////////////////////End of definition of the function Jarnik-Prim//////////////////////////////////

int main() {

	string name;

	cout << "Welcome!\n" << endl; // prints Welcome!

	cout << "The graph will be generated by a .txt file. The file format should be an initial integer that is the node size of the graph and the further values will be integer triples: (i, j, cost). \n" << endl;

	cout << "Please input the address of the .txt file: (ends with .txt)\n" << endl;
	cout << "(e.g.: /ta/yiwen/workspace/Homework 3/src/graph_data.txt)" << endl;

	getline(cin, name);

	Graph G = Graph(name);

		cout << "\nThe un-directed graph G has " << G.GetV() << " vertices and " << G.GetE() << " edges \n" << endl;

		cout << "\nThe adjacent matrix of the graph G is: \n" << endl;

		for (int k=0; k<G.GetV(); k++)
		{
			for (int l=0; l<G.GetV(); l++)
			{
				cout << G.Get_edge_value(k, l) << " , ";
	    	}
			cout << endl;
		}                                                   //for testing the adjacent matrix of the graph

	cout << "\n\n\n[[ Jarnik-Prim's algorithm gives the Minimum Spanning Tree of the above graph as follows: ]]\n\n" << endl;

	MinTree MT;

	JarnikPrim(G, MT);

	cout << "( vertex ---[distance]--- vertex )\n" << endl;

	MT.PrintEdges();

	cout << "\nThe total cost of the Minimum Spanning Tree is:\n" << endl;

	cout << MT.ShowSize() << endl;

	return 0;
}
