//============================================================================
// Name        : Dijkstra algorithm.cpp
// Author      : Yiwen Zhou
// Version     :
// Copyright   : Your copyright notice
// Description : Exercise of Dijkstra algorithm in C++, Ansi-style
//============================================================================


#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>

using namespace std;
using std::vector;

const int NUMBER = 50;                  // 50 vertices
const double MAX = 10 * NUMBER;

int mod(int a, unsigned int b)
{
	if(a<0)
	{
		return b + (a%b);
	}
	else
	{
		return a%b;
	}
}

class Graph                                 // definition of a graph
{
public:

	Graph();
	Graph(vector<double> vexs, vector<vector<double> > arcs);
	Graph& operator=(const Graph& other);
	virtual ~Graph();

	int GetV();            //returns the number of vertices in the graph
	int GetE();            //returns the number of edges in the graph
    bool Adjacent(int x, int y);    //tests whether there is an edge from node x to node y
    vector<int> Neighbors (int x);  //lists all nodes y such that there is an edge from x to y
    void Add(int x, int y);        //adds to G the edge from x to y, if it is not there
    void Remove(int x, int y);     //removes the edge from x to y, if it is there
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


class ShortestPath                        //Note: if the START is the vertex u, then the index of vertex w is (w-u)%NUMBER.
{
public:
	ShortestPath();
	ShortestPath(vector<int> done, vector<Path> queue);

	~ShortestPath();

	vector<int> ShowVexReverse(int w);   //find shortest path from START (vertx u) to vertex w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
	double ShowSize(int w);       //return the path cost associated with the shortest path from START (vertx u) to vertex w.
    void Modify(Path P);          //modify the shortest path to P.vex
    void ModifyDone(int x);        // modify the list of vertices that are done
    bool IsDone(int x);          //is the vertex x done?

private:
    vector<int> done_;            //mark the vertices that are already done
    vector<Path> queue_;
};


ShortestPath::ShortestPath()
{
	vector<Path> q;
	queue_=q;
	vector<int> d;
	done_=d;
}

ShortestPath::ShortestPath(vector<int> done, vector<Path> queue)
{
	done_=done;
	queue_=queue;
}

ShortestPath::~ShortestPath()
{
//	cout<<"\n\nShortestPath destructor used\n\n"<< endl;
}

vector<int> ShortestPath::ShowVexReverse(int w)
{
	vector<int> reverse;
	reverse.push_back(w);
	int i=w;
	int u=queue_[0].vex;
	while (i!=u && (queue_[mod((i-u),NUMBER)].prevex)!=-1)
	{
		reverse.push_back(queue_[mod((i-u),NUMBER)].prevex);
		i=queue_[mod((i-u),NUMBER)].prevex;
	}
	if(i==u)
	{
		return reverse;
	}
	else
	{
		vector<int> NO;
		NO.push_back(-1);
		return NO;
	}
}


double ShortestPath::ShowSize(int w)
{
	int u=queue_[0].vex;
	if (queue_[mod((w-u),NUMBER)].length!=MAX)
	{
		return queue_[mod((w-u),NUMBER)].length;
	}
	else
	{
		return -1;
	}
}


void ShortestPath::Modify(Path P)
{
	int i=P.vex;
	int u=queue_[0].vex;
	queue_[mod((i-u),NUMBER)]=P;
}


void ShortestPath::ModifyDone(int x)
{
	bool y=0;
	int i=0;
	for (i=0; i<done_.size(); i++)
	{
		if (x==done_[i])
		{
			y=1;
		}
	}
    if (!y)
    {
    	done_.push_back(x);
    }
}

bool ShortestPath::IsDone(int x)
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



///////////////////////////////////End of definition of ShortestPath//////////////////////////////////////////////



class RoadGraph : public Graph
{

public:
	RoadGraph(vector<double> vexs, vector<vector<double> > arcs, vector<vector<double> > weight, vector<vector<vector<int> > > road);

	~RoadGraph();

	vector<int> ShPath(int u, int w);   //find shortest path between u-w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
	double ShPathSize(int u, int w);       //return the path cost associated with the shortest path.

private:

    vector<vector<double> > weight_;            //path cost associated with the shortest path between any two vertices.
    vector<vector<vector<int> > > road_;    //shortest path between any two vertices

};

RoadGraph::~RoadGraph()
{
	cout<<"\n\nShortestPath destructor used\n\n"<< endl;
}

RoadGraph::RoadGraph(vector<double> vexs, vector<vector<double> > arcs, vector<vector<double> > weight, vector<vector<vector<int> > > road)
  : Graph(vexs, arcs), weight_(weight), road_(road)
{
}

vector<int> RoadGraph::ShPath(int u, int w)
{
	return road_[u][w];
}

double RoadGraph::ShPathSize(int u, int w)
{
	return weight_[u][w];
}


///////////////////////////////////End of definition of RoadGraph (graph with shortest path between any two vertices)//////////////////////////////////////////////



void dijkstra(Graph &graph, ShortestPath &SP, int u)            //u is the starting vertex
{

	Path P(0.0, u, u);
	vector<Path> road;
	road.push_back(P);   //initializing the road from u to v. Road contains the vertex u at the beginning.

	PriorityQueue priorityqueue;


	for (int i=1; i<NUMBER; i++)                        // initializing the priority queue and the road
	{
		if (graph.Get_edge_value(u, (u+i)%NUMBER)!=0)
		{
			Path temp = Path(graph.Get_edge_value(u, (u+i)%NUMBER), u, (u+i)%NUMBER);

			priorityqueue.InsertQueue(temp);
			road.push_back(temp);
		}
		else
		{
			Path temp = Path(MAX, -1, (u+i)%NUMBER);   //prevex is -1 if no path to u+i
			road.push_back(temp);
		}
	}


	vector<int> done;
	done.push_back(u);

    ShortestPath shpath(done, road);     // initializing the shortest path, distance from u to any other vertices
    SP = shpath;



    while(!priorityqueue.IsEmpty())
    {
    	Path current = priorityqueue.TopQueue();
    	int x = current.vex;

    	priorityqueue.MinPriority();

    	SP.Modify(current);
    	SP.ModifyDone(x);

    	for (int i=1; i<NUMBER; i++)
    	{
    		if(!SP.IsDone((x+i)%NUMBER)&& graph.Get_edge_value(x,(x+i)%NUMBER))
    		{

    			Path temp(graph.Get_edge_value(x,(x+i)%NUMBER)+current.length, x, (x+i)%NUMBER);


    			if (!(priorityqueue.ContainsQueue((x+i)%NUMBER)))
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




////////////////////////////////End of Dijkstra function////////////////////////////////////



int main() {
	cout << "Welcome!\n" << endl; // prints !!!Hello World!!!




	vector<double> vexs;
	for (int i=0; i<NUMBER; ++i)
	{
	vexs.push_back(1);
	}                                     // information of the vertices = 1 by default.

	vector<vector<double> > arcs;
//	arcs.resize(NUMBER);

	float density;

	cout << "Input density: (?)%\n" << endl;
	cin >> density;

	srand((int)time(0));
		int i = 0;
		while(i < NUMBER)
		{
			int j = 0;
			vector<double> temp;
			while(j < NUMBER)
			{
				if (i==j)
				{
					temp.push_back(0.0);
				}
				else
				{
					int r = (rand() % 100)+1;
					if (r<=density)
	    			{
						temp.push_back(static_cast<float>(r)/density * 9 + 1);
					}
        				else
    				{
						temp.push_back(0.0);
					}
				}

				j++;
			}
			arcs.push_back(temp);
			i++;
		}


	Graph G = Graph(vexs, arcs);

	cout << "\nThe oriented random graph G has " << G.GetV() << " vertices and " << G.GetE() << " edges \n" << endl;

	cout << "\nThe adjacent matrix of the graph G is: \n" << endl;

	for (int k=0; k<NUMBER; k++)
	{
		for (int l=0; l<NUMBER; l++)
		{
			cout << arcs[k][l] << " , ";
    	}
		cout << endl;
	}                                                   //for testing the adjacent matrix of the graph



    int START;
    int END;

	cout << "\n\n\n[[ Testing Dijkstra's algorithm on shortest path ]].\n\n\n" << endl;

	cout << "For example, shortest paths that starts with vertex 0: \n" << endl;

	ShortestPath SP1;

	dijkstra(G, SP1, 0);


	for (int d=0; d<NUMBER; d++)
	{
		vector<int> finalreverse1;
		finalreverse1 = SP1.ShowVexReverse(d);
		cout << "\nShortest Path from vertex 0 to vertex " <<  d << " is: \n" << endl;
		for (int s=0; s<finalreverse1.size(); s++)
		{
			cout  << " -> " << finalreverse1[finalreverse1.size()-s-1];
		}
		cout << " \n " << endl;
		cout << "The total distance of the above path is " << SP1.ShowSize(d) << " \n" << endl;
	}





	cout << "\n\nNow you can input starting vertex (from 0 to " << NUMBER-1 << " ): (?)\n" << endl;

	cin >> START;

	cout << "\nInput ending vertex (from 0 to " << NUMBER-1 << " ): (?)\n" << endl;
	cin >> END;

	ShortestPath SP;

	dijkstra(G, SP, START);


	vector<int> finalreverse;
	finalreverse = SP.ShowVexReverse(END);

	cout << "\nShortest Path from vertex " << START << " to vertex " <<  END << " is: \n" << endl;
	for (int k=0; k<finalreverse.size(); k++)
	{
		cout  << " -> " << finalreverse[finalreverse.size()-k-1];
	}
	cout << " \n " << endl;

	cout << "The total distance of the above path is " << SP.ShowSize(END) << " \n" << endl;

	return 0;
}
