#ifndef _GRAPH_H
#define _GRAPH_H
#include <vector>
#include <utility>
#include <cassert>

class graph
{
public:
	typedef long int				  Vertex;
    typedef float				  Weight;
	typedef std::pair<Vertex, Vertex> Edge;

	graph(Vertex vCount = 0)
	{
		graph_.reserve(vCount + 1);

		for (Vertex i = 0; i != vCount; ++i)
		{
			std::vector<Weight> w(vCount);
			graph_.push_back(w);
		}
	}
	
	Vertex vertexCount() const
	{
		return graph_.size();
	}

	std::vector<Weight>& operator[](Vertex v)
	{
		return graph_[v];
	}

	void addEdge(Vertex v1, Vertex v2, Weight w)
	{
		assert(v1 < vertexCount());
		assert(v2 < vertexCount());

		graph_[v1][v2] = w;
	}

private:
	std::vector<std::vector<Weight>>  graph_;
};

#endif
