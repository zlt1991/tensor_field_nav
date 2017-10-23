/***
 Implementaion of minimum spanning tree
 ***/
#ifndef  _MST_H
#define  _MST_H
#include "graph.h"
#include <bitset>
#include <climits>
#include <algorithm>
#include <cstdio>

class MSTStrategy
{
public:
    virtual std::vector<graph::Vertex> mst(graph& g) = 0;

protected:
//	void printMST(std::vector<graph::Vertex>& parent, graph& g)
//	{
//		printf("Edge   Weight\n");
//		for (auto i = 1; i < g.vertexCount(); i++)
//			printf("%d - %d    %d \n", parent[i], i, g[i][parent[i]]);
//	}
};

class PrimeMST : public MSTStrategy
{
public:
    std::vector<graph::Vertex> mst(graph& g)
	{
		std::vector<graph::Vertex> parent(g.vertexCount());
		std::vector<graph::Weight> key(g.vertexCount());
		std::vector<bool> mstSet(g.vertexCount());

		for (auto i = 0; i < g.vertexCount(); ++i)
		{
            key[i] = std::numeric_limits<graph::Vertex>::min();
			mstSet[i] = false;
		}
		key[0] = 0;
		parent[0] = -1;

		for (int count = 0; count < g.vertexCount(); ++count)
		{
            auto u = maxKey(key, mstSet, g.vertexCount());
			mstSet[u] = true;
			for (auto v = 0; v != g.vertexCount(); ++v)
			{
                if (g[u][v] && !mstSet[v] && g[u][v] > key[v])
				{
					parent[v] = u;
					key[v] = g[u][v];
				}
			}
		}
        return parent;
        //printMST(parent, g);
	}

private:
    graph::Vertex maxKey(std::vector<graph::Weight>& key,
						 std::vector<bool>&	mstSet,
						 graph::Vertex vCount)
	{
        auto max = std::numeric_limits<graph::Vertex>::min();
        graph::Vertex max_index;

		for (auto v = 0; v < vCount; v++)
            if (mstSet[v] == false && key[v] > max)
                max = key[v], max_index = v;

        return max_index;
	}
};

#endif
