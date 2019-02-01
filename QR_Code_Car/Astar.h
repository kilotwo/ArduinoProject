#ifndef ASTAR_H_INCLUDED
#define ASTAR_H_INCLUDED

#pragma once
#include <vector>
#include <iostream>

using namespace std;

struct DIRECTION
{
	int x;
	int y;
};

typedef struct _NODE
{
	int nX;
	int nY;
	DIRECTION parent;
	int f;
	int g;
	int h;
	_NODE(int x, int y)
	{
		nX = x;
		nY = y;
		f = g = h = 0;
	}
	_NODE() { nX = nY = f = g = h = 0; }
}NODE, *PNODE;

class CAStar
{
public:
	CAStar();
	CAStar(int nColumn, int nRow);
	void SetRange(int nColumn, int nRow);
	void SetStartAndEnd(NODE startNode, NODE endNode);
	void SetBlock(vector<NODE> &vecBlock);
	void FindPath();
	int SetCallBack(int(*pFun)(void *), void *pThis);
	bool IsFindPath();
	~CAStar(void);

	const vector<NODE>* GetPathVector()
	{
		return &m_path;
	}
	const vector<NODE>* GetOpenVector()
	{
		return &m_open;
	}
	const vector<NODE>* GetCloseVector()
	{
		return &m_close;
	}

	bool InOpen(int x, int y, NODE **pNode = NULL);
	bool InClose(int x, int y, NODE **pNode = NULL);
	bool InBarrier(int x, int y, NODE **pNode = NULL);

	void ClearAllList();

private:
	void CalcValue(NODE &node);
	int GetDistance(NODE &node1, NODE &node2);
	void GetMinFromOpen(vector<NODE> &minVec);
	void DoNeighbors(NODE &node);
	void DoNeighborNode(int gValue, DIRECTION *direct, NODE &node);
	bool IsValid(DIRECTION *direct, NODE &node);
	void GetFinalShortestPath();

	int m_nColumn;
	int m_nRow;

	NODE m_startNode;
	NODE m_endNode;

	vector<NODE> m_open;
	vector<NODE> m_close;
	vector<NODE> m_barrier;
	vector<NODE> m_path;

	static int m_moveCostHorizontal;
	static int m_moveCostDiagonal;

	bool m_bFind;
	bool m_haveNeighbor;

	int(*m_CallBack)(void *);
	void *m_pArg;
};

#endif // ASTAR_H_INCLUDED
