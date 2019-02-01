//此处代码源自CSDN，地址不记得了。。。。。。。
#include <algorithm>
#include "Astar.h"
#include <limits.h>

int CAStar::m_moveCostHorizontal = 5;
int CAStar::m_moveCostDiagonal = 7;

DIRECTION g_direct[] = {
	//{-1, -1}, //left up
	//{-1, +1}, //left down
	//{+1, -1}, //right up
	//{+1, +1}, //right down
	{ -1, 0 }, //left
	{ +1, 0 }, //right
	{ 0, -1 }, //up
	{ 0, +1 }, //down
};

CAStar::CAStar() :m_startNode(0, 0), m_endNode(0, 0)
{
	m_CallBack = NULL;
}

CAStar::CAStar(int nColumn, int nRow) : m_startNode(0, 0), m_endNode(0, 0)
{
	m_nColumn = nColumn;
	m_nRow = nRow;
	m_CallBack = NULL;
}

CAStar::~CAStar(void)
{
}

void CAStar::SetRange(int nColumn, int nRow)
{
	m_nColumn = nColumn;
	m_nRow = nRow;
}

void CAStar::SetBlock(vector<NODE> &vecBlock)
{
	m_barrier.clear();
	for (unsigned int i = 0; i < vecBlock.size(); i++)
	{
		m_barrier.push_back(vecBlock[i]);
	}
}

void CAStar::SetStartAndEnd(NODE startNode, NODE endNode)
{
	m_startNode = startNode;
	m_endNode = endNode;
	m_open.clear();
	m_close.clear();
	CalcValue(startNode);
	m_open.push_back(startNode);
}

void CAStar::GetMinFromOpen(vector<NODE> &minVec)
{
	minVec.clear();
	vector<NODE>::iterator it = m_open.begin();
	int fMin = INT_MAX;
	for (; it != m_open.end(); it++)
	{
		if (it->f < fMin)
			fMin = it->f;
	}
	for (it = m_open.begin(); it != m_open.end(); )
	{
		if (it->f == fMin)
		{
			minVec.push_back(*it);
			it = m_open.erase(it);
			continue;
		}
		it++;
	}
}

int CAStar::SetCallBack(int(*pFun)(void *), void *pThis)
{
	m_CallBack = pFun;
	m_pArg = pThis;
	return 1;
}

bool CAStar::IsFindPath()
{
	return m_bFind;
}

void CAStar::FindPath()
{
	m_bFind = false;
	vector<NODE> minVec;
	while (!m_bFind)
	{
		minVec.clear();
		GetMinFromOpen(minVec);
		if (minVec.empty())
			break;
		m_close.insert(m_close.end(), minVec.begin(), minVec.end());
		for (vector<NODE>::iterator it = minVec.begin(); it != minVec.end(); it++)
			DoNeighbors(*it);

		if (m_bFind)
			GetFinalShortestPath();

		if (m_CallBack) m_CallBack(m_pArg);
	}
}

void CAStar::GetFinalShortestPath()
{
	NODE node = m_close[m_close.size() - 1];
	m_path.clear();
	while (true)
	{
		DIRECTION direct = node.parent;
		NODE parentNode = node;
		parentNode.nX -= direct.x;
		parentNode.nY -= direct.y;

		for (unsigned int i = 0; i < m_close.size(); i++)
		{
			if (m_close[i].nX == parentNode.nX && m_close[i].nY == parentNode.nY)
			{
				node = m_close[i];
				if (node.nX == m_startNode.nX && node.nY == m_startNode.nY)
					return;
				m_path.push_back(node);
				break;
			}
		}
	}
}

void CAStar::CalcValue(NODE &node)
{
	node.g = GetDistance(node, m_startNode);
	node.h = GetDistance(node, m_endNode);
	node.f = node.g + node.h;
}

int CAStar::GetDistance(NODE &node1, NODE &node2)
{
	int xSub = abs(node1.nX - node2.nX);
	int ySub = abs(node1.nY - node2.nY);

	return  abs(xSub - ySub) * m_moveCostHorizontal +
		min(xSub, ySub) * m_moveCostDiagonal;
}

void CAStar::DoNeighbors(NODE &node)
{

	for (int i = 0; i < sizeof(g_direct) / sizeof(g_direct[0]); i++)
	{
		NODE nodeNeighbor = node;
		nodeNeighbor.nX += g_direct[i].x;
		nodeNeighbor.nY += g_direct[i].y;
		DoNeighborNode(node.g, &g_direct[i], nodeNeighbor);
	}
}

void CAStar::DoNeighborNode(int gValue, DIRECTION *direct, NODE &node)
{
	if (!IsValid(direct, node))
		return;

	if (InClose(node.nX, node.nY))
		return;

	node.parent = *direct;
	if (direct->x & direct->y)
		node.g = gValue + m_moveCostDiagonal;
	else
		node.g = gValue + m_moveCostHorizontal;

	node.h = GetDistance(node, m_endNode);
	node.f = node.g + node.h;
	NODE *pNode = NULL;
	if (InOpen(node.nX, node.nY, &pNode))
	{
		if (node.f < pNode->f)
			*pNode = node;
	}
	else
	{
		m_haveNeighbor = true;
		m_open.push_back(node);
		if (node.nX == m_endNode.nX && node.nY == m_endNode.nY)
		{
			m_close.push_back(node);
			m_bFind = true;
		}
	}
}

bool CAStar::IsValid(DIRECTION *direct, NODE &node)
{
	if (node.nX < 0 || node.nX >= m_nColumn ||
		node.nY < 0 || node.nY >= m_nRow)
		return false;

	if (InBarrier(node.nX, node.nY))
		return false;

	if (direct->x & direct->y)
	{
		if (InBarrier(node.nX - direct->x, node.nY) ||
			InBarrier(node.nX, node.nY - direct->y))
		{
			return false;
		}
	}

	return true;
}

bool CAStar::InBarrier(int x, int y, NODE **pNode)
{
	for (unsigned int i = 0; i < m_barrier.size(); i++)
	{
		if (m_barrier[i].nX == x && m_barrier[i].nY == y)
		{
			if (pNode)
				*pNode = &m_barrier[i];
			return true;
		}
	}
	return false;
}

bool CAStar::InOpen(int x, int y, NODE **pNode)
{
	for (vector<NODE>::iterator it = m_open.begin(); it != m_open.end(); it++)
	{
		if (it->nX == x && it->nY == y)
		{
			if (pNode)
				*pNode = &(*it);
			return true;
		}
	}
	return false;
}

bool CAStar::InClose(int x, int y, NODE **pNode)
{
	for (vector<NODE>::iterator it = m_close.begin(); it != m_close.end(); it++)
	{
		if (it->nX == x && it->nY == y)
		{
			if (pNode)
				*pNode = &(*it);
			return true;
		}
	}
	return false;
}

void CAStar::ClearAllList()
{
	m_close.clear();
	m_open.clear();
	m_path.clear();
	m_bFind = false;
}
