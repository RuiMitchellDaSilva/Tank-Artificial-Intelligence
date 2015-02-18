//DFS + BFS PROGRAM
#include <iostream>       // std::cout
#include <queue>
#include <stack>
#include "GraphNode.h"

using namespace std;

void SearchBFS(GraphNode* rootNode)
{
	std::queue<GraphNode*> myQueue;
	myQueue.push(rootNode);

	while (!myQueue.empty())
	{
		cout << myQueue.front()->GetID() << endl;
		myQueue.front()->SetVisited(true);
		GraphNode* node = myQueue.front();
		myQueue.pop();

		int j = node->GetChildNodes().size();

		for (int i = 0; i < node->GetChildNodes().size(); i++)
		{
			if (node->GetChildNodes()[i] != NULL)
				if (!node->GetChildNodes()[i]->GetVisited())
					myQueue.push(node->GetChildNodes()[i]);
		}
	}

	cout << endl;
}

//------------------------------------------------------------------------------------

void SearchDFS(GraphNode* rootNode)
{
	std::stack<GraphNode*> myStack;
	myStack.push(rootNode);

	while (!myStack.empty())
	{
		cout << myStack.top()->GetID() << endl;
		myStack.top()->SetVisited(true);
		GraphNode* node = myStack.top();
		myStack.pop();
		
		int j = node->GetChildNodes().size();

		for (int i = 0; i < node->GetChildNodes().size(); i++)
		{
			if (node->GetChildNodes()[i] != NULL)
				if (!node->GetChildNodes()[i]->GetVisited())
					myStack.push(node->GetChildNodes()[i]);
		}
	}

	cout << endl;
}

//------------------------------------------------------------------------------------

void ReverseSearchDFS(GraphNode* rootNode)
{
	std::stack<GraphNode*> myStack;
	myStack.push(rootNode);

	while (!myStack.empty())
	{
		cout << myStack.top()->GetID() << endl;
		myStack.top()->SetVisited(true);
		GraphNode* node = myStack.top();
		myStack.pop();

		int j = node->GetChildNodes().size();

		for (int i = node->GetChildNodes().size() - 1; i >= 0; i--)
		{
			if (node->GetChildNodes()[i] != NULL)
			if (!node->GetChildNodes()[i]->GetVisited())
				myStack.push(node->GetChildNodes()[i]);
		}
	}

	cout << endl;
}

//------------------------------------------------------------------------------------

//void DijkstraSearch(GraphNode* rootNode, char goal) // cant do this search in this example
//{
//	std::vector<GraphNode*> closedList;
//	std::vector<GraphNode*> frontier;
//
//	GraphNode* startingNode = rootNode;
//	startingNode->SetCostToReach(0);
//
//	closedList.push_back(rootNode);
//
//	for (int i = 0; i < startingNode->GetChildNodes().size(); i++)
//		frontier.push_back(startingNode->GetChildNodes()[i]);
//
//	if (frontier.size() == 0)
//		int i = 0; // Exit search, no path found
//
//
//	//while (path not found)
//	//{
//		GraphNode* bestNode = frontier.at(0);
//
//		for (int i = 1; frontier.size(); i++)
//		{
//			if (frontier.at(i)->GetCostToReach() > bestNode->GetCostToReach())
//				bestNode = frontier.at(i);
//		}
//
//		closedList.push_back(bestNode);
//
//		if (bestNode->GetID() == 'E')
//			int i = 0; // Exit search
//
//		for (int i = 0; i < bestNode->GetChildNodes().size(); i++)
//		{
//			GraphNode* validConnectedNode = bestNode->GetChildNodes()[i];
//
//			bool isWithinFrontier = false;
//			bool isWithinClosedList = false;
//
//			for (int i = 0; i < frontier.size() && !isWithinFrontier; i++) // Added another conditional statement
//			{
//				if (validConnectedNode == frontier.at(i))
//				{
//					// Check for lower g-value
//					frontier.erase(frontier.begin() + i);
//					isWithinFrontier = true;
//				}
//
//			}
//
//			for (int i = 0; i < closedList.size() && !isWithinClosedList; i++) // Added another conditional statement
//			{
//				if (validConnectedNode == closedList.at(i))
//				{
//					// Check for lower g-value
//					closedList.erase(closedList.begin() + i);
//					isWithinClosedList = true;
//				}
//
//			}
//
//			if (!isWithinFrontier && !isWithinClosedList)
//			{
//				validConnectedNode->SetCostToReach(current cost + parent cost);
//				frontier.push_back(validConnectedNode);
//			}
//		}
//	//}
//}

//------------------------------------------------------------------------------------

int main()
{
	//-----------------------------------------------------------------------------
	//Set up tree structure from lecture.
	GraphNode* pNodeA = new GraphNode('A');
	pNodeA->SetCostToReach(9999);
	GraphNode* pNodeB = new GraphNode('B');
	pNodeB->SetCostToReach(9999);
	GraphNode* pNodeC = new GraphNode('C');
	pNodeC->SetCostToReach(9999);
	GraphNode* pNodeD = new GraphNode('D');
	pNodeD->SetCostToReach(9999);
	pNodeA->AddAChildNode(pNodeB);
	pNodeA->AddAChildNode(pNodeC);
	pNodeA->AddAChildNode(pNodeD);

	GraphNode* pNodeE = new GraphNode('E');
	pNodeE->SetCostToReach(9999);
	GraphNode* pNodeF = new GraphNode('F');
	pNodeF->SetCostToReach(9999);
	pNodeB->AddAChildNode(pNodeE);
	pNodeB->AddAChildNode(pNodeF);

	GraphNode* pNodeG = new GraphNode('G');
	pNodeG->SetCostToReach(9999);
	pNodeC->AddAChildNode(pNodeG);

	GraphNode* pNodeH = new GraphNode('H');
	pNodeH->SetCostToReach(9999);
	GraphNode* pNodeI = new GraphNode('I');
	pNodeI->SetCostToReach(9999);
	pNodeD->AddAChildNode(pNodeH);
	pNodeD->AddAChildNode(pNodeI);
	//-----------------------------------------------------------------------------


	//Depth First Search.
	//ReverseSearchDFS(pNodeA);

	//Breadth First Search.
	SearchBFS(pNodeA);

	int stop;
	cin >> stop;

	return 0;
}

