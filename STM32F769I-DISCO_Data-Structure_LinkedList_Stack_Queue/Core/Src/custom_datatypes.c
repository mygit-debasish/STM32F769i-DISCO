/*
 * custom_datatypes.c
 *
 *  Created on: Feb 18, 2026
 *      Author: Debasish Das
 */
#include "custom_datatypes.h"

void LinkedList_Init(LinkedList_t *list)
{
	list->head = NULL;

	for (uint8_t i = 0; i < MAX_NODE; i++)
	{
		list->nodeUsed[i] = false;
	}
}

DataNode_t* createLinkedList(LinkedList_t *list, uint32_t data)
{
	if (!list)
		return NULL;

	for (uint8_t i = 0; i < MAX_NODE; i++)
	{
		if (list->nodeUsed[i] == false)
		{
			list->nodeUsed[i] = true;
			list->nodePool[i].Data = data;
			list->nodePool[i].next = NULL;
			//list->size++;

			return &list->nodePool[i];
		}
	}
	return NULL;
}

DataNode_t* appendNodeAfterHead(LinkedList_t *list, uint32_t data)
{
	DataNode_t *tmpNode = list->head;

	if (!list)
		return NULL;

	DataNode_t *newDataNode = createLinkedList(list, data);
	if (!newDataNode)
		return NULL;
	newDataNode->next = NULL;

	while (tmpNode->next != NULL)
	{
		tmpNode = tmpNode->next;
	}

	newDataNode->next = tmpNode->next;
	tmpNode->next = newDataNode;
	list->size++;

	return newDataNode;
}

DataNode_t* insertdNodeatPositon(LinkedList_t *list, uint8_t pos, uint32_t data)
{
	uint8_t index = 0;
	DataNode_t *newNode = createLinkedList(list, data);
	newNode->next = NULL;

	if (list->head == NULL)
	{
		list->head = newNode;
		return newNode;
	}
	DataNode_t *tmpNode = list->head;
	/* Reaching the end of the node */
	while (index < pos - 1)
	{
		tmpNode = tmpNode->next;
		index++;
	}
	newNode->next = tmpNode->next;
	tmpNode->next = newNode;
	list->size++;

	return newNode;
}

void printNodesUptoPosition(LinkedList_t *list, uint8_t pos)
{
	uint8_t count = 0;

	if (!list || pos >= list->size)

	{
		writeFormatData(&huart1, "Position Error: %s() Line: %d \r\n", __func__,
				__LINE__);
		return;
	}

	DataNode_t *currNode = list->head;
	while (currNode && count < pos)
	{
		writeFormatData(&huart1, "Address: %p  Data : %d\r\n", (void*) currNode,
				currNode->Data);
		currNode = currNode->next;
		count++;
	}
}

DataNode_t* deleteNodeatPositon(LinkedList_t *list, uint8_t pos)
{
	uint8_t index = 0;
	DataNode_t *nodeDelete;

	if (!list || !list->head)
	{
		writeFormatData(&huart1, "NULL Error: %s() Line: %d \r\n", __func__,
				__LINE__);
		return NULL;
	}

	DataNode_t *currNode = list->head;

	if (pos >= list->size)
	{
		writeFormatData(&huart1, "Postion Error: %s %d", __func__, __LINE__);
		return NULL;
	}

	if (pos == 0)
	{
		list->head = currNode->next;
		list->size--;
		return currNode; /* Return deleted node */
	}

	/* Traversing the end of the node */
	while (currNode && index < pos - 1)
	{
		currNode = currNode->next;
		index++;
	}
	nodeDelete = currNode->next;

	if (!nodeDelete)
		return NULL;

	currNode->next = nodeDelete->next;
	list->size--;

	return nodeDelete;
}

