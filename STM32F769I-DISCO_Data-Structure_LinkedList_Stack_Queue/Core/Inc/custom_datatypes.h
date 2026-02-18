/*
 * custom_datatypes.h
 *
 *  Created on: Feb 18, 2026
 *      Author: Debasish Das
 */

#ifndef INC_CUSTOM_DATATYPES_H_
#define INC_CUSTOM_DATATYPES_H_

#include "custom.h"

#define MAX_NODE 10

typedef struct
{
	uint8_t Temperature;
	uint16_t Pressure;
} SensorData_t;

typedef struct
{
	uint8_t SensorNum;
	uint8_t Precision;
} ControlData_t;

typedef enum
{
	SENSOR,
	CONTROL
} DataYype_t;

typedef struct Node_struct
{
	uint32_t Data;
//	union
//	{
//		SensorData_t SensorData;
//		ControlData_t ControlData;
//	}data;
	struct Node_struct *next;
} DataNode_t;

typedef struct LinkedList
{
	DataNode_t *head;
	DataNode_t nodePool[MAX_NODE];	/*This reserve memory for MAX_NODE structure of type DataNode_t*/
	bool nodeUsed[MAX_NODE];
	uint8_t size;
} LinkedList_t;


/*********** Private function prototype starts ****************/

void LinkedList_Init(LinkedList_t *list);
DataNode_t* createLinkedList(LinkedList_t *list, uint32_t data);
DataNode_t *appendNodeAfterHead(LinkedList_t *list, uint32_t data);
DataNode_t* insertdNodeatPositon(LinkedList_t *list, uint8_t pos, uint32_t data);
DataNode_t* deleteNodeatPositon(LinkedList_t *list, uint8_t pos);
void printNodesUptoPosition(LinkedList_t *list, uint8_t pos);

/************Private function prototype ends *******************/

#endif /* INC_CUSTOM_DATATYPES_H_ */
