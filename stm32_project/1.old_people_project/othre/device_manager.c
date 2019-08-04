
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "device_manager.h"
#include "list.h"
#define num_of_list 2
//设备管理结构
typedef struct{
	LHead head;
	int lock;
}DevManCtx;

//设备节点
typedef struct{
	LHead node;
	NDevice dev;
}DeviceNode;


static DevManCtx _DevManCtx[num_of_list];
static int _DevManInited = 0;




void Lock_Init(int *lock)
{
	*lock=0;
}
void Lock_Lock(int *lock)
{
	*lock=1;
}
void Lock_ULock(int *lock)
{
	*lock=0;
}
int Lock_check(int *lock)
{
	return *lock;
}


int dev_man_add(int type, NDevice newdev)
{
	DeviceNode *pDevNode;
	LHead *pos;
	if (type >= num_of_list)return -1;
	if (!_DevManInited)return -1;

	Lock_Lock(&_DevManCtx[type].lock);
	list_for_each(pos,&_DevManCtx[type].head)//遍历链表
	{

		pDevNode = (DeviceNode*)pos;
		//通过mac判定，指定的设备存在则退出，否则，添加设备
		if (0 == strncmp(pDevNode->dev.id,newdev.id,sizeof(pDevNode->dev.id)))
		{
			break;
		}
	}

	if (pos == &_DevManCtx[type].head){
		pDevNode = (DeviceNode*)malloc(sizeof(DeviceNode));//申请设备空间
		if (NULL == pDevNode)
		{
			Lock_ULock(&_DevManCtx[type].lock);
			return -1;
		}
		memcpy(&pDevNode->dev,&newdev,sizeof(newdev));
		list_add(&pDevNode->node,&_DevManCtx[type].head);   //将设备加入到链表中
	}
	Lock_ULock(&_DevManCtx[type].lock);

	return 0;
}

int dev_man_modify(int type,NDevice newdev)
{
	DeviceNode *pDevNode;
	LHead *pos;
	if (type >= num_of_list)return -1;
	if (!_DevManInited)return -1;
	Lock_Lock(&_DevManCtx[type].lock);
	list_for_each(pos,&_DevManCtx[type].head)//遍历链表
	{

		pDevNode = (DeviceNode*)pos;
		if (0 == memcmp(pDevNode->dev.id,newdev.id,sizeof(pDevNode->dev.id)))
		{
			break;
		}
	}

	if (pos != &_DevManCtx[type].head)
	{
		memcpy(&pDevNode->dev,&newdev,sizeof(pDevNode->dev));
	}

	Lock_ULock(&_DevManCtx[type].lock);

	return 0;
}




int dev_man_get_device_by_id(int type,unsigned char *id,NDevice *outDev)
{
	DeviceNode *pDevNode = NULL;
	LHead *pos;
	
	if (type >= num_of_list)return -1;
	if (NULL == id || NULL == outDev)return -1;
	if (!_DevManInited)return -1;

	Lock_Lock(&_DevManCtx[type].lock);
	list_for_each(pos,&_DevManCtx[type].head)//遍历链表，查找设备
	{
		pDevNode = (DeviceNode*)pos;
		//查找匹配的mac设备
		if (0 == memcmp(pDevNode->dev.id,id,sizeof(pDevNode->dev.id)))
		{
			break;
		}
	}
	Lock_ULock(&_DevManCtx[type].lock);
	if (pos == &_DevManCtx[type].head)return -1;
	memcpy(outDev,&pDevNode->dev,sizeof(pDevNode->dev));

	return 0;
}

int dev_man_del_by_id(int type,unsigned char *id)
{
	DeviceNode *pDevNode = NULL;
	LHead *pos;
	int delFlag = 0;
	if (type >= num_of_list)return -1;
	if (NULL == id)return -1;
	if (!_DevManInited)return -1;

	Lock_Lock(&_DevManCtx[type].lock);
	list_for_each(pos,&_DevManCtx[type].head)//遍历链表
	{
		pDevNode = (DeviceNode*)pos;
		//查找指定的mac的设备，存在，则从链表中删除、更新配置文件、并释放设备空间，若不存在，则退出
		if (0 == memcmp(&pDevNode->dev.id,id,sizeof(pDevNode->dev.id)))
		{
			list_del(pos);//从链表中删除该节点
			delFlag = 1;
			break;
		}
	}
	Lock_ULock(&_DevManCtx[type].lock);
	if (pos != &_DevManCtx[type].head){
		if (pDevNode && 1 == delFlag)
		{
			free(pDevNode);//释放空间
		}
	}



	return 0;
}


int dev_man_get_devices_num(int type)
{
	int num;
	if (type >= num_of_list)return -1;
	if (!_DevManInited)return -1;

	Lock_Lock(&_DevManCtx[type].lock);
	num = list_size(&_DevManCtx[type].head);
	Lock_ULock(&_DevManCtx[type].lock);

	return num;
}

int dev_man_init(void)
{
	if (_DevManInited)return 0;
	int i;
	for(i=0;i<num_of_list;i++)
	{
		INIT_LIST_HEAD(&_DevManCtx[i].head);//初始化链表
		Lock_Init(&_DevManCtx[i].lock);//初始化锁
	}
	_DevManInited = 1;

	return 0;
}



