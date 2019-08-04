#ifndef _YC_LLIST_H
#define _YC_LLIST_H

#ifdef __cplusplus
extern "C"
{
#endif

//链表结构
struct list_head {
	struct list_head *next;
	struct list_head *prev;
};

typedef struct list_head LHead;

//初始化链表头
#define INIT_LIST_HEAD(head) do {			\
		(head)->next = (head)->prev = head;	\
	} while (0)

#define list_entry(ptr, type, member)					\
	((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

/*遍历链表*/
#define list_for_each(pos, head)				     \
	for (pos = (head)->next; pos != (head); pos = pos->next)



//添加节点到链表，从头添加
void list_add (struct list_head *_new, struct list_head *head);

//从尾部添加设备节点到链表
void list_add_tail (struct list_head *_new, struct list_head *head);

//从链表中删除节点
void list_del (struct list_head *old);

//判断链表是否为空
//return 1:空 ，0;不为空
int list_empty (struct list_head *head);

//获取链表中的设备数量
int list_size(struct list_head *head);


#ifdef __cplusplus
}
#endif
#endif /* _LLIST_H */


