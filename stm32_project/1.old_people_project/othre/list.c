#include "list.h"

void list_add (struct list_head *_new, struct list_head *head)
{
	_new->prev = head;
	_new->next = head->next;

	_new->prev->next = _new;
	_new->next->prev = _new;
}

void list_add_tail (struct list_head *_new, struct list_head *head)
{
	_new->next = head;
	_new->prev = head->prev;

	_new->prev->next = _new;
	_new->next->prev = _new;
}

void list_del (struct list_head *old)
{
	old->prev->next = old->next;
	old->next->prev = old->prev;

    old->next = (struct list_head *)0xaaaaaaaa;
    old->prev = (struct list_head *)0xbbbbbbbb;

}


int list_empty (struct list_head *head)
{
	return (head->next == head);
}

int list_size(struct list_head *head)
{
	struct list_head* find;
	int num;

	num = 0;
	find = head->next;
	while(find != head){
		num ++;
		find = find->next;
	}

	return num;
}

