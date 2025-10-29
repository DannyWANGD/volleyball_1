/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_list.h
 *  Description  : This document contains linked list related functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-02-08 01:19:47
 */


#ifndef LIST_LIB_H
#define LIST_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdlib.h"


// Library version

#define LIST_VERSION "0.2.0"

// Memory management macros
#ifdef LIST_CONFIG_H
#define _STR(x) #x
#define STR(x) _STR(x)
#include STR(LIST_CONFIG_H)
#undef _STR
#undef STR
#endif

#ifndef LIST_MALLOC
#define LIST_MALLOC malloc
#endif

#ifndef LIST_FREE
#define LIST_FREE free
#endif


typedef enum {
	LIST_HEAD,
    LIST_TAIL
} List_DirectionTypeDef;

typedef struct list_node {
  	struct list_node *prev;
  	struct list_node *next;
  	void *val;
} List_NodeTypeDef;

typedef struct {
  	List_NodeTypeDef *head;
  	List_NodeTypeDef *tail;
  	unsigned int len;
  	void (*free)(void *val);
  	int (*match)(void *a, void *b);
} List_TypeDef;

typedef struct {
  	List_NodeTypeDef *next;
  	List_DirectionTypeDef direction;
} List_IteratorTypeDef;


List_NodeTypeDef *List_NodeNew(void *val);
List_TypeDef *List_New(void);
List_NodeTypeDef *List_Rpush(List_TypeDef *self, List_NodeTypeDef *node);
List_NodeTypeDef *List_Lpush(List_TypeDef *self, List_NodeTypeDef *node);
List_NodeTypeDef *List_Find(List_TypeDef *self, void *val);
List_NodeTypeDef *List_At(List_TypeDef *self, int index);
List_NodeTypeDef *List_Rpop(List_TypeDef *self);
List_NodeTypeDef *List_Lpop(List_TypeDef *self);
void List_IteratorDestroy(List_IteratorTypeDef *self);
void List_Remove(List_TypeDef *self, List_NodeTypeDef *node);
void List_Destroy(List_TypeDef *self);
List_IteratorTypeDef *List_IteratorNew(List_TypeDef *list, List_DirectionTypeDef direction);
List_IteratorTypeDef *List_IteratorNewFromNode(List_NodeTypeDef *node, List_DirectionTypeDef direction);
List_NodeTypeDef *List_IteratorNext(List_IteratorTypeDef *self);
void list_iterator_destroy(List_IteratorTypeDef *self);


#ifdef __cplusplus
}
#endif

#endif
