/*
 *  Project      : Polaris Robot
 *
 *  FilePath     : lib_list.c
 *  Description  : This document contains linked list related functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-08-04 03:01:34
 */

/*  
Notice :
    (The MIT License)

    Copyright (c) 2009-2010 TJ Holowaychuk <tj@vision-media.ca>

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    'Software'), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    This linked list function library is referenced from https://github.com/clibs/list
    In order to adapt to the uniform code file specification, the case of some 
    functions and variable naming specifications have been changed, and the newline 
    format has been slightly modified, but the code content has not been modified.
*/

#include "lib_list.h"


/*
 * Allocate a new List_TypeDef. NULL on failure.
 */
List_TypeDef *List_New(void) {
    List_TypeDef *self;
    if ((self = LIST_MALLOC(sizeof(List_TypeDef))) == NULL)
        return NULL;
    self->head = NULL;
    self->tail = NULL;
    self->free = NULL;
    self->match = NULL;
    self->len = 0;
    return self;
}


/*
 * Free the list.
 * @self: Pointer to the list
 */
void List_Destroy(List_TypeDef *self) {
    unsigned int len = self->len;
    List_NodeTypeDef *next;
    List_NodeTypeDef *curr = self->head;

    while (len--) {
        next = curr->next;
        if (self->free)
            self->free(curr->val);
        LIST_FREE(curr);
        curr = next;
    }

    LIST_FREE(self);
}



/*
 * Append the given node to the list
 * and return the node, NULL on failure.
 * @self: Pointer to the list for popping node
 * @node: the node to push
 */
List_NodeTypeDef *List_Rpush(List_TypeDef *self, List_NodeTypeDef *node) {
    if (!node)
        return NULL;

    if (self->len) {
        node->prev = self->tail;
        node->next = NULL;
        self->tail->next = node;
        self->tail = node;
    }
    else {
        self->head = self->tail = node;
        node->prev = node->next = NULL;
    }

    ++self->len;
    return node;
}


/*
 * Return / detach the last node in the list, or NULL.
 * @self: Pointer to the list for popping node
 */
List_NodeTypeDef *List_Rpop(List_TypeDef *self) {
    if (!self->len)
        return NULL;

    List_NodeTypeDef *node = self->tail;

    if (--self->len) {
        (self->tail = node->prev)->next = NULL;
    }
    else {
        self->tail = self->head = NULL;
    }

    node->next = node->prev = NULL;
    return node;
}


/*
 * Return / detach the first node in the list, or NULL.
 * @self: Pointer to the list for popping node
 */
List_NodeTypeDef *List_Lpop(List_TypeDef *self) {
    if (!self->len)
        return NULL;

    List_NodeTypeDef *node = self->head;

    if (--self->len) {
        (self->head = node->next)->prev = NULL;
    }
    else {
        self->head = self->tail = NULL;
    }

    node->next = node->prev = NULL;
    return node;
}


/*
 * Prepend the given node to the list
 * and return the node, NULL on failure.
 * @self: Pointer to the list for pushing node
 * @node: the node to push
 */
List_NodeTypeDef *List_Lpush(List_TypeDef *self, List_NodeTypeDef *node) {
    if (!node)
        return NULL;

    if (self->len) {
        node->next = self->head;
        node->prev = NULL;
        self->head->prev = node;
        self->head = node;
    }
    else {
        self->head = self->tail = node;
        node->prev = node->next = NULL;
    }

    ++self->len;
    return node;
}


/*
 * Return the node associated to val or NULL.
 * @self: Pointer to the list for finding given value
 * @val: Value to find
 */
List_NodeTypeDef *List_Find(List_TypeDef *self, void *val) {
    List_IteratorTypeDef *it = List_IteratorNew(self, LIST_HEAD);
    List_NodeTypeDef *node;

    while ((node = List_IteratorNext(it)) != NULL) {
        if (self->match) {
            if (self->match(val, node->val)) {
                List_IteratorDestroy(it);
                return node;
            }
        }
        else {
            if (val == node->val) {
                List_IteratorDestroy(it);
                return node;
            }
        }
    }

    List_IteratorDestroy(it);
    return NULL;
}


/*
 * Return the node at the given index or NULL.
 * @self: Pointer to the list for finding given index
 * @index: the index of node in the list
 */
List_NodeTypeDef *List_At(List_TypeDef *self, int index) {
    List_DirectionTypeDef direction = LIST_HEAD;

    if (index < 0) {
        direction = LIST_TAIL;
        index = ~index;
    }

    if ((unsigned)index < self->len) {
        List_IteratorTypeDef *it = List_IteratorNew(self, direction);
        List_NodeTypeDef *node = List_IteratorNext(it);
        while (index--)
            node = List_IteratorNext(it);
        List_IteratorDestroy(it);
        return node;
    }

    return NULL;
}


/*
 * Remove the given node from the list, freeing it and it's value.
 * @self: Pointer to the list to delete a node
 * @node: Pointer the node to be deleted
 */
void List_Remove(List_TypeDef *self, List_NodeTypeDef *node) {
    node->prev
        ? (node->prev->next = node->next)
        : (self->head = node->next);

    node->next
        ? (node->next->prev = node->prev)
        : (self->tail = node->prev);

    if (self->free)
        self->free(node->val);

    LIST_FREE(node);
    --self->len;
}


/*
 * Allocate a new List_IteratorTypeDef. NULL on failure.
 * Accepts a direction, which may be LIST_HEAD or LIST_TAIL.
 */
List_IteratorTypeDef *List_IteratorNew(List_TypeDef *list, List_DirectionTypeDef direction) {
    List_NodeTypeDef *node = direction == LIST_HEAD
                            ? list->head
                            : list->tail;
    return List_IteratorNewFromNode(node, direction);
}


/*
 * Allocate a new List_IteratorTypeDef with the given start
 * node. NULL on failure.
 */
List_IteratorTypeDef *List_IteratorNewFromNode(List_NodeTypeDef *node, List_DirectionTypeDef direction) {
    List_IteratorTypeDef *self;
    if ((self = LIST_MALLOC(sizeof(List_IteratorTypeDef))) == NULL)
        return NULL;
    self->next = node;
    self->direction = direction;
    return self;
}


/*
 * Return the next List_NodeTypeDef or NULL when no more
 * nodes remain in the list.
 */
List_NodeTypeDef *List_IteratorNext(List_IteratorTypeDef *self) {
    List_NodeTypeDef *curr = self->next;
    if (curr) {
        self->next = self->direction == LIST_HEAD
                         ? curr->next
                         : curr->prev;
    }
    return curr;
}


/*
 * Free the list iterator.
 */
void List_IteratorDestroy(List_IteratorTypeDef *self) {
    LIST_FREE(self);
    self = NULL;
}


/*
 * Allocates a new List_NodeTypeDef. NULL on failure.
 */
List_NodeTypeDef *List_NodeNew(void *val) {
    List_NodeTypeDef *self;
    if ((self = LIST_MALLOC(sizeof(List_NodeTypeDef))) == NULL)
        return NULL;
    self->prev = NULL;
    self->next = NULL;
    self->val = val;
    return self;
}
