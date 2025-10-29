/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : lib_tree.c
 *  Description  : File tree function library
 *  LastEditors  : Polaris
 *  Date         : 2023-01-29 04:49:41
 *  LastEditTime : 2023-08-15 01:43:18
 */


#include "lib_tree.h"
#include "string.h"

const char TREE_ROOT_NAME[] = "ROOT";
const char TREE_PATH_DELIM[] = "/";
const char TREE_PATH_SELF[] = ".";
const char TREE_PATH_PARENT[] = "..";

Tree_NodeTypeDef* DataTreeRoot;

void Tree_Init() {
    if (!DataTreeRoot) {
        DataTreeRoot = Tree_CreateRootNode();
    }
}


Tree_NodeTypeDef* newNode(const char *name) {
    Tree_NodeTypeDef *node;
    if ((node = malloc(sizeof(Tree_NodeTypeDef))) == NULL) {
        return NULL;            // malloc failed
    }
    uint8_t name_len = strlen(name) + 1;
    if ((node->name = malloc(name_len)) == NULL) {
        free(node);
        return NULL;            // malloc failed
    }
    memcpy(node->name, name, name_len);
    return node;
}


Tree_NodeTypeDef* findNode(Tree_NodeTypeDef *parent, const char *name) {
    if ((parent == NULL) || (name == NULL)) return NULL;
    if (parent->type != TREE_NODE_DIR) {
        return NULL;            // parent is not dir
    }
    if (parent->children == NULL) {
        return NULL;            // parent has no nodes
    }
    Tree_NodeTypeDef *node = NULL;
    for (List_IteratorTypeDef *iter = List_IteratorNew(parent->children, LIST_HEAD);
         iter->next; List_IteratorNext(iter)) 
    {
        Tree_NodeTypeDef *cur = (Tree_NodeTypeDef *)(iter->next->val);
        if (!strcmp(cur->name, name)) {
            node = cur;
        }
    }
    return node;
}


Tree_NodeTypeDef* createNode(Tree_NodeTypeDef *parent, const char *name) {
    if ((parent == NULL) || (name == NULL)) return NULL;
    if (strstr(name, TREE_PATH_DELIM)) {
        return NULL;            // name contains delim
    }
    if (parent->type != TREE_NODE_DIR) {
        return NULL;            // parent is not a dir
    }
    if (findNode(parent, name)) {
        return NULL;            // node already existed
    }
    Tree_NodeTypeDef *node;
    if ((node = newNode(name)) == NULL) {
        return NULL;            // Tree_NewNode failed
    }
    List_NodeTypeDef *lnode;
    if ((lnode = List_NodeNew(node)) == NULL) {
        return NULL;            // List_NodeNew failed
    }
    if ((List_Rpush(parent->children, lnode)) == NULL) {
        return NULL;            // List_Rpush failed
    }
    return node;
}


Tree_NodeTypeDef* Tree_CreateRootNode() {
    Tree_NodeTypeDef *node;
    if ((node = newNode(TREE_ROOT_NAME)) == NULL) {
        return NULL;            // newNode failed
    }
    node->type = TREE_NODE_DIR;
    node->parent = node;
    if ((node->children = List_New()) == NULL) {
        free(node);
        return NULL; // List_New failed
    }
    return node;
}


Tree_NodeTypeDef* Tree_CreateDirNode(Tree_NodeTypeDef *parent, const char *name) {
    Tree_NodeTypeDef *node;
    if ((node = createNode(parent, name)) == NULL) {
        return NULL;            // Tree_CreateNode failed
    }
    node->type = TREE_NODE_DIR;
    node->parent = parent;
    if ((node->children = List_New()) == NULL) {
        free(node);
        return NULL;            // List_New failed
    }
    return node;
}


Tree_NodeTypeDef* _Tree_CreateObjNode(Tree_NodeTypeDef *parent, const char *name, 
                                      const void *data, Tree_DataTypeEnum data_type, uint8_t data_len) 
{
    Tree_NodeTypeDef *node;
    if ((node = createNode(parent, name)) == NULL) {
        return NULL;            // createNode failed
    }
    node->type = TREE_NODE_OBJ;
    node->parent = parent;
    node->data = (void *)data;
    node->data_type = data_type;
    node->data_len = data_len;
    return node;
}


uint8_t Tree_DeleteNode(Tree_NodeTypeDef *node) {
    if (node == NULL) {
        return 1;               // null node
    }
    Tree_NodeTypeDef *parent;
    if ((parent = node->parent) == NULL) {
        return 2;               // null parent
    }
    if (parent == node) {
        return 3;               // root node
    }
    if (node->type == TREE_NODE_DIR) {
        List_IteratorTypeDef *iter = List_IteratorNew(node->children, LIST_HEAD);
        for (; iter->next; List_IteratorNext(iter))
        {
            Tree_NodeTypeDef *cur = (Tree_NodeTypeDef *)(iter->next->val);
            Tree_DeleteNode(cur);
        }
        List_IteratorDestroy(iter);
        List_Destroy(node->children);
    }
    free(node->name);
    List_IteratorTypeDef *iter = List_IteratorNew(parent->children, LIST_HEAD);
    for (; iter->next; List_IteratorNext(iter))
    {
        Tree_NodeTypeDef *cur = (Tree_NodeTypeDef *)(iter->next->val);
        if (cur == node) {
            List_Remove(parent->children, iter->next);
        }
    }
    List_IteratorDestroy(iter);
    free(node);
    return 0;
}


Tree_NodeTypeDef* Tree_OpenNode(Tree_NodeTypeDef *pwd, const char *path) {
    if ((pwd == NULL) || (path == NULL)) return NULL;
    uint8_t is_absolute = (path[0] == TREE_PATH_DELIM[0]);
    uint8_t path_len = strlen(path) + 1;
    char *_path = malloc(path_len);
    memcpy(_path, path, path_len);
    Tree_NodeTypeDef *now = is_absolute ? DataTreeRoot : pwd;
    char *tok = strtok(_path + (is_absolute ? 1 : 0), TREE_PATH_DELIM);
    while (tok) {
        if (!strcmp(tok, TREE_PATH_SELF)) {
            now = now;
        }
        else if (!strcmp(tok, TREE_PATH_PARENT)) {
            now = now->parent;
        }
        else {
            now = findNode(now, tok);
        }
        if (now == NULL) {
            free(_path);
            return NULL;        // failed to follow path
        }
        tok = strtok(NULL, TREE_PATH_DELIM); 
    }
    free(_path);
    return now;
}


uint8_t Tree_IsDirNode(Tree_NodeTypeDef *node) {
    if (node == NULL) {
        return 0;               // null node
    }
    return node->type == TREE_NODE_OBJ;
}


uint8_t Tree_GetNodePath(Tree_NodeTypeDef *node, char *path) {
    if (node == NULL) {
        return 0;               // null node
    }
    char *buf = path;
    if (node == DataTreeRoot || node == node->parent) {
        buf += sprintf(buf, "%s", TREE_PATH_DELIM);
    }
    else {
        uint8_t ret = Tree_GetNodePath(node->parent, path);
        if (ret == 0) {
            return 0;           // fail to follow parent
        }
        buf += ret;
        buf += sprintf(buf, "%s%s", node->name, TREE_PATH_DELIM);
    }
    *buf = 0;
    return buf - path;
}


List_TypeDef *Tree_GetNodeChildren(Tree_NodeTypeDef *node) {
    if (node == NULL) {
        return NULL;            // null node
    }
    if (node->type != TREE_NODE_DIR) {
        return NULL;            // not a dir node
    }
    return node->children;
}


void *_Tree_GetNodeDataPtr(Tree_NodeTypeDef *node) {
    if (node == NULL) {
        return NULL;            // null node
    }
    if (node->type != TREE_NODE_OBJ) {
        return NULL;            // not a object node
    }
    return node->data;
}


/********** Type Generic Code **********/

uint8_t T_IntToString(char *buf, const void *data, uint8_t data_len) {
    int64_t _data;
    if (data_len == 1) {
        _data = *((int8_t *)data);
    }
    else if (data_len == 2) {
        _data = *((int16_t *)data);
    }
    else if (data_len == 4) {
        _data = *((int32_t *)data);
    }
    else if (data_len == 8) {
        _data = *((int64_t *)data);
    }
    else {
        return 1;               // bad data_len
    }
    sprintf(buf, "%lld", _data);
    return 0;
}


uint8_t T_IntFromString(const char *buf, void *data, uint8_t data_len) {
    int64_t _data;
    if (!(sscanf(buf, "%lld", &_data))) {
        return 1;               // failed to convert
    }
    if (data_len == 1) {
        *((int8_t *)data) = _data;
    }
    else if (data_len == 2) {
        *((int16_t *)data) = _data;
    }
    else if (data_len == 4) {
        *((int32_t *)data) = _data;
    }
    else if (data_len == 8) {
        *((int64_t *)data) = _data;
    }
    else {
        return 2;               // bad data_len
    }
    return 0;
}


uint8_t T_UIntToString(char *buf, const void *data, uint8_t data_len) {
    uint64_t _data;
    if (data_len == 1) {
        _data = *((uint8_t *)data);
    }
    else if (data_len == 2) {
        _data = *((uint16_t *)data);
    }
    else if (data_len == 4) {
        _data = *((uint32_t *)data);
    }
    else if (data_len == 8) {
        _data = *((uint64_t *)data);
    }
    else {
        return 1;               // bad data_len
    }
    sprintf(buf, "%llu", _data);
    return 0;
}


uint8_t T_UIntFromString(const char *buf, void *data, uint8_t data_len) {
    uint64_t _data;
    if (!(sscanf(buf, "%llu", &_data))) {
        return 1;               // failed to convert
    }
    if (data_len == 1) {
        *((uint8_t *)data) = _data;
    }
    else if (data_len == 2) {
        *((uint16_t *)data) = _data;
    }
    else if (data_len == 4) {
        *((uint32_t *)data) = _data;
    }
    else if (data_len == 8) {
        *((uint64_t *)data) = _data;
    }
    else {
        return 2;               // bad data_len
    }
    return 0;
}


uint8_t T_FloatToString(char *buf, const void *data, uint8_t data_len) {
    long double _data;
    if (data_len == 2) {
        _data = *((float *)data);
    }
    else if (data_len == 4) {
        _data = *((double *)data);
    }
    else if (data_len == 8) {
        _data = *((long double *)data);
    }
    else {
        return 1;               // bad data_len
    }
    sprintf(buf, "%Lf", _data);
    return 0;
}


uint8_t T_FloatFromString(const char *buf, void *data, uint8_t data_len) {
    long double _data;
    if (!(sscanf(buf, "%Lf", &_data))) {
        return 1;               // failed to convert
    }
    if (data_len == 2) {
        *((float *)data) = _data;
    }
    else if (data_len == 4) {
        *((double *)data) = _data;
    }
    else if (data_len == 8) {
        *((long double *)data) = _data;
    }
    else {
        return 2;               // bad data_len
    }
    return 0;
}


uint8_t T_StringToString(char *buf, const void *data, uint8_t data_len) {
    sprintf(buf, "%s", *((char **)data));
    return 0;
}


#define Tree_DataTypeHandlerNum  4
const Tree_DataTypeHandlerTypeDef Tree_DataTypeHandlers[Tree_DataTypeHandlerNum] = {
    {TREE_DATA_INT,     &T_IntToString,     &T_IntFromString},
    {TREE_DATA_UINT,    &T_UIntToString,    &T_UIntFromString},
    {TREE_DATA_FLOAT,   &T_FloatToString,   &T_FloatFromString},
    {TREE_DATA_STRING,  &T_StringToString,  NULL}                   // string node as read-only
};


uint8_t Tree_NodeDataToStr(char *buf, Tree_NodeTypeDef *node) {
    if (node == NULL) {
        return 1;               // null node
    }
    if (node->type != TREE_NODE_OBJ) {
        return 2;               // not a object node
    }
    if (node->data_type == TREE_DATA_NULL) {
        return 3;               // null node data type
    }
    for (int i = 0; i < Tree_DataTypeHandlerNum; ++i) {
        if (Tree_DataTypeHandlers[i].data_type == node->data_type) {
            Tree_DataToStringFuncTypeDef to_string;
            if ((to_string = Tree_DataTypeHandlers[i].to_string) == NULL) {
                return 4;       // action not supported
            }
            if ((*to_string)(buf, node->data, node->data_len)) {
                return 5;       // action failed
            } 
            else {
                return 0;
            }
        }
    }
    return 6;                   // bad node data type, no handler found
}


uint8_t Tree_NodeDataFromStr(const char *buf, Tree_NodeTypeDef *node) {
    if (node == NULL) {
        return 1;               // null node
    }
    if (node->type != TREE_NODE_OBJ) {
        return 2;               // not a object node
    }
    if (node->data_type == TREE_DATA_NULL) {
        return 3;               // null node data type
    }
    for (int i = 0; i < Tree_DataTypeHandlerNum; ++i) {
        if (Tree_DataTypeHandlers[i].data_type == node->data_type) {
            Tree_DataFromStringFuncTypeDef from_string;
            if ((from_string = Tree_DataTypeHandlers[i].from_string) == NULL) {
                return 4;       // action not supported
            }
            if ((*from_string)(buf, node->data, node->data_len)) {
                return 5;       // action failed
            }
            else {
                return 0;
            }
        }
    }
    return 6;                   // bad node data type, no handler found
}
