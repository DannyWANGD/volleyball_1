/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : lib_tree.h
 *  Description  : File tree function library
 *  LastEditors  : Polaris
 *  Date         : 2023-01-29 04:49:52
 *  LastEditTime : 2023-08-14 23:28:26
 */


#ifndef LIB_TREE_H
#define LIB_TREE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"
#include "stdio.h"
#include "lib_list.h"


typedef enum {
    TREE_NODE_DIR,
    TREE_NODE_OBJ
} Tree_NodeTypeEnum;

typedef enum {
    TREE_DATA_NULL,     
    TREE_DATA_INT,      // all integer types
    TREE_DATA_UINT,     // all unsigned types
    TREE_DATA_FLOAT,    // all float point types
    TREE_DATA_STRING
} Tree_DataTypeEnum;

typedef struct _tree_node{
    Tree_NodeTypeEnum type;
    char *name;
    //uint8_t name_len;
    void *data;
    Tree_DataTypeEnum data_type;
    uint8_t data_len;
    struct _tree_node *parent;
    List_TypeDef *children;
} Tree_NodeTypeDef;

typedef uint8_t (*Tree_DataToStringFuncTypeDef)(char *, const void *, uint8_t);
typedef uint8_t (*Tree_DataFromStringFuncTypeDef)(const char *, void *, uint8_t);

typedef struct {
    Tree_DataTypeEnum data_type;
    //uint8_t data_len;
    Tree_DataToStringFuncTypeDef to_string;
    Tree_DataFromStringFuncTypeDef from_string;
} Tree_DataTypeHandlerTypeDef;

void Tree_Init(void);
Tree_NodeTypeDef* Tree_CreateRootNode(void);
Tree_NodeTypeDef* Tree_CreateDirNode(Tree_NodeTypeDef *parent, const char *name);
uint8_t Tree_DeleteNode(Tree_NodeTypeDef *node);
Tree_NodeTypeDef* Tree_OpenNode(Tree_NodeTypeDef *pwd, const char *path);

Tree_NodeTypeDef* _Tree_CreateObjNode(Tree_NodeTypeDef *parent, const char *name, 
                                      const void *data, Tree_DataTypeEnum data_type, uint8_t data_len);
#define Tree_CreateObjNode(parent, name, data) _Generic((data),\
    int8_t:         _Tree_CreateObjNode(parent, name, &data, TREE_DATA_INT,     1), \
    int16_t:        _Tree_CreateObjNode(parent, name, &data, TREE_DATA_INT,     2), \
    int32_t:        _Tree_CreateObjNode(parent, name, &data, TREE_DATA_INT,     4), \
    int64_t:        _Tree_CreateObjNode(parent, name, &data, TREE_DATA_INT,     8), \
    uint8_t:        _Tree_CreateObjNode(parent, name, &data, TREE_DATA_UINT,    1), \
    uint16_t:       _Tree_CreateObjNode(parent, name, &data, TREE_DATA_UINT,    2), \
    uint32_t:       _Tree_CreateObjNode(parent, name, &data, TREE_DATA_UINT,    4), \
    uint64_t:       _Tree_CreateObjNode(parent, name, &data, TREE_DATA_UINT,    8), \
    float:          _Tree_CreateObjNode(parent, name, &data, TREE_DATA_FLOAT,   2), \
    double:         _Tree_CreateObjNode(parent, name, &data, TREE_DATA_FLOAT,   4), \
    long double:    _Tree_CreateObjNode(parent, name, &data, TREE_DATA_FLOAT,   8), \
    char *:         _Tree_CreateObjNode(parent, name, &data, TREE_DATA_STRING,  0), \
    default:        _Tree_CreateObjNode(parent, name, &data, TREE_DATA_NULL,    0))

uint8_t Tree_IsDirNode(Tree_NodeTypeDef *node);
uint8_t Tree_GetNodePath(Tree_NodeTypeDef *node, char *path);
List_TypeDef *Tree_GetNodeChildren(Tree_NodeTypeDef *node);

#define Tree_GetNodeData(var, node) (var = *((typeof(var) *)_Tree_GetNodeDataPtr(node)))

uint8_t Tree_NodeDataToStr(char *buf, Tree_NodeTypeDef *node);
uint8_t Tree_NodeDataFromStr(const char *buf, Tree_NodeTypeDef *node);


extern Tree_NodeTypeDef* DataTreeRoot;


#endif

#ifdef __cplusplus
}
#endif
