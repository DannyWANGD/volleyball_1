/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_queue.c
 *  Description  : This file contains the functions of queue
 *  LastEditors  : Polaris
 *  Date         : 2022-04-29 13:12:08
 *  LastEditTime : 2023-02-08 02:43:00
 */


#include "lib_queue.h"


/**
 * @brief        : Queue for char create
 * @param         [int] uint_cnt
 * @return        [type]
 */
Queue_CharTypeDef *Queue_CharCreate(int uint_cnt) {
    Queue_CharTypeDef *p_queue = NULL;  
    char *p_base_addr = NULL; 

    p_queue = (Queue_CharTypeDef *)malloc(sizeof(Queue_CharTypeDef));
    if (NULL == p_queue) {
      return (NULL);
    }

    p_base_addr = malloc(uint_cnt);
    if (NULL == p_base_addr) {
      free(p_queue);
      return (NULL);
    }

    Queue_CharInit(p_queue, p_base_addr, uint_cnt);
    return (p_queue);
}


/**
 * @brief        : Queue for char destroy
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
void Queue_CharDestroy(Queue_CharTypeDef *p_queue) {
    free(p_queue->p_start_addr);
    free(p_queue);

    return; 
}


/**
 * @brief        : Queue for char init
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [void] *p_base_addr
 * @param         [int] uint_cnt
 * @return        [type]
 */
int Queue_CharInit(Queue_CharTypeDef *p_queue, void *p_base_addr, int uint_cnt) {
    p_queue->p_start_addr = (char *)p_base_addr;
    p_queue->p_end_addr = (char *)p_base_addr + uint_cnt - 1;
    p_queue->free_num = uint_cnt;
    p_queue->used_num = 0;
    p_queue->p_read_addr = (char *)p_base_addr;
    p_queue->p_write_addr = (char *)p_base_addr;

    return 0;
}


/**
 * @brief        : Queue for char Put element
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [char] element
 * @return        [type]
 */
int Queue_CharPut(Queue_CharTypeDef *p_queue, char element) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (0 == p_queue->free_num) {
        goto end;
    }
    if (p_queue->p_write_addr > p_queue->p_end_addr)
        p_queue->p_write_addr = p_queue->p_start_addr;

    *(p_queue->p_write_addr) = element;
    p_queue->p_write_addr++;
    p_queue->free_num--;
    p_queue->used_num++;

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return 0;

end:
    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (-1);
}


/**
 * @brief        : Queue for char Put elements
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [char] *p_source
 * @param         [int] len
 * @return        [type]
 */
int Queue_CharPuts(Queue_CharTypeDef *p_queue, char *p_source, int len) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    int retval;
    int len_to_end;
    int len_from_start;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (NULL == p_source) {
        goto end;
    }

    if (0 == p_queue->free_num) {
        goto end;
    }

    if (p_queue->p_write_addr > p_queue->p_end_addr)
        p_queue->p_write_addr = p_queue->p_start_addr;

    len = (len < p_queue->free_num) ? len : p_queue->free_num;
    len_to_end = p_queue->p_end_addr - p_queue->p_write_addr + 1;

    if (len_to_end >= len) {
        len_to_end = len;
        memcpy(p_queue->p_write_addr, p_source, len_to_end);
        p_queue->p_write_addr += len_to_end;
    }
    else  {
        len_from_start = len - len_to_end;
        memcpy(p_queue->p_write_addr, p_source, len_to_end);
        memcpy(p_queue->p_start_addr, p_source + len_to_end, len_from_start);
        p_queue->p_write_addr = p_queue->p_start_addr + len_from_start;
    }

    p_queue->free_num -= len;
    p_queue->used_num += len;
    retval = len;
    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return retval;
end:

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (-1);
}


/**
 * @brief        : Queue for char noprotect
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [char] *p_source
 * @param         [int] len
 * @return        [type]
 */
int Queue_CharPutsNoProtect(Queue_CharTypeDef *p_queue, char *p_source, int len) {
    int retval;
    int len_to_end;
    int len_from_start;

    if (NULL == p_source)
        return -1;

    if (0 == p_queue->free_num)
        return 0;

    if (p_queue->p_write_addr > p_queue->p_end_addr)
        p_queue->p_write_addr = p_queue->p_start_addr;

    len = (len < p_queue->free_num) ? len : p_queue->free_num;
    len_to_end = p_queue->p_end_addr - p_queue->p_write_addr + 1;

    if (len_to_end >= len) {
        len_to_end = len;
        memcpy(p_queue->p_write_addr, p_source, len_to_end);
        p_queue->p_write_addr += len_to_end;
    }
    else {
        len_from_start = len - len_to_end;
        memcpy(p_queue->p_write_addr, p_source, len_to_end);
        memcpy(p_queue->p_start_addr, p_source + len_to_end, len_from_start);
        p_queue->p_write_addr = p_queue->p_start_addr + len_from_start;
    }

    p_queue->free_num -= len;
    p_queue->used_num += len;
    retval = len;

    return retval;
}


/**
 * @brief        : Queue for char Get element
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
char Queue_CharGet(Queue_CharTypeDef *p_queue) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    char retval = 0;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (p_queue->p_read_addr > p_queue->p_end_addr)
        p_queue->p_read_addr = p_queue->p_start_addr;

    retval = *p_queue->p_read_addr;

    p_queue->p_read_addr++;
    p_queue->free_num++;
    p_queue->used_num--;

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (retval);
}


/**
 * @brief        : Queue for char get elements
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [char] *p_dest
 * @param         [int] len
 * @return        [type]
 */
int Queue_CharGets(Queue_CharTypeDef *p_queue, char *p_dest, int len) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    int retval;
    int len_to_end;
    int len_from_start;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (NULL == p_dest) {
        goto end;
    }

    if (0 == p_queue->used_num) {
        goto end;
    }

    if (p_queue->p_read_addr > p_queue->p_end_addr)
        p_queue->p_read_addr = p_queue->p_start_addr;

    len = (len < p_queue->used_num) ? len : p_queue->used_num;
    len_to_end = p_queue->p_end_addr - p_queue->p_read_addr + 1;

    if (len_to_end >= len) {
        len_to_end = len;
        memcpy(p_dest, p_queue->p_read_addr, len_to_end);
        p_queue->p_read_addr += len_to_end;
    }
    else  {
        len_from_start = len - len_to_end;
        memcpy(p_dest, p_queue->p_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_queue->p_start_addr, len_from_start);
        p_queue->p_read_addr = p_queue->p_start_addr + len_from_start;
    }

    p_queue->free_num += len;
    p_queue->used_num -= len;
    retval = len;

    QUEUE_RESTORE_CPU_SR(cpu_sr);

    return retval;
end:

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (-1);
}


/**
 * @brief        : Queue for char Get elements no protected
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [char] *p_dest
 * @param         [int] len
 * @return        [type]
 */
int Queue_CharGetsNoProtect(Queue_CharTypeDef *p_queue, char *p_dest, int len) {
    int retval;
    int len_to_end;
    int len_from_start;

    if (NULL == p_dest)
      return -1;

    if (0 == p_queue->used_num)
      return 0;

    if (p_queue->p_read_addr > p_queue->p_end_addr)
        p_queue->p_read_addr = p_queue->p_start_addr;

    len = (len < p_queue->used_num) ? len : p_queue->used_num;
    len_to_end = p_queue->p_end_addr - p_queue->p_read_addr + 1;

    if (len_to_end >= len) {
        len_to_end = len;
        memcpy(p_dest, p_queue->p_read_addr, len_to_end);
        p_queue->p_read_addr += len_to_end;
    }
    else {
        len_from_start = len - len_to_end;
        memcpy(p_dest, p_queue->p_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_queue->p_start_addr, len_from_start);
        p_queue->p_read_addr = p_queue->p_start_addr + len_from_start;
    }

    p_queue->free_num += len;
    p_queue->used_num -= len;
    retval = len;

    return retval;
}


/**
 * @brief        : Queue for char preread
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [int] offset
 * @return        [type]
 */
char Queue_CharPreread(Queue_CharTypeDef *p_queue, int offset) {
    char *tmp_read_addr;

    if (offset > p_queue->used_num) {
        return 0;
    }
    else {
        tmp_read_addr = p_queue->p_read_addr + offset;
        if (tmp_read_addr > p_queue->p_end_addr)
            tmp_read_addr = tmp_read_addr - p_queue->p_end_addr + p_queue->p_start_addr - 1;  
        return *tmp_read_addr;
    }
}


/**
 * @brief        : Queue for char prereads
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [char] *p_dest
 * @param         [int] offset
 * @param         [int] len
 * @return        [type]
 */
int Queue_CharPrereads(Queue_CharTypeDef *p_queue, char *p_dest, int offset, int len) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    int retval;
    char *tmp_read_addr;
    int len_to_end;
    int len_from_start;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (NULL == p_dest)
        goto end;

    if (0 == p_queue->used_num)
        goto end;

    if (offset >= p_queue->used_num)
        goto end;

    tmp_read_addr = p_queue->p_read_addr + offset;
    if (tmp_read_addr > p_queue->p_end_addr)
        tmp_read_addr = tmp_read_addr - p_queue->p_end_addr + p_queue->p_start_addr - 1;

    len = (len < (p_queue->used_num - offset)) ? len : (p_queue->used_num - offset);
    len_to_end = p_queue->p_end_addr - tmp_read_addr + 1;

    if (len_to_end >= len) {
        len_to_end = len;
        memcpy(p_dest, tmp_read_addr, len_to_end);
    }
    else {
        len_from_start = len - len_to_end;
        memcpy(p_dest, tmp_read_addr, len_to_end);
        memcpy(p_dest + len_to_end, p_queue->p_start_addr, len_from_start);
    }

    retval = len;    
    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return retval;
end:

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (-1);
}


/**
 * @brief        : Queue for char Is Empty
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
char Queue_CharIsEmpty(Queue_CharTypeDef *p_queue) {
    return (p_queue->used_num ? 0 : 1);
}


/**
 * @brief        : Queue for char Is Full
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
char Queue_CharIsFull(Queue_CharTypeDef *p_queue) {
    return (p_queue->free_num ? 0 : 1);
}


/**
 * @brief        : Queue for charUsed
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
int Queue_CharUsed(Queue_CharTypeDef *p_queue) {
    return p_queue->used_num;
}


/**
 * @brief        : Queue for char Free
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
int Queue_CharFree(Queue_CharTypeDef *p_queue) {
    return p_queue->free_num;
}


/**
 * @brief        : Queue for char Flush
 * @param         [Queue_CharTypeDef] *p_queue
 * @return        [type]
 */
void Queue_CharFlush(Queue_CharTypeDef *p_queue) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    p_queue->free_num = p_queue->p_end_addr - p_queue->p_start_addr + 1;
    p_queue->used_num = 0;
    p_queue->p_read_addr = p_queue->p_start_addr;
    p_queue->p_write_addr = p_queue->p_start_addr;

    QUEUE_RESTORE_CPU_SR(cpu_sr);
}


/**
 * @brief        : Queue for char Discard
 * @param         [Queue_CharTypeDef] *p_queue
 * @param         [int] len
 * @return        [type]
 */
int Queue_CharDiscard(Queue_CharTypeDef *p_queue, int len) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    char *tmp_index;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (len > p_queue->used_num)
        len = p_queue->used_num;

    tmp_index = len + p_queue->p_read_addr;
    if (tmp_index > p_queue->p_end_addr)
        tmp_index = tmp_index - p_queue->p_end_addr + p_queue->p_start_addr - 1;
    p_queue->p_read_addr = tmp_index;
    p_queue->free_num += len;
    p_queue->used_num -= len;

    QUEUE_RESTORE_CPU_SR(cpu_sr);

    return len;
}


/**
 * @brief        : Queue Create
 * @param         [char] unit_size
 * @param         [int] unit_cnt
 * @return        [type]
 */
Queue_TypeDef *Queue_Create(char unit_size, int unit_cnt) {
    Queue_TypeDef *p_queue = NULL;    
    char *p_base_addr = NULL; 
    p_queue = (Queue_TypeDef *)malloc(sizeof(Queue_TypeDef));
    if (NULL == p_queue) {
        return (NULL);
    }
    
    p_base_addr = malloc(unit_size * unit_cnt);
    if (NULL == p_base_addr) {
        free(p_queue);
        return (NULL);
    }
    Queue_Init(p_queue, p_base_addr, unit_size, unit_cnt);
    
    return (p_queue);
}


/**
 * @brief        : Queue Destory
 * @param         [Queue_TypeDef] *p_queue
 * @return        [type]
 */
void Queue_Destory(Queue_TypeDef *p_queue) {
    free(p_queue->p_start_addr);
    free(p_queue);
    return; 
}


/**
 * @brief        : Queue Init
 * @param         [Queue_TypeDef] *p_queue
 * @param         [void] *p_base_addr
 * @param         [char] unit_size
 * @param         [int] unit_cnt
 * @return        [type]
 */
int Queue_Init(Queue_TypeDef *p_queue, void *p_base_addr, char unit_size, int unit_cnt) {
    p_queue->p_start_addr = (char *)p_base_addr;
    p_queue->p_end_addr = (char *)p_base_addr + unit_size * unit_cnt - 1;
    p_queue->free_num = unit_cnt;
    p_queue->used_num = 0;
    p_queue->unit_size = unit_size;
    p_queue->p_read_addr = (char *)p_base_addr;
    p_queue->p_write_addr = (char *)p_base_addr;

    return (0);
}


/**
 * @brief        : Queue Put
 * @param         [Queue_TypeDef] *p_queue
 * @param         [void] *p_element
 * @return        [type]
 */
int Queue_Put(Queue_TypeDef *p_queue, void *p_element) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();
    if (0 == p_queue->free_num) {
        goto end;
    }
    if (p_queue->p_write_addr > p_queue->p_end_addr)
        p_queue->p_write_addr = p_queue->p_start_addr;

    memcpy(p_queue->p_write_addr, p_element, p_queue->unit_size);
    p_queue->p_write_addr += p_queue->unit_size;
    p_queue->free_num--;
    p_queue->used_num++;

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (0);
end:

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (-1);
}


/**
 * @brief        : Queue Put No Protect
 * @param         [Queue_TypeDef] *p_queue
 * @param         [void] *p_element
 * @return        [type]
 */
int Queue_PutNoProtect(Queue_TypeDef *p_queue, void *p_element) {
    if (0 == p_queue->free_num) {
        return (-1);
    }

    if (p_queue->p_write_addr > p_queue->p_end_addr)
        p_queue->p_write_addr = p_queue->p_start_addr;

    memcpy(p_queue->p_write_addr, p_element, p_queue->unit_size);
    p_queue->p_write_addr += p_queue->unit_size;
    p_queue->free_num--;
    p_queue->used_num++;

    return (0);
}


/**
 * @brief        : Queue Get
 * @param         [Queue_TypeDef] *p_queue
 * @param         [void] *p_element
 * @return        [type]
 */
int Queue_Get(Queue_TypeDef *p_queue, void *p_element) {
    QUEUE_CPU_SR_TYPE cpu_sr;  
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();

    if (0 == p_queue->used_num) {
        goto end;
    }
    if (p_queue->p_read_addr > p_queue->p_end_addr) {
        p_queue->p_read_addr = p_queue->p_start_addr;
    }
    memcpy(p_element, p_queue->p_read_addr, p_queue->unit_size);
    p_queue->p_read_addr += p_queue->unit_size;
    p_queue->free_num++;
    p_queue->used_num--;

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (0);
end:

    QUEUE_RESTORE_CPU_SR(cpu_sr);
    return (-1);
}


/**
 * @brief        : Queue Get No Protect
 * @param         [Queue_TypeDef] *p_queue
 * @param         [void] *p_element
 * @return        [type]
 */
int Queue_GetNoProtect(Queue_TypeDef *p_queue, void *p_element) {
    if (0 == p_queue->used_num) {
        return (-1);
    }
    if (p_queue->p_read_addr > p_queue->p_end_addr) {
        p_queue->p_read_addr = p_queue->p_start_addr;
    }
    memcpy(p_element, p_queue->p_read_addr, p_queue->unit_size);
    p_queue->p_read_addr += p_queue->unit_size;
    p_queue->free_num++;
    p_queue->used_num--;

    return (0);
}


/**
 * @brief        : Queue Preread
 * @param         [Queue_TypeDef] *p_queue
 * @param         [char] offset
 * @param         [void] *p_element
 * @return        [type]
 */
int Queue_Preread(Queue_TypeDef *p_queue, char offset, void *p_element) {
    char *_pre_red_index = (void *)0;
    if (offset >= p_queue->used_num) {
        return (-1);
    }
    _pre_red_index = p_queue->p_read_addr + p_queue->unit_size * offset;
    while (_pre_red_index > p_queue->p_end_addr) {
        _pre_red_index = _pre_red_index - p_queue->p_end_addr + p_queue->p_start_addr - 1;
    }

    memcpy(p_element, _pre_red_index, p_queue->unit_size);

    return (0);
}


/**
 * @brief        : Queue Is Empty
 * @param         [Queue_TypeDef] *p_queue
 * @return        [type]
 */
int Queue_IsEmpty(Queue_TypeDef *p_queue) {
  return (0 == p_queue->used_num);
}


/**
 * @brief        : Queue Is Full
 * @param         [Queue_TypeDef] *p_queue
 * @return        [type]
 */
int Queue_IsFull(Queue_TypeDef *p_queue) {
  return (0 == p_queue->free_num);
}


/**
 * @brief        : Queue Used
 * @param         [Queue_TypeDef] *p_queue
 * @return        [type]
 */
int Queue_Used(Queue_TypeDef *p_queue) {
  return (p_queue->used_num);
}


/**
 * @brief        : Queue Free
 * @param         [Queue_TypeDef] *p_queue
 * @return        [type]
 */
int Queue_Free(Queue_TypeDef *p_queue) {
  return (p_queue->free_num);
}


/**
 * @brief        : Queue Flush
 * @param         [Queue_TypeDef] *p_queue
 * @return        [type]
 */
int Queue_Flush(Queue_TypeDef *p_queue) {
    QUEUE_CPU_SR_TYPE cpu_sr;
    cpu_sr = QUEUE_GET_CPU_SR();
    QUEUE_ENTER_CRITICAL();
    
    p_queue->free_num = (p_queue->p_end_addr - p_queue->p_start_addr) / (p_queue->unit_size);
    p_queue->used_num = 0;
    p_queue->p_read_addr = p_queue->p_start_addr;
    p_queue->p_write_addr = p_queue->p_start_addr;
    
    QUEUE_RESTORE_CPU_SR(cpu_sr);
    
    return (0);
}
