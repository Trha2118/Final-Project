/*
 * fifo.h
 *
 *  Created on: April 25th, 2021
 *      Author: Travis H
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>



// holds node information

struct node_t {
    uint8_t btn;
    struct node_t* next;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///     Creates queue.
///
/// @param[in]
///     task information
/// @param[in]
///     size of the task array
///
/// @return
///     head of queue
//----------------------------------------------------------------------------------------------------------------------------------


struct node_t* create_queue(void);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///     new node for queue
///
/// @param
///     task information
///
/// @return
///      task
//----------------------------------------------------------------------------------------------------------------------------------

struct node_t* create_new_node(uint8_t btn);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///    returns top node
///
/// @param
////   head of the queue
///
/// @return
///    btn value at top of queue
//----------------------------------------------------------------------------------------------------------------------------------

uint8_t peek(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///    Removes element at top
///
/// @param
///    head of queue.
//----------------------------------------------------------------------------------------------------------------------------------

void pop(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///    Push a new btn action into the queue
///
/// @param
///   head of the queue
/// @param
///    action to put into queue
//----------------------------------------------------------------------------------------------------------------------------------

void push(struct node_t** head, uint8_t btn);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///     is head empty?
///
/// @param
///     head of Queue
///
/// @return
///     True if head empty, False otherwise.
//----------------------------------------------------------------------------------------------------------------------------------

int is_empty(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief
///     remove items from queue
///
/// @param
///     head of the queue
//----------------------------------------------------------------------------------------------------------------------------------

void empty_queue(struct node_t** head);


// Array FIFO function declarations
void fifo_push(uint8_t *fifo, uint8_t *wr_ptr, uint8_t len, uint8_t val);
// Array FIFO function declarations
uint8_t fifo_pop(uint8_t *fifo, uint8_t *rd_ptr, uint8_t len);
bool fifo_isempty(uint8_t *fifo, uint8_t *wr_ptr, uint8_t *rd_ptr);


#endif /* FIFO_H_*/
