#ifndef __TASK_TEST_H
#define __TASK_TEST_H

#include "System_Config.h"

#ifdef  __TASK_TEST_GLOBALS
#define __TASK_TEST_EXT
#else
#define __TASK_TEST_EXT extern
#endif


void Task_Test1(void *parameter);
void Task_Test2(void *parameter);
void PrintSystemState(void);
void PrintTaskInfo(char str[]);
void PrintTaskState(char str[]);
void PrintTaskListing(void);

#endif

