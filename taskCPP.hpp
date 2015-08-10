/*
 * taskCPP.h
 *
 *  Created on: Aug 8, 2015
 *      Author: chris
 */

#ifndef TASKCPP_HPP_
#define TASKCPP_HPP_

#include "FreeRTOS.h"
#include "task.h"

class TaskBase {
public:
	xTaskHandle handle;

	~TaskBase() {
#if INCLUDE_vTaskDelete
		vTaskDelete(handle);
#endif
		return;
	}
};

class Task: public TaskBase {
public:
	Task(char const*name, void (*taskfun)(void *), unsigned portBASE_TYPE priority,
			unsigned portSHORT stackDepth=configMINIMAL_STACK_SIZE) {
		xTaskCreate(taskfun, (signed char*)name, stackDepth, this, priority, &handle);
	}

};

class TaskClass: public TaskBase {
public:
	TaskClass(char const*name, unsigned portBASE_TYPE priority,
			unsigned portSHORT stackDepth=configMINIMAL_STACK_SIZE) {
		xTaskCreate(&taskfun, (signed char*)name, stackDepth, this, priority, &handle);
	}
	virtual void task() = 0;
	static void taskfun(void* parm) {
		static_cast(parm)->task();
#if INCLUDE_vTaskDelete
		xTaskDelete(handle);
#else
		while (1)
			vTaskDelay(10000);
#endif
	}
};

#endif /* TASKCPP_HPP_ */
