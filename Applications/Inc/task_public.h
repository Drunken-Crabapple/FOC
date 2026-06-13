#ifndef TASK_PUBLIC_H
#define TASK_PUBLIC_H

#ifdef __cplusplus
extern "C" {
#endif

void StartDebugTask(void *argument);
void StartFOCTask(void *argument);
void StartCommunicateTask(void *argument);
void StartStartShell(void *argument);

#ifdef __cplusplus
}
#endif

#endif
