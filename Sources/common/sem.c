/****************************************************************************
* ����: �����ź���
*       �������ⳡ�ϵ��߳�ͬ��
* ������<kerndev@foxmail.com>
* 2018.07.16 �����ļ�
****************************************************************************/
#include "kernel.h"
#include "sem.h"

bool sem_init(sem_t *sem, uint32_t value)
{
    sem->event = event_create();
    sem->mutex = mutex_create();
    sem->value = value;
    return true;
}

void sem_delete(sem_t *sem)
{
    event_delete(sem->event);
    mutex_delete(sem->mutex);
}

void sem_wait(sem_t *sem)
{
    mutex_lock(sem->mutex);
    if(sem->value != 0)
    {
        sem->value--;
        mutex_unlock(sem->mutex);
    }
    else
    {
        mutex_unlock(sem->mutex);
        event_wait(sem->event);
    }
}

bool sem_timed_wait(sem_t *sem, uint32_t timeout)
{
    mutex_lock(sem->mutex);
    if(sem->value != 0)
    {
        sem->value--;
        mutex_unlock(sem->mutex);
        return true;
    }
    else
    {
        mutex_unlock(sem->mutex);
        return event_timed_wait(sem->event, timeout);
    }
}

void sem_post(sem_t *sem)
{
    mutex_lock(sem->mutex);
    if(!event_signal(sem->event))
    {
        sem->value++;
    }
    mutex_unlock(sem->mutex);
}
