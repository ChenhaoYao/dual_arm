/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */
#include <osal.h>
#include <stdlib.h>
#include <string.h>

/** 获取单调时间（从某个未指定的过去时刻开始，严格递增）
 * @param ts 指向时间结构体的指针，用于存储获取的时间
 * 
 * 功能：
 * - 使用 CLOCK_MONOTONIC 时钟获取单调递增的时间
 * - 适用于时间间隔测量，不受系统时间调整影响
 * - 使用 clock_gettime 而非 gettimeofday 以避免可能的活锁
 * - 在 XENOMAI 环境下，只有 clock_gettime 是实时安全的
 */
void osal_get_monotonic_time(ec_timet *ts)
{
   /* 使用 clock_gettime 获取单调时间，避免 gettimeofday 使用的系统实时时钟
    * 受到 NTP 校时或手动修改系统时间的影响。
    * 在某些实时场景下，gettimeofday 可能因为时间调整和 vpage 机制导致活锁；
    * XENOMAI 环境中 clock_gettime 也是实时安全的选择。 */
   clock_gettime(CLOCK_MONOTONIC, ts);
}

/** 获取当前系统时间（挂钟时间）
 * @return 返回当前系统时间（CLOCK_REALTIME）
 * 
 * 功能：
 * - 使用 CLOCK_REALTIME 获取系统挂钟时间
 * - 会受到 NTP 时间调整和系统时间设置的影响
 * - 适用于需要绝对时间的场景
 */
ec_timet osal_current_time(void)
{
   struct timespec ts;

   clock_gettime(CLOCK_REALTIME, &ts);
   return ts;
}

/** 计算时间差
 * @param start 起始时间
 * @param end 结束时间
 * @param diff 输出参数，存储计算出的时间差（end - start）
 * 
 * 功能：
 * - 计算两个时间点之间的差值
 * - 结果存储在 diff 中
 */
void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
   osal_timespecsub(end, start, diff);
}

/** 启动定时器
 * @param self 指向定时器结构体的指针
 * @param timeout_usec 超时时间（微秒）
 * 
 * 功能：
 * - 获取当前单调时间作为起始时间
 * - 将微秒转换为时间结构体
 * - 计算定时器的停止时间（起始时间 + 超时时间）
 */
void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
   struct timespec start_time;
   struct timespec timeout;

   osal_get_monotonic_time(&start_time);
   osal_timespec_from_usec(timeout_usec, &timeout);
   osal_timespecadd(&start_time, &timeout, &self->stop_time);
}

/** 检查定时器是否已过期
 * @param self 指向定时器结构体的指针
 * @return TRUE 表示定时器已过期，FALSE 表示未过期
 * 
 * 功能：
 * - 获取当前单调时间
 * - 比较当前时间与定时器的停止时间
 * - 如果当前时间 >= 停止时间，则定时器已过期
 */
boolean osal_timer_is_expired(osal_timert *self)
{
   struct timespec current_time;
   int is_not_yet_expired;

   osal_get_monotonic_time(&current_time);
   is_not_yet_expired = osal_timespeccmp(&current_time, &self->stop_time, <);

   return is_not_yet_expired == FALSE;
}

/** 微秒级休眠
 * @param usec 休眠时间（微秒）
 * @return 成功返回 0，失败返回 -1
 * 
 * 功能：
 * - 将微秒转换为时间结构体
 * - 使用 clock_nanosleep 进行高精度休眠
 * - 使用 CLOCK_MONOTONIC 时钟，不受系统时间调整影响
 */
int osal_usleep(uint32 usec)
{
   struct timespec ts;
   int result;

   osal_timespec_from_usec(usec, &ts);
   result = clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
   return result == 0 ? 0 : -1;
}

/** 绝对时间休眠（休眠到指定的绝对时间点）
 * @param ts 指向目标绝对时间点的指针
 * @return 成功返回 0，失败返回 -1
 * 
 * 功能：
 * - 使用 TIMER_ABSTIME 标志，表示 ts 是绝对时间而非相对时间
 * - 适用于周期性任务，可以补偿休眠时间
 * - 使用 CLOCK_MONOTONIC 时钟
 */
int osal_monotonic_sleep(ec_timet *ts)
{
   int result;
   result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL);
   return result == 0 ? 0 : -1;
}

/** 内存分配
 * @param size 要分配的内存大小（字节）
 * @return 返回分配的内存指针，失败返回 NULL
 * 
 * 功能：
 * - 封装标准库的 malloc 函数
 * - 提供统一的内存分配接口
 */
void *osal_malloc(size_t size)
{
   return malloc(size);
}

/** 内存释放
 * @param ptr 要释放的内存指针
 * 
 * 功能：
 * - 封装标准库的 free 函数
 * - 释放之前通过 osal_malloc 分配的内存
 */
void osal_free(void *ptr)
{
   free(ptr);
}

/** 创建普通线程
 * @param thandle 线程句柄指针（pthread_t 类型）
 * @param stacksize 线程栈大小（字节）
 * @param func 线程函数指针
 * @param param 传递给线程函数的参数
 * @return 成功返回 1，失败返回 0
 * 
 * 功能：
 * - 初始化线程属性
 * - 设置线程栈大小
 * - 创建线程并启动执行
 */
int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   int ret;
   pthread_attr_t attr;
   pthread_t *threadp;

   threadp = thandle;
   pthread_attr_init(&attr);
   pthread_attr_setstacksize(&attr, stacksize);
   ret = pthread_create(threadp, &attr, func, param);
   if (ret < 0)
   {
      return 0;
   }
   return 1;
}

/** 创建实时线程
 * @param thandle 线程句柄指针（pthread_t 类型）
 * @param stacksize 线程栈大小（字节）
 * @param func 线程函数指针
 * @param param 传递给线程函数的参数
 * @return 成功返回 1，失败返回 0
 * 
 * 功能：
 * - 创建线程并设置实时调度策略（SCHED_FIFO）
 * - 设置线程优先级为 40
 * - 适用于需要高优先级、确定性行为的实时任务
 * - 销毁线程属性后设置调度参数
 */
int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   int ret;
   pthread_attr_t attr;
   struct sched_param schparam;
   pthread_t *threadp;

   threadp = thandle;
   pthread_attr_init(&attr);
   pthread_attr_setstacksize(&attr, stacksize);
   ret = pthread_create(threadp, &attr, func, param);
   pthread_attr_destroy(&attr);
   if (ret < 0)
   {
      return 0;
   }
   memset(&schparam, 0, sizeof(schparam));
   schparam.sched_priority = 40;
   ret = pthread_setschedparam(*threadp, SCHED_FIFO, &schparam);
   if (ret < 0)
   {
      return 0;
   }

   return 1;
}

/** 创建互斥锁
 * @return 返回互斥锁指针，失败返回 NULL
 * 
 * 功能：
 * - 分配互斥锁内存
 * - 初始化互斥锁属性
 * - 设置优先级继承协议（PTHREAD_PRIO_INHERIT）
 * - 防止优先级反转问题
 */
void *osal_mutex_create(void)
{
   pthread_mutexattr_t mutexattr;
   osal_mutext *mutex;
   mutex = (osal_mutext *)osal_malloc(sizeof(osal_mutext));
   if (mutex)
   {
      pthread_mutexattr_init(&mutexattr);
      pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
      pthread_mutex_init(mutex, &mutexattr);
   }
   return (void *)mutex;
}

/** 销毁互斥锁
 * @param mutex 互斥锁指针
 * 
 * 功能：
 * - 销毁互斥锁
 * - 释放互斥锁占用的内存
 */
void osal_mutex_destroy(void *mutex)
{
   pthread_mutex_destroy((osal_mutext *)mutex);
   osal_free(mutex);
}

/** 加锁（阻塞）
 * @param mutex 互斥锁指针
 * 
 * 功能：
 * - 尝试获取互斥锁
 * - 如果锁已被其他线程持有，则阻塞等待
 * - 获取成功后返回
 */
void osal_mutex_lock(void *mutex)
{
   pthread_mutex_lock((osal_mutext *)mutex);
}

/** 解锁
 * @param mutex 互斥锁指针
 * 
 * 功能：
 * - 释放互斥锁
 * - 允许其他等待的线程获取锁
 */
void osal_mutex_unlock(void *mutex)
{
   pthread_mutex_unlock((osal_mutext *)mutex);
}
