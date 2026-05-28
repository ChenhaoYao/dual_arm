/** \file
 * \brief Simple Open EtherCAT Master 示例代码（新版）
 *
 * 使用方法: simple_ng IFNAME1
 * IFNAME1 是网络接口名称，例如 'eth0'
 *
 * 这是一个最小化的测试程序，用于演示如何使用 SOEM 库：
 * - 初始化 EtherCAT 主站
 * - 自动配置从站
 * - 读取和写入过程数据
 * - 监控从站状态
 *
 * 主要功能：
 * 1. 扫描并配置所有 EtherCAT 从站
 * 2. 建立从站 I/O 映射
 * 3. 配置分布式时钟（DC）
 * 4. 启动从站到 OPERATIONAL 状态
 * 5. 循环读取从站数据并显示
 * 6. 监控从站状态变化
 */

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 定义现场总线结构体，封装 EtherCAT 上下文和相关参数
typedef struct
{
   ecx_contextt context;      // EtherCAT 上下文，包含从站列表、组列表等所有信息
   char *iface;               // 网络接口名称（如 "eth0"）这是地址变量
   uint8 group;               // 从站组号（默认为 0）
   int roundtrip_time;        // 数据往返时间（微秒）
   uint8 map[4096];           // I/O 映射缓冲区，用于存储从站的 PDO 映射配置
} Fieldbus;

/**
 * 初始化现场总线结构体
 * @param fieldbus 指向 Fieldbus 结构体的指针
 * @param iface 网络接口名称字符串
 * 
 * 功能：
 * - 将结构体清零，避免未初始化的数据
 * - 设置网络接口名称
 * - 设置默认组号为 0
 * - 初始化往返时间为 0
 */
// Fieldbus *fieldbus这里的*是指针，指向Fieldbus结构体的地址
static void
fieldbus_initialize(Fieldbus *fieldbus, char *iface)
{
   /* 首先将 fieldbus 清零，避免未初始化的数据导致意外行为 */
   // memset 是 C 标准库函数，用于将一段内存区域设置为指定的值
   // *fieldbus这里的*是解引用操作符，获取fieldbus指针指向的结构体对象，和上面的用法不一样
   memset(fieldbus, 0, sizeof(*fieldbus));

   fieldbus->iface = iface;       // 保存网络接口名称
   fieldbus->group = 0;           // 设置默认组号为 0
   fieldbus->roundtrip_time = 0; // 初始化往返时间为 0
}

/**
 * 执行一次完整的数据往返（发送和接收过程数据）
 * @param fieldbus 指向 Fieldbus 结构体的指针
 * @return 工作计数器（WKC），表示成功响应的从站数量
 * 
 * 功能：
 * 1. 发送输出数据到所有从站
 * 2. 接收从站的输入数据
 * 3. 计算并记录往返时间
 * 
 * 这是 EtherCAT 数据交换的核心函数，每次调用都会：
 * - 将主站的输出数据发送给从站
 * - 从从站读取输入数据
 * - 返回工作计数器，用于验证数据交换是否成功
 */
static int
fieldbus_roundtrip(Fieldbus *fieldbus)
{
   ecx_contextt *context;   // EtherCAT 上下文指针
   ec_timet start, end, diff; // 时间变量，用于计算往返时间
   int wkc;                 // 工作计数器（Working Counter）

   context = &fieldbus->context; // 获取上下文指针

   start = osal_current_time();                    // 记录开始时间
   ecx_send_processdata(context);                 // 发送过程数据到从站
   wkc = ecx_receive_processdata(context, EC_TIMEOUTRET); // 接收从站的过程数据
   end = osal_current_time();                     // 记录结束时间
   osal_time_diff(&start, &end, &diff);          // 计算时间差
   fieldbus->roundtrip_time = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000); // 转换为微秒

   return wkc; // 返回工作计数器
}

/**
 * 启动现场总线，完成从站配置和状态转换
 * @param fieldbus 指向 Fieldbus 结构体的指针
 * @return 成功返回 TRUE，失败返回 FALSE
 * 
 * 功能流程：
 * 1. 初始化 SOEM 库，打开网络接口
 * 2. 扫描并自动配置所有从站
 * 3. 配置 I/O 映射
 * 4. 配置分布式时钟（DC）
 * 5. 等待从站进入 SAFE_OP 状态
 * 6. 发送一次数据使从站输出数据有效
 * 7. 将所有从站转换到 OPERATIONAL 状态
 */
static boolean
fieldbus_start(Fieldbus *fieldbus)
{
   ecx_contextt *context; // EtherCAT 上下文指针
   ec_groupt *grp;        // 从站组指针
   ec_slavet *slave;      // 从站指针
   int i;                 // 循环变量

   context = &fieldbus->context;           // 获取上下文指针
   grp = context->grouplist + fieldbus->group; // 获取指定组的指针

   // 步骤1: 初始化 SOEM 库，打开网络接口
   printf("Initializing SOEM on '%s'... ", fieldbus->iface);
   if (!ecx_init(context, fieldbus->iface)) // 初始化网络套接字
   {
      printf("no socket connection\n"); // 初始化失败
      return FALSE;
   }
   printf("done\n");

   // 步骤2: 扫描并自动配置所有从站
   // 此函数会：
   // - 读取每个从站的 EEPROM 信息
   // - 获取制造商 ID、产品代码、名称等信息
   // - 填充 context->slavelist 数组
   printf("Finding autoconfig slaves... ");
   if (ecx_config_init(context) <= 0) // 配置从站，返回从站数量
   {
      printf("no slaves found\n"); // 未找到从站
      return FALSE;
   }
   printf("%d slaves found\n", context->slavecount); // 显示找到的从站数量

   // 步骤3: 配置 I/O 映射
   // 将从站的 PDO 映射到连续的内存区域
   // 配置后可以通过 grp->outputs 和 grp->inputs 数组访问从站数据
   printf("Sequential mapping of I/O... ");
   ecx_config_map_group(context, fieldbus->map, fieldbus->group);
   printf("mapped %dO+%dI bytes from %d segments",
          grp->Obytes, grp->Ibytes, grp->nsegments); // 显示输出/输入字节数和段数
   if (grp->nsegments > 1) // 如果有多个段，显示每个段的从站分布
   {
      /* 显示从站在各个段的分布情况 */
      for (i = 0; i < grp->nsegments; ++i)
      {
         printf("%s%d", i == 0 ? " (" : "+", grp->IOsegment[i]);
      }
      printf(" slaves)");
   }
   printf("\n");

   // 步骤4: 配置分布式时钟（DC）
   // 用于同步从站的时钟，实现精确的时间控制
   printf("Configuring distributed clock... ");
   ecx_configdc(context);
   printf("done\n");

   // 步骤5: 等待所有从站进入 SAFE_OP 状态
   // SAFE_OP 状态下从站可以交换数据但不能执行应用程序
   printf("Waiting for all slaves in safe operational... ");
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4); // 检查状态，超时时间为默认的4倍
   printf("done\n");

   // 步骤6: 发送一次数据交换
   // 这是为了让从站的输出数据有效，避免从站输出未知数据
   printf("Send a roundtrip to make outputs in slaves happy... ");
   fieldbus_roundtrip(fieldbus);
   printf("done\n");

   // 步骤7: 将所有从站转换到 OPERATIONAL 状态
   // OPERATIONAL 状态下从站可以正常执行应用程序
   printf("Setting operational state..");
   /* 操作从站 0（一个虚拟从站，用于广播） */
   slave = context->slavelist;              // 获取从站 0（广播从站）
   slave->state = EC_STATE_OPERATIONAL;    // 设置目标状态为 OPERATIONAL
   ecx_writestate(context, 0);             // 写入状态，广播到所有从站
   /* 轮询结果，最多尝试10次 */
   for (i = 0; i < 10; ++i)
   {
      printf(".");
      fieldbus_roundtrip(fieldbus);                              // 执行数据交换
      ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10); // 检查状态
      if (slave->state == EC_STATE_OPERATIONAL)                  // 如果已进入 OPERATIONAL 状态
      {
         printf(" all slaves are now operational\n");
         return TRUE; // 成功
      }
   }

   // 如果10次尝试后仍未进入 OPERATIONAL 状态，显示失败信息
   printf(" failed,");
   ecx_readstate(context); // 读取所有从站的当前状态
   for (i = 1; i <= context->slavecount; ++i) // 遍历所有从站
   {
      slave = context->slavelist + i; // 获取从站 i
      if (slave->state != EC_STATE_OPERATIONAL) // 如果该从站未进入 OPERATIONAL 状态
      {
         printf(" slave %d is 0x%04X (AL-status=0x%04X %s)",
                i, slave->state, slave->ALstatuscode, // 显示状态和 AL 状态码
                ec_ALstatuscode2string(slave->ALstatuscode)); // 显示 AL 状态码的字符串描述
      }
   }
   printf("\n");

   return FALSE; // 返回失败
}

/**
 * 停止现场总线，关闭所有从站
 * @param fieldbus 指向 Fieldbus 结构体的指针
 * 
 * 功能：
 * 1. 将所有从站转换到 INIT 状态
 * 2. 关闭网络套接字
 */
static void
fieldbus_stop(Fieldbus *fieldbus)
{
   ecx_contextt *context; // EtherCAT 上下文指针
   ec_slavet *slave;      // 从站指针

   context = &fieldbus->context; // 获取上下文指针
   /* 操作从站 0（一个虚拟从站，用于广播） */
   slave = context->slavelist; // 获取从站 0

   // 将所有从站转换到 INIT 状态
   printf("Requesting init state on all slaves... ");
   slave->state = EC_STATE_INIT; // 设置目标状态为 INIT
   ecx_writestate(context, 0);  // 写入状态，广播到所有从站
   printf("done\n");

   // 关闭网络套接字
   printf("Close socket... ");
   ecx_close(context); // 关闭网络连接
   printf("done\n");
}

/**
 * 读取并显示导出从站数据
 * @param fieldbus 指向 Fieldbus 结构体的指针
 * @return 数据交换成功返回 TRUE，失败返回 FALSE
 * 
 * 功能：
 * 1. 执行一次数据往返
 * 2. 验证工作计数器（WKC）
 * 3. 显示输出数据（主站→从站）
 * 4. 显示输入数据（从站→主站）
 * 5. 显示分布式时钟时间
 */
static boolean
fieldbus_dump(Fieldbus *fieldbus)
{
   ecx_contextt *context; // EtherCAT 上下文指针
   ec_groupt *grp;        // 从站组指针
   uint32 n;              // 循环变量
   int wkc, expected_wkc; // 工作计数器和期望的工作计数器

   context = &fieldbus->context;           // 获取上下文指针
   grp = context->grouplist + fieldbus->group; // 获取指定组的指针

   wkc = fieldbus_roundtrip(fieldbus); // 执行数据往返，获取工作计数器
   expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC; // 计算期望的工作计数器
   printf("%6d usec  WKC %d", fieldbus->roundtrip_time, wkc); // 显示往返时间和 WKC
   if (wkc < expected_wkc) // 如果 WKC 小于期望值，说明数据交换失败
   {
      printf(" wrong (expected %d)\n", expected_wkc);
      return FALSE; // 返回失败
   }

   // 显示输出数据（主站发送给从站的数据）
   printf("  O:");
   for (n = 0; n < grp->Obytes; ++n) // 遍历所有输出字节
   {
      printf(" %02X", grp->outputs[n]); // 以十六进制显示每个字节
   }
   // 显示输入数据（从站发送给主站的数据）
   printf("  I:");
   for (n = 0; n < grp->Ibytes; ++n) // 遍历所有输入字节
   {
      printf(" %02X", grp->inputs[n]); // 以十六进制显示每个字节
   }
   // 显示分布式时钟时间
   printf("  T: %lld\r", (long long)context->DCtime);
   return TRUE; // 返回成功
}

/**
 * 检查并处理从站状态变化
 * @param fieldbus 指向 Fieldbus 结构体的指针
 * 
 * 功能：
 * 1. 读取所有从站的当前状态
 * 2. 检测从站是否丢失
 * 3. 尝试恢复丢失的从站
 * 4. 处理从站错误状态
 * 5. 尝试将从站恢复到 OPERATIONAL 状态
 */
static void
fieldbus_check_state(Fieldbus *fieldbus)
{
   ecx_contextt *context; // EtherCAT 上下文指针
   ec_groupt *grp;        // 从站组指针
   ec_slavet *slave;      // 从站指针
   int i;                 // 循环变量

   context = &fieldbus->context;           // 获取上下文指针
   grp = context->grouplist + fieldbus->group; // 获取指定组的指针
   grp->docheckstate = FALSE;              // 初始化状态检查标志
   ecx_readstate(context);                 // 读取所有从站的当前状态
   for (i = 1; i <= context->slavecount; ++i) // 遍历所有从站
   {
      slave = context->slavelist + i; // 获取从站 i
      if (slave->group != fieldbus->group) // 如果该从站不属于当前组
      {
         /* 该从站属于其他组：不做处理 */
      }
      else if (slave->state != EC_STATE_OPERATIONAL) // 如果从站不在 OPERATIONAL 状态
      {
         grp->docheckstate = TRUE; // 标记需要检查状态
         if (slave->state == EC_STATE_SAFE_OP + EC_STATE_ERROR) // 如果从站处于 SAFE_OP+ERROR 状态
         {
            printf("* Slave %d is in SAFE_OP+ERROR, attempting ACK\n", i);
            slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK; // 设置为 ACK 状态以确认错误
            ecx_writestate(context, i); // 写入状态
         }
         else if (slave->state == EC_STATE_SAFE_OP) // 如果从站处于 SAFE_OP 状态
         {
            printf("* Slave %d is in SAFE_OP, change to OPERATIONAL\n", i);
            slave->state = EC_STATE_OPERATIONAL; // 设置为 OPERATIONAL 状态
            ecx_writestate(context, i); // 写入状态
         }
         else if (slave->state > EC_STATE_NONE) // 如果从站状态有效但非 OPERATIONAL
         {
            if (ecx_reconfig_slave(context, i, EC_TIMEOUTRET)) // 尝试重新配置从站
            {
               slave->islost = FALSE; // 标记从站未丢失
               printf("* Slave %d reconfigured\n", i);
            }
         }
         else if (!slave->islost) // 如果从站未标记为丢失
         {
            ecx_statecheck(context, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET); // 检查状态
            if (slave->state == EC_STATE_NONE) // 如果状态为 NONE，说明从站丢失
            {
               slave->islost = TRUE; // 标记从站丢失
               printf("* Slave %d lost\n", i);
            }
         }
      }
      else if (slave->islost) // 如果从站之前标记为丢失但现在处于 OPERATIONAL 状态
      {
         if (slave->state != EC_STATE_NONE) // 如果状态有效
         {
            slave->islost = FALSE; // 标记从站已找到
            printf("* Slave %d found\n", i);
         }
         else if (ecx_recover_slave(context, i, EC_TIMEOUTRET)) // 尝试恢复从站
         {
            slave->islost = FALSE; // 标记从站已恢复
            printf("* Slave %d recovered\n", i);
         }
      }
   }

   if (!grp->docheckstate) // 如果没有从站需要检查状态
   {
      printf("All slaves resumed OPERATIONAL\n"); // 显示所有从站已恢复 OPERATIONAL 状态
   }
}

/**
 * 主函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 成功返回 0，失败返回 1
 * 
 * 功能：
 * 1. 检查命令行参数
 * 2. 如果参数错误，显示可用网络适配器
 * 3. 初始化现场总线
 * 4. 启动现场总线
 * 5. 循环读取从站数据（10000次）
 * 6. 显示往返时间统计
 * 7. 停止现场总线
 */
int main(int argc, char *argv[])
{
   Fieldbus fieldbus; // 现场总线结构体

   // 检查命令行参数
   if (argc != 2) // 如果参数个数不为 2 两个参数分别是程序名和网络接口名
   {
      ec_adaptert *adapter = NULL; // 适配器指针
      ec_adaptert *head = NULL;    // 适配器链表头指针
      printf("Usage: simple_ng IFNAME1\n"
             "IFNAME1 is the NIC interface name, e.g. 'eth0'\n"); // 显示用法

      // 显示所有可用的网络适配器
      printf("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters(); // 查找所有适配器
      while (adapter != NULL) // 遍历适配器链表
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc); // 显示适配器名称和描述
         adapter = adapter->next; // 移动到下一个适配器
      }
      ec_free_adapters(head); // 释放适配器链表内存
      return 1; // 返回错误
   }

   // 初始化现场总线
   fieldbus_initialize(&fieldbus, argv[1]); // 使用命令行参数指定的网络接口
   if (fieldbus_start(&fieldbus)) // 启动现场总线
   {
      int i, min_time, max_time; // 循环变量和时间统计变量
      min_time = max_time = 0; // 初始化最小和最大时间
      for (i = 1; i <= 1000; ++i) // 循环 1000 次
      {
         printf("Iteration %4d:", i); // 显示迭代次数
         if (!fieldbus_dump(&fieldbus)) // 读取并显示从站数据
         {
            fieldbus_check_state(&fieldbus); // 如果数据交换失败，检查从站状态
         }
         else if (i == 1) // 第一次成功的数据交换
         {
            min_time = max_time = fieldbus.roundtrip_time; // 初始化最小和最大时间
         }
         else if (fieldbus.roundtrip_time < min_time) // 更新最小时间
         {
            min_time = fieldbus.roundtrip_time;
         }
         else if (fieldbus.roundtrip_time > max_time) // 更新最大时间
         {
            max_time = fieldbus.roundtrip_time;
         }
         osal_usleep(5000); // 延时 5ms
      }
      printf("\nRoundtrip time (usec): min %d max %d\n", min_time, max_time); // 显示往返时间统计
      fieldbus_stop(&fieldbus); // 停止现场总线
   }

   return 0; // 返回成功
}
