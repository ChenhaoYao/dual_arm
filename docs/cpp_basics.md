# C++ 基础语法笔记

## 内存：栈 vs 堆

程序运行时内存布局：

```
┌──────────────────────┐ 高地址
│      栈 (Stack)       │ ← 自动管理，函数调用用
│          ↓            │
│                       │
│          ↑            │
│      堆 (Heap)        │ ← 手动管理，大对象用
├──────────────────────┤
│    全局变量/常量      │
├──────────────────────┤
│    程序代码           │
└──────────────────────┘ 低地址
```

| | 栈 | 堆 |
|---|---|---|
| 分配 | 自动（函数进入时） | 手动（new/malloc） |
| 释放 | 自动（函数退出时） | 手动（delete/free）或智能指针 |
| 大小 | 小（几 MB） | 大（几 GB） |
| 速度 | 快 | 慢 |
| 适合 | 小对象、临时变量 | 大对象、需要跨函数传递 |

```cpp
// 栈上：函数结束自动释放
int x = 10;
std::string name = "hello";

// 堆上：需要手动释放或用智能指针
int* p = new int(10);
delete p;  // 手动释放

auto node = std::make_shared<SoemBridgeNode>();  // 智能指针自动释放
```

## 智能指针

### shared_ptr（共享所有权）

多个指针可以指向同一个对象，引用计数管理生命周期。

```cpp
auto a = std::make_shared<int>(10);
auto b = a;   // b 和 a 指向同一个 int，引用计数 = 2
auto c = a;   // 引用计数 = 3

// a, b, c 都销毁后，对象才自动释放
```

```
a ──→ ┌────────┐
b ──→ │ int(10)│ ← 引用计数 = 3
c ──→ └────────┘
```

### unique_ptr（独占所有权）

只有一个指针拥有对象，不能拷贝，只能移动。

```cpp
auto a = std::make_unique<int>(10);
auto b = a;                    // ❌ 编译错误
auto b = std::move(a);        // ✅ 移动，a 变成空
```

### 选择原则

| 场景 | 选择 |
|------|------|
| 只有一个地方使用 | `unique_ptr` |
| 多个地方共享同一个对象 | `shared_ptr` |
| ROS2 节点（spin 和 main 都需要持有） | `shared_ptr` |
| 成员变量，只有所属类使用 | `unique_ptr` |

## 多态

一个接口，多种实现。

```cpp
// 基类：定义接口
class HardwareInterface {
public:
    virtual void read() = 0;   // 纯虚函数
    virtual void write() = 0;
};

// 派生类：具体实现
class MockHardware : public HardwareInterface {
public:
    void read() override { /* 读假数据 */ }
    void write() override { /* 什么都不做 */ }
};

class RealHardware : public HardwareInterface {
public:
    void read() override { /* 读真实编码器 */ }
    void write() override { /* 发送给电机 */ }
};

// 使用：同一个基类指针，可以指向不同的派生类
HardwareInterface* hw;
hw = new MockHardware();   // 仿真时
hw->read();                // 调用 MockHardware::read

hw = new RealHardware();   // 实物时
hw->read();                // 调用 RealHardware::read
```

## ROS2 中的 spin 循环

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                      // 初始化 ROS2
    auto node = std::make_shared<SoemBridgeNode>(); // 创建节点（堆上）
    rclcpp::spin(node);                             // 事件循环（阻塞）
    rclcpp::shutdown();                             // 清理退出
}
```

`spin` 内部逻辑：

```
while (rclcpp::ok()) {
    等待事件（阻塞）
    ├── 收到话题消息 → 调用 subscription 回调
    ├── 收到服务请求 → 调用 service 回调
    ├── 定时器到期   → 调用 timer 回调
    └── Action 请求  → 调用 action 回调
}
```

## 常用语法速查

```cpp
// auto：自动推导类型
auto x = 10;                    // int
auto name = std::string("hi");  // std::string
auto node = std::make_shared<SoemBridgeNode>();  // std::shared_ptr<SoemBridgeNode>

// std::move：移动语义（避免拷贝）
std::vector<int> a = {1, 2, 3};
std::vector<int> b = std::move(a);  // a 变空，b 拥有数据
```

## Lambda 表达式

Lambda 是**匿名函数**，可以当作参数传递，常用于回调。

### 基本语法

```cpp
[捕获列表](参数列表) { 函数体 }
```

```cpp
// 最简单的 lambda
auto add = [](int a, int b) { return a + b; };
int result = add(1, 2);  // 3
```

### 捕获列表

```cpp
int x = 10;
std::string name = "hello";

// [] 不捕获任何外部变量
auto f1 = []() { /* 不能访问 x 或 name */ };

// [x] 值捕获：拷贝一份，lambda 内修改不影响外部
auto f2 = [x]() { return x; };  // 可以读 x
auto f3 = [x]() { x++; };       // ❌ 不能修改（const）

// [&x] 引用捕获：直接引用外部变量，lambda 内修改会影响外部
auto f4 = [&x]() { x++; };      // ✅ 可以修改 x

// [this] 捕获 this 指针：可以访问类成员
class Foo {
    int member_ = 10;
    void bar() {
        auto f = [this]() { return member_; };  // 访问成员变量
    }
};

// [=] 值捕获所有外部变量
// [&] 引用捕获所有外部变量
```

### 作为函数参数

```cpp
// lambda 可以作为参数传给函数
void do_something(std::function<void(int)> callback) {
    callback(42);
}

// 使用
do_something([](int x) { printf("%d\n", x); });

// 在 ROS2 中，lambda 作为回调传入
create_service<Trigger>(
    "~/clear_fault",                                    // 参数 1：服务名
    [this](auto req, auto res) {                        // 参数 2：lambda 回调
        handle_clear_fault(req, res);
    }
);
```

### Lambda 本质上是可调用对象

```cpp
// lambda 可以存到变量
auto callback = [this](auto req, auto res) {
    handle_clear_fault(req, res);
};

// 然后当作参数传入
create_service<Trigger>("~/clear_fault", callback);

// 两种写法等价
create_service<Trigger>("~/clear_fault", [this](auto req, auto res) {
    handle_clear_fault(req, res);
});
```

### Lambda vs 直接调用

```cpp
// lambda：传入一个"函数对象"，等收到请求时再调用
create_service<Trigger>("name",
    [this](auto req, auto res) {        // ← 这是一个对象，不会立即执行
        handle_clear_fault(req, res);    // ← 收到请求时才执行
    }
);

// 直接调用：立即执行，返回值传入（类型错误）
create_service<Trigger>("name",
    handle_clear_fault(req, res)         // ← 立即执行，结果传入（类型不匹配）
);
```

**lambda 是传入一个"待执行的函数"，直接调用是"立即执行并传入结果"。**

### 为什么成员函数需要 lambda

```cpp
// 普通函数：可以直接传入
void global_callback(auto req, auto res) { ... }
create_service<Trigger>("name", global_callback);  // ✅

// 成员函数：不能直接传入
class Foo {
    void callback(auto req, auto res) { ... }
};
create_service<Trigger>("name", Foo::callback);  // ❌ 编译错误

// 用 lambda + [this] 解决
create_service<Trigger>("name",
    [this](auto req, auto res) {
        callback(req, res);  // this->callback(req, res)
    }
);
```

### C vs C++ 函数指针

```c
// C 中，普通函数名自动变成函数指针
void my_func() { ... }
ctx.PO2SOconfig = my_func;  // ✅ 函数名自动转为函数指针
```

```cpp
// C++ 中，成员函数不能自动转为函数指针
class Foo {
    void handle_clear_fault(auto req, auto res) { ... }
};
create_service<Trigger>("name", this->handle_clear_fault);  // ❌

// 必须用 lambda 包装
create_service<Trigger>("name",
    [this](auto req, auto res) {
        handle_clear_fault(req, res);
    }
);
```

| 类型 | 能直接当回调？ | 原因 |
|------|---------------|------|
| C 普通函数 | ✅ | 函数名自动转为函数指针 |
| C++ 普通函数 | ✅ | 同上 |
| C++ 成员函数 | ❌ | 需要对象才能调用 |

### 成员函数 vs 普通函数

```cpp
// 普通函数：不属于任何类
void global_function() { ... }

// 成员函数：类里面定义的函数
class Foo {
public:
    void member_function() { ... }  // 成员函数
};

// 成员函数必须通过对象调用
Foo foo;
foo.member_function();         // 通过对象调用

// 在类内部，隐式使用 this
class Foo {
    void bar() { ... }
    void baz() {
        bar();        // 等价于 this->bar()
        this->bar();  // 显式写 this
    }
};
```

**为什么回调必须用成员函数？** 因为回调需要访问成员变量（如 `master_`、`dry_run_`），普通函数无法访问。

## if / else if / else 短路

`else if` 是短路的——前面条件满足就不再执行后面。

```cpp
if (a) {
    RCLCPP_ERROR(...);   // 条件 a 为真，执行这里，跳过所有后续分支
} else if (b) {
    RCLCPP_ERROR(...);   // a 为假，b 为真，执行这里
} else if (c) {
    RCLCPP_ERROR(...);   // a、b 都为假，c 为真，执行这里
} else {
    // 全部为假，执行这里
}
```

**同一时刻只执行一个分支。** 如果想让所有条件都检查（顺序执行），用独立的 `if`：

```cpp
if (a) { ... }  // 无论如何都检查
if (b) { ... }  // 无论如何都检查
if (c) { ... }  // 无论如何都检查
```

## 三元运算符

```cpp
// 条件 ? 真值 : 假值
cfg.slave = (i < slaves.size()) ? static_cast<uint16_t>(slaves[i])
                                : static_cast<uint16_t>(i + 1);
// 等价于
if (i < slaves.size()) {
    cfg.slave = static_cast<uint16_t>(slaves[i]);
} else {
    cfg.slave = static_cast<uint16_t>(i + 1);
}
```

## static_cast 类型转换

C++ 推荐的显式类型转换，编译时检查。

```cpp
int64_t big = 42;
uint16_t small = static_cast<uint16_t>(big);  // int64_t → uint16_t

int bits = 19;
int32_t val = static_cast<int32_t>(bits);      // int → int32_t
```

## std::vector::resize()

调整 vector 大小。多出来的位置用给定值填充，已有的不变。

```cpp
std::vector<double> v = {100.0, 50.0};  // size = 2
v.resize(5, 100.0);                      // size = 5，v = {100.0, 50.0, 100.0, 100.0, 100.0}
//              ↑ 填充值               ↑ 目标大小
// 原来的 100.0, 50.0 保持不变
```

## 模板函数调用

尖括号 `<>` 指定模板参数类型。

```cpp
// declare_parameter 是模板函数，<std::vector<double>> 指定返回类型
auto vec = declare_parameter<std::vector<double>>("gear_ratio", std::vector<double>(7, 100.0));
//         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//         模板参数：返回类型                         参数名        默认值（找不到时用这个）

// 常见模板用法
auto x = static_cast<int>(3.14);           // 类型转换
auto p = std::make_shared<Foo>(args...);   // 智能指针
auto v = declare_parameter<std::vector<double>>(name, default);  // ROS2 参数
```

```cpp
// 成员函数：可以访问成员变量
void handle_clear_fault(...) {
    if (dry_run_) { ... }   // ✅ 访问 dry_run_
    master_->feedback();     // ✅ 访问 master_
}

// 普通函数：无法访问成员变量
void handle_clear_fault(...) {
    if (dry_run_) { ... }   // ❌ 编译错误：dry_run_ 是谁的？
    master_->feedback();     // ❌ 编译错误：master_ 是谁的？
}
```

## 原子变量 std::atomic

### 为什么需要原子变量

多个线程同时读写同一个普通变量会导致数据竞争（未定义行为）。

```cpp
// 问题示例：两个线程同时操作普通变量
bool send_enabled = false;  // 普通变量

// 线程 A：写
send_enabled = true;

// 线程 B：读
if (send_enabled) { ... }   // 可能读到旧值（缓存不一致）或中间状态（撕裂读）
```

### std::atomic 用法

```cpp
#include <atomic>

std::atomic<bool> send_enabled{false};  // 声明，初始值 false

send_enabled.store(true);    // 原子写入
bool val = send_enabled.load();  // 原子读取
send_enabled = true;         // 等价于 store(true)，有隐式转换
```

### 原子操作保证

```
普通 bool:
  线程 A: send_enabled = true    ──→  可能只写了一半
  线程 B: if (send_enabled)      ──→  读到中间状态（撕裂读）

std::atomic<bool>:
  线程 A: send_enabled.store(true) ──→  完整写入，不会被打断
  线程 B: send_enabled.load()      ──→  要么读到旧值，要么读到新值，不会读到中间状态
```

### 在 ROS2 中的应用

soem_bridge_node 有两个线程同时访问共享变量：

```
spin 线程（主线程）              RT 线程（实时线程）
┌─────────────────────┐        ┌─────────────────────┐
│ handle_enable()     │        │ rt_loop()           │
│   send_enabled_     │        │   running_          │
│   .store(true)      │        │   .load()           │
│                     │        │                     │
│ on_controller_state │        │ submit_waypoints()  │
│   send_enabled_     │        │   running_          │
│   .load()           │        │   .load()           │
└─────────────────────┘        └─────────────────────┘
```

```cpp
// 声明
std::atomic<bool> send_enabled_{false};
std::atomic<bool> running_{false};

// spin 线程中写入
void handle_enable(...) {
    send_enabled_.store(request->data);   // enable 服务设置 true/false
}

// spin 线程中读取
void on_controller_state(...) {
    if (!send_enabled_.load()) {          // 检查是否允许发送
        return;
    }
    master_->submit_waypoints(waypoints);
}

// RT 线程中读取
bool SoemCsvMaster::submit_waypoints(...) {
    if (!running_.load()) {               // 检查 EtherCAT 是否启动
        return false;
    }
    // ... 发送指令
}
```

### atomic vs mutex

| | std::atomic | std::mutex |
|---|---|---|
| 适用 | 单个变量的读写 | 多个变量的复合操作 |
| 开销 | 极小（CPU 指令级） | 较大（系统调用） |
| 用法 | `.load()` / `.store()` | `lock()` / `unlock()` |
| 示例 | `running_.store(true)` | 保护一段代码块 |

```cpp
// atomic：适合单个 bool/int 的简单读写
std::atomic<bool> flag{false};
flag.store(true);

// mutex：适合多个变量需要同时修改
std::mutex mtx;
void update(int a, int b) {
    std::lock_guard<std::mutex> lock(mtx);  // 加锁
    x = a;   // 两个变量必须同时更新
    y = b;   // 如果用 atomic，x 和 y 之间可能被其他线程插入
}
```
