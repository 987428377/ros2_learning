/**
 *这段代码使用 ROS 2 和 C++ 创建了一个自定义内存分配器（MyAllocator），并通过这个自定义分配器来管理消息发布和订阅的内存分配。
 *  主要目的是展示如何跟踪程序执行过程中内存的分配和释放情况。
*/


#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

// For demonstration purposes only, not necessary for allocator_traits
//num_allocs 和 num_deallocs：用来记录自定义分配器的分配和释放次数。
static uint32_t num_allocs = 0;
static uint32_t num_deallocs = 0;
// A very simple custom allocator. Counts calls to allocate and deallocate.
//这是一个非常简单的自定义内存分配器类，MyAllocator。它的目的是通过 allocate 和 deallocate 函数来管理内存分配，同时计数分配和释放的次数。
template<typename T = void>
struct MyAllocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
  }
    // 方法分配内存，并增加分配计数。
  T * allocate(size_t size, const void * = 0)
  {
    if (size == 0) {
      return nullptr;
    }
    num_allocs++;
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }
    //deallocate 方法释放内存，并增加释放计数。
  void deallocate(T * ptr, size_t size)
  {
    (void)size;
    if (!ptr) {
      return;
    }
    num_deallocs++;
    std::free(ptr);
  }
    //rebind 是标准的 C++ 分配器模式，用于使分配器可以处理不同类型的数据。
  template<typename U>
  struct rebind
  {
    typedef MyAllocator<U> other;
  };
};


//自定义分配器比较运算符
template<typename T, typename U>
constexpr bool operator==(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return true;
}

template<typename T, typename U>
constexpr bool operator!=(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return false;
}

// Override global new and delete to count calls during execution.

static bool is_running = false;
static uint32_t global_runtime_allocs = 0;
static uint32_t global_runtime_deallocs = 0;
//重载 new 操作符：重载的 new 操作符用于在程序中创建对象时分配内存。
void * operator new(std::size_t size)
{
    //这用于跟踪程序在运行过程中使用全局 new 进行的内存分配次数。
  if (is_running) {
    global_runtime_allocs++;//每次调用 new 分配内存时，如果 is_running 为 true，则增加 global_runtime_allocs 计数器。
  }
  return std::malloc(size);
}

//提供了两个版本的 delete，一个带有大小参数，一个不带大小参数。这两个版本都处理指针的释放。
void operator delete(void * ptr, size_t size) noexcept
{
  (void)size;
  if (ptr != nullptr) {
    if (is_running) {
        //这用于跟踪程序中发生的全局 delete 操作次数。
      global_runtime_deallocs++;//每次调用 delete 释放内存时，如果 is_running 为 true，则增加 global_runtime_deallocs 计数器。
    }
    std::free(ptr);
  }
}

void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

int main(int argc, char ** argv)
{
//它使得在后续代码中使用 AllocatorMemoryStrategy 时不再需要写完整的命名空间路径。
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

//定义了 Alloc 作为 MyAllocator<void> 的别名。
  using Alloc = MyAllocator<void>;

//rclcpp::allocator::AllocRebind 是一个模板工具类，用于将一个分配器 (Alloc) 绑定到特定的消息类型 (std_msgs::msg::UInt32) 上。
//这个工具类允许你创建适用于特定类型消息的分配器。
//通过 AllocRebind，你可以定义一个特定于消息类型的分配器，以便在 ROS2 消息的分配和释放过程中使用自定义的内存管理策略。
  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<std_msgs::msg::UInt32, Alloc>;//将一个分配器 (Alloc) 绑定到特定的消息类型 (std_msgs::msg::UInt32)

//allocator_type 是 AllocRebind 模板类的成员类型，它定义了适用于指定消息类型的分配器类型。
//使用 MessageAlloc 来表示适用于 std_msgs::msg::UInt32 类型消息的分配器类型
  using MessageAlloc = MessageAllocTraits::allocator_type;

//定义了一个名为 MessageDeleter 的类型别名，用于表示与特定消息类型和分配器类型兼容的删除器（deleter）类型。
//rclcpp::allocator::Deleter 是一个模板类，用于创建与特定消息类型和分配器兼容的删除器。它配合自定义分配器使用，确保消息资源在不再需要时被正确释放。
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, std_msgs::msg::UInt32>;

//定义了一个名为 MessageUniquePtr 的智能指针类型，它结合了 std::unique_ptr 和自定义删除器 MessageDeleter。
  using MessageUniquePtr = std::unique_ptr<std_msgs::msg::UInt32, MessageDeleter>;
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;

  std::list<std::string> keys = {"intra", "intraprocess", "intra-process", "intra_process"};
  bool intra_process = false;

  printf(
    "This simple demo shows off a custom memory allocator to count all\n"
    "instances of new/delete in the program.  It can be run in either regular\n"
    "mode (no arguments), or in intra-process mode (by passing 'intra' as a\n"
    "command-line argument).  It will then publish a message to the\n"
    "'/allocator_tutorial' topic every 10 milliseconds until Ctrl-C is pressed.\n"
    "At that time it will print a count of the number of allocations and\n"
    "deallocations that happened during the program.\n\n");

  if (argc > 1) {
    for (auto & key : keys) {
      if (std::string(argv[1]) == key) {
        intra_process = true;
        break;
      }
    }
  }

//是否使用进程内通讯
  if (intra_process) {
    printf("Intra-process pipeline is ON.\n");
    //获取 ROS 2 的全局默认上下文对象。上下文对象提供了 ROS 2 环境的全局配置和状态管理。
    auto context = rclcpp::contexts::get_global_default_context();
    //创建节点选项，指定上下文对象，并启用进程内通信。
    auto options = rclcpp::NodeOptions()
      .context(context)
      .use_intra_process_comms(true);
    node = rclcpp::Node::make_shared("allocator_tutorial", options);
  } else {
    auto options = rclcpp::NodeOptions()
      .use_intra_process_comms(false);
    printf("Intra-process pipeline is OFF.\n");
    node = rclcpp::Node::make_shared("allocator_tutorial", options);
  }


//回调函数的作用是增加一个计数器，用于统计接收到的消息数量。
  uint32_t counter = 0;
  //std_msgs::msg::UInt32::ConstSharedPtr msg 是回调函数的参数，表示接收到的消息。
  //这是一个 std::shared_ptr 类型，指向 std_msgs::msg::UInt32 消息类型的常量版本。
  //-> void 表示回调函数的返回类型是 void，即函数没有返回值。
  auto callback = [&counter](std_msgs::msg::UInt32::ConstSharedPtr msg) -> void
    {
      (void)msg;//这一行代码将 msg 参数标记为未使用。这样做可以避免编译器关于未使用变量的警告，因为在这个回调函数中 msg 参数实际上没有被使用。
      ++counter;
    };





//这段代码展示了如何在 ROS 2 中使用自定义分配器（Allocator）来创建发布者（Publisher）和订阅者（Subscriber）。
//自定义分配器可以用于控制内存分配和释放策略，尤其是在需要优化内存管理或监控内存使用的情况下。
  // Create a custom allocator and pass the allocator to the publisher and subscriber.

  //创建自定义分配器:创建了一个 Alloc 类型的自定义分配器的智能指针 alloc。
  auto alloc = std::make_shared<Alloc>();

    //设置发布者选项:将自定义分配器分配给 publisher_options，以便在发布消息时使用这个分配器。
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
  publisher_options.allocator = alloc;
  auto publisher = node->create_publisher<std_msgs::msg::UInt32>(
    "allocator_tutorial", 10, publisher_options);

    //设置订阅者选项: 将自定义分配器分配给 subscription_options，以便在处理订阅消息时使用这个分配器。
  rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;
  subscription_options.allocator = alloc;

    //创建消息内存策略:
    //rclcpp::message_memory_strategy::MessageMemoryStrategy 是 ROS 2 中的一个类，用于管理消息的内存分配。
    //MessageMemoryStrategy 使用 Alloc 作为分配器来管理消息的内存。
    //msg_mem_strat 是一个智能指针，指向这个消息内存策略对象。
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      std_msgs::msg::UInt32, Alloc>>(alloc);
  auto subscriber = node->create_subscription<std_msgs::msg::UInt32>(
    "allocator_tutorial", 10, callback, subscription_options, msg_mem_strat);

  // Create a MemoryStrategy, which handles the allocations made by the Executor during the
  // execution path, and inject the MemoryStrategy into the Executor.
  //创建内存策略（MemoryStrategy）：AllocatorMemoryStrategy<Alloc> 是一个自定义的内存策略，它使用了先前定义的 Alloc 内存分配器。
  //memory_strategy 对象将用于处理执行器在执行过程中进行的内存分配和释放操作。
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);

  //配置执行器选项（ExecutorOptions）：options.memory_strategy 设置为之前创建的 memory_strategy。这将确保执行器在运行时使用自定义的内存策略来管理内存。
  rclcpp::ExecutorOptions options;
  options.memory_strategy = memory_strategy;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  // Add our node to the executor.
  executor.add_node(node);


    //创建消息的删除器：该删除器会被用来在消息生命周期结束时调用自定义的 deallocate 方法。
  MessageDeleter message_deleter;
    //设置消息的内存分配器
    //message_alloc 是从共享的 alloc 对象中获取的内存分配器实例。
  MessageAlloc message_alloc = *alloc;
  //函数用于将自定义的内存分配器设置到 message_deleter 上。这意味着，当 message_deleter 释放消息时，它将使用 message_alloc 进行内存释放操作。
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

  rclcpp::sleep_for(1ms);
  is_running = true;

  uint32_t i = 0;
  while (rclcpp::ok()) {
    // Create a message with the custom allocator, so that when the Executor deallocates the
    // message on the execution path, it will use the custom deallocate.
    auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
    MessageAllocTraits::construct(message_alloc, ptr);
    MessageUniquePtr msg(ptr, message_deleter);
    msg->data = i;
    ++i;
    publisher->publish(std::move(msg));
    rclcpp::sleep_for(10ms);
    executor.spin_some();
  }
  is_running = false;

  uint32_t final_global_allocs = global_runtime_allocs;
  uint32_t final_global_deallocs = global_runtime_deallocs;
  printf("Global new was called %u times during spin\n", final_global_allocs);
  printf("Global delete was called %u times during spin\n", final_global_deallocs);

  printf("Allocator new was called %u times during spin\n", num_allocs);
  printf("Allocator delete was called %u times during spin\n", num_deallocs);

  return 0;
}