#ifndef REFLECTION_H
#define REFLECTION_H

#include <context_monitor/serialize/eigen_serialize.h>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>
// 类型信息基类
class TypeInfo {
public:
  virtual ~TypeInfo() = default;
  virtual std::string getTypeName() const = 0;
  virtual std::string toString(const void *obj) const = 0;
  virtual bool fromString(void *obj, const std::string &str) const = 0;
  virtual std::unique_ptr<TypeInfo> clone() const = 0;
};

// 具体类型信息模板
template <typename T> class ConcreteTypeInfo : public TypeInfo {
public:
  std::string getTypeName() const override { return typeid(T).name(); }

  std::string toString(const void *obj) const override {
    std::stringstream ss;
    ss << *static_cast<const T *>(obj);
    return ss.str();
  }

  bool fromString(void *obj, const std::string &str) const override {
    std::stringstream ss(str);
    T temp;
    ss >> temp;
    if (!ss.fail()) {
      *static_cast<T *>(obj) = temp;
    }
    return !ss.fail();
  }

  std::unique_ptr<TypeInfo> clone() const override {
    return std::make_unique<ConcreteTypeInfo<T>>();
  }
};

// 字符串类型的特殊化处理
template <> class ConcreteTypeInfo<std::string> : public TypeInfo {
public:
  std::string getTypeName() const override { return "std::string"; }

  std::string toString(const void *obj) const override {
    return *static_cast<const std::string *>(obj);
  }

  bool fromString(void *obj, const std::string &str) const override {
    *static_cast<std::string *>(obj) = str;
    return true;
  }

  std::unique_ptr<TypeInfo> clone() const override {
    return std::make_unique<ConcreteTypeInfo<std::string>>();
  }
};
// 为std::shared_ptr添加特殊化处理
template <typename T>
class ConcreteTypeInfo<std::shared_ptr<T>> : public TypeInfo {
public:
  std::string getTypeName() const override {
    return "std::shared_ptr<" + ConcreteTypeInfo<T>().getTypeName() + ">";
  }

  std::string toString(const void *obj) const override {
    const auto &ptr = *static_cast<const std::shared_ptr<T> *>(obj);
    if (!ptr) {
      return "[nullptr]";
    }
    // 委托给指向类型的toString方法
    return ConcreteTypeInfo<T>().toString(ptr.get());
  }

  bool fromString(void *obj, const std::string &str) const override {
    auto &ptr = *static_cast<std::shared_ptr<T> *>(obj);
    if (!ptr) {
      // 如果指针为空，创建一个新对象
      ptr = std::make_shared<T>();
    }
    // 委托给指向类型的fromString方法
    return ConcreteTypeInfo<T>().fromString(ptr.get(), str);
  }

  std::unique_ptr<TypeInfo> clone() const override {
    return std::make_unique<ConcreteTypeInfo<std::shared_ptr<T>>>();
  }
};
// 反射系统单例类
class ReflectionSystem {
private:
  ReflectionSystem() = default;

  struct VariableData {
    void *pointer;
    std::unique_ptr<TypeInfo> typeInfo;
  };

  std::unordered_map<std::string, VariableData> variables_;
  mutable std::timed_mutex mutex_;

public:
  // 获取单例实例
  static ReflectionSystem &getInstance() {
    static ReflectionSystem instance;
    return instance;
  }

  ReflectionSystem(const ReflectionSystem &) = delete;
  ReflectionSystem &operator=(const ReflectionSystem &) = delete;
  ReflectionSystem(ReflectionSystem &&) = delete;
  ReflectionSystem &operator=(ReflectionSystem &&) = delete;

  // 注册变量
  template <typename T>
  void registerVariable(const std::string &name, T &variable) {
    std::lock_guard<std::timed_mutex> lock(mutex_);
    variables_[name] = {&variable, std::make_unique<ConcreteTypeInfo<T>>()};
  }

  // 获取变量值
  bool getVariable(const std::string &name, std::string &value,
                   std::string &type) const {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    // 避免服务器长时间阻塞
    if (!lock.try_lock_for(std::chrono::milliseconds(100))) {
      return false;
    }
    auto it = variables_.find(name);
    if (it == variables_.end()) {
      return false;
    }

    value = it->second.typeInfo->toString(it->second.pointer);
    type = it->second.typeInfo->getTypeName();
    return true;
  }

  // 设置变量值
  bool setVariable(const std::string &name, const std::string &value) {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    // 避免服务器长时间阻塞
    if (!lock.try_lock_for(std::chrono::milliseconds(100))) {
      return false;
    }
    auto it = variables_.find(name);
    if (it == variables_.end()) {
      return false;
    }

    return it->second.typeInfo->fromString(it->second.pointer, value);
  }

  // 获取所有变量名
  std::vector<std::string> listVariables() const {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    // 避免服务器长时间阻塞
    if (!lock.try_lock_for(std::chrono::milliseconds(100))) {
      return {};
    }
    std::vector<std::string> names;
    for (const auto &pair : variables_) {
      names.push_back(pair.first);
    }
    return names;
  }
};

// 注册变量的宏
#define REGISTER_MONITOR_VARIABLE(name)                                        \
  ReflectionSystem::getInstance().registerVariable(#name, name)

#endif // REFLECTION_H
