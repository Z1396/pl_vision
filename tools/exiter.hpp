#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
class Exiter
{
public:
  Exiter();

  /*2. 核心特性：const 修饰成员函数的作用
  const 是这行声明的关键，其唯一核心作用是保证该成员函数执行过程中，不会修改所属类对象的任何成员变量（包括普通成员变量、非mutable修饰的成员变量）。
  简单说：调用exit()函数时，只会 “读取” 对象的成员变量（如判断状态），不会 “修改” 对象的任何数据，是一个只读操作的成员函数。
  补充：mutable 例外情况
  若类中有成员变量被mutable关键字修饰（如mutable int count;），则const成员函数可以修改该变量（这是 C++ 为 “只读操作中需要轻微修改辅助变量” 设计的例外，如统计函数调用次数）。
  3. 常成员函数的调用规则（重要）
  const修饰的成员函数有严格的调用限制，核心规则：普通对象和 const 常量对象都能调用，且是 const 对象的唯一可调用成员函数类型，具体分两种场景：

  对于const 常量对象（如const 类名 obj;）：只能调用类中被const修饰的成员函数，无法调用普通成员函数（防止修改常量对象的成员）；
  对于普通对象（如类名 obj;）：既可以调用const成员函数，也可以调用普通成员函数。*/
  bool exit() const;
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP