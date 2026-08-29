# stage8_toml_reflection —— toml 模块（反射式解析 + 配置合并）

## 功能分析

`toml` 模块（真实项目约 3,960 行，INTERFACE header-only）的核心命题：
**定义 struct 后自动从 TOML 表填充，不写解析代码**。

| 能力 | 语义 |
|---|---|
| 普通字段 | 缺省时保留 struct 内默认值 |
| `required<T>` | 必填，缺失即失败（fail-fast，对应项目"配置缺字段直接退出"规范），错误信息带字段名 |
| `std::optional<T>` | 缺省 = `nullopt`，与显式赋值可区分 |
| `flatten<T>` | serde 扁平化：`T` 的字段来自同一张表（如 `[camera]` 下直接放 `fx`） |
| `std::vector<T>` | TOML array 容器 |
| 嵌套 struct | 子表递归（`[team]`） |
| `merge_configs` | 配置分层合并：base <- overlay，双侧 table 递归深合并，其余覆盖 |

实现机制：`TOML_REFLECT(Struct, field...)` 宏注册"字段名 + 成员指针"列表，
`from_table<T>` 按类型分派逐字段解析，返回 `std::expected<T, std::string>`。
本 stage 自带 toml++（v3.4.0 单头文件）作为 TOML 解析前端。

## 运行方法

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/demo
```

## 预期输出（节选）

```
=== stage8_toml_reflection ===
[test 1] 全字段解析
  Team Color: blue
  Team ID: 7  Members: [1, 2, 3]
  Camera: front
  Camera Intrinsics: fx=920.000000 fy=920.500000 cx=640.000000 cy=360.000000
  [PASS] flatten：[camera] 表内直接取到 fx/cy
[test 2] required 缺失 fail-fast
  错误信息: table 'team': missing required field 'id'
  [PASS] 错误信息带字段名 id
[test 4] merge_configs 分层合并
  aim.max_distance = 12.000000   <- overlay 覆盖
  aim.fire_rate    = 5           <- base 保留

ALL TESTS PASSED
```

踩过的三个坑（都值得写进复盘）：

1. **宏续行符后不能有尾随空格**——`\` 不在行尾即续行断裂，宏体被截断；
2. **FOR_EACH 递归展开要补逗号**——`m(s, x) FE_N-1(...)` 之间缺 `,` 时
   tuple 元素粘连，预处理不报错、编译才炸；
3. **模板特化必须在命名空间内**——`TOML_REFLECT` 不能写在匿名 namespace
   或函数体内。

## 与真实项目的对应

- 本 stage 的 `include/toml_reflect/` 对应 `crates/toml/src/` 的教学核心；
  真实版还有 `toml_helper_eigen.hpp`（Eigen 矩阵字段扩展），思路一致：
  在 `parse_into` 的 `if constexpr` 链上加一个分支。
- demo 的 Robot/Team/Camera 结构即 `container_example.cpp` 的骨架。
