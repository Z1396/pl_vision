// stage8_toml_reflection：toml 模块（反射式解析 + 配置合并）教学 demo。
//
// 以项目的 container_example.cpp 为骨架：一个 Robot 结构演示全部六种能力 ——
//   普通默认值 / required 必填 / optional 可选 / flatten 扁平化 / vector 容器 / 嵌套子表。
// 改造成 4 个 test，断言失败 -> 非零退出码。

#include <cstdio>
#include <string>
#include <vector>

#include "toml_reflect/merge.hpp"
#include "toml_reflect/reflect.hpp"

// ---------------------------------------------------------------------------
// 业务结构体：定义 struct 即可解析，无需手写任何解析代码
// （TOML_REFLECT 是模板特化，必须放在全局作用域）
// ---------------------------------------------------------------------------

struct Intrinsics {
  double fx{};  // 普通字段：缺省保留默认值 0.0
  double fy{};
  double cx{};
  double cy{};
};
TOML_REFLECT(Intrinsics, fx, fy, cx, cy);

struct Camera {
  rm::toml_reflect::required<std::string> name;  // 必填：缺失即失败
  rm::toml_reflect::flatten<Intrinsics> intrin;   // 扁平化：字段来自同一张表
};
TOML_REFLECT(Camera, name, intrin);

struct Team {
  std::string color{"blue"};          // 普通默认值
  rm::toml_reflect::required<int> id;  // 必填
  std::optional<std::string> motto;    // 可选：缺省 = nullopt
  std::vector<int> member_ids;        // 容器：TOML array
};
TOML_REFLECT(Team, color, id, motto, member_ids);

struct Robot {
  Team team;  // 嵌套 struct：子表 [team]
  Camera camera;
};
TOML_REFLECT(Robot, team, camera);

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (ok) {
    std::printf("  [PASS] %s\n", what);
  } else {
    ++g_failures;
    std::printf("  [FAIL] %s\n", what);
  }
}

// 与 container_example 同款 TOML
constexpr std::string_view kFullConfig = R"toml(
[team]
id = 7
member_ids = [1, 2, 3]
motto = "aim high"

[camera]
name = "front"
fx = 920.0
fy = 920.5
cx = 640.0
cy = 360.0
)toml";

// ---------------------------------------------------------------------------
// test 1: 全字段解析成功（复刻 container_example 的输出作对照）
// ---------------------------------------------------------------------------
void test_full_parse() {
  std::puts("[test 1] 全字段解析");

  auto res = rm::toml_reflect::parse<Robot>(kFullConfig);
  if (!res) {
    std::printf("  意外失败: %s\n", res.error().c_str());
    check(false, "解析成功");
    return;
  }
  const Robot& r = *res;

  // container_example 的对照输出
  std::printf("  Team Color: %s\n", r.team.color.c_str());
  std::printf("  Team ID: %d  Members: [", r.team.id.value);
  for (std::size_t i = 0; i < r.team.member_ids.size(); ++i) {
    std::printf("%d%s", r.team.member_ids[i], i + 1 < r.team.member_ids.size() ? ", " : "");
  }
  std::printf("]\n  Camera: %s\n", r.camera.name.value.c_str());
  std::printf("  Camera Intrinsics: fx=%f fy=%f cx=%f cy=%f\n", r.camera.intrin.value.fx,
              r.camera.intrin.value.fy, r.camera.intrin.value.cx, r.camera.intrin.value.cy);

  check(r.team.color == "blue", "普通字段缺省用默认值 blue");
  check(r.team.id.value == 7, "required id == 7");
  check(r.team.motto.has_value() && *r.team.motto == "aim high", "optional motto 显式赋值");
  check(r.team.member_ids == std::vector<int>({1, 2, 3}), "vector member_ids == [1,2,3]");
  check(r.camera.name.value == "front", "required camera.name");
  check(r.camera.intrin.value.fx == 920.0 && r.camera.intrin.value.cy == 360.0,
        "flatten：[camera] 表内直接取到 fx/cy");
}

// ---------------------------------------------------------------------------
// test 2: 删掉 required 字段 -> 解析失败，错误信息带字段名（fail-fast）
// ---------------------------------------------------------------------------
void test_missing_required() {
  std::puts("[test 2] required 缺失 fail-fast");

  // 删掉 team.id
  {
    constexpr std::string_view bad = R"toml(
[team]
member_ids = [1]

[camera]
name = "front"
)toml";
    auto res = rm::toml_reflect::parse<Robot>(bad);
    check(!res.has_value(), "缺 team.id -> 解析失败");
    if (!res) {
      std::printf("  错误信息: %s\n", res.error().c_str());
      check(res.error().find("'id'") != std::string::npos, "错误信息带字段名 id");
    }
  }

  // 删掉 camera.name（嵌套在子表里的 required）
  {
    constexpr std::string_view bad = R"toml(
[team]
id = 7

[camera]
fx = 920.0
)toml";
    auto res = rm::toml_reflect::parse<Robot>(bad);
    check(!res.has_value(), "缺 camera.name -> 解析失败");
    if (!res) {
      std::printf("  错误信息: %s\n", res.error().c_str());
      check(res.error().find("'name'") != std::string::npos, "错误信息带字段名 name");
    }
  }
}

// ---------------------------------------------------------------------------
// test 3: optional 缺省 = nullopt，与显式赋值可区分
// ---------------------------------------------------------------------------
void test_optional_semantics() {
  std::puts("[test 3] optional 语义");

  constexpr std::string_view no_motto = R"toml(
[team]
id = 7

[camera]
name = "front"
)toml";
  auto res = rm::toml_reflect::parse<Robot>(no_motto);
  check(res.has_value(), "无 motto 解析成功");
  if (res) {
    check(!res->team.motto.has_value(), "motto 缺省 = nullopt");
    check(res->team.member_ids.empty(), "member_ids 缺省 = 空 vector");
  }

  auto res2 = rm::toml_reflect::parse<Robot>(kFullConfig);
  check(res2.has_value() && res2->team.motto.has_value(), "motto 显式赋值 = 有值");
  check(res.has_value() && !res->team.motto.has_value(), "两者可区分");
}

// ---------------------------------------------------------------------------
// test 4: merge_configs —— 覆盖层同名 key 生效、未覆盖 key 保留 base
// ---------------------------------------------------------------------------
void test_merge_configs() {
  std::puts("[test 4] merge_configs 分层合并");

  const auto merged = rm::toml_reflect::merge_texts(
      R"toml(
version = 1
robot_name = "sentry"

[aim]
max_distance = 8.0
fire_rate = 5

[camera]
name = "front"
fx = 900.0
)toml",
      R"toml(
robot_name = "hero-override"

[aim]
max_distance = 12.0

[extra]
new_key = true
)toml");

  // 合并语义验证：直接检查合并后表的内容
  std::printf("  version     = %lld\n", *merged.get("version")->value<long long>());
  std::printf("  robot_name  = %s\n", merged.get("robot_name")->value<std::string>()->c_str());
  std::printf("  aim.max_distance = %f\n",
              merged.get("aim")->as_table()->get("max_distance")->value<double>().value());
  std::printf("  aim.fire_rate    = %lld\n",
              merged.get("aim")->as_table()->get("fire_rate")->value<long long>().value());
  std::printf("  camera.name = %s\n",
              merged.get("camera")->as_table()->get("name")->value<std::string>()->c_str());
  std::printf("  extra.new_key = %s\n",
              merged.get("extra")->as_table()->get("new_key")->value<bool>().value() ? "true"
                                                                                     : "false");

  check(merged.get("version")->value<long long>() == 1, "base 独有 key 保留：version=1");
  check(merged.get("robot_name")->value<std::string>() == "hero-override",
        "overlay 同名标量覆盖");
  check(merged.get("aim")->as_table()->get("max_distance")->value<double>() == 12.0,
        "子表内同名 key 覆盖");
  check(merged.get("aim")->as_table()->get("fire_rate")->value<long long>() == 5,
        "子表内未覆盖 key 保留");
  check(merged.get("camera")->as_table()->get("name")->value<std::string>() == "front",
        "整表未覆盖的子表保留");
  check(merged.get("extra")->as_table()->get("new_key")->value<bool>() == true,
        "overlay 新增子表生效");
}

}  // namespace

int main() {
  std::puts("=== stage8_toml_reflection ===");
  test_full_parse();
  test_missing_required();
  test_optional_semantics();
  test_merge_configs();
  std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  return g_failures == 0 ? 0 : 1;
}
