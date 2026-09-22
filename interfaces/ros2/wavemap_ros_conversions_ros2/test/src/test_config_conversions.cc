#include <filesystem>
#include <string>

#include <gtest/gtest.h>
#include <wavemap/core/config/param.h>

#include "wavemap_ros_conversions_ros2/config_conversions.h"

namespace wavemap {
namespace {
param::Value parse(const std::string& yaml) {
  const auto params = param::convert::yamlStringToParams(yaml);
  EXPECT_TRUE(params.has_value()) << "failed to parse: " << yaml;
  return params.value_or(param::Value{param::Map{}});
}

// Reads one scalar out of `key: <scalar>` and reports which param type it
// resolved to, so expectations read as a plain truth table below.
std::string typeOf(const std::string& scalar) {
  const auto value = parse("key: " + scalar).getChild("key");
  if (!value) {
    return "missing";
  }
  if (value->as<bool>()) {
    return value->as<bool>().value() ? "bool:true" : "bool:false";
  }
  if (const auto i = value->as<int>(); i) {
    return "int:" + std::to_string(*i);
  }
  if (const auto f = value->as<FloatingPoint>(); f) {
    return "float:" + std::to_string(*f);
  }
  if (const auto s = value->as<std::string>(); s) {
    return "str:" + *s;
  }
  return "other";
}
}  // namespace

// Scalar resolution must follow PyYAML's YAML 1.1 rules, not yaml-cpp's
// native YAML 1.2 ones, because ROS1 loaded these same files via roslaunch's
// <rosparam> tag, which parses with PyYAML. Matching PyYAML is what makes a
// ROS1 user's existing config behave identically here.
//
// Every expectation below was verified against PyYAML directly rather than
// assumed: see claude/progress.md.
TEST(ConfigConversions, Yaml11BooleansMatchPyYaml) {
  EXPECT_EQ(typeOf("yes"), "bool:true");
  EXPECT_EQ(typeOf("no"), "bool:false");
  EXPECT_EQ(typeOf("on"), "bool:true");
  EXPECT_EQ(typeOf("off"), "bool:false");
  EXPECT_EQ(typeOf("true"), "bool:true");
  EXPECT_EQ(typeOf("False"), "bool:false");
  // PyYAML deliberately omits bare y/n from its bool resolver.
  EXPECT_EQ(typeOf("y"), "str:y");
  EXPECT_EQ(typeOf("n"), "str:n");
}

TEST(ConfigConversions, QuotingSuppressesCoercion) {
  EXPECT_EQ(typeOf("\"yes\""), "str:yes");
  EXPECT_EQ(typeOf("'true'"), "str:true");
  EXPECT_EQ(typeOf("\"128\""), "str:128");
  EXPECT_EQ(typeOf("\"\""), "str:");
}

TEST(ConfigConversions, NumericTypesArePreserved) {
  // int vs float must not collapse: num_cells is an int field, while
  // scaling_free is a float one.
  EXPECT_EQ(typeOf("128"), "int:128");
  EXPECT_EQ(typeOf("-7"), "int:-7");
  EXPECT_EQ(typeOf("0"), "int:0");
  EXPECT_EQ(typeOf("0x1F"), "int:31");
  EXPECT_EQ(typeOf("0.2").substr(0, 5), "float");
  EXPECT_EQ(typeOf("-180.0").substr(0, 5), "float");
  EXPECT_EQ(typeOf(".5").substr(0, 5), "float");
}

TEST(ConfigConversions, ExponentFormsMatchPyYaml) {
  // PyYAML's float pattern requires a signed exponent, so an unsigned one is
  // a string. yaml-cpp's native YAML 1.2 behaviour would disagree.
  EXPECT_EQ(typeOf("1e3"), "str:1e3");
  EXPECT_EQ(typeOf("1.0e+3").substr(0, 5), "float");
}

TEST(ConfigConversions, NestedSequencesOfMapsSurvive) {
  // The structure ROS2's own parameter system cannot represent, and the
  // reason this converter exists at all.
  const auto params = parse(
      "map_operations:\n"
      "  - type: threshold_map\n"
      "    once_every: { seconds: 2.0 }\n"
      "  - type: prune_map\n");
  const auto operations = params.getChildAs<param::Array>("map_operations");
  ASSERT_TRUE(operations.has_value());
  ASSERT_EQ(operations->size(), 2u);
  EXPECT_EQ((*operations)[0].getChildAs<std::string>("type").value_or(""),
            "threshold_map");
  // Order matters: map_operations is a pipeline, run in the listed order.
  EXPECT_EQ((*operations)[1].getChildAs<std::string>("type").value_or(""),
            "prune_map");
  const auto once_every = (*operations)[0].getChild("once_every");
  ASSERT_TRUE(once_every.has_value());
  EXPECT_TRUE(once_every->getChildAs<FloatingPoint>("seconds").has_value());
}

TEST(ConfigConversions, MissingFileIsReportedNotSilentlyEmpty) {
  EXPECT_FALSE(
      param::convert::yamlFileToParams("/nonexistent/wavemap.yaml").has_value());
}

TEST(ConfigConversions, MalformedYamlIsReportedNotSilentlyEmpty) {
  EXPECT_FALSE(param::convert::yamlStringToParams("key: [unclosed").has_value());
}

// Every config shipped for ROS1 must parse unchanged, since user-facing
// parity is the whole point: a ROS1 user points general.config_file at the
// same file they already have.
TEST(ConfigConversions, AllShippedRos1ConfigsParse) {
  const std::filesystem::path config_dir{kRos1ConfigDir};
  ASSERT_TRUE(std::filesystem::is_directory(config_dir))
      << "ROS1 config dir not found at " << config_dir;

  int configs_checked = 0;
  for (const auto& entry : std::filesystem::directory_iterator(config_dir)) {
    if (entry.path().extension() != ".yaml") {
      continue;
    }
    const auto params = param::convert::yamlFileToParams(entry.path());
    ASSERT_TRUE(params.has_value()) << "failed to parse " << entry.path();
    // Every wavemap config has these two top-level sections.
    EXPECT_TRUE(params->hasChild("general")) << entry.path();
    EXPECT_TRUE(params->hasChild("map")) << entry.path();
    ++configs_checked;
  }
  EXPECT_GT(configs_checked, 0) << "no configs were found to check";
}
}  // namespace wavemap
