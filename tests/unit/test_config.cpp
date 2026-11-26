#include "aeroforge/config.hpp"
#include "aeroforge/pipeline.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <string>

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace
{

const char* kValid = R"(
pipeline:
  input:
    type: file
    path: clip.avi
    loop: true
    size: { width: 640, height: 480, fps: 25 }
  modules:
    - name: detector
      type: colorball
      params:
        hsv_lo: [30, 120, 80]
        hsv_hi: [90, 255, 255]
        min_area_px: 200
    - name: estimator
      type: size_based
      params: { object_diameter_m: 0.2, fx: 600, fy: 600.0, cx: 320, cy: 240 }
    - name: controller
      type: pid_velocity
      params:
        desired_offset_m: [-3, 0, 0]
        kp: [1, 1, 1]
        max_speed_mps: 3
safety:
  require_hold_to_enable: false
  stale_timeout_s: 0.25
  limits: { max_speed_mps: 4.0, min_follow_dist_m: 1.0, max_follow_dist_m: 8.0 }
  geofence: { min_alt_m: 2.0, max_alt_m: 50.0, max_radius_m: 100.0 }
)";

std::string with_input(const std::string& input_block)
{
  return "pipeline:\n  input:\n" + input_block;
}

} // namespace

TEST_CASE("Config: a valid file is parsed completely", "[config]")
{
  const af::PipelineConfig c = af::parse_config(kValid, "test");
  REQUIRE(c.input.type == "file");
  REQUIRE(c.input.path == "clip.avi");
  REQUIRE(c.input.loop);
  REQUIRE_FALSE(c.input.mirror.has_value());
  REQUIRE(c.input.width == 640);
  REQUIRE(c.input.fps == 25);
  REQUIRE(c.modules.size() == 3);
  REQUIRE(c.modules[0].type == "colorball");
  REQUIRE_FALSE(c.safety.require_hold_to_enable);
  REQUIRE(c.safety.stale_timeout_s == Approx(0.25));
  REQUIRE(c.safety.max_speed_mps == Approx(4.0));
  REQUIRE(c.safety.max_radius_m == Approx(100.0));
}

TEST_CASE("Config: integer and floating-point numbers are interchangeable", "[config]")
{
  const af::PipelineConfig c = af::parse_config(kValid, "test");
  const auto& est = c.modules[1].params;
  REQUIRE(af::get_number(est, "fx").value() == Approx(600.0)); // written as an integer
  REQUIRE(af::get_number(est, "fy").value() == Approx(600.0)); // written as a float
  REQUIRE_FALSE(af::get_number(est, "missing").has_value());

  const auto& ctl = c.modules[2].params;
  const auto kp = af::get_numbers(ctl, "kp", 3); // [1, 1, 1] parses as integers
  REQUIRE(kp.has_value());
  REQUIRE((*kp)[2] == Approx(1.0));
  REQUIRE(af::get_vec3(ctl, "desired_offset_m", Eigen::Vector3d::Zero()).x() == Approx(-3.0));
  REQUIRE_THROWS_WITH(af::get_numbers(ctl, "kp", 2), ContainsSubstring("exactly 2"));
  REQUIRE_THROWS_WITH(af::get_number(ctl, "kp"), ContainsSubstring("must be a number"));
}

TEST_CASE("Config: errors name the file and the key", "[config]")
{
  REQUIRE_THROWS_WITH(af::load_config("does/not/exist.yml"),
                      ContainsSubstring("does/not/exist.yml") && ContainsSubstring("not found"));

  REQUIRE_THROWS_WITH(af::parse_config("safety: {}", "a.yml"),
                      ContainsSubstring("a.yml") && ContainsSubstring("pipeline"));

  REQUIRE_THROWS_WITH(af::parse_config("pipeline: [unclosed", "a.yml"), ContainsSubstring("not valid YAML"));

  REQUIRE_THROWS_WITH(af::parse_config(with_input("    type: dji\n"), "a.yml"),
                      ContainsSubstring("pipeline.input.type") && ContainsSubstring("webcam"));

  REQUIRE_THROWS_WITH(af::parse_config(with_input("    type: file\n"), "a.yml"),
                      ContainsSubstring("pipeline.input.path"));

  REQUIRE_THROWS_WITH(af::parse_config(with_input("    size: { width: 0, height: 480, fps: 30 }\n"), "a.yml"),
                      ContainsSubstring("pipeline.input.size"));

  REQUIRE_THROWS_WITH(af::parse_config(with_input("    device: front\n"), "a.yml"),
                      ContainsSubstring("pipeline.input.device"));

  REQUIRE_THROWS_WITH(af::parse_config("pipeline:\n  modules:\n    - name: x\n", "a.yml"),
                      ContainsSubstring("pipeline.modules[0]") && ContainsSubstring("type"));

  REQUIRE_THROWS_WITH(af::parse_config("pipeline: {}\nsafety:\n  limits: { max_speed_mps: -1 }\n", "a.yml"),
                      ContainsSubstring("max_speed_mps"));

  REQUIRE_THROWS_WITH(af::parse_config("pipeline: {}\nsafety:\n  limits: { min_follow_dist_m: 9, "
                                       "max_follow_dist_m: 3 }\n",
                                       "a.yml"),
                      ContainsSubstring("min_follow_dist_m"));

  REQUIRE_THROWS_WITH(af::parse_config("pipeline: {}\nsafety:\n  geofence: { min_alt_m: 80 }\n", "a.yml"),
                      ContainsSubstring("min_alt_m"));
}

TEST_CASE("Config: defaults apply when sections are omitted", "[config]")
{
  const af::PipelineConfig c = af::parse_config("pipeline: {}", "test");
  REQUIRE(c.input.type == "webcam");
  REQUIRE(c.input.width == 1280);
  REQUIRE(c.modules.empty());
  REQUIRE(c.safety.require_hold_to_enable);
  REQUIRE(c.safety.stale_timeout_s == Approx(0.5));
}

TEST_CASE("Pipeline construction reports unusable modules", "[config][pipeline]")
{
  auto build = [](const std::string& modules)
  { af::Pipeline p(af::parse_config("pipeline:\n  modules:\n" + modules, "test")); };

  REQUIRE_THROWS_WITH(build("    - { name: det, type: yolo }\n"), ContainsSubstring("det") &&
                                                                      ContainsSubstring("unknown type") &&
                                                                      ContainsSubstring("colorball"));

  // The object size must be given explicitly: there is no silent default
  REQUIRE_THROWS_WITH(build("    - name: est\n      type: size_based\n"
                            "      params: { fx: 600, fy: 600, cx: 320, cy: 240 }\n"),
                      ContainsSubstring("est") && ContainsSubstring("object_diameter_m"));

  REQUIRE_THROWS_WITH(build("    - name: est\n      type: size_based\n"
                            "      params: { object_diameter_m: 0, fx: 600, fy: 600, cx: 320, cy: 240 }\n"),
                      ContainsSubstring("real size"));

  REQUIRE_THROWS_WITH(build("    - name: ctl\n      type: pid_velocity\n"
                            "      params: { kp: [1, 1], desired_offset_m: [-3, 0, 0], "
                            "max_speed_mps: 3 }\n"),
                      ContainsSubstring("ctl") && ContainsSubstring("kp"));

  REQUIRE_THROWS_WITH(build("    - { name: a, type: colorball, params: { hsv_lo: [0, 0, 0], "
                            "hsv_hi: [10, 255, 255] } }\n"
                            "    - { name: b, type: aruco }\n"),
                      ContainsSubstring("more than one detector"));

  SECTION("With no detector the interactive tracker is used")
  {
    af::Pipeline p(af::parse_config("pipeline: {}", "test"));
    REQUIRE(p.target_tracker() != nullptr);
    REQUIRE_FALSE(p.has_estimator());
    REQUIRE_FALSE(p.has_controller());
  }
}
