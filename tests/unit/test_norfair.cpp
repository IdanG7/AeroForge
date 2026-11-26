#include "aeroforge/config.hpp"
#include "aeroforge/ipc/msgpack_serializer.hpp"
#include "aeroforge/modules.hpp"
#include "aeroforge/pipeline.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using af::Detection;
using af::ipc::MessagePackSerializer;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace
{

std::vector<uint8_t> bytes_of(const msgpack::sbuffer& buffer)
{
  return {buffer.data(), buffer.data() + buffer.size()};
}

Detection make_detection(int id, int x, int y, int w, int h, float score)
{
  Detection det;
  det.id = id;
  det.bbox = cv::Rect(x, y, w, h);
  det.centroid = cv::Point2f(x + w / 2.0f, y + h / 2.0f);
  det.score = score;
  return det;
}

// A socket path nothing listens on (and, on Windows, a transport that does not exist).
const char* kNoService = "/tmp/aeroforge_test_no_such_service.sock";

} // namespace

TEST_CASE("Norfair request carries version, command, dt and detections", "[norfair]")
{
  const auto request =
      MessagePackSerializer::serialize_update_request(0.04, {make_detection(7, 10, 20, 30, 40, 0.5f)});

  const auto handle = msgpack::unpack(reinterpret_cast<const char*>(request.data()), request.size());
  std::map<std::string, msgpack::object> map;
  handle.get().convert(map);

  REQUIRE(map.at("version").as<int>() == 1);
  REQUIRE(map.at("cmd").as<std::string>() == "update");
  REQUIRE(map.at("dt").as<double>() == Approx(0.04));

  const auto dets = map.at("detections").as<std::vector<msgpack::object>>();
  REQUIRE(dets.size() == 1);
  std::map<std::string, msgpack::object> det;
  dets[0].convert(det);
  REQUIRE(det.at("id").as<int>() == 7);
  REQUIRE(det.at("bbox").as<std::vector<int>>() == std::vector<int>{10, 20, 30, 40});
  const auto centroid = det.at("centroid").as<std::vector<double>>();
  REQUIRE(centroid.size() == 2);
  REQUIRE(centroid[0] == Approx(25.0));
  REQUIRE(centroid[1] == Approx(40.0));
  REQUIRE(det.at("score").as<double>() == Approx(0.5));
}

TEST_CASE("Norfair response with tracks is parsed", "[norfair]")
{
  msgpack::sbuffer buffer;
  msgpack::packer<msgpack::sbuffer> packer(buffer);
  packer.pack_map(3);
  packer.pack("version");
  packer.pack(1);
  packer.pack("success");
  packer.pack(true);
  packer.pack("detections");
  packer.pack_array(1);
  packer.pack_map(4);
  packer.pack("id");
  packer.pack(3);
  packer.pack("bbox");
  packer.pack(std::vector<int>{1, 2, 3, 4});
  packer.pack("centroid");
  packer.pack(std::vector<double>{2.5, 4.0});
  packer.pack("score");
  packer.pack(0.75);

  std::vector<Detection> out;
  std::string error;
  REQUIRE(MessagePackSerializer::deserialize_update_response(bytes_of(buffer), out, error));
  REQUIRE(out.size() == 1);
  REQUIRE(out[0].id == 3);
  REQUIRE(out[0].bbox == cv::Rect(1, 2, 3, 4));
  REQUIRE(out[0].centroid.x == Approx(2.5));
  REQUIRE(out[0].centroid.y == Approx(4.0));
  REQUIRE(out[0].score == Approx(0.75));
}

TEST_CASE("Norfair response reporting failure returns the service error", "[norfair]")
{
  msgpack::sbuffer buffer;
  msgpack::packer<msgpack::sbuffer> packer(buffer);
  packer.pack_map(2);
  packer.pack("success");
  packer.pack(false);
  packer.pack("error");
  packer.pack("bad request");

  std::vector<Detection> out;
  std::string error;
  REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response(bytes_of(buffer), out, error));
  REQUIRE(error == "bad request");
}

TEST_CASE("Malformed Norfair responses are rejected without throwing", "[norfair]")
{
  std::vector<Detection> out;
  std::string error;

  SECTION("empty payload")
  {
    REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response({}, out, error));
    REQUIRE_FALSE(error.empty());
  }

  SECTION("truncated payload")
  {
    // Map header announcing 3 entries, then nothing
    REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response({0x83}, out, error));
    REQUIRE_FALSE(error.empty());
  }

  SECTION("not a map")
  {
    msgpack::sbuffer buffer;
    msgpack::pack(buffer, 42);
    REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response(bytes_of(buffer), out, error));
    REQUIRE_THAT(error, ContainsSubstring("not a map"));
  }

  SECTION("missing success field")
  {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> packer(buffer);
    packer.pack_map(1);
    packer.pack("version");
    packer.pack(1);
    REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response(bytes_of(buffer), out, error));
    REQUIRE_THAT(error, ContainsSubstring("success"));
  }

  SECTION("detections is not an array")
  {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> packer(buffer);
    packer.pack_map(2);
    packer.pack("success");
    packer.pack(true);
    packer.pack("detections");
    packer.pack("oops");
    REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response(bytes_of(buffer), out, error));
    REQUIRE_THAT(error, ContainsSubstring("not an array"));
  }

  SECTION("field of the wrong type")
  {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> packer(buffer);
    packer.pack_map(2);
    packer.pack("success");
    packer.pack(true);
    packer.pack("detections");
    packer.pack_array(1);
    packer.pack_map(1);
    packer.pack("id");
    packer.pack("seven");
    REQUIRE_FALSE(MessagePackSerializer::deserialize_update_response(bytes_of(buffer), out, error));
    REQUIRE_THAT(error, ContainsSubstring("parse error"));
  }
}

TEST_CASE("Norfair tracker factory validates its parameters", "[norfair]")
{
  REQUIRE_THROWS_AS(af::make_norfair_tracker("", 500, false, 10.0), std::invalid_argument);
  REQUIRE_THROWS_AS(af::make_norfair_tracker(kNoService, 0, false, 10.0), std::invalid_argument);
  REQUIRE_THROWS_AS(af::make_norfair_tracker(kNoService, 500, false, 0.0), std::invalid_argument);
}

TEST_CASE("Norfair tracker passes detections through when the service is unreachable", "[norfair]")
{
  auto tracker = af::make_norfair_tracker(kNoService, 100, false, 10.0);

  REQUIRE(tracker->update(0.033, {}).empty());

  const std::vector<Detection> in = {make_detection(1, 10, 20, 30, 40, 0.9f),
                                     make_detection(2, 50, 60, 70, 80, 0.8f)};
  const auto out = tracker->update(0.033, in);
  REQUIRE(out.size() == 2);
  REQUIRE(out[0].id == 1);
  REQUIRE(out[0].bbox == in[0].bbox);
  REQUIRE(out[1].id == 2);
  REQUIRE(out[1].bbox == in[1].bbox);
}

TEST_CASE("Pipeline accepts a norfair tracker module", "[norfair][config]")
{
  const char* yaml = R"(
pipeline:
  input:
    type: file
    path: clip.avi
    size: { width: 640, height: 480, fps: 25 }
  modules:
    - name: detector
      type: colorball
      params:
        hsv_lo: [30, 120, 80]
        hsv_hi: [90, 255, 255]
    - name: tracker
      type: norfair
      params:
        socket_path: "/tmp/aeroforge_test_no_such_service.sock"
        timeout_ms: 100
        auto_reconnect: false
)";
  const af::PipelineConfig cfg = af::parse_config(yaml, "test");
  REQUIRE_NOTHROW(af::Pipeline(cfg));
}
