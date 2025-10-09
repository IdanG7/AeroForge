#include <catch2/catch_test_macros.hpp>
#include "aeroforge/core.hpp"

TEST_CASE("SpscRingBuffer basic operations", "[core]")
{
  af::SpscRingBuffer<int, 4> buf;

  SECTION("Initially empty")
  {
    REQUIRE(buf.empty());
    REQUIRE(buf.size() == 0);
    REQUIRE(buf.capacity() == 3); // N-1 for ring buffer
  }

  SECTION("Push and pop")
  {
    REQUIRE(buf.push(10));
    REQUIRE(buf.size() == 1);
    REQUIRE_FALSE(buf.empty());

    int val;
    REQUIRE(buf.pop(val));
    REQUIRE(val == 10);
    REQUIRE(buf.empty());
  }

  SECTION("Fill to capacity")
  {
    REQUIRE(buf.push(1));
    REQUIRE(buf.push(2));
    REQUIRE(buf.push(3));
    REQUIRE(buf.size() == 3);

    // Should be full
    REQUIRE_FALSE(buf.push(4));
  }

  SECTION("Wrap-around")
  {
    buf.push(1);
    buf.push(2);

    int val;
    buf.pop(val);
    REQUIRE(val == 1);

    buf.push(3);
    buf.push(4);

    REQUIRE(buf.size() == 3);

    buf.pop(val);
    REQUIRE(val == 2);
    buf.pop(val);
    REQUIRE(val == 3);
    buf.pop(val);
    REQUIRE(val == 4);

    REQUIRE(buf.empty());
  }
}
