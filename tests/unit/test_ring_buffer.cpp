#include "aeroforge/core.hpp"

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <thread>

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

TEST_CASE("SpscRingBuffer keeps order across two threads", "[core][concurrency]")
{
  // One producer, one consumer, a small buffer so that both "full" and "empty" are hit
  // many times. Every value must arrive exactly once and in order.
  constexpr int kCount = 200000;
  af::SpscRingBuffer<int, 16> buf;
  std::atomic<bool> producer_done{false};

  std::thread producer(
      [&]
      {
        for (int i = 0; i < kCount; ++i)
        {
          while (!buf.push(i))
          {
            std::this_thread::yield();
          }
        }
        producer_done.store(true, std::memory_order_release);
      });

  int expected = 0;
  bool in_order = true;
  int value = -1;
  while (expected < kCount)
  {
    if (buf.pop(value))
    {
      in_order = in_order && (value == expected);
      ++expected;
    }
    else
    {
      std::this_thread::yield();
    }
  }
  producer.join();

  REQUIRE(in_order);
  REQUIRE(producer_done.load());
  REQUIRE(buf.empty());
  REQUIRE_FALSE(buf.pop(value));
}

TEST_CASE("FramePool hands out and takes back preallocated frames", "[core]")
{
  af::FramePool pool(2, 64, 48);
  REQUIRE(pool.available() == 2);

  auto a = pool.acquire();
  auto b = pool.acquire();
  REQUIRE(a.has_value());
  REQUIRE(b.has_value());
  REQUIRE(a->img.cols == 64);
  REQUIRE(a->img.rows == 48);
  REQUIRE(a->img.type() == CV_8UC3);
  REQUIRE(pool.available() == 0);

  SECTION("An exhausted pool says so instead of allocating")
  {
    REQUIRE_FALSE(pool.acquire().has_value());
  }

  SECTION("A released frame comes back with its buffer and cleared metadata")
  {
    const unsigned char* data = a->img.data;
    a->seq = 42;
    a->timestamp_s = 1.5;
    pool.release(std::move(*a));
    REQUIRE(pool.available() == 1);
    auto again = pool.acquire();
    REQUIRE(again.has_value());
    REQUIRE(again->img.data == data);
    REQUIRE(again->seq == 0);
    REQUIRE(again->timestamp_s == 0.0);
  }
}
