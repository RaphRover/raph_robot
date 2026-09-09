#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

static std::atomic<bool> quitEvent{false};

void signalHandler(int) { quitEvent = true; }

int main()
{
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);

  dai::Pipeline pipeline;

  auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
  auto * leftOutput = monoLeft->requestOutput({640, 400});

  auto blockedQueue = leftOutput->createOutputQueue(1, true);
  auto healthyQueue = leftOutput->createOutputQueue(1, true);

  healthyQueue->addCallback([&healthyQueue]() {
    auto frame = healthyQueue->tryGet<dai::ImgFrame>();
    if (!frame) {
      return;
    }
    std::cout << "[healthy] seq=" << frame->getSequenceNum() << std::endl;
  });

  pipeline.start();

  while (pipeline.isRunning() && !quitEvent) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  pipeline.stop();
  return 0;
}
