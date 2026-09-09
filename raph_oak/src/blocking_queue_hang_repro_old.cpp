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
  auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

  auto * leftOutput = monoLeft->requestOutput({640, 400});
  auto * rightOutput = monoRight->requestOutput({640, 400});

  auto leftQueue = leftOutput->createOutputQueue(1, true);    // blocking
  auto rightQueue = rightOutput->createOutputQueue(1, true);  // blocking

  leftQueue->addCallback([&leftQueue]() {
    std::shared_ptr<dai::ADatatype> data = leftQueue->tryGet();
    if (!data) return;
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::cout << "[left]  seq=" << (frame ? frame->getSequenceNum() : -1) << std::endl;
  });

  rightQueue->addCallback([&rightQueue]() {
    std::shared_ptr<dai::ADatatype> data = rightQueue->tryGet();
    if (!data) return;
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::cout << "[right] seq=" << (frame ? frame->getSequenceNum() : -1) << std::endl;
  });

  pipeline.start();

  while (pipeline.isRunning() && !quitEvent) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  pipeline.stop();
  return 0;
}
