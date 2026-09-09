// Copyright 2026 Fictionlab sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Minimal reproduction for a bug where setting a single output queue to
// "blocking" mode appears to hang delivery to *other* queues too - this time
// testing whether it's specifically about two queues sharing the SAME
// Node::Output (e.g. "left" and "left_compressed" both hanging off the same
// camera output), rather than two independent camera nodes.
//
// Node::Output::send() (see depthai-core src/pipeline/Node.cpp) fans out to
// every connectedInputs queue with a plain sequential loop:
//   for (auto& messageQueue : connectedInputs) { messageQueue->send(msg); }
// And Pipeline.cpp reuses a single XLink bridge (one host-side thread) per
// Node::Output*, not per queue. So if queueA (first in the list) is
// blocking=true and unconsumed, the loop can never reach queueB's send() -
// even though queueB has its own healthy, actively-draining consumer.
//
// queueA_has_consumer is toggled live from stdin (type "a" to toggle queueA's
// consumer, "q" to quit). queueB always has an active consumer.
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

static std::atomic<bool> queueAHasConsumer{false};
static std::atomic<bool> quitEvent{false};

void signalHandler(int) { quitEvent = true; }

void stdinToggleThread()
{
  std::cout << "Type 'a' to toggle queueA's consumer, 'q' to quit." << std::endl;
  char c;
  while (!quitEvent && std::cin.get(c)) {
    if (c == 'a') {
      queueAHasConsumer = !queueAHasConsumer;
      std::cout << ">>> queueA consumer = " << std::boolalpha << queueAHasConsumer << std::endl;
    } else if (c == 'q') {
      quitEvent = true;
      break;
    }
  }
}

// Mirrors OakWrapper::manage_callback().
void manage_callback(
  bool has_consumer, const std::shared_ptr<dai::MessageQueue> & queue, int & callback_id,
  const std::function<void()> & callback)
{
  const bool is_active = callback_id >= 0;

  if (has_consumer && !is_active) {
    std::cout << "Activating callback for \"" << queue->getName() << "\" queue" << std::endl;
    callback_id = queue->addCallback(callback);
    auto msgs = queue->tryGetAll();
    std::cout << "Callback for \"" << queue->getName() << "\" queue activated. Cleared "
              << msgs.size() << " existing messages." << std::endl;
  } else if (!has_consumer && is_active) {
    std::cout << "Deactivating callback for \"" << queue->getName() << "\" queue" << std::endl;
    queue->removeCallback(callback_id);
    callback_id = -1;
    std::cout << "Callback for \"" << queue->getName() << "\" queue deactivated" << std::endl;
  }
}

int main()
{
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);

  dai::Pipeline pipeline;

  // Single camera, single Node::Output.
  auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
  auto * leftOutput = monoLeft->requestOutput({640, 400});

  // TWO output queues from the SAME Node::Output - matching left/left_compressed
  // both hanging off left_rotate->out in pipeline.cpp.
  auto queueA = leftOutput->createOutputQueue(1, true);
  auto queueB = leftOutput->createOutputQueue(1, true);
  queueA->setName("queueA");
  queueB->setName("queueB");

  int callbackIdA = -1;
  int callbackIdB = -1;

  auto callbackA = [&queueA]() {
    auto frame = queueA->tryGet<dai::ImgFrame>();
    if (!frame) {
      return;
    }
    std::cout << "[A] seq=" << frame->getSequenceNum() << std::endl;
  };

  auto callbackB = [&queueB]() {
    auto frame = queueB->tryGet<dai::ImgFrame>();
    if (!frame) {
      return;
    }
    std::cout << "[B] seq=" << frame->getSequenceNum() << std::endl;
  };

  pipeline.start();

  std::thread toggleThread(stdinToggleThread);

  while (pipeline.isRunning() && !quitEvent) {
    manage_callback(queueAHasConsumer, queueA, callbackIdA, callbackA);
    manage_callback(true, queueB, callbackIdB, callbackB);  // queueB always has a consumer
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  pipeline.stop();
  toggleThread.join();
  return 0;
}
