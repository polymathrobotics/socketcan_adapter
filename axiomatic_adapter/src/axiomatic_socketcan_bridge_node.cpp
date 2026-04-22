// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>

#include "axiomatic_adapter/axiomatic_socketcan_bridge.hpp"

bool verbose(false);
bool retry_connection(false);
int max_retry_attempts(-1);
int retry_counter(0);
std::atomic<bool> shutdown_requested(false);
std::condition_variable shutdown_conditional;

// Signal handler
void signalHandler(int signal)
{
  if (signal == SIGINT || signal == SIGTERM) {
    std::cout << "Signal received, shutting down..." << std::endl;
    shutdown_requested.store(true);
    shutdown_conditional.notify_all();
  }
}

// CLI argument parser
void configureArguments(
  CLI::App & app,
  std::string & can_interface,
  std::string & ip,
  std::string & port,
  bool & verbose,
  bool & retry_connection,
  int & max_retry_attempts)
{
  app.add_option("can_interface", can_interface, "CAN interface to use (default: vcan0)")->default_val("vcan0");
  app.add_option("ip", ip, "IP address of the bridge (default: 192.168.0.34)")->default_val("192.168.0.34");
  app.add_option("port", port, "Port number of the bridge (default: 4000)")->default_val("4000");
  app.add_flag("-v,--verbose", verbose, "Enable verbose logging");
  app.add_flag("-r,--retry-connection", retry_connection, "Retry configure/activate if it fails");
  app
    .add_option("--max-retry-attempts", max_retry_attempts, "Maximum number of retry attempts (default: -1 = infinite)")
    ->default_val(-1);
}

int main(int argc, char * argv[])
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::string can_interface, ip, port;
  CLI::App app{"Axiomatic SocketCAN Bridge"};
  configureArguments(app, can_interface, ip, port, verbose, retry_connection, max_retry_attempts);
  CLI11_PARSE(app, argc, argv);

  polymath::can::AxiomaticSocketcanBridge bridge(can_interface, ip, port, verbose);

  std::cout << "Axiomatic Socketcan Bridge configuring..." << std::endl;

  while (!shutdown_requested.load() && !bridge.on_configure()) {
    std::cerr << "Configuration failed.";
    if (!retry_connection) {
      std::cerr << " Exiting..." << std::endl;
      bridge.on_shutdown();
      return 1;
    }
    if (max_retry_attempts >= 0 && retry_counter >= max_retry_attempts) {
      std::cerr << " (max retry attempts reached). Exiting." << std::endl;
      bridge.on_shutdown();
      return 1;
    }
    retry_counter += 1;
    std::cerr << " Retrying in 3 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
  if (shutdown_requested.load()) {
    std::cerr << "Shutdown requested during configuration. Exiting early." << std::endl;
    bridge.on_shutdown();
    return 0;
  }

  std::cerr << "Configuration succeeded." << std::endl;
  std::cout << "Axiomatic Socketcan Bridge activating..." << std::endl;

  while (!shutdown_requested.load() && !bridge.on_activate()) {
    std::cerr << "Activation failed.";
    if (!retry_connection) {
      std::cerr << " Exiting..." << std::endl;
      bridge.on_shutdown();
      return 1;
    }
    if (max_retry_attempts >= 0 && retry_counter >= max_retry_attempts) {
      std::cerr << " (max retry attempts reached). Exiting." << std::endl;
      bridge.on_shutdown();
      return 1;
    }
    retry_counter += 1;
    std::cerr << " Retrying in 3 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  if (shutdown_requested.load()) {
    std::cerr << "Shutdown requested during activation. Exiting early." << std::endl;
    bridge.on_shutdown();
    return 0;
  }
  std::cerr << "Activation succeeded." << std::endl;

  std::mutex shutdown_mutex;
  {
    std::unique_lock<std::mutex> lock(shutdown_mutex);
    shutdown_conditional.wait(lock, [] { return shutdown_requested.load(); });
  }

  std::cout << "Axiomatic Socketcan Bridge deactivating..." << std::endl;
  bridge.on_deactivate();
  std::cout << "Axiomatic Socketcan Bridge shutting down..." << std::endl;
  bridge.on_shutdown();

  return 0;
}
