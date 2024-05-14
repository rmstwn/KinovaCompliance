/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2021 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

#ifndef KORTEX_EXAMPLES_UTILITIES_H
#define KORTEX_EXAMPLES_UTILITIES_H

#include <string>
#include <memory>
#include <fstream>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <cxxopts.hpp>

// #define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) *  3.14159265358979323846 / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 /  3.14159265358979323846
#define ACTUATOR_COUNT 7

namespace k_api = Kinova::Api;
namespace sc = std::chrono;

struct ExampleArgs
{
    std::string ip_address;
    std::string username;
    std::string password;
};

ExampleArgs ParseExampleArguments(int argc, char *argv[]);

bool waitMicroSeconds(const sc::time_point<sc::steady_clock> &pStartTime, const sc::microseconds &pDuration);


#endif // KORTEX_EXAMPLES_UTILITIES_H