// Copyright 2021 Factor Robotics
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

#pragma once

#include <iostream>
#include <cstring>
#include <vector>
#include <libusb-1.0/libusb.h>

#include "odrive_hardware_interface/odrive_endpoints.hpp"

#define ODRIVE_USB_VENDORID 0x1209
#define ODRIVE_USB_PRODUCTID 0x0d32

#define ODRIVE_OUT_ENDPOINT 0x03
#define ODRIVE_IN_ENDPOINT 0x83

#define ODRIVE_PROTOCOL_VERSION 1
#define ODRIVE_MAX_PACKET_SIZE 16

typedef std::vector<uint8_t> bytes;

namespace odrive
{
class ODriveUSB
{
public:
  libusb_device_handle* odrive_handle_;

  ODriveUSB();
  ~ODriveUSB();

  int init();

  template <typename T>
  int read(libusb_device_handle* odrive_handle, short endpoint_id, T& value);
  template <typename T>
  int write(libusb_device_handle* odrive_handle, short endpoint_id, const T& value);
  int call(libusb_device_handle* odrive_handle, short endpoint_id);

private:
  libusb_context* libusb_context_;

  short sequence_number_;

  int endpointOperation(libusb_device_handle* odrive_handle, short endpoint_id, short response_size,
                        bytes request_payload, bytes& response_payload, bool MSB);

  bytes encodePacket(short sequence_number, short endpoint_id, short response_size, const bytes& request_payload);
  bytes decodePacket(bytes& response_packet);
};
}  // namespace odrive
