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

#include "odrive_hardware_interface/odrive_usb.hpp"
// #include "rclcpp/rclcpp.hpp"


namespace odrive
{
ODriveUSB::ODriveUSB()
{
  std::cout << "Connected to ODrive " << std::endl;
  libusb_context_ = NULL;
}

ODriveUSB::~ODriveUSB()
{
  
  for (auto it = odrive_map_.begin(); it != odrive_map_.end(); it++)
  {
    libusb_release_interface(it->second, 2);
    libusb_close(it->second);
  }
  odrive_map_.clear();

  if (libusb_context_)
  {
    libusb_exit(libusb_context_);
    libusb_context_ = NULL;
  }
}
//  Initial Retrieve List USB serial number.  

// Note: Can be error can't connect to devices, need to config proper with odrive 

int ODriveUSB::init(const std::vector<std::vector<int64_t>>& serial_numbers)
{
  int ret = libusb_init(&libusb_context_);
  
  if (ret != LIBUSB_SUCCESS)
  {
    //  RCLCPP_ERROR(rclcpp::get_logger()); 
    std::cout << "Connected to ODrive " << std::endl;
    return ret;
  }

  libusb_device** device_list;
  ssize_t device_count = libusb_get_device_list(libusb_context_, &device_list);
  if (device_count <= 0)
  {
    return device_count;
  }

  for (ssize_t i = 0; i < device_count; ++i)
  {
    libusb_device* device = device_list[i];
    libusb_device_descriptor descriptor;

    if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS)
    {
      continue;
    }

    if (descriptor.idVendor == ODRIVE_USB_VENDORID && descriptor.idProduct == ODRIVE_USB_PRODUCTID)
    {
      libusb_device_handle* device_handle;
      if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS)
      {
        continue;
      }
      if ((libusb_kernel_driver_active(device_handle, 2) != LIBUSB_SUCCESS) &&
          (libusb_detach_kernel_driver(device_handle, 2) != LIBUSB_SUCCESS))
      {
        libusb_close(device_handle);
        continue;
      }
      if ((libusb_claim_interface(device_handle, 2)) != LIBUSB_SUCCESS)
      {
        libusb_close(device_handle);
        continue;
      }
      uint64_t serial_number;
      if ((read(device_handle, SERIAL_NUMBER, serial_number)) != LIBUSB_SUCCESS)
      {
        libusb_release_interface(device_handle, 2);
        libusb_close(device_handle);
        continue;
      }
      odrive_map_.insert(std::pair<int64_t, libusb_device_handle*>(-serial_number, device_handle));
    }
  }

  libusb_free_device_list(device_list, 1);
  if (!odrive_map_.size())
  {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  if (odrive_map_.size() == 1)
  {
    auto it = odrive_map_.begin();
    odrive_map_.insert(std::pair<int64_t, libusb_device_handle*>(-it->first, it->second));
    std::cout << "Connected to ODrive " << std::hex << -it->first << std::endl;
    odrive_map_.erase(it);
  }
  else
  {
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < serial_numbers[0].size(); j++)
      {
        if (!odrive_map_.count(serial_numbers[i][j]))
        {
          auto it = odrive_map_.find(-serial_numbers[i][j]);
          if (it != odrive_map_.end())
          {
            odrive_map_.insert(std::pair<int64_t, libusb_device_handle*>(-it->first, it->second));
            std::cout << "Connected to ODrive " << std::hex << -it->first << std::endl;
            odrive_map_.erase(it);
          }
          else
          {
            return LIBUSB_ERROR_NO_DEVICE;
          }
        }
      }
    }
  }

  for (auto it = odrive_map_.begin(); it != odrive_map_.end(); it++)
  {
    if (it->first < 0)
    {
      libusb_release_interface(it->second, 2);
      libusb_close(it->second);
      odrive_map_.erase(it);
    }
  }

  return LIBUSB_SUCCESS;
}

template <typename T>
int ODriveUSB::read(int64_t& serial_number, short endpoint_id, T& value)
{
  if (serial_number)
  {
    return read(odrive_map_[serial_number], endpoint_id, value);
  }
  else
  {
    return read(odrive_map_.begin()->second, endpoint_id, value);
  }
}

template <typename T>
int ODriveUSB::read(libusb_device_handle* odrive_handle, short endpoint_id, T& value)
{
  bytes request_payload;
  bytes response_payload;

  int ret = endpointOperation(odrive_handle, endpoint_id, sizeof(value), request_payload, response_payload, 1);
  if (ret != LIBUSB_SUCCESS)
  {
    return ret;
  }

  std::memcpy(&value, &response_payload[0], sizeof(value));

  return LIBUSB_SUCCESS;
}

template <typename T>
int ODriveUSB::write(int64_t& serial_number, short endpoint_id, const T& value)
{
  if (serial_number)
  {
    return write(odrive_map_[serial_number], endpoint_id, value);
  }
  else
  {
    return write(odrive_map_.begin()->second, endpoint_id, value);
  }
}

template <typename T>
int ODriveUSB::write(libusb_device_handle* odrive_handle, short endpoint_id, const T& value)
{
  bytes request_payload;
  bytes response_payload;

  for (size_t i = 0; i < sizeof(value); i++)
  {
    request_payload.emplace_back(((unsigned char*)&value)[i]);
  }

  return endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, 1);
}

int ODriveUSB::call(int64_t& serial_number, short endpoint_id)
{
  if (serial_number)
  {
    return call(odrive_map_[serial_number], endpoint_id);
  }
  else
  {
    return call(odrive_map_.begin()->second, endpoint_id);
  }
}

int ODriveUSB::call(libusb_device_handle* odrive_handle, short endpoint_id)
{
  bytes request_payload;
  bytes response_payload;

  return endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, 1);
}

int ODriveUSB::endpointOperation(libusb_device_handle* odrive_handle, short endpoint_id, short response_size,
                                 bytes request_payload, bytes& response_payload, bool MSB)
{
  int transferred = 0;
  bytes response_packet;
  unsigned char response_data[ODRIVE_MAX_PACKET_SIZE] = { 0 };

  if (MSB)
  {
    endpoint_id |= 0x8000;
  }
  sequence_number_ = (sequence_number_ + 1) & 0x7fff;
  sequence_number_ |= LIBUSB_ENDPOINT_IN;
  short sequence_number = sequence_number_;

  bytes request_packet = encodePacket(sequence_number, endpoint_id, response_size, request_payload);

  int ret = libusb_bulk_transfer(odrive_handle, ODRIVE_OUT_ENDPOINT, request_packet.data(), request_packet.size(),
                                 &transferred, 0);
  if (ret != LIBUSB_SUCCESS)
  {
    return ret;
  }

  if (MSB)
  {
    ret =
        libusb_bulk_transfer(odrive_handle, ODRIVE_IN_ENDPOINT, response_data, ODRIVE_MAX_PACKET_SIZE, &transferred, 0);
    if (ret != LIBUSB_SUCCESS)
    {
      return ret;
    }

    for (int i = 0; i < transferred; i++)
    {
      response_packet.emplace_back(response_data[i]);
    }

    response_payload = decodePacket(response_packet);
  }

  return LIBUSB_SUCCESS;
}

bytes ODriveUSB::encodePacket(short sequence_number, short endpoint_id, short response_size,
                              const bytes& request_payload)
{
  bytes packet;

  packet.emplace_back((sequence_number >> 0) & 0xFF);
  packet.emplace_back((sequence_number >> 8) & 0xFF);
  packet.emplace_back((endpoint_id >> 0) & 0xFF);
  packet.emplace_back((endpoint_id >> 8) & 0xFF);
  packet.emplace_back((response_size >> 0) & 0xFF);
  packet.emplace_back((response_size >> 8) & 0xFF);

  for (uint8_t b : request_payload)
  {
    packet.emplace_back(b);
  }

  short crc = ((endpoint_id & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : json_crc;
  packet.emplace_back((crc >> 0) & 0xFF);
  packet.emplace_back((crc >> 8) & 0xFF);

  return packet;
}

bytes ODriveUSB::decodePacket(bytes& response_packet)
{
  bytes payload;

  for (bytes::size_type i = 2; i < response_packet.size(); ++i)
  {
    payload.emplace_back(response_packet[i]);
  }

  return payload;
}

template int ODriveUSB::read(int64_t&, short, bool&);
template int ODriveUSB::read(int64_t&, short, float&);
template int ODriveUSB::read(int64_t&, short, int32_t&);
template int ODriveUSB::read(int64_t&, short, int64_t&);
template int ODriveUSB::read(int64_t&, short, uint8_t&);
template int ODriveUSB::read(int64_t&, short, uint16_t&);
template int ODriveUSB::read(int64_t&, short, uint32_t&);
template int ODriveUSB::read(int64_t&, short, uint64_t&);

template int ODriveUSB::write(int64_t&, short, const bool&);
template int ODriveUSB::write(int64_t&, short, const float&);
template int ODriveUSB::write(int64_t&, short, const int32_t&);
template int ODriveUSB::write(int64_t&, short, const int64_t&);
template int ODriveUSB::write(int64_t&, short, const uint8_t&);
template int ODriveUSB::write(int64_t&, short, const uint16_t&);
template int ODriveUSB::write(int64_t&, short, const uint32_t&);
template int ODriveUSB::write(int64_t&, short, const uint64_t&);
}  // namespace odrive
