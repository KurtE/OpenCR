/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include <algorithm>

#if defined(__linux__)
#include "group_sync_read.h"
#elif defined(__APPLE__)
#include "group_sync_read.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_sync_read.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#include "../../include/dynamixel_sdk/group_sync_read.h"
#endif

using namespace dynamixel;

GroupSyncRead::GroupSyncRead(PortHandler *port, PacketHandler *ph, 
      uint16_t start_address, uint16_t data_length, uint8_t max_ids)
  : port_(port),
    ph_(ph),
    last_result_(false),
    is_user_buffer_(false),
    max_ids_(max_ids),
    count_ids_(0),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

GroupSyncRead::GroupSyncRead(uint16_t start_address, uint16_t data_length, uint8_t max_ids)
  : port_(NULL),
    ph_(NULL),
    last_result_(false),
    is_user_buffer_(false),
    max_ids_(max_ids),
    count_ids_(0),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
}

GroupSyncRead::~GroupSyncRead()
{
  if (!is_user_buffer_ && param_)
  {
    delete[] param_;
    param_ = NULL;  // probably not needed
  }
}

void GroupSyncRead::init(PortHandler *port, PacketHandler *ph) 
{
  port_ = port;
  ph_ = ph;
  //Serial.printf("GroupSyncRead::Init called %x %x\n", (uint32_t)port_, (uint32_t)ph_); Serial.flush();
  clearParam();
}

bool GroupSyncRead::setBuffer(uint8_t *buffer_pointer, uint16_t buffer_size)
{
  // See if there already was a buffer 
  if (!is_user_buffer_ && param_)
  {
    delete[] param_;
  }  
  // User passed in a buffer.
  param_ = buffer_pointer;
  count_ids_ = 0; 
  if (buffer_pointer)
  {
    is_user_buffer_ = true;
    max_ids_ = buffer_size / (data_length_ + EXTRA_BYTES_PER_ITEM);  // calculate how many servos this buffer could service
    return (max_ids_ > 0);
  }
  is_user_buffer_ = false;
  return true;
}  

// Different than group write as I will keep all of the IDS at the start
// of this list, also will allocate if necessary... 
// And will fill in the ID for the caller... 
uint8_t *GroupSyncRead::findParam(uint8_t id, bool add_if_not_found)
{
  if (!param_)
  {
    //Serial.println("GroupSyncRead::findParam create buffer");
    param_ = new uint8_t[max_ids_ * (EXTRA_BYTES_PER_ITEM + data_length_ + 1)]; // ID(1) + DATA(data_length + error value)
    is_user_buffer_ = false;
    if (!param_)
      return NULL; 
    count_ids_ = 0;
  }


  uint8_t *pb = param_;
  for (uint8_t i = 0; i < count_ids_; i++)
  {
    if (*pb == id)
    {
      // Item was found, now calculate it's buffer location
      // first part of buffer is setup to store all of the ids, followed
      // by the data per item
      return (param_ + max_ids_ + i * (data_length_ + 1));    
    }
    pb++;
  }
  // Not found, lets see if there is room for new one
  if ((count_ids_ >= max_ids_) || !add_if_not_found)
    return NULL;
  
  // Have room, so save away the id and calculate the address
  param_[count_ids_] = id;
  count_ids_++; // increment our count
  return (param_ + max_ids_ + (count_ids_ - 1) * (data_length_ + 1));
}

bool GroupSyncRead::addParam(uint8_t id)
{

  // If we don't have packet handler or this is version 1 bail
  //Serial.printf("GroupSyncRead::addParam %d\n", id);
  if (!ph_ || (ph_->getProtocolVersion() == 1.0))
    return false;

  uint8_t *item_pointer = findParam(id, true);  
  if (!item_pointer)
  {
    return false;
  }

  return true;
}

void GroupSyncRead::removeParam(uint8_t id)
{
  if (!ph_ || (ph_->getProtocolVersion() == 1.0) || !param_)
    return;

  // Warning, I am not worrying about copy data parts back as
  // bet no one uses this anyway...
  uint8_t *pb = param_;
  for (uint8_t i = 0; i < count_ids_; i++)
  {
    if (*pb == id)
    {
      while (i < (count_ids_ - 1))
      {
        param_[i] = param_[i+1];
        i++;
      }
      count_ids_--; // decrement the count
    }
  }
}
void GroupSyncRead::clearParam()
{
  count_ids_ = 0;
}

int GroupSyncRead::txPacket()
{
  if (!ph_ || ph_->getProtocolVersion() == 1.0 || count_ids_ == 0)
    return COMM_NOT_AVAILABLE;

  return ph_->syncReadTx(port_, start_address_, data_length_, param_, count_ids_);
}

int GroupSyncRead::rxPacket()
{
  last_result_ = false;

  if (!ph_ || ph_->getProtocolVersion() == 1.0)
    return COMM_NOT_AVAILABLE;

  int result         = COMM_RX_FAIL;

  if (count_ids_ == 0)
    return COMM_NOT_AVAILABLE;

  uint8_t *item_buffer_pointer = param_ + max_ids_;  // points to buffer for first item
  for (int i = 0; i < count_ids_; i++)
  {
    // Not used, wonder if we should verify that we have the right one...
    //uint8_t id = param_[i];   // Again id list is the first items in our data storage

    result = ph_->readRx(port_, id, data_length_, item_buffer_pointer, item_buffer_pointer+data_length_);
    if (result != COMM_SUCCESS)
      return result;
    item_buffer_pointer += data_length_ + 1;  // point to storage for next item Also save room for error
  }

  if (result == COMM_SUCCESS)
    last_result_ = true;

  return result;
}

int GroupSyncRead::txRxPacket()
{
  //Serial.printf("GroupSyncRead::txRxPacket count: %d\n", count_ids_);
  if (!ph_ || ph_->getProtocolVersion() == 1.0)
    return COMM_NOT_AVAILABLE;

  int result         = COMM_TX_FAIL;

  result = txPacket();
  if (result != COMM_SUCCESS)
    return result;

  return rxPacket();
}

bool GroupSyncRead::isAvailable(uint8_t id, uint16_t address, uint16_t data_length)
{
  //Serial.printf("\nGroupSyncRead(%u %d)::isAvailable %u %u %u ", start_address_, data_length_, id, address, data_length);
  if (!ph_ || ph_->getProtocolVersion() == 1.0 || last_result_ == false || !findParam(id, false))
    return false;

  if (address < start_address_ || start_address_ + data_length_ - data_length < address)
    return false;

  return true;
}

uint32_t GroupSyncRead::getData(uint8_t id, uint16_t address, uint16_t data_length)
{
  //Serial.printf("\nGroupSyncRead(%u %d)::getData %u %u %u ", start_address_, data_length_, id, address, data_length);
  if (isAvailable(id, address, data_length) == false)
    return 0;

  uint8_t *item_buffer_pointer = findParam(id, false); // available alredy verified it existed...
  if (!item_buffer_pointer) 
    return 0;

  item_buffer_pointer += (address - start_address_);  // increment up to start of actual field...
  switch(data_length)
  {
    case 1:
      return item_buffer_pointer[0];

    case 2:
      return DXL_MAKEWORD(item_buffer_pointer[0], item_buffer_pointer[1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(item_buffer_pointer[0], item_buffer_pointer[1]),
                DXL_MAKEWORD(item_buffer_pointer[2], item_buffer_pointer[3]));

    default:
      return 0;
  }
}

bool GroupSyncRead::getError(uint8_t id, uint8_t* error)
{
  if (!ph_ || ph_->getProtocolVersion() == 1.0 || last_result_ == false ) 
    return false;

  uint8_t *item_buffer_pointer = findParam(id, false); // available alredy verified it existed...
  if (!item_buffer_pointer)
    return false;     // id not in list

  error[0] = item_buffer_pointer[data_length_]; // stored at end of each item

  if (error[0] != 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}