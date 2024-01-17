// Copyright 2023 Simon Sagmeister
#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ros2_parser.h"
#include "tum_msgs/msg/tum_debug_signal_names.hpp"
#include "tum_msgs/msg/tum_debug_values.hpp"
class DebugConfigStorage
{
public:
  static DebugConfigStorage & getInstance()
  {
    static DebugConfigStorage instance;
    return instance;
  }
  bool has_definition(std::string debug_topic) { return debug_definitions_.count(debug_topic) > 0; }
  void register_definition(std::string debug_topic, std::vector<std::string> debug_signal_names)
  {
    debug_definitions_[debug_topic] = debug_signal_names;
  }
  void reset_definition(std::string debug_topic)
  {
    if (has_definition(debug_topic)) {
      debug_definitions_.erase(debug_topic);
    }
  }
  std::vector<std::string> get_definition(std::string debug_topic)
  {
    return debug_definitions_.at(debug_topic);
  }

private:
  std::map<std::string, std::vector<std::string>> debug_definitions_;
  DebugConfigStorage() {}
  DebugConfigStorage(DebugConfigStorage const &);
  void operator=(DebugConfigStorage const &);
};
class TUMDebugValuesMsgParser : public BuiltinMessageParser<tum_msgs::msg::TUMDebugValues>
{
public:
  TUMDebugValuesMsgParser(const std::string & topic_name, PJ::PlotDataMapRef & plot_data)
  : BuiltinMessageParser<tum_msgs::msg::TUMDebugValues>(topic_name, plot_data)
  {
    debug_channel_name_ = topic_name;
    debug_channel_name_.erase(debug_channel_name_.find("/values"));
    debug_config_storage_.reset_definition(debug_channel_name_);
  }
  void parseMessageImpl(const tum_msgs::msg::TUMDebugValues & msg, double & timestamp) override
  {
    if (!debug_config_storage_.has_definition(debug_channel_name_)) {
      // Add signals into buffer
      msg_buffer_.push_back({timestamp, msg});
      return;
    }
    if (!_initialized) {
      _initialized = true;
      std::cout << "Parsing TUM Debug Signals: " << debug_channel_name_ << std::endl;
      for (auto & signal_name : debug_config_storage_.get_definition(debug_channel_name_)) {
        // Create the individual time series objects
        _data.push_back(&getSeries(debug_channel_name_ + "/" + signal_name));
      }
      // Process all buffered messages
      for (auto & buffered_msg : msg_buffer_) {
        for (std::size_t i = 0; i < _data.size(); i++) {
          _data[i]->pushBack({std::get<0>(buffered_msg), std::get<1>(buffered_msg).values[i]});
        }
      }
      // Clear the buffer
      msg_buffer_.clear();
    }
    // Split the message in the individual time series
    for (std::size_t i = 0; i < _data.size(); i++) {
      _data[i]->pushBack({timestamp, msg.values[i]});
    }
  }

private:
  std::string debug_channel_name_;
  DebugConfigStorage & debug_config_storage_ = DebugConfigStorage::getInstance();
  bool _initialized = false;
  std::vector<PJ::PlotData *> _data;
  std::vector<std::pair<double, tum_msgs::msg::TUMDebugValues>> msg_buffer_;
};
class TUMDebugSignalNamesMsgParser : public BuiltinMessageParser<tum_msgs::msg::TUMDebugSignalNames>
{
public:
  TUMDebugSignalNamesMsgParser(const std::string & topic_name, PJ::PlotDataMapRef & plot_data)
  : BuiltinMessageParser<tum_msgs::msg::TUMDebugSignalNames>(topic_name, plot_data)
  {
    debug_channel_name_ = topic_name;
    debug_channel_name_.erase(debug_channel_name_.find("/signal_names"));
    debug_config_storage_.reset_definition(debug_channel_name_);
  }
  void parseMessageImpl(const tum_msgs::msg::TUMDebugSignalNames & msg, double & timestamp) override
  {
    if (!_initialized) {
      _initialized = true;
      debug_config_storage_.register_definition(debug_channel_name_, msg.names);
    }
  }

private:
  std::string debug_channel_name_;
  DebugConfigStorage & debug_config_storage_ = DebugConfigStorage::getInstance();
  bool _initialized = false;
};
