// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * audio_play.cpp
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <iterator>
#include <sstream>
#include <audio_common_msgs/AudioData.h>
#include <ros/ros.h>
#include <glibmm/main.h>
#include <gstreamermm-1.0/gstreamermm.h>
#include <gstreamermm-1.0/gstreamermm/audiotestsrc.h>
#include <gstreamermm-1.0/gstreamermm/alsasink.h>
#include <gstreamermm-1.0/gstreamermm/appsrc.h>


namespace audio_play
{
class AudioPlay
{
 public:
  AudioPlay() : nh_(""), pnh_("~")
  {
    bool test_mode = pnh_.param("test_mode", false);
    bool play_sound = pnh_.param("play_sound", true);
    bool save_sound = pnh_.param("save_sound", false);

    std::string device = pnh_.param<std::string>("device", "default");
    std::string file_path = pnh_.param<std::string>("file_path", "sound.mp3");

    pipeline_ = Gst::Pipeline::create();
    pipeline_->get_bus()->add_watch(sigc::mem_fun(*this, &AudioPlay::onBusMessage));

    Glib::RefPtr<Gst::Element> src;
    if (test_mode)
    {
      Glib::RefPtr<Gst::AudioTestSrc> test_src = Gst::AudioTestSrc::create("source");
      test_src->set_live(true);
      pipeline_->add(test_src);
      src = Glib::RefPtr<Gst::Element>::cast_dynamic(test_src);
    }
    else
    {
      Glib::RefPtr<Gst::AppSrc> app_src = Gst::AppSrc::create("source");
      app_src->set_live(true);
      pipeline_->add(app_src);
      app_src_ = app_src;
      src = Glib::RefPtr<Gst::Element>::cast_dynamic(app_src);
    }

    Glib::RefPtr<Gst::DecodeBin> decode = Gst::DecodeBin::create("decode");
    pipeline_->add(decode);
    decode->signal_pad_added().connect(sigc::mem_fun(*this, &AudioPlay::onPadAdded));
    src->link(decode);

    Glib::RefPtr<Gst::Bin> audio = Gst::Bin::create("audio");
    {
      Glib::RefPtr<Gst::Element> convert
          = Gst::ElementFactory::create_element("audioconvert", "convert");
      audio->add(convert);
      audio->add_ghost_pad(convert, "sink", "sink");

      Glib::RefPtr<Gst::Element> tee
          = Gst::ElementFactory::create_element("tee", "tee");
      audio->add(tee);
      convert->link(tee);
      Glib::RefPtr<Gst::PadTemplate> tee_src = tee->get_pad_template("src_%u");

      if (play_sound)
      {
        Glib::RefPtr<Gst::Element> queue
            = Gst::ElementFactory::create_element("queue", "queueplay");
        audio->add(queue);
        if (tee->request_pad(tee_src)->link(queue->get_static_pad("sink")) != Gst::PAD_LINK_OK)
        {
          ROS_ERROR_STREAM("Failed to link tee -> queueplay");
          ros::shutdown();
        }

        Glib::RefPtr<Gst::Element> resample
            = Gst::ElementFactory::create_element("audioresample", "resample");
        audio->add(resample);
        queue->link(resample);

        Glib::RefPtr<Gst::AlsaSink> alsasink = Gst::AlsaSink::create("alsasink");
        alsasink->property_device() = device;
        audio->add(alsasink);
        resample->link(alsasink);
      }
      if (save_sound)
      {
        Glib::RefPtr<Gst::Element> queue
            = Gst::ElementFactory::create_element("queue", "queuesave");
        audio->add(queue);
        if (tee->request_pad(tee_src)->link(queue->get_static_pad("sink")) != Gst::PAD_LINK_OK)
        {
          ROS_ERROR_STREAM("Failed to link tee -> queuesave");
          ros::shutdown();
        }

        Glib::RefPtr<Gst::FileSink> filesink = Gst::FileSink::create("filesink");
        filesink->property_location() = file_path;
        audio->add(filesink);

        if (hasExtension(file_path, "mp3"))
        {
          int bitrate = pnh_.param("file_bitrate", 192);
          Glib::RefPtr<Gst::Element> lamemp3enc
              = Gst::ElementFactory::create_element("lamemp3enc", "lamemp3enc");
          lamemp3enc->property("target", 1); // bitrate
          lamemp3enc->property("bitrate", bitrate);
          audio->add(lamemp3enc);
          queue->link(lamemp3enc)->link(filesink);
        }
        else if (hasExtension(file_path, "wav"))
        {
          Glib::RefPtr<Gst::Element> wavenc
              = Gst::ElementFactory::create_element("wavenc", "wavenc");
          audio->add(wavenc);
          queue->link(wavenc)->link(filesink);
        }
        else
        {
          queue->link(filesink);
        }
      }
      if (!play_sound && !save_sound)
      {
        Glib::RefPtr<Gst::Element> queue
            = Gst::ElementFactory::create_element("queue", "queuefake");
        audio->add(queue);
        if (tee->request_pad(tee_src)->link(queue->get_static_pad("sink")) != Gst::PAD_LINK_OK)
        {
          ROS_ERROR_STREAM("Failed to link tee -> queuefake");
          ros::shutdown();
        }

        Glib::RefPtr<Gst::Element> fakesink
            = Gst::ElementFactory::create_element("fakesink", "fakesink");
        audio->add(fakesink);
        queue->link(fakesink);
      }
    }
    pipeline_->add(audio);

    gst_thread_ = boost::thread(boost::bind(&AudioPlay::start, this));
    sub_audio_ = nh_.subscribe("audio", 10, &AudioPlay::onAudio, this);
 }

  ~AudioPlay()
  {
    if (mainloop_)
    {
      mainloop_->quit();
    }
    gst_thread_.join();
  }

  void start()
  {
    mainloop_ = Glib::MainLoop::create();
    mainloop_->run();
    pipeline_->set_state(Gst::STATE_NULL);
  }

 protected:

  bool onBusMessage(const Glib::RefPtr<Gst::Bus>& bus, const Glib::RefPtr<Gst::Message>& msg)
  {
    switch (msg->get_message_type())
    {
      case Gst::MESSAGE_EOS:
        ros::shutdown();
        break;
      case Gst::MESSAGE_ERROR:
        {
          Glib::Error err = Glib::RefPtr<Gst::MessageError>::cast_static(msg)->parse();
          ROS_ERROR_STREAM("Failed in Gstreamer: " << err.what());
          ros::shutdown();
          break;
        }
      default:
        break;
    }
    return true;
  }

  void onPadAdded(const Glib::RefPtr<Gst::Pad>& new_pad)
  {
    ROS_DEBUG_STREAM("Detected new pad " << new_pad->get_name());
    Glib::RefPtr<Gst::Element> e = pipeline_->get_element("audio");
    if (new_pad->link(e->get_static_pad("sink")) != Gst::PAD_LINK_OK)
    {
      ROS_ERROR_STREAM("Failed to link pad.");
      ros::shutdown();
    }
    else
    {
      e->set_state(Gst::STATE_PLAYING);
    }
  }

  void onAudio(const audio_common_msgs::AudioData::ConstPtr& msg)
  {
    if (paused_)
    {
      pipeline_->set_state(Gst::STATE_PLAYING);
      paused_ = false;
    }

    buffer_ = Gst::Buffer::create(msg->data.size())->create_writable();
    Glib::RefPtr<Gst::MapInfo> map(new Gst::MapInfo());
    if (buffer_->map(map, Gst::MAP_WRITE))
    {
      std::copy(msg->data.begin(), msg->data.end(), map->get_data());
      buffer_->unmap(map);
      if (app_src_)
      {
        Gst::FlowReturn ret = app_src_->push_buffer(buffer_);
        if (ret != Gst::FLOW_OK)
        {
          ROS_ERROR_STREAM("Failed to push buffer");
          app_src_->end_of_stream();
        }
      }
      else
      {
        ROS_ERROR_STREAM("AppSrc is dead.");
        ros::shutdown();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Buffer not created");
      ros::shutdown();
    }
  }

  bool hasExtension(const std::string &str, const std::string &ext)
  {
    if (str.size() < ext.size() + 1) return false;
    std::list<std::string> lst;
    boost::split(lst, str, boost::is_any_of("."));
    return lst.back() == ext;
  }

  bool paused_;
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_audio_;
  boost::thread gst_thread_;
  Glib::RefPtr<Glib::MainLoop> mainloop_;
  Glib::RefPtr<Gst::Pipeline> pipeline_;
  Glib::RefPtr<Gst::AppSrc> app_src_;
  Glib::RefPtr<Gst::Buffer> buffer_;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "audio_play");
  Gst::init(argc, argv);
  audio_play::AudioPlay play;
  ros::spin();
  return 0;
}
