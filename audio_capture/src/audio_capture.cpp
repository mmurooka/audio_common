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
 * audio_capture.cpp
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <boost/thread.hpp>
#include <iterator>
#include <sstream>
#include <audio_common_msgs/AudioData.h>
#include <ros/ros.h>
#include <glibmm/main.h>
#include <gstreamermm-1.0/gstreamermm.h>
#include <gstreamermm-1.0/gstreamermm/audiotestsrc.h>
#include <gstreamermm-1.0/gstreamermm/alsasrc.h>
#include <gstreamermm-1.0/gstreamermm/appsink.h>


namespace audio_capture
{
class AudioCapture
{
 public:
  AudioCapture() : nh_(""), pnh_("~")
  {
    std::string device = pnh_.param<std::string>("device", "default");
    std::string format = pnh_.param<std::string>("format", "mp3");
    int channels = pnh_.param("channels", 1);
    bool test_mode = pnh_.param("test_mode", false);

    // params for mp3
    int bitrate = pnh_.param("bitrate", 192);

    // params for wav
    int bitdepth = pnh_.param("bitdepth", 16);
    int samplerate = pnh_.param("samplerate", 16000);

    pub_msg_.reset(new audio_common_msgs::AudioData);
    pub_audio_ = nh_.advertise<audio_common_msgs::AudioData>("audio", 10);


    // initialize gstreamer
    Glib::RefPtr<Gst::Element> src;
    pipeline_ = Gst::Pipeline::create();
    pipeline_->get_bus()->add_watch(sigc::mem_fun(*this, &AudioCapture::onBusMessage));

    if (test_mode)
    {
      Glib::RefPtr<Gst::AudioTestSrc> test_src = Gst::AudioTestSrc::create("source");
      test_src->property_is_live() = true;
      pipeline_->add(test_src);
      src = Glib::RefPtr<Gst::Element>::cast_dynamic(test_src);
    }
    else
    {
      Glib::RefPtr<Gst::AlsaSrc> alsa_src = Gst::AlsaSrc::create("source");
      alsa_src->property_device() = device;
      pipeline_->add(alsa_src);
      src = Glib::RefPtr<Gst::Element>::cast_dynamic(alsa_src);
    }

    Glib::RefPtr<Gst::AudioConvert> convert = Gst::AudioConvert::create("convert");
    pipeline_->add(convert);
    src->link(convert);

    Glib::RefPtr<Gst::AppSink> app_sink = Gst::AppSink::create("sink");
    app_sink->property_max_buffers() = 100;
    app_sink->property_emit_signals() = true;
    app_sink->property_drop() = true;
    if (format == "wav" || format == "wave")
    {
      std::ostringstream caps;
      caps << "audio/x-raw,channels=" << channels;
      caps << ",format=S" << bitdepth << (bitdepth > 8 ? "LE" : "");
      caps << ",rate=" << samplerate;
      app_sink->property_caps() = Gst::Caps::create_from_string(caps.str());
    }
    app_sink->signal_new_sample().connect(sigc::mem_fun(*this, &AudioCapture::onNewSample));
    pipeline_->add(app_sink);

    if (format == "mp3")
    {
      Glib::RefPtr<Gst::Element> lamemp3enc = Gst::ElementFactory::create_element("lamemp3enc");
      lamemp3enc->property("target", 1); // bitrate
      lamemp3enc->property("bitrate", bitrate);
      pipeline_->add(lamemp3enc);
      convert->link(lamemp3enc);
      lamemp3enc->link(app_sink);
    }
    else
    {
      convert->link(app_sink);
    }
    app_sink_ = app_sink;

    gst_thread_ = boost::thread(boost::bind(&AudioCapture::start, this));
  }

  ~AudioCapture()
  {
    if (mainloop_)
    {
      mainloop_->quit();
    }
    gst_thread_.join();
  }

  void start()
  {
    pipeline_->set_state(Gst::STATE_PLAYING);
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

  Gst::FlowReturn onNewSample()
  {
    Glib::RefPtr<Gst::Sample> sample = app_sink_->pull_sample();
    Glib::RefPtr<Gst::Buffer> buffer = sample->get_buffer();
    if (buffer)
    {
      Glib::RefPtr<Gst::MapInfo> map(new Gst::MapInfo);
      if (buffer->map(map, Gst::MAP_READ))
      {
        pub_msg_->data = std::vector<uint8_t>(map->get_data(), map->get_data() + map->get_size());
        buffer->unmap(map);
        pub_audio_.publish(*pub_msg_);
      }
    }
    return Gst::FLOW_OK;
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_audio_;
  boost::shared_ptr<audio_common_msgs::AudioData> pub_msg_;

  boost::thread gst_thread_;
  Glib::RefPtr<Glib::MainLoop> mainloop_;
  Glib::RefPtr<Gst::Pipeline> pipeline_;
  Glib::RefPtr<Gst::AppSink> app_sink_;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "audio_capture");
  Gst::init(argc, argv);
  audio_capture::AudioCapture cap;
  ros::spin();
  return 0;
}
