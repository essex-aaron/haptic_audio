//tried to change ros props
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
//#include "rdda_interface/rdda_interface.h"

#include <soundio/soundio.h>
#include "beginner_tutorials/RDDAPacket.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chrono>
#include <ctime>



//Start of sound code
//const float PI = 3.1415926535f;
static float seconds_offset = 0.0f;
float pitch =0.0f; //create filter
float pitch2 =0.0f;
static void write_callback(struct SoundIoOutStream *outstream,
        int frame_count_min, int frame_count_max)

{
    const struct SoundIoChannelLayout *layout = &outstream->layout;
    float float_sample_rate = outstream->sample_rate;
    float seconds_per_frame = 1.0f / float_sample_rate;
    struct SoundIoChannelArea *areas;
    int frames_left = frame_count_max;
    int err;
    ROS_INFO("ENDING!");
    while (frames_left > 0) {
        int frame_count = frames_left;

        if ((err = soundio_outstream_begin_write(outstream, &areas, &frame_count))) {
            fprintf(stderr, "%s\n", soundio_strerror(err));
            exit(1);
        }

        if (!frame_count)
            break;


        for (int frame = 0; frame < frame_count; frame += 1) {
            float sample = pitch; //sinf((seconds_offset + frame * seconds_per_frame) * radians_per_second);
           // float sample2 = pitch2;

            for (int channel = 0; channel < layout->channel_count; channel += 1) {
                float *ptr = (float*)(areas[channel].ptr + areas[channel].step * frame);
                *ptr = sample;

            }
        }
        for (int frame = 1; frame < frame_count - 1; frame += 1) {
            float sample2 = pitch2;

            for (int channel = 1; channel < layout->channel_count - 1; channel += 1) {
                float *ptr = (float*)(areas[channel].ptr + areas[channel].step * frame);
                *ptr = sample2;
            }
        }


        seconds_offset = fmodf(seconds_offset +
            seconds_per_frame * frame_count, 1.0f);

        if ((err = soundio_outstream_end_write(outstream))) {
            fprintf(stderr, "%s\n", soundio_strerror(err));
            exit(1);
        }

        frames_left -= frame_count;
    }
}

void packet_msgCallback(const beginner_tutorials::RDDAPacket::ConstPtr& msg)
//void packet_msgCallback(const std_msgs::String::ConstPtr& msg)
{

  auto pressure = msg->pressure[0];
  auto pressure2 = msg->pressure[1];
  //ROS_INFO("Raw Pres Val: [%f]", pressure);
  //ROS_INFO("Raw Pres2 Val: [%f]", pressure2);

//try to filter out noise
  if (pressure < pitch) {
    pitch = pressure * 1;
  }
  if (pressure2 < pitch2) {
    pitch2 = pressure2 * 1;
  }


  //ROS_INFO("Output Val: [%f]", pitch);
  //ROS_INFO("Output2 Val: [%f]", pitch2);
}

int main(int argc, char **argv)
{

    int err;
    struct SoundIo *soundio = soundio_create();
    if (!soundio) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    if ((err = soundio_connect(soundio))) {
        fprintf(stderr, "error connecting: %s", soundio_strerror(err));
        return 1;
    }

    soundio_flush_events(soundio);

    int default_out_device_index = soundio_default_output_device_index(soundio);
    if (default_out_device_index < 0) {
        fprintf(stderr, "no output device found");
        return 1;
    }

    struct SoundIoDevice *device = soundio_get_output_device(soundio, default_out_device_index);
    if (!device) {
        fprintf(stderr, "out of memory");
        return 1;
    }

    fprintf(stderr, "Output device: %s\n", device->name);

    struct SoundIoOutStream *outstream = soundio_outstream_create(device);
    outstream->format = SoundIoFormatFloat32NE;
    outstream->write_callback = write_callback;

    if ((err = soundio_outstream_open(outstream))) {
        fprintf(stderr, "unable to open device: %s", soundio_strerror(err));
        return 1;
    }

    if (outstream->layout_error)
        fprintf(stderr, "unable to set channel layout: %s\n", soundio_strerror(outstream->layout_error));

    if ((err = soundio_outstream_start(outstream))) {
        fprintf(stderr, "unable to start device: %s", soundio_strerror(err));
        return 1;
    }

        ros::init(argc, argv, "phapt");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/rdda_master_output", 1000, packet_msgCallback);
    ros::Subscriber sub = n.subscribe("/rdda_master_input", 1000, packet_msgCallback);

    //return 0;

    ros::spin();

//    for (;;)
//    soundio_wait_events(soundio);
//    soundio_outstream_destroy(outstream);
//    soundio_device_unref(device);
//    soundio_destroy(soundio);

// end of sound code

    return 0;

}