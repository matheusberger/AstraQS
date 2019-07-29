// AstraQS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <cstdio>
#include <ctime>
#include <string>
#include <astra/astra.hpp>

class DepthFrameListener : public astra::FrameListener
{
public:
	DepthFrameListener(int maxFramesToProcess)
		: maxFramesToProcess_(maxFramesToProcess)
	{}

	bool is_finished() const { return isFinished_; }

private:
	void on_frame_ready(astra::StreamReader& reader,
		astra::Frame& frame) override
	{
		const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

		if (depthFrame.is_valid())
		{
			print_depth_frame(depthFrame);
			++framesProcessed_;
		}

		isFinished_ = framesProcessed_ >= maxFramesToProcess_;
	}

	void print_depth_frame(const astra::DepthFrame& depthFrame) const
	{
		const int frameIndex = depthFrame.frame_index();
		const short middleValue = get_middle_value(depthFrame);

		//std::printf("Depth frameIndex: %d value: %d \n", frameIndex, middleValue);
	}

	short get_middle_value(const astra::DepthFrame& depthFrame) const
	{
		const int width = depthFrame.width();
		const int height = depthFrame.height();

		const size_t middleIndex = ((width * (height / 2.f)) + (width / 2.f));

		const short* frameData = depthFrame.data();
		const short middleValue = frameData[middleIndex];

		return middleValue;
	}

	bool isFinished_{ false };
	int framesProcessed_{ 0 };
	int maxFramesToProcess_{ 0 };
};

class BodyFrameListener : public astra::FrameListener
{
public:
	void check_fps()
	{
		double fpsFactor = 0.02;

		std::clock_t newTimepoint = std::clock();
		long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);

		frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
		lastTimepoint_ = newTimepoint;
		double fps = 1.0 / frameDuration_;

		printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
	}

	void processBodies(astra::Frame& frame)
	{
		astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

		if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
		{
			std::cout << "body frame is not valid!" << std::endl;
			return;
		}

		const float jointScale = bodyFrame.info().width() / 120.f;

		const auto& bodies = bodyFrame.bodies();

		for (auto& body : bodies)
		{
			update_body(body, jointScale);
		}
	}

	void update_body(astra::Body body,
		const float jointScale)
	{
		const auto& joints = body.joints();

		if (joints.empty())
		{
			return;
		}

		joints_ = joints;
	}

	void printBody()
	{
		if (!joints_.empty())
		{
			std::cout << "Currently tracking " << joints_.size() << " joints!!!" << std::endl << "joints tracked are: " << std::endl << std::endl;
			for (auto& joint : joints_)
			{
				std::string type;
				switch (joint.type())
				{
				case astra::JointType::Head:
					type = "Head";
					break;
				case astra::JointType::Neck:
					type = "Neck";
					break;
				case astra::JointType::RightShoulder:
					type = "RightShoulder";
					break;
				case astra::JointType::LeftShoulder:
					type = "LeftShoulder";
					break;
				case astra::JointType::ShoulderSpine:
					type = "ShoulderSpine";
					break;
				case astra::JointType::MidSpine:
					type = "MidSpine";
					break;
				default:
					type = "-----";
					break;
				}

				std::cout  << type << std::endl;
			}
			std::cout << std::endl << "===================================================" << std::endl << std::endl;
		}
	}

	void on_frame_ready(astra::StreamReader& reader,
		astra::Frame& frame) override
	{
		processBodies(frame);
		printBody();
		//check_fps();
	}

private:
	long double frameDuration_{ 0 };
	std::clock_t lastTimepoint_{ 0 };

	astra::JointList joints_;
};

int main(int argc, char** argv)
{
	astra::initialize();

	astra::StreamSet streamSet;
	astra::StreamReader reader = streamSet.create_reader();

	reader.stream<astra::BodyStream>().start();
	reader.stream<astra::DepthStream>().start();

	DepthFrameListener listener(300);
	reader.add_listener(listener);

	BodyFrameListener bodyTracker;
	reader.add_listener(bodyTracker);
	reader.stream<astra::BodyStream>().set_skeleton_profile(astra::SkeletonProfile::Full);

	do {
		astra_update();
	} while (!listener.is_finished());

	reader.remove_listener(listener);
	astra::terminate();

	return 0;
}