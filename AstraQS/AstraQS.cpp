// AstraQS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <cstdio>
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

		std::printf("Depth frameIndex: %d value: %d \n", frameIndex, middleValue);
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

int main(int argc, char** argv)
{
	astra::initialize();

	astra::StreamSet streamSet;
	astra::StreamReader reader = streamSet.create_reader();

	reader.stream<astra::DepthStream>().start();

	DepthFrameListener listener(500);
	reader.add_listener(listener);

	do {
		astra_update();
	} while (!listener.is_finished());

	reader.remove_listener(listener);
	astra::terminate();

	std::cout << "hit enter to exit program" << std::endl;
	std::cin.get();

	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
