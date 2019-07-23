// AstraQS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <cstdio>
#include <astra/astra.hpp>

int main(int argc, char** argv)
{
	astra::initialize();

	astra::StreamSet streamSet;
	astra::StreamReader reader = streamSet.create_reader();

	reader.stream<astra::DepthStream>().start();

	astra::Frame frame = reader.get_latest_frame();
	const auto depthFrame = frame.get<astra::DepthFrame>();


	const int frameIndex = depthFrame.frame_index();
	const short pixelValue = depthFrame.data()[0];

	std::cout << std::endl << "Depth frameIndex: " << frameIndex << " pixelValue: " << pixelValue << std::endl << std::endl;

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
