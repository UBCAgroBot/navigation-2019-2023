// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	//Contruct a pipeline which abstracts the device
	rs2::pipeline pipe;

	//Create a configuration for configuring the pipeline with a non default profile
    	rs2::config cfg;

	//Filter declaration
	rs2::decimation_filter dec_filter;
	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;
	rs2::disparity_transform depth_to_disparity(true);
	rs2::disparity_transform disparity_to_depth(false);

	//Filter configuration
	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    	//Add desired streams to configuration
    	cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    	//Instruct pipeline to start streaming with the requested configuration
    	pipe.start(cfg);

    	//Camera warmup - dropping several first frames to let auto-exposure stabilize
    	rs2::frameset frames;
    	for(int i = 0; i < 30; i++)
    	{
        	//Wait for all configured streams to produce a frame
        	frames = pipe.wait_for_frames();
    	}

    	while(true)
    	{
    		frames = pipe.wait_for_frames();

    		//Get each frame
    		rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
    		rs2::frame depth_frame = frames.get_depth_frame();

		// Apply filters
		rs2::frame filtered = depth_frame;
		filtered = dec_filter.process(filtered);

    		// Creating OpenCV matrix from IR image
    		Mat ir(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);		

    		// Apply Histogram Equalization
    		equalizeHist( ir, ir );
    		applyColorMap(ir, ir, COLORMAP_JET);		

    		// Display ir  image in GUI
    		namedWindow("Display Image", WINDOW_AUTOSIZE );
    		imshow("Display Image", ir);

		/*
		// Creating depth matrix
		Mat depth(Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

		Size sze = Size(45, 45);
		GaussianBlur(depth, depth, sze, 0);
		normalize(depth, depth, 65535, NORM_MINMAX);
		*/

		// Creating OpenCV matrix from 
		Mat edge(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);

		Mat gray = cvtColor(edge, COLOR_BGR2GRAY);

		Mat edges;

		Canny(gray, edges, 50, 200);

		vector<Vec4i> lines;

		HoughLinesP(edges, lines, 1, CV_PI/180, thresh, 10, 250)

				for (size_t i=0; i<lines.size(); i++) {
					Vec4i l = lines[i];
					line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, LINE_AA);
				}
		
		imshow("Result Image", edge);

		// Display edge image in GUI
		namedWindow("Display edge", WINDOW_AUTOSIZE );
		imshow("Display edge", edge);
		
		/*
		// Creating filtered depth matrix
		Mat filter(Size(320, 240), CV_16UC1, (void*)filtered.get_data(), Mat::AUTO_STEP);
		GaussianBlur(depth, depth, sze, 0);
		//medianBlur(depth, depth, 6);
		normalize(depth, depth, 0, 65535, NORM_MINMAX);
		medianBlur(depth, depth, 5);	

		// Display filtered depth image in GUI
		namedWindow("Display depth (decimation + median blur)", WINDOW_AUTOSIZE );
		imshow("Display depth (decimation + median blur)", filter);
		*/

    		waitKey(1);
    	}
    
    return 0;
}
