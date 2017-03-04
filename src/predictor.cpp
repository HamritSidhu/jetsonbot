#include <ros/ros.h>
#include <stdio.h>
#include "fydp/MoveData.h"

bool callFlag = false;

const int queueSize = 4;
fydp::MoveData moveDataQueue[queueSize];
fydp::MoveData predictedData;

void initMoveDataQueue(const fydp::MoveData& msg) {
	fydp::MoveData centroid;
	centroid.x = 250;
	centroid.y = 188;
	centroid.area = msg.area;
	ROS_INFO("Initial Area: %d", centroid.area);
	
	for (int i=0; i<queueSize; i++) {
		moveDataQueue[i] = centroid;
	}

}

fydp::MoveData predictMoveData() {
        int counter = 0;
        int firstOrderDifferenceSumX = 0;
		int firstOrderDifferenceSumY = 0;
        long firstOrderDifferenceSumArea = 0;
        
        for (int i=0; i<queueSize-1; i++) {
            firstOrderDifferenceSumX += (moveDataQueue[i+1].x - moveDataQueue[i].x);
            firstOrderDifferenceSumY += (moveDataQueue[i+1].y - moveDataQueue[i].y);
			firstOrderDifferenceSumArea += (moveDataQueue[i+1].area - moveDataQueue[i].area);
        	counter++;
        }

        // integer division here
        int firstOrderDifferenceAverageX = firstOrderDifferenceSumX / counter;
        int firstOrderDifferenceAverageY = firstOrderDifferenceSumY / counter;
        long firstOrderDifferenceAverageArea = firstOrderDifferenceSumArea / counter;

        predictedData.x = moveDataQueue[queueSize - 1].x + firstOrderDifferenceAverageX; 
        predictedData.y = moveDataQueue[queueSize - 1].y + firstOrderDifferenceAverageY;
		predictedData.area = moveDataQueue[queueSize -1].area + firstOrderDifferenceAverageArea;

        fydp::MoveData result = predictedData;
        return result;
}

void predictionProcessing(const fydp::MoveData& msg) {
	// shift everyting 1 to the left
	for (int i = 0; i < queueSize - 1; i++) {
		moveDataQueue[i] = moveDataQueue[i+1];
	}
	
	// find moving average
	int sum_x = 0;
	int sum_y = 0;
	long sum_area = 0;
	for (int i = 0; i < queueSize - 1; i++) {
		sum_x += moveDataQueue[i].x;
		sum_y += moveDataQueue[i].y;
		sum_area += moveDataQueue[i].area;
	}
	sum_x += msg.x;
	sum_y += msg.y;
	sum_area += msg.area;
	
	fydp::MoveData res;
	res.x = sum_x/queueSize;
	res.y = sum_y/queueSize;
	res.area = sum_area/queueSize;
	
    ROS_INFO("%%%%%%%% Averaged Data %%%%%%%%");
    ROS_INFO("%d", res.x);
	ROS_INFO("%d", res.y);
	ROS_INFO("%d", res.area);
	
	moveDataQueue[queueSize - 1] = res;
	callFlag = true;
}

int main(int argc, char **argv) {
	//initMoveDataQueue();
	ros::init(argc, argv, "predictor");
	ros::NodeHandle nodeHanle;
	ros::Subscriber in = nodeHanle.subscribe("init", 1000, initMoveDataQueue);
	ros::Publisher pub = nodeHanle.advertise<fydp::MoveData>("follower", 1000);
	ros::Subscriber sub = nodeHanle.subscribe("camera", 1000, predictionProcessing);
	
	
	while(1) {
		while(!callFlag) {
			ros::spinOnce();
		}
		fydp::MoveData result = predictMoveData();
		ROS_INFO("%%%%%%%% Predicted Data %%%%%%%%");
        ROS_INFO("%d", result.x);
		ROS_INFO("%d", result.y);
		ROS_INFO("%d", result.area);
		pub.publish(result);
		callFlag = false;
	}
}
