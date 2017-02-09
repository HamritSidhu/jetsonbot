#include <ros/ros.h>
#include <stdio.h>
#include "fydp/MoveData.h"

bool callFlag = false;

const int queueSize = 5;
fydp::MoveData moveDataQueue[queueSize];
fydp::MoveData predictedData;

void initMoveDataQueue() {
	fydp::MoveData centroid;
	centroid.x = 256;
	centroid.y = 0;
	centroid.area = 22000;

	for (int i = 0; i < queueSize; i++)
		moveDataQueue[i] = centroid;
}

fydp::MoveData predictMoveData() {
        int counter = 0;
        int firstOrderDifferenceSumX = 0;
        int firstOrderDifferenceSumArea = 0;

        for (int i = 0; i < queueSize - 1; i++) {
             	firstOrderDifferenceSumX = moveDataQueue[i+1].x - moveDataQueue[i].x;
               	firstOrderDifferenceSumArea = moveDataQueue[i+1].area - moveDataQueue[i].area;
        	counter++;
        }

        // integer division here
        int firstOrderDifferenceAverageX = firstOrderDifferenceSumX / counter;
        int firstOrderDifferenceAverageArea = firstOrderDifferenceSumArea / counter;

        predictedData.x = moveDataQueue[queueSize - 1].x + firstOrderDifferenceAverageX;
        predictedData.area - moveDataQueue[queueSize -1].area + firstOrderDifferenceAverageArea;

        fydp::MoveData result = predictedData;
        return result;
}

void predictionProcessing(const fydp::MoveData& msg) {
	ROS_INFO("%d", msg.x);
	ROS_INFO("%d", msg.y);
	ROS_INFO("%d", msg.area);
	
	// updating the queue
	for (int i = 0; i < queueSize - 1; i++) {
		moveDataQueue[i] = moveDataQueue[i+1];
	}
	moveDataQueue[queueSize - 1] = msg;
	callFlag = true;
}

int main(int argc, char **argv) {
	initMoveDataQueue();
	ros::init(argc, argv, "predictor");
	ros::NodeHandle nodeHanle;
	ros::Publisher pub = nodeHanle.advertise<fydp::MoveData>("follower", 1000);
	ros::Subscriber sub = nodeHanle.subscribe("camera", 1000, predictionProcessing);
	while(1) {
		while(!callFlag) {
			ros::spinOnce();
		}
		pub.publish(predictMoveData());
		callFlag = false;
	}
}
