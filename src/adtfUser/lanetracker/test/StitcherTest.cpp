#include "Stitcher.h"
#include <cmath>
#include <iostream>
#include "DrawUtil.h"
#include <vector>
#include "TrackSegment.h"


int main( int argc, const char* argv[] )
{
    Patch patch = PatchFactory::getPatchByType(katana::PatchType::STRAIGHT, 6);
    vector<TrackSegment> consistentPatches;

    TrackSegment start(patch, 0);
    start.translate(Point(134, 387));

    cout << "Location: [" << start.getPatchLocation().x << ", " << start.getPatchLocation().y << "]" << endl;
    cout << "Start: " << start.getStartJunction().toString() << endl;
    cout << "End: " << start.getEndJunction().toString() << endl;



    consistentPatches.push_back(start);

    Stitcher stitcher;

    cout << "Easy case" << endl;
    string easy = "/home/odroid/lanetracker_test/stitcher_easy.png";
    Mat easyInput = imread(easy, 0);

    vector<TrackSegment> track = stitcher.stitch(easyInput, consistentPatches);


    Mat easyOutput;
    cvtColor(easyInput, easyOutput, CV_GRAY2BGR);

    for (TrackSegment t : track) {
        DrawUtil::drawTrack(easyOutput, t);
    }
    DrawUtil::writeImageToFile(easyOutput, "_easy");

    cout << "Curve case" << endl;
    string curve = "/home/odroid/lanetracker_test/stitcher_curve.png";
    Mat curveInput = imread(curve, 0);

    track = stitcher.stitch(curveInput, consistentPatches);


    Mat curveOutput;
    cvtColor(curveInput, curveOutput, CV_GRAY2BGR);

    for (TrackSegment t : track) {
        DrawUtil::drawTrack(curveOutput, t);
    }
    DrawUtil::writeImageToFile(curveOutput, "_curve");
}



