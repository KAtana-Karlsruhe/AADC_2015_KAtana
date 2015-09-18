#include "Matcher.h"
#include <iostream>
#include "PatchFactory.h"
#include <vector>
#include "DrawUtil.h"

using namespace cv;

const string PATH = "/home/odroid/lanetracker_test";

int main( int argc, const char* argv[] )
{

    Patch patch = PatchFactory::getPatchByType(katana::PatchType::STRAIGHT, 0);
    Patch rotatedPatch = PatchFactory::getPatchByType(katana::PatchType::STRAIGHT, 4);
    Patch wrongPatch = PatchFactory::getPatchByType(katana::PatchType::STRAIGHT, -4);
    vector<Patch> patches = {patch, rotatedPatch, wrongPatch};

    float width = patch.getSize().width * 1.2;
    float height = patch.getSize().height * 1.2;
    Size size(width, height);

    // base case
    cout << "Base case" << endl;
    string base = "/home/odroid/lanetracker_test/base_case.png";

    Mat baseInput = imread(base, 0);

    Point2f center(baseInput.cols / 2.0f, baseInput.rows / 2.0f);
    RotatedRect baseRect(center, size, 0);

    Matcher m;
    TrackSegment track = m.matchPatterns(baseInput, baseRect, patches);

    cout << "Location: [" << track.getPatchLocation().x << ", " << track.getPatchLocation().y << "]" << endl;
    cout << "Start: " << track.getStartJunction().toString() << endl;
    cout << "End: " << track.getEndJunction().toString() << endl;

    Mat baseOutput;
    cvtColor(baseInput, baseOutput, CV_GRAY2BGR);
    DrawUtil::drawTrack(baseOutput, track);
    DrawUtil::writeImageToFile(baseOutput, "_base");


    cout << "Rotated case" << endl;
    string rotated = "/home/odroid/lanetracker_test/4_rot.png";
    Mat rotatedInput = imread(rotated, 0);
    RotatedRect rotatedRect(center, size, 4);

    track = m.matchPatterns(rotatedInput, rotatedRect, patches);
    cout << "Location: [" << track.getPatchLocation().x << ", " << track.getPatchLocation().y << "]" << endl;
    cout << "Start: " << track.getStartJunction().toString() << endl;
    cout << "End: " << track.getEndJunction().toString() << endl;
    Mat rotatedOutput;
    cvtColor(rotatedInput, rotatedOutput, CV_GRAY2BGR);
    DrawUtil::drawTrack(rotatedOutput, track);
    DrawUtil::writeImageToFile(rotatedOutput, "_rotated");


    cout << "Shifted case" << endl;
    string shifted = "/home/odroid/lanetracker_test/4_rot.png";
    Mat shiftedInput = imread(shifted, 0);
    Point2f shift(50, 50);
    RotatedRect shiftedRect(center + shift, size, 4);
    track = m.matchPatterns(shiftedInput, shiftedRect, patches);

    cout << "Location: [" << track.getPatchLocation().x << ", " << track.getPatchLocation().y << "]" << endl;
    cout << "Start: " << track.getStartJunction().toString() << endl;
    cout << "End: " << track.getEndJunction().toString() << endl;

    Mat shiftedOutput;
    cvtColor(shiftedInput, shiftedOutput, CV_GRAY2BGR);
    DrawUtil::drawTrack(shiftedOutput, track);
    DrawUtil::writeImageToFile(shiftedOutput, "_shifted");



    cout << "Advanced case" << endl;
    string advanced = "/home/odroid/lanetracker_test/4_rot.png";
    Mat advancedInput = imread(shifted, 0);
    RotatedRect advancedRect(center, size, 6);
    Patch small = PatchFactory::getPatchByType(katana::PatchType::STRAIGHT, -2);
    patches.push_back(small);

    track = m.matchPatterns(advancedInput, advancedRect, patches);

    cout << "Location: [" << track.getPatchLocation().x << ", " << track.getPatchLocation().y << "]" << endl;
    cout << "Start: " << track.getStartJunction().toString() << endl;
    cout << "End: " << track.getEndJunction().toString() << endl;

    Mat advancedOutput;
    cvtColor(advancedInput, advancedOutput, CV_GRAY2BGR);
    DrawUtil::drawTrack(advancedOutput, track);
    DrawUtil::writeImageToFile(advancedOutput, "_advanced");


}




