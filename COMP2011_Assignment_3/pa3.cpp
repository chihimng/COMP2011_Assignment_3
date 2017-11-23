/*
 *       File: pa3.cpp
 * Created on: Nov 1, 2017
 *     Author: Yao Yao (yyaoag@cse.ust.hk)
 *       Note: Implement all functions without changing any function prototype.
 *             This is the only file you should modify and submit.
 */

#include <iostream>
#include "structure.h"
using namespace std;

/*
 * Description: get the pointer to the frame through frame index.
 *
 *       video: input video structure.
 * frame_index: the index of the frame to be extracted.
 *      return: the pointer to the frame or nullptr if the frame does not exsist.
 */
Frame * GetFrame(const Video & video, const int frame_index)
{
	// your implementation
    int i = 0;
    for (Frame* p = video.first_frame; p != nullptr; p = p->next_frame, i++) {
        if (i == frame_index) {
            return p;
        }
    }
    return nullptr;
}

/*
 * Description: get the pointer to the vehicle through vehicle index.
 *
 *         video: input video structure.
 * vehicle_index: the index of the vehicle to be extracted.
 *        return: the pointer to the vehicle or nullptr if the vehicle does not exsist.
 */
Vehicle * GetVehicle(const Video & video, const int vehicle_index)
{
	// your implementation
    return vehicle_index < video.num_vehicles ? video.vehicles[vehicle_index] : nullptr;
}

/*
 * Description: get the pointer to vf_info of a vehicle in one frame.
 *
 *     vehicle: input vehicle structure.
 * frame_index: the frame index of the vf_info to be extracted from this vehicle.
 *      return: the pointer to the vf_info or nullptr if the vehicle does not visible in
 *              this frame.
 */
VehicleFrameInfo * GetVFInfo(const Vehicle * vehicle, const int frame_index)
{
	// your implementation
    for (VehicleFrameInfo* p = vehicle->first_frame_info; p != nullptr; p = p->next_frame_info) {
        if (p->frame_index == frame_index) {
            return p;
        }
    }
    return nullptr;
}

/*
 * Description: initialize a new frame in the video during frame processing. You need to
 *              create a new frame, and initialize this new frame and add it to the link
 *              list. In this function, you don't need to use hard copy to initialize
 *              Frame.image. Also, you don't need to analyze the vehicles in this frame
 *              (set num_vehicles of this new frame as 0 and all pointers to vehicles as
 *              nullptr).
 *
 *  video: input video structure.
 * return: true if finish and false if all frames has been processed.
 */
bool InitializeNewFrame(Video & video)
{
	// your implementation
    if (video.num_processed_frames >= video.num_frames) {
        return false;
    }
    // init data
    Frame* newFramePtr = new Frame;
    newFramePtr->index = video.num_processed_frames;
    newFramePtr->image = video.raw_data[video.num_processed_frames];
    newFramePtr->num_vehicles = 0;
    for (int i = 0; i < MAX_VEHICLE_NUM; i++) {
        newFramePtr->vehicles[i] = nullptr;
    }
    newFramePtr->next_frame = nullptr;
    // append linked list
    if (video.first_frame == nullptr) {
        video.first_frame = newFramePtr;
    } else {
        for (Frame* p = video.first_frame; p != nullptr; p = p->next_frame) {
            if (p->next_frame == nullptr) {
                p->next_frame = newFramePtr;
                break;
            }
        }
    }
    video.num_processed_frames += 1;
    return true;
}

/*
 * Description: add one vf_info to the video. You need to check if the vf_info, the
 *              corresponding video and the corresponding the vehicle has been in the
 *              video before you add the VFInfo.
 *
 *              video: input video structure.
 * vehicle_frame_info: pointer to the new info to be added. (vf_info)
 *             return: return true if finish and false if
 *                     1. the frame or the vehicle of this vf_info is not in this video or
 *                     2. vf_info has already been in the list of the vehicle.
 */
bool AddVFInfo(Video & video, VehicleFrameInfo * vehicle_frame_info)
{
	// your implementation
    // check if frame and vehicle in video
    Frame* targetFramePtr = GetFrame(video, vehicle_frame_info->frame_index);
    Vehicle* targetVehiclePtr = GetVehicle(video, vehicle_frame_info->vehicle_index);
    if (targetFramePtr == nullptr || targetVehiclePtr == nullptr) {
        return false;
    }
    // append vf_info
    for (VehicleFrameInfo* p = targetVehiclePtr->first_frame_info; p != nullptr; p = p->next_frame_info) {
        // check if vf_info added already
        if (p->next_frame_info == vehicle_frame_info) {
            return false;
        }
        if (p->next_frame_info == nullptr) {
            p->next_frame_info = vehicle_frame_info;
            targetFramePtr->vehicles[targetVehiclePtr->index] = targetVehiclePtr;
            targetFramePtr->num_vehicles += 1;
            targetVehiclePtr->num_visible_frames += 1;
            break;
        }
    }
    return true;
}

bool isVehicleInFrame(const Vehicle* vehicle, const Frame* frame) {
    for (int i = 0; i < MAX_VEHICLE_NUM && frame->vehicles[i] != nullptr; i++) {
        if (frame->vehicles[i]->index == vehicle->index) {
            return true;
        }
    }
    return false;
}

/*
 * Description: track the vehicle in the new frame and return the new info. The speed
 *              is the distance between positions of the vehicle in two frames and note
 *              that 1. the maximum speed of one vehicle is MAX_SPEED and 2. the minimum
 *              distance between vehicles in the same lane is MAX_SPEED and 3. the vehicle
 *              is assumed always to stay in the same lane.
 *       vehicle: pointer to the vehicle to be tracked.
 * current_frame: pointer to the frame the vehicle to be tracked in.
 *    prev_frame: pointer to the previous frame the vehicle was in.
 *        return: the pointer to the new info or nullptr if 1. the vehicle is not in
 *                prev_frame or 2. the vehicle left scope of current frame.
 */
VehicleFrameInfo * TrackVehicle(const Vehicle * vehicle, const Frame * current_frame, const Frame * prev_frame)
{
	// your implementation
    if (!isVehicleInFrame(vehicle, prev_frame)) {
        return nullptr;
    }
    VehicleFrameInfo* newVFInfoPtr = new VehicleFrameInfo;
    newVFInfoPtr->vehicle_index = vehicle->index;
    newVFInfoPtr->frame_index = current_frame->index;
    VehicleFrameInfo* lastVFInfoPtr = vehicle->first_frame_info;
    while (lastVFInfoPtr->next_frame_info != nullptr) {
        lastVFInfoPtr = lastVFInfoPtr->next_frame_info;
    }
    int row = lastVFInfoPtr->position[0];
    int lastCol = lastVFInfoPtr->position[1];
    int newCol = lastCol;
    for (; newCol <= lastCol+MAX_SPEED; newCol++) {
        if (current_frame->image[row][newCol] == '*') {
            break;
        }
    }
    if (newCol >= COLS) {
        delete newVFInfoPtr;
        newVFInfoPtr = nullptr;
        return nullptr;
    }
    newVFInfoPtr->position[0] = row;
    newVFInfoPtr->position[1] = newCol;
    newVFInfoPtr->speed = newCol - lastCol;
    newVFInfoPtr->next_frame_info = nullptr;
    return newVFInfoPtr;
}

/*
 * Description: find and add new vehicles in the last frame of the frame list. You need to 1. find the
 *              new vehicles in the first colume of each lane, and 2. create and initialize a new vehicle
 *              is a new vehicle is found and, 3. add the new vehicle to the video and, 4. create a
 *              vf_info for the new vehicle in this frame and, 5. add the vf_info to the video.
 *
 *  video: input video structure.
 * return: true is finish or false if the frame index is invalid.
 */
bool FindAndAddNewVehicles(Video & video)
{
	// your implementation

	// Hints:
	// detect new vehicles lane by lane

		// check if there is a new vehicle in the lane

		// construct and add a new vehicle

		// construct and add a new vf_info
    Frame* lastFrame = video.first_frame;
    while (lastFrame->next_frame != nullptr) {
        lastFrame = lastFrame->next_frame;
    }
    for (int i = 1; i < ROWS; i += 2) {
        if (lastFrame->image[i][0] == '*') {
            Vehicle* newVehiclePtr = new Vehicle;
            newVehiclePtr->index = video.num_vehicles;
            newVehiclePtr->num_visible_frames = 1;

            VehicleFrameInfo* newVFInfoPtr = new VehicleFrameInfo;
            newVFInfoPtr->vehicle_index = video.num_vehicles;
            newVFInfoPtr->frame_index = lastFrame->index;
            newVFInfoPtr->position[0] = i;
            newVFInfoPtr->position[1] = 0;
            newVFInfoPtr->speed = 1;
            newVFInfoPtr->next_frame_info = nullptr;

            newVehiclePtr->first_frame_info = newVFInfoPtr;
            video.vehicles[video.num_vehicles] = newVehiclePtr;
            video.num_vehicles += 1;
            lastFrame->vehicles[lastFrame->num_vehicles] = newVehiclePtr;
            lastFrame->num_vehicles += 1;
        }
    }
    return true;
    // FIXME: what the fuck is frame index invalid?????????????
}

/*
 * Description: calcute the average speed of all vehicles in all their visible frames (the average of all speeds).
 *
 *  video: input video structure.
 * return: the average speed or 0 if no vehicle is in the video.
 */
double AverageRoadSpeed(Video & video)
{
	// your implementation
    double sum = 0;
    int count = 0;
    for (int i = 0; i < video.num_vehicles; i++) {
        VehicleFrameInfo* vhInfoPtr = video.vehicles[i]->first_frame_info;
        while (vhInfoPtr->next_frame_info != nullptr) {
            sum += vhInfoPtr->speed;
            count += 1;
            vhInfoPtr = vhInfoPtr->next_frame_info;
        }
    }
    if (count <= 0) {
        return 0;
    } else {
        return sum / count;
    }
}

void deleteFrame(Frame* framePtr) {
    if (framePtr == nullptr) {
        return;
    }
    deleteFrame(framePtr->next_frame);
    delete framePtr;
    framePtr = nullptr;
}

void deleteVFInfo(VehicleFrameInfo* vfInfoPtr) {
    if (vfInfoPtr == nullptr) {
        return;
    }
    deleteVFInfo(vfInfoPtr->next_frame_info);
    delete vfInfoPtr;
    vfInfoPtr = nullptr;
}

/*
 * Description: clean all memories of the video, including raw_data, all frames, vehicles and vf_infos.
 *
 *  video: input video to be clean.
 * return: no return.
 */
void CleanVideo(Video & video)
{
	// your implementation
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            delete [] video.raw_data[i][j];
            video.raw_data[i][j] = nullptr;
        }
        delete [] video.raw_data[i];
        video.raw_data[i] = nullptr;
    }
    delete [] video.raw_data;
    video.raw_data = nullptr;

    deleteFrame(video.first_frame);
    video.first_frame = nullptr;

    for (int i = 0; i < video.num_vehicles; i++) {
        deleteVFInfo(video.vehicles[i]->first_frame_info);
        delete video.vehicles[i];
        video.vehicles[i] = nullptr;
    }
}
