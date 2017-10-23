/***
 Implementation of keyframe class for keyframe data storage and time-varying tensor field computing
 ***/
#ifndef _KEYFRAMELIST_H
#define _KEYFRAMELIST_H
#include "dataStructure.h"
#include "QuadMesh.h"
typedef struct OneKeyFrame{
	icVector2 *vec_vals;
	icMatrix2x2 *jacobian_vals;
	int nverts;
	int keyFrameID;        // the frame ID in the total sequence
}OneKeyFrame;



/*   The class of the user specified key frame list 
*/
class KeyFrameList
{
public:
	OneKeyFrame **keyframes;
	int nkeyframes;
	int curMaxNum;

	/*    Constructor    */

	KeyFrameList(int initsize = 2);


	/*    Destructor    */
	~KeyFrameList();


	/*   release the space of the given pointer to the frame data   */
    void release_oneSlice(OneKeyFrame *onekeyframe);

	/*   add one new slice (allocate memory)   */
	void add_oneSlice(int nverts = 1, int spec_sliceID = 0);

	/*  add the vector value at a vertex to current frame  */
	void add_vec_to_curFrame(int vertID, icMatrix2x2 &jarcobian);

	/*  add the vector value at a vertex to the specified frame  */
	bool add_jar_to_specFrame(int frameID, int vertID, icMatrix2x2 &jarcobian);
	bool add_jar_to_specFrame(OneKeyFrame *theframe, int vertID,icMatrix2x2 &jarcobian);

    bool add_vec_to_specFrame(int frameID, int vertID, double vx, double vy);
    bool add_vec_to_specFrame(OneKeyFrame *theframe, int vertID, double vx, double vy);


	OneKeyFrame *get_frame_pointer(int frameID);

	/*   save the current design vector field at the specified frame   */
	void save_cur_field(int frameID, QuadMesh *obj);

	/*  save the current field to a specified frame in the list 
	(assume it has been initialized)  */
	void add_curField_to_frame (int pos, QuadMesh *obj, int frameID);

	void copy_frame_to_specFrame(OneKeyFrame *source, int frameID);

	/*   reset the specified frame given the frame ID   */
	bool reset_specFrame(int frameID);

	/*   reset the specified frame given the pointer of the frame data   */
	void reset_one_frame(OneKeyFrame *onekeyframe);
	/*   extend the list   */
	bool isFull();

	bool extend(int step = 2);
};

#endif
