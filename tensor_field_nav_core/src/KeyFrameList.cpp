#include "tensor_field_nav_core/KeyFrameList.h"
KeyFrameList::KeyFrameList(int initsize)
{
	if(initsize == 0)
	{
		keyframes = NULL;
		nkeyframes = curMaxNum = 0;
		return;
	}

	keyframes=(OneKeyFrame**)malloc(sizeof(OneKeyFrame*)*initsize);
	int i;
	for(i=0; i<initsize; i++)
		keyframes[i] = NULL;
	nkeyframes = 0;
	curMaxNum = initsize;
}

/*    Destructor    */
KeyFrameList::~KeyFrameList()
{
	if(keyframes == NULL)
		return;

	int i;
	for(i=0; i<curMaxNum; i++)
	{
		if(keyframes[i] == NULL)
			continue;

		release_oneSlice(keyframes[i]);
	}

	free(keyframes);
}

/*   release the space of the given pointer to the frame data   */
void KeyFrameList::release_oneSlice(OneKeyFrame *onekeyframe)
{
	free(onekeyframe->jacobian_vals);
	onekeyframe->nverts = onekeyframe->keyFrameID = 0;
	onekeyframe = NULL;
	nkeyframes--;
}

/*   add one new slice (allocate memory)   */
void KeyFrameList::add_oneSlice(int nverts, int spec_sliceID)
{
	if(isFull())
	{
		if(!extend())
			exit(-1);
	}

	keyframes[nkeyframes]=(OneKeyFrame*)malloc(sizeof(OneKeyFrame));
	keyframes[nkeyframes]->jacobian_vals =(icMatrix2x2*)malloc(sizeof(icMatrix2x2)*nverts);

	if(keyframes[nkeyframes]->jacobian_vals == NULL) exit(-1);
    keyframes[nkeyframes]->vec_vals = (icVector2*)malloc(sizeof(icVector2)*nverts);

    if(keyframes[nkeyframes]->vec_vals == NULL) exit(-1);
	keyframes[nkeyframes]->nverts = nverts;
	keyframes[nkeyframes]->keyFrameID = spec_sliceID;

	/*  re-order the key frames  */
	int i, pos = -1;
	OneKeyFrame *curFrame = keyframes[nkeyframes];

	for(i=0; i<nkeyframes; i++)
	{
		if(curFrame->keyFrameID<keyframes[i]->keyFrameID)
		{
			pos = i;
			break;
		}
	}

	if(pos >= 0 && pos < nkeyframes)
	{
		/*  move backward  */
		for(i=nkeyframes-1; i>=pos; i--)
		{
			keyframes[i+1]=keyframes[i];
		}

		keyframes[pos]=curFrame;
	}

	nkeyframes ++;
}

/*  add the vector value at a vertex to current frame  */
void KeyFrameList::add_vec_to_curFrame(int vertID, icMatrix2x2 &jarcobian)
{
	if(keyframes == NULL) return;

	if(vertID>=keyframes[nkeyframes]->nverts) return;

	keyframes[nkeyframes]->jacobian_vals[vertID]=jarcobian;
}

/*  add the vector value at a vertex to the specified frame  */
bool KeyFrameList::add_jar_to_specFrame(int frameID, int vertID, icMatrix2x2 &jarcobian){
	/*   search for the frame   */
	int i, pos = -1;

	for(i=0; i<nkeyframes; i++)
	{
		if(keyframes[i]->keyFrameID == frameID)
		{
			pos = i;
			break;
		}
	}

	if(pos < 0) return false;

	if(vertID>=keyframes[pos]->nverts) return false;
	keyframes[pos]->jacobian_vals[vertID]=jarcobian;

	return true;
}

bool KeyFrameList::add_jar_to_specFrame(OneKeyFrame *theframe, int vertID,icMatrix2x2 &jarcobian)
{

	if(theframe == NULL) return false;

	if(vertID>=theframe->nverts) return false;

	theframe->jacobian_vals[vertID]=jarcobian;
	return true;
}

bool KeyFrameList::add_vec_to_specFrame(int frameID, int vertID, double vx, double vy)
{
    /*   search for the frame   */
    int i, pos = -1;

    for(i=0; i<nkeyframes; i++)
    {
        if(keyframes[i]->keyFrameID == frameID)
        {
            pos = i;
            break;
        }
    }

    if(pos < 0) return false;

    if(vertID>=keyframes[pos]->nverts) return false;

    keyframes[pos]->vec_vals[vertID].entry[0]=vx;
    keyframes[pos]->vec_vals[vertID].entry[1]=vy;
    return true;
}


bool KeyFrameList::add_vec_to_specFrame(OneKeyFrame *theframe, int vertID, double vx, double vy)
{

    if(theframe == NULL) return false;

    if(vertID>=theframe->nverts) return false;

    theframe->vec_vals[vertID].entry[0]=vx;
    theframe->vec_vals[vertID].entry[1]=vy;
    return true;
}


OneKeyFrame* KeyFrameList::get_frame_pointer(int frameID)
{
	int i, pos = -1;

	for(i=0; i<nkeyframes; i++)
	{
		if(keyframes[i]->keyFrameID == frameID)
		{
			pos = i;
			break;
		}
	}

	if(pos < 0) return NULL;

	return keyframes[pos];
}

/*   save the current design vector field at the specified frame   */
void KeyFrameList::save_cur_field(int frameID, QuadMesh *obj)
{
	add_oneSlice(obj->nverts, frameID);

	OneKeyFrame *theframe = get_frame_pointer(frameID);

	int i;

	for(i=0; i<obj->nverts; i++)
	{
		add_jar_to_specFrame(theframe,i, obj->quad_verts[i]->Jacobian);
        add_vec_to_specFrame(theframe, i, obj->quad_verts[i]->major.entry[0], obj->quad_verts[i]->major.entry[1]);
	}
}

/*  save the current field to a specified frame in the list 
(assume it has been initialized)  */
void KeyFrameList::add_curField_to_frame (int pos, QuadMesh *obj, int frameID)
{
	if (pos < 0 || pos >= nkeyframes) return;

	OneKeyFrame *frame = keyframes[pos];

	for (int i=0; i<obj->nverts; i++)
	{
		add_jar_to_specFrame (frame, i, obj->quad_verts[i]->Jacobian);
	}

	frame->keyFrameID = frameID;
}

void KeyFrameList::copy_frame_to_specFrame(OneKeyFrame *source, int frameID)
{
	OneKeyFrame *theframe = get_frame_pointer(frameID);

	if (theframe == NULL) return;

	int i;

	for (i=0; i<theframe->nverts; i++)
	{
		add_jar_to_specFrame(theframe, i, source->jacobian_vals[i]);
	}
}

/*   reset the specified frame given the frame ID   */
bool KeyFrameList::reset_specFrame(int frameID)
{
	/*   search for the frame   */
	int i, pos = -1;

	for(i=0; i<nkeyframes; i++)
	{
		if(keyframes[i]->keyFrameID == frameID)
		{
			pos = i;
			break;
		}
	}

	if(pos < 0) return false;

	for(i=0; i<keyframes[pos]->nverts; i++)
		keyframes[pos]->jacobian_vals[i]=0.0;

	return true;
}

/*   reset the specified frame given the pointer of the frame data   */
void KeyFrameList::reset_one_frame(OneKeyFrame *onekeyframe)
{
	int i;

	for(i=0; i<onekeyframe->nverts; i++)
		onekeyframe->jacobian_vals[i]=0.0;
}

bool KeyFrameList::isFull()
{
	if(nkeyframes >= curMaxNum) return true;
	return false;
}

/*   extend the list   */
bool KeyFrameList::extend(int step)
{
	OneKeyFrame **temp = keyframes;

	keyframes = (OneKeyFrame**)malloc(sizeof(OneKeyFrame*)*(curMaxNum+step));

	if(keyframes == NULL)
		return false;

	int i;
	for(i=0; i<curMaxNum; i++)
		keyframes[i]=temp[i];

	for(i=curMaxNum; i<curMaxNum+step; i++)
		keyframes[i]=NULL;

	curMaxNum += step;
	return true;
}
