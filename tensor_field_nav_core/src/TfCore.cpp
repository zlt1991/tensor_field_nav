#include"tensor_field_nav_core/TfCore.h"

TfCore::TfCore(ros::NodeHandle private_nh_ ):nh_(),m_it(nh_)
{
    //tensor field
	major_iframe = 0;
	quadmesh = NULL;
	ntenelems = 0;
	nten_regelems = 0;
	ndegpts = 0;
	ten_regularelems=NULL;
	degpts = NULL;
	curMaxNumTenRegElems =100;
	MaxNumDegeneratePts = 200;
	ten_tmax   = NPIX/(SCALE*NPN);
    ten_dmax   = SCALE/NPIX;
	iframe = 0; 
	Npat   = 32;
	alpha  = (0.12*255);
	tmax   = NPIX/(SCALE*NPN);
	dmax   = SCALE/NPIX;
	ten_designelems = NULL;
	curMaxNumTenDesignElems = 100;
	ndegenerate_tris = 0;
	degenerate_tris = NULL;
	major_path=NULL;
    NUM_Slices = 0;
    keyframes = NULL;
    enRelaxKeyframeOn=false;
    InterpScheme=1;
    curSliceID=0;
    showGridOn=false;
    showIBFVOn=true;
    showRegularElemOn=false;
    showSingularitiesOn=false;
    showMajorTenLine=true;
    interFrameNum=3;
    interFrameTime=0.2;
    //
    isReceiveNewMap=false;
    ifReachTrisector=false;
    targetTrisectorIndex=-1;
    safeDistance=0.3;
    realWorld_to_field_scale=25;
    realWorld_to_field_offset_x=0.5;
    realWorld_to_field_offset_y=0.5;
    rgb_im = (unsigned char *)malloc(NPIX * NPIX*3* sizeof(unsigned char));
    //
    //ros
    robotInitialPos_x=0.5;
    robotInitialPos_y=0.5;
    rgbTopicName="/tf_image";
    gridMapTopicName="/projected_map";
    worldFrameId="odom";
    baseFrameId="base_footprint";
    recoverMode=false;
    ifCancelPath=false;
    ifFinish_goTri=false;
    ifFinish_recover=false;
    ifFinish_turn=false;
    //
    ros::NodeHandle private_nh(private_nh_);
    private_nh.param("world_to_field_scale", realWorld_to_field_scale, realWorld_to_field_scale);
    private_nh.param("world_to_field_offset_x", realWorld_to_field_offset_x, realWorld_to_field_offset_x);
    private_nh.param("world_to_field_offset_y", realWorld_to_field_offset_y, realWorld_to_field_offset_y);
    private_nh.param("robot_initial_pos_x", robotInitialPos_x, robotInitialPos_x);
    private_nh.param("robot_initial_pos_y", robotInitialPos_y, robotInitialPos_y);
    private_nh.param("rgb_pub_topic_name", rgbTopicName, rgbTopicName);
    private_nh.param("gridMap_topic_name", gridMapTopicName, gridMapTopicName);
    private_nh.param("world_frame_id", worldFrameId, worldFrameId);
    private_nh.param("base_frame_id", baseFrameId, baseFrameId);
    private_nh.param("safe_disance", safeDistance, safeDistance);
    private_nh.param("inter_frame_num", interFrameNum, interFrameNum);
    private_nh.param("inter_frame_time", interFrameTime, interFrameTime);

    rgb_pub = m_it.advertise(rgbTopicName, 1);
    gridmap_sub=nh_.subscribe(gridMapTopicName,1,&TfCore::gridMap_callback,this);
    frontierPoints_sub=nh_.subscribe("frontier_points",1,&TfCore::frontier_point_callback,this);
    goTriExecuteclient=nh_.serviceClient<pure_pursuit_controller::executePath>("execute_goTri");
    pathExecuteClient=nh_.serviceClient<pure_pursuit_controller::executePath>("execute_path");
    pathCancelClient=nh_.serviceClient<pure_pursuit_controller::cancelPath>("cancel_path");
    pathCutClient=nh_.serviceClient<pure_pursuit_controller::cutPath>("cut_path");
    pathRecoverClient=nh_.serviceClient<pure_pursuit_controller::recoverPath>("recover_path");
    turnDirectClient=nh_.serviceClient<pure_pursuit_controller::recoverPath>("turn_direction");
    finishGoTri=nh_.subscribe("/pure_pursuit_controller/finish_go_tri",1,&TfCore::goTri_callback,this);
    finish_recover=nh_.subscribe("/pure_pursuit_controller/finish_recover",1,&TfCore::finishRecover_callback,this);
    finish_turn=nh_.subscribe("/pure_pursuit_controller/finish_turn",1,&TfCore::finishTurn_callback,this);
    //
    robot_loc=new Seed();
    robot_loc->pos[0]=robotInitialPos_x;
    robot_loc->pos[1]=robotInitialPos_y;
    robot_loc->triangle=0;
    realWorld_to_field_scale=realWorld_to_field_scale;
    realWorld_to_field_offset_x=realWorld_to_field_offset_x;
    realWorld_to_field_offset_y=realWorld_to_field_offset_y;
	//
}

TfCore::~TfCore(){

}

void TfCore::TfCoreInit(){
    quadmesh = new QuadMesh(NMESH, NMESH, -0.01, 1.01, -0.01, 1.01);
    init_majorPath();
    init_separatrices();
	init_ten_designelems();
	init_degpts();
	zoom_factor = 1;
	trans_x=trans_y=0;
	cal_inverse_transform();
	InitGL();	

	make_tens_Patterns();
	tensor_init_tex();

    init_keyframeList();
    init_recover_points();
    create_paraThread();

}
void TfCore::makePatterns(void)
{
    int lut[256];
    int phase[NPN][NPN];
    GLubyte pat[NPN][NPN][4];
    int i, j, k, t;

    for (i = 0; i < 256; i++) lut[i] = i < 127 ? 0 : 255;
    for (i = 0; i < NPN; i++)
        for (j = 0; j < NPN; j++) phase[i][j] = rand() % 256;

    for (k = 0; k < Npat; k++) {
        t = k*256/Npat;
        for (i = 0; i < NPN; i++)
            for (j = 0; j < NPN; j++) {
                pat[i][j][0] =
                    pat[i][j][1] =
                    pat[i][j][2] = lut[(t + phase[i][j]) % 255];
                pat[i][j][3] = alpha;
            }
            glNewList(k + 1, GL_COMPILE);
            glTexImage2D(GL_TEXTURE_2D, 0, 4, NPN, NPN, 0,
                GL_RGBA, GL_UNSIGNED_BYTE, pat);
            glEndList();
    }
}

void TfCore::make_tens_Patterns(void)
{
	int lut[256];
	int phase[NPN][NPN];
	GLubyte pat[NPN][NPN][4];
	GLubyte spat[NPN][NPN][4];
	int i, j, k, t;

	for (i = 0; i < 256; i++) lut[i] = i < 127 ? 0 : 255;
	for (i = 0; i < NPN; i++)
		for (j = 0; j < NPN; j++) phase[i][j] = rand() % 256; 

	for (k = 200; k < Npat+200; k++) {
		//t = k*256/Npat;                           //t is used to control the animation of the image
		for (i = 0; i < NPN; i++) 
			for (j = 0; j < NPN; j++) {
				pat[i][j][0] = 
					pat[i][j][1] = 
					pat[i][j][2] = (GLubyte)lut[ phase[i][j] % 255];
				pat[i][j][3] = alpha+5;

				spat[i][j][0] = 
					spat[i][j][1] = 
					spat[i][j][2] = (GLubyte)lut[ phase[i][j] % 255];
				spat[i][j][3] = alpha+5;
			}

			glNewList(k + 1, GL_COMPILE);  //major texture
			glTexImage2D(GL_TEXTURE_2D, 0, 4, NPN, NPN, 0, 
				GL_RGBA, GL_UNSIGNED_BYTE, pat);
			glEndList();


			glNewList(k + 1 + 100, GL_COMPILE);       //This is for minor image
			glTexImage2D(GL_TEXTURE_2D, 0, 4, NPN, NPN, 0, 
				GL_RGBA, GL_UNSIGNED_BYTE, spat);
			glEndList();   
	}
}

int TfCore::InitGL()								// All Setup For OpenGL Goes Here
{
    glViewport(0, 0, (GLsizei)REALWINSIZE, (GLsizei)REALWINSIZE);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); 
	gluOrtho2D(0, 1, 0, 1);
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_WRAP_S, GL_REPEAT); 
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_WRAP_T, GL_REPEAT); 
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, 
		GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClear(GL_COLOR_BUFFER_BIT);

	glDisable(GL_STENCIL_TEST);

	return true;										// Initialization Went OK
}
void TfCore::init_regular_ten_designelems()
{
	/*initialize regular design element*/
	if(ten_regularelems == NULL)
	{
        ten_regularelems=(TenRegularElem *)malloc(sizeof(TenRegularElem)*curMaxNumTenRegElems);
		if(ten_regularelems == NULL)
			exit(-1);
	}
	nten_regelems = 0;
}

void TfCore::init_degpts()
{
    /*initialize degerate design element*/
    if(degpts!=NULL) {free(degpts); degpts=NULL;}
    if(degpts == NULL)
	{
		degpts = (DegeneratePt*)malloc(sizeof(DegeneratePt)*MaxNumDegeneratePts);
		if(degpts == NULL)
			exit(-1);
	}
	ndegpts = 0;

    int i;
    for(i=0; i<quadmesh->nfaces; i++)
    {
		quadmesh->quadcells[i]->degpt_index = -1;
    }
}


/*initialize the textures*/
void TfCore::tensor_init_tex()
{
	glDrawBuffer(GL_BACK);

	glCallList(201);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,  0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0,  ten_dmax); glVertex2f(0.0, 1.0);
	glTexCoord2f(ten_dmax, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(ten_dmax, ten_dmax); glVertex2f(1.0, 1.0);
	glEnd();

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex1);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex2);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex);
}

void TfCore::init_majorPath(){
    major_path=new EvenStreamlinePlace(1);
    major_path->setTfCore(this);
    major_path->init();
    major_path->set_default_parameters();
    major_path->init_major_line_info();
}

void TfCore::init_separatrices(){
    separatrices=new EvenStreamlinePlace(50);
    separatrices->setTfCore(this);
    separatrices->init();
    separatrices->set_default_parameters();
    separatrices->init_major_line_info();
}

void TfCore::cal_inverse_transform()
{
	glMatrixMode(GL_MODELVIEW_MATRIX);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(-trans_x, -trans_y, 0);
	glTranslatef(0.5, 0.5, 0);
	glScalef(1./zoom_factor, 1./zoom_factor, 1./zoom_factor);
	glTranslatef(-.5,-.5, 0);

	glGetFloatv(GL_MODELVIEW_MATRIX, inverse_tran);

	glPopMatrix();
}

void TfCore::transform_fun(){
    glTranslatef(trans_x, trans_y, 0);
    glTranslatef(0.5, 0.5, 0);
    glScalef(zoom_factor, zoom_factor, zoom_factor);
    glTranslatef(-.5,-.5, 0);
}


int TfCore::without_anti_aliasing(GLenum mode)
{
	glClearColor(0.93, 0.93, 0.87, 1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix ();
	transform_fun();
	if(showIBFVOn) /*visualize tensor field*/
	{
		glEnable(GL_TEXTURE_2D);
		glShadeModel(GL_FLAT);
        /*using quad mesh */
        render_majorfield_quad();  /*  showing major field only  */

	}
	else
	{
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glDisable(GL_TEXTURE_2D);
	}					
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_COLOR_MATERIAL);
	glLineWidth(2.0);


	if(showRegularElemOn)
		display_tenRegElem(mode);

	if(showGridOn)
		display_design_grid();

	if(showSingularitiesOn)
		display_degenerate_pts(mode);

    if (showMajorTenLine)
        display_major_tenlines(mode);

    display_valid_trisectors(mode);
    display_valid_degenerate_pts(mode);
    display_bresenham_line();

    display_separatrices(mode);
    display_trisectorVec(mode);

	glPopMatrix ();
	glDisable(GL_COLOR_MATERIAL);
  	glEnable(GL_TEXTURE_2D);
    //glutSwapBuffers();
    glFlush();
	return true;	
}

void TfCore::render_majorfield_quad()
{

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity(); 
	gluOrtho2D(0, 1, 0, 1);

	major_vis_quad();

	glPopMatrix();

    render_tensor_final();

	glDisable(GL_TEXTURE_2D);
}



void TfCore::render_tensor_final()
{
    /*use the mesh to display the texture instead */
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
        GL_RGB, GL_UNSIGNED_BYTE, major_tex);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0);  glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0);  glVertex2f(1.0, 1.0);
	glEnd();
}

void TfCore::major_vis_quad()
{
    /*reset the view point here */
    glViewport(0, 0, (GLsizei)NPIX, (GLsizei)NPIX);

	/*rendering the positive x direction*/
	glBindTexture(GL_TEXTURE_2D, tentextnames[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGB, GL_UNSIGNED_BYTE, major_tex1);

    render_ibfv_tens_quad(false);

	/*save image*/
	glDisable(GL_BLEND);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex1);

	/***************************************************************/

	/*rendering the positive y direction*/
	major_iframe--;
	glBindTexture(GL_TEXTURE_2D, tentextnames[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGB, GL_UNSIGNED_BYTE, major_tex2);

    render_ibfv_tens_quad(true);

	glDisable(GL_BLEND);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex2);
	/***************************************************************/

	/*blend them*/

	int i, j;
	for(i=0; i<NPIX; i++) /*y direction*/
		for(j=0; j<NPIX; j++)
		{
			major_temp[i][j][0] = major_tex1[i][j][0];
			major_temp[i][j][1] = major_tex1[i][j][1];
			major_temp[i][j][2] = major_tex1[i][j][2];
			major_temp[i][j][3] = major_alpha_map[i][j][0];
		}

    for(i=0; i<NPIX; i++) /*x direction*/
        for(j=0; j<NPIX; j++)
        {
            minor_temp[i][j][0] = major_tex2[i][j][0];
            minor_temp[i][j][1] = major_tex2[i][j][1];
            minor_temp[i][j][2] = major_tex2[i][j][2];
            minor_temp[i][j][3] = 255-major_alpha_map[i][j][0];
        }
    glEnable(GL_BLEND);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, NPIX, NPIX, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, major_temp);
    render_tensor_blend();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, NPIX, NPIX, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, minor_temp);
    render_tensor_blend();

    ////

    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex);

    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGBA, GL_UNSIGNED_BYTE, major_temp);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGBA, GL_UNSIGNED_BYTE, minor_temp);

    glViewport(0, 0, (GLsizei)REALWINSIZE, (GLsizei)REALWINSIZE);
}
void TfCore::render_ibfv_tens_quad(bool x_y)
{
	int i, j;
	QuadCell *face;
	QuadVertex *vert;
	double px, py;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity(); 
	gluOrtho2D(0, 1, 0, 1);
    if(!x_y) /*positive x*/
    {
        for (i=0; i<quadmesh->nfaces; i++) {
            face = quadmesh->quadcells[i];
            glBegin(GL_POLYGON);
            for (j=0; j<face->nverts; j++) {
                vert = quadmesh->quad_verts[face->verts[j]];
                glTexCoord2f(vert->x, vert->y);

                if(vert->major_cos)
                {
                    px = vert->x - vert->major.entry[0];
                    py = vert->y - vert->major.entry[1];
                }
                else
                {
                    px = vert->x + vert->major.entry[0];
                    py = vert->y + vert->major.entry[1];
                }

                glVertex2f(px, py);
            }
            glEnd();
        }
    }
    else  /*positive y*/
    {
        for (i=0; i<quadmesh->nfaces; i++) {
            face = quadmesh->quadcells[i];
            glBegin(GL_POLYGON);
            for (j=0; j<face->nverts; j++) {
                vert = quadmesh->quad_verts[face->verts[j]];
                glTexCoord2f(vert->x, vert->y);
                if(vert->major_sin)
                {
                    px = vert->x - vert->major.entry[0];
                    py = vert->y - vert->major.entry[1];
                }
                else
                {
                    px = vert->x + vert->major.entry[0];
                    py = vert->y + vert->major.entry[1];
                }
                glVertex2f(px, py);
            }
            glEnd();
        }
    }
    major_iframe ++;

	glEnable(GL_BLEND); 

	/*double check the following codes*/
    glCallList(major_iframe % Npat + 200+ 1);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,  0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0,  ten_tmax); glVertex2f(0.0, 1.0);
	glTexCoord2f(ten_tmax, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(ten_tmax, ten_tmax); glVertex2f(1.0, 1.0);
	glEnd();

	glPopMatrix();

}
void TfCore::render_tensor_blend()
{
	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0);  glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0);  glVertex2f(1.0, 1.0);
	glEnd();
}
void TfCore::drawSolidRect_size(double cx, double cy, double R)
{
	glBegin(GL_POLYGON);
	glVertex2f(cx-R, cy-R);
	glVertex2f(cx-R, cy+R);
	glVertex2f(cx+R, cy+R);
	glVertex2f(cx+R, cy-R);
	glEnd();
}
void TfCore::drawSolidCircle_size(double cx, double cy, double R)
{
	int i;
	double theta, deta ;
	deta = 2 * M_PI/49.;
	double x, y;
	theta = 0.;
	glBegin(GL_POLYGON);
	for(i = 0; i < 50; i++, theta += deta)
	{
		x =  cx + R * cos(theta);
		y =  cy + R * sin(theta);

		glVertex2f(x, y);
	}
	glEnd();
}

////display the regular elements using arrows
void TfCore::display_tenRegElem(GLenum mode)
{
    glLineWidth(1.);
	for(int i = 0; i < nten_regelems; i++)
	{
		if(ten_regularelems[i].ID>0)
		{
			////Display the arrow
			if(mode == GL_SELECT)
				glLoadName(ten_regularelems[i].ID);

			if(ten_regularelems[i].deleted)
				continue;

			////perform the user defined transformation for editing

			if(ten_regularelems[i].type == 0) ////basic regular element
				glColor3f(0, 1, 1);
			else if(ten_regularelems[i].type == 1) ////convergent element
				glColor3f(1, 0.5, 0);
			else                         ////divergent element
				glColor3f(0, 1, 0.5);

			/*we draw two line segments*/
			glBegin(GL_LINES);
			glVertex2f(ten_regularelems[i].base[0], ten_regularelems[i].base[1]);
			glVertex2f(ten_regularelems[i].end[0], ten_regularelems[i].end[1]);
			glEnd();

			glBegin(GL_LINES);
			glVertex2f(ten_regularelems[i].base[0], ten_regularelems[i].base[1]);
			glVertex2f(ten_regularelems[i].base[0]-ten_regularelems[i].Direct.entry[0], 
				ten_regularelems[i].base[1]-ten_regularelems[i].Direct.entry[1]);
			glEnd();

		}
	}
}

void TfCore::display_trisectorVec(GLenum mode)
{
    glLineWidth(1.5);
    for(int i = 0; i < validDegpts.size(); i++)
    {
        if(validDegpts[i].type==1)
        {
            for(int j=0;j<validDegpts[i].nseps;j++){
                if(j == 0)
                    glColor3f(1, 0, 0);
                else if(j == 1)
                    glColor3f(204.0/255, 102.0/255, 0);
                else                         ////divergent element
                    glColor3f(0, 0, 1);
                /*we draw two line segments*/
                glBegin(GL_LINES);
                glVertex2f(validDegpts[i].gcx, validDegpts[i].gcy);
                glVertex2f(validDegpts[i].gcx+ validDegpts[i].s_vec[j].entry[0]*0.015, validDegpts[i].gcy+ validDegpts[i].s_vec[j].entry[1]*0.015);
                glEnd();

            }
        }
    }
}

void TfCore::display_design_grid()
{
	glColor3f(0.8, 0.8, 0.5);
	glLineWidth(1.);

	QuadCell* face;
	QuadVertex *v;
	int j;
	for(int i=0;i<quadmesh->nfaces;i++)
	{
		face=quadmesh->quadcells[i];

		glBegin(GL_LINE_LOOP);
		for(j=0;j<face->nverts;j++)
		{
			v=quadmesh->quad_verts[face->verts[j]];
			glVertex2f(v->x, v->y);
		}
		glEnd();
	}
}

void TfCore::display_degenerate_pts(GLenum mode)
{
	int i, singular_id = 0;

	for(i = 0; i < ndegpts; i++)
	{
		if(degpts[i].type == 0) /*wedge*/
			glColor3f(1, 0, 0);
		else if(degpts[i].type == 1) /*trisector*/
			glColor3f(0, 1, 0);
		else if(degpts[i].type == 2) /*node*/
			glColor3f(1, 1, 0);
		else if(degpts[i].type == 3) /*center*/
			glColor3f(1, 0, 1);
		else if(degpts[i].type == 4) /*saddle*/
			glColor3f(0, 0, 1);
		else
			glColor3f(1, 1, 1);

		drawSolidCircle_size(degpts[i].gcx, degpts[i].gcy, 0.006/zoom_factor);

		glColor3f(0, 0, 0);
		glLineWidth(1.4);
		draw_hollow_circle_size(degpts[i].gcx, degpts[i].gcy, 0.0065/zoom_factor);
	}
    glFlush();
}

void TfCore::draw_hollow_circle_size(double cx, double cy, double R)
{
	int i;
	double theta, deta ;
	deta = 2 * M_PI/49.;
	double x, y;
	theta = 0.;
	glBegin(GL_LINE_LOOP);
	for(i = 0; i < 50; i++, theta += deta)
	{
		x =  cx + R * cos(theta);
		y =  cy + R * sin(theta);

		glVertex2f(x, y);
	}
	glEnd();
}

int TfCore::DrawGLScene(GLenum mode)					// Here's Where We Do All The Drawing
{
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glDisable(GL_LIGHTING);
	without_anti_aliasing(mode);
    publish_image();
	return 1;
}

void TfCore::set_ten_regBasis(double x, double y, int type)
{
	////Add to the regular elements list
	if(nten_regelems >= curMaxNumTenRegElems-1 )
	{
		//Allocate new space for the element list
		curMaxNumTenRegElems += 50;
		ten_regularelems = (TenRegularElem*)realloc(ten_regularelems, sizeof(TenRegularElem) * curMaxNumTenRegElems);
		if(ten_regularelems == NULL)
			exit(-1);

	}
	ten_regularelems[nten_regelems].end[0] = ten_regularelems[nten_regelems].base[0] = x;
	ten_regularelems[nten_regelems].end[1] = ten_regularelems[nten_regelems].base[1] = y;
    ten_regularelems[nten_regelems].ID = nten_regelems+1;
	ten_regularelems[nten_regelems].type = type;
	ten_regularelems[nten_regelems].Direct.set(0, 0);

	////Initialize the transformation parameters for this regular element
	ten_regularelems[nten_regelems].transform_matrix.setIdentity();
	//ten_regularelems[nten_regelems].transposeRot.setIdentity();

	ten_regularelems[nten_regelems].rotang = 0;
	ten_regularelems[nten_regelems].s = 1;

    ten_regularelems[nten_regelems].deleted=false;

    /*mark which region the design element belongs to*/
	ten_regularelems[nten_regelems].which_region=get_region_id(x, y);

	nten_regelems ++;
}


unsigned char TfCore::get_region_id(double x, double y)
{
	int i=(x-quadmesh->xstart)/quadmesh->xinterval;
	int j=(y-quadmesh->ystart)/quadmesh->yinterval;

	if(i>=quadmesh->XDIM-1) i=quadmesh->XDIM-2;
	if(j>=quadmesh->YDIM-1) j=quadmesh->YDIM-2;

	int cellid=j*(quadmesh->XDIM-1)+i;
	return quadmesh->quadcells[cellid]->which_region;
}

void TfCore::set_ten_regDir(double x, double y)
{
	////Update the Cur_regularID here

	ten_regularelems[nten_regelems - 1].end[0] = x ;
	ten_regularelems[nten_regelems - 1].end[1] = y ;

	ten_regularelems[nten_regelems - 1].Direct.entry[0] = x - ten_regularelems[nten_regelems - 1].base[0];
	ten_regularelems[nten_regelems - 1].Direct.entry[1] = y - ten_regularelems[nten_regelems - 1].base[1];

	ten_regularelems[nten_regelems - 1].rotang = 
		atan2(ten_regularelems[nten_regelems - 1].Direct.entry[1],
		ten_regularelems[nten_regelems - 1].Direct.entry[0]);
}

void TfCore::set_ten_regDir(double x, double y,icVector2 &vec)
{
    ////Update the Cur_regularID here
    ten_regularelems[nten_regelems - 1].end[0] = x+vec.entry[0];
    ten_regularelems[nten_regelems - 1].end[1] = y+vec.entry[1] ;

    ten_regularelems[nten_regelems - 1].Direct.entry[0] =vec.entry[0];
    ten_regularelems[nten_regelems - 1].Direct.entry[1] = vec.entry[1];

    ten_regularelems[nten_regelems - 1].rotang =
        atan2(ten_regularelems[nten_regelems - 1].Direct.entry[1],
        ten_regularelems[nten_regelems - 1].Direct.entry[0]);
}


void TfCore::cal_tensorvals_quad()
{
	int i;
	QuadVertex *v;
	double t[4]={0.};

	for(i=0; i<quadmesh->nverts; i++)
	{
		v = quadmesh->quad_verts[i];

		if(!v->inland)
			continue;
        get_tensor(v->x, v->y, t);
        v->Jacobian.entry[0][0] = t[0];
        v->Jacobian.entry[0][1] = t[1];
        v->Jacobian.entry[1][0] = t[2];
        v->Jacobian.entry[1][1] = t[3];
	}
}


void TfCore::get_tensor(double x, double y, double t[4])
{
	int i;
	double  dx, dy, vx, vy, t00, t01, t10, t11, r=0.;
	double d;

	icMatrix3x3 tempJacobian, transposerot;
	double ang;

	vx = vy = 0.;

	t[0]=t[1]=t[2]=t[3]=0.;

	///Combine all the degenerate elements 
	for(i = 0; i < ntenelems; i++)
	{
		if(ten_designelems[i].ID >= 0 && !ten_designelems[i].deleted)
		{
			dx = x - ten_designelems[i].centerx;
			dy = y - ten_designelems[i].centery;

			r  = dx*dx + dy*dy; 
			d = exp(-1000*r);

			if (r < DistanceThreshold)   r = DistanceThreshold;

			tempJacobian.set(ten_designelems[i].transform_matrix);

			ang = ten_designelems[i].rotang;

			transposerot.set(cos(ang), sin(ang), 0,
				-sin(ang),  cos(ang), 0,
				0,0,1);

			tempJacobian.rightMultiply(transposerot);

			if(ten_designelems[i].type == 0) /*wedge*/
			{
				t00 = (dx * tempJacobian.entry[0][0] + dy * tempJacobian.entry[0][1])/r;  
				t01 = (dy * tempJacobian.entry[0][0] - dx * tempJacobian.entry[0][1])/r;
				t10 = (dx * tempJacobian.entry[1][0] + dy * tempJacobian.entry[1][1])/r;
				t11 = (dy * tempJacobian.entry[1][0] - dx * tempJacobian.entry[1][1])/r;
			}
			else if(ten_designelems[i].type == 1) /*trisector*/
			{
				t00 = (dx * tempJacobian.entry[0][0] - dy * tempJacobian.entry[0][1])/r;  
				t01 = (-dy * tempJacobian.entry[0][0] - dx * tempJacobian.entry[0][1])/r;
				t10 = (dx * tempJacobian.entry[1][0] - dy * tempJacobian.entry[1][1])/r;
				t11 = (-dy * tempJacobian.entry[1][0] - dx * tempJacobian.entry[1][1])/r;
			}
			else if(ten_designelems[i].type == 2) /*node*/
			{
				t00 = ((dx*dx-dy*dy)*tempJacobian.entry[0][0]+2*dx*dy*tempJacobian.entry[0][1])/(r*sqrt(r));
				t01 = (2*dx*dy*tempJacobian.entry[0][0]-(dx*dx-dy*dy)*tempJacobian.entry[0][1])/(r*sqrt(r));
				t10 = ((dx*dx-dy*dy)*tempJacobian.entry[1][0]+2*dx*dy*tempJacobian.entry[1][1])/(r*sqrt(r));
				t11 = (2*dx*dy*tempJacobian.entry[1][0]-(dx*dx-dy*dy)*tempJacobian.entry[1][1])/(r*sqrt(r));

			}
			else if(ten_designelems[i].type == 3) /*center*/
			{
				t00 = ((dy*dy-dx*dx)*tempJacobian.entry[0][0]-2*dx*dy*tempJacobian.entry[0][1])/(r*sqrt(r));
				t01 = (-2*dx*dy*tempJacobian.entry[0][0]+(dx*dx-dy*dy)*tempJacobian.entry[0][1])/(r*sqrt(r));
				t10 = ((dy*dy-dx*dx)*tempJacobian.entry[1][0]-2*dx*dy*tempJacobian.entry[1][1])/(r*sqrt(r));
				t11 = (-2*dx*dy*tempJacobian.entry[1][0]+(dx*dx-dy*dy)*tempJacobian.entry[1][1])/(r*sqrt(r));
			}
			else if(ten_designelems[i].type == 4) /*saddle*/
			{
				t00 = ((dx*dx-dy*dy)*tempJacobian.entry[0][0]-2*dx*dy*tempJacobian.entry[0][1])/(r*sqrt(r));
				t01 = (-2*dx*dy*tempJacobian.entry[0][0]-(dx*dx-dy*dy)*tempJacobian.entry[0][1])/(r*sqrt(r));
				t10 = ((dx*dx-dy*dy)*tempJacobian.entry[1][0]-2*dx*dy*tempJacobian.entry[1][1])/(r*sqrt(r));
				t11 = (-2*dx*dy*tempJacobian.entry[1][0]-(dx*dx-dy*dy)*tempJacobian.entry[1][1])/(r*sqrt(r));
			}

			/*you may also need to multiply the transpose of the transformation matrix*/


			t[0] += t00/sqrt(r)*LOWER;
			t[1] += t01/sqrt(r)*LOWER;
			t[2] += t10/sqrt(r)*LOWER;
			t[3] += t11/sqrt(r)*LOWER;

		}

	}

	/*the following we combine the regular element*/
    //icMatrix3x3 regten, temp;
	for(i = 0; i < nten_regelems; i++)
	{
		if(ten_regularelems[i].ID > 0 && !ten_regularelems[i].deleted)
		{
			dx = x - ten_regularelems[i].base[0];
			dy = y - ten_regularelems[i].base[1];

			r  = dx*dx + dy*dy; 

			if (r < DistanceThreshold)   r = DistanceThreshold;

			if(ten_regularelems[i].type == 0) ////regular element
			{
                /*the creation of regular element before */
				double strength = length(ten_regularelems[i].Direct);

				t[0] += strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[1] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[2] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[3] += -strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));

			}

		}

		else if(ten_regularelems[i].ID > 0 && 
			!ten_regularelems[i].deleted && ten_regularelems[i].which_region == 0)
		{
			dx = x - ten_regularelems[i].base[0];
			dy = y - ten_regularelems[i].base[1];

			r  = dx*dx + dy*dy; 

			if (r < DistanceThreshold)   r = DistanceThreshold;

			if(ten_regularelems[i].type == 0) ////regular element
			{
                /*the creation of regular element before */
				double strength = length(ten_regularelems[i].Direct);

				t[0] += strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[1] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[2] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[3] += -strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));

			}
		}
	}

}

void TfCore::init_ten_designelems()
{
    /*initialize degenerate design element*/
	if(ten_designelems == NULL)
	{
		ten_designelems=(Degenerate_Design *)malloc(sizeof(Degenerate_Design)*curMaxNumTenDesignElems);
		if(ten_designelems == NULL)
			exit(-1);
	}
	ntenelems = 0;

	init_regular_ten_designelems();
}

void TfCore::cal_all_eigenvecs_quad()
{
    int i;

    for(i=0; i<quadmesh->nverts; i++)
    {
        if(quadmesh->quad_verts[i]->inland)
            cal_eigenvecs_onevert_quad(i);
    }

	/*normalize the major and minor field*/
	normalized_tensorfield_quad();
}

void TfCore::cal_eigenvecs_onevert_quad(int ver)
{
	QuadVertex *v=quadmesh->quad_verts[ver];
	icVector2 ev[2];

	if(fabs(v->Jacobian.entry[0][0])<=1.e-7
		&&fabs(v->Jacobian.entry[0][1])<=1.e-7
		&&fabs(v->Jacobian.entry[1][0])<=1.e-7
		&&fabs(v->Jacobian.entry[1][1])<=1.e-7)
	{
		v->major.set(0,0);
		v->minor.set(0,0);
		v->major_ang=0;
		v->minor_ang=0;
		return;
	}

	cal_eigen_vector_sym(v->Jacobian, ev);

	v->major = ev[0];
	v->minor = ev[1];

	/*compute the angle*/
	v->major_ang = atan2(v->major.entry[1], v->major.entry[0]);
	v->minor_ang = atan2(v->minor.entry[1], v->minor.entry[0]);

	/*save the angle of major field for degenerate points/singularity detection*/
    v->tensor_major_ang = v->major_ang;
		
	/*obtain the multiplied vector, we will use this obtained
	vector field to extract singularities*/
	v->tran_vec.set(cos(2*v->tensor_major_ang), sin(2*v->tensor_major_ang));


	/*transfer to cos^2*/
	double major_ang_cos = cos(v->major_ang);
	double major_ang_sin = sin(v->major_ang);

	v->major_ang = major_ang_cos*major_ang_cos;
	if(major_ang_cos<0)
		v->major_cos = true;
	else
		v->major_cos = false;
	if(major_ang_sin<0)
		v->major_sin = true;
	else
		v->major_sin = false;

	double minor_ang_cos = cos(v->minor_ang);
	double minor_ang_sin = sin(v->minor_ang);

	v->minor_ang = minor_ang_cos*minor_ang_cos;
	//v->minor_ang = minor_ang_sin*minor_ang_sin;
	if(minor_ang_cos<0)
		v->minor_cos = true;
	else
		v->minor_cos = false;
	if(minor_ang_sin<0)
		v->minor_sin = true;
	else
		v->minor_sin = false;

}

void TfCore::cal_eigen_vector_sym(icMatrix2x2 m, icVector2 ev[2])
{
	/*first get the deviator of m*/
	icMatrix2x2 dev;
	double half_trace = 0.5*(m.entry[0][0]+m.entry[1][1]);
	dev.entry[0][0] = m.entry[0][0]-half_trace;
	dev.entry[1][1] = m.entry[1][1]-half_trace;
	dev.entry[0][1] = m.entry[0][1];
	dev.entry[1][0] = m.entry[1][0];

	/*compute the eigen vectors*/
	double theta = atan2(dev.entry[0][1], dev.entry[0][0]);

	//if(theta < 0) theta += 2*M_PI;

	/*major eigen vector*/
	ev[0].entry[0] = cos(theta/2.);
	ev[0].entry[1] = sin(theta/2.);

	//ev[0] = half_trace*ev[0];

	/*minor eigen vector*/
	ev[1].entry[0] = cos((theta+M_PI)/2.);
	ev[1].entry[1] = sin((theta+M_PI)/2.);

	//ev[1] = half_trace*ev[1];
}

void TfCore::normalized_tensorfield_quad()
{
	/*normalize the major and minor field*/
	int i;
	double r;
	QuadVertex *cur_v;

	for(i = 0; i < quadmesh->nverts; i++)
	{
		cur_v = quadmesh->quad_verts[i];

		/*normalize major field*/
		r = length(cur_v->major);
		r *= r;
		if (r < DistanceThreshold) 
		{
			r = DistanceThreshold;
			cur_v->major *= ten_dmax/r; 
		}
		r = length(cur_v->major);
		r *= r;
		if (r > ten_dmax*ten_dmax) { 
			r  = sqrt(r); 
			cur_v->major *= ten_dmax/r; 
		}

		/*normalize minor field*/
		r = length(cur_v->minor);
		r *= r;
		if (r < DistanceThreshold) 
		{
			r = DistanceThreshold;
			cur_v->minor *= ten_dmax/r; 
		}
		r = length(cur_v->minor);
		r *= r;
		if (r > ten_dmax*ten_dmax) { 
			r  = sqrt(r); 
			cur_v->minor *= ten_dmax/r; 
		}
	}
}

void TfCore::render_alpha_map_quad()
{
	/**/
	int i, j;
	QuadCell *face;
	QuadVertex *vert;

	glViewport(0, 0, (GLsizei)NPIX, (GLsizei)NPIX);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glShadeModel(GL_SMOOTH);
    for (i=0; i<quadmesh->nfaces; i++) {
        face = quadmesh->quadcells[i];
        glBegin(GL_POLYGON);
        for (j=0; j<face->nverts; j++) {
            vert = quadmesh->quad_verts[face->verts[j]];
            glColor3f(vert->major_ang,
                vert->major_ang,
                vert->major_ang);

            glVertex2f(vert->x, vert->y);
        }
        glEnd();
    }

    /*copy to the major_alpha_map*/
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_alpha_map);
	glDisable(GL_BLEND);
    glViewport(0, 0, (GLsizei)REALWINSIZE, (GLsizei)REALWINSIZE);
}

/*we use quad mesh to locate the degenerate points*/
void TfCore::locate_degpts_cells_tranvec_quad(void)
{
    //std::cout<<"befor locate degpts points, num is :"<<ndegpts<<std::endl;
    unsigned int i, j;
    QuadCell *face;
    QuadVertex *v;
    icVector2 vec[4];      //vectors at four vertices
    double  theta[4];      //for storing the angles between two vector for Gauss circle judgement

    double  vec_ang[4];  //the angle for each vector under the polar frame of current triangle
    double  ang_sum;

    ////Initialize
    ndegenerate_tris = 0;

    if(degenerate_tris == NULL)
        degenerate_tris = (int*) malloc(sizeof(int) * MaxNumDegeneratePts); //default range is 200


    ////Calculate the Poincare index
    for (i = 0; i < quadmesh->nfaces; i++) {
        face = quadmesh->quadcells[i];

        ang_sum = 0;

        for(j=0; j<face->nverts; j++)
        {
            v = quadmesh->quad_verts[face->verts[j]];
            vec_ang[j] = atan2(v->tran_vec.entry[1], v->tran_vec.entry[0]);
            if(vec_ang[j] < 0) vec_ang[j] += 2 * M_PI;
        }

        for(j = 0; j < face->nverts; j++)
        {
            theta[j] = vec_ang[(j+1)%face->nverts] - vec_ang[j];

            if( theta[j] < -M_PI)
                theta[j] += 2 * M_PI;

            if( theta[j] > M_PI)
                theta[j] -= 2 * M_PI;

            ang_sum += theta[j];
        }

        double index = ang_sum/(2*M_PI);


        /*here we need to allow some numerical errors 09/26/2007*/
        if(fabs(index) >= 1.- 1.e-1)
        {
            //The triangle must have singularities inside, mark it as yellow color
            //Still need to judge whether it is one of current singularities or not
            degenerate_tris[ndegenerate_tris] = i;
            ndegenerate_tris ++;

            if(fabs(index-1)<1e-2)  /*it is a wedge*/
                quadmesh->quadcells[i]->degenerate_type = 0;
            else if(fabs(index+1)<1e-2)  /*it is a trisector*/
                quadmesh->quadcells[i]->degenerate_type = 1;
            else if(fabs(index-2)<1e-2)    /*it is a node/center*/
                quadmesh->quadcells[i]->degenerate_type = 2;
            else if(fabs(index+2)<1e-2)    /*it is a saddle*/
                quadmesh->quadcells[i]->degenerate_type = 3;

            if(ndegenerate_tris >= MaxNumDegeneratePts - 1)
            {
                MaxNumDegeneratePts += 50;
                degenerate_tris = (int*) realloc(degenerate_tris, sizeof(int) * MaxNumDegeneratePts);
                degpts = (DegeneratePt*)realloc(degpts, sizeof(DegeneratePt)*MaxNumDegeneratePts);
            }
        }
    }

    if(ndegenerate_tris>0) /*we find some degenerate triangles*/
        compute_degpts_pos_tranvec_quad();
}


void TfCore::compute_degpts_pos_tranvec_quad()
{
    int i;
    double x_cp, y_cp;
    ndegpts = 0;

    for(i=0; i<ndegenerate_tris; i++)
    {
        compute_onedegpt_pos_tranvec_quad(degenerate_tris[i], x_cp, y_cp);

        /*save the information to the degenerate point list*/
        degpts[ndegpts].gcx = x_cp;
        degpts[ndegpts].gcy = y_cp;
        degpts[ndegpts].degpt_index = ndegpts;
        degpts[ndegpts].type = quadmesh->quadcells[degenerate_tris[i]]->degenerate_type;
        degpts[ndegpts].Triangle_ID = degenerate_tris[i];
        quadmesh->quadcells[degenerate_tris[i]]->degpt_index = ndegpts;

        /*compute the separatrices*/
        degpts[ndegpts].nseps = 0;
        degpts[ndegpts].nlinks = 0;
        ndegpts++;
    }

}
void TfCore::compute_onedegpt_pos_tranvec_quad(int cellid, double &x, double &y)
{
	/*first, we find an a along x direction, such that with this coefficient
	the interpolated vectors between v0v1 and v2v3 have opposite direction*/

	/*second, on this a, we find an b along y direction, such that the
	magnitude of v0v1 equal the magnitude of v2v3*/
	QuadCell *qc = quadmesh->quadcells[cellid];


	QuadVertex *v00 = quadmesh->quad_verts[qc->verts[0]];
	QuadVertex *v01 = quadmesh->quad_verts[qc->verts[3]];
	QuadVertex *v10 = quadmesh->quad_verts[qc->verts[1]];
	QuadVertex *v11 = quadmesh->quad_verts[qc->verts[2]];

	icVector2 v0v1, v2v3;
	double a, b;

	/*get a: the most difficult step*/
	compute_a_alongx_degptlocate(a, v00->tran_vec, v10->tran_vec, v11->tran_vec, v01->tran_vec,
		v0v1, v2v3);

	/*get b. after first step, v0v1 and v2v3 are opposite to each other*/
	if(fabs(v0v1.entry[0])>1e-8) /*use x direction*/
	{
		b = (v0v1.entry[0])/(v0v1.entry[0]-v2v3.entry[0]);
	}
	else /*use y direction*/
	{
		b = (v0v1.entry[1])/(v0v1.entry[1]-v2v3.entry[1]);
	}
    if(a<0||a>1) a=0.5;
    if(b<0 ||b>1)b=0.5;
	/*obtain the position*/
    x = bilinear_interpolate(a, b, v00->x, v01->x, v10->x, v11->x);
    y = bilinear_interpolate(a, b, v00->y, v01->y, v10->y, v11->y);
}


void TfCore::compute_a_alongx_degptlocate(double &a, icVector2 v0, icVector2 v1, icVector2 v2, icVector2 v3,
								  icVector2 &v0v1, icVector2 &v2v3)
{
	/*use binary search*/
	/*initialization*/
	double a0, a1;
	bool orient = false;  //false -- CCW, true -- CW
	a0=0.; a1=1.;
	a = 0.5;
	v0v1 = 0.5*(v0+v1);
	v2v3 = 0.5*(v3+v2);
	double theta1, theta2, theta;
	double theta_v0, theta_v1, theta_v2, theta_v3, theta_v03;
	theta_v0 = atan2(v0.entry[1], v0.entry[0]);
	//if(theta_v0<0) theta_v0 += 2*M_PI;
	//theta_v1 = atan2(v1.entry[1], v1.entry[0]);
	//if(theta_v1<0) theta_v1 += 2*M_PI;
	//theta_v2 = atan2(v2.entry[1], v2.entry[0]);
	//if(theta_v2<0) theta_v2 += 2*M_PI;
	theta_v3 = atan2(v3.entry[1], v3.entry[0]);
	//if(theta_v3<0) theta_v3 += 2*M_PI;

	theta_v03 = theta_v3-theta_v0;
	if(theta_v03>=0)
		orient = false; //CCW
	else
		orient = true;
	if(theta_v03<-M_PI) orient = false; //CCW
	if(theta_v03>M_PI) orient = true;   //CW

	/*NOTE: we interpolate from v0 to v1
	                       from v3 to v2 */
	//normalize(v0v1);
	//normalize(v2v3);
	theta1 = atan2(v0v1.entry[1], v0v1.entry[0]); //obtain angle for v0v1;
	//if(theta1<0) theta1 += 2*M_PI;
	theta2 = atan2(v2v3.entry[1], v2v3.entry[0]); //obtain angle for v2v3;
	//if(theta2<0) theta2 += 2*M_PI;
	/*subtract the two angles*/
	theta = theta1-theta2;

	bool s_orient = false;
	while(fabs(fabs(theta)-M_PI)>1e-8 && fabs(a0-a1)>1e-9) /*if they are not opposite to each other*/
	{
		if(theta>=0)
			s_orient = false;                   //CCW
		else
			s_orient = true;                    //CW

		if(theta>M_PI) s_orient = true;         //CW
		if(theta<-M_PI) s_orient = false;       //CCW

		/*if they have the same orientation*/
		if((orient&&s_orient) || (!orient&&!s_orient))
		{
			/*we need to increase a*/
			a1 = a;
			a = (a0+a1)/2.;
		}
		else
		{
			/*we need to decrease a*/
			a0 = a;
			a = (a0+a1)/2.;
		}

		/*recalculate v0v1 and v2v3*/
		v0v1 = (1-a)*v0+a*v1;
		v2v3 = (1-a)*v3+a*v2;

		/*recompute the angles of v0v1 and v2v3*/
		theta1 = atan2(v0v1.entry[1], v0v1.entry[0]); //obtain angle for v0v1;
		if(theta1<0) theta1 += 2*M_PI;
		theta2 = atan2(v2v3.entry[1], v2v3.entry[0]); //obtain angle for v2v3;
		if(theta2<0) theta2 += 2*M_PI;
		/*subtract the two angles*/
		theta = theta1-theta2;

	}
	
	v0v1 = (1-a)*v0+a*v1;
	v2v3 = (1-a)*v3+a*v2;
}

void TfCore::compute_separatrixVec_degpt_quad(int degpt_index)
{
    QuadCell *face = quadmesh->quadcells[validDegpts[degpt_index].Triangle_ID];
    QuadVertex *v;
    icMatrix2x2 dv[3]; // the deviators of the tensors at the three vertices

    int i,start;
    icVector3 point0(quadmesh->quad_verts[face->verts[0]]->x,quadmesh->quad_verts[face->verts[0]]->y,0);
    icVector3 point1(quadmesh->quad_verts[face->verts[1]]->x,quadmesh->quad_verts[face->verts[1]]->y,0);
    icVector3 point2(quadmesh->quad_verts[face->verts[2]]->x,quadmesh->quad_verts[face->verts[2]]->y,0);
    icVector3 degpt_point(validDegpts[degpt_index].gcx,validDegpts[degpt_index].gcy,0);
    if (PointInTriangle(point0,point1,point2,degpt_point)) start=0;
    else start=1;
    /*obtain the deviators*/
    for(i=start; i<3+start; i++)
    {
        dv[i-start].set(0.);
        v=quadmesh->quad_verts[face->verts[i]];
        double half_trace = 0.5*(v->Jacobian.entry[0][0]+v->Jacobian.entry[1][1]);
        dv[i-start].entry[0][0] = v->Jacobian.entry[0][0]-half_trace;
        dv[i-start].entry[1][1] = v->Jacobian.entry[1][1]-half_trace;
        dv[i-start].entry[0][1] = v->Jacobian.entry[0][1];
        dv[i-start].entry[1][0] = v->Jacobian.entry[1][0];
    }

    /*translate the tensor system according to the center of the degenerate point*/
    //for(i=0; i<3; i++)
    //{
    //}

    double a, b, c, d, e, f; //
    double x[3], y[3]; //
    double vx[3], vy[3];//

    for (i=start; i<3+start; i++) {
        v=quadmesh->quad_verts[face->verts[i]];
        x[i-start] = v->x;
        y[i-start] = v->y;

        ////using the vectors stored in the vertices

        /* Use normalized vector field*/
        vx[i-start] = dv[i-start].entry[0][0];
        vy[i-start] = dv[i-start].entry[0][1];

    }

    /////Calculate the jacobian of this linear system
    double coord[3][3],  *inver_coord ;  //inver_coord[3][3];
    for(int k = 0; k < 3; k++)
    {
        coord[0][k] = x[k];
        coord[1][k] = y[k];
        coord[2][k] = 1.;
    }

    inver_coord = MatrixOpp((double*)coord, 3, 3);

    icMatrix3x3 result, rightM;
    result.set(vx[0],vx[1],vx[2],  vy[0],vy[1],vy[2],  1,1,1);
    rightM.set(inver_coord[0], inver_coord[1], inver_coord[2],
        inver_coord[3], inver_coord[4], inver_coord[5],
        inver_coord[6], inver_coord[7], inver_coord[8]);


    result.rightMultiply(rightM);

    a = result.entry[0][0];
    b = result.entry[0][1];
    c = result.entry[0][2];
    d = result.entry[1][0];
    e = result.entry[1][1];
    f = result.entry[1][2];

    /*compute the solution for a cubic equations*/
    double solutions[4] = {0.};
    //int nroots = solve_ten_cubic(e, (d+2*b), (2*a-e), -d, solutions);
    int nroots = solve_ten_cubic_3(e, (d+2*b), (2*a-e), -d, solutions);
    //std::cout<<"degpt type is: "<<degpts[degpt_index].type<<";  nroots value is: "<<nroots<<std::endl;
    if(nroots == 0)
        return;
    else if(nroots == 1 /*|| nroots == 4*/)
    {
        /*it is the separatrix of a wedge*/
        validDegpts[degpt_index].nseps = 1;
        validDegpts[degpt_index].s1_ang = atan(solutions[0]);
        validDegpts[degpt_index].s[0].entry[0] = cos(validDegpts[degpt_index].s1_ang);
        validDegpts[degpt_index].s[0].entry[1] = sin(validDegpts[degpt_index].s1_ang);
    }
    else if(nroots == 2|| nroots == 4)
    {
        /*they are the separatrices of a wedge*/
        validDegpts[degpt_index].nseps = 2;
        validDegpts[degpt_index].s1_ang = atan(solutions[0]);
        validDegpts[degpt_index].s[0].entry[0] = cos(validDegpts[degpt_index].s1_ang);
        validDegpts[degpt_index].s[0].entry[1] = sin(validDegpts[degpt_index].s1_ang);
       // validDegpts[degpt_index].s_ifLinks[0]=true;

        validDegpts[degpt_index].s2_ang = atan(solutions[1]);
        validDegpts[degpt_index].s[1].entry[0] = cos(validDegpts[degpt_index].s2_ang);
        validDegpts[degpt_index].s[1].entry[1] = sin(validDegpts[degpt_index].s2_ang);
    }
    else if(nroots == 3) /*they are the separatrices of a trisector*/
    {
        validDegpts[degpt_index].nseps = 3;
        validDegpts[degpt_index].s1_ang = atan(solutions[0]);
        validDegpts[degpt_index].s[0].entry[0] = cos(validDegpts[degpt_index].s1_ang);
        validDegpts[degpt_index].s[0].entry[1] = sin(validDegpts[degpt_index].s1_ang);
       // validDegpts[degpt_index].s_ifLinks[0]=true;

        validDegpts[degpt_index].s2_ang = atan(solutions[1]);
        validDegpts[degpt_index].s[1].entry[0] = cos(validDegpts[degpt_index].s2_ang);
        validDegpts[degpt_index].s[1].entry[1] = sin(validDegpts[degpt_index].s2_ang);
       // validDegpts[degpt_index].s_ifLinks[1]=true;

        validDegpts[degpt_index].s3_ang = atan(solutions[2]);
        validDegpts[degpt_index].s[2].entry[0] = cos(validDegpts[degpt_index].s3_ang);
        validDegpts[degpt_index].s[2].entry[1] = sin(validDegpts[degpt_index].s3_ang);
       // validDegpts[degpt_index].s_ifLinks[2]=true;

        if(validDegpts[degpt_index].type==0){
            int target1=0,target2=1;
            float cosAngle1= fabs(dot(validDegpts[degpt_index].s[0],validDegpts[degpt_index].s[1]));
            float cosAngle2= fabs(dot(validDegpts[degpt_index].s[0],validDegpts[degpt_index].s[2]));
            float cosAngle3= fabs(dot(validDegpts[degpt_index].s[1],validDegpts[degpt_index].s[2]));
            if(cosAngle1>cosAngle2){
                target1=0;
                target2=2;
                if(cosAngle2>cosAngle3)
                   target1=1;target2=2;
            }else{
               if(cosAngle1>cosAngle3)
                   target1=1;target2=2;
            }
             validDegpts[degpt_index].nseps = 2;
             validDegpts[degpt_index].s[0]=validDegpts[degpt_index].s[target1];
             validDegpts[degpt_index].s[1]=validDegpts[degpt_index].s[target2];
        }
    }
    for(int i=0;i<3;i++)
        validDegpts[degpt_index].s_ifLink[i]=true;
}


void TfCore::cal_separatrix_vec(){
    Seed *seed1=new Seed;
    Seed *seed2=new Seed;
    float factor=0.5*1e-2;
    for(int i=0; i<validDegpts.size(); i++){
        bool ifTriSepSim=false;
        int target1,target2,target3;
        double degpt_loc[2];
        degpt_loc[0]=validDegpts[i].gcx;
        degpt_loc[1]=validDegpts[i].gcy;
        if(validDegpts[i].type==1){
            if(fabs(dot(validDegpts[i].s[0],validDegpts[i].s[1]))>0.7){ifTriSepSim=true;target1=0;target2=1;target3=2;}
            else if(fabs(dot(validDegpts[i].s[0],validDegpts[i].s[2]))>0.7){ifTriSepSim=true;target1=0;target2=2;target3=1;}
            else if(fabs(dot(validDegpts[i].s[1],validDegpts[i].s[2]))>0.7){ifTriSepSim=true;target1=1;target2=2;target3=0;}
        }
        if(ifTriSepSim){
            seed1->pos[0]=validDegpts[i].gcx+ validDegpts[i].s[target3].entry[0]*factor;
            seed1->pos[1]=validDegpts[i].gcy+ validDegpts[i].s[target3].entry[1]*factor;
            seed1->triangle=separatrices->get_cellID_givencoords(seed1->pos[0],seed1->pos[1]);

            seed2->pos[0]=validDegpts[i].gcx - validDegpts[i].s[target3].entry[0]*factor;
            seed2->pos[1]=validDegpts[i].gcy - validDegpts[i].s[target3].entry[1]*factor;
            seed2->triangle=separatrices->get_cellID_givencoords(seed2->pos[0],seed2->pos[1]);

            icMatrix2x2 ten1,ten2;
            separatrices->compute_tensor_at_quad(seed1->triangle,seed1->pos[0], seed1->pos[1], ten1);
            separatrices->compute_tensor_at_quad(seed2->triangle,seed2->pos[0], seed2->pos[1], ten2);
            icVector2 ev1[2],ev2[2];
            cal_eigen_vector_sym(ten1, ev1);
            cal_eigen_vector_sym(ten2, ev2);
            double deta1=fabs(dot(ev1[0],validDegpts[i].s[target3]));
            double deta2=fabs(dot(ev2[0],validDegpts[i].s[target3]));
            if(deta1>deta2){
                validDegpts[i].s_vec[target3]=validDegpts[i].s[target3];
            }else{
                validDegpts[i].s_vec[target3]=-validDegpts[i].s[target3];
            }

            seed1->pos[0]=validDegpts[i].gcx+ validDegpts[i].s[target1].entry[0]*factor;
            seed1->pos[1]=validDegpts[i].gcy+ validDegpts[i].s[target1].entry[1]*factor;
            seed1->triangle=separatrices->get_cellID_givencoords(seed1->pos[0],seed1->pos[1]);

            seed2->pos[0]=validDegpts[i].gcx - validDegpts[i].s[target1].entry[0]*factor;
            seed2->pos[1]=validDegpts[i].gcy - validDegpts[i].s[target1].entry[1]*factor;
            seed2->triangle=separatrices->get_cellID_givencoords(seed2->pos[0],seed2->pos[1]);

            separatrices->compute_tensor_at_quad(seed1->triangle,seed1->pos[0], seed1->pos[1], ten1);
            separatrices->compute_tensor_at_quad(seed2->triangle,seed2->pos[0], seed2->pos[1], ten2);
            cal_eigen_vector_sym(ten1, ev1);
            cal_eigen_vector_sym(ten2, ev2);
            deta1=fabs(dot(ev1[0],validDegpts[i].s[target1]));
            deta2=fabs(dot(ev2[0],validDegpts[i].s[target1]));

            if(deta1>deta2){
                validDegpts[i].s_vec[target1]=validDegpts[i].s[target1];
                if(dot(validDegpts[i].s[target1],validDegpts[i].s[target2])>0)
                    validDegpts[i].s_vec[target2]=-validDegpts[i].s[target2];
                else
                    validDegpts[i].s_vec[target2]=validDegpts[i].s[target2];
            }else{
                validDegpts[i].s_vec[target1]=-validDegpts[i].s[target1];
                if(dot(validDegpts[i].s[target1],validDegpts[i].s[target2])>0)
                    validDegpts[i].s_vec[target2]=validDegpts[i].s[target2];
                else
                    validDegpts[i].s_vec[target2]=-validDegpts[i].s[target2];
            }
        }else{
            for(int j=0; j<validDegpts[i].nseps;j++)
            {
                seed1->pos[0]=validDegpts[i].gcx+ validDegpts[i].s[j].entry[0]*factor;
                seed1->pos[1]=validDegpts[i].gcy+ validDegpts[i].s[j].entry[1]*factor;
                seed1->triangle=separatrices->get_cellID_givencoords(seed1->pos[0],seed1->pos[1]);

                seed2->pos[0]=validDegpts[i].gcx - validDegpts[i].s[j].entry[0]*factor;
                seed2->pos[1]=validDegpts[i].gcy - validDegpts[i].s[j].entry[1]*factor;
                seed2->triangle=separatrices->get_cellID_givencoords(seed2->pos[0],seed2->pos[1]);
                icMatrix2x2 ten1,ten2;
                separatrices->compute_tensor_at_quad(seed1->triangle,seed1->pos[0], seed1->pos[1], ten1);
                separatrices->compute_tensor_at_quad(seed2->triangle,seed2->pos[0], seed2->pos[1], ten2);
                icVector2 ev1[2],ev2[2];
                cal_eigen_vector_sym(ten1, ev1);
                cal_eigen_vector_sym(ten2, ev2);
                double deta1=fabs(dot(ev1[0],validDegpts[i].s[j]));
                double deta2=fabs(dot(ev2[0],validDegpts[i].s[j]));
                if(deta1>deta2){
                    validDegpts[i].s_vec[j]=validDegpts[i].s[j];
                }else{
                     validDegpts[i].s_vec[j]=-validDegpts[i].s[j];
                }
            }
        }
    }
}





void TfCore::gen_separatrix(){
    reset_separatrices();
    double degpt_loc[2];
    double startpt[2];
    int startcell;
    int fieldtype;
    fieldtype=0;	/*major direction*/
    float factor=1e-2;
    for(int i=0; i<validDegpts.size(); i++)
    {
        if(validDegpts[i].type==1){
            for(int j=0; j<validDegpts[i].nseps;j++)
            {
                degpt_loc[0]=validDegpts[i].gcx;
                degpt_loc[1]=validDegpts[i].gcy;
                startpt[0]=validDegpts[i].gcx+ validDegpts[i].s_vec[j].entry[0]*factor;
                startpt[1]=validDegpts[i].gcy+ validDegpts[i].s_vec[j].entry[1]*factor;
                startcell=separatrices->get_cellID_givencoords(startpt[0],startpt[1]);
                separatrices->grow_a_separatrix(degpt_loc,startpt, startcell, separatrices->percentage_dsep*separatrices->dsep,
                    separatrices->discsize, separatrices->sample_interval, separatrices->loopdsep, separatrices->dist2sing,
                    separatrices->streamlinelength, fieldtype,validDegpts[i].s_vec[j],i,j);
            }
        }
    }

}

void TfCore::display_major_tenlines(GLenum mode)
{
    if(major_path == NULL)
        return;
    int i, j, k;
    Trajectory *cur_traj;
    glColor3f(1, 0, 0);
    glLineWidth(1.5);
    cur_traj = major_path->evenstreamlines->trajs[0];

    for(k=0; k<cur_traj->nlinesegs; k++)
    {
        glBegin(GL_LINES);
        glVertex2f(cur_traj->linesegs[k].gstart[0], cur_traj->linesegs[k].gstart[1]);
        glVertex2f(cur_traj->linesegs[k].gend[0], cur_traj->linesegs[k].gend[1]);
        glEnd();
    }
    glFlush();
}

void TfCore::display_separatrices(GLenum mode){
    if(separatrices == NULL)
        return;
    int i, j, k;
    Trajectory *cur_traj;
    glColor3f(0, 1, 0);
    glLineWidth(1.5);
    glShadeModel(GL_SMOOTH);
    for(j=0; j<separatrices->evenstreamlines->ntrajs; j++)
    {
        cur_traj = separatrices->evenstreamlines->trajs[j];
         glColor3f(1, 1, 0);
         drawSolidCircle_size(cur_traj->linesegs[0].gend[0], cur_traj->linesegs[0].gend[1], 0.003/zoom_factor);
         glColor3f(0, 1, 0);
        for(k=0; k<cur_traj->nlinesegs; k++)
        {
            glBegin(GL_LINES);
            glVertex2f(cur_traj->linesegs[k].gstart[0], cur_traj->linesegs[k].gstart[1]);
            glVertex2f(cur_traj->linesegs[k].gend[0], cur_traj->linesegs[k].gend[1]);
            glEnd();
        }
    }
    glFlush();
}

void TfCore::reset_major_path(){
    free(major_path);
    init_majorPath();
}

void TfCore::reset_separatrices(){
    free(separatrices);
    init_separatrices();
}

void TfCore::set_robot_loc(double x,double y){
    robot_loc->pos[0]=x;
    robot_loc->pos[1]=y;
}

void TfCore::gen_major_path(){
    listen_to_robot_loc();
    double startpt[2];
    int startcell;
    robot_loc->triangle=major_path->get_cellID_givencoords(robot_loc->pos[0],robot_loc->pos[1]);
    startcell=robot_loc->triangle;
    startpt[0]=robot_loc->pos[0];
    startpt[1]=robot_loc->pos[1];
    major_path->grow_a_majRoad(startpt, startcell, major_path->percentage_dsep*major_path->dsep,
        major_path->discsize, major_path->sample_interval, major_path->loopdsep, major_path->dist2sing,
        major_path->streamlinelength, 0,m_robotDirect);
}

void TfCore::listen_to_robot_loc(){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform(worldFrameId, baseFrameId,
                                  now, ros::Duration(3.0));
      listener.lookupTransform(worldFrameId, baseFrameId,
                               now, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    robot_world_pos[0]=transform.getOrigin().x();
    robot_world_pos[1]=transform.getOrigin().y();

    double loc_x=transform.getOrigin().x()/realWorld_to_field_scale+realWorld_to_field_offset_x;
    double loc_y=transform.getOrigin().y()/realWorld_to_field_scale+realWorld_to_field_offset_y;
    set_robot_loc(loc_x,loc_y);

    double x = transform.getRotation().getX();
    double y = transform.getRotation().getY();
    double z = transform.getRotation().getZ();
    double w = transform.getRotation().getW();
    double robot_angel = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    m_robotDirect.entry[0]=cos(robot_angel);
    m_robotDirect.entry[1]=sin(robot_angel);
}
//////////////////////////////////////////////////////////////////////////
void TfCore::add_separatrix_points_toRobot(Trajectory * target_traj){
    nav_msgs::Path path;
    path.header.frame_id=worldFrameId;
    Trajectory *cur_traj=target_traj;
    for (int i=0;i<cur_traj->nlinesegs;i++)
    {
         geometry_msgs::PoseStamped pose;
         pose.header.stamp = ros::Time::now();
         double x=(cur_traj->linesegs[i].gend[0]-realWorld_to_field_offset_x)*realWorld_to_field_scale;
         double y=(cur_traj->linesegs[i].gend[1]-realWorld_to_field_offset_y)*realWorld_to_field_scale;
         pose.pose.position.x=x;
         pose.pose.position.y=y;
         pose.header.frame_id=worldFrameId;
         pose.pose.position.z=0;
         pose.pose.orientation.x=0;
         pose.pose.orientation.y=0;
         pose.pose.orientation.z=0;
         pose.pose.orientation.w=1;
         path.poses.push_back(pose);
    }
    pure_pursuit_controller::executePath srv;
    srv.request.curPath=path;
    srv.request.ifFirstPoint=true;
    if(pathExecuteClient.call(srv)){
        ROS_INFO("Reach a trisector, request execute a branch");
        sleep(3);
    }
    else{
        ROS_ERROR("Failed to call service execute_path");
    }
}

void TfCore::add_robot_wayPoints()
{
    Trajectory *cur_traj=major_path->evenstreamlines->trajs[0];
    icVector2 tmp_path_dir(cur_traj->linesegs[6].gend[0]-cur_traj->linesegs[6].gstart[0],cur_traj->linesegs[6].gend[1]-cur_traj->linesegs[6].gstart[1]);
    normalize(tmp_path_dir);
    nav_msgs::Path path;
    path.header.frame_id=worldFrameId;
    for (int i=0;i<cur_traj->nlinesegs;i++)
    {
         geometry_msgs::PoseStamped pose;
         pose.header.stamp = ros::Time::now();
         double x=(cur_traj->linesegs[i].gend[0]-realWorld_to_field_offset_x)*realWorld_to_field_scale;
         double y=(cur_traj->linesegs[i].gend[1]-realWorld_to_field_offset_y)*realWorld_to_field_scale;
         pose.pose.position.x=x;
         pose.pose.position.y=y;
         pose.header.frame_id=worldFrameId;
         pose.pose.position.z=0;
         pose.pose.orientation.x=0;
         pose.pose.orientation.y=0;
         pose.pose.orientation.z=0;
         pose.pose.orientation.w=1;
         path.poses.push_back(pose);
    }
    pure_pursuit_controller::executePath srv;
    srv.request.curPath=path;
    if(fabs(dot(m_robotDirect,tmp_path_dir))<0.5)
        srv.request.ifFirstPoint=true;
    else
        srv.request.ifFirstPoint=false;
    if(pathExecuteClient.call(srv)){
        ROS_INFO("Request execute path");
    }
    else{
        ROS_ERROR("Failed to call service execute_path");
    }
}

void TfCore::cancelPath(){
    pure_pursuit_controller::cancelPath srv;
    srv.request.req="cancel";
    if(pathCancelClient.call(srv)){
        ROS_INFO("Request cancel path");
    }
    else{
        ROS_ERROR("Failed to call service cancel_path");
    }
}
void TfCore::init_recover_points(){
    recoverPoints.clear();
    for(int i=0;i<SAMPLES_RECOVER;i++)
    {
        cv::Point2f tmp;
        tmp.x=SAMPLES_RADIU*sin(i*2*M_PI/SAMPLES_RECOVER);
        tmp.y=SAMPLES_RADIU*cos(i*2*M_PI/SAMPLES_RECOVER);
        recoverPoints.push_back(tmp);
    }
}
void TfCore::recover_robot_state(){
    listen_to_robot_loc();
    double max_min_dist=0.0;
    int target_recoverPoint=0;
    for (int k=0;k<recoverPoints.size();k++){
        double cur_min_dist=std::numeric_limits<double>::max();
        for(int i=0;i<contoursInWorld.size();i++)
        {
            for (int j=0;j<contoursInWorld[i].size();j++)
            {
                double x_dist=recoverPoints[k].x+robot_world_pos[0]-contoursInWorld[i][j].x;
                double y_dist=recoverPoints[k].y+robot_world_pos[1]-contoursInWorld[i][j].y;
                double cur_dist=sqrt(x_dist*x_dist+y_dist*y_dist);
                if (cur_dist<cur_min_dist)
                    cur_min_dist=cur_dist;

            }
        }
        if (cur_min_dist>max_min_dist)
        {
           max_min_dist=cur_min_dist;
           target_recoverPoint=k;
        }
    }
    ROS_INFO("select reccover point %d!",target_recoverPoint);
    nav_msgs::Path path;
    path.header.frame_id=worldFrameId;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x=recoverPoints[target_recoverPoint].x+robot_world_pos[0];
    pose.pose.position.y=recoverPoints[target_recoverPoint].y+robot_world_pos[1];
    pose.header.frame_id=worldFrameId;
    pose.pose.position.z=0;
    pose.pose.orientation.x=0;//no meaning
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=0;
    pose.pose.orientation.w=1;
    path.poses.push_back(pose);
    pure_pursuit_controller::recoverPath srv;
    srv.request.recoverPath=path;
    if(pathRecoverClient.call(srv)){
        ROS_INFO("Request recover path");
        while(!ifFinish_recover){
            sleep(2);
            ros::spinOnce();
        }
        ifFinish_recover=false;
        recoverMode=false;
    }
    else{
        ROS_ERROR("Failed to call service recover_path");
    }
}

void TfCore::setRegularElems(){

    constraints_vecs.clear();
    for(int i=0;i<contours2Field.size();i++)
    {
        if (contours2Field[i].size()<10)
            continue;
        std::vector<icVector2> cur_vecs;
        for (int j=0;j<contours2Field[i].size()-1;j++)
        {
           std::vector<cv::Point2f> points;
           icVector2 tmp_dir(contours2Field[i][j+1].x-contours2Field[i][j].x,contours2Field[i][j+1].y-contours2Field[i][j].y);
           float cur_strength=length(tmp_dir);
           if(j<3){
               for(int k=0;k<5;k++)
                   points.push_back(cv::Point2f(contours2Field[i][k].x,contours2Field[i][k].y));
           }else if(j>contours2Field[i].size()-6){
               for(int k=contours2Field[i].size()-5;k<contours2Field[i].size();k++)
                   points.push_back(cv::Point2f(contours2Field[i][k].x,contours2Field[i][k].y));
           }else{
               for(int k=j-2;k<j+3;k++)
                   points.push_back(cv::Point2f(contours2Field[i][k].x,contours2Field[i][k].y));
           }
           cv::Vec4f line;
           cv::fitLine(cv::Mat(points), line, CV_DIST_L2, 0, 0.01, 0.01);
           icVector2 tmp_vec(line[0],line[1]);
           if(dot(tmp_vec,tmp_dir)<0) tmp_vec=-tmp_vec;
           tmp_vec=tmp_vec*cur_strength;
           cur_vecs.push_back(tmp_vec);
        }
        constraints_vecs.push_back(cur_vecs);
    }

    int count=0;
    for(int i=0;i<contours2Field.size();i++)
    {
        if (contours2Field[i].size()<10)
            continue;
        for (int j=0;j<contours2Field[i].size()-1;j++)
        {
            set_ten_regBasis(contours2Field[i][j].x, contours2Field[i][j].y,0);
            set_ten_regDir(contours2Field[i][j].x,contours2Field[i][j].y,constraints_vecs[count][j]);
        }
        count++;
    }

}

void TfCore::finishRecover_callback(const std_msgs::String &msg){
    ifFinish_recover=true;
}

void TfCore::finishTurn_callback(const std_msgs::String &msg){
    ifFinish_turn=true;
}

void TfCore::goTri_callback(const std_msgs::String &msg){
    ifFinish_goTri=true;
}
void TfCore::gridMap_callback(const nav_msgs::OccupancyGrid &msg){
    if(!isReceiveNewMap)
        isReceiveNewMap=true;
    dirMap.info=msg.info;
    dirMap.info.width=dirMap.info.width+2;
    dirMap.info.height=dirMap.info.height+2;
    dirMap.info.origin.position.x=dirMap.info.origin.position.x-dirMap.info.resolution*0.5;
    dirMap.info.origin.position.y=dirMap.info.origin.position.y-dirMap.info.resolution*0.5;
    dirMap.data.clear();
    for(int i=dirMap.info.height-1;i>=0;i--)
        for(int j=0; j<dirMap.info.width;j++){
            if(i>0 && i<dirMap.info.height-1 && j>0 && j<dirMap.info.width-1){
                if (msg.data[msg.info.width*(i-1)+j-1]>50)
                    dirMap.data.push_back(100);
                else
                    dirMap.data.push_back(0);
            }else
                dirMap.data.push_back(0);
        }
    for(int i=0;i<dirMap.info.height;i++){
        for(int j=0;j<dirMap.info.width;j++){
            if(dirMap.data[i*dirMap.info.width+j]==100)
            {
                int x_min=max(j-2,0); int x_max=min(dirMap.info.width-1,j+2);
                int y_min=max(i-2,0); int y_max=min(dirMap.info.height-1,i+2);
                for(int l=y_min;l<=y_max;l++){
                    for(int n=x_min;n<=x_max;n++)
                        dirMap.data[l*dirMap.info.width+n]==100;
                }
            }
        }
    }
    get_obstacles_contour();
    if(!recoverMode)
    {
        ensure_robot_safety();
        cut_robot_path();
    }
}

void TfCore::frontier_point_callback(const std_msgs::Float64MultiArray &msg){
    frontierPoints=msg;
}

void TfCore::set_obstacles(){
    reset_regular_and_degenrateElem();
    bresenham_line_points.clear();
    validTriDegpts.clear();
    ifCancelPath=false;
    setRegularElems();
    parallel_update_tensorField();
    init_degpts();
    render_alpha_map_quad();
    locate_degpts_cells_tranvec_quad();
    NUM_Slices+=interFrameNum;
    set_cur_as_keyframe(NUM_Slices);
    listen_to_robot_loc();
    if(enRelaxKeyframeOn)
      Laplace_relax_timeDep();
    if(major_path->evenstreamlines->ntrajs!=0 && ifReachTrisector && check_reach_trisector()){
        reset_major_path();
        gen_separatrix();
        cal_sep_infoGain();
        select_target_trisector_branch_move();
    }else{
        reset_separatrices();
        ifReachTrisector=false;
        reset_major_path();
        gen_major_path();
        cut_robot_path();
        if( major_path->evenstreamlines->trajs[0]->nlinesegs<10 || major_path->evenstreamlines->trajs[0]->get_length()*realWorld_to_field_scale<THRESHOLD_LENGTH)
        {

            curSliceID=keyframes->keyframes[keyframes->nkeyframes-1]->keyFrameID;
            update_tensor_field();
            if(check_reach_wedge())
                req_turn_service();
            else
                recoverMode=true;
            return;
        }
        add_robot_wayPoints();
    }
    curSliceID=keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID+1;
    play_all_frames();
}

void TfCore::play_all_frames()
{
    int t=0;
    while(t<interFrameNum)
    {
        ros::spinOnce();
        ifFinish_goTri=false;
        ifFinish_recover=false;
        update_tensor_field();
        filterDegpts();
        cal_separatrix_vec();
        if(!ifReachTrisector)
        {
            if(check_bypass_trisector())
                return;
        }
        if(ifReachTrisector)
        {
            gen_separatrix();
            cal_sep_infoGain();
            //usleep(0.2*1000000);
        }
        DrawGLScene(GL_RENDER);
        usleep(interFrameTime*1000000);
//        std::cout<<"t: "<<t<<std::endl;
        t++;
    }
}

bool TfCore::check_bypass_trisector(){
    validTriDegpts.clear();
    std::vector<DegeneratePt> tmp_validTriDegpts;
    bresenham_line_points.clear();
    std::vector<float> all_min_dists;
    std::vector<int > all_indexes;
    if(major_path->evenstreamlines->ntrajs==0) return false;
    Trajectory *cur_traj= major_path->evenstreamlines->trajs[0];
    for(int i=0;i<validDegpts.size();i++){
        if(validDegpts[i].type==1 && validDegpts[i].nseps==3){
            float cur_min_dist=std::numeric_limits<float>::max();
            int target_min_index=-1;
            for(int j=0;j<cur_traj->nlinesegs;j++){
                float deta_x=cur_traj->linesegs[j].gstart[0]-validDegpts[i].gcx;
                float deta_y=cur_traj->linesegs[j].gstart[1]-validDegpts[i].gcy;
                float dist=sqrt(pow(deta_x,2)+pow(deta_y,2));
                if(dist<cur_min_dist)
                {
                    target_min_index=j;
                    cur_min_dist=dist;
                }
            }
            if(target_min_index>6 && target_min_index<cur_traj->nlinesegs-6){
                //check sep_vec
                std::vector<cv::Point2f> in_points;
                for(int k=target_min_index-6;k<=target_min_index-4;k++)
                    in_points.push_back(cv::Point2f(cur_traj->linesegs[k].gstart[0],cur_traj->linesegs[k].gstart[1]));
                cv::Vec4f in_line;
                cv::fitLine(cv::Mat(in_points),in_line,CV_DIST_L2,0,0.01,0.01);
                icVector2 in_dir(in_line[0],in_line[1]);
                icVector2 tmp_in_dir(cur_traj->linesegs[target_min_index-6].gend[0]-cur_traj->linesegs[target_min_index-6].gstart[0],cur_traj->linesegs[target_min_index-6].gend[1]-cur_traj->linesegs[target_min_index-6].gstart[1]);
                if(dot(in_dir,tmp_in_dir)<0) in_dir=-in_dir;
                normalize(in_dir);

                std::vector<cv::Point2f> out_points;
                for(int k=target_min_index+4;k<=target_min_index+6;k++)
                    out_points.push_back(cv::Point2f(cur_traj->linesegs[k].gstart[0],cur_traj->linesegs[k].gstart[1]));
                cv::Vec4f out_line;
                cv::fitLine(cv::Mat(out_points),out_line,CV_DIST_L2,0,0.01,0.01);
                icVector2 out_dir(out_line[0],out_line[1]);
                icVector2 tmp_out_dir(cur_traj->linesegs[target_min_index+6].gend[0]-cur_traj->linesegs[target_min_index+6].gstart[0],cur_traj->linesegs[target_min_index+6].gend[1]-cur_traj->linesegs[target_min_index+6].gstart[1]);
                if(dot(out_dir,tmp_out_dir)<0) out_dir=-out_dir;
                normalize(out_dir);

                int target_in_sep=-1;
                int target_out_sep=-1;
                float cur_in_dir_dot_min=std::numeric_limits<float>::max();
                float cur_out_dir_dot_max=std::numeric_limits<float>::min();
                for(int k=0;k<validDegpts[i].nseps;k++){
                    if(dot(validDegpts[i].s_vec[k],in_dir)<cur_in_dir_dot_min){
                        cur_in_dir_dot_min=dot(validDegpts[i].s_vec[k],in_dir);
                        target_in_sep=k;
                    }
                    if(dot(validDegpts[i].s_vec[k],out_dir)>cur_out_dir_dot_max) {
                        cur_out_dir_dot_max=dot(validDegpts[i].s_vec[k],out_dir);
                        target_out_sep=k;
                    }
                }
                if(cur_out_dir_dot_max>0.7 && cur_in_dir_dot_min<-0.7 && target_in_sep!=target_out_sep && cur_min_dist*realWorld_to_field_scale<0.8){
                    tmp_validTriDegpts.push_back(validDegpts[i]);
                    all_min_dists.push_back(cur_min_dist);
                    all_indexes.push_back(i);
                }
            }
        }
    }
    if(tmp_validTriDegpts.size()>=1){
        auto smallest = std::min_element(std::begin(all_min_dists), std::end(all_min_dists));
        int smallest_index=std::distance(std::begin(all_min_dists),smallest);
        int target_tri_index=all_indexes[smallest_index];
        //check line insection with obstacles
        //construct line between robot loc and target triscector
        cv::Point2i target_tri_loc, robot_cur_loc;
        double target_tri_pos[2],robot_cur_pos[2];
        target_tri_pos[0]=validDegpts[target_tri_index].gcx;
        target_tri_pos[1]=validDegpts[target_tri_index].gcy;
        selected_target_tri=target_tri_index;
        locat_point_inMap(target_tri_pos,target_tri_loc);
        listen_to_robot_loc();
        robot_cur_pos[0]=robot_loc->pos[0];
        robot_cur_pos[1]=robot_loc->pos[1];

        locat_point_inMap(robot_cur_pos,robot_cur_loc);
        bresenham_line_points.clear();
        CalcBresenhamLocs(Location(target_tri_loc.x,target_tri_loc.y),Location(robot_cur_loc.x,robot_cur_loc.y),bresenham_line_points);
        if(check_line_insect_obstacle())
            bresenham_line_points.clear();
        else{
            validTriDegpts.push_back(validDegpts[target_tri_index]);
            nav_msgs::Path path;
            path.header.frame_id=worldFrameId;
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            if( bresenham_line_points[0].x<robot_cur_loc.x+2 && bresenham_line_points[0].x>robot_cur_loc.x-2&& bresenham_line_points[0].y<robot_cur_loc.y+2 && bresenham_line_points[0].y>robot_cur_loc.y-2)
            {
                std::vector<Location>::iterator cur_iter;
                for(cur_iter=bresenham_line_points.begin();cur_iter!=bresenham_line_points.end();cur_iter++){
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time::now();
                    double x=dirMap.info.resolution*((*cur_iter).y+0.5)+dirMap.info.origin.position.x;
                    double y=dirMap.info.resolution*(dirMap.info.height-1-(*cur_iter).x+0.5)+dirMap.info.origin.position.y;
                    pose.pose.position.x=x;
                    pose.pose.position.y=y;
                    pose.header.frame_id=worldFrameId;
                    pose.pose.position.z=0;
                    pose.pose.orientation.x=0;
                    pose.pose.orientation.y=0;
                    pose.pose.orientation.z=0;
                    pose.pose.orientation.w=1;
                    path.poses.push_back(pose);
                }

            }else{
                std::vector<Location>::reverse_iterator cur_iter;
                for(cur_iter=bresenham_line_points.rbegin();cur_iter!=bresenham_line_points.rend();cur_iter++){
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time::now();
                    double x=dirMap.info.resolution*((*cur_iter).y+0.5)+dirMap.info.origin.position.x;
                    double y=dirMap.info.resolution*(dirMap.info.height-1-(*cur_iter).x+0.5)+dirMap.info.origin.position.y;
                    pose.pose.position.x=x;
                    pose.pose.position.y=y;
                    pose.header.frame_id=worldFrameId;
                    pose.pose.position.z=0;
                    pose.pose.orientation.x=0;
                    pose.pose.orientation.y=0;
                    pose.pose.orientation.z=0;
                    pose.pose.orientation.w=1;
                    path.poses.push_back(pose);
                }

            }

            pure_pursuit_controller::executePath srv;
            srv.request.curPath=path;
            srv.request.ifFirstPoint=true;
            if(goTriExecuteclient.call(srv)){
                DrawGLScene(GL_RENDER);
                ROS_INFO("Request go to the trisectror");
                while(!ifFinish_goTri){
                    sleep(2);
                    ros::spinOnce();
                    if(ifCancelPath)
                        break;
                    //wait for go to trisector
                }
                ROS_INFO("go to the trisectror finish");
                bresenham_line_points.clear();
                ifFinish_goTri=false;
                ifReachTrisector=true;
                return true;
            }
            else{
                ROS_ERROR("Failed to call service go to the trisectror");
            }
         }
    }
    return false;
           // ifReachTrisector

}

void TfCore::draw_map_contour(){
    glColor3f(1, 0, 0);
    glLineWidth(1);
    for(int i=0;i<contours2Field.size();i++)
    {
        for (int j=0;j<contours2Field[i].size()-1;j++)
        {
            glBegin(GL_LINES);
            glVertex2f(contours2Field[i][j].x, contours2Field[i][j].y);
            glVertex2f(contours2Field[i][j+1].x, contours2Field[i][j+1].y);
            glEnd();
        }
    }

}

void TfCore::get_obstacles_contour(){
      cv::Mat img = cv::Mat::zeros(dirMap.info.height, dirMap.info.width, CV_8UC1);
      dirMap2Field.clear();
      dirMapInWorld.clear();
      for (unsigned i=0;i<dirMap.info.height;i++)
      {
          std::vector<cv::Point2f> tmp_world,tmp_field;
          for(unsigned j=0;j<dirMap.info.width;j++)
          {
              cv::Point2f tmp_world_point,tmp_field_point;
              tmp_world_point.x=dirMap.info.resolution*(j+0.5)+dirMap.info.origin.position.x;
              tmp_world_point.y=dirMap.info.resolution*(dirMap.info.height-1-i+0.5)+dirMap.info.origin.position.y;
              tmp_world.push_back(tmp_world_point);
              tmp_field_point.x=tmp_world_point.x/realWorld_to_field_scale+realWorld_to_field_offset_x;
              tmp_field_point.y=tmp_world_point.y/realWorld_to_field_scale+realWorld_to_field_offset_y;
              if(i==0 && j==dirMap.info.width-1){
                  rightTop[0]=tmp_field_point.x;
                  rightTop[1]=tmp_field_point.y;
              }
              tmp_field.push_back(tmp_field_point);
              if(dirMap.data[dirMap.info.width*i+j]==100)
                 img.at<uchar>(i,j)=255;
              else
                 img.at<uchar>(i,j)=0;
          }
          dirMapInWorld.push_back(tmp_world);
          dirMap2Field.push_back(tmp_field);
      }
      leftBottom[0]=dirMap.info.origin.position.x/realWorld_to_field_scale+realWorld_to_field_offset_x;
      leftBottom[1]=dirMap.info.origin.position.y/realWorld_to_field_scale+realWorld_to_field_offset_y;

      contours.clear();
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
      contours2Field.clear();
      contoursInWorld.clear();
      for( size_t k = 0; k < contours.size(); k++ )
      {
          std::vector<cv::Point2f> tmp_contour_to_field,tmp_contour_in_world;
          for(size_t i=0;i< contours[k].size();i++){
              tmp_contour_to_field.push_back(dirMap2Field[contours[k][i].y][contours[k][i].x]);
              tmp_contour_in_world.push_back(dirMapInWorld[contours[k][i].y][contours[k][i].x]);
          }
          contours2Field.push_back(tmp_contour_to_field);
          contoursInWorld.push_back(tmp_contour_in_world);
      }
}

void TfCore::cut_robot_path(){
    if(major_path->evenstreamlines->ntrajs==0) return;
    Trajectory *cur_traj = major_path->evenstreamlines->trajs[0];
    int Num=500;
    int maxNum=Num;
    if(maxNum>cur_traj->nlinesegs)
        maxNum=cur_traj->nlinesegs;
    for (int k=0;k<maxNum;k++){
        for(int i=0;i<contours2Field.size();i++)
        {
            for (int j=0;j<contours2Field[i].size();j++)
            {
                double x_dist=cur_traj->linesegs[k].gstart[0]-contours2Field[i][j].x;
                double y_dist=cur_traj->linesegs[k].gstart[1]-contours2Field[i][j].y;
                double cur_dist=sqrt(x_dist*x_dist+y_dist*y_dist);
                cur_dist=cur_dist*realWorld_to_field_scale;
                if (cur_dist<DANGERDIST)
                {
                    cur_traj->nlinesegs=k;
                    pure_pursuit_controller::cutPath srv;
                    srv.request.safeWayPoint=k;
                    if(pathCutClient.call(srv))
                    {
                        ROS_INFO("Request cut path");
                    }else{
                        ROS_ERROR("Failed to call service cut_path");
                    }
                    return;
                }
            }
        }
    }
}

void TfCore::cal_sep_infoGain(){
    degptsPathsInfoGain.clear();
    float *pointCollection=new float[frontierPoints.data.size()];
    for(int i=0;i<frontierPoints.data.size();i++)
        pointCollection[i]=frontierPoints.data[i];
    for(int i=0;i<separatrices->evenstreamlines->ntrajs;i++){
        Trajectory *cur_traj = separatrices->evenstreamlines->trajs[i];
        float *curPathPoints=new float[cur_traj->nlinesegs*2];
        for(int j=0;j<cur_traj->nlinesegs;j++){
            curPathPoints[2*j]=(cur_traj->linesegs[j].start[0]-realWorld_to_field_offset_x)*realWorld_to_field_scale;
            curPathPoints[2*j+1]=(cur_traj->linesegs[j].start[1]-realWorld_to_field_offset_x)*realWorld_to_field_scale;
        }
        int *curPathInfoGain_cu=new int[cur_traj->nlinesegs];
        calPathInfoGain(pointCollection, curPathPoints, curPathInfoGain_cu,frontierPoints.data.size()/3,cur_traj->nlinesegs);
        std::vector<int> curPathInfoGain;
        float averageInfoGain=0;
        int maxInfoGain= 0;
        int minInfoGain=std::numeric_limits<int>::max();
        for(int k=0;k<cur_traj->nlinesegs;k++){
            curPathInfoGain.push_back(curPathInfoGain_cu[k]);
            if(maxInfoGain< curPathInfoGain_cu[k])
                maxInfoGain=curPathInfoGain_cu[k];
            if(minInfoGain>curPathInfoGain_cu[k])
                minInfoGain=curPathInfoGain_cu[k];
            averageInfoGain+=curPathInfoGain_cu[k];
        }
        averageInfoGain/=cur_traj->nlinesegs;
        cur_traj->infoGain=maxInfoGain;
        degptsPathsInfoGain.push_back(curPathInfoGain);
        delete curPathPoints;
        delete curPathInfoGain_cu;
    }
    delete pointCollection;
}

bool TfCore::check_reach_trisector(){
    listen_to_robot_loc();
    if(validDegpts.size()==0) return false;
    float min_dist=std::numeric_limits<float>::max();
    for(int i=0;i<validDegpts.size();i++){
        if(validDegpts[i].type == 1){
            float deta_x=validDegpts[i].gcx-robot_loc->pos[0];
            float deta_y=validDegpts[i].gcy-robot_loc->pos[1];
            float dist=sqrt(deta_x*deta_x+deta_y*deta_y);
            if(dist<min_dist){
                targetTrisectorIndex=i;
                min_dist=dist;
            }
        }
    }
    if(min_dist<TRISECTOR_REACH_THRESHOLD){
        return true;
    }else{
        return false;
    }
}

void TfCore::ensure_robot_safety(){
    if(ifCancelPath) return;
    listen_to_robot_loc();
    double min_dist=std::numeric_limits<double>::max();
    for(int i=0;i<contours2Field.size();i++)
    {
        for (int j=0;j<contours2Field[i].size();j++)
        {
            double x_dist=robot_loc->pos[0]-contours2Field[i][j].x;
            double y_dist=robot_loc->pos[1]-contours2Field[i][j].y;
            double cur_dist=sqrt(x_dist*x_dist+y_dist*y_dist);
            cur_dist=cur_dist*realWorld_to_field_scale;
            if(min_dist>cur_dist)
                min_dist=cur_dist;
            if (cur_dist<safeDistance)
            {
                cancelPath();
                ifCancelPath=true;
                recoverMode=true;
                return;
            }
        }
    }
}
void TfCore::reset_regular_and_degenrateElem(){
    /*initialize regular design element*/
    free(ten_regularelems);
    ten_regularelems=NULL;
    if(ten_regularelems == NULL)
    {
        ten_regularelems=(TenRegularElem *)malloc(sizeof(TenRegularElem)*curMaxNumTenRegElems);
        if(ten_regularelems == NULL)
            exit(-1);
    }
    nten_regelems = 0;
    /*initialize singular design element*/
    free(ten_designelems);
    ten_designelems = NULL;
    if(ten_designelems == NULL)
    {
        ten_designelems=(Degenerate_Design *)malloc(sizeof(Degenerate_Design)*curMaxNumTenDesignElems);
        if(ten_designelems == NULL)
            exit(-1);
    }
    ntenelems = 0;
}

//topo connection

void TfCore::locat_point_inMap(double pos[2],cv::Point2i &mapLoc){
    //translate opengl point to real world point
    double x=(pos[0]-realWorld_to_field_offset_x)*realWorld_to_field_scale;
    double y=(pos[1]-realWorld_to_field_offset_y)*realWorld_to_field_scale;

    int lowerX = 0;
    int upperX = dirMap.info.width;
    do
    {
        int delim = (lowerX + upperX)/2;//change to shift later
        double tmp_x = dirMapInWorld[0][delim].x-dirMap.info.resolution*0.5;

        if(x < tmp_x)
            upperX = delim;
        else
            lowerX = delim;
    }
    while(upperX - lowerX > 1);
    mapLoc.y=lowerX;

    // for y-direction
    int lowerY = 0;
    int upperY = dirMap.info.height;

    do
    {
        int delim = (lowerY + upperY)/2;
        double tmp_y=dirMapInWorld[delim][0].y-dirMap.info.resolution*0.5;

        if(y>tmp_y)
            upperY = delim;

        else
            lowerY = delim;
    }
    while(upperY - lowerY > 1);
    mapLoc.x=lowerY;
}

void TfCore::filterDegpts(){
    if(ndegpts==0) return;
    validDegpts.clear();
    std::vector<DegeneratePt> tmp_validDegpts;
    for (int i=0;i<ndegpts; i++){
        double x=(degpts[i].gcx-realWorld_to_field_offset_x)*realWorld_to_field_scale;
        double y=(degpts[i].gcy-realWorld_to_field_offset_y)*realWorld_to_field_scale;
        if(x>=dirMapInWorld[0][dirMap.info.width-1].x-dirMap.info.resolution*0.5 || x<=dirMapInWorld[0][0].x-dirMap.info.resolution*0.5 ||
              y>=dirMapInWorld[0][0].y-dirMap.info.resolution*0.5 ||  y<=dirMapInWorld[dirMap.info.height-1][0].y-dirMap.info.resolution*0.5 ){
            continue;
        }
        cv::Point2i mapLoc;
        int region_num=8;
        double pos[2];
        pos[0]=degpts[i].gcx;
        pos[1]=degpts[i].gcy;
        locat_point_inMap(pos,mapLoc);
        int region_x_min,region_x_max,region_y_min,region_y_max;
        if(mapLoc.x-region_num>-1) region_y_min=mapLoc.x-region_num; else region_y_min=0;
        if(mapLoc.x+region_num<dirMap.info.height) region_y_max=mapLoc.x+region_num; else region_y_max=dirMap.info.height-1;
        if(mapLoc.y-region_num>-1) region_x_min=mapLoc.y-region_num; else region_x_min=0;
        if(mapLoc.y+region_num<dirMap.info.width) region_x_max=mapLoc.y+region_num; else region_x_max=dirMap.info.width-1;
        bool if_valid=true;
        for(int l=region_x_min;l<=region_x_max;l++)
            for(int m=region_y_min; m<=region_y_max; m++)
            {
                if(dirMap.data[l+dirMap.info.width*m]!=0)
                {
                    if_valid=false;
                    break;
                }

            }
         if(if_valid)
            tmp_validDegpts.push_back(degpts[i]);
   }
   std::set<int> deleteIndex;
   if(tmp_validDegpts.size()>1){
       for(int i=0;i<tmp_validDegpts.size();i++){
            for(int j=i+1;j<tmp_validDegpts.size();j++){
                if(deleteIndex.find(j)!=deleteIndex.end()) continue;
                float deta_x=tmp_validDegpts[i].gcx-tmp_validDegpts[j].gcx;
                float deta_y=tmp_validDegpts[i].gcy-tmp_validDegpts[j].gcy;
                if(sqrt(deta_x*deta_x+deta_y*deta_y)<2*1e-2){
                    if(fabs(tmp_validDegpts[i].type-tmp_validDegpts[j].type)!=1)continue;
                    deleteIndex.insert(i);
                    deleteIndex.insert(j);
                    break;
                }
            }
       }
   }
   for(int i=0;i<tmp_validDegpts.size();i++){
       if(deleteIndex.find(i)==deleteIndex.end())
           validDegpts.push_back(tmp_validDegpts[i]);
   }
   for(int i=0;i<validDegpts.size();i++){
      validDegpts[i].valid_index=i;
      compute_separatrixVec_degpt_quad(i);
   }
}


void TfCore::display_valid_trisectors(GLenum mode)
{
    if (validTriDegpts.size()==0) return;
    for(int i = 0; i < validTriDegpts.size(); i++)
    {
        glColor3f(0, 1, 0);

        drawSolidCircle_size(validTriDegpts[i].gcx, validTriDegpts[i].gcy, 0.008/zoom_factor);

        glColor3f(0, 0, 1);
        glLineWidth(1.4);
        draw_hollow_circle_size(validTriDegpts[i].gcx, validTriDegpts[i].gcy, 0.0085/zoom_factor);
    }
    glFlush();
}

void TfCore::display_valid_degenerate_pts(GLenum mode)
{
    if (validDegpts.size()==0) return;
    for(int i = 0; i < validDegpts.size(); i++)
    {
        //if (validDegpts[i].type==1) continue;
        if(validDegpts[i].type == 0) /*wedge*/
            glColor3f(1, 0, 0);
        else if(validDegpts[i].type == 1) /*trisector*/
            glColor3f(0, 1, 0);
        else
            glColor3f(1, 1, 1);

        drawSolidCircle_size(validDegpts[i].gcx, validDegpts[i].gcy, 0.0045/zoom_factor);

        glColor3f(0, 0, 0);
        glLineWidth(1.4);
        draw_hollow_circle_size(validDegpts[i].gcx, validDegpts[i].gcy, 0.005/zoom_factor);
    }
    glFlush();
}


//

void TfCore::get_laplace_TVTF(int which_slice){
    /*  We now save the read file into the vec variable of the mesh  */
    int curStep=which_slice-keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID;
    int cur_vertID;
    int variableID;
    for(int j=0; j<quadmesh->nverts; j++)
    {
        cur_vertID = curStep*quadmesh->nverts+j;
        QuadVertex *v=quadmesh->quad_verts[j];
        if(vertTimeList->vertsTime[cur_vertID]->isConstraint)
        {
            OneKeyFrame *thekeyframe = keyframes->get_frame_pointer(which_slice);
            v->Jacobian.entry[0][0] = thekeyframe->jacobian_vals[j].entry[0][0];
            v->Jacobian.entry[0][1] = thekeyframe->jacobian_vals[j].entry[0][1];
        }
        else
        {
            variableID=(curStep-1)*quadmesh->nverts+j;
            v->Jacobian.entry[0][0]  = vecsTime[variableID].entry[0];
            v->Jacobian.entry[0][1]  = vecsTime[variableID].entry[1];
        }
        double theta = atan2( v->Jacobian.entry[0][1], v->Jacobian.entry[0][0]);
        /*major eigen vector*/
        v->major.entry[0] = cos(theta/2.);
        v->major.entry[1] = sin(theta/2.);

        /*minor eigen vector*/
        v->minor.entry[0] = cos((theta+M_PI)/2.);
        v->minor.entry[1] = sin((theta+M_PI)/2.);

        v->major_ang = atan2(v->major.entry[1],v->major.entry[0]);
        v->minor_ang = atan2(v->minor.entry[1], v->minor.entry[0]);

        v->minor.entry[0]=cos(v->minor_ang);
        v->minor.entry[1]=sin(v->minor_ang);

        v->tensor_major_ang = v->major_ang;

        v->tran_vec.set(cos(2*v->tensor_major_ang), sin(2*v->tensor_major_ang));
        /*transfer to cos^2*/
        double major_ang_cos = cos(v->major_ang);
        double major_ang_sin = sin(v->major_ang);

        v->major_ang = major_ang_cos*major_ang_cos;
        if(major_ang_cos<0)
            v->major_cos = true;
        else
            v->major_cos = false;
        if(major_ang_sin<0)
            v->major_sin = true;
        else
            v->major_sin = false;

        double minor_ang_cos = cos(v->minor_ang);
        double minor_ang_sin = sin(v->minor_ang);

        v->minor_ang = minor_ang_cos*minor_ang_cos;
        //v->minor_ang = minor_ang_sin*minor_ang_sin;
        if(minor_ang_cos<0)
            v->minor_cos = true;
        else
            v->minor_cos = false;
        if(minor_ang_sin<0)
            v->minor_sin = true;
        else
            v->minor_sin = false;
    }
}

void TfCore::update_cur_TF_keyframe()
{
    if(keyframes == NULL) return;
    /*  if interpolation scheme is used to obtain the in-between slices  */
    if(!enRelaxKeyframeOn)
    {
        cal_TF_at_slice_keyframe_interp(curSliceID);
        cal_all_eigenvecs_quad();
    }
    else
    {
        //read_timeDep_TF_oneSlice_bin("keyframe_tempOut.bin", curSliceID);
        get_laplace_TVTF(curSliceID);
        normalized_tensorfield_quad();
    }
    curSliceID ++;

}

void TfCore::cal_TF_at_slice_keyframe_interp(int which_slice)
{
    /*   first, we need to judge which two neighboring key frames in the list containting this slice   */
    int i;
    bool is_keyframe = false;
    int keyframeID1, keyframeID2;
    double ratio = 0;

    keyframeID1 = keyframeID2 = -1;

    for(i=0; i<keyframes->nkeyframes; i++)
    {
        if(which_slice < keyframes->keyframes[i]->keyFrameID)
        {
            keyframeID1 = i-1;
            keyframeID2 = i;
            ratio = (double)(which_slice-keyframes->keyframes[i-1]->keyFrameID)/
                (double)(keyframes->keyframes[i]->keyFrameID-keyframes->keyframes[i-1]->keyFrameID);
            break;
        }
        else if(which_slice == keyframes->keyframes[i]->keyFrameID)
        {
            keyframeID1 = i;
            is_keyframe = true;
            break;
        }
    }

    if(i==keyframes->nkeyframes-1 && keyframeID1<0)
        keyframeID1 = keyframes->nkeyframes-1;

    /*  current computed frame is a key frame  */
    if(is_keyframe)
    {
        copy_from_keyframe(keyframeID1);
    }

    else
    {
        if(keyframeID2 < 0)
        {
            ROS_ERROR("out of range, cur slice id is %d",which_slice);
            /*   we may have to use relaxation to obtain the slices after the last key frame slice   */
        }

        else
        {
            cal_TF_at_inbetween_slice_keyframe_interp(ratio, keyframeID1, keyframeID2);
        }
    }
}

void TfCore::copy_from_keyframe(int keyframelistPos)
{
    int i;

    for(i=0; i<quadmesh->nverts; i++)
    {
        quadmesh->quad_verts[i]->Jacobian = keyframes->keyframes[keyframelistPos]->jacobian_vals[i];
    }

    /*  maybe normalize the field?  */
}

void TfCore::cal_TF_at_inbetween_slice_keyframe_interp(double ratio, int start, int end)
{
    /*
        We now assume only two keyframes available
    */
    int i;
    icMatrix2x2 vect, vec1, vec2;

    double ang0, ang1, ang2;  /*  for angle-based interpolation  */

    for(i=0; i<quadmesh->nverts; i++)
    {
        vec1 = keyframes->keyframes[start]->jacobian_vals[i];
        vec2 = keyframes->keyframes[end]->jacobian_vals[i];

        if(InterpScheme == 1)
        {
            vect = vec1+ratio*(vec2-vec1);
        }

        else if (InterpScheme == 2)  // Bezier
        {
            /*  make a quick test with #keyframes = 3  */
            icMatrix2x2 vec0;
            vec0 = keyframes->keyframes[0]->jacobian_vals[i];
            vec1 = keyframes->keyframes[1]->jacobian_vals[i];
            vec2 = keyframes->keyframes[2]->jacobian_vals[i];

            vect = (1-ratio)*((1-ratio)*vec0+ratio*vec1)
                +ratio*((1-ratio)*vec1+ratio*vec2);
        }

        else if (InterpScheme == 3)  // Try Hermite
        {
            icMatrix2x2 pseudo_vec1 = 0.5*(vec2-vec1);  //Catmull-Rom splines
            icMatrix2x2 pseudo_vec2;
            float s = ratio;    // scale s to go from 0 to 1
            float h1 =  2*s*s*s - 3*s*s*s + 1;          // calculate basis function 1
            float h2 = -2*s*s*s + 3*s*s;              // calculate basis function 2
            float h3 =   s*s*s - 2*s*s + s;         // calculate basis function 3
            float h4 =   s*s*s -  s*s;

            if (end+1 >= keyframes->nkeyframes)
                pseudo_vec2 = pseudo_vec1;
            else
            {
                pseudo_vec2 = 0.5*(keyframes->keyframes[end+1]->jacobian_vals[i]-vec2); //Catmull-Rom splines
            }
            vect = h1*vec1 +                    // multiply and sum all funtions
                     h2*vec2 +                    // together to build the interpolated
                     h3*pseudo_vec1 +                    // point along the curve.
                     h4*pseudo_vec2;
        }

        QuadVertex *v=quadmesh->quad_verts[i];
        v->Jacobian = vect;
    }
}


void TfCore::update_tensor_field(){
    update_cur_TF_keyframe();
    init_degpts();
    /*calculate the alpha map here*/
    render_alpha_map_quad();
    locate_degpts_cells_tranvec_quad();
}

void TfCore::init_keyframeList(int size){
    if(keyframes == NULL)
        keyframes = new KeyFrameList(size);
}

void TfCore::set_cur_as_keyframe(int frameID)
{
    keyframes->save_cur_field(frameID, quadmesh);
}

void TfCore::publish_image(){
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0,NPIX,NPIX, GL_RGB, GL_UNSIGNED_BYTE, rgb_im);
    cv_bridge::CvImage img_ptr_rgb;//=new cv_bridge::CvImage ;
    img_ptr_rgb.image=cv::Mat(NPIX,NPIX, CV_8UC3);
    cv::Mat& mat_rgb = img_ptr_rgb.image;

    cv::MatIterator_<cv::Vec3b> it=mat_rgb.begin<cv::Vec3b>();
    for (int i = NPIX-1;i >-1; i--)
    {
        for (int j = 0; j <  NPIX; j++)
        {
            (*it)[2]=rgb_im[i * NPIX*3 + j*3+0];  (*it)[1]=rgb_im[i * NPIX*3 + j*3+1]; (*it)[0]=rgb_im[i * NPIX*3 + j*3+2];
            ++it;
        }
    }
    sensor_msgs::ImagePtr msg_rgb=cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_rgb).toImageMsg();
    int count=0;
    while(count<3){
        rgb_pub.publish(msg_rgb);
        count++;
    }

}
//parallel
void TfCore::create_paraThread(){
    for (int i = 0; i < EXTRA_THREAD_NUM; i++)
    {
        ParaThread *myThread = new ParaThread();
        myThread->setID(i+1);
        myThread->connectTensorField(this);
        m_myThreads.push_back(myThread);
    }
}
void TfCore::parallel_update_tensorField(){
    int x1 = 0, x2 = 0;
    for (int i = 0; i < EXTRA_THREAD_NUM; i++)
    {
        split_tensorField(i+1, EXTRA_THREAD_NUM+1, x1, x2);
        m_myThreads[i]->setUpdateTensorFieldMission(x1, x2);
        m_myThreads[i]->start();
    }
    split_tensorField(0, EXTRA_THREAD_NUM+1, x1, x2);
    parallel_cal_tensorvals_quad(x1,x2);
    for (int i = 0; i < EXTRA_THREAD_NUM; i++)
        m_myThreads[i]->wait(); //

    normalized_tensorfield_quad();

}
void TfCore::split_tensorField( int i, int threadNum, int &x1, int &x2){
    int width = quadmesh->nverts/ threadNum;
    int last_remainder = quadmesh->nverts % width;
    x1 = i * width + (i<last_remainder%threadNum? i: last_remainder%threadNum);  //last_remainder < threadNum
    x2 = (i+1) * width + (i+1<last_remainder%threadNum? i+1: last_remainder%threadNum);
}
void TfCore::parallel_cal_tensorvals_quad(int &x1, int &x2){
    QuadVertex *v;
    double t[4]={0.};
    for(int i=x1; i<x2; i++)
    {
        v = quadmesh->quad_verts[i];
        get_tensor(v->x, v->y, t);
        v->Jacobian.entry[0][0] = t[0];
        v->Jacobian.entry[0][1] = t[1];
        v->Jacobian.entry[1][0] = t[2];
        v->Jacobian.entry[1][1] = t[3];
        cal_eigenvecs_onevert_quad(i);
    }
}
//
void TfCore::Laplace_relax_timeDep(){
    /*  initialize the list  */
    init_vertTimeList();
    add_verts_to_vertTimeList();
    set_constraints_keyframes();
    count_nonconstrained_verts();

    int num_nonconstrained_verts = vertTimeList->total_notconstraint_verts;

    if(num_nonconstrained_verts < 2)
    {
        fprintf(stderr, "Can not find enough inner vertices. \n");
        return;
    }

    int i;
    Vertex *cur_v;
    int NMAX=10*num_nonconstrained_verts;

    Vec_INT tempija(NMAX);
    Vec_DP tempsa(NMAX);

    DP err;
    Vec_DP b(num_nonconstrained_verts),/*bcmp(num_nonconstrained_verts),*/x(num_nonconstrained_verts);

    Mat_DP bound_vert(0.0, num_nonconstrained_verts, 2);  ////store the vectors that on the boundary vertices

    const int ITOL=1,ITMAX=500;
    const DP TOL=ERROR_LAPLACE;
    int iter;

    con_linear_Sys_spatioTemp(tempsa, tempija, bound_vert);

    Vec_INT ija(tempija[num_nonconstrained_verts]);
    Vec_DP sa(tempija[num_nonconstrained_verts]);

    for(i = 0; i < tempija[num_nonconstrained_verts]; i++)
    {
        ija[i] = tempija[i];
        sa[i] = tempsa[i];
    }

    ija_p = &ija;
    sa_p = &sa;

    /*   allocate temporary space to store the computed vector values   */
   vecsTime=new icVector2[num_nonconstrained_verts];


    ////Smoothing the x component of the vectors
    for (i = 0; i < vertTimeList->total_notconstraint_verts; i++)
    {
        x[i]=0.0;
        b[i]= bound_vert[i][0];
    }
    linbcg(b,x,ija_p,sa_p,ITOL,TOL,ITMAX,iter,err);

    ////Store the results back to the vertices
    for (i = 0; i < num_nonconstrained_verts; i++)
    {
        vecsTime[i].entry[0] = x[i];
    }

    ////Smoothing the y component of the vectors
    for (i = 0; i < vertTimeList->total_notconstraint_verts; i++)
    {
        x[i]=0.0;
        b[i]= bound_vert[i][1];
    }
    linbcg(b,x,ija_p,sa_p,ITOL,TOL,ITMAX,iter,err);

    ////Store the results back to the vertices
    for (i = 0; i < num_nonconstrained_verts; i++)
    {
        vecsTime[i].entry[1] = x[i];
    }
}

void TfCore::init_vertTimeList(){
    if(vertTimeList != NULL){
        delete vertTimeList;
        vertTimeList=NULL;
        if(vecsTime!= NULL){
            delete vecsTime;
            vecsTime=NULL;
        }
    }
    vertTimeList = new VertTimeList(quadmesh->nverts,quadmesh->nverts*(interFrameNum+1));
}
void TfCore::add_verts_to_vertTimeList(){
    int i, j;

    if(vertTimeList == NULL)
        init_vertTimeList();

    for(i=0; i<interFrameNum+1; i++)
    {
        for(j=0; j<quadmesh->nverts; j++)
            vertTimeList->add_new(i, j);
    }
}
void TfCore::set_constraints_keyframes(){
    int i, j;
    int cur_step;
    for(i=0; i<2;i++){
        if (i==0)
            cur_step=0;
        else
            cur_step=interFrameNum;
        for(j=0; j<quadmesh->nverts; j++)
            vertTimeList->set_constraint_at_vert(cur_step,j);
    }
}
void TfCore::count_nonconstrained_verts(){
    int i;
    vertTimeList->total_notconstraint_verts = 0;
    for(i=0; i<vertTimeList->nvertsTime; i++)
    {
        if(!vertTimeList->vertsTime[i]->isConstraint)
        {
            /*  assign the variable index to the non-constrained vertex  */
            vertTimeList->vertsTime[i]->variableID = vertTimeList->total_notconstraint_verts;
            /*  increase the number of the non-constrained vertices  */
            vertTimeList->total_notconstraint_verts ++;
        }
    }
}

void TfCore::con_linear_Sys_spatioTemp(Vec_DP &tsa, Vec_INT &tija, Mat_DP &bound_v){
    if(vertTimeList == NULL) return;

    int i, j;
    QuadVertex *adj_v, *cur_v = NULL;
    QuadEdge *adj_e;
    int *RegionIndex;
    int num_nonzeroarow = 0;
    int num_elements = 0;
    int cur_variableID = 0;
    unsigned char type = -1;
    double time_axis_weight = /*0.1*/30;

    /*    fill the diagnal elements first   */
    for(i = 0; i < vertTimeList->total_notconstraint_verts; i++)
        tsa[i] = 1. + time_axis_weight*2.;

    num_elements = vertTimeList->total_notconstraint_verts+1;

    /*   store other the remaining off-diagnal non-zero elements  */

    for(i = 0; i < quadmesh->nverts*(interFrameNum+1)/*vertTimeList->total_notconstraint_verts*/; i++)
    {
        if(vertTimeList->vertsTime[i]->isConstraint)  /* consider non-constrained verts only  */
            continue;

        cur_variableID = vertTimeList->vertsTime[i]->variableID;
        cur_v = quadmesh->quad_verts[vertTimeList->vertsTime[i]->which_vert];
        RegionIndex = new int[cur_v->Num_edge+2]; /*  we consider two more neighbors along time axis 09/19/2008  */

        int cur_time = vertTimeList->vertsTime[i]->time_step;

        num_nonzeroarow = 0;

        type = -1;

        for(j = 0; j < cur_v->Num_edge; j++)
        {
            adj_e = cur_v->edges[j];

            ////get the adjacent vertex on the other side of current edge
            if( quadmesh->quad_verts[adj_e->verts[0]] != cur_v)
                adj_v = quadmesh->quad_verts[adj_e->verts[0]];
            else
                adj_v = quadmesh->quad_verts[adj_e->verts[1]];

            if(vertTimeList->vertsTime[cur_time*quadmesh->nverts+adj_v->index]->isConstraint)
            {
                RegionIndex[j] = -1;

                OneKeyFrame *thekeyframe = keyframes->get_frame_pointer(cur_time+keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID);

                /*  need to obtain the key frame vector values  */
                if(thekeyframe != NULL)
                {
                    bound_v[cur_variableID][0] += (1./cur_v->Num_edge)*thekeyframe->jacobian_vals[adj_v->index].entry[0][0];
                    bound_v[cur_variableID][1] += (1./cur_v->Num_edge)*thekeyframe->jacobian_vals[adj_v->index].entry[0][1];
                }
            }
            else
            {
                RegionIndex[j] = vertTimeList->vertsTime[cur_time*quadmesh->nverts+adj_v->index]->variableID;
                num_nonzeroarow++;
            }
        }

        /*   consider the two neighbors along the time axis   */
        if(cur_time == 0)
        {
            RegionIndex[cur_v->Num_edge] = -1;

            if(vertTimeList->vertsTime[i+quadmesh->nverts]->isConstraint)
            {
                RegionIndex[cur_v->Num_edge+1] = -1;
                OneKeyFrame *thekeyframe = keyframes->get_frame_pointer(cur_time+1+keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID);
                bound_v[cur_variableID][0] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][0];
                bound_v[cur_variableID][1] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][1];
            }
            else
            {
                RegionIndex[cur_v->Num_edge+1] = vertTimeList->vertsTime[i+quadmesh->nverts]->variableID;
                num_nonzeroarow++;
                type = 0;
            }
        }
        else if(cur_time == interFrameNum+1-1)
        {
            RegionIndex[cur_v->Num_edge+1] = -1;

            if(vertTimeList->vertsTime[i-quadmesh->nverts]->isConstraint)
            {
                RegionIndex[cur_v->Num_edge] = -1;
                OneKeyFrame *thekeyframe = keyframes->get_frame_pointer(cur_time-1+keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID);
                bound_v[cur_variableID][0] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][0];
                bound_v[cur_variableID][1] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][1];
            }
            else
            {
                RegionIndex[cur_v->Num_edge] = vertTimeList->vertsTime[i-quadmesh->nverts]->variableID;
                num_nonzeroarow++;
                type = 1;
            }
        }
        else
        {

            if(vertTimeList->vertsTime[i-quadmesh->nverts]->isConstraint)
            {
                RegionIndex[cur_v->Num_edge] = -1;
                OneKeyFrame *thekeyframe = keyframes->get_frame_pointer(cur_time-1+keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID);
                bound_v[cur_variableID][0] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][0];
                bound_v[cur_variableID][1] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][1];
            }
            else
            {
                RegionIndex[cur_v->Num_edge] = vertTimeList->vertsTime[i-quadmesh->nverts]->variableID;
                num_nonzeroarow++;
                type = 0;
            }

            if(vertTimeList->vertsTime[i+quadmesh->nverts]->isConstraint)
            {
                RegionIndex[cur_v->Num_edge+1] = -1;
                OneKeyFrame *thekeyframe = keyframes->get_frame_pointer(cur_time+1+keyframes->keyframes[keyframes->nkeyframes-2]->keyFrameID);
                bound_v[cur_variableID][0] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][0];
                bound_v[cur_variableID][1] += /*(1./(cur_v->Num_edge+2))*/time_axis_weight*thekeyframe->jacobian_vals[cur_v->index].entry[0][1];
            }
            else
            {
                RegionIndex[cur_v->Num_edge+1] = vertTimeList->vertsTime[i+quadmesh->nverts]->variableID;
                num_nonzeroarow++;

                if(type == 0)
                    type = 2;
                else
                    type = 1;
            }
        }

        ////sorting the regionindex array
        BubbleSorting(RegionIndex, cur_v->Num_edge+2);

        ////move all non -1 element in the 'RegionIndex' to the front
        if(num_nonzeroarow < cur_v->Num_edge+2)
        {
            int firstnonfuyielement = cur_v->Num_edge+2 - num_nonzeroarow;

            for(j = 0; j < num_nonzeroarow; j++)
            {
                RegionIndex[j] = RegionIndex[firstnonfuyielement + j];
            }
        }

        ////Add elements to the corresponding positions of sa and ija array
        tija[cur_variableID] = num_elements;
        for(j = 0; j < num_nonzeroarow; j++)
        {
            if(type < 0)
                tsa[num_elements + j] = -1./cur_v->Num_edge;  //tsa[num_elements + j] = -SMOOTHSTEP/cur_v->Num_edge;
            else if(type == 0)   // the last element is the constraint along the time axis
            {
                if(j == 0)
                    tsa[num_elements+j] = -time_axis_weight;
                else
                    tsa[num_elements+j] = -1./cur_v->Num_edge;
            }
            else if(type == 1)
            {
                if(j == num_nonzeroarow - 1)    //  the first element is the constraint along the time axis
                    tsa[num_elements+j] = -time_axis_weight;
                else
                    tsa[num_elements+j] = -1./cur_v->Num_edge;
            }
            else if(type == 2)  // the first and last elements are both the constraints along the time axis
            {
                if(j == 0 || j == num_nonzeroarow - 1)    //  no constraint along the time axis
                    tsa[num_elements+j] = -time_axis_weight;
                else
                    tsa[num_elements+j] = -1./cur_v->Num_edge;
            }

            /*   we have to consider the weights for the neighbors along the time axis   */
            tija[num_elements + j] = RegionIndex[j];
        }

        num_elements += num_nonzeroarow;

        delete [] RegionIndex;
    }

    tsa[vertTimeList->total_notconstraint_verts] = 0.;
    tija[vertTimeList->total_notconstraint_verts] = num_elements-1;
}

void TfCore::display_bresenham_line(){
    if(bresenham_line_points.size()<2) return;
    glColor3f(0,0,1);
    for(int i=0;i<bresenham_line_points.size()-1;i++){
        glBegin(GL_LINES);
        double point1_x=(dirMap.info.resolution*(bresenham_line_points[i].y+0.5)+dirMap.info.origin.position.x)/realWorld_to_field_scale+realWorld_to_field_offset_x;
        double point1_y=(dirMap.info.resolution*(dirMap.info.height-1-bresenham_line_points[i].x+0.5)+dirMap.info.origin.position.y)/realWorld_to_field_scale+realWorld_to_field_offset_y;
        double point2_x=(dirMap.info.resolution*(bresenham_line_points[i+1].y+0.5)+dirMap.info.origin.position.x)/realWorld_to_field_scale+realWorld_to_field_offset_x;
        double point2_y=(dirMap.info.resolution*(dirMap.info.height-1-bresenham_line_points[i+1].x+0.5)+dirMap.info.origin.position.y)/realWorld_to_field_scale+realWorld_to_field_offset_y;
        glVertex2f(point1_x,point1_y);
        glVertex2f(point2_x,point2_y);
        glEnd();
    }

}
 bool TfCore::check_line_insect_obstacle(){
    for(int i=0; i<bresenham_line_points.size();i++){
        int region_num=5;
        int region_x_min,region_x_max,region_y_min,region_y_max;
        if(bresenham_line_points[i].x-region_num>-1) region_y_min=bresenham_line_points[i].x-region_num; else region_y_min=0;
        if(bresenham_line_points[i].x+region_num<dirMap.info.height) region_y_max=bresenham_line_points[i].x+region_num; else region_y_max=dirMap.info.height-1;
        if(bresenham_line_points[i].y-region_num>-1) region_x_min=bresenham_line_points[i].y-region_num; else region_x_min=0;
        if(bresenham_line_points[i].y+region_num<dirMap.info.width) region_x_max=bresenham_line_points[i].y+region_num; else region_x_max=dirMap.info.width-1;
        for(int l=region_x_min;l<=region_x_max;l++)
            for(int m=region_y_min; m<=region_y_max; m++)
            {
                if(dirMap.data[l+dirMap.info.width*m]!=0)
                    return true;
            }
    }

    return false;
 }

bool TfCore::insertLinks(int i, int j){
    pair<int,int> tmp_pair;
    set<pair<int,int>>::iterator tmp_iter;
    if(i<j){
        tmp_pair=make_pair(i,j);
        if(degptsLinks.find(tmp_pair)==degptsLinks.end())
        {
            degptsLinks.insert(tmp_pair);
            return true;
        }else
            return false;

    }else{
        tmp_pair=make_pair(j,i);
        if(degptsLinks.find(tmp_pair)==degptsLinks.end()){
            degptsLinks.insert(tmp_pair);
            return true;
        }else
            return false;
    }
}

std::map<pair<int ,int>,float>  TfCore::search_degpts_mst(std::set<int> connect_degpts,int target_tri_index){
    std::map<pair<int ,int>,float> cur_mst;
    if(connect_degpts.size()==1) return cur_mst;
    degptsLinks.clear();
    int num_seps=separatrices->evenstreamlines->ntrajs;
    std::map<pair<int,int>,float> linkValues;
    for(int i=0;i<num_seps;i++){
        Trajectory *cur_traj=separatrices->evenstreamlines->trajs[i];
        int degpt_id=cur_traj->degpt_id;
        int linked_degpt_id=cur_traj->linked_degpt_id;
        pair<int,int> tmp_pair;
        if(degpt_id==target_tri_index && connect_degpts.find(linked_degpt_id)!=connect_degpts.end()){
            if(degpt_id<linked_degpt_id){
                if(insertLinks(degpt_id,linked_degpt_id)){
                    tmp_pair=make_pair(degpt_id,linked_degpt_id);
                    linkValues.insert(pair<pair<int,int>,float>(tmp_pair,cur_traj->infoGain));
                }
            }else{
                if(insertLinks(linked_degpt_id,degpt_id)){
                    tmp_pair=make_pair(linked_degpt_id,degpt_id);
                    linkValues.insert(pair<pair<int,int>,float>(tmp_pair,cur_traj->infoGain));
                }
            }
        }
    }

    for(int i=0;i<num_seps;i++){
        Trajectory *cur_traj=separatrices->evenstreamlines->trajs[i];
        int degpt_id=cur_traj->degpt_id;
        if(degpt_id==target_tri_index) continue;
        int linked_degpt_id=cur_traj->linked_degpt_id;
        pair<int,int> tmp_pair;
        if(connect_degpts.find(degpt_id)!=connect_degpts.end() && connect_degpts.find(linked_degpt_id)!=connect_degpts.end()){
            if(degpt_id<linked_degpt_id){
                if(insertLinks(degpt_id,linked_degpt_id)){
                    tmp_pair=make_pair(degpt_id,linked_degpt_id);
                    linkValues.insert(pair<pair<int,int>,float>(tmp_pair,cur_traj->infoGain));
                }
            }else{
                if(insertLinks(linked_degpt_id,degpt_id)){
                    tmp_pair=make_pair(linked_degpt_id,degpt_id);
                    linkValues.insert(pair<pair<int,int>,float>(tmp_pair,cur_traj->infoGain));
                }
            }

        }
    }
    graph g_graph(connect_degpts.size());
    std::map<int,int> connect_degpts_index_map_forward;
    std::map<int,int> connect_degpts_index_map_backward;
    std::set<int>::iterator iter_set;
    int count=0;
    for(iter_set=connect_degpts.begin();iter_set!=connect_degpts.end();iter_set++){
        connect_degpts_index_map_forward.insert(pair<int,int>(*iter_set,count));
        connect_degpts_index_map_backward.insert(pair<int,int>(count,*iter_set));
        count++;
    }
    std::map<pair<int,int>,float>::iterator iter;
    for(iter=linkValues.begin();iter!=linkValues.end();iter++){
        g_graph.addEdge(connect_degpts_index_map_forward[(*iter).first.first],connect_degpts_index_map_forward[(*iter).first.second],(*iter).second);
        g_graph.addEdge(connect_degpts_index_map_forward[(*iter).first.second],connect_degpts_index_map_forward[(*iter).first.first],(*iter).second);
    }
    //std::unique_ptr<MSTStrategy> s_mst = std::make_unique<PrimeMST>();
    MSTStrategy *s_mst =new PrimeMST();
    std::vector<graph::Vertex> parent=s_mst->mst(g_graph);
    delete s_mst;

    for(int i=1;i<g_graph.vertexCount();i++){
        int link1=connect_degpts_index_map_backward[i];
        int link2=connect_degpts_index_map_backward[parent[i]];
        pair<int,int> tmp_pair;
        if(link1<link2)
        {
           tmp_pair=make_pair(link1,link2);
           cur_mst.insert(pair<pair<int,int>,float>(tmp_pair,g_graph[i][parent[i]]));
        }
        else
        {
           tmp_pair=make_pair(link2,link1);
           cur_mst.insert(pair<pair<int,int>,float>(tmp_pair,g_graph[i][parent[i]]));
        }
     }
    return cur_mst;
    //
    //
}

std::set<int> TfCore::find_connect_degpts(int target_tri_index){
    std::set<int> connect_degpts;
    connect_degpts.insert(target_tri_index);
    int num_seps=separatrices->evenstreamlines->ntrajs;
    for(int i=0;i<num_seps;i++){
        Trajectory *cur_traj=separatrices->evenstreamlines->trajs[i];
        if(cur_traj->linked_degpt_id=-1) continue;
        if(cur_traj->degpt_id==target_tri_index)
        {
            if(connect_degpts.find(cur_traj->linked_degpt_id)==connect_degpts.end())
                connect_degpts.insert(cur_traj->linked_degpt_id);
        }
    }
    if(connect_degpts.size()==1) return connect_degpts;

    while(true){
        int cur_connect_degpts_num=connect_degpts.size();
        for(int i=0;i<num_seps;i++){
            Trajectory *cur_traj=separatrices->evenstreamlines->trajs[i];
            if(cur_traj->linked_degpt_id==-1) continue;
            if(connect_degpts.find(cur_traj->degpt_id)!=connect_degpts.end()){
                if(connect_degpts.find(cur_traj->linked_degpt_id)==connect_degpts.end())
                {
                    connect_degpts.insert(cur_traj->linked_degpt_id);
                    break;
                }
            }
            if(connect_degpts.find(cur_traj->linked_degpt_id)!=connect_degpts.end()){
                if(connect_degpts.find(cur_traj->degpt_id)==connect_degpts.end()){
                    connect_degpts.insert(cur_traj->degpt_id);
                    break;
                }
            }
        }
        if(connect_degpts.size()==cur_connect_degpts_num)
            break;
    }
    return connect_degpts;
}


Trajectory * TfCore::select_a_branch(std::map<pair<int,int>,float> cur_mst, int target_tri_index){
    std::vector<Trajectory *> cur_trajs;
    std::vector<Trajectory *> tmp_cur_trajs;
    std::vector<float> info_gains;
    int num_seps=separatrices->evenstreamlines->ntrajs;
    bool reach_unkown=false;
    for(int i=0;i<num_seps;i++){
        Trajectory *cur_traj=separatrices->evenstreamlines->trajs[i];
        if(cur_traj->degpt_id==target_tri_index)
        {
            tmp_cur_trajs.push_back(cur_traj);
            if(cur_traj->is_reach_unknown)  reach_unkown=true;
        }
    }
    if(reach_unkown){
        for(int i=0;i<tmp_cur_trajs.size();i++){
            if(tmp_cur_trajs[i]->is_reach_unknown)
            {
               cur_trajs.push_back(tmp_cur_trajs[i]);
               info_gains.push_back(1.0/tmp_cur_trajs[i]->get_length());
            }

        }
    }else{
        for(int i=0;i<tmp_cur_trajs.size();i++)
            cur_trajs.push_back(tmp_cur_trajs[i]);
        if(cur_mst.size()==0){
            for(int i=0;i<cur_trajs.size();i++){
                info_gains.push_back(cur_trajs[i]->infoGain);
            }
        }else{
            for(int i=0;i<cur_trajs.size();i++){
                if(cur_trajs[i]->linked_degpt_id==-1)
                {
                    info_gains.push_back(cur_trajs[i]->infoGain);
                }
                else
                {

                    bool flag=false;
                    std::map<pair<int,int>,float>::iterator iter;
                    for(iter=cur_mst.begin();iter!=cur_mst.end();iter++)
                    {
                        if(((*iter).first.first==target_tri_index && (*iter).first.second==cur_trajs[i]->linked_degpt_id) || ((*iter).first.second==target_tri_index && (*iter).first.first==cur_trajs[i]->linked_degpt_id))
                            flag=true;
                    }
                    if(!flag){
                       info_gains.push_back(cur_trajs[i]->infoGain);
                    }else{
                        float cur_infoGain=cur_trajs[i]->infoGain;
                        int cur_count=1;
                        std::set<int> cur_tri_link_set;
                        cur_tri_link_set.insert(cur_trajs[i]->linked_degpt_id);
                        for(iter=cur_mst.begin();iter!=cur_mst.end();iter++){
                            if((*iter).first.first==cur_trajs[i]->linked_degpt_id && (*iter).first.second!=target_tri_index)
                                cur_tri_link_set.insert((*iter).first.second);
                            if((*iter).first.second==cur_trajs[i]->linked_degpt_id && (*iter).first.first!=target_tri_index)
                                cur_tri_link_set.insert((*iter).first.first);
                        }
                        std::set<int>::iterator iter_link=cur_tri_link_set.find(cur_trajs[i]->linked_degpt_id);
                        cur_tri_link_set.erase(iter_link);

                        while(true){
                            int cur_size=cur_tri_link_set.size();
                            for(iter=cur_mst.begin();iter!=cur_mst.end();iter++){
                                int first=(*iter).first.first;
                                int second=(*iter).first.second;
                                if(cur_tri_link_set.find(first)!=cur_tri_link_set.end() && cur_tri_link_set.find(second)==cur_tri_link_set.end())
                                {
                                    cur_tri_link_set.insert(second);
                                    break;
                                }
                                if(cur_tri_link_set.find(second)!=cur_tri_link_set.end() && cur_tri_link_set.find(first)==cur_tri_link_set.end())
                                {
                                    cur_tri_link_set.insert(first);
                                    break;
                                }
                            }
                            if(cur_size==cur_tri_link_set.size()) break;
                        }
                        iter_link=cur_tri_link_set.find(cur_trajs[i]->linked_degpt_id);
                        cur_tri_link_set.erase(iter_link);
                        for(iter=cur_mst.begin();iter!=cur_mst.end();iter++){
                            int first=(*iter).first.first;
                            int second=(*iter).first.second;
                            if(cur_tri_link_set.find(first)!=cur_tri_link_set.end() || cur_tri_link_set.find(second)!=cur_tri_link_set.end())
                            {
                                if(cur_infoGain<(*iter).second){
                                    cur_infoGain=(*iter).second;
                                }
                                //sum_infoGain+=(*iter).second;
                                cur_count++;
                            }
                        }
                        info_gains.push_back(cur_infoGain);
                    }
                }
            }
        }

    }


     auto biggest = std::max_element(std::begin(info_gains), std::end(info_gains));
     int target_branch=std::distance(std::begin(info_gains), biggest);
     return cur_trajs[target_branch];
}

void TfCore::select_target_trisector_branch_move(){
    std::set<int> connect_degpts=find_connect_degpts(selected_target_tri);
    std::map<pair<int ,int>,float> cur_mst=search_degpts_mst(connect_degpts,selected_target_tri);
    Trajectory *target_branch=select_a_branch(cur_mst,selected_target_tri);
    add_separatrix_points_toRobot(target_branch);
}

void TfCore::req_turn_service(){
    nav_msgs::Path path;
    path.header.frame_id=worldFrameId;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x=0;
    pose.pose.position.y=0;
    pose.header.frame_id=worldFrameId;
    pose.pose.position.z=0;
    pose.pose.orientation.x=0;//no meaning
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=0;
    pose.pose.orientation.w=1;
    path.poses.push_back(pose);
    pure_pursuit_controller::recoverPath srv;
    srv.request.recoverPath=path;
    if(turnDirectClient.call(srv)){
        ROS_INFO("Request turn direction");
        ifFinish_turn=false;
        while(!ifFinish_turn){
            sleep(2);
            ros::spinOnce();
            if(ifCancelPath) break;
        }
        //ifFinish_turn=false;
    }
    else{
        ROS_ERROR("Failed to call service turn direction");
    }
}


bool TfCore::check_reach_wedge(){
    if(major_path==NULL || major_path->evenstreamlines->ntrajs==0) return false;
    Trajectory *cur_traj= major_path->evenstreamlines->trajs[0];
    int index=cur_traj->nlinesegs-1;
    double loc_x=cur_traj->linesegs[index].gend[0];
    double loc_y=cur_traj->linesegs[index].gend[1];
    if(validDegpts.size()==0) return false;
    float min_dist=std::numeric_limits<float>::max();
    for(int i=0;i<validDegpts.size();i++){
        if(validDegpts[i].type == 0){
            float deta_x=validDegpts[i].gcx-loc_x;
            float deta_y=validDegpts[i].gcy-loc_y;
            float dist=sqrt(deta_x*deta_x+deta_y*deta_y);
            if(dist<min_dist){
                min_dist=dist;
            }
        }
    }
    if(min_dist<0.01){
        return true;
    }else{
        return false;
    }
}
