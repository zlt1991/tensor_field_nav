/*
 * MeshDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DImesh, INDImesh, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h> 
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreFrustum.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"

#include <image_transport/camera_common.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mesh_display_custom.h"

namespace rviz
{

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
    bool valid = true;
    valid = valid && validateFloats( msg.D );
    valid = valid && validateFloats( msg.K );
    valid = valid && validateFloats( msg.R );
    valid = valid && validateFloats( msg.P );
    return valid;
}

MeshDisplayCustom::MeshDisplayCustom()
    : Display()
    , time_since_last_transform_( 0.0f )
    , initialized_(false)
{
    display_images_topic_property_ = new RosTopicProperty( "Display Images Topic", "",
                                            QString::fromStdString( ros::message_traits::datatype<rviz_textured_quads::TexturedQuadArray>() ),
                                            "shape_msgs::Mesh topic to subscribe to.",
                                            this, SLOT( updateDisplayImages() ));

    text_color_property_ = new ColorProperty (  "Text Color", QColor( 255, 255, 255 ),
                                              "caption color.", this, SLOT( updateMeshProperties() )  );

    text_height_property_ = new FloatProperty( "Text Height", 0.1f,
                                              "font size of caption", this, SLOT( updateMeshProperties() ) );

    text_bottom_offset_ =  new FloatProperty( "Text Bottom Offset", 0.18f,
                                              "text placement offset below", this, SLOT( updateMeshProperties() ) );
}

MeshDisplayCustom::~MeshDisplayCustom()
{
    unsubscribe();

    // TODO: Why am I doing this? switch to shared ptrs Argh!!!!!! 

    // clear manual objects
    for (std::vector<Ogre::ManualObject*>::iterator it = manual_objects_.begin() ; it != manual_objects_.end(); ++it)
    {
      delete (*it);
    } 
    manual_objects_.clear();

    // clear decal frustrums
    for (std::vector<Ogre::Frustum*>::iterator it = decal_frustums_.begin() ; it != decal_frustums_.end(); ++it)
    {
      delete (*it);
    } 
    decal_frustums_.clear();

    // clear textures
    for (std::vector<ROSImageTexture*>::iterator it = textures_.begin() ; it != textures_.end(); ++it)
    {
      delete (*it);
    } 
    textures_.clear();

    // clear textures
    for (std::vector<Ogre::SceneNode*>::iterator it = mesh_nodes_.begin() ; it != mesh_nodes_.end(); ++it)
    {
      delete (*it);
    } 
    mesh_nodes_.clear();

    // clear text nodes
    for (std::vector<rviz_textured_quads::TextNode*>::iterator it = text_nodes_.begin() ; it != text_nodes_.end(); ++it)
    {
      delete (*it);
    } 
    text_nodes_.clear();

    // TODO: clean up other things
}

void MeshDisplayCustom::onInitialize()
{
    Display::onInitialize();
}

void MeshDisplayCustom::createProjector(int index)
{
    decal_frustums_[index] = new Ogre::Frustum();

    projector_nodes_[index] = scene_manager_->getRootSceneNode()->createChildSceneNode();
    projector_nodes_[index]->attachObject(decal_frustums_[index]);

    Ogre::SceneNode* filter_node;

    //back filter
    filter_frustums_[index].push_back(new Ogre::Frustum());
    filter_frustums_[index].back()->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    filter_node = projector_nodes_[index]->createChildSceneNode();
    filter_node->attachObject(filter_frustums_[index].back());
    filter_node->setOrientation(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_Y));
}

void MeshDisplayCustom::addDecalToMaterial(int index, const Ogre::String& matName)
{
    Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(matName);
    mat->setCullingMode(Ogre::CULL_NONE);
    Ogre::Pass* pass = mat->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    //pass->setLightingEnabled(true);

    // need the decal_filter to avoid back projection
    Ogre::String resource_group_name = "decal_textures_folder";
    Ogre::ResourceGroupManager& resource_manager = Ogre::ResourceGroupManager::getSingleton();
    if(!resource_manager.resourceGroupExists(resource_group_name))
    {
        resource_manager.createResourceGroup(resource_group_name);
        resource_manager.addResourceLocation(ros::package::getPath("rviz_textured_quads")+"/tests/textures/", "FileSystem", resource_group_name, false);
        resource_manager.initialiseResourceGroup(resource_group_name);
    }
    // loads files into our resource manager
    resource_manager.loadResourceGroup(resource_group_name);

    Ogre::TextureUnitState* tex_state = pass->createTextureUnitState();//"Decal.png");
    tex_state->setTextureName(textures_[index]->getTexture()->getName());
    tex_state->setProjectiveTexturing(true, decal_frustums_[index]);

    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);
    tex_state->setColourOperation(Ogre::LBO_REPLACE); //don't accept additional effects

    for(int i = 0; i < filter_frustums_[index].size(); i++)
    {
        tex_state = pass->createTextureUnitState("Decal_filter.png");
        tex_state->setProjectiveTexturing(true, filter_frustums_[index][i]);
        tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
        tex_state->setTextureFiltering(Ogre::TFO_NONE);
    }
}

shape_msgs::Mesh MeshDisplayCustom::constructMesh( geometry_msgs::Pose mesh_origin, float width, float height, float border_size )
{
    shape_msgs::Mesh mesh;

    Eigen::Affine3d trans_mat;
    tf::poseMsgToEigen(mesh_origin, trans_mat);

    // Rviz Coordinate System: x-right, y-forward, z-down
    // create mesh vertices and tranform them to the specified pose

    Eigen::Vector4d top_left(-width/2.0f - border_size, 0.0f, -height/2.0f - border_size, 1.0f);
    Eigen::Vector4d top_right(width/2.0f + border_size, 0.0f, -height/2.0f - border_size, 1.0f);
    Eigen::Vector4d bottom_left(-width/2.0f - border_size, 0.0f, height/2.0f + border_size, 1.0f);
    Eigen::Vector4d bottom_right(width/2.0f + border_size, 0.0f, height/2.0f + border_size, 1.0f);

    Eigen::Vector4d trans_top_left = trans_mat.matrix() * top_left;
    Eigen::Vector4d trans_top_right = trans_mat.matrix() * top_right;
    Eigen::Vector4d trans_bottom_left = trans_mat.matrix() * bottom_left;
    Eigen::Vector4d trans_bottom_right = trans_mat.matrix() * bottom_right;

    std::vector<geometry_msgs::Point> vertices(4);
    vertices.at(0).x = trans_top_left[0]; 
    vertices.at(0).y = trans_top_left[1];  
    vertices.at(0).z = trans_top_left[2];  
    vertices.at(1).x = trans_top_right[0];
    vertices.at(1).y = trans_top_right[1];
    vertices.at(1).z = trans_top_right[2];
    vertices.at(2).x = trans_bottom_left[0];
    vertices.at(2).y = trans_bottom_left[1];
    vertices.at(2).z = trans_bottom_left[2];
    vertices.at(3).x = trans_bottom_right[0];
    vertices.at(3).y = trans_bottom_right[1];
    vertices.at(3).z = trans_bottom_right[2];
    mesh.vertices = vertices;

    std::vector<shape_msgs::MeshTriangle> triangles(2);
    triangles.at(0).vertex_indices[0] = 0;
    triangles.at(0).vertex_indices[1] = 1; 
    triangles.at(0).vertex_indices[2] = 2; 
    triangles.at(1).vertex_indices[0] = 1;
    triangles.at(1).vertex_indices[1] = 2; 
    triangles.at(1).vertex_indices[2] = 3; 
    mesh.triangles = triangles;

    return mesh;
}

void MeshDisplayCustom::clearStates(int num_quads)
{
    for (int q=0; q<manual_objects_.size(); q++)
    {
        manual_objects_[q]->clear();
    }

    for (int q=0; q<text_nodes_.size(); q++)
    {
        text_nodes_[q]->clear();
    }

    // resize state vectors   
    mesh_poses_.resize(num_quads);
    img_widths_.resize(num_quads);
    img_heights_.resize(num_quads);
    physical_widths_.resize(num_quads);
    physical_heights_.resize(num_quads);

    manual_objects_.resize(num_quads);
    last_meshes_.resize(num_quads);
    
    last_images_.resize(num_quads);
    textures_.resize(num_quads);
    decal_frustums_.resize(num_quads);
    projector_nodes_.resize(num_quads);
    filter_frustums_.resize(num_quads);
    mesh_materials_.resize(num_quads);
    mesh_nodes_.resize(num_quads);
    text_nodes_.resize(num_quads);

    border_colors_.resize(num_quads);
    border_sizes_.resize(num_quads);  
}

void MeshDisplayCustom::constructQuads( const rviz_textured_quads::TexturedQuadArray::ConstPtr& images )
{
    int num_quads = images->quads.size();

    clearStates(num_quads);

    for (int q=0; q<num_quads; q++)
    {
        processImage(q, images->quads[q].image);

        geometry_msgs::Pose mesh_origin = images->quads[q].pose;

        // Rotate from x-y to x-z plane:
        Eigen::Affine3d trans_mat;
        tf::poseMsgToEigen(mesh_origin, trans_mat);
        trans_mat = trans_mat * Eigen::Quaterniond(0.70710678, -0.70710678f, 0.0f, 0.0f);
        
        Eigen::Quaterniond xz_quat(trans_mat.rotation());
        mesh_origin.orientation.x = xz_quat.x();
        mesh_origin.orientation.y = xz_quat.y();
        mesh_origin.orientation.z = xz_quat.z();
        mesh_origin.orientation.w = xz_quat.w();

        float width = images->quads[q].width;
        float height = images->quads[q].height;

        // set properties
        mesh_poses_[q] = mesh_origin;
        img_widths_[q] = images->quads[q].image.width;
        img_heights_[q] = images->quads[q].image.height;

        border_colors_[q].resize(4);

        if (images->quads[q].border_color.size() == 4) 
        {
            border_colors_[q][0] = images->quads[q].border_color[0];
            border_colors_[q][1] = images->quads[q].border_color[1];
            border_colors_[q][2] = images->quads[q].border_color[2];
            border_colors_[q][3] = images->quads[q].border_color[3];
        }
        else
        {
            // default white border
            border_colors_[q][0] = 255.0f;
            border_colors_[q][1] = 255.0f;
            border_colors_[q][2] = 255.0f;
            border_colors_[q][3] = 255.0f;
        }

        if (images->quads[q].border_size >= 0.0f)
        {
            border_sizes_[q] = images->quads[q].border_size;
        }
        else
        {
            // default border size (no border)
            border_sizes_[q] = 0.0f;
        }

        shape_msgs::Mesh mesh = constructMesh(mesh_origin, width, height, border_sizes_[q]);

        physical_widths_[q] = width;
        physical_heights_[q] = height;

        boost::mutex::scoped_lock lock( mesh_mutex_ );

        // create our scenenode and material
        load(q);

        Ogre::Vector3 caption_position = Ogre::Vector3(mesh_origin.position.x, mesh_origin.position.y + 0.5f*height + text_bottom_offset_->getFloat(), mesh_origin.position.z);

        if (!manual_objects_[q])
        {
            static uint32_t count = 0;
            std::stringstream ss;
            ss << "MeshObject" << count++ << "Index" << q;
            manual_objects_[q] = context_->getSceneManager()->createManualObject(ss.str());
            mesh_nodes_[q]->attachObject(manual_objects_[q]);
        }

        // If we have the same number of tris as previously, just update the object
        if (last_meshes_[q].vertices.size() > 0 && mesh.vertices.size()*2 == last_meshes_[q].vertices.size())
        {
            manual_objects_[q]->beginUpdate(0);
        }
        else // Otherwise clear it and begin anew
        {
            manual_objects_[q]->clear();
            manual_objects_[q]->estimateVertexCount(mesh.vertices.size()*2);
            manual_objects_[q]->begin(mesh_materials_[q]->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
        }

        const std::vector<geometry_msgs::Point>& points = mesh.vertices;
        for(size_t i = 0; i < mesh.triangles.size(); i++)
        {
            // make sure we have front-face/back-face triangles
            for(int side = 0; side < 2; side++)
            {
                std::vector<Ogre::Vector3> corners(3);
                for(size_t c = 0; c < 3; c++)
                {
                    size_t corner = side ? 2-c : c; // order of corners if side == 1
                    corners[corner] = Ogre::Vector3(points[mesh.triangles[i].vertex_indices[corner]].x, points[mesh.triangles[i].vertex_indices[corner]].y, points[mesh.triangles[i].vertex_indices[corner]].z);
                }
                Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
                normal.normalise();

                for(size_t c = 0; c < 3; c++)
                {
                    manual_objects_[q]->position(corners[c]);
                    manual_objects_[q]->normal(normal);
                }
            }
        }

        manual_objects_[q]->end();

        mesh_materials_[q]->setCullingMode(Ogre::CULL_NONE);

        last_meshes_[q] = mesh;

        Ogre::ColourValue text_color(text_color_property_->getColor().redF(), text_color_property_->getColor().greenF(), text_color_property_->getColor().blueF(), 1.0f);

        if (!text_nodes_[q])
        {
            text_nodes_[q] = new rviz_textured_quads::TextNode(context_->getSceneManager(), manual_objects_[q]->getParentSceneNode(), caption_position);
            text_nodes_[q]->setCaption(images->quads[q].caption);
            text_nodes_[q]->setCharacterHeight(text_height_property_->getFloat());
            text_nodes_[q]->setColor(text_color);
        } 
        else
        {
            text_nodes_[q]->setCaption(images->quads[q].caption);
            text_nodes_[q]->setPosition(caption_position);
            text_nodes_[q]->setCharacterHeight(text_height_property_->getFloat());
            text_nodes_[q]->setColor(text_color);

        }

    }
}

void MeshDisplayCustom::updateImageMeshes( const rviz_textured_quads::TexturedQuadArray::ConstPtr& images )
{
    constructQuads(images);
    updateMeshProperties();
}

void MeshDisplayCustom::updateMeshProperties()
{
    for (int i=0; i<mesh_materials_.size(); i++)
    {
        // update color/alpha
        Ogre::Technique* technique = mesh_materials_[i]->getTechnique(0);
        Ogre::Pass* pass = technique->getPass(0);

        Ogre::ColourValue self_illumination_color(0.0f, 0.0f, 0.0f, 0.0f);// border_colors_[i][3]);
        pass->setSelfIllumination(self_illumination_color);

        Ogre::ColourValue diffuse_color(0.0f, 0.0f, 0.0f, 1.0f/*border_colors_[i][0], border_colors_[i][1], border_colors_[i][2], border_colors_[i][3]*/);
        pass->setDiffuse(diffuse_color);

        Ogre::ColourValue ambient_color(border_colors_[i][0], border_colors_[i][1], border_colors_[i][2], border_colors_[i][3]);
        pass->setAmbient(ambient_color);

        Ogre::ColourValue specular_color(0.0f, 0.0f, 0.0f, 1.0f);
        pass->setSpecular(specular_color);

        Ogre::Real shininess = 64.0f;
        pass->setShininess(shininess);

        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(false);

        context_->queueRender();
    }

}

void MeshDisplayCustom::updateDisplayImages()
{
    unsubscribe();
    subscribe();
}

void MeshDisplayCustom::subscribe()
{
    if ( !isEnabled() )
    {
        return;
    }

    if( !display_images_topic_property_->getTopic().isEmpty() )
    {
        try
        {
            rviz_display_images_sub_ = nh_.subscribe(display_images_topic_property_->getTopicStd(), 1, &MeshDisplayCustom::updateImageMeshes, this);
            setStatus( StatusProperty::Ok, "Display Images Topic", "OK" );
        }
        catch( ros::Exception& e )
        {
            setStatus( StatusProperty::Error, "Display Images Topic", QString( "Error subscribing: " ) + e.what() );
        }
    }
}

void MeshDisplayCustom::unsubscribe()
{
    rviz_display_images_sub_.shutdown();
}

void MeshDisplayCustom::load(int index)
{   
    if(mesh_nodes_[index] != NULL)
        return;

    static int count = 0;
    std::stringstream ss;
    ss << "MeshNode" << count++ << "GroupIndex" << index;
    Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();
    Ogre::String resource_group_name =  ss.str();

    Ogre::ResourceGroupManager& rg_mgr = Ogre::ResourceGroupManager::getSingleton();

    Ogre::String material_name = resource_group_name+"MeshMaterial";

    if(!rg_mgr.resourceGroupExists(resource_group_name))
    {
        rg_mgr.createResourceGroup(resource_group_name);

        mesh_materials_[index] = material_manager.create(material_name,resource_group_name);
        Ogre::Technique* technique = mesh_materials_[index]->getTechnique(0);
        Ogre::Pass* pass = technique->getPass(0);

        Ogre::ColourValue self_illumnation_color(0.0f, 0.0f, 0.0f, border_colors_[index][3]);
        pass->setSelfIllumination(self_illumnation_color);

        Ogre::ColourValue diffuse_color(border_colors_[index][0], border_colors_[index][1], border_colors_[index][2], border_colors_[index][3]);
        pass->setDiffuse(diffuse_color);

        Ogre::ColourValue ambient_color(border_colors_[index][0], border_colors_[index][1], border_colors_[index][2], border_colors_[index][3]);
        pass->setAmbient(ambient_color);

        Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
        pass->setSpecular(specular_color);

        Ogre::Real shininess = 64.0f;
        pass->setShininess(shininess);

        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        mesh_materials_[index]->setCullingMode(Ogre::CULL_NONE);
    }

    mesh_nodes_[index] = this->scene_node_->createChildSceneNode();

}

void MeshDisplayCustom::onEnable()
{
    subscribe();
}

void MeshDisplayCustom::onDisable()
{
    unsubscribe();
}

void MeshDisplayCustom::update( float wall_dt, float ros_dt )
{
    time_since_last_transform_ += wall_dt;

    if( !display_images_topic_property_->getTopic().isEmpty() )
    {
        try
        {
            for (int i=0; i<textures_.size(); i++)
            {
                updateCamera(i, textures_[i]->update());
            }
        }
        catch( UnsupportedImageEncoding& e )
        {
            setStatus(StatusProperty::Error, "Display Image", e.what());
        }
    }
}

bool MeshDisplayCustom::updateCamera(int index, bool update_image)
{
    if(update_image)
    {
        last_images_[index] = textures_[index]->getImage();
    }

    if(!img_heights_[index] || !img_widths_[index] || 
       !physical_widths_[index] || !physical_heights_[index] || 
       !last_images_[index])
    {        
        return false;
    }

    boost::mutex::scoped_lock lock( mesh_mutex_ );

    float img_width  = img_widths_[index];
    float img_height = img_heights_[index];

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    context_->getFrameManager()->getTransform( last_images_[index]->header.frame_id, last_images_[index]->header.stamp, position, orientation );

    Eigen::Affine3d trans_mat;
    tf::poseMsgToEigen(mesh_poses_[index], trans_mat);

    // Rotate by 90 deg to get xz plane
    trans_mat = trans_mat * Eigen::Quaterniond(0.70710678, -0.70710678f, 0.0f, 0.0f);

    float z_offset = (img_width > img_height) ? img_width : img_height;
    float scale_factor = 1.0f / (physical_widths_[index] > physical_heights_[index] ? physical_widths_[index] : physical_heights_[index]);

    Eigen::Vector4d projector_origin(0.0f, 0.0f, 1.0f / (z_offset * scale_factor), 1.0f);
    Eigen::Vector4d projector_point = trans_mat.matrix() * projector_origin;

    position = Ogre::Vector3(projector_point[0], projector_point[1], projector_point[2] );
    orientation = Ogre::Quaternion(mesh_poses_[index].orientation.w, mesh_poses_[index].orientation.x, mesh_poses_[index].orientation.y, mesh_poses_[index].orientation.z);

    // Update orientation with 90 deg offset (xy to xz)
    orientation = orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_X );

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation * Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_Z );


    // std::cout << "CameraInfo dimensions: " << last_info_->width << " x " << last_info_->height << std::endl;
    // std::cout << "Texture dimensions: " << last_image_->width << " x " << last_image_->height << std::endl;
    //std::cout << "Original image dimensions: " << last_image_->width*full_image_binning_ << " x " << last_image_->height*full_image_binning_ << std::endl;

    // If the image width/height is 0 due to a malformed caminfo, try to grab the width from the image.
    if( img_width <= 0 )
    {
      ROS_ERROR( "Malformed CameraInfo on camera [%s], width = 0", qPrintable( getName() ));
      // use texture size, but have to remove border from the perspective calculations
      img_width = textures_[index]->getWidth()-2;
    }

    if (img_height <= 0)
    {
        ROS_ERROR( "Malformed CameraInfo on camera [%s], height = 0", qPrintable( getName() ));
        // use texture size, but have to remove border from the perspective calculations
        img_height = textures_[index]->getHeight()-2;
    }

    // if even the texture has 0 size, return
    if( img_height <= 0.0 || img_width <= 0.0 )
    {
        setStatus( StatusProperty::Error, "Camera Info",
                 "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0) and texture." );
        return false;
    }

    // projection matrix
    float P[12] = {1.0, 0.0, img_width / 2.0f, 0.0, 
                   0.0, 1.0, img_height / 2.0f, 0.0, 
                   0.0, 0.0, 1.0, 0.0 };

    // calculate projection matrix
    double fx = P[0];
    double fy = P[5];

    // Add the camera's translation relative to the left camera (from P[3]);
    double tx = -1 * (P[3] / fx);
    Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
    position = position + (right * tx);

    double ty = -1 * (P[7] / fy);
    Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
    position = position + (down * ty);

    if( !validateFloats( position ))
    {
        ROS_ERROR( "position error");
        setStatus( StatusProperty::Error, "Camera Info", "CameraInfo/P resulted in an invalid position calculation (nans or infs)" );
        return false;
    }

    if(projector_nodes_[index] != NULL)
    {
        projector_nodes_[index]->setPosition( position );
        projector_nodes_[index]->setOrientation( orientation );
    }

    // calculate the projection matrix
    double cx = P[2];
    double cy = P[6];

    double far_plane = 100;
    double near_plane = 0.01;

    Ogre::Matrix4 proj_matrix;
    proj_matrix = Ogre::Matrix4::ZERO;

    proj_matrix[0][0]= 2.0 * fx/img_width;
    proj_matrix[1][1]= 2.0 * fy/img_height;

    proj_matrix[0][2]= 2.0 * (0.5 - cx/img_width);
    proj_matrix[1][2]= 2.0 * (cy/img_height - 0.5);

    proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
    proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

    proj_matrix[3][2]= -1;

    if(decal_frustums_[index] != NULL)
        decal_frustums_[index]->setCustomProjectionMatrix(true, proj_matrix);

    // ROS_INFO(" Camera (%f, %f)", proj_matrix[0][0], proj_matrix[1][1]);
    // ROS_INFO(" Render Panel: %x   Viewport: %x", render_panel_, render_panel_->getViewport());


    setStatus( StatusProperty::Ok, "Time", "ok" );
    setStatus( StatusProperty::Ok, "Camera Info", "ok" );

    if(mesh_nodes_[index] != NULL && filter_frustums_[index].size() == 0 && !mesh_materials_[index].isNull())
    {
        createProjector(index);

        addDecalToMaterial(index, mesh_materials_[index]->getName());
    }

    return true;
}

void MeshDisplayCustom::clear()
{
    for (int i=0; i<textures_.size(); i++)
    {
        textures_[i]->clear();
    }

    context_->queueRender();

    setStatus( StatusProperty::Warn, "Image", "No Image received");
}

void MeshDisplayCustom::reset()
{
    Display::reset();
    clear();
}

void MeshDisplayCustom::processImage(int index, const sensor_msgs::Image& msg)
{
    //std::cout<<"camera image received"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr;

    // simply converting every image to RGBA
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("MeshDisplayCustom: cv_bridge exception: %s", e.what());
        return;
    }

    // add completely white transparent border to the image so that it won't replicate colored pixels all over the mesh
    cv::Scalar value(255,255,255,0);
    cv::copyMakeBorder(cv_ptr->image,cv_ptr->image,1,1,1,1,cv::BORDER_CONSTANT,value);
    cv::flip(cv_ptr->image,cv_ptr->image,-1);

    // Output modified video stream
    if (textures_[index] == NULL)
        textures_[index] = new ROSImageTexture();

    textures_[index]->addMessage(cv_ptr->toImageMsg());
}

} // namespace rviz
