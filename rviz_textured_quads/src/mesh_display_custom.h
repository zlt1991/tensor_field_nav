/*
 * MeshDisplayCustom declaration.
 *
 * Author: Felipe Bacim.
 *
 * help with selection of robot parts 
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
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_MESH_DISPLAY_H
#define RVIZ_MESH_DISPLAY_H

#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>

#include <rviz_textured_quads/TexturedQuad.h>
#include <rviz_textured_quads/TexturedQuadArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <map>
#include <vector>

#include "text_node.h"

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class Axes;
class RenderPanel;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class VectorProperty;
class StringProperty;
class QuaternionProperty;
}

namespace rviz
{

/**
 * \class MeshDisplayCustom
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class MeshDisplayCustom: public rviz::Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  MeshDisplayCustom();
  virtual ~MeshDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

private Q_SLOTS:
  void updateMeshProperties();
  void updateDisplayImages();

protected:
  virtual void load(int index);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  // This is called by incomingMessage().
  void processImage(int index, const sensor_msgs::Image& msg);

  virtual void subscribe();
  virtual void unsubscribe();

private:
  void clear();
  bool updateCamera(int index, bool update_image);

  void createProjector(int index);
  void addDecalToMaterial(int index, const Ogre::String& matName);
  void updateImageMeshes( const rviz_textured_quads::TexturedQuadArray::ConstPtr& images );
  
  void constructQuads( const rviz_textured_quads::TexturedQuadArray::ConstPtr& images );
  shape_msgs::Mesh constructMesh( geometry_msgs::Pose mesh_origin, float width, float height, float border_size );
  void clearStates(int num_quads);

  float time_since_last_transform_;

  RosTopicProperty* display_images_topic_property_;
  ColorProperty* text_color_property_;
  FloatProperty* text_height_property_;
  FloatProperty* text_bottom_offset_;

  std::vector<shape_msgs::Mesh> last_meshes_;
  std::vector<geometry_msgs::Pose> mesh_poses_;
  std::vector<int> img_widths_, img_heights_;
  std::vector<float> physical_widths_, physical_heights_;
  std::vector<std::vector<float> > border_colors_;
  std::vector<float> border_sizes_;
  std::vector<rviz_textured_quads::TextNode*> text_nodes_;

  ros::NodeHandle nh_;

  std::vector<sensor_msgs::Image::ConstPtr> last_images_;

  std::vector<Ogre::SceneNode*> mesh_nodes_;
  std::vector<Ogre::ManualObject*> manual_objects_;
  std::vector<Ogre::MaterialPtr> mesh_materials_;
  std::vector<ROSImageTexture*> textures_;

  ros::Subscriber pose_sub_;
  ros::Subscriber rviz_display_images_sub_;

  std::vector<Ogre::Frustum*> decal_frustums_;
  std::vector<std::vector<Ogre::Frustum*> > filter_frustums_; //need multiple filters (back, up, down, left, right)
  std::vector<Ogre::SceneNode*> projector_nodes_;

  std::vector<RenderPanel*> render_panel_list_;
  RenderPanel* render_panel_; // this is the active render panel

  bool initialized_;

  boost::mutex mesh_mutex_;
};

} // namespace rviz

#endif


