/**
 * 日志记录并不是绝对必要的，但有助于调试。
 * 为了让 RViz 找到我们的插件，我们需要在代码中调用这个 PLUGINLIB（以及下面的其他内容）。
 */


//v2
// #include <rviz_plugin_tutorial/point_display.hpp>
// #include <rviz_common/logging.hpp>

// namespace rviz_plugin_tutorial
// {
// void PointDisplay::onInitialize()
// {
//   MFDClass::onInitialize();
//   point_shape_ =
//     std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
//       scene_node_);
// }

// void PointDisplay::processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg)
// {
//   RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);

//   Ogre::Vector3 position;
//   Ogre::Quaternion orientation;
//   if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
//     RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id <<
//         "' to frame '" << qPrintable(fixed_frame_) << "'");
//   }

//   scene_node_->setPosition(position);
//   scene_node_->setOrientation(orientation);

//   Ogre::Vector3 point_pos;
//   point_pos.x = msg->x;
//   point_pos.y = msg->y;
//   point_shape_->setPosition(point_pos);
// }
// }  // namespace rviz_plugin_tutorial

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorial::PointDisplay, rviz_common::Display)


// //v3  带有颜色选项
// #include <rviz_plugin_tutorial/point_display.hpp>
// #include <rviz_common/properties/parse_color.hpp>
// #include <rviz_common/logging.hpp>

// namespace rviz_plugin_tutorial
// {
// void PointDisplay::onInitialize()
// {
//   MFDClass::onInitialize();
//   point_shape_ =
//     std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
//       scene_node_);

//   color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
//       "Point Color", QColor(36, 64, 142), "Color to draw the point.", this, SLOT(updateStyle()));
//   updateStyle();
// }

// void PointDisplay::processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg)
// {
//   RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);

//   Ogre::Vector3 position;
//   Ogre::Quaternion orientation;
//   if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
//     RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id <<
//         "' to frame '" << qPrintable(fixed_frame_) << "'");
//   }

//   scene_node_->setPosition(position);
//   scene_node_->setOrientation(orientation);

//   Ogre::Vector3 point_pos;
//   point_pos.x = msg->x;
//   point_pos.y = msg->y;
//   point_shape_->setPosition(point_pos);
// }

// void PointDisplay::updateStyle()
// {
//   Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
//   point_shape_->setColor(color);
// }

// }  // namespace rviz_plugin_tutorial

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorial::PointDisplay, rviz_common::Display)


//v4  v5 设置显示的状态。
#include <rviz_plugin_tutorial/point_display.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/logging.hpp>

namespace rviz_plugin_tutorial
{
using rviz_common::properties::StatusProperty;

void PointDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_shape_ =
    std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
      scene_node_);

  color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Point Color", QColor(36, 64, 142), "Color to draw the point.", this, SLOT(updateStyle()));
  updateStyle();
}

void PointDisplay::processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg)
{
  RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id <<
        "' to frame '" << qPrintable(fixed_frame_) << "'");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (msg->x < 0) {
    setStatus(StatusProperty::Warn, "Message",
        "I will complain about points with negative x values.");
  } else {
    setStatus(StatusProperty::Ok, "Message", "OK");
  }

  Ogre::Vector3 point_pos;
  point_pos.x = msg->x;
  point_pos.y = msg->y;
  point_shape_->setPosition(point_pos);
}

void PointDisplay::updateStyle()
{
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  point_shape_->setColor(color);
}

}  // namespace rviz_plugin_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorial::PointDisplay, rviz_common::Display)