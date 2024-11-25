/**
 * 我们正在实现 MessageFilterDisplay 类，该类可用于任何带有 std_msgs/Header 的消息。
 * 该类使用我们的 Point2D 消息类型进行模板化。
 * 由于超出本教程范围的原因，您需要其中的 Q_OBJECT 宏才能使 GUI 的 QT 部分正常工作。
 * processMessage 是唯一需要实现的方法，我们将在 cpp 文件中实现。
 */



//V2
// #ifndef RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
// #define RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_

// #include <memory>

// #include <rviz_common/message_filter_display.hpp>
// #include <rviz_plugin_tutorial_msgs/msg/point2_d.hpp>
// #include <rviz_rendering/objects/shape.hpp>

// namespace rviz_plugin_tutorial
// {
// class PointDisplay
//   : public rviz_common::MessageFilterDisplay<rviz_plugin_tutorial_msgs::msg::Point2D>
// {
//   Q_OBJECT

// protected:
//   void onInitialize() override;

//   void processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg) override;

//   std::unique_ptr<rviz_rendering::Shape> point_shape_;
// };
// }  // namespace rviz_plugin_tutorial

// #endif  // RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_


//v3 v4 v5

#ifndef RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
#define RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_plugin_tutorial_msgs/msg/point2_d.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace rviz_plugin_tutorial
{
class PointDisplay
  : public rviz_common::MessageFilterDisplay<rviz_plugin_tutorial_msgs::msg::Point2D>
{
  Q_OBJECT

private Q_SLOTS:
  void updateStyle();

protected:
  void onInitialize() override;

  void processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg) override;

  std::unique_ptr<rviz_rendering::Shape> point_shape_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
};
}  // namespace rviz_plugin_tutorial

#endif  // RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_