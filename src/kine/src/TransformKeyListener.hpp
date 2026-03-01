
#ifndef TRANSFORMKEYLISTENER_HPP
#define TRANSFORMKEYLISTENER_HPP

#include <threepp/controls/TransformControls.hpp>
#include <threepp/input/KeyListener.hpp>

#include <rclcpp/logger.hpp>

class TransformKeyListener : public threepp::KeyListener
{

public:
    explicit TransformKeyListener(threepp::TransformControls* transform_controls)
        : transformControls(transform_controls)
    {
    }

    void onKeyPressed(KeyEvent evt) override
    {
        switch (evt.key)
        {
        case Key::Q:
            {
                transformControls->setSpace(transformControls->getSpace() == "local" ? "world" : "local");
                RCLCPP_INFO(rclcpp::get_logger("TransformKeyListener"), "TransformControls space: %s", transformControls->getSpace().c_str());
                break;
            }
        case Key::W:
            {
                transformControls->setMode("translate");
                RCLCPP_INFO(rclcpp::get_logger("TransformKeyListener"), "TransformControls mode: translate");
                break;
            }
        case Key::E:
            {
                transformControls->setMode("rotate");
                RCLCPP_INFO(rclcpp::get_logger("TransformKeyListener"), "TransformControls mode: rotate");
                break;
            }
        default:
            break;
        }
    }

private:
    threepp::TransformControls* transformControls;

};

#endif //TRANSFORMKEYLISTENER_HPP
