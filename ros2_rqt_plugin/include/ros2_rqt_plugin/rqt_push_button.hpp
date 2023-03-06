#ifndef PUSH_BUTTON_H__
#define PUSH_BUTTON_H__

// ROS2
#include <rclcpp/rclcpp.hpp>

// msgs
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include"sensor_msgs/msg/imu.hpp"
#include"sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Rqt
#include <rqt_gui_cpp/plugin.h>

// Qt
#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QChartView>
#include <QTableWidget>


// Custom UI
#include <ros2_rqt_plugin/ui_rqt_push_button.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include <image_transport/subscriber.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

//car_control
#include <ros2_rqt_plugin/car_init.hpp>
#include <private_interface/msg/car_control.hpp>
#include <private_interface/msg/control_state.hpp>
#include "private_interface/msg/car_basic_sensor.hpp"






QT_CHARTS_USE_NAMESPACE

namespace rqt_plugin
{
  class pushButton : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
  public:
    pushButton();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context) override;

    virtual void shutdownPlugin() override;

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const override;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings) override;

    virtual ~pushButton();

    



  private:
    Ui::myWidget ui_;
    QWidget *widget_;

    /* 用于模拟生成实时数据的定时器 */
    QTimer* m_timer;
    struct timespec lastTime = {0, 0};

    /* 图表对象 */
    QTabWidget m_tabWidget;
    QChart* Acc_chart,*Angual_chart,*Mag_chart;

    /* 横纵坐标轴对象 */
    QValueAxis *Acc_axisX, *Acc_axisY,*Acc_axisZ;
    QValueAxis *Angual_axisX,*Angual_axisY,*Angual_axisZ;
    QValueAxis *Mag_axisX,*Mag_axisY,*Mag_axisZ;

    /* 曲线图对象 */
    QLineSeries* Acc_seriesx,* Acc_seriesy,* Acc_seriesz;
    QLineSeries* Angual_seriesx,* Angual_seriesy,* Angual_seriesz;
    QLineSeries* Mag_seriesx,* Mag_seriesy,* Mag_seriesz;

    /* 横纵坐标最大显示范围 */
    const int AXIS_MAX_X = 6, AXIS_MAX_Y = 10;
    const int MAG_AXIS_MAX_X = 6, MAG_AXIS_MAX_Y = 2;

    /* 用来记录数据点数 */
    int pointCount = 0;
    float Acc_x=0,Acc_y=0,Acc_z=0;
    float Angual_x=0,Angual_y=0,Angual_z=0;
    float Mag_x=0,Mag_y=0,Mag_z=0;
    int imu_sec_data,imu_nanosec_data;
    int imageFront_sec_data,imageFront_nanosec_data;
    int imageBack_sec_data,imageBack_nanosec_data;
    int control_sec_data,control_nanosec_data;
    
    
        

  protected:
    
    private_interface::msg::CarControl remote_control_message;
    QString str;


    // image订阅者的回调函数from image_view.cpp
    
    //发送的图片type
    // cv::Mat img;
    cv::Mat imgCallback;
    //回调函数中发送图像的信号
    void imageSignal(cv::Mat); 
   
    // ROS中用于接收图像消息类型的订阅者，用image_transport声明
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub; // image_transport::ImageTransport it(nh);用之前声明的节点句柄初始化it，其实这里的it和nh的功能基本一样，你可以向之前一样使用it来发布和订阅相消息
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_true_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_virtual_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_back_compressed_sub;
    rclcpp::Publisher<private_interface::msg::CarControl>::SharedPtr remote_control_button_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_button_virtual_pub;
    rclcpp::Subscription<private_interface::msg::CarBasicSensor>::SharedPtr sensor_data_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr actualDataLine_sub;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr actualMag_data_sub;

    cv_bridge::CvImagePtr cv_ptr;


    
 




  public slots:
    

  protected slots:


    void onTabCloseRequested(int index);
    void on_run_button_clicked();
    void up_button_clicked();
    void down_button_clicked();
    void left_button_clicked();
    void right_button_clicked();
    void stop_button_clicked();
    void slotTimeout();


  };
} // namespace rqt_plugin

#endif // PUSH_BUTTON_H__
