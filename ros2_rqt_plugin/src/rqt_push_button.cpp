#include "ros2_rqt_plugin/rqt_push_button.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QStringList>
#include <QBoxLayout>
#include <cv_bridge/cv_bridge.h>
#include <QObject>
#include <QComboBox>
#include <image_transport/image_transport.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "private_interface/msg/car_control.hpp"
#include "private_interface/msg/car_basic_sensor.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QChartView>
#include <sys/time.h>
#include <qdatetime.h>

PLUGINLIB_EXPORT_CLASS(rqt_plugin::pushButton, rqt_gui_cpp::Plugin)

namespace rqt_plugin
{
    pushButton::pushButton()
        : rqt_gui_cpp::Plugin(), widget_(0)
    {
        
    }

    void pushButton::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        // Access standalone command line arguments访问独立命令行参数
        QStringList argv = context.argv();

        // Create QWidget
        widget_ = new QWidget();

        // Extend the widget with all attributes and children from UI file
        ui_.setupUi(widget_);

        // add widget to the user interface
        context.addWidget(widget_);

        auto intra_comms_options = rclcpp::NodeOptions{}.use_intra_process_comms(true);


        // image_sub = node_->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, [this](const sensor_msgs::msg::Image::ConstSharedPtr msg){

        //     QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);

        //     QImage image;
        //     image=temp;

        //     this->ui_.label_image->setPixmap(QPixmap::fromImage(image).scaled(ui_.label_image->width(),ui_.label_image->height()));
        //     this->widget_->show();
        //     //cv::waitKey(1);
        // }); 
        
        //image_sub
        image_compressed_true_sub = node_->create_subscription<sensor_msgs::msg::CompressedImage>("/camera/color/image_raw/compressed", 30, 
        [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){

            try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    //cv::imshow("label_image",imgCallback);
                    cv::Mat rgb;
                   
                    QImage img;
                    if (imgCallback.channels() == 3)
                    {
                        // cvt Mat BGR 2 QImage RGB
                        cv::cvtColor(imgCallback, rgb, CV_BGR2RGB);
                        img = QImage((const unsigned char *)(rgb.data),
                                    rgb.cols, rgb.rows,
                                    rgb.cols * rgb.channels(),
                                    QImage::Format_RGB888);
                        
                    }
                    else
                    {
                        img = QImage((const unsigned char *)(imgCallback.data),
                                    imgCallback.cols, imgCallback.rows,
                                    imgCallback.cols * imgCallback.channels(),
                                    QImage::Format_RGB888);
                       
                    }
                    ui_.label_image->setPixmap(QPixmap::fromImage(img).scaled(ui_.label_image->width(),ui_.label_image->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            this->imageFront_sec_data=msg->header.stamp.sec;
            this->imageFront_nanosec_data=msg->header.stamp.nanosec;
        }); 
        image_compressed_virtual_sub = node_->create_subscription<sensor_msgs::msg::CompressedImage>("/mobot/camera/image_raw/compressed", 30, 
        [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){

            try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    //cv::imshow("label_image",imgCallback);
                    cv::Mat rgb;
                   
                    QImage img;
                    if (imgCallback.channels() == 3)
                    {
                        // cvt Mat BGR 2 QImage RGB
                        cv::cvtColor(imgCallback, rgb, CV_BGR2RGB);
                        img = QImage((const unsigned char *)(rgb.data),
                                    rgb.cols, rgb.rows,
                                    rgb.cols * rgb.channels(),
                                    QImage::Format_RGB888);
                        
                    }
                    else
                    {
                        img = QImage((const unsigned char *)(imgCallback.data),
                                    imgCallback.cols, imgCallback.rows,
                                    imgCallback.cols * imgCallback.channels(),
                                    QImage::Format_RGB888);
                       
                    }
                    ui_.label_image->setPixmap(QPixmap::fromImage(img).scaled(ui_.label_image->width(),ui_.label_image->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            this->imageFront_sec_data=msg->header.stamp.sec;
            this->imageFront_nanosec_data=msg->header.stamp.nanosec;
        }); 
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
        image_back_compressed_sub = node_->create_subscription<sensor_msgs::msg::CompressedImage>("/rear_camera/camera/image_raw/compressed", qos, 
        [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){

            try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    //cv::imshow("label_image",imgCallback);
                    cv::Mat rgb;
                   
                    QImage img;
                    if (imgCallback.channels() == 3)
                    {
                        // cvt Mat BGR 2 QImage RGB
                        cv::cvtColor(imgCallback, rgb, CV_BGR2RGB);
                        img = QImage((const unsigned char *)(rgb.data),
                                    rgb.cols, rgb.rows,
                                    rgb.cols * rgb.channels(),
                                    QImage::Format_RGB888);
                        
                    }
                    else
                    {
                        img = QImage((const unsigned char *)(imgCallback.data),
                                    imgCallback.cols, imgCallback.rows,
                                    imgCallback.cols * imgCallback.channels(),
                                    QImage::Format_RGB888);
                       
                    }
                    ui_.label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_.label_image_back->width(),ui_.label_image_back->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            
            this->imageBack_sec_data=msg->header.stamp.sec;
            this->imageBack_nanosec_data=msg->header.stamp.nanosec;
        });

        //remote_control
        remote_control_button_pub=node_->create_publisher<private_interface::msg::CarControl>("remote_control",10);
        remote_control_button_virtual_pub=node_->create_publisher<geometry_msgs::msg::Twist>("mobot/cmd_vel",10);

        //sensor_data_sub
        sensor_data_sub = node_->create_subscription<private_interface::msg::CarBasicSensor>("/controller_state_log", 10,
        [this](const private_interface::msg::CarBasicSensor::ConstSharedPtr sensor_message){
        this->ui_.gear->setText(QString::number(sensor_message->state.gear));
        this->ui_.rightTyreSpeed->setText(QString::number(sensor_message->right.npm));
        this->ui_.leftTyreSpeed->setText(QString::number(sensor_message->left.npm));
        this->ui_.mode->setText(QString::number(sensor_message->state.is_auto));
        this->ui_.actual_lineal_speed->setText(QString::number((sensor_message->left.npm+sensor_message->right.npm)*car_setting::WHEEL_RADIUS*M_PI));
        });

        //IMU
        actualDataLine_sub=node_->create_subscription<sensor_msgs::msg::Imu>("/imu/data",100,
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line){
            this->Acc_x=actualData_line->linear_acceleration.x;
            this->Acc_y=actualData_line->linear_acceleration.y;
            this->Acc_z=actualData_line->linear_acceleration.z;

            this->Angual_x=actualData_line->angular_velocity.x;
            this->Angual_y=actualData_line->angular_velocity.y;
            this->Angual_z=actualData_line->angular_velocity.z;

            this->imu_sec_data=actualData_line->header.stamp.sec;
            this->imu_nanosec_data=actualData_line->header.stamp.nanosec;

            if (this->imu_nanosec_data)
            {
                struct timespec timeImu = {0, 0};
                clock_gettime(CLOCK_REALTIME, &timeImu);
                if (this->lastTime.tv_sec!=0)
                {
                    //如果不等于0，有数据传入
                    //当前时间减去上一刻时间
                    double hz=1/((timeImu.tv_sec - this->lastTime.tv_sec)+(timeImu.tv_nsec-this->lastTime.tv_nsec)*1e-9);
                    this->ui_.imuHz_label->setText(QString::number(hz));
                }
                
                this->lastTime=timeImu;

            }
            
        });
        actualMag_data_sub=node_->create_subscription<sensor_msgs::msg::MagneticField>("/imu/mag",10,[this]
        (const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line){
            this->Mag_x=actualMagData_line->magnetic_field.x;
            this->Mag_y=actualMagData_line->magnetic_field.y;
            this->Mag_z=actualMagData_line->magnetic_field.z;
        });


        //Chart set

        m_tabWidget.setParent(ui_.chart_label);
        // m_tabWidget.setWindowTitle("yemian1");
        m_tabWidget.resize(383,384);
        m_tabWidget.setTabPosition(QTabWidget::North);
        m_tabWidget.setTabShape(QTabWidget::Triangular);
        m_tabWidget.setTabsClosable(false);
        //set timer
        m_timer = new QTimer(this);
        m_timer->setSingleShot(false);
        m_timer->start(100);
        // set font;
        QFont labelsFont;    
        labelsFont.setPixelSize(12);   //参数字号，数字越小，字就越小
        //Acc_line
        // 创建横纵坐标轴并设置显示范围
        Acc_axisX = new QValueAxis();
        Acc_axisY = new QValueAxis();
        Acc_axisX->setLabelsFont(labelsFont);
        Acc_axisY->setLabelsFont(labelsFont);
        Acc_axisX->setTitleText("time");
        Acc_axisY->setTitleText("Accelerate");
        // Acc_axisY->setMin(0);
        Acc_axisY->setRange(-10, 10);
        Acc_axisX->setMax(0);
        Acc_axisY->setMax(AXIS_MAX_Y);
        Acc_axisX->setMax(AXIS_MAX_X);
        Acc_axisX->setMinorTickCount(4);
        Acc_axisX->setGridLineVisible(true);

        Acc_seriesx  = new QLineSeries();                     // 创建曲线绘制对象
        Acc_seriesx->setPointsVisible(false);                 // 设置数据点可见
        Acc_seriesx->setName("X");                            // 图例名称
        Acc_seriesy  = new QLineSeries();
        Acc_seriesy->setPointsVisible(false);
        Acc_seriesy->setName("Y");
        Acc_seriesz  = new QLineSeries();
        Acc_seriesz->setPointsVisible(false);
        Acc_seriesz->setName("Z");

        Acc_chart = new QChart();
        Acc_chart->legend()->setVisible(false);                         //图例bu显示
        Acc_chart->legend()->setFont(labelsFont);
        // Acc_chart->layout()->setContentsMargins(0,0,0,0); 
        Acc_chart->legend()->setAlignment(Qt::AlignCenter);
        Acc_chart->setBackgroundRoundness(0);                           //设置背景区域无圆角
        Acc_chart->setContentsMargins(0,0,0,0);                         //设置外边界全部为0
        Acc_chart->setMargins(QMargins(0,0,0,0));                       //设置内边界全部为0
        Acc_chart->addAxis(Acc_axisY, Qt::AlignLeft);                   // 将X轴添加到图表上
        Acc_chart->addAxis(Acc_axisX, Qt::AlignBottom);                 // 将Y轴添加到图表上
        Acc_chart->addSeries(Acc_seriesx);                              // 将曲线对象添加到图表上
        Acc_chart->addSeries(Acc_seriesy);
        Acc_chart->addSeries(Acc_seriesz);
        Acc_chart->setAnimationOptions(QChart::SeriesAnimations);       // 动画：能使曲线绘制显示的更平滑，过渡效果更好看

        Acc_seriesx->attachAxis(Acc_axisX);                             // 曲线对象关联上X轴，此步骤必须在Acc_chart->addSeries之后
        Acc_seriesx->attachAxis(Acc_axisY);                             // 曲线对象关联上Y轴，此步骤必须在Acc_chart->addSeries之后
        Acc_seriesy->attachAxis(Acc_axisX);                             
        Acc_seriesy->attachAxis(Acc_axisY);
        Acc_seriesz->attachAxis(Acc_axisX);                             
        Acc_seriesz->attachAxis(Acc_axisY);   

        QtCharts::QChartView * yemian1 = new QtCharts::QChartView (&m_tabWidget);
        yemian1->setChart(Acc_chart); 
        m_tabWidget.addTab(yemian1,"Acc");
        

        // ui_.Acc_graphicsView->setChart(Acc_chart);                   // 将图表对象设置到graphicsView上进行显示
        // ui_.tabview_label->setChart(Acc_chart);
        yemian1->setRenderHint(QPainter::Antialiasing);                 // 设置渲染：抗锯齿，如果不设置那么曲线就显得不平滑
        
        

        
        //Angual_line
        Angual_axisX = new QValueAxis();
        Angual_axisY = new QValueAxis();
        Angual_axisX->setLabelsFont(labelsFont);
        Angual_axisY->setLabelsFont(labelsFont);
        Angual_axisX->setTitleText("time");
        Angual_axisY->setTitleText("Angual");
        // Angual_axisY->setMin(0);
        Angual_axisY->setRange(-10, 10);
        Angual_axisX->setMax(0);
        Angual_axisY->setMax(AXIS_MAX_Y);
        Angual_axisX->setMax(AXIS_MAX_X);
        Angual_axisX->setMinorTickCount(4);
        Angual_axisX->setGridLineVisible(true);

        Angual_seriesx  = new QLineSeries(); 
        Angual_seriesx->setPointsVisible(false);     
        Angual_seriesx->setName("X");
  
        Angual_seriesy  = new QLineSeries();  
        Angual_seriesy->setPointsVisible(false); 
        Angual_seriesy->setName("Y");

        Angual_seriesz  = new QLineSeries();  
        Angual_seriesz->setPointsVisible(false); 
        Angual_seriesz->setName("Z");

        Angual_chart = new QChart();
        Angual_chart->legend()->setVisible(false);
        Angual_chart->setBackgroundRoundness(0);
        Angual_chart->setContentsMargins(0, 0, 0, 0);
        Angual_chart->setMargins(QMargins(0,0,0,0));
        Angual_chart->addAxis(Angual_axisY, Qt::AlignLeft); 
        Angual_chart->addAxis(Angual_axisX, Qt::AlignBottom);
        Angual_chart->addSeries(Angual_seriesx);  
        Angual_chart->addSeries(Angual_seriesy);
        Angual_chart->addSeries(Angual_seriesz);
        Angual_chart->setAnimationOptions(QChart::SeriesAnimations); 

        Angual_seriesx->attachAxis(Angual_axisX);
        Angual_seriesx->attachAxis(Angual_axisY);
        Angual_seriesy->attachAxis(Angual_axisX);                             
        Angual_seriesy->attachAxis(Angual_axisY);
        Angual_seriesz->attachAxis(Angual_axisX);                             
        Angual_seriesz->attachAxis(Angual_axisY);                            

        QtCharts::QChartView * yemian2 = new QtCharts::QChartView (&m_tabWidget);
        yemian2->setChart(Angual_chart); 
        m_tabWidget.addTab(yemian2,"Angual");
        m_tabWidget.setCurrentIndex(1);
        // ui_.Angual_graphicsView->setChart(Angual_chart);    
        yemian2->setRenderHint(QPainter::Antialiasing);

        //Mag_line
        Mag_axisX = new QValueAxis();
        Mag_axisY = new QValueAxis();
        Mag_axisX->setLabelsFont(labelsFont);
        Mag_axisY->setLabelsFont(labelsFont);
        Mag_axisX->setTitleText("time");
        Mag_axisY->setTitleText("Mag");
        // Mag_axisY->setMin(0);
        Mag_axisY->setRange(-2, 2);
        Mag_axisX->setMax(0);
        Mag_axisY->setMax(MAG_AXIS_MAX_Y);
        Mag_axisX->setMax(MAG_AXIS_MAX_X);
        Mag_axisX->setMinorTickCount(4);
        Mag_axisX->setGridLineVisible(true);

        Mag_seriesx  = new QLineSeries(); 
        Mag_seriesx->setPointsVisible(false);     
        Mag_seriesx->setName("X");
  
        Mag_seriesy  = new QLineSeries();  
        Mag_seriesy->setPointsVisible(false); 
        Mag_seriesy->setName("Y");

        Mag_seriesz  = new QLineSeries();  
        Mag_seriesz->setPointsVisible(false); 
        Mag_seriesz->setName("Z");

        Mag_chart = new QChart();
        Mag_chart->legend()->setVisible(false);
        Mag_chart->setBackgroundRoundness(0);
        Mag_chart->setContentsMargins(0, 0, 0, 0);
        Mag_chart->setMargins(QMargins(0,0,0,0));
        Mag_chart->addAxis(Mag_axisY, Qt::AlignLeft); 
        Mag_chart->addAxis(Mag_axisX, Qt::AlignBottom);
        Mag_chart->addSeries(Mag_seriesx);  
        Mag_chart->addSeries(Mag_seriesy);
        Mag_chart->addSeries(Mag_seriesz);
        Mag_chart->setAnimationOptions(QChart::SeriesAnimations); 

        Mag_seriesx->attachAxis(Mag_axisX);
        Mag_seriesx->attachAxis(Mag_axisY);
        Mag_seriesy->attachAxis(Mag_axisX);                             
        Mag_seriesy->attachAxis(Mag_axisY);
        Mag_seriesz->attachAxis(Mag_axisX);                             
        Mag_seriesz->attachAxis(Mag_axisY);                            

        QtCharts::QChartView * yemian3 = new QtCharts::QChartView (&m_tabWidget);
        yemian3->setChart(Mag_chart); 
        m_tabWidget.addTab(yemian3,"Mag");
        m_tabWidget.setCurrentIndex(2);
        // ui_.Mag_graphicsView->setChart(Mag_chart);
        yemian3->setRenderHint(QPainter::Antialiasing);
        

        //pushbutton
        ui_.up_pushButton->setAutoRepeat(true); //启用长按
        ui_.up_pushButton->setAutoRepeatDelay(0);//触发长按的时间
        ui_.up_pushButton->setAutoRepeatInterval(0);//长按时click信号间隔

        ui_.down_pushButton->setAutoRepeat(true); 
        ui_.down_pushButton->setAutoRepeatDelay(0);
        ui_.down_pushButton->setAutoRepeatInterval(0);

        ui_.left_pushButton->setAutoRepeat(true); 
        ui_.left_pushButton->setAutoRepeatDelay(0);
        ui_.left_pushButton->setAutoRepeatInterval(0);

        ui_.right_pushButton->setAutoRepeat(true); 
        ui_.right_pushButton->setAutoRepeatDelay(0);
        ui_.right_pushButton->setAutoRepeatInterval(0);

        ui_.stop_pushButton->setAutoRepeat(true); 
        ui_.stop_pushButton->setAutoRepeatDelay(0);
        ui_.stop_pushButton->setAutoRepeatInterval(0);

        //按钮与键盘绑定
        ui_.up_pushButton->setShortcut(Qt::Key_Up);
        ui_.down_pushButton->setShortcut(Qt::Key_Down);
        ui_.left_pushButton->setShortcut(Qt::Key_Left);
        ui_.right_pushButton->setShortcut(Qt::Key_Right);
        ui_.stop_pushButton->setShortcut(Qt::Key_Space);

        // Connect with Qt Widget信号与槽函数的绑定
        connect(&m_tabWidget, SIGNAL(tabCloseRequested(int)), this, SLOT(onTabCloseRequested(int)));
        connect(ui_.run_pushButton, SIGNAL(clicked()), this, SLOT(on_run_button_clicked()));
        connect(ui_.up_pushButton, SIGNAL(clicked()), this, SLOT(up_button_clicked()));
        connect(ui_.down_pushButton, SIGNAL(clicked()), this, SLOT(down_button_clicked()));
        connect(ui_.left_pushButton, SIGNAL(clicked()), this, SLOT(left_button_clicked()));
        connect(ui_.right_pushButton, SIGNAL(clicked()), this, SLOT(right_button_clicked()));
        connect(ui_.stop_pushButton, SIGNAL(clicked()), this, SLOT(stop_button_clicked()));
        connect(m_timer, SIGNAL(timeout()), this, SLOT(slotTimeout()));


        
        QTimer *CurrentTime = new QTimer(this);
        CurrentTime->start(100);
        connect(CurrentTime,&QTimer::timeout,[=]()
        {
            //时钟
            QDateTime current_time = QDateTime::currentDateTime(); 
            QString StrCurrentTime = current_time.toString("yyyy-MM-dd hh:mm:ss.zzz ddd"); 
            ui_.dateTime_label->setText(StrCurrentTime);
            QFont font2("color:rgb(50,205,50);font:bold;font-size:15px");
            ui_.dateTime_label->setFont(font2);

            //启动时间
            struct timespec time1 = {0, 0};
            clock_gettime(CLOCK_REALTIME, &time1);//此时获取当前系统时间
            int t0=time1.tv_sec;
            int t1=time1.tv_nsec;
            float t=(t0%10000+t1/1000000*0.001);
            float imu_second=(imu_sec_data%10000+0.001*imu_nanosec_data/1000000);
            
            float imageFront_second=(imageFront_sec_data%10000+0.001*imageFront_nanosec_data/1000000);
            float imageBack_second=(imageBack_sec_data%10000+0.001*imageBack_nanosec_data/1000000);
            float control_second=(control_sec_data%10000+0.001*control_nanosec_data/1000000);
            float imu_delay=t-imu_second;
            float imageFront_delay=t-imageFront_second;
            float imageBack_delay=t-imageBack_second;
            float control_delay=t-control_second;

            this->ui_.imuTime_label->setText(QString::number(imu_delay));
            this->ui_.imageFrontTime_label->setText(QString::number(imageFront_delay));
            this->ui_.imageBackTime_label->setText(QString::number(imageBack_delay));
            this->ui_.controlTime_label->setText(QString::number(control_delay));
        
        });

        // rviz_common::RenderPanel* render_pendel_=new rviz_common::RenderPanel;
        // ui_.rviz2_layout->addWidget(render_pendel_);
        // rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abstraction;
        // rviz_common::WindowManagerInterface * wm;
        // rclcpp::Clock::SharedPtr clock;
        // rviz_common::VisualizationManager* manager_=new rviz_common::VisualizationManager(render_pendel_,ros_node_abstraction,wm,clock);
        // manager_->getSceneManager();
        // render_pendel_->initialize(manager_);
        // manager_->initialize();
        // manager_->removeAllDisplays();
        // manager_->startUpdate();
        
        

    }

    void pushButton::shutdownPlugin()
    {
          //rclcpp::shutdown();
          ;
    }

    void pushButton::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
    {
        ;
    }

    void pushButton::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
    {
        ;
    }

    pushButton::~pushButton()
    {
        //   delete frame;
        if (widget_)
            delete widget_;
            
    }
    
    void pushButton::onTabCloseRequested(int index){
        m_tabWidget.removeTab(index);
    }
    
    void pushButton::on_run_button_clicked()
    {
        // auto action_message = std_msgs::msg::String();
        auto remote_control_message = private_interface::msg::CarControl();
        remote_control_message.linear.velocity =(ui_.v_linedit->text()).toFloat();
        remote_control_message.angular.velocity= (ui_.w_linedit->text()).toFloat();
        // remote_control_message.linear.velocity =std::stof((ui_.w_linedit->text()).toStdString());
        // remote_control_message.angular.velocity= std::stof((ui_.v_linedit->text()).toStdString());
        remote_control_button_pub->publish(remote_control_message);
        
        //pingjie 1. str=a+","+b
        // str.sprintf("%s,%s",set_angular_velocity,set_linear_velocity);
        // action_message.data=str.toStdString();
        //remote_control_pub->publish(str);
        // std::cout<<angular_velocity.toStdString()<<std::endl;
        
    }
  
    void pushButton::up_button_clicked()
    {
        auto remote_control_message = private_interface::msg::CarControl();
        remote_control_message.linear.velocity = 1 * car_setting::RemoteForwardSpeed;
        remote_control_message.angular.velocity = 0;
        remote_control_message.linear.acceleration = 0.0;
        remote_control_message.angular.acceleration = 0.0;
        remote_control_message.mode.is_auto = 0;
        remote_control_message.mode.gear = 1;
        remote_control_button_pub->publish(remote_control_message);

        control_nanosec_data=remote_control_message.header.stamp.nanosec;

        auto remote_control_message_virtual = geometry_msgs::msg::Twist();
        remote_control_message_virtual.angular.x=0;
        remote_control_message_virtual.angular.y=0;
        remote_control_message_virtual.angular.z=0;
        remote_control_message_virtual.linear.x=1* car_setting::RemoteForwardSpeed;
        remote_control_message_virtual.linear.y=0;
        remote_control_message_virtual.linear.z=0;
        remote_control_button_virtual_pub->publish(remote_control_message_virtual);

    }
    void pushButton::down_button_clicked()
    {
        auto remote_control_message = private_interface::msg::CarControl();
        remote_control_message.linear.velocity = -1* car_setting::RemoteForwardSpeed;
        remote_control_message.angular.velocity = 0;
        remote_control_message.linear.acceleration = 0.0;
        remote_control_message.angular.acceleration = 0.0;
        remote_control_message.mode.is_auto = 0;
        remote_control_message.mode.gear = 2;
        remote_control_button_pub->publish(remote_control_message);

        auto remote_control_message_virtual = geometry_msgs::msg::Twist();
        remote_control_message_virtual.angular.x=0;
        remote_control_message_virtual.angular.y=0;
        remote_control_message_virtual.angular.z=0;
        remote_control_message_virtual.linear.x= -1* car_setting::RemoteForwardSpeed;
        remote_control_message_virtual.linear.y=0;
        remote_control_message_virtual.linear.z=0;
        remote_control_button_virtual_pub->publish(remote_control_message_virtual);


    }
    void pushButton::left_button_clicked()
    {
        auto remote_control_message = private_interface::msg::CarControl();
        remote_control_message.linear.velocity = 1 * car_setting::RemoteLeftSpeed;
        remote_control_message.angular.velocity = -1* car_setting::RemoteLeftAngularSpeed;
        remote_control_message.linear.acceleration = 0.0;
        remote_control_message.angular.acceleration = 0.0;
        remote_control_message.mode.is_auto = 0;
        remote_control_message.mode.gear = 1;  
        remote_control_button_pub->publish(remote_control_message);

        auto remote_control_message_virtual = geometry_msgs::msg::Twist();
        remote_control_message_virtual.angular.x=0;
        remote_control_message_virtual.angular.y=0;
        remote_control_message_virtual.angular.z=1* car_setting::RemoteLeftAngularSpeed;
        remote_control_message_virtual.linear.x= 1* car_setting::RemoteForwardSpeed;
        remote_control_message_virtual.linear.y=0;
        remote_control_message_virtual.linear.z=0;
        remote_control_button_virtual_pub->publish(remote_control_message_virtual);
    }
    void pushButton::right_button_clicked()
    {
        auto remote_control_message = private_interface::msg::CarControl();
        remote_control_message.linear.velocity = 1 * car_setting::RemoteLeftSpeed;
        remote_control_message.angular.velocity = 1* car_setting::RemoteLeftAngularSpeed;
        remote_control_message.linear.acceleration = 0.0;
        remote_control_message.angular.acceleration = 0.0;
        remote_control_message.mode.is_auto = 0;
        remote_control_message.mode.gear = 1; 
        remote_control_button_pub->publish(remote_control_message);

        auto remote_control_message_virtual = geometry_msgs::msg::Twist();
        remote_control_message_virtual.angular.x=0;
        remote_control_message_virtual.angular.y=0;
        remote_control_message_virtual.angular.z=-1* car_setting::RemoteLeftAngularSpeed;
        remote_control_message_virtual.linear.x= 1* car_setting::RemoteForwardSpeed;
        remote_control_message_virtual.linear.y=0;
        remote_control_message_virtual.linear.z=0;
        remote_control_button_virtual_pub->publish(remote_control_message_virtual);
    }
    void pushButton::stop_button_clicked()
    {
        auto remote_control_message = private_interface::msg::CarControl();
        remote_control_message.linear.velocity = 0 ;
        remote_control_message.angular.velocity = 0;
        remote_control_message.linear.acceleration = 0.0;
        remote_control_message.angular.acceleration = 0.0;
        remote_control_message.mode.is_auto = 0;
        remote_control_message.mode.gear = 0;
        
        remote_control_button_pub->publish(remote_control_message);

        auto remote_control_message_virtual = geometry_msgs::msg::Twist();
        remote_control_message_virtual.angular.x=0;
        remote_control_message_virtual.angular.y=0;
        remote_control_message_virtual.angular.z=0;
        remote_control_message_virtual.linear.x=0;
        remote_control_message_virtual.linear.y=0;
        remote_control_message_virtual.linear.z=0;
        remote_control_button_virtual_pub->publish(remote_control_message_virtual);
    }

    void pushButton::slotTimeout()
    {
        if(pointCount > AXIS_MAX_Y)
        {
            Acc_seriesx->remove(0);
            Acc_seriesy->remove(0);
            Acc_seriesz->remove(0);
            Angual_seriesx->remove(0);
            Angual_seriesy->remove(0);
            Angual_seriesz->remove(0);
            Mag_seriesx->remove(0);
            Mag_seriesy->remove(0);
            Mag_seriesz->remove(0);
            Acc_chart->axisX()->setMin(pointCount - AXIS_MAX_Y);
            Acc_chart->axisX()->setMax(pointCount);                    // 更新X轴范围
            Angual_chart->axisX()->setMin(pointCount - AXIS_MAX_Y);
            Angual_chart->axisX()->setMax(pointCount);                    // 更新X轴范围
            Mag_chart->axisX()->setMin(pointCount - AXIS_MAX_Y);
            Mag_chart->axisX()->setMax(pointCount);                    // 更新X轴范围
        }
        
        Acc_seriesx->append(QPointF(pointCount, Acc_x));
        Acc_seriesy->append(QPointF(pointCount,Acc_y));
        Acc_seriesz->append(QPointF(pointCount, Acc_z));

        Angual_seriesx->append(QPointF(pointCount, Angual_x));  
        Angual_seriesy->append(QPointF(pointCount,Angual_y));
        Angual_seriesz->append(QPointF(pointCount, Angual_z));

        Mag_seriesx->append(QPointF(pointCount, Mag_x));
        Mag_seriesy->append(QPointF(pointCount, Mag_y));
        Mag_seriesz->append(QPointF(pointCount, Mag_z));

        pointCount++;
    }
    

    
} // namespace rqt_plugin